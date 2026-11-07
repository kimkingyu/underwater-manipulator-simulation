"""Execute both standalone templates against fake editor APIs, never ANSYS."""
import json
import os
import sys
import types
from pathlib import Path

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import definition_probe as probe


class FakeFile:
    @staticmethod
    def Exists(path):
        return Path(path).exists()
    @staticmethod
    def Replace(source, target, backup):
        os.replace(source, target)
    @staticmethod
    def Move(source, target):
        if Path(target).exists():
            raise FileExistsError(target)
        os.rename(source, target)
    @staticmethod
    def WriteAllText(path, value, encoding):
        Path(path).write_text(value, encoding="utf-8")
    @staticmethod
    def AppendAllText(path, value, encoding):
        with Path(path).open("a", encoding="utf-8") as stream:
            stream.write(value)
    @staticmethod
    def ReadAllText(path):
        return Path(path).read_text(encoding="utf-8")


@pytest.fixture
def dotnet(monkeypatch):
    system = types.ModuleType("System")
    system.Environment = types.SimpleNamespace(NewLine="\n")
    io = types.ModuleType("System.IO")
    io.File = FakeFile
    text = types.ModuleType("System.Text")
    text.Encoding = types.SimpleNamespace(UTF8="utf-8")
    monkeypatch.setitem(sys.modules, "System", system)
    monkeypatch.setitem(sys.modules, "System.IO", io)
    monkeypatch.setitem(sys.modules, "System.Text", text)


class Quantity:
    def __init__(self, text):
        self.Value = float(text.split()[0])
    def ConvertUnit(self, unit):
        assert unit == "m"
        return self


class NativeObject:
    ObjectState = "FullyDefined"
    ObjectId = 31
    GrowthRate = 0.0
    def __init__(self):
        self.Name = "native-object"
        self.Location = types.SimpleNamespace(Ids=[])


class FakeMesh(NativeObject):
    Nodes = 0
    Elements = 0
    def AddSizing(self):
        return NativeObject()
    def AddAutomaticMethod(self):
        return NativeObject()
    def __getattr__(self, name):
        if name in ("GenerateMesh", "Update"):
            pytest.fail("template touched a forbidden mesh operation: " + name)
        raise AttributeError(name)


def fake_meshing(tmp_path, *, object_state="FullyDefined", messages=None, missing_selection=False):
    contract = json.loads((STAGE / "contract.json").read_text(encoding="utf-8"))
    settings = probe.domain_settings(contract, tmp_path)[0]
    mesh = FakeMesh()
    selections = [types.SimpleNamespace(Name=name, Location=types.SimpleNamespace(Ids=[index + 1]))
                  for index, name in enumerate(settings["required_selections"])]
    if missing_selection:
        selections.pop()
    body = NativeObject()
    body.GetGeoBody = lambda: types.SimpleNamespace(Id=100, Faces=[types.SimpleNamespace(Id=i) for i in range(1, 6)])
    coords = types.SimpleNamespace(AddCoordinateSystem=lambda: NativeObject())
    model = types.SimpleNamespace(Mesh=mesh, NamedSelections=types.SimpleNamespace(Children=selections),
                                  Geometry=types.SimpleNamespace(GetChildren=lambda *a: [body]), CoordinateSystems=coords)
    api = types.SimpleNamespace(DataModel=types.SimpleNamespace(Project=types.SimpleNamespace(Model=model)),
          SelectionManager=types.SimpleNamespace(CreateSelectionInfo=lambda _: types.SimpleNamespace(Ids=[])),
          Application=types.SimpleNamespace(Messages=[] if messages is None else messages))
    enum = types.SimpleNamespace(GeometryEntities="GeometryEntities")
    namespace = {"ExtAPI": api, "Quantity": Quantity, "SelectionTypeEnum": enum,
                 "SizingType": types.SimpleNamespace(ElementSize="ElementSize", SphereOfInfluence="SphereOfInfluence"),
                 "MethodType": types.SimpleNamespace(AllTriAllTet="AllTriAllTet"),
                 "DataModelObjectCategory": types.SimpleNamespace(Body="Body")}
    return settings, namespace, model


def execute_meshing(settings, namespace):
    template = (STAGE / "mesh/meshing_definition_probe.py.in").read_text(encoding="utf-8")
    source = template.replace("__CONFIG_JSON__", repr(json.dumps(settings)))
    exec(compile(source, "meshing_definition_probe.py", "exec"), namespace)


def test_editor_template_uses_realistic_native_objects_and_readbacks(tmp_path, dotnet):
    settings, namespace, _ = fake_meshing(tmp_path)
    execute_meshing(settings, namespace)
    stats = json.loads((tmp_path / "background_meshing_stats.json").read_text())
    assert stats["script_entered"] and stats["definitions_ready"]
    assert stats["mesh_generated"] is False
    assert stats["flow_iterations"] == 0
    assert len(stats["controls"]) == 4
    probe.validate_definition_stats(stats, settings)
    with pytest.raises(Exception, match="rerun"):
        execute_meshing(settings, namespace)


def test_editor_reads_native_counts_before_and_after_controls(tmp_path, dotnet):
    settings, namespace, model = fake_meshing(tmp_path)
    class CountingMesh(FakeMesh):
        def __init__(self):
            super().__init__()
            self.events = []
        @property
        def Nodes(self):
            self.events.append("nodes")
            return 0
        @property
        def Elements(self):
            self.events.append("elements")
            return 0
        def AddAutomaticMethod(self):
            assert self.events == ["nodes", "elements"]
            self.events.append("controls")
            return super().AddAutomaticMethod()
    mesh = CountingMesh()
    model.Mesh = mesh
    (tmp_path / "empty.mshdb").write_bytes(b"empty native database")
    execute_meshing(settings, namespace)
    assert mesh.events == ["nodes", "elements", "controls", "nodes", "elements"]
    stats = json.loads((tmp_path / "background_meshing_stats.json").read_text())
    assert stats["mesh_counts_before"] == {"nodes": 0, "elements": 0}
    assert stats["mesh_counts_after"] == {"nodes": 0, "elements": 0}
    assert stats["mesh_generated"] is False


@pytest.mark.parametrize("phase,key,value", [
    ("before", "nodes", 1), ("before", "elements", 2),
    ("after", "nodes", 1), ("after", "elements", 2),
    ("before", "nodes", -1), ("after", "elements", False),
    ("before", "elements", 1.5), ("after", "nodes", "0"),
])
def test_editor_native_nonzero_or_invalid_counts_fail_closed(tmp_path, dotnet, phase, key, value):
    settings, namespace, model = fake_meshing(tmp_path)
    class CountMesh(FakeMesh):
        def __init__(self):
            super().__init__()
            self.phase = "before"
        @property
        def Nodes(self):
            return value if self.phase == phase and key == "nodes" else 0
        @property
        def Elements(self):
            result = value if self.phase == phase and key == "elements" else 0
            self.phase = "after"
            return result
    model.Mesh = CountMesh()
    with pytest.raises(Exception, match="definition probe failed"):
        execute_meshing(settings, namespace)
    stats = json.loads((tmp_path / "background_meshing_stats.json").read_text())
    assert stats["status"] == "failed" and stats["definitions_ready"] is False
    assert stats["mesh_generated"] is False
    if value in (1, 2) and not isinstance(value, bool):
        assert stats["mesh_counts_" + phase][key] == value


def test_editor_missing_native_count_property_fails_closed(tmp_path, dotnet):
    settings, namespace, model = fake_meshing(tmp_path)
    class MissingCountMesh(FakeMesh):
        @property
        def Nodes(self):
            raise AttributeError("native node count unavailable")
    model.Mesh = MissingCountMesh()
    with pytest.raises(Exception, match="definition probe failed"):
        execute_meshing(settings, namespace)
    stats = json.loads((tmp_path / "background_meshing_stats.json").read_text())
    assert stats["status"] == "failed" and stats["mesh_counts_before"] == {}


def test_editor_writes_entered_receipt_before_native_access(tmp_path, dotnet):
    settings, namespace, _ = fake_meshing(tmp_path)
    class FailingAPI:
        @property
        def DataModel(self):
            entered = json.loads((tmp_path / "background_meshing_stats.json").read_text())
            assert entered["script_entered"] is True and entered["definitions_ready"] is False
            raise RuntimeError("中文后台失联")
        Application = types.SimpleNamespace(Messages=[])
    namespace["ExtAPI"] = FailingAPI()
    with pytest.raises(Exception, match="definition probe failed"):
        execute_meshing(settings, namespace)
    stats = json.loads((tmp_path / "background_meshing_stats.json").read_text())
    assert stats["status"] == "failed" and stats["exception"] == "中文后台失联"


@pytest.mark.parametrize("mode", ["underdefined", "missing", "native_error", "bad_size", "bad_scope", "bad_messages"])
def test_editor_fails_closed_with_unicode_and_definition_failures(tmp_path, dotnet, monkeypatch, mode):
    settings, namespace, model = fake_meshing(tmp_path, missing_selection=mode == "missing")
    if mode == "underdefined":
        monkeypatch.setattr(NativeObject, "ObjectState", "UnderDefined")
    if mode == "native_error":
        namespace["ExtAPI"].Application.Messages = [types.SimpleNamespace(Severity="Error", DisplayString="中文原生错误")]
    if mode == "bad_size":
        class BadQuantity(Quantity):
            def ConvertUnit(self, unit):
                return types.SimpleNamespace(Value=self.Value + 1)
        namespace["Quantity"] = BadQuantity
    if mode == "bad_scope":
        model.NamedSelections.Children[0].Location.Ids = [9999]
    if mode == "bad_messages":
        namespace["ExtAPI"].Application.Messages = [object()]
    with pytest.raises(Exception, match="definition probe failed"):
        execute_meshing(settings, namespace)
    stats = json.loads((tmp_path / "background_meshing_stats.json").read_text())
    assert stats["status"] == "failed" and stats["definitions_ready"] is False
    assert stats["mesh_generated"] is False
    if mode == "native_error":
        assert stats["messages"][0]["text"] == "中文原生错误"


def execute_journal(tmp_path, dispatch):
    operations = []
    properties = types.SimpleNamespace()
    class Component:
        def __init__(self, name):
            self.name = name
        def Update(self, **kw):
            assert self.name == "Geometry" and kw == {"AllDependencies": False}
            operations.append("geometry_update")
        def Refresh(self):
            assert self.name == "Mesh"
            operations.append("mesh_refresh")
    class Container:
        def __init__(self, name):
            self.name = name
        def SetFile(self, **kw):
            assert self.name == "Geometry"
        def GetGeometryProperties(self):
            return properties
        def Edit(self, **kw):
            assert self.name == "Mesh" and kw == {"Interactive": False}
            operations.append("mesh_edit")
        def SendCommand(self, **kw):
            operations.append("mesh_definitions")
            dispatch()
        def Exit(self):
            operations.append("mesh_exit")
    class FakeSystem:
        def GetContainer(self, ComponentName):
            assert ComponentName in ("Geometry", "Mesh")
            return Container(ComponentName)
        def GetComponent(self, Name):
            assert Name in ("Geometry", "Mesh")
            return Component(Name)
    template = types.SimpleNamespace(CreateSystem=FakeSystem)
    config = {"cad_path": str(tmp_path / "background.scdoc"), "meshing_script": "definition script",
              "project": str(tmp_path / "stage15_definition_probe.wbpj"),
              "stats_path": str(tmp_path / "background_meshing_stats.json"),
              "trace": str(tmp_path / "workbench.trace.txt"), "completion": str(tmp_path / "workbench_completion.json")}
    source = (STAGE / "mesh/wb_definition_probe.wbjn.in").read_text(encoding="utf-8")
    source = source.replace("__CONFIG_JSON__", repr(json.dumps(config)))
    def save(**kwargs):
        assert kwargs == {"FilePath": str(tmp_path / "stage15_definition_probe.wbpj"), "Overwrite": False}
        assert operations == ["geometry_update"]
        path = Path(kwargs["FilePath"])
        with path.open("x", encoding="utf-8") as stream:
            stream.write("offline new project")
        operations.append("save_new_project")
    namespace = {"SetScriptVersion": lambda **k: None,
                 "GetTemplate": lambda **k: template, "Save": save}
    exec(compile(source, "wb_definition_probe.wbjn", "exec"), namespace)
    return operations, json.loads((tmp_path / "workbench_completion.json").read_text()), properties


def test_journal_only_loads_geometry_refreshes_and_edits_mesh(tmp_path, dotnet):
    def dispatch():
        (tmp_path / "background_meshing_stats.json").write_text(json.dumps({
            "status": "definitions_ready", "script_entered": True, "definitions_ready": True,
            "mesh_generated": False}), encoding="utf-8")
    operations, completion, properties = execute_journal(tmp_path, dispatch)
    assert operations == ["geometry_update", "save_new_project", "mesh_refresh", "mesh_edit", "mesh_definitions", "mesh_exit"]
    assert (tmp_path / "stage15_definition_probe.wbpj").read_text() == "offline new project"
    assert completion["status"] == "journal_completed"
    assert completion["journal_completed"] is True and completion["mesh_generated"] is False
    assert properties.GeometryImportNamedSelections is True
    assert (tmp_path / "workbench.trace.txt").read_text().splitlines() == [
        "journal-entered", "geometry-loaded", "meshing-script-dispatched", "definitions-ready", "journal-completed"]


def test_journal_refuses_existing_new_project_before_any_native_call(tmp_path, dotnet):
    project = tmp_path / "stage15_definition_probe.wbpj"
    project.write_bytes(b"prior contents")
    with pytest.raises(Exception, match="rerun"):
        execute_journal(tmp_path, lambda: pytest.fail("must not enter Meshing"))
    assert project.read_bytes() == b"prior contents"
    assert not (tmp_path / "workbench.trace.txt").exists()


@pytest.mark.parametrize("mode", ["exception", "missing", "definitions_failed", "generation_claimed"])
def test_journal_native_failures_write_failure_receipt_without_unicode_loss(tmp_path, dotnet, mode):
    def dispatch():
        if mode == "exception":
            raise RuntimeError("中文后台连接失败")
        if mode == "missing":
            return
        (tmp_path / "background_meshing_stats.json").write_text(json.dumps({
            "status": "failed" if mode == "definitions_failed" else "definitions_ready",
            "script_entered": True, "definitions_ready": True,
            "mesh_generated": mode == "generation_claimed"}), encoding="utf-8")
    operations, completion, _ = execute_journal(tmp_path, dispatch)
    assert completion["status"] == "failed" and completion["journal_completed"] is False
    assert operations[-1] == "mesh_exit"
    if mode == "exception":
        assert completion["exception"] == "中文后台连接失败"
