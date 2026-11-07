"""Execute the actual rendered native templates against a deliberately tiny fake API."""
import json
import sys
import types
from pathlib import Path

import pytest

STAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(STAGE))
import sizing_diagnostic as probe


class Quantity:
    def __init__(self, source):
        value, unit = source.split(" ", 1)
        self.Value = float(value)
        self.Unit = unit.strip("[]")
        self.conversions = []
    def ConvertUnit(self, unit):
        self.conversions.append(unit)
        assert unit == "m" and self.Unit == "m"
        return self
    def __str__(self):
        return "%s [%s]" % (self.Value, self.Unit)


class Control:
    def __init__(self, *, state="UnderDefined"):
        self.Name = ""
        self.Location = types.SimpleNamespace(Ids=[])
        self.Type = "ElementSize"
        self.ElementSize = Quantity("1 [m]")
        self.growth_writes = []
        object.__setattr__(self, "GrowthRate", 0.0)
        self.ObjectState = state
        self.CaptureCurvature = True
        self.CaptureProximity = True
        self.ScopingMethod = "NamedSelection"
        self.NamedSelection = None
        self.Active = True
        self.Suppressed = False
        self.Behavior = "Soft"
    def __setattr__(self, name, value):
        if name == "GrowthRate" and hasattr(self, "growth_writes"):
            self.growth_writes.append(value)
        object.__setattr__(self, name, value)
    @property
    def CurvatureNormalAngle(self):
        raise RuntimeError("documented field not available in this release context")
    @property
    def InternalObject(self):
        pytest.fail("private API must never be touched")


class Mesh:
    def __init__(self, state="UnderDefined", nodes=0):
        self.Nodes = nodes
        self.Elements = 0
        self.Children = []
        self.state = state
    def AddSizing(self):
        obj = Control(state=self.state)
        self.Children.append(obj)
        return obj
    def AddAutomaticMethod(self):
        obj = Control(state="FullyDefined")
        self.Children.append(obj)
        return obj
    def GenerateMesh(self):
        pytest.fail("discretization must never be requested")
    def Update(self, **kwargs):
        pytest.fail("Mesh.Update must never be requested")


@pytest.fixture
def fake(tmp_path, monkeypatch):
    calls = []
    class File:
        @staticmethod
        def Exists(path):
            return Path(path).exists()
        @staticmethod
        def Move(source, target):
            assert not Path(target).exists()
            Path(source).rename(target)
        @staticmethod
        def WriteAllText(path, content, encoding):
            Path(path).write_text(content, encoding="utf-8")
        @staticmethod
        def ReadAllText(path):
            return Path(path).read_text(encoding="utf-8")
        @staticmethod
        def AppendAllText(path, content, encoding):
            with Path(path).open("a", encoding="utf-8") as stream:
                stream.write(content)
    system_module = types.ModuleType("System")
    system_module.Environment = types.SimpleNamespace(NewLine="\n")
    io_module = types.ModuleType("System.IO")
    io_module.File = File
    text_module = types.ModuleType("System.Text")
    text_module.Encoding = types.SimpleNamespace(UTF8="utf-8")
    monkeypatch.setitem(sys.modules, "System", system_module)
    monkeypatch.setitem(sys.modules, "System.IO", io_module)
    monkeypatch.setitem(sys.modules, "System.Text", text_module)
    stage = tmp_path / "stage"
    (stage / "mesh").mkdir(parents=True)
    for name in ("meshing_sizing_diagnostic.py.in", "wb_sizing_diagnostic.wbjn.in"):
        (stage / "mesh" / name).write_text((STAGE / "mesh" / name).read_text(encoding="utf-8"), encoding="utf-8")
    attempt = stage / probe.ATTEMPT_ROOT_RELATIVE / "attempt_001"
    attempt.mkdir(parents=True)
    frozen = dict(contract=json.loads((STAGE / "contract.json").read_text(encoding="utf-8")), cad={"background": {"path": "mock-background.scdoc"}})
    mesh = Mesh()
    geo = types.SimpleNamespace(Id=1, Faces=[types.SimpleNamespace(Id=index) for index in range(7, 12)])
    body = types.SimpleNamespace(Name="background", GetGeoBody=lambda: geo)
    class Coordinates:
        def __init__(self):
            self.Children = []
        def AddCoordinateSystem(self):
            obj = types.SimpleNamespace(Name="", ObjectId=99 + len(self.Children), ObjectState="FullyDefined",
                  OriginX=Quantity("0 [m]"), OriginY=Quantity("0 [m]"), OriginZ=Quantity("0 [m]"))
            self.Children.append(obj)
            return obj
    names = ("wall_base0", "wall_auv", "inlet", "outlet", "tank_wall")
    selections = [types.SimpleNamespace(Name=name, Location=types.SimpleNamespace(Ids=[index + 7])) for index, name in enumerate(names)]
    model = types.SimpleNamespace(Mesh=mesh, Geometry=types.SimpleNamespace(GetChildren=lambda *a: [body]),
              NamedSelections=types.SimpleNamespace(Children=selections), CoordinateSystems=Coordinates())
    def refresh():
        calls.append("tree_refresh")
    api = types.SimpleNamespace(DataModel=types.SimpleNamespace(Project=types.SimpleNamespace(Model=model), Tree=types.SimpleNamespace(Refresh=refresh)),
          SelectionManager=types.SimpleNamespace(CreateSelectionInfo=lambda *a: types.SimpleNamespace(Ids=[])),
          Application=types.SimpleNamespace(Messages=[]))
    def native_globals():
        return dict(ExtAPI=api, Quantity=Quantity, DataModelObjectCategory=types.SimpleNamespace(Body="Body"),
                    SelectionTypeEnum=types.SimpleNamespace(GeometryEntities="GeometryEntities"),
                    SizingType=types.SimpleNamespace(ElementSize="ElementSize", SphereOfInfluence="SphereOfInfluence"), MethodType=types.SimpleNamespace(AllTriAllTet="AllTriAllTet"),
                    GeometryDefineByType=types.SimpleNamespace(Geometry="Geometry"))
    return stage, attempt, frozen, mesh, calls, native_globals


@pytest.mark.parametrize("profile", probe.PROFILES)
def test_actual_two_commands_three_receipts_and_profiles(fake, profile):
    stage, attempt, frozen, mesh, calls, globals_factory = fake
    rendered = probe.render_scripts(attempt, stage, frozen, profile)
    first_globals = globals_factory()
    exec(compile(rendered["define_script"], "mock-define", "exec"), first_globals)
    assert calls == ["tree_refresh"]
    assert not (attempt / probe.RECEIPTS["readback"]).exists()
    # Intentionally discard all globals from the first SendCommand.
    second_globals = globals_factory()
    assert "size" not in second_globals
    exec(compile(rendered["readback_script"], "mock-readback", "exec"), second_globals)
    request = dict(job_id="sizing-b001-a001", profile=profile)
    for phase, name in probe.RECEIPTS.items():
        value = json.loads((attempt / name).read_text(encoding="utf-8"))
        probe.validate_receipt(value, phase, request, rendered["settings"])
        assert value["diagnostic_completed"] and not value["control_fully_defined"]
        assert value["mesh_counts"] == dict(nodes=0, elements=0)
        missing = value["control"]["fields"]["CurvatureNormalAngle"]
        assert missing["available"] is False and "not available" in missing["error"]
    define = json.loads((attempt / probe.RECEIPTS["define"]).read_text(encoding="utf-8"))
    steps = [row["step"] for row in define["snapshots"]]
    override_applied = profile not in probe.UNSET_LOCAL_GROWTH_PROFILES
    assert steps[:5] == ["created", "set_type", "set_location", "set_element_size", "set_growth_rate" if override_applied else "leave_local_growth_unset"]
    size = next(obj for obj in mesh.Children if obj.Name == "stage15_wall_base0")
    assert size.ElementSize.Value == .002 and size.ElementSize.Unit == "m"
    assert size.growth_writes == ([1.3] if override_applied else [])
    assert size.GrowthRate == (1.3 if override_applied else 0.0)
    assert define["local_growth_override_applied"] is override_applied
    assert define["global_readback"]["GrowthRate"]["value"] == 1.3
    assert size.ScopingMethod == ("Geometry" if profile == "geometry_scope" else "NamedSelection")
    assert (size.CaptureCurvature, size.CaptureProximity) == ((False, False) if profile == "capture_off" else (True, True))
    assert calls == ["tree_refresh"]


def test_nonzero_native_counts_fail_before_definitions(fake):
    stage, attempt, frozen, mesh, _, factory = fake
    mesh.Nodes = 1
    rendered = probe.render_scripts(attempt, stage, frozen)
    with pytest.raises(Exception, match="nonzero native"):
        exec(rendered["define_script"], factory())
    assert mesh.Children == []
    assert not (attempt / probe.RECEIPTS["define"]).exists()


def test_core_field_failure_not_defaulted(fake):
    stage, attempt, frozen, mesh, _, factory = fake
    rendered = probe.render_scripts(attempt, stage, frozen)
    exec(rendered["define_script"], factory())
    size = mesh.Children[-1]
    del size.ElementSize
    with pytest.raises(Exception, match="required sizing field unavailable"):
        exec(rendered["readback_script"], factory())
    assert not (attempt / probe.RECEIPTS["readback"]).exists()


def test_readback_requires_actual_unique_named_object(fake):
    stage, attempt, frozen, mesh, _, factory = fake
    rendered = probe.render_scripts(attempt, stage, frozen)
    exec(rendered["define_script"], factory())
    mesh.Children.append(mesh.Children[-1])
    with pytest.raises(Exception, match="unique actual"):
        exec(rendered["readback_script"], factory())


def test_journal_real_order_and_fresh_command_globals(fake):
    stage, attempt, frozen, mesh, calls, factory = fake
    rendered = probe.render_scripts(attempt, stage, frozen)
    class Geometry:
        def SetFile(self, **kwargs):
            calls.append("geometry_setfile")
        def GetGeometryProperties(self):
            return types.SimpleNamespace()
    class Component:
        def Update(self, **kwargs):
            assert kwargs == dict(AllDependencies=False)
            calls.append("geometry_update")
    class MeshComponent:
        def Refresh(self):
            calls.append("mesh_component_refresh")
        def Update(self, **kwargs):
            pytest.fail("Mesh component Update forbidden")
    class Container:
        def Edit(self, **kwargs):
            assert kwargs == dict(Interactive=False)
            calls.append("mesh_edit")
        def SendCommand(self, **kwargs):
            assert kwargs["Language"] == "Python"
            calls.append("send_command")
            exec(kwargs["Command"], factory())
        def Exit(self):
            calls.append("mesh_exit")
    geometry, container = Geometry(), Container()
    class System:
        def GetContainer(self, **kwargs):
            return geometry if kwargs["ComponentName"] == "Geometry" else container
        def GetComponent(self, **kwargs):
            assert kwargs["Name"] in ("Geometry", "Mesh")
            return Component() if kwargs["Name"] == "Geometry" else MeshComponent()
    def save(**kwargs):
        assert kwargs == dict(FilePath=str(attempt / "diagnostic.wbpj"), Overwrite=False)
        calls.append("save_new_attempt")
    scope = dict(GetTemplate=lambda **k: types.SimpleNamespace(CreateSystem=lambda: System()), Save=save, SetScriptVersion=lambda **k: None)
    exec(rendered["journal_script"], scope)
    assert calls == ["geometry_setfile", "geometry_update", "save_new_attempt", "mesh_component_refresh", "mesh_edit", "send_command", "tree_refresh", "send_command", "mesh_exit"]
    result = json.loads((attempt / "workbench_completion.json").read_text(encoding="utf-8"))
    assert result["status"] == "completed" and result["diagnostic_completed"]
    assert len(mesh.Children) == 2


@pytest.mark.parametrize("injected", ["mesh.GenerateMesh()", "mesh.Update()", "mesh.Refresh()", "mesh.InternalObject", "exec('x')", "import subprocess", "ExtAPI.DataModel.Tree.Refresh()"])
def test_ast_rejects_native_forbidden_paths(fake, injected):
    stage, _, _, _, _, _ = fake
    template = (stage / "mesh/meshing_sizing_diagnostic.py.in").read_text(encoding="utf-8")
    with pytest.raises(probe.RuntimeGateError):
        probe.audit_template(template + "\n" + injected)


def test_hypothesis_not_an_executable_script_input(fake):
    stage, attempt, frozen, _, _, _ = fake
    rendered = probe.render_scripts(attempt, stage, frozen)
    assert "hypothesis" not in rendered["configuration"]
    assert rendered["configuration"]["project"] == str(attempt / "diagnostic.wbpj")


@pytest.mark.parametrize("name", ["Nodes", "Elements"])
@pytest.mark.parametrize("raw", [0.0, False, True, "0", None])
def test_native_counts_reject_float_bool_and_coercible_values(fake, name, raw):
    stage, attempt, frozen, mesh, _, factory = fake
    setattr(mesh, name, raw)
    rendered = probe.render_scripts(attempt, stage, frozen)
    with pytest.raises(Exception, match="invalid native mesh count"):
        exec(rendered["define_script"], factory())
    assert mesh.Children == []


def test_refresh_native_errors_preserved_without_calling_definition_pass(fake):
    stage, attempt, frozen, _, _, factory = fake
    api = factory()["ExtAPI"]
    refresh = api.DataModel.Tree.Refresh
    def refresh_and_message():
        refresh()
        api.Application.Messages.append(types.SimpleNamespace(Severity="Error", DisplayString="Sizing remains underdefined after refresh"))
    api.DataModel.Tree.Refresh = refresh_and_message
    rendered = probe.render_scripts(attempt, stage, frozen)
    exec(rendered["define_script"], factory())
    exec(rendered["readback_script"], factory())
    for phase in probe.RECEIPTS:
        value = json.loads((attempt / probe.RECEIPTS[phase]).read_text(encoding="utf-8"))
        probe.validate_receipt(value, phase, dict(job_id="sizing-b001-a001", profile="baseline"), rendered["settings"])
        assert value["status"] == "completed" and not value["control_fully_defined"]
        assert value["message_capture_complete"] is True
        assert value["native_errors_present"] is (phase != "define")
        assert len(value["messages"]) == (0 if phase == "define" else 1)


@pytest.mark.parametrize("mode", ["row_limit", "character_limit", "capture_error", "partial_capture_error"])
def test_message_capture_bounds_and_unknown_error_status(fake, mode):
    stage, attempt, frozen, _, _, factory = fake
    api = factory()["ExtAPI"]
    if mode == "row_limit":
        api.Application.Messages = [types.SimpleNamespace(Severity="Warning", DisplayString="observation") for _ in range(101)]
    elif mode == "character_limit":
        api.Application.Messages = [types.SimpleNamespace(Severity="Warning", DisplayString="x" * 25000)]
    elif mode == "capture_error":
        api.Application = types.SimpleNamespace()
    else:
        def failing():
            yield types.SimpleNamespace(Severity="Warning", DisplayString="first message retained")
            raise RuntimeError("native enumeration failed")
        api.Application.Messages = failing()
    rendered = probe.render_scripts(attempt, stage, frozen)
    exec(rendered["define_script"], factory())
    value = json.loads((attempt / probe.RECEIPTS["define"]).read_text(encoding="utf-8"))
    probe.validate_receipt(value, "define", dict(job_id="sizing-b001-a001", profile="baseline"), rendered["settings"])
    assert value["message_capture_complete"] is False
    assert value["native_errors_present"] is None
    assert len(value["messages"]) <= 100 and value["message_characters"] <= 20000
    if mode.endswith("capture_error"):
        assert value["message_capture_error"] and not value["messages_truncated"]
        assert len(value["messages"]) == (1 if mode == "partial_capture_error" else 0)
    else:
        assert value["messages_truncated"] and value["message_capture_error"] is None
        assert len(value["messages"]) == (100 if mode == "row_limit" else 1)


@pytest.mark.parametrize("replacement", ["mesh_container.Refresh()", "mesh_component.Update()", "mesh_component.Refresh()\n    mesh_component.Refresh()"])
def test_journal_ast_allows_only_one_fixed_mesh_component_refresh(fake, replacement):
    stage, _, _, _, _, _ = fake
    template = (stage / "mesh/wb_sizing_diagnostic.wbjn.in").read_text(encoding="utf-8")
    with pytest.raises(probe.RuntimeGateError):
        probe.audit_template(template.replace("mesh_component.Refresh()", replacement), journal=True)


@pytest.mark.parametrize("profile", ["baseline", "omit_local_growth"])
def test_empirical_local_growth_transition_controlled_contrast(fake, profile):
    stage, attempt, frozen, mesh, _, factory = fake
    class EmpiricalControl(Control):
        @property
        def ObjectState(self):
            return "FullyDefined" if self.Location.Ids and not self.growth_writes else "UnderDefined"
        @ObjectState.setter
        def ObjectState(self, value):
            pass
    def add_sizing():
        obj = EmpiricalControl()
        mesh.Children.append(obj)
        return obj
    mesh.AddSizing = add_sizing
    rendered = probe.render_scripts(attempt, stage, frozen, profile)
    exec(rendered["define_script"], factory())
    exec(rendered["readback_script"], factory())
    define = json.loads((attempt / probe.RECEIPTS["define"]).read_text(encoding="utf-8"))
    by_step = {row["step"]: row for row in define["snapshots"]}
    assert by_step["created"]["object_state"] == "UnderDefined"
    assert by_step["set_type"]["object_state"] == "UnderDefined"
    assert by_step["set_location"]["object_state"] == "FullyDefined"
    assert by_step["set_element_size"]["object_state"] == "FullyDefined"
    for phase in probe.RECEIPTS:
        value = json.loads((attempt / probe.RECEIPTS[phase]).read_text(encoding="utf-8"))
        probe.validate_receipt(value, phase, dict(job_id="sizing-b001-a001", profile=profile), rendered["settings"])
        assert value["control_fully_defined"] is (profile == "omit_local_growth")
        assert value["local_growth_override_applied"] is (profile == "baseline")
        assert value["global_readback"]["GrowthRate"]["value"] == 1.3
    size = mesh.Children[-1]
    assert size.growth_writes == ([] if profile == "omit_local_growth" else [1.3])


def test_full_background_native_three_phases_all_controls_fresh_readback(fake):
    stage, attempt, frozen, mesh, calls, factory = fake
    mesh.state = "FullyDefined"
    rendered = probe.render_scripts(attempt, stage, frozen, "background_all_controls")
    exec(rendered["define_script"], factory())
    assert len(mesh.Children) == 4
    exec(rendered["readback_script"], factory())
    for phase in probe.RECEIPTS:
        value = json.loads((attempt / probe.RECEIPTS[phase]).read_text(encoding="utf-8"))
        request = dict(job_id="sizing-b001-a001", profile="background_all_controls")
        probe.validate_receipt(value, phase, request, rendered["settings"])
        assert probe.background_definitions_match(value, rendered["settings"])
        assert value["scope"] == "background_all_required_controls"
        assert value["full_background_definitions_verified"] is False  # Native editor cannot verify owned cleanup.
        assert len(value["background_controls"]) == 4 and len(value["named_scopes"]) == 5
        assert value["mesh_counts"] == dict(nodes=0, elements=0)
        assert value["local_growth_override_applied"] is False
        assert value["native_errors_present"] is False and value["message_capture_complete"]
        sphere = next(row for row in value["background_controls"] if row["kind"] == "sphere")
        assert sphere["sphere_radius"] == dict(value=.65, unit="m", text="0.65 [m]")
        assert sphere["coordinate_system"]["object_state"] == "FullyDefined"
        assert sphere["sphere_center_object_id"] == sphere["coordinate_system"]["object_id"]
    for size in mesh.Children[1:]:
        assert size.growth_writes == []
        assert size.ElementSize.conversions == []
        if size.Type == "SphereOfInfluence":
            assert size.SphereRadius.conversions == []
            assert all(getattr(size.SphereCenter, name).conversions == [] for name in ("OriginX", "OriginY", "OriginZ"))
    assert calls == ["tree_refresh"]


@pytest.mark.parametrize("mutation", ["missing", "empty", "duplicate", "overlap", "outside"])
def test_full_background_requires_all_five_valid_named_scopes_before_creation(fake, mutation):
    stage, attempt, frozen, mesh, _, factory = fake
    model = factory()["ExtAPI"].DataModel.Project.Model
    selections = model.NamedSelections.Children
    if mutation == "missing":
        selections.pop()
    elif mutation == "empty":
        selections[-1].Location.Ids = []
    elif mutation == "duplicate":
        selections.append(selections[-1])
    elif mutation == "overlap":
        selections[-1].Location.Ids = [7]
    else:
        selections[-1].Location.Ids = [999]
    rendered = probe.render_scripts(attempt, stage, frozen, "background_all_controls")
    with pytest.raises(Exception, match="required"):
        exec(rendered["define_script"], factory())
    assert mesh.Children == [] and not (attempt / probe.RECEIPTS["define"]).exists()


@pytest.mark.parametrize("target", ["stage15_wall_auv", "stage15_overlap_0", "coordinate"])
def test_full_background_second_command_reobtains_every_required_object(fake, target):
    stage, attempt, frozen, mesh, _, factory = fake
    rendered = probe.render_scripts(attempt, stage, frozen, "background_all_controls")
    exec(rendered["define_script"], factory())
    if target == "coordinate":
        factory()["ExtAPI"].DataModel.Project.Model.CoordinateSystems.Children.clear()
    else:
        mesh.Children[:] = [obj for obj in mesh.Children if obj.Name != target]
    with pytest.raises(Exception, match="unique actual object"):
        exec(rendered["readback_script"], factory())
    assert not (attempt / probe.RECEIPTS["readback"]).exists()


def test_full_background_readback_preserves_raw_mm_quantities_without_conversion(fake):
    stage, attempt, frozen, mesh, _, factory = fake
    mesh.state = "FullyDefined"
    rendered = probe.render_scripts(attempt, stage, frozen, "background_all_controls")
    exec(rendered["define_script"], factory())
    quantities = []
    for size in mesh.Children[1:]:
        quantities.append(size.ElementSize)
        if size.Type == "SphereOfInfluence":
            quantities.append(size.SphereRadius)
            quantities.extend(getattr(size.SphereCenter, key) for key in ("OriginX", "OriginY", "OriginZ"))
    for actual in quantities:
        actual.Value *= 1000
        actual.Unit = "mm"
    exec(rendered["readback_script"], factory())
    value = json.loads((attempt / probe.RECEIPTS["readback"]).read_text(encoding="utf-8"))
    probe.validate_receipt(value, "readback", dict(job_id="sizing-b001-a001", profile="background_all_controls"), rendered["settings"])
    assert probe.background_definitions_match(value, rendered["settings"])
    assert value["control"]["fields"]["ElementSize"]["value"]["unit"] == "mm"
    assert all(actual.Unit == "mm" and actual.conversions == [] for actual in quantities)
