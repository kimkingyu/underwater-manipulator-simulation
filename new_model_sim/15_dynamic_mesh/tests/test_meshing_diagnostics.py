import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

STAGE = Path(__file__).resolve().parents[1]


class MeshingFailureDiagnosticsTests(unittest.TestCase):
    def execute(self, missing_selection=False, positive_mesh=False):
        with tempfile.TemporaryDirectory(prefix="_test_mesh_diag_", dir=str(STAGE / "runs")) as directory:
            path = Path(directory) / "stats.json"
            messages = []
            mesh = SimpleNamespace(Nodes=0, Elements=0, Children=[], ObjectState="UnderDefined", Properties=[])
            method = SimpleNamespace(Name="Automatic Method", ObjectState="UnderDefined", Properties=[])
            def add_method():
                mesh.Children.append(method)
                return method
            mesh.AddAutomaticMethod = add_method
            def generate():
                if positive_mesh:
                    mesh.Nodes, mesh.Elements = 10, 20
                    mesh.ObjectState = "Meshed"
                else:
                    messages.append(SimpleNamespace(Severity="Error", DisplayString="synthetic invalid mesh definition"))
                return None
            mesh.GenerateMesh = generate
            body = SimpleNamespace(Name="fluid_background", GetGeoBody=lambda: SimpleNamespace(Id=7), Properties=[])
            selection = SimpleNamespace(Name="wall_auv", Location=SimpleNamespace(Ids=[1]))
            model = SimpleNamespace(Mesh=mesh, Geometry=SimpleNamespace(GetChildren=lambda *args: [body]),
                                    NamedSelections=SimpleNamespace(Children=[] if missing_selection else [selection]),
                                    CoordinateSystems=SimpleNamespace(Children=[]))
            if missing_selection:
                messages.append(SimpleNamespace(Severity="Error", DisplayString="synthetic missing named selection"))
            cfg = {"domain": "background", "stats_path": str(path), "cell_zone": "fluid_background",
                   "required_selections": ["wall_auv"], "global_size_m": 0.15, "maximum_size_m": 0.2,
                   "minimum_size_m": 0.0003, "growth_rate": 1.3, "face_controls": [], "spheres": []}
            ns = {"ExtAPI": SimpleNamespace(DataModel=SimpleNamespace(Project=SimpleNamespace(Model=model)),
                                               Application=SimpleNamespace(Messages=messages),
                                               SelectionManager=SimpleNamespace(CreateSelectionInfo=lambda *args: SimpleNamespace(Ids=[]))),
                  "DataModelObjectCategory": SimpleNamespace(Body="Body"),
                  "SelectionTypeEnum": SimpleNamespace(GeometryEntities="GeometryEntities"),
                  "MethodType": SimpleNamespace(AllTriAllTet="AllTriAllTet"),
                  "Quantity": lambda value: value}
            source = (STAGE / "mesh/meshing_stage15.py.in").read_text(encoding="utf-8").replace("__CONFIG_JSON__", repr(json.dumps(cfg)))
            error = None
            try:
                exec(compile(source, "meshing_diagnostic_fixture", "exec"), ns)
            except Exception as exc:
                error = str(exc)
            return json.loads(path.read_text(encoding="utf-8")), error

    def test_zero_cells_retains_native_error_and_definition_states(self):
        report, error = self.execute()
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["mesh_generated"])
        self.assertEqual(report["elements"], 0)
        self.assertIn("synthetic invalid mesh definition", report["messages"][0]["text"])
        self.assertEqual(report["definitions_before_generate"]["mesh_children"][0]["properties"]["ObjectState"], "UnderDefined")
        self.assertIsNotNone(error)

    def test_pre_generation_failure_still_retains_messages(self):
        report, error = self.execute(missing_selection=True)
        self.assertEqual(report["status"], "failed")
        self.assertIn("synthetic missing named selection", report["messages"][0]["text"])
        self.assertIsNotNone(error)

    def test_positive_counts_are_execution_evidence_not_mesh_acceptance(self):
        report, error = self.execute(positive_mesh=True)
        self.assertEqual(report["status"], "generated_pending_export_and_fluent_checks")
        self.assertTrue(report["mesh_generated"])
        self.assertFalse(report["valid"])
        self.assertFalse(report["motion_authorized"])
        self.assertIsNone(error)


if __name__ == "__main__":
    unittest.main()
