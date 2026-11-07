import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

STAGE = Path(__file__).resolve().parents[1]


def load_functions(filename, names, ns):
    tree = ast.parse((STAGE / "cad" / filename).read_text(encoding="utf-8"))
    nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    if len(nodes) != len(names):
        raise AssertionError("missing template functions")
    exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, "exec"), ns)


def point(x, y=0.0, z=0.0):
    return SimpleNamespace(X=x, Y=y, Z=z)


class WallOwnershipSamplingTests(unittest.TestCase):
    def setUp(self):
        self.owner = object()
        self.ns = {"math": math, "TOL_OWNER": 1e-5, "TOL_SEAL": 1e-7,
                   "WALL_NAME": {"jaw1": "wall_jaw1", "jaw2": "wall_jaw2"},
                   "Point": SimpleNamespace(Create=point)}
        load_functions("sc_common.py.in", {"u", "finite", "xyz", "dist", "as_point", "BlockedEvidence"}, self.ns)
        load_functions("sc_domains.py.in", {"face_record", "classify_faces"}, self.ns)
        self.ns["expected_names"] = lambda domain: ["wall_jaw1", "wall_jaw2"]
        self.ns["bbox"] = lambda face: face.bounds
        self.ns["outer_side"] = lambda face, lo, hi: None
        self.sampled_owners = []
        def sample(face, owner=None, diagnostics=None):
            self.sampled_owners.append(owner)
            diagnostics.update(state="passed", fallback_used=True)
            return face.points
        self.ns["surface_face_samples"] = sample
        self.tools = {
            "jaw1": SimpleNamespace(ProjectPoint=lambda p: SimpleNamespace(Point=point(max(0.0, p.X), p.Y, p.Z))),
            "jaw2": SimpleNamespace(ProjectPoint=lambda p: SimpleNamespace(Point=point(min(0.0, p.X), p.Y, p.Z))),
        }

    def body(self):
        faces = []
        for sign in (1.0, -1.0):
            face = SimpleNamespace(Area=1.3e-8, bounds=[[-1e-5, -1e-5, 0.0], [1e-5, 1e-5, 0.0]],
                                   Geometry=SimpleNamespace(GetType=lambda: SimpleNamespace(Name="Plane")),
                                   points=[point(sign * 2e-6), point(sign * 5e-6, 1e-6), point(sign * 8e-6, 2e-6)])
            faces.append(SimpleNamespace(Shape=face))
        return SimpleNamespace(Faces=faces, GetMaster=lambda: SimpleNamespace(Shape=self.owner))

    def classify(self, body):
        return self.ns["classify_faces"](body, "arm", ["jaw1", "jaw2"], self.tools, [-1, -1, -1], [1, 1, 1])

    def test_shared_boundary_point_tie_resolves_only_with_spread_validated_points(self):
        center = point(0, 0, 0)
        self.assertEqual(self.ns["dist"](self.tools["jaw1"].ProjectPoint(center).Point, center), 0)
        self.assertEqual(self.ns["dist"](self.tools["jaw2"].ProjectPoint(center).Point, center), 0)
        groups, report = self.classify(self.body())
        self.assertEqual(report["state"], "passed")
        self.assertEqual(len(groups["wall_jaw1"]), 1)
        self.assertEqual(len(groups["wall_jaw2"]), 1)
        self.assertEqual(self.sampled_owners, [self.owner, self.owner])
        self.assertTrue(all(row["sample_count"] >= 3 for row in report["face_records"]))

    def test_one_point_is_never_accepted_for_naming(self):
        body = self.body()
        for item in body.Faces:
            item.Shape.points = [point(0)]
        _, report = self.classify(body)
        self.assertEqual(report["state"], "blocked")
        self.assertEqual(len(report["sampling_unresolved"]), 2)

    def test_real_multisample_tie_remains_blocked(self):
        self.tools = {name: SimpleNamespace(ProjectPoint=lambda p: SimpleNamespace(Point=p)) for name in ("jaw1", "jaw2")}
        _, report = self.classify(self.body())
        self.assertEqual(report["state"], "blocked")
        self.assertEqual(len(report["ambiguous"]), 2)

    def test_sampling_failure_retains_geometric_fingerprint(self):
        def fail(*args, **kwargs):
            raise self.ns["BlockedEvidence"]("synthetic sampling failure")
        self.ns["surface_face_samples"] = fail
        _, report = self.classify(self.body())
        self.assertEqual(report["state"], "blocked")
        row = report["sampling_unresolved"][0]
        self.assertIn("bbox_m", row)
        self.assertEqual(row["area_m2"], 1.3e-8)
        self.assertIn("synthetic sampling failure", row["sampling_error"])

    def test_no_geometric_owner_remains_failed(self):
        self.tools = {name: SimpleNamespace(ProjectPoint=lambda p: SimpleNamespace(Point=point(1.0, 1.0, 1.0))) for name in ("jaw1", "jaw2")}
        _, report = self.classify(self.body())
        self.assertEqual(report["state"], "failed")
        self.assertEqual(len(report["unassigned"]), 2)

    def test_independent_verify_uses_the_same_classifier(self):
        tree = ast.parse((STAGE / "cad/sc_domains.py.in").read_text(encoding="utf-8"))
        verify = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "verify_domain")
        calls = [node.func.id for node in ast.walk(verify) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)]
        self.assertIn("classify_faces", calls)


if __name__ == "__main__":
    unittest.main()
