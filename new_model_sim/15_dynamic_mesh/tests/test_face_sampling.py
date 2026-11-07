import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

STAGE = Path(__file__).resolve().parents[1]


def declarations(filename, names):
    text = (STAGE / "cad" / filename).read_text(encoding="utf-8")
    tree = ast.parse(text)
    found = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names]
    if len(found) != len(names):
        raise AssertionError("Missing template declarations")
    return compile(ast.Module(body=found, type_ignores=[]), filename, "exec")


def point(x, y, z=0.0):
    return SimpleNamespace(X=x, Y=y, Z=z)


class TrimmedFaceSamplingTests(unittest.TestCase):
    def setUp(self):
        self.ns = {"math": math, "TOL_OWNER": 1e-5,
                   "Point": SimpleNamespace(Create=point),
                   "PointUV": SimpleNamespace(Create=lambda u, v: SimpleNamespace(U=u, V=v))}
        exec(declarations("sc_common.py.in", {"u", "finite", "xyz", "dist", "as_point", "BlockedEvidence"}), self.ns)
        exec(declarations("sc_domains.py.in", {"projected_facet_samples", "surface_face_samples", "directed_surface_check"}), self.ns)
        self.owner = object()
        self.calls = 0
        self.tess = SimpleNamespace(
            Vertices=[SimpleNamespace(Position=p) for p in (point(0.013, 0.017), point(0.0134, 0.017), point(0.013, 0.0174))],
            Facets=[SimpleNamespace(Vertex0=0, Vertex1=1, Vertex2=2)])
        def native(owner, face, deviation):
            self.assertIs(owner, self.owner)
            self.assertEqual(deviation, 2e-8)
            self.calls += 1
            return self.tess
        self.ns["native_face_tessellation"] = native
        self.ns["samples"] = lambda face: face.initial
        self.blocked = self.ns["BlockedEvidence"]

    def face(self):
        class Face:
            Area = 8e-8
            initial = [point(0.013, 0.017)]
            BoxUV = SimpleNamespace(RangeU=SimpleNamespace(Start=0.0, End=1.0), RangeV=SimpleNamespace(Start=0.0, End=1.0))
            bounds = [[0.013, 0.017, 0.0], [0.0134, 0.0174, 0.0]]
            def ContainsParam(self, uv):
                u, v = uv.U - 0.013, uv.V - 0.017
                return u >= -1e-15 and v >= -1e-15 and u + v <= 0.0004 + 1e-15
            def ProjectPoint(self, p):
                return SimpleNamespace(Point=point(p.X, p.Y, 0.0), Param=SimpleNamespace(U=p.X, V=p.Y))
        face = Face()
        face.Geometry = SimpleNamespace(Evaluate=lambda uv: SimpleNamespace(Point=point(uv.U, uv.V)))
        return face

    def test_narrow_trimmed_face_missed_by_uv_grid_gets_projected_points(self):
        face, details = self.face(), {}
        result = self.ns["surface_face_samples"](face, self.owner, details)
        self.assertGreaterEqual(len(result), 3)
        self.assertEqual(details["uv_distinct_count"], 1)
        self.assertEqual(details["uv_grids_tried"], [12, 24])
        self.assertTrue(details["fallback_used"])
        self.assertEqual(details["state"], "passed")
        self.assertFalse(details["geometry_modified"])
        for sample in result:
            self.assertTrue(face.ContainsParam(SimpleNamespace(U=sample.X, V=sample.Y)))
        self.assertEqual(self.calls, 1)

    def test_sufficient_uv_points_do_not_call_sdk_fallback(self):
        face, details = self.face(), {}
        face.initial = [v.Position for v in self.tess.Vertices]
        result = self.ns["surface_face_samples"](face, None, details)
        self.assertEqual(len(result), 3)
        self.assertFalse(details["fallback_used"])
        self.assertEqual(self.calls, 0)

    def test_missing_owner_stays_blocked(self):
        details = {}
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](self.face(), None, details)
        self.assertEqual(details["state"], "blocked")
        self.assertEqual(self.calls, 0)

    def test_projection_outside_trim_does_not_rescue_face(self):
        face, details = self.face(), {}
        face.ContainsParam = lambda uv: False
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](face, self.owner, details)
        self.assertEqual(details["projected_facet_sampling"]["accepted_candidate_count"], 0)

    def test_duplicate_projected_points_do_not_count_as_three(self):
        face, details = self.face(), {}
        face.ProjectPoint = lambda p: SimpleNamespace(Point=point(0.013, 0.017), Param=SimpleNamespace(U=0.013, V=0.017))
        # Keep projection displacement within tolerance while collapsing all points.
        self.tess.Vertices = [SimpleNamespace(Position=p) for p in (point(0.013, 0.017), point(0.01300001, 0.017), point(0.013, 0.01700001))]
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](face, self.owner, details)
        self.assertEqual(details["final_distinct_count"], 1)
        self.assertFalse(details["fallback_used"])

    def test_no_tessellation_facets_is_not_an_exemption(self):
        self.tess.Facets = []
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](self.face(), self.owner, {})

    def test_tessellation_budget_is_enforced(self):
        self.tess.Facets *= 4097
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](self.face(), self.owner, {})

    def test_bad_vertex_index_is_rejected(self):
        self.tess.Facets[0].Vertex0 = -1
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](self.face(), self.owner, {})

    def test_degenerate_facets_are_not_points_evidence(self):
        self.tess.Facets[0].Vertex2 = 1
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](self.face(), self.owner, {})

    def test_missing_trim_parameter_is_rejected(self):
        face = self.face()
        face.ProjectPoint = lambda p: SimpleNamespace(Point=p)
        with self.assertRaises(self.blocked):
            self.ns["surface_face_samples"](face, self.owner, {})

    def test_nonfinite_tessellation_vertex_rejected(self):
        self.tess.Vertices[0].Position.X = float("nan")
        with self.assertRaises(ValueError):
            self.ns["surface_face_samples"](self.face(), self.owner, {})

    def test_sampling_error_keeps_face_fingerprint(self):
        face = self.face()
        source = SimpleNamespace(Faces=[face])
        self.ns["bbox"] = lambda obj: obj.bounds
        self.ns["surface_index"] = lambda target: {"faces": [face], "cell_size_m": 1.0, "bins": {}, "large_faces": []}
        def reject(*args, **kwargs):
            kwargs["diagnostics"]["state"] = "blocked"
            raise self.blocked("synthetic sampling failure")
        self.ns["surface_face_samples"] = reject
        result = self.ns["directed_surface_check"](source, source)
        row = result["face_results"][0]
        self.assertEqual(result["state"], "blocked")
        self.assertEqual(row["state"], "blocked")
        self.assertEqual(row["bbox_m"], face.bounds)
        self.assertEqual(row["area_m2"], face.Area)
        self.assertIn("synthetic sampling failure", row["sampling_error"])
        self.assertFalse(result["all_source_faces_sampled"])


if __name__ == "__main__":
    unittest.main()
