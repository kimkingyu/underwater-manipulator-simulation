import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

STAGE = Path(__file__).resolve().parents[1]


class MeshingFaceControlTests(unittest.TestCase):
    def namespace(self, remain_undefined=False):
        class Size:
            Properties = []
            Name = "new"
            Type = None
            Location = None
            ElementSize = None
            GrowthRate = 0.0
            @property
            def ObjectState(self):
                # Native A/B reproduction: a local growth override invalidates
                # this otherwise valid face control with capture disabled.
                if remain_undefined or self.GrowthRate != 0.0:
                    return "UnderDefined"
                return "FullyDefined" if self.Type == "ElementSize" and hasattr(self.Location, "Ids") else "UnderDefined"
        sizes = []
        def add():
            size = Size()
            sizes.append(size)
            return size
        ns = {"R": {}, "Quantity": lambda v: v,
              "SizingType": SimpleNamespace(ElementSize="ElementSize"),
              "SelectionTypeEnum": SimpleNamespace(GeometryEntities="GeometryEntities"),
              "ExtAPI": SimpleNamespace(SelectionManager=SimpleNamespace(CreateSelectionInfo=lambda *args: SimpleNamespace(Ids=[])),
                                         Application=SimpleNamespace(Messages=[]))}
        tree = ast.parse((STAGE / "mesh/meshing_stage15.py.in").read_text(encoding="utf-8"))
        names = {"native_text", "object_snapshot", "capture_messages", "configure_face_size"}
        nodes = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
        exec(compile(ast.Module(body=nodes, type_ignores=[]), "meshing_controls", "exec"), ns)
        return ns, SimpleNamespace(AddSizing=add), sizes

    def test_named_selection_ids_are_copied_to_explicit_geometry_scope(self):
        ns, mesh, sizes = self.namespace()
        named = SimpleNamespace(Location=SimpleNamespace(Ids=[11, 22, 33]))
        size = ns["configure_face_size"](mesh, named, {"name": "wall_base0", "size_m": 0.002}, 1.3)
        self.assertIsNot(size.Location, named)
        self.assertIsNot(size.Location, named.Location)
        self.assertEqual(size.Location.Ids, [11, 22, 33])
        self.assertEqual(size.Type, "ElementSize")
        self.assertEqual(size.ObjectState, "FullyDefined")
        self.assertEqual(ns["R"]["local_readback"][0]["actual_geometry_ids"], [11, 22, 33])

    def test_native_growth_counterexample_is_avoided(self):
        ns, mesh, _ = self.namespace()
        named = SimpleNamespace(Location=SimpleNamespace(Ids=[11, 22]))
        size = ns["configure_face_size"](mesh, named, {"name": "wall_base0", "size_m": 0.002}, 1.3)
        self.assertEqual(size.ObjectState, "FullyDefined")
        self.assertEqual(size.GrowthRate, 0.0)
        row = ns["R"]["local_readback"][0]
        self.assertFalse(row["local_growth_override_applied"])
        self.assertEqual(row["global_growth_rate_requested"], 1.3)
        size.GrowthRate = 1.3
        self.assertEqual(size.ObjectState, "UnderDefined")

    def test_face_helpers_do_not_assign_local_growth(self):
        for filename in ("meshing_stage15.py.in", "meshing_definition_probe.py.in"):
            tree = ast.parse((STAGE / "mesh" / filename).read_text(encoding="utf-8"))
            helper = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "configure_face_size")
            for node in ast.walk(helper):
                self.assertFalse(isinstance(node, ast.Attribute) and node.attr == "GrowthRate" and isinstance(node.ctx, ast.Store))
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in ("setattr", "exact_setting"):
                    self.assertFalse(len(node.args) > 1 and isinstance(node.args[1], ast.Constant) and node.args[1].value == "GrowthRate")

    def test_undefined_control_is_not_silently_used(self):
        ns, mesh, _ = self.namespace(remain_undefined=True)
        named = SimpleNamespace(Location=SimpleNamespace(Ids=[11]))
        with self.assertRaisesRegex(Exception, "remains undefined"):
            ns["configure_face_size"](mesh, named, {"name": "wall_auv", "size_m": 0.04}, 1.3)
        self.assertEqual(ns["R"]["local_readback"][0]["definition"]["properties"]["ObjectState"], "UnderDefined")

    def test_empty_or_duplicate_scope_is_rejected(self):
        for ids in ([], [1, 1]):
            ns, mesh, sizes = self.namespace()
            with self.assertRaises(Exception):
                ns["configure_face_size"](mesh, SimpleNamespace(Location=SimpleNamespace(Ids=ids)),
                                           {"name": "wall_auv", "size_m": 0.04}, 1.3)
            self.assertFalse(sizes)

    def test_unicode_messages_are_retained(self):
        ns, _, _ = self.namespace()
        ns["ExtAPI"].Application.Messages = [SimpleNamespace(Severity="Error", DisplayString="网格定义不完整")]
        ns["capture_messages"]()
        self.assertEqual(ns["R"]["messages"][0]["text"], "网格定义不完整")
        self.assertTrue(ns["R"]["message_capture_complete"])

    def test_snapshot_does_not_evaluate_hidden_property_values(self):
        reads = []
        class Property:
            Name = "hidden-native-property"
            IsValid = False
            ReadOnly = True
            @property
            def Value(self):
                reads.append("Value")
                raise RuntimeError("must not read")
            @property
            def InternalValue(self):
                reads.append("InternalValue")
                raise RuntimeError("must not read")
        ns, _, _ = self.namespace()
        result = ns["object_snapshot"](SimpleNamespace(Name="control", Properties=[Property()]))
        self.assertFalse(reads)
        self.assertEqual(result["definition_properties"][0]["Name"], "hidden-native-property")

    def test_unreadable_text_does_not_hide_error_severity(self):
        class Message:
            Severity = "Error"
            @property
            def DisplayString(self):
                raise UnicodeError("synthetic native string failure")
        ns, _, _ = self.namespace()
        ns["ExtAPI"].Application.Messages = [Message()]
        ns["capture_messages"]()
        self.assertEqual(ns["R"]["messages"][0]["severity"], "Error")
        self.assertIn("text_error", ns["R"]["messages"][0])
        self.assertFalse(ns["R"]["message_capture_complete"])


if __name__ == "__main__":
    unittest.main()
