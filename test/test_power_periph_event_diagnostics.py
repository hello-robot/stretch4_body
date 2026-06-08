import ast
import pathlib
import unittest


ROOT_DIR = pathlib.Path(__file__).resolve().parents[1]
POWER_PERIPH_PATH = ROOT_DIR / "stretch4_body" / "subsystem" / "power_periph.py"
ROBOT_CLIENT_PATH = ROOT_DIR / "stretch4_body" / "robot" / "robot_client.py"


def parse(path):
    return ast.parse(path.read_text(), filename=str(path))


def get_class(tree, name):
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == name:
            return node
    raise AssertionError(f"missing class {name}")


def get_method(class_node, name):
    for node in class_node.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"missing method {class_node.name}.{name}")


class TestPowerPeriphEventDiagnostics(unittest.TestCase):
    def test_trigger_bits_are_defined(self):
        power_periph = get_class(parse(POWER_PERIPH_PATH), "PowerPeriphDefn")
        assignments = {
            node.targets[0].id: ast.unparse(node.value)
            for node in power_periph.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
        }

        self.assertEqual(assignments["TRIGGER_EVENT_DIAGNOSTICS_ENABLE"], "1 << 24")
        self.assertEqual(assignments["TRIGGER_EVENT_DIAGNOSTICS_DISABLE"], "1 << 25")

    def test_direct_api_sets_diagnostic_trigger_bits(self):
        trace = get_class(parse(POWER_PERIPH_PATH), "PowerPeriphTrace")
        enable_source = ast.unparse(get_method(trace, "enable_event_diagnostics"))
        disable_source = ast.unparse(get_method(trace, "disable_event_diagnostics"))

        self.assertIn("TRIGGER_EVENT_DIAGNOSTICS_ENABLE", enable_source)
        self.assertIn("_dirty_trigger = True", enable_source)
        self.assertIn("TRIGGER_EVENT_DIAGNOSTICS_DISABLE", disable_source)
        self.assertIn("_dirty_trigger = True", disable_source)

    def test_robot_client_queues_diagnostic_commands(self):
        client = get_class(parse(ROBOT_CLIENT_PATH), "PowerPeriphClient")
        enable_source = ast.unparse(get_method(client, "enable_event_diagnostics"))
        disable_source = ast.unparse(get_method(client, "disable_event_diagnostics"))

        self.assertIn("power_periph", enable_source)
        self.assertIn("enable_event_diagnostics", enable_source)
        self.assertIn("power_periph", disable_source)
        self.assertIn("disable_event_diagnostics", disable_source)


if __name__ == "__main__":
    unittest.main()
