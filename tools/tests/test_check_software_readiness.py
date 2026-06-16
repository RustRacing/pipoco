import unittest

from tools.check_software_readiness import readiness_commands


class SoftwareReadinessToolTests(unittest.TestCase):
    def test_readiness_commands_cover_expected_software_gates(self):
        commands = readiness_commands()
        names = {command.name for command in commands}

        self.assertIn("profile-target compatibility", names)
        self.assertIn("simulator readiness aggregation", names)
        self.assertIn("tuner studio page validity", names)
        self.assertIn("evidence metadata tooling", names)

    def test_readiness_commands_use_python3_for_python_gates(self):
        python_commands = [
            command for command in readiness_commands() if command.argv[0].startswith("python")
        ]

        self.assertTrue(python_commands)
        self.assertTrue(all(command.argv[0] == "python3" for command in python_commands))


if __name__ == "__main__":
    unittest.main()
