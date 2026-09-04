import json
import os
import subprocess
import sys
import tempfile

import pytest

TOOL = "stretch4_body.tools.stretch_package_create"

EXPECTED = [
    "AGENTS.md",
    "CLAUDE.md",
    "README.md",
    "LICENSE",
    ".gitignore",
    ".claude/settings.json",
    ".claude/hooks/guard_hardware.py",
    ".antigravity/settings.json",
    "routines/example_status_log.py",
    "routines/example_random_pose.py",
]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run_tool(*args):
    """Invoke the scaffolder as a subprocess.

    Deliberately not an import: modules under stretch4_body/tools are not safe
    to import from a test, and an `if __name__` guard on one of them is not a
    reason to make an exception.
    """
    return subprocess.run(
        [sys.executable, "-m", TOOL] + list(args),
        cwd=REPO_ROOT, capture_output=True, text=True,
    )


@pytest.fixture
def parent():
    with tempfile.TemporaryDirectory() as d:
        yield d


def test_creates_every_expected_file(parent):
    result = run_tool("proj", "-C", parent)
    assert result.returncode == 0, result.stderr
    for rel in EXPECTED:
        assert os.path.isfile(os.path.join(parent, "proj", rel)), rel


def test_hook_and_routines_are_executable(parent):
    run_tool("proj", "-C", parent)
    for rel in (".claude/hooks/guard_hardware.py",
                "routines/example_random_pose.py",
                "routines/example_status_log.py"):
        assert os.access(os.path.join(parent, "proj", rel), os.X_OK), rel


def test_settings_are_valid_json(parent):
    run_tool("proj", "-C", parent)
    for rel in (".claude/settings.json", ".antigravity/settings.json"):
        with open(os.path.join(parent, "proj", rel)) as f:
            json.load(f)


def test_placeholders_are_substituted(parent):
    run_tool("proj", "-C", parent, "--author", "Someone")
    root = os.path.join(parent, "proj")
    for rel in EXPECTED:
        with open(os.path.join(root, rel)) as f:
            assert "{{" not in f.read(), rel
    with open(os.path.join(root, "LICENSE")) as f:
        assert "Someone" in f.read()
    with open(os.path.join(root, "AGENTS.md")) as f:
        assert f.read().startswith("# proj")


def test_guard_hook_self_test_passes(parent):
    run_tool("proj", "-C", parent)
    root = os.path.join(parent, "proj")
    result = subprocess.run(
        [sys.executable, ".claude/hooks/guard_hardware.py", "--self-test"],
        cwd=root, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_refuses_to_clobber_without_force(parent):
    assert run_tool("proj", "-C", parent).returncode == 0
    second = run_tool("proj", "-C", parent)
    assert second.returncode == 1
    assert "--force" in second.stderr
    assert run_tool("proj", "-C", parent, "--force").returncode == 0


def test_list_does_not_write_anything(parent):
    result = run_tool("ghost", "-C", parent, "--list")
    assert result.returncode == 0
    assert "AGENTS.md" in result.stdout
    assert not os.path.exists(os.path.join(parent, "ghost"))
