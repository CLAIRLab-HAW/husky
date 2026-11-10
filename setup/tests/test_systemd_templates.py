"""The units and wrappers under ``systemd/`` are rolled out, verified and fully rendered by the installer.

Each template has to appear twice in the installer -- in an ``install_template`` call and in the ``--verify`` manifest
-- and every ``@NAME@`` in it has to be a variable the installer sets.  The rendering runs the installer's own
``render_template``, the way ``test_arm_files`` runs ``arm_files``.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

SETUP = Path(__file__).resolve().parents[1]
INSTALLER = SETUP / "install-clearpath-custom-setup.sh"
TEMPLATES = sorted(p.relative_to(SETUP).as_posix() for p in (SETUP / "systemd").rglob("*.in"))
PLACEHOLDER = re.compile(r"@([A-Z_][A-Z0-9_]*)@")


def render(template: Path, **values: str) -> subprocess.CompletedProcess[str]:
    lines = INSTALLER.read_text().splitlines()
    start = lines.index("render_template() {")
    function = "\n".join(lines[start : lines.index("}", start) + 1])
    assignments = "\n".join(f"{name}={value!r}" for name, value in values.items())
    return subprocess.run(
        ["bash", "-c", f'set -euo pipefail\n{assignments}\n{function}\nrender_template "$1"', "_", str(template)],
        capture_output=True,
        text=True,
        timeout=10,
    )


@pytest.mark.parametrize("rel", TEMPLATES)
def test_template_is_installed_and_verified(rel):
    text = INSTALLER.read_text()
    assert f"install_template {rel} " in text, f"{rel}: no install_template call rolls it out"
    assert f'|{rel}"' in text, f"{rel}: missing from the --verify manifest"


@pytest.mark.parametrize("rel", TEMPLATES)
def test_every_placeholder_is_an_installer_variable(rel):
    assigned = set(re.findall(r"^\s*([A-Z_][A-Z0-9_]*)=", INSTALLER.read_text(), re.MULTILINE))
    missing = set(PLACEHOLDER.findall((SETUP / rel).read_text())) - assigned
    assert not missing, f"{rel} names {sorted(missing)}, which the installer never sets"


def test_render_substitutes_values_verbatim(tmp_path):
    template = tmp_path / "x.service.in"
    template.write_text("User=@REAL_USER@\nExecStart=@BIN@ @BIN@\n")
    result = render(template, REAL_USER="robot", BIN="/usr/local/bin/a&b")
    assert result.returncode == 0, result.stderr
    # Neither literal quotes (the first draft wrote User="robot") nor bash 5.2's & for the match.
    assert result.stdout == "User=robot\nExecStart=/usr/local/bin/a&b /usr/local/bin/a&b\n"


def test_render_refuses_an_unknown_placeholder(tmp_path):
    template = tmp_path / "x.service.in"
    template.write_text("User=@NOPE@\n")
    result = render(template)
    assert result.returncode != 0
    assert "@NOPE@" in result.stderr
