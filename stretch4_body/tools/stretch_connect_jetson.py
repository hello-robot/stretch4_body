#!/usr/bin/env python3
"""
stretch_connect_jetson — Full Jetson bring-up: power cycle, SSH, Wi-Fi, terminal.

Phases:
  [1/5] Power cycle Jetson via PowerPeriph (aux-CPU off → on)
  [2/5] Wait for SSH to become available (ping + SSH poll)
  [3/5] Set up passwordless SSH (sshpass + ssh-copy-id)
  [4/5] Configure Wi-Fi on Jetson (RTL8188EUS driver + nmcli)
  [5/5] Open interactive SSH terminal

Usage:
    stretch_connect_jetson                   # full flow
    stretch_connect_jetson --skip-power-cycle  # Jetson already on
    stretch_connect_jetson --skip-wifi         # Wi-Fi already configured
"""

import os
import subprocess
import sys
import time

import click

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JETSON_HOST    = "192.168.1.101"
JETSON_USER    = "jetson1"
JETSON_SSH     = f"{JETSON_USER}@{JETSON_HOST}"
JETSON_PASS    = "hello2020"          # default factory password
SSH_OPTS       = [
    "-o", "StrictHostKeyChecking=no",
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "LogLevel=ERROR",
]

BOOT_WAIT_S    = 30      # seconds after power-on before starting SSH poll
SSH_ATTEMPTS   = 10
SSH_RETRY_S    = 5

# ---------------------------------------------------------------------------
# Phase 1 — Power cycle
# ---------------------------------------------------------------------------

SHUTDOWN_WAIT_S  = 25    # seconds to wait for a graceful halt before giving up

def _graceful_shutdown_jetson() -> bool:
    """Halt the Jetson OS over SSH before its power is cut.
    """
    if not _ping(JETSON_HOST):
        return False  # already unreachable, nothing to shut down

    shutdown_cmd = "sync; echo " + JETSON_PASS + " | sudo -S shutdown -h now"
    r = subprocess.run(
        ["ssh", *SSH_OPTS, "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
         JETSON_SSH, shutdown_cmd],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        # Passwordless SSH not set up yet — fall back to password auth.
        r = subprocess.run(
            ["sshpass", f"-p{JETSON_PASS}", "ssh", *SSH_OPTS,
             "-o", "ConnectTimeout=5", JETSON_SSH, shutdown_cmd],
            capture_output=True, text=True,
        )
        if r.returncode != 0:
            return False

    for _ in range(SHUTDOWN_WAIT_S // 2):
        time.sleep(2)
        if not _ping(JETSON_HOST):
            return True
    return False


def power_cycle_jetson() -> None:
    """Power-cycle the Jetson via PowerPeriph aux-CPU control."""
    click.secho("\n[1/5] Power cycling Jetson via PowerPeriph…", fg="cyan", bold=True)

    import stretch4_body.subsystem.power_periph as pp_mod
    pp = pp_mod.PowerPeriph()
    if not pp.startup():
        click.secho("  ⚠  PowerPeriph startup failed — skipping power cycle.", fg="yellow")
        return

    click.secho("  Requesting graceful shutdown over SSH…", fg="cyan")
    if _graceful_shutdown_jetson():
        click.secho("  ✓ Jetson halted cleanly.", fg="green")
    else:
        click.secho("  ⚠  Could not confirm a clean halt — cutting power anyway.", fg="yellow")

    click.secho("  Powering OFF aux CPU…", fg="cyan")
    pp.set_aux_cpu_off()
    pp.push_command()
    time.sleep(5)

    click.secho("  Powering ON aux CPU…", fg="cyan")
    pp.set_aux_cpu_on()
    pp.push_command()
    pp.stop()

    click.secho(f"  Waiting {BOOT_WAIT_S}s for Jetson to boot…", fg="cyan")
    time.sleep(BOOT_WAIT_S)

    input(click.style(
        "  Confirm the Jetson fan is spinning and LEDs are active, then press Enter… ",
        fg="yellow",
    ))
    click.secho("  ✓ Power cycle complete.", fg="green")


# ---------------------------------------------------------------------------
# Phase 2 — Wait for SSH
# ---------------------------------------------------------------------------

def _ping(host: str) -> bool:
    r = subprocess.run(["ping", "-c", "1", "-W", "2", host],
                       capture_output=True)
    return r.returncode == 0


def _ssh_ok() -> bool:
    r = subprocess.run(
        ["ssh", *SSH_OPTS, "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
         JETSON_SSH, "echo ok"],
        capture_output=True, text=True,
    )
    return r.returncode == 0 and r.stdout.strip() == "ok"


def _ssh_service_up() -> bool:
    """Check that the SSH daemon is listening, regardless of auth outcome.
    """
    r = subprocess.run(
        ["ssh", *SSH_OPTS, "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
         JETSON_SSH, "echo ok"],
        capture_output=True, text=True,
    )
    if r.returncode == 0 and r.stdout.strip() == "ok":
        return True
    return "permission denied" in r.stderr.lower()


def wait_for_ssh() -> None:
    """Poll until the SSH service is available on the Jetson."""
    click.secho("\n[2/5] Waiting for Jetson SSH…", fg="cyan", bold=True)

    for attempt in range(1, SSH_ATTEMPTS + 1):
        click.echo(f"  Attempt {attempt}/{SSH_ATTEMPTS}: ping {JETSON_HOST}… ", nl=False)
        if _ping(JETSON_HOST):
            click.secho("reachable", fg="green")
            click.echo(f"  Attempt {attempt}/{SSH_ATTEMPTS}: SSH… ", nl=False)
            if _ssh_service_up():
                click.secho("up", fg="green")
                click.secho("  ✓ Jetson is up and accepting SSH connections.", fg="green")
                return
            else:
                click.secho("not ready yet", fg="yellow")
        else:
            click.secho("not reachable", fg="yellow")
        time.sleep(SSH_RETRY_S)

    click.secho("  ✗ Could not connect to Jetson after all attempts.", fg="red")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Phase 3 — Passwordless SSH
# ---------------------------------------------------------------------------

def _run_ssh(cmd: str, use_pass: bool = False) -> subprocess.CompletedProcess:
    """Run a command on the Jetson over SSH."""
    if use_pass:
        prefix = ["sshpass", f"-p{JETSON_PASS}"]
    else:
        prefix = []
    return subprocess.run(
        [*prefix, "ssh", *SSH_OPTS, JETSON_SSH, cmd],
        capture_output=True, text=True,
    )


def setup_passwordless_ssh() -> None:
    """Install sshpass, generate a local key if needed, and copy it to Jetson."""
    click.secho("\n[3/5] Setting up passwordless SSH…", fg="cyan", bold=True)

    # Install sshpass if missing
    if subprocess.run(["which", "sshpass"], capture_output=True).returncode != 0:
        click.echo("  Installing sshpass… ")
        subprocess.run(["sudo", "apt-get", "install", "-y", "sshpass"],
                       check=True, capture_output=True)

    # Generate key if not present
    key_path = os.path.expanduser("~/.ssh/id_rsa")
    if not os.path.exists(key_path):
        click.echo("  Generating SSH key… ")
        subprocess.run(["ssh-keygen", "-t", "rsa", "-N", "", "-f", key_path],
                       check=True, capture_output=True)

    # Copy key to Jetson
    click.echo("  Copying public key to Jetson… ")
    r = subprocess.run(
        ["sshpass", f"-p{JETSON_PASS}",
         "ssh-copy-id", *SSH_OPTS, JETSON_SSH],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        click.secho(f"  ⚠  ssh-copy-id failed: {r.stderr.strip()}", fg="yellow")

    # Verify
    if _ssh_ok():
        click.secho("  ✓ Passwordless SSH working.", fg="green")
    else:
        click.secho("  ⚠  SSH still requires password — check key/network.", fg="yellow")


# ---------------------------------------------------------------------------
# Phase 4 — Wi-Fi setup
# ---------------------------------------------------------------------------

WIFI_SSID        = None   # Will prompt if not set
WIFI_DRIVER_REPO = "https://github.com/lwfinger/rtl8188eu.git"
WIFI_DRIVER_DIR  = "/tmp/rtl8188eu"


def _jetson_run(cmd: str) -> tuple[int, str, str]:
    r = subprocess.run(
        ["ssh", *SSH_OPTS, JETSON_SSH, cmd],
        capture_output=True, text=True,
    )
    return r.returncode, r.stdout.strip(), r.stderr.strip()


def setup_wifi() -> None:
    """Configure Wi-Fi on the Jetson, installing RTL8188EUS driver if needed."""
    click.secho("\n[4/5] Configuring Wi-Fi on Jetson…", fg="cyan", bold=True)

    # Check if already connected
    rc, out, _ = _jetson_run("nmcli -t -f STATE general")
    if rc == 0 and "connected" in out:
        click.secho("  ✓ Jetson already connected to Wi-Fi.", fg="green")
        return

    # Detect Wi-Fi interface
    rc, out, _ = _jetson_run(
        "iw dev 2>/dev/null | awk '/Interface/{print $2}' | head -1"
    )
    wifi_iface = out.strip() if rc == 0 and out.strip() else None

    if not wifi_iface:
        click.secho("  No Wi-Fi interface found — checking for RTL8188EUS driver…", fg="yellow")
        _install_rtl8188eus_driver()
        # Re-detect
        rc, out, _ = _jetson_run(
            "iw dev 2>/dev/null | awk '/Interface/{print $2}' | head -1"
        )
        wifi_iface = out.strip() if rc == 0 and out.strip() else None
        if not wifi_iface:
            click.secho("  ✗ No Wi-Fi interface after driver install. Skipping Wi-Fi.", fg="red")
            return

    click.secho(f"  Wi-Fi interface: {wifi_iface}", fg="cyan")

    # Get SSID and password
    ssid     = click.prompt("  Enter Wi-Fi SSID")
    password = click.prompt("  Enter Wi-Fi password", hide_input=True)

    # Connect via nmcli
    click.echo(f"  Connecting to '{ssid}'… ")
    rc, out, err = _jetson_run(
        f"nmcli dev wifi connect '{ssid}' password '{password}' ifname '{wifi_iface}'"
    )
    if rc != 0:
        click.secho(f"  ✗ nmcli failed: {err or out}", fg="red")
        return

    # Verify internet
    rc, _, _ = _jetson_run("ping -c 2 8.8.8.8")
    if rc == 0:
        click.secho(f"  ✓ Jetson connected to '{ssid}' with internet access.", fg="green")
    else:
        click.secho(f"  ⚠  Connected to '{ssid}' but no internet (ping 8.8.8.8 failed).", fg="yellow")


def _install_rtl8188eus_driver() -> None:
    """Build and install the RTL8188EUS Wi-Fi driver on the Jetson via NUC NAT routing."""
    click.secho("  Installing RTL8188EUS driver on Jetson…", fg="cyan")

    cmds = [
        f"git clone --depth 1 {WIFI_DRIVER_REPO} {WIFI_DRIVER_DIR} 2>/dev/null || true",
        f"cd {WIFI_DRIVER_DIR} && make -j$(nproc) && sudo make install",
        "sudo modprobe 8188eu 2>/dev/null || true",
    ]
    for cmd in cmds:
        rc, out, err = _jetson_run(cmd)
        if rc != 0:
            click.secho(f"  ⚠  Command failed: {cmd}\n     {err or out}", fg="yellow")


# ---------------------------------------------------------------------------
# Phase 5 — Interactive terminal
# ---------------------------------------------------------------------------

def open_terminal() -> None:
    """Replace this process with an interactive SSH session to the Jetson."""
    click.secho("\n[5/5] Opening interactive terminal…", fg="cyan", bold=True)
    click.secho(f"  Connecting to {JETSON_SSH}\n", fg="cyan")
    os.execvp("ssh", ["ssh", *SSH_OPTS, JETSON_SSH])


# ---------------------------------------------------------------------------
# CLI entry-point
# ---------------------------------------------------------------------------

@click.command()
@click.option("--skip-power-cycle", is_flag=True,
              help="Skip power cycle (Jetson already on).")
@click.option("--skip-wifi",        is_flag=True,
              help="Skip Wi-Fi setup (already configured).")
def main(skip_power_cycle: bool, skip_wifi: bool) -> None:
    """
    Full Jetson bring-up: power cycle, SSH setup, Wi-Fi, and interactive terminal.
    """
    click.clear()
    click.secho("=========================================", fg="cyan", bold=True)
    click.secho("   Stretch Connect Jetson                ", fg="cyan", bold=True)
    click.secho("=========================================", fg="cyan", bold=True)

    if not skip_power_cycle:
        power_cycle_jetson()
    else:
        click.secho("\n[1/5] Skipping power cycle (--skip-power-cycle).", fg="yellow")

    wait_for_ssh()
    setup_passwordless_ssh()

    if not skip_wifi:
        setup_wifi()
    else:
        click.secho("\n[4/5] Skipping Wi-Fi setup (--skip-wifi).", fg="yellow")

    open_terminal()


if __name__ == "__main__":
    main()
