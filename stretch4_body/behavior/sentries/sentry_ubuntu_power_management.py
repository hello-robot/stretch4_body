from multiprocessing import Event, Process
from typing import TypedDict

from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.core import hello_utils
from stretch4_body.core.hello_utils import *
import subprocess
import time
import logging

from stretch4_body.core.worker_loop import worker_loop
from stretch4_body.robot.robot import Robot


class SentryUbuntuPowerManagementStatus(TypedDict):
    last_check: float
    last_known_mode: str | None


class SentryUbuntuPowerManagement(Sentry):
    """Watches Ubuntu Power Management `powerprofilesctl get` and if it's not 'performance', sets it to performance"""

    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_ubuntu_power_management", robot=robot)
        self.status = SentryUbuntuPowerManagementStatus(
            last_check=time.time(), last_known_mode=None
        )
        self.valid_profiles = ['performance', 'balanced', 'power-saver']
        self.power_profile = self.params["power_profile"]
        if self.power_profile not in self.valid_profiles:
            self.logger.warn(f"Cannot find power profile {self.power_profile}. Setting to 'performance'")
            self.power_profile = 'performance'

        self.q_cmd = hello_utils.CircularMultiprocessingQueue(10)
        self.q_status = hello_utils.CircularMultiprocessingQueue(10)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(10)

        self.do_exit = Event()

        self.watcher_process = Process(target=self.main_loop, args=(), daemon=True)

    def main_loop(self):
        worker_loop(
            loop_name=self.name,
            rate_hz=1 / self.params["check_rate_seconds"],
            worker_instance=self,
            q_admin=self.q_admin,
            q_status=self.q_status,
            q_cmd=self.q_cmd,
            do_exit=self.do_exit,
            callback_step=watcher_subprocess_step,
            callback_pause=self.pause,
            callback_unpause=self.unpause,
            callback_exit=self.pause,
        )

    def pause(self, *args):
        if not self.do_exit.is_set():
            self.do_exit.set()

    def stop(self):
        self.do_exit.set()
        self.q_admin.put('exit')
        if self.watcher_process.is_alive():
            self.watcher_process.join()
            

        return super().stop()

    def unpause(self, *args):
        self.do_exit.clear()

        if not self.watcher_process.is_alive():
            self.watcher_process = Process(target=self.main_loop, args=(), daemon=True)
            self.watcher_process.start()

    def do_startup_check(self):
        output = get_ubuntu_power_mode()

        if output == self.power_profile:
            return 

        self.logger.warning(
            f"Changing Ubuntu Power Management Mode from {output} to {self.power_profile}"
        )
        if not set_power_profile(self.power_profile, logger=self.logger):
            raise RuntimeError(f"Failed to set Ubuntu Power Management Mode to {self.power_profile}, please do this manually before starting Stretch Body Server.")

    def startup(self):
        self.do_startup_check()
        self.unpause()
        return super().startup()

    def step(self):
        s = self.q_status.get_latest()
        if s is not None:
            self.status.update(s)


def watcher_subprocess_step(instance:SentryUbuntuPowerManagement, cmd_in, status_out: dict):
    """This runs in a subprocess to avoid bogging down SBS. Check Ubuntu Power Management `powerprofilesctl get` and if it's not 'performance', sets it to performance"""

    output = get_ubuntu_power_mode()

    if output != instance.power_profile:
        instance.logger.warning(
            f"Changing Ubuntu Power Management Mode from {output} to {self.power_profile}"
        )
        set_power_profile(instance.power_profile, logger=instance.logger)

    status_out.update(
        SentryUbuntuPowerManagementStatus(
            last_check=time.time(), last_known_mode=output
        )
    )


def get_ubuntu_power_mode():
    output = subprocess.check_output(["powerprofilesctl", "get"], text=True).strip()

    return output


def set_power_profile(power_profile, logger:logging.Logger):
    """Sets Ubuntu Power Management to `performance`.

    Note: this sentry calls `sudo`, so to avoid the interactive password
    prompt, `powerprofilesctl` should be configured in /etc/sudoers.d/hello_sudoers.
    Robots that have been set up via stretch4_install will have this configured.
    """
    command = ["sudo", "-n", "powerprofilesctl", "set", power_profile]

    try:
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        logger.info(f"Power profile successfully set to {power_profile}.")
        if result.stdout:
            logger.info(f"Output: {result.stdout.strip()}")
        return True

    except subprocess.CalledProcessError as e:
        # Check if the failure was specifically because sudo prompted for a password
        if "password is required" in e.stderr.lower() or e.returncode == 1:
            logger.error("""
[Error] Sudo password required to set performance power mode.
To allow this script to run in the background, please run the following command in your terminal to enable passwordless access for powerprofilesctl:
                          
    echo '%users ALL=(ALL) NOPASSWD: /usr/bin/powerprofilesctl' | sudo tee -a /etc/sudoers.d/hello_sudoers > /dev/null && sudo chmod 0440 /etc/sudoers.d/hello_sudoers
                          
""")
            return False

        logger.error("Command failed.")
        logger.error(f"Return code: {e.returncode}")
        if e.stderr:
            logger.error(f"Error output: {e.stderr.strip()}")
        return False

    except FileNotFoundError:
        logger.error("Error: 'powerprofilesctl' or 'sudo' not found on this system.")
        return False

    except PermissionError:
        logger.error("Permission denied. Try running the script with sufficient privileges.")
        return False

    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return False
