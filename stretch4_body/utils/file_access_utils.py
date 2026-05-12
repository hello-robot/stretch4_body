import fcntl
import grp
import os
import pwd
import pathlib


def is_user_in_group(group_name: str) -> bool:
    try:
        group = grp.getgrnam(group_name)
    except KeyError:
        return False
    # Only check active kernel-level groups to ensure permissions match actual shell state
    return group.gr_gid in os.getgroups() or group.gr_gid == os.getgid()

def setup_shared_directory(directory: pathlib.Path):
    """Creates a shared directory that is 770 for the standard user group, making any file created inside it editable by any logged in user."""

    directory.mkdir(parents=True, exist_ok=True)

    try:
        users_gid = grp.getgrnam("users").gr_gid

        # Change the directory's group to the current user's group
        # (-1 means do not change the owner, just the group)
        os.chown(directory, -1, users_gid)

        # Apply the 2770 permissions (SetGID + Owner rwx + Group rwx)
        os.chmod(directory, 0o2770)

    except PermissionError:
        if directory.exists():
            # If the folder exists, assume it's got the right permissions.
            # There is a bug where if User 1 create the folder, User 2 will NOT be able to chown it.
            # This "hack" works around this.
            # print(f"Warning: Shared folder {directory} exists, but setting permissions failed.")
            return
        raise PermissionError(f"Could not create the shared folder {directory}.")
    except Exception:
        raise NotImplementedError(
            f"This is an unexpected state. Could not create the shared folder {directory}."
        )


def acquire_lock_if_available(file_path, *, remove_if_exists_and_unused: bool):
    try:
        return acquire_lock(filepath=file_path, blocking=False)
    except PermissionError:
        # We cannot open it because of permissions (maybe owned by other user and not 0o666).
        if remove_if_exists_and_unused:
            try:
                os.remove(file_path)
                return acquire_lock(filepath=file_path, blocking=False)
            except (OSError, PermissionError):
                return False
        return False


def is_file_in_use(file_path: str) -> bool:
    """
    Checks if an active process is bound to a file.
    Returns True if a process is using it, False if it is orphaned or missing.
    """
    if not os.path.exists(file_path):
        return False

    if is_locked(file_path):
        return True

    return False


def get_file_owner(file_path):
    return pathlib.Path(file_path).owner()


def is_file_owned_by_current_user(file_path) -> bool:
    whoami = pwd.getpwuid(os.getuid()).pw_name
    return get_file_owner(file_path) == whoami


_locks = {}


def acquire_lock(filepath, blocking=False):
    """
    Attempts to acquire an exclusive POSIX lock on a file.
    Returns True if successfull, or False if it is already locked (when blocking=False). 
    May raise PermissionError or OSError if the file is not accessible, which is not handled by this function.
    """
    try:
        # If the file exists, open it without O_CREAT to avoid Linux fs.protected_regular permission errors
        fd = os.open(filepath, os.O_RDWR)
    except FileNotFoundError:
        # The 0o666 mask allows other users to open it for locking later.
        fd = os.open(filepath, os.O_CREAT | os.O_RDWR, 0o666)

    # Force permissions just in case the system umask restricted O_CREAT
    try:
        os.chmod(filepath, 0o666)
    except PermissionError:
        pass

    flags = fcntl.LOCK_EX
    if not blocking:
        flags |= fcntl.LOCK_NB  # Add non-blocking flag

    try:
        fcntl.flock(fd, flags)
    except BlockingIOError as e:
        os.close(fd)
        return False

    global _locks
    _locks[filepath] = fd
    
    return True
    

def is_locked(filepath):
    """
    Checks if a file is currently locked by another process.
    """
    if not os.path.exists(filepath):
        return False

    try:
        # Use 'r+' instead of 'a+' to avoid O_CREAT, which triggers fs.protected_regular
        fd = open(filepath, "r+")

        try:
            # Try to acquire a non-blocking lock
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

            # If successful, it wasn't locked. We must unlock and close it immediately.
            fcntl.flock(fd, fcntl.LOCK_UN)
            fd.close()
            return False
        except (BlockingIOError, PermissionError):
            # We couldn't get the lock; someone else has it.
            fd.close()
            return True
    except OSError:
        return False


def release_lock(filepath):
    """
    Releases the lock and closes the file descriptor.
    """
    global _locks
    fd = _locks.get(filepath)
    if fd is not None:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)

    _locks.pop(filepath, None)
