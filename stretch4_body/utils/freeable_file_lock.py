import pathlib
from stretch4_body.utils.file_access_utils import is_file_in_use, acquire_lock_if_available, release_lock, setup_shared_directory

class FreeableFileLock:
    """
    Extends file locking to add admin-level actions like
     - Killing zombie processing
     - Freeing up the file lock

    The typical use-case is to lock some resource among multiple
    programs. Only the first program will successfully be able to
    FreeableFileLock.acquire(); all others will get back False.
    An "admin" process can call FreeableFileLock.free() to kill
    the process holding the lock and force the lock to be freed.

    Programs do not need to release, it happens automatically
    when the process exits.
    """
    FILE_LOCK_DIR = '/tmp/stretch_lock_dir/'

    def __init__(self, name):
        self.pid_filepath = f"{FreeableFileLock.FILE_LOCK_DIR}{name}.txt"

    @property
    def is_locked(self):
        return is_file_in_use(self.pid_filepath)

    def release(self):
        release_lock(self.pid_filepath)

    def acquire(self):
        """
        Returns
        -------
        bool
            whether acquire succeeded
        """
        pid_file = pathlib.Path(self.pid_filepath)

        setup_shared_directory(pid_file.parent)
        
        return acquire_lock_if_available(self.pid_filepath, remove_if_exists_and_unused=True)

    def free(self):
        release_lock(self.pid_filepath)
        return True
