from collections import UserDict
from threading import RLock


class ThreadSafeDict(UserDict):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = RLock()

    def __len__(self):
        with self._lock:
            return super().__len__()

    def __setitem__(self, key, value):
        with self._lock:
            super().__setitem__(key, value)

    def __getitem__(self, key):
        with self._lock:
            return super().__getitem__(key)

    def __delitem__(self, key):
        with self._lock:
            super().__delitem__(key)

    def clear(self):
        with self._lock:
            super().clear()

    # --- Serialization Logic for PyZMQ/Pickle ---

    def __getstate__(self):
        """
        Called when pickling (sending via ZMQ).
        We MUST remove the lock, otherwise pickle throws a TypeError.
        """
        with self._lock:
            state = self.__dict__.copy()
            # Remove the lock (cannot be pickled)
            del state['_lock']
            # IMPORTANT: Create a snapshot copy of the data. 
            # If we don't copy, and the dict changes *during* the pickle 
            # process (after this method returns), you might get a RuntimeError.
            state['data'] = self.data.copy()
            return state

    def __setstate__(self, state):
        """
        Called when unpickling (receiving from ZMQ).
        Restore data and create a NEW lock for this new instance.
        """
        self.__dict__.update(state)
        # Give the receiver their own fresh lock
        self._lock = RLock()
