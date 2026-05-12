import os
import time
import tempfile
import multiprocessing
import pytest

from stretch4_body.utils import file_access_utils
from stretch4_body.core.hello_utils import acquire_transport_filelock, free_transport_filelock

def worker_acquire_and_hold(filepath, duration, ready_event):
    """Worker function to hold a lock for a specified duration to test multiprocess locking."""
    file_access_utils.acquire_lock(filepath)
    ready_event.set()
    time.sleep(duration)
    file_access_utils.release_lock(filepath)

def worker_acquire_transport_and_hold(device_name, duration, ready_event):
    """Worker function to hold a transport lock to test `acquire_transport_filelock`."""
    success = acquire_transport_filelock(device_name)
    if success:
        ready_event.set()
        time.sleep(duration)
        free_transport_filelock(device_name)
    else:
        # failed to acquire
        pass

@pytest.fixture
def filepath():
    with tempfile.TemporaryDirectory() as test_dir:
        yield os.path.join(test_dir, "test_lock.txt")

def test_acquire_and_release(filepath):
    # Basic single-process acquire and release
    assert not file_access_utils.is_locked(filepath)
    
    file_access_utils.acquire_lock(filepath)
    
    file_access_utils.release_lock(filepath)
    assert not file_access_utils.is_locked(filepath)

def test_is_locked_multiprocess(filepath):
    # Spawn a process to hold the lock
    ready_event = multiprocessing.Event()
    p = multiprocessing.Process(target=worker_acquire_and_hold, args=(filepath, 2.0, ready_event))
    p.start()
    
    # Wait until the worker successfully acquires the lock
    assert ready_event.wait(timeout=3.0), "Worker failed to acquire lock in time"
    
    # Main process should detect that the file is locked
    assert file_access_utils.is_locked(filepath), "File should be locked by worker process"
    
    # Wait for the worker to finish and release the lock naturally
    p.join(timeout=3.0)
    
    # Verify it is no longer locked
    assert not file_access_utils.is_locked(filepath), "File should be unlocked after worker exits"

def test_transport_filelock():
    device_name = "test_testdevicexD"
    lock_path = file_access_utils.pathlib.Path(f'/tmp/stretch_pid_dir/stretch_body_transport_pid_{device_name}.txt')
    
    # Cleanup any existing lock for this mock device
    if lock_path.exists():
        try:
            lock_path.unlink()
        except OSError:
            pass

    # Spawn a process to hold the transport lock
    ready_event = multiprocessing.Event()
    p = multiprocessing.Process(target=worker_acquire_transport_and_hold, args=(device_name, 2.0, ready_event))
    p.start()
    
    assert ready_event.wait(timeout=3.0), "Worker failed to acquire transport lock"
    
    # Verify it is recognized as locked
    assert file_access_utils.is_file_in_use(str(lock_path))
    assert file_access_utils.is_locked(str(lock_path))
    
    # Main process trying to acquire the same transport lock should fail
    assert not acquire_transport_filelock(device_name), "Should fail to acquire an already held transport lock"
    
    p.join(timeout=3.0)
    
    # After worker finishes, main process should be able to acquire it
    assert acquire_transport_filelock(device_name), "Should be able to acquire transport lock after worker releases it"
    
    # And release it
    free_transport_filelock(device_name)
    
    # Ensure it's no longer in use
    assert not file_access_utils.is_file_in_use(str(lock_path))

def test_performance_is_locked(filepath):
    # Test performance of is_locked on an unlocked file
    iterations = 1000
    start_time = time.perf_counter()
    for _ in range(iterations):
        file_access_utils.is_locked(filepath)
    duration = time.perf_counter() - start_time
    
    print(f"\\nis_locked performance: {duration / iterations:.6f} sec/call (unlocked file)")
    assert (duration / iterations) < 0.005, "is_locked is too slow"

def test_performance_is_file_in_use(filepath):
    # Test performance of is_file_in_use on an unused file
    # Creating a file so it exists but is not locked
    with open(filepath, 'w') as f:
        f.write('test')
        
    iterations = 50
    start_time = time.perf_counter()
    for _ in range(iterations):
        file_access_utils.is_file_in_use(filepath)
    duration = time.perf_counter() - start_time
    
    print(f"\\nis_file_in_use performance: {duration / iterations:.6f} sec/call (unused file)")
    assert (duration / iterations) < 0.1, "is_file_in_use is too slow"

def test_performance_locked_file_in_use(filepath):
    # Test performance of is_file_in_use when file is locked
    # Spawn a process to hold the lock
    ready_event = multiprocessing.Event()
    p = multiprocessing.Process(target=worker_acquire_and_hold, args=(filepath, 1.0, ready_event))
    p.start()
    
    assert ready_event.wait(timeout=3.0), "Worker failed to acquire lock in time"
    
    iterations = 50
    start_time = time.perf_counter()
    for _ in range(iterations):
        # is_file_in_use checks is_locked first, so it should be fast if it's locked!
        file_access_utils.is_file_in_use(filepath)
    duration = time.perf_counter() - start_time
    
    p.join(timeout=3.0)
    
    print(f"\nis_file_in_use performance (locked file): {duration / iterations:.6f} sec/call")
    assert (duration / iterations) < 0.005, "is_file_in_use on a locked file is too slow"

def test_concurrent_acquire(filepath):
    # This test simulates multiple clients trying to acquire the same lock simultaneously,
    # specifically targeting the server singleton / multi-client bug.
    
    # Touch the file so it exists but is not locked (like an abandoned lock)
    with open(filepath, 'w') as f:
        f.write('abandoned')

    def concurrent_worker(path, results_queue):
        try:
            # Simulate a client or server attempting to acquire the lock
            success = file_access_utils.acquire_lock_if_available(path, remove_if_exists_and_unused=True)
            results_queue.put(("success", success))
            if success:
                time.sleep(2)
        except Exception as e:
            # Catch crashes like FileNotFoundError from os.remove or BlockingIOError from flock
            results_queue.put(("error", type(e).__name__))

    q = multiprocessing.Queue()
    processes = []
    
    # Start multiple processes at the same time to increase chance of race condition
    for _ in range(4):
        p = multiprocessing.Process(target=concurrent_worker, args=(filepath, q))
        processes.append(p)
        p.start()

    for p in processes:
        p.join()

    results = []
    while not q.empty():
        results.append(q.get())

    # Only one process should succeed, the others should cleanly return False without crashing.
    successes = [r for r in results if r[0] == "success" and r[1] == True]
    failures = [r for r in results if r[0] == "success" and r[1] == False]
    errors = [r for r in results if r[0] == "error"]

    print(f"\nConcurrent acquire results: {results}")

    # The bug is that some processes will crash with FileNotFoundError or BlockingIOError.
    # If the bug exists, this assert will fail. We expect 0 errors.
    assert len(errors) == 0, f"Race condition detected! Exceptions raised: {errors}"
    assert len(successes) == 1, f"Expected exactly 1 successful acquire, got {len(successes)}"
    assert len(failures) == 3, f"Expected exactly 3 clean failures, got {len(failures)}"

if __name__ == '__main__':
    pytest.main(['-s', __file__])