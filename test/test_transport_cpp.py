import unittest
from stretch4_body.subsystem.power_periph import PowerPeriph
import time
import os
import psutil


def get_current_process_memory():
    process = psutil.Process(os.getpid())
    mem_info = process.memory_info()
    # Resident Set Size (RSS) is the non-swapped physical memory a process has used.
    # Virtual Memory Size (VMS) is the total virtual memory used by the process.
    return mem_info.rss, mem_info.vms


class TestTransportCpp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pp=PowerPeriph()
        cls.pp.startup()

    @classmethod
    def tearDownClass(cls):
        cls.pp.stop()

    def test_load_test_blocking(self):
        ts=time.time()
        for i in range(100):
            print('---- Transport load test blocking %d ---'%i)
            self.pp.push_load_test()
            self.assertTrue(self.pp.pull_load_test(),"pull_load_test fail")
        self.assertLess(time.time()-ts,2.0,"load test took too long")

    def test_spam_push_non_blocking(self):
        print('---test_spam_push non-blocking ---')
        for i in range(10000):
            #print('---- test_spam_push %d ---'%i)
            self.pp.push_load_test(blocking=False)

    def test_memory_leak(self):
        print('---test_memory_leak ---')
        rss1, vms1 = get_current_process_memory()
        print(f"Memory (1) RSS: {rss1 / (1024 * 1024):.2f} MB")
        print(f"Memory (1) VMS: {vms1 / (1024 * 1024):.2f} MB")
        for i in range(1000):
            self.pp.push_load_test()
            self.assertTrue(self.pp.pull_load_test(quiet=True),"pull_load_test fail")
        rss2, vms2 = get_current_process_memory()
        print(f"Memory (2) RSS: {rss2 / (1024 * 1024):.2f} MB")
        print(f"Memory (2) VMS: {vms2 / (1024 * 1024):.2f} MB")
        self.assertLess((rss2-rss1)/rss1,0.1,'memory leak rss2')
        self.assertLess((vms2 - vms1) / vms1, 0.1, 'memory leak vms2')
        for i in range(1000):
            self.pp.push_load_test()
            self.assertTrue(self.pp.pull_load_test(quiet=True),"pull_load_test fail")
        rss3, vms3 = get_current_process_memory()
        print(f"Memory (3) RSS: {rss3 / (1024 * 1024):.2f} MB")
        print(f"Memory (3) VMS: {vms3 / (1024 * 1024):.2f} MB")
        self.assertLess((rss3 - rss1) / rss1, 0.1, 'memory leak rss3')
        self.assertLess((vms3 - vms1) / vms1, 0.1, 'memory leak vms3')

if __name__ == '__main__':
    unittest.main()