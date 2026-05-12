import unittest
from stretch4_body.core.hello_utils import CircularMultiprocessingQueue
import copy
from multiprocessing import   Process
import time

q = CircularMultiprocessingQueue(100)
def consumer_loop():
    while True:
        cfg = q.get_nowait()
        if cfg == 'exit':
            break

class TestMultiprocessCircularQueue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        q.put('exit')

    def test_overflow(self):
        for i in range(101):
            q.put('nop')
        self.assertEqual(q.qsize(),100, "qsize not 100")

    def test_consumer(self):
        ts=time.time()
        loop = Process(target=consumer_loop, args=())
        for i in range(10000):
            q.put(i)
        dt=time.time()-ts
        self.assertLess(dt,0.5,'Took too long blocking on q')


if __name__ == '__main__':
    unittest.main()