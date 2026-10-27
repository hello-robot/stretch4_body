#!/usr/bin/env python3
"""Tests for stretch4_body.core.scheduling: topology parsing from a fake sysfs, policy
resolution, and apply() on this process with no privilege (affinity applies, FIFO is skipped
cleanly when the rtprio limit is 0). Run: python3 -m pytest test/test_scheduling.py"""

import logging
import os
import tempfile
import unittest

from stretch4_body.core import scheduling


def fake_sysfs(root, p='0-3', atom='4-13', caps=None):
    os.makedirs(os.path.join(root, 'cpu_core'))
    os.makedirs(os.path.join(root, 'cpu_atom'))
    with open(os.path.join(root, 'cpu_core', 'cpus'), 'w') as f:
        f.write(p + '\n')
    with open(os.path.join(root, 'cpu_atom', 'cpus'), 'w') as f:
        f.write(atom + '\n')
    for c, cap in (caps or {}).items():
        d = os.path.join(root, 'system', 'cpu', f'cpu{c}')
        os.makedirs(d)
        with open(os.path.join(d, 'cpu_capacity'), 'w') as f:
            f.write(f'{cap}\n')


class TestTopology(unittest.TestCase):
    def test_hybrid_with_low_power_tier(self):
        with tempfile.TemporaryDirectory() as root:
            caps = {c: 790 for c in range(4, 12)}
            caps.update({12: 324, 13: 324})
            fake_sysfs(root, caps=caps)
            cl = scheduling.cpu_classes(sysfs=root)
        self.assertEqual(cl['p'], [0, 1, 2, 3])
        self.assertEqual(cl['e'], list(range(4, 12)))
        self.assertEqual(cl['lp'], [12, 13])

    def test_hybrid_without_capacity_nodes(self):
        with tempfile.TemporaryDirectory() as root:
            fake_sysfs(root)
            cl = scheduling.cpu_classes(sysfs=root)
        self.assertEqual(cl['p'], [0, 1, 2, 3])
        self.assertEqual(cl['e'], list(range(4, 14)))
        self.assertEqual(cl['lp'], [])

    def test_not_hybrid_resolves_to_all(self):
        with tempfile.TemporaryDirectory() as root:
            cl = scheduling.cpu_classes(sysfs=root)
        self.assertEqual(cl['p'], cl['all'])
        self.assertEqual(cl['lp'], [])

    def test_parse_cpu_list(self):
        self.assertEqual(scheduling._parse_cpu_list('0-3,8,10-11'), [0, 1, 2, 3, 8, 10, 11])
        self.assertEqual(scheduling._parse_cpu_list(''), [])


class TestResolve(unittest.TestCase):
    classes = {'all': list(range(14)), 'p': [0, 1, 2, 3], 'e': list(range(4, 12)), 'lp': [12, 13]}

    def test_specs(self):
        self.assertIsNone(scheduling.resolve_cpus('all', self.classes))
        self.assertIsNone(scheduling.resolve_cpus(None, self.classes))
        self.assertEqual(scheduling.resolve_cpus('p', self.classes), {0, 1, 2, 3})
        self.assertEqual(scheduling.resolve_cpus('p+e', self.classes), set(range(12)))
        self.assertEqual(scheduling.resolve_cpus([1, 2, 99], self.classes), {1, 2})

    def test_empty_mask_becomes_none(self):
        self.assertIsNone(scheduling.resolve_cpus([99], self.classes))

    def test_bad_spec(self):
        with self.assertRaises(ValueError):
            scheduling.resolve_cpus('fast', self.classes)


class TestApply(unittest.TestCase):
    def setUp(self):
        self.saved = os.sched_getaffinity(0)

    def tearDown(self):
        os.sched_setaffinity(0, self.saved)
        os.environ.pop(scheduling.TRANSPORT_RT_ENV, None)

    def test_affinity_applies_to_this_process(self):
        cpus = sorted(self.saved)
        if len(cpus) < 2:
            self.skipTest('needs two cpus')
        res = scheduling.apply({'cpus': cpus[:1]}, logger=logging.getLogger('t'))
        self.assertEqual(res['cpus'], cpus[:1])
        self.assertEqual(os.sched_getaffinity(0), set(cpus[:1]))
        self.assertEqual(res['policy'], 'other')

    def test_all_leaves_affinity_alone(self):
        res = scheduling.apply({'cpus': 'all'})
        self.assertIsNone(res['cpus'])
        self.assertEqual(os.sched_getaffinity(0), self.saved)

    def test_empty_policy_is_noop(self):
        res = scheduling.apply(None)
        self.assertEqual(res, {'cpus': None, 'policy': 'other', 'rt_priority': 0, 'skipped': []})

    def test_fifo_skipped_without_rtprio_limit(self):
        if scheduling.rtprio_limit() > 0:
            self.skipTest('this user has an rtprio limit; the skip path is not reachable')
        res = scheduling.apply({'rt_priority': 10})
        self.assertEqual(res['policy'], 'other')
        self.assertTrue(any('rtprio' in s for s in res['skipped']))
        self.assertEqual(scheduling.export_transport_priority({'rt_priority': 10}), 0)
        self.assertNotIn(scheduling.TRANSPORT_RT_ENV, os.environ)

    def test_transport_env_cleared_for_zero(self):
        os.environ[scheduling.TRANSPORT_RT_ENV] = '7'
        self.assertEqual(scheduling.export_transport_priority({'rt_priority': 0}), 0)
        self.assertNotIn(scheduling.TRANSPORT_RT_ENV, os.environ)


if __name__ == '__main__':
    unittest.main()
