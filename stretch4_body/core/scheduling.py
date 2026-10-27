"""Core placement and scheduling class for the body server's loops.

The Stretch 4 NUC (Core Ultra) has P-cores, E-cores and two low-power E-cores on the SoC tile
that run at about a third of a P-core's capacity. The kernel parks lightly loaded periodic
threads on the slow pair whenever it can, and under load it evicts the control loop there,
which is where the worst control frames came from on stretch-se4-4056 (Stretch Lab, Stretch
4.5 wiki, "NUC cores", 2026-10-15). This module turns a small policy dict from robot params
into sched_setaffinity and sched_setscheduler calls. It never fails the caller: anything the
kernel or the user's limits refuse is logged and skipped, and the result says what was applied.

    policy = {'cpus': 'p+e', 'rt_priority': 0}

    cpus         'all' leaves the affinity alone; 'p' is the P-cores; 'p+e' is every core except
                 the low-power pair; a list of cpu numbers is used as given.
    rt_priority  0 keeps SCHED_OTHER; 1 to 99 asks for SCHED_FIFO at that priority. That needs an
                 rtprio limit for the user (RTPRIO_LIMITS_HINT below), otherwise it is skipped.

On a machine without the hybrid sysfs nodes every class resolves to all cores, so the policy is
a no-op for the affinity and only the priority request remains.
"""
import logging
import os
import resource

SYSFS_DEVICES = '/sys/devices'
TRANSPORT_RT_ENV = 'HELLO_TRANSPORT_RT_PRIORITY'

RTPRIO_LIMITS_HINT = (
    "rt_priority requested but the rtprio limit for this user is 0; SCHED_FIFO skipped. "
    "To allow it: echo 'hello-robot - rtprio 50' | sudo tee /etc/security/limits.d/90-hello-robot-rt.conf, "
    "then log out and in (user systemd units pick it up on the next login)."
)


def _parse_cpu_list(text):
    """'0-3,8,10-11' -> [0, 1, 2, 3, 8, 10, 11]"""
    cpus = []
    for part in text.replace('\n', '').split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            a, b = part.split('-', 1)
            cpus.extend(range(int(a), int(b) + 1))
        else:
            cpus.append(int(part))
    return cpus


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def cpu_classes(sysfs=SYSFS_DEVICES):
    """Which cpus are P-cores, E-cores and low-power E-cores on this machine.

    Returns {'all': [...], 'p': [...], 'e': [...], 'lp': [...]}. Reads the hybrid topology the
    kernel exposes (cpu_core/cpus, cpu_atom/cpus) and splits the atom cores by cpu_capacity:
    the lowest tier is the low-power pair. Without those nodes every class is all cores.
    """
    try:
        all_cpus = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        all_cpus = list(range(os.cpu_count() or 1))
    cores = _read(os.path.join(sysfs, 'cpu_core', 'cpus'))
    atoms = _read(os.path.join(sysfs, 'cpu_atom', 'cpus'))
    if cores is None or atoms is None:
        return {'all': all_cpus, 'p': list(all_cpus), 'e': list(all_cpus), 'lp': []}
    p = _parse_cpu_list(cores)
    atom = _parse_cpu_list(atoms)
    caps = {}
    for c in atom:
        v = _read(os.path.join(sysfs, 'system', 'cpu', f'cpu{c}', 'cpu_capacity'))
        if v is not None and v.isdigit():
            caps[c] = int(v)
    lp = []
    if caps and len(set(caps.values())) > 1:
        low = min(caps.values())
        lp = sorted(c for c, v in caps.items() if v == low)
    e = sorted(c for c in atom if c not in lp)
    return {'all': all_cpus, 'p': sorted(p), 'e': e, 'lp': lp}


def resolve_cpus(spec, classes=None):
    """Policy 'cpus' value -> set of cpu numbers, or None for 'all' / unchanged."""
    if spec is None or spec == 'all':
        return None
    classes = classes or cpu_classes()
    if isinstance(spec, (list, tuple, set)):
        cpus = set(int(c) for c in spec) & set(classes['all'])
    elif spec == 'p':
        cpus = set(classes['p'])
    elif spec == 'p+e':
        cpus = set(classes['p']) | set(classes['e'])
    else:
        raise ValueError(f"scheduling cpus must be 'all', 'p', 'p+e' or a list of cpu numbers, not {spec!r}")
    # never hand back an empty mask: that would be an error from the kernel, and no placement
    # is better than a crash at startup
    return cpus or None


def rtprio_limit():
    try:
        return resource.getrlimit(resource.RLIMIT_RTPRIO)[0]
    except (AttributeError, ValueError, OSError):
        return 0


def apply(policy, pid=0, logger=None, label='', classes=None):
    """Apply a policy to a process (pid, 0 = this process; for a child, its main thread).

    Returns {'cpus': sorted list or None, 'policy': 'fifo' | 'other', 'rt_priority': int,
    'skipped': [reasons]} describing what is now in force, for the status stream and the log.
    """
    logger = logger or logging.getLogger('scheduling')
    policy = policy or {}
    result = {'cpus': None, 'policy': 'other', 'rt_priority': 0, 'skipped': []}
    tag = f'{label}: ' if label else ''

    cpus = resolve_cpus(policy.get('cpus'), classes)
    if cpus is not None:
        try:
            os.sched_setaffinity(pid, cpus)
            result['cpus'] = sorted(cpus)
        except (OSError, AttributeError) as e:
            result['skipped'].append(f'affinity {sorted(cpus)}: {e}')
            logger.warning(f'{tag}could not set cpu affinity {sorted(cpus)}: {e}')

    rt = int(policy.get('rt_priority', 0) or 0)
    if rt > 0:
        rt = max(1, min(99, rt))
        if rtprio_limit() < rt:
            result['skipped'].append('rt_priority: rtprio limit too low')
            logger.info(f'{tag}{RTPRIO_LIMITS_HINT}')
        else:
            try:
                os.sched_setscheduler(pid, os.SCHED_FIFO, os.sched_param(rt))
                result['policy'] = 'fifo'
                result['rt_priority'] = rt
            except (OSError, AttributeError) as e:
                result['skipped'].append(f'rt_priority {rt}: {e}')
                logger.warning(f'{tag}could not set SCHED_FIFO {rt}: {e}')

    if result['cpus'] is not None or result['policy'] == 'fifo':
        logger.info(f"{tag}scheduling: cpus {result['cpus'] or 'all'}, {result['policy']}"
                    f"{' ' + str(result['rt_priority']) if result['policy'] == 'fifo' else ''}")
    return result


def export_transport_priority(policy):
    """Tell the C transport (transport.cpp) which SCHED_FIFO priority its queue threads get.

    The threads are created inside the C library when a device starts, so the request travels
    through the environment. 0 or an unusable rtprio limit means the threads stay SCHED_OTHER.
    """
    rt = int((policy or {}).get('rt_priority', 0) or 0)
    if rt > 0 and rtprio_limit() >= rt:
        os.environ[TRANSPORT_RT_ENV] = str(rt)
    else:
        os.environ.pop(TRANSPORT_RT_ENV, None)
    return rt if TRANSPORT_RT_ENV in os.environ else 0
