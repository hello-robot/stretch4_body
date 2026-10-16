"""
The SentryStatusLogger file format: what the logger leaves out of each sample and how a
reader puts it back.

A status_*.json file is a JSON array of RobotClient status samples, about 100 a second.
Three parts of a sample repeat themselves:

- status['pimu'] and status['base'] are legacy aliases of status['power_periph'] and
  status['omnibase'] (the same dicts), so they were written twice.
- RobotClient.pull_status gives every subsystem the top-level server, routines and
  safety_layer dicts (SHARED), so each was written once per subsystem as well.
- The P14 power keys in CARRIED change at the 2 Hz diagnostics rate or slower; the event
  log changes only when its cursor moves, the power-state transition list only on a
  transition. They make up most of a P14 sample.

So, in the logged copy only (the published status is never changed):

- the aliases are dropped;
- a subsystem whose server, routines and safety_layer all equal the top-level ones has
  them dropped and is listed under '_shared'. A copy that differs is written in place;
- a CARRIED key (a dotted path inside its subsystem) is written only when its value
  differs from the last one written to the same file. A sample that leaves keys out lists
  them, e.g. '_carried': {'power_periph': ['power_event_log',
  'status_aux_meta.transitions']};
- the first sample of every file has every CARRIED value, so each file reads on its own.

StatusLogExpander.expand() undoes this exactly: it fills each carried key with the last
value written before it in the same file, gives each '_shared' subsystem the top-level
dicts, and adds the aliases back. Files written before the trim have none of the markers
and come back unchanged.
"""
import copy

FORMAT_KEY = '_carried'
SHARED_KEY = '_shared'

# subsystem -> dotted paths inside it written only when they change
CARRIED = {
    'power_periph': ('power_event_log', 'power_diagnostics', 'power_diagnostics_meta',
                     'power_health', 'status_aux_meta.transitions'),
}

# top-level dicts RobotClient also puts into every subsystem's status
SHARED = ('server', 'routines', 'safety_layer')

# alias -> the status key it is the same dict as (RobotClient "legacy naming")
ALIASES = {'pimu': 'power_periph', 'base': 'omnibase'}

_MISSING = object()


def _split(path):
    return tuple(path.split('.'))


def _get(d, parts):
    for part in parts:
        if not isinstance(d, dict) or part not in d:
            return _MISSING
        d = d[part]
    return d


class StatusLogTrimmer:
    """Logger side. One instance per writer; reset() at the start of every file."""

    def __init__(self):
        self._last = {}

    def reset(self):
        self._last = {}

    def trim(self, status):
        """
        The copy of status to log. Shallow: untouched dicts are the caller's, so encode
        the result before the next pull_status changes them.
        """
        out = {k: v for k, v in status.items()
               if not (k in ALIASES and v is status.get(ALIASES[k]))}
        copied = set()   # paths in out already replaced by copies

        def writable(parts):
            """out's dict at parts (a path of dicts), copied on first write."""
            node = out
            for i in range(len(parts)):
                key = parts[:i + 1]
                if key not in copied:
                    node[parts[i]] = dict(node[parts[i]])
                    copied.add(key)
                node = node[parts[i]]
            return node

        carried = {}
        for sub, paths in CARRIED.items():
            for path in paths:
                parts = (sub,) + _split(path)
                value = _get(out, parts)
                if value is _MISSING:
                    continue
                if self._last.get(parts, _MISSING) == value:
                    del writable(parts[:-1])[parts[-1]]
                    carried.setdefault(sub, []).append(path)
                else:
                    # Copied: the next pull_status may update this dict in place.
                    self._last[parts] = copy.deepcopy(value)

        top = [k for k in SHARED if k in status]
        shared = []
        if top:
            for sub, value in out.items():
                if (sub in SHARED or sub.startswith('_') or not isinstance(value, dict)
                        or any(k not in value for k in top)):
                    continue
                if all(value[k] is status[k] or value[k] == status[k] for k in top):
                    node = writable((sub,))
                    for k in top:
                        del node[k]
                    shared.append(sub)
        if carried:
            out[FORMAT_KEY] = carried
        if shared:
            out[SHARED_KEY] = shared
        return out


class StatusLogExpander:
    """
    Reader side. One instance per file, fed the samples in file order. Filled-in values
    are shared between samples; treat them as read-only.
    """

    def __init__(self):
        self._last = {}
        self.unresolved = 0   # carried keys with no earlier value (file lost its start)

    def expand(self, sample):
        if not isinstance(sample, dict):
            return sample
        carried = sample.pop(FORMAT_KEY, None) or {}
        shared = sample.pop(SHARED_KEY, None) or []
        for sub, paths in CARRIED.items():
            for path in paths:
                parts = (sub,) + _split(path)
                value = _get(sample, parts)
                if value is not _MISSING:
                    self._last[parts] = value
        for sub, paths in carried.items():
            for path in paths:
                parts = (sub,) + _split(path)
                parent = _get(sample, parts[:-1])
                value = self._last.get(parts, _MISSING)
                if value is _MISSING or not isinstance(parent, dict):
                    self.unresolved += 1
                else:
                    parent[parts[-1]] = value
        for sub in shared:
            node = sample.get(sub)
            if isinstance(node, dict):
                for k in SHARED:
                    if k in sample:
                        node[k] = sample[k]
        for alias, source in ALIASES.items():
            if alias not in sample and source in sample:
                sample[alias] = sample[source]
        return sample


def expand_samples(samples):
    """Yield the full samples of one logged file's elements, in order."""
    expander = StatusLogExpander()
    for sample in samples:
        yield expander.expand(sample)
