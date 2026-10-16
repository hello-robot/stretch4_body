"""
The SentryStatusLogger file format: what the logger leaves out of each sample and how a
reader puts it back.

A status_*.json file is a JSON array of RobotClient status samples, about 100 a second.
Two parts of a sample repeat themselves:

- status['pimu'] and status['base'] are legacy aliases of status['power_periph'] and
  status['omnibase'] (the same dicts), so they were written twice.
- The P14 power keys in CARRIED change at the 2 Hz diagnostics rate or slower; the event
  log changes only when its cursor moves. They make up most of a P14 sample.

So, in the logged copy only (the published status is never changed):

- the aliases are dropped;
- a CARRIED key is written only when its value differs from the last one written to the
  same file. A sample that leaves keys out lists them, e.g.
  '_carried': {'power_periph': ['power_event_log', 'power_health']};
- the first sample of every file is complete, so each file reads on its own.

StatusLogExpander.expand() undoes this: it fills each listed key with the last value
written before it in the same file and adds the aliases back. Files written before the
trim have neither the marker nor missing keys and come back unchanged.
"""
import copy

FORMAT_KEY = '_carried'

# subsystem -> keys written only when they change
CARRIED = {
    'power_periph': ('power_event_log', 'power_diagnostics', 'power_diagnostics_meta',
                     'power_health'),
}

# alias -> the status key it is the same dict as (RobotClient "legacy naming")
ALIASES = {'pimu': 'power_periph', 'base': 'omnibase'}

_MISSING = object()


class StatusLogTrimmer:
    """Logger side. One instance per writer; reset() at the start of every file."""

    def __init__(self):
        self._last = {}

    def reset(self):
        self._last = {}

    def trim(self, status):
        """
        The copy of status to log. Shallow: untouched subsystems are the caller's dicts,
        so encode the result before the next pull_status changes them.
        """
        out = {k: v for k, v in status.items()
               if not (k in ALIASES and v is status.get(ALIASES[k]))}
        carried = {}
        for sub, keys in CARRIED.items():
            src = out.get(sub)
            if not isinstance(src, dict):
                continue
            kept = None
            for key in keys:
                if key not in src:
                    continue
                value = src[key]
                if self._last.get((sub, key), _MISSING) == value:
                    if kept is None:
                        kept = dict(src)
                    del kept[key]
                    carried.setdefault(sub, []).append(key)
                else:
                    # Copied: the next pull_status may update this dict in place.
                    self._last[(sub, key)] = copy.deepcopy(value)
            if kept is not None:
                out[sub] = kept
        if carried:
            out[FORMAT_KEY] = carried
        return out


class StatusLogExpander:
    """
    Reader side. One instance per file, fed the samples in file order. Filled-in values
    are shared between samples; treat them as read-only.
    """

    def __init__(self):
        self._last = {}
        self.unresolved = 0   # listed keys with no earlier value (file lost its start)

    def expand(self, sample):
        if not isinstance(sample, dict):
            return sample
        carried = sample.pop(FORMAT_KEY, None) or {}
        for sub, keys in CARRIED.items():
            src = sample.get(sub)
            if isinstance(src, dict):
                for key in keys:
                    if key in src:
                        self._last[(sub, key)] = src[key]
        for sub, keys in carried.items():
            src = sample.get(sub)
            if not isinstance(src, dict):
                continue
            for key in keys:
                value = self._last.get((sub, key), _MISSING)
                if value is _MISSING:
                    self.unresolved += 1
                else:
                    src[key] = value
        for alias, source in ALIASES.items():
            if alias not in sample and source in sample:
                sample[alias] = sample[source]
        return sample


def expand_samples(samples):
    """Yield the full samples of one logged file's elements, in order."""
    expander = StatusLogExpander()
    for sample in samples:
        yield expander.expand(sample)
