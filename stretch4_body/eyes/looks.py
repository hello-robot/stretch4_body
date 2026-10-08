"""
Eye sequences and the looks library.

A look is a Sequence of Steps saved as one JSON file (format 'stretch-eyes/1'); a
single look is a one-step sequence. Library finds them in the user dir, then any shared
dirs, then the built-ins shipped in stretch4_body/eyes/looks/.
"""
import errno
import json
import logging
import numbers
import os
import re
import tempfile
from dataclasses import dataclass, field

from stretch4_body.eyes.animations import find_animation
from stretch4_body.eyes.api import parse_intensity
from stretch4_body.eyes.colors import parse_color, to_hex

logger = logging.getLogger(__name__)

FORMAT = 'stretch-eyes/1'
NAME_PATTERN = re.compile(r'[a-z0-9][a-z0-9_-]{0,47}')  # use fullmatch: '$' also matches before a final newline
MIN_HOLD_S = 0.05
MAX_HOLD_S = 600.0
MAX_STEPS = 200
STEP_KEYS = ('left', 'right', 'color', 'intensity', 'hold')
SEQUENCE_KEYS = ('format', 'name', 'title', 'author', 'note', 'loop', 'steps')

BUILTIN_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'looks')
SHARED_ENV = 'STRETCH_EYES_LIBRARY'


def _extra_keys(d, known):
    """Split off 'x-' keys; any other key not in known is an error naming it."""
    extra = {}
    for key in d:
        if key in known:
            continue
        if isinstance(key, str) and key.startswith('x-'):
            extra[key] = d[key]
        else:
            raise ValueError('{}: unknown key. Valid keys: {} (or any key starting with x-)'.format(
                key, ', '.join(known)))
    return extra


def _check_extra(extra):
    """extra holds only 'x-' keys, so it can never override a core field in to_dict()."""
    for key in extra:
        if not (isinstance(key, str) and key.startswith('x-')):
            raise ValueError('extra: {!r} is not an x- key'.format(key))
    return extra


def _check(path, fn, value):
    try:
        return fn(value)
    except ValueError as e:
        raise ValueError('{}: {}'.format(path, e)) from None


def _hold(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError('must be a number of seconds, not {!r}'.format(value))
    if not MIN_HOLD_S <= value <= MAX_HOLD_S:
        raise ValueError('{} s is outside {}-{} s'.format(value, MIN_HOLD_S, MAX_HOLD_S))
    return float(value)


def _text(value):
    if not isinstance(value, str):
        raise ValueError('must be text, not {!r}'.format(value))
    return value


def _name(value):
    if not isinstance(value, str) or not NAME_PATTERN.fullmatch(value):
        raise ValueError('{!r} is not a look name: 1-48 of a-z, 0-9, _ and -, starting with a letter or digit'
                         .format(value))
    return value


@dataclass
class Step:
    """
    One step of a sequence: what to send, then how long to hold it. left, right, color
    and intensity take what Eyes.set takes; None keeps the previous step's value (for the
    first step, whatever the eyes were commanded last). hold is seconds, 0.05-600.
    Values are stored normalised: animation names, (r, g, b) and an int 0-255. extra
    keeps any 'x-' keys from the file.
    """
    left: str = None
    right: str = None
    color: tuple = None
    intensity: int = None
    hold: float = 1.0
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        if self.left is not None:
            self.left = _check('left', lambda v: find_animation(v).name, self.left)
        if self.right is not None:
            self.right = _check('right', lambda v: find_animation(v).name, self.right)
        if self.color is not None:
            self.color = _check('color', parse_color, self.color)
        if self.intensity is not None:
            self.intensity = _check('intensity', parse_intensity, self.intensity)
        self.hold = _check('hold', _hold, self.hold)
        _check_extra(self.extra)

    @classmethod
    def from_dict(cls, d):
        extra = _extra_keys(d, STEP_KEYS)
        if 'hold' not in d:
            raise ValueError('hold: missing; every step needs a hold in seconds')
        return cls(left=d.get('left'), right=d.get('right'), color=d.get('color'),
                   intensity=d.get('intensity'), hold=d['hold'], extra=extra)

    def to_dict(self):
        d = {}
        if self.left is not None:
            d['left'] = self.left
        if self.right is not None:
            d['right'] = self.right
        if self.color is not None:
            d['color'] = to_hex(self.color)
        if self.intensity is not None:
            d['intensity'] = self.intensity
        d['hold'] = self.hold
        d.update(self.extra)
        return d


class Sequence:
    """
    A named list of Steps, played in order by Eyes.play. loop is the default when played
    without an explicit loop. Invalid values raise ValueError naming the field, for
    example 'steps[2].hold'.
    """
    def __init__(self, name, steps, title=None, author='', note='', loop=False, extra=None):
        self.name = _check('name', _name, name)
        self.title = name if title is None else _check('title', _text, title)
        self.author = _check('author', _text, author)
        self.note = _check('note', _text, note)
        if not isinstance(loop, bool):
            raise ValueError('loop: must be true or false, not {!r}'.format(loop))
        self.loop = loop
        if isinstance(steps, (str, bytes, dict)) or not hasattr(steps, '__iter__'):
            raise ValueError('steps: must be a list of steps, not {!r}'.format(steps))
        self.steps = []
        for i, step in enumerate(steps):
            if isinstance(step, Step):
                self.steps.append(step)
                continue
            if not isinstance(step, dict):
                raise ValueError('steps[{}]: must be an object, not {!r}'.format(i, step))
            try:
                self.steps.append(Step.from_dict(step))
            except ValueError as e:
                raise ValueError('steps[{}].{}'.format(i, e)) from None
        if not 1 <= len(self.steps) <= MAX_STEPS:
            raise ValueError('steps: {} steps, a sequence has 1-{}'.format(len(self.steps), MAX_STEPS))
        self.extra = _check_extra(dict(extra or {}))

    def __repr__(self):
        return 'Sequence({!r}, {} steps, {:.2f} s{})'.format(
            self.name, len(self.steps), self.duration, ', loop' if self.loop else '')

    def __eq__(self, other):
        return isinstance(other, Sequence) and self.to_dict() == other.to_dict()

    @property
    def duration(self):
        """Seconds for one pass."""
        return sum(s.hold for s in self.steps)

    @classmethod
    def from_dict(cls, d):
        if not isinstance(d, dict):
            raise ValueError('A look must be a JSON object, not {!r}'.format(d))
        extra = _extra_keys(d, SEQUENCE_KEYS)
        if d.get('format') != FORMAT:
            raise ValueError('format: {!r} is not {!r}'.format(d.get('format'), FORMAT))
        for key in ('name', 'steps'):
            if key not in d:
                raise ValueError('{}: missing'.format(key))
        return cls(d['name'], d['steps'], title=d.get('title'), author=d.get('author', ''),
                   note=d.get('note', ''), loop=d.get('loop', False), extra=extra)

    def to_dict(self):
        d = {'format': FORMAT, 'name': self.name, 'title': self.title, 'author': self.author,
             'note': self.note, 'loop': self.loop, 'steps': [s.to_dict() for s in self.steps]}
        d.update(self.extra)
        return d

    def to_json(self):
        """The canonical file text: what save() writes."""
        return json.dumps(self.to_dict(), indent=2) + '\n'

    @classmethod
    def load(cls, path):
        """Read a look file. Raises ValueError naming the file and the field."""
        try:
            with open(path) as f:
                d = json.load(f)
        except json.JSONDecodeError as e:
            raise ValueError('{}: not JSON ({})'.format(path, e)) from None
        try:
            return cls.from_dict(d)
        except ValueError as e:
            raise ValueError('{}: {}'.format(path, e)) from None

    def save(self, path, exclusive=False):
        """
        Write the canonical file, replacing path in one step. exclusive raises
        FileExistsError instead when path exists, checked in that same step.
        """
        # A dot name ending in .tmp stays out of the library's *.json scan
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or '.', prefix='.' + os.path.basename(path), suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                os.fchmod(f.fileno(), 0o644)  # mkstemp makes it 0600
                f.write(self.to_json())
                f.flush()
                os.fsync(f.fileno())  # a new look survives a battery pull
            if exclusive:
                os.link(tmp, path)
            else:
                os.replace(tmp, path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        if exclusive:
            os.unlink(tmp)
        return path


def user_dir():
    """$HELLO_FLEET_PATH/$HELLO_FLEET_ID/eyes when both are set, else ~/stretch_user/eyes."""
    fleet_path, fleet_id = os.environ.get('HELLO_FLEET_PATH'), os.environ.get('HELLO_FLEET_ID')
    if fleet_path and fleet_id:
        return os.path.join(fleet_path, fleet_id, 'eyes')
    return os.path.join(os.path.expanduser('~'), 'stretch_user', 'eyes')


def shared_dirs():
    """The dirs in $STRETCH_EYES_LIBRARY, os.pathsep separated."""
    return [p for p in os.environ.get(SHARED_ENV, '').split(os.pathsep) if p]


class Library:
    """
    Looks found by name, first match wins: the user dir (user_dir()), then the shared
    dirs ($STRETCH_EYES_LIBRARY, or paths= when given), then the built-ins. Only the user
    dir is written. A file is read as <name>.json and must hold that name; files that do
    not validate are skipped and reported by errors(). The dirs are read on every call,
    so files added by hand or by a sync show up at once.
    """
    def __init__(self, paths=None):
        self.user_dir = user_dir()
        self.shared_dirs = shared_dirs() if paths is None else [str(p) for p in paths]
        self.builtin_dir = BUILTIN_DIR

    def dirs(self):
        """(source, dir) in search order."""
        return ([('user', self.user_dir)] + [('shared', d) for d in self.shared_dirs]
                + [('builtin', self.builtin_dir)])

    def _scan(self):
        """Every valid look as (source, path, Sequence) in search order, and the errors."""
        found, errors = [], []
        for source, d in self.dirs():
            try:
                names = sorted(n for n in os.listdir(d) if n.endswith('.json'))
            except OSError:
                continue  # a dir that does not exist yet holds no looks
            for n in names:
                path = os.path.join(d, n)
                try:
                    seq = Sequence.load(path)
                    if seq.name != n[:-len('.json')]:
                        raise ValueError('{}: name {!r} does not match the file name'.format(path, seq.name))
                except (OSError, ValueError) as e:
                    errors.append({'path': path, 'source': source, 'error': str(e)})
                    continue
                found.append((source, path, seq))
        return found, errors

    def list(self):
        """
        One entry per name, the first in search order: {name, title, source, path,
        steps (count), duration, loop, shadowed}. shadowed is True when a lower-priority
        look with the same name exists (a user copy of a built-in, say).
        """
        return self.scan_all()[0]

    def scan_all(self):
        """list(), errors() and {name: Sequence} for each listed look, from one read of the dirs."""
        found, errors = self._scan()
        entries, seqs = {}, {}
        for source, path, seq in found:
            if seq.name in entries:
                entries[seq.name]['shadowed'] = True
                continue
            entries[seq.name] = {'name': seq.name, 'title': seq.title, 'source': source, 'path': path,
                                 'steps': len(seq.steps), 'duration': seq.duration, 'loop': seq.loop,
                                 'shadowed': False}
            seqs[seq.name] = seq
        return list(entries.values()), errors, seqs

    def errors(self):
        """Look files that were skipped: [{path, source, error}]."""
        return self._scan()[1]

    def _find(self, name):
        hits = [(source, path, seq) for source, path, seq in self._scan()[0] if seq.name == name]
        if not hits:
            raise KeyError('No look named {!r}. Looks: {}'.format(
                name, ', '.join(e['name'] for e in self.list()) or 'none'))
        return hits

    def get(self, name):
        return self._find(name)[0][2]

    def export(self, name):
        return self.get(name).to_dict()

    def save(self, seq, overwrite=False):
        """Save to the user dir and return the path. seq is a Sequence or a dict."""
        if not isinstance(seq, Sequence):
            seq = Sequence.from_dict(seq)
        path = os.path.join(self.user_dir, seq.name + '.json')
        os.makedirs(self.user_dir, exist_ok=True)
        try:
            return seq.save(path, exclusive=not overwrite)
        except FileExistsError:
            raise FileExistsError(errno.EEXIST, 'Look {!r} already exists; pass overwrite=True to replace it'
                                  .format(seq.name), path) from None

    def delete(self, name):
        """Delete a user look; a shared or built-in look of that name then shows again."""
        source, path, _ = self._find(name)[0]
        if source != 'user':
            raise PermissionError('{} is a {} look and is read-only here: {}'.format(name, source, path))
        os.remove(path)

    def import_file(self, path, overwrite=False):
        """Validate a look file and save it to the user dir under its own name."""
        seq = Sequence.load(path)
        self.save(seq, overwrite=overwrite)
        return seq
