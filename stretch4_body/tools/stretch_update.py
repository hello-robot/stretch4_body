#!/usr/bin/env python3
"""
stretch_update — A "Software Updater" style GUI for Stretch 4.

Shows:
  * Installed hello-robot-stretch4-* pip packages, with an Install button when
    PyPI has a newer release
  * Git repos in ~/repos, with a Git Pull button when behind upstream
  * Public Stretch 4 packages (PyPI) and repos (GitHub) from Hello Robot that
    aren't installed yet, with Install / Clone buttons
"""
import argparse
import json
import os
import re
import subprocess
import sys
import threading
import tkinter as tk
import tkinter.font as tkfont
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from importlib.metadata import distributions
from tkinter import ttk

HTTP_TIMEOUT_S = 8.0
GIT_TIMEOUT_S = 30.0
GITHUB_ORG = 'hello-robot'
PREFIX = 'stretch4'                     # only Stretch 4 repos / packages
PIP_PREFIX = 'hello-robot-' + PREFIX

# Never block on credential / host-key prompts; allow pip on Ubuntu's system python
ENV = dict(os.environ,
           GIT_TERMINAL_PROMPT='0',
           GIT_ASKPASS='',
           SSH_ASKPASS='',
           GIT_SSH_COMMAND='ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new',
           PIP_BREAK_SYSTEM_PACKAGES='1')


# ==============================================================================
# Update checks
# ==============================================================================

def _norm(name):
    return re.sub(r'[-_.]+', '-', name).lower()


def _is_newer(candidate, installed):
    try:
        from packaging.version import Version
        return Version(candidate) > Version(installed)
    except Exception:
        to_tuple = lambda v: tuple(int(n) for n in re.findall(r'\d+', v))
        return to_tuple(candidate) > to_tuple(installed)


def _get_json(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'stretch_update'})
    with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as resp:
        return json.load(resp)


def _pypi_info(name):
    """Return {'version', 'summary'} for a PyPI project, or None."""
    try:
        info = _get_json(f'https://pypi.org/pypi/{name}/json').get('info') or {}
        return {'version': info.get('version'), 'summary': info.get('summary') or ''}
    except Exception:
        return None


def _editable_path(dist):
    """Source folder of an editable (pip install -e) install, else None."""
    try:
        info = json.loads(dist.read_text('direct_url.json') or '{}')
        if info.get('dir_info', {}).get('editable') and info.get('url', '').startswith('file://'):
            return urllib.request.url2pathname(info['url'][len('file://'):])
    except Exception:
        pass
    return None


def installed_packages():
    """{normalized name: (display name, version, editable path or None)} of installed Stretch 4 packages."""
    out = {}
    for dist in distributions():
        name = (dist.metadata.get('Name') or '').strip()
        if _norm(name).startswith(PIP_PREFIX) and _norm(name) not in out:
            out[_norm(name)] = (name, (dist.metadata.get('Version') or '').strip(), _editable_path(dist))
    return out


def github_repos():
    """Public, non-archived Stretch 4 repos in the hello-robot org."""
    repos, page = [], 1
    while True:
        batch = _get_json(f'https://api.github.com/orgs/{GITHUB_ORG}/repos'
                          f'?type=public&per_page=100&page={page}')
        repos += batch
        if len(batch) < 100:
            break
        page += 1
    return sorted(({'name': r['name'], 'description': r.get('description') or '',
                    'url': r['clone_url']}
                   for r in repos if r['name'].lower().startswith(PREFIX) and not r.get('archived')),
                  key=lambda r: r['name'].lower())


def _git(repo, *cmd, timeout=GIT_TIMEOUT_S):
    """Run git in repo. Returns (ok, output)."""
    try:
        res = subprocess.run(['git', '-C', repo, *cmd], capture_output=True, text=True,
                             timeout=timeout, env=ENV, stdin=subprocess.DEVNULL)
        return res.returncode == 0, (res.stdout + res.stderr).strip()
    except Exception as e:
        return False, str(e)


def check_repo(repo):
    """Fetch repo and return a status dict."""
    r = {'name': os.path.basename(repo), 'path': repo, 'branch': '', 'behind': 0, 'status': 'ok', 'error': ''}
    ok, r['branch'] = _git(repo, 'rev-parse', '--abbrev-ref', 'HEAD')
    if not ok or r['branch'] == 'HEAD':
        r['status'] = 'detached'
        return r
    ok, _ = _git(repo, 'rev-parse', '--abbrev-ref', '@{u}')
    if not ok:
        r['status'] = 'no-upstream'
        return r
    ok, out = _git(repo, 'fetch', '--quiet')
    if not ok:
        r['status'] = 'error'
        r['error'] = out.splitlines()[-1] if out else 'fetch failed'
        return r
    ok, n = _git(repo, 'rev-list', '--count', 'HEAD..@{u}')
    r['behind'] = int(n) if ok and n.isdigit() else 0
    r['status'] = 'behind' if r['behind'] else 'ok'
    return r


def check_all(repos_dir):
    """
    Run every check concurrently. Returns a dict with:
      packages:  [{name, version, latest}]      installed Stretch 4 packages
      repos:     [check_repo dicts]             git repos in repos_dir
      avail_pip: [{name, version, summary}]     public packages not installed
      avail_git: [{name, description, url}]     public repos not cloned
      warnings:  [str]
    """
    warnings = []
    installed = installed_packages()
    local_dirs = sorted(os.listdir(repos_dir)) if os.path.isdir(repos_dir) else []
    repo_paths = [os.path.join(repos_dir, d) for d in local_dirs
                  if os.path.isdir(os.path.join(repos_dir, d, '.git'))]

    with ThreadPoolExecutor(max_workers=16) as pool:
        gh_future = pool.submit(github_repos)
        repo_futures = [pool.submit(check_repo, p) for p in repo_paths]
        try:
            gh = gh_future.result()
        except Exception as e:
            gh = []
            warnings.append(f"Couldn't reach GitHub — available repos not listed ({e}).")

        # Candidate PyPI packages: installed ones + hello-robot-<github repo name>
        candidates = set(installed) | {_norm('hello-robot-' + r['name']) for r in gh}
        names = sorted(candidates)
        infos = dict(zip(names, pool.map(_pypi_info, names)))
        repos = [f.result() for f in repo_futures]

    if installed and not any(infos.get(n) for n in installed):
        warnings.append("Couldn't reach PyPI — package updates not checked.")

    packages = []
    editable = {}   # real repo path -> package name
    for n, (display, ver, local) in sorted(installed.items()):
        info = infos.get(n)
        # Editable installs are updated with git pull, never from PyPI
        latest = None if local else (
            info['version'] if info and info['version'] and _is_newer(info['version'], ver) else None)
        packages.append({'name': display, 'version': ver, 'latest': latest, 'local': local})
        if local:
            editable[os.path.realpath(local)] = display
    for r in repos:
        r['pip_pkg'] = editable.get(os.path.realpath(r['path']))

    avail_pip = [{'name': n, 'version': infos[n]['version'], 'summary': infos[n]['summary']}
                 for n in names if n not in installed and infos.get(n)]
    local_lc = {d.lower() for d in local_dirs}
    avail_git = [r for r in gh if r['name'].lower() not in local_lc]

    errors = [r for r in repos if r['status'] == 'error']
    if errors:
        warnings.append(f"Couldn't fetch {len(errors)} repo(s): " + ', '.join(r['name'] for r in errors)
                        + '. They may need GitHub credentials.')
    return {'packages': packages, 'repos': repos, 'avail_pip': avail_pip,
            'avail_git': avail_git, 'warnings': warnings}


# ==============================================================================
# GUI
# ==============================================================================

C = {
    'bg':        '#f4f5f7',
    'card':      '#ffffff',
    'border':    '#e1e4e8',
    'text':      '#1f2328',
    'muted':     '#656d76',
    'accent':    '#e95420',   # Ubuntu orange
    'accent_hi': '#c7461a',
    'blue':      '#0969da',
    'blue_hi':   '#0550ae',
    'blue_bg':   '#ddf4ff',
    'green':     '#1a7f37',
    'green_bg':  '#dafbe1',
    'amber':     '#9a6700',
    'amber_bg':  '#fff8c5',
    'red':       '#cf222e',
    'red_hi':    '#a40e26',
    'red_bg':    '#ffebe9',
    'grey_bg':   '#eaeef2',
    'chip_py':   '#3776ab',
    'chip_git':  '#f05032',
    'log_bg':    '#24292f',
    'log_fg':    '#d0d7de',
}


def _font_family(root):
    families = set(tkfont.families(root))
    for f in ('Ubuntu', 'Cantarell', 'Noto Sans', 'DejaVu Sans'):
        if f in families:
            return f
    return 'TkDefaultFont'


def _plural(n, word):
    return f'{n} {word}{"" if n == 1 else "s"}'


class FlatButton(tk.Label):
    """A flat, colored button with hover effect."""

    def __init__(self, parent, text, command, bg, fg='white', hover=None, **kw):
        super().__init__(parent, text=text, bg=bg, fg=fg, cursor='hand2', padx=14, pady=6, **kw)
        self._bg, self._hover, self._command, self._enabled = bg, hover or bg, command, True
        self.bind('<Enter>', lambda e: self._enabled and self.configure(bg=self._hover))
        self.bind('<Leave>', lambda e: self._enabled and self.configure(bg=self._bg))
        self.bind('<Button-1>', lambda e: self._enabled and self._command and self._command())

    def set(self, text=None, bg=None, fg=None, hover=None, enabled=None):
        if text is not None:
            self.configure(text=text)
        if bg is not None:
            self._bg = self._hover = bg
            self.configure(bg=bg)
        if hover is not None:
            self._hover = hover
        if fg is not None:
            self.configure(fg=fg)
        if enabled is not None:
            self._enabled = enabled
            self.configure(cursor='hand2' if enabled else 'arrow')


class UpdaterApp:
    def __init__(self, root, repos_dir):
        self.root = root
        self.repos_dir = repos_dir
        self.actions = []    # pending (button, run) updates, for Update All
        self.busy = False
        fam = _font_family(root)
        self.f_title = (fam, 16, 'bold')
        self.f_sub = (fam, 10)
        self.f_name = (fam, 11, 'bold')
        self.f_body = (fam, 10)
        self.f_small = (fam, 9, 'bold')
        self.f_btn = (fam, 10, 'bold')

        root.title('Stretch Software Updater')
        root.geometry('800x720')
        root.minsize(640, 480)
        root.configure(bg=C['bg'])

        self._build_header()
        self._build_footer()
        self._build_details()
        self._build_content()
        self.check()

    # ---- layout ------------------------------------------------------------
    def _build_header(self):
        header = tk.Frame(self.root, bg=C['card'], padx=20, pady=16)
        header.pack(fill='x')
        self.header = header
        tk.Frame(self.root, bg=C['border'], height=1).pack(fill='x')

        self.icon = tk.Canvas(header, width=52, height=52, bg=C['card'], highlightthickness=0)
        self.icon.pack(side='left')
        text = tk.Frame(header, bg=C['card'])
        text.pack(side='left', fill='x', expand=True, padx=14)
        self.title = tk.Label(text, font=self.f_title, bg=C['card'], fg=C['text'], anchor='w')
        self.title.pack(fill='x')
        self.subtitle = tk.Label(text, font=self.f_sub, bg=C['card'], fg=C['muted'], anchor='w')
        self.subtitle.pack(fill='x')

        self.check_btn = FlatButton(header, 'Check Again', self.check, bg=C['bg'], fg=C['text'],
                                    hover=C['border'], font=self.f_btn)
        self.check_btn.pack(side='right')

        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('Thin.Horizontal.TProgressbar', thickness=3, background=C['accent'],
                        troughcolor=C['card'], bordercolor=C['card'], lightcolor=C['accent'], darkcolor=C['accent'])
        self.progress = ttk.Progressbar(self.root, mode='indeterminate', style='Thin.Horizontal.TProgressbar')

    def _build_content(self):
        wrap = tk.Frame(self.root, bg=C['bg'])
        wrap.pack(fill='both', expand=True)
        self.canvas = tk.Canvas(wrap, bg=C['bg'], highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient='vertical', command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=sb.set)
        sb.pack(side='right', fill='y')
        self.canvas.pack(side='left', fill='both', expand=True)
        self.content = tk.Frame(self.canvas, bg=C['bg'], padx=20, pady=12)
        win = self.canvas.create_window(0, 0, window=self.content, anchor='nw')
        self.content.bind('<Configure>', lambda e: self.canvas.configure(scrollregion=self.canvas.bbox('all')))
        self.canvas.bind('<Configure>', lambda e: self.canvas.itemconfigure(win, width=e.width))
        for seq, d in (('<Button-4>', -1), ('<Button-5>', 1)):
            self.root.bind_all(seq, lambda e, d=d: self.canvas.yview_scroll(d, 'units'))
        self.root.bind_all('<MouseWheel>', lambda e: self.canvas.yview_scroll(-1 if e.delta > 0 else 1, 'units'))

    def _build_details(self):
        self.details_open = False
        bar = tk.Frame(self.root, bg=C['bg'], padx=20)
        bar.pack(side='bottom', fill='x')
        self.details_toggle = tk.Label(bar, text='▸ Show details', font=self.f_body, bg=C['bg'],
                                       fg=C['blue'], cursor='hand2')
        self.details_toggle.pack(anchor='w', pady=(4, 4))
        self.details_toggle.bind('<Button-1>', lambda e: self._toggle_details())
        self.log = tk.Text(bar, height=8, state='disabled', wrap='word', bg=C['log_bg'], fg=C['log_fg'],
                           font=('DejaVu Sans Mono', 9), relief='flat', padx=10, pady=8)

    def _build_footer(self):
        tk.Frame(self.root, bg=C['border'], height=1).pack(side='bottom', fill='x')
        footer = tk.Frame(self.root, bg=C['card'], padx=20, pady=12)
        footer.pack(side='bottom', fill='x')
        FlatButton(footer, 'Close', self.root.destroy, bg=C['bg'], fg=C['text'],
                   hover=C['border'], font=self.f_btn).pack(side='right')
        self.update_all_btn = FlatButton(footer, 'Update All', self.update_all, bg=C['accent'],
                                         hover=C['accent_hi'], font=self.f_btn)
        self.update_all_btn.pack(side='right', padx=8)

    def _toggle_details(self):
        self.details_open = not self.details_open
        if self.details_open:
            self.log.pack(fill='x', pady=(0, 8))
            self.details_toggle.configure(text='▾ Hide details')
        else:
            self.log.pack_forget()
            self.details_toggle.configure(text='▸ Show details')

    # ---- header state ------------------------------------------------------
    def _set_header(self, state, title, subtitle=''):
        color, symbol = {'checking': (C['blue'], '↻'), 'updates': (C['accent'], '↑'),
                         'ok': (C['green'], '✓'), 'working': (C['blue'], '↓')}[state]
        self.icon.delete('all')
        self.icon.create_oval(2, 2, 50, 50, fill=color, outline='')
        self.icon.create_text(26, 26, text=symbol, fill='white', font=(self.f_title[0], 20, 'bold'))
        self.title.configure(text=title)
        self.subtitle.configure(text=subtitle)

    def _set_busy(self, busy):
        self.busy = busy
        self.check_btn.set(enabled=not busy, fg=C['muted'] if busy else C['text'])
        if busy:
            self.progress.pack(fill='x', after=self.header)
            self.progress.start(12)
        else:
            self.progress.stop()
            self.progress.pack_forget()
        self._refresh_update_all()

    def _refresh_update_all(self):
        n = len(self.actions)
        enabled = n > 0 and not self.busy
        self.update_all_btn.set(text=f'Update All ({n})' if n else 'Update All',
                                bg=C['accent'] if enabled else C['border'], hover=C['accent_hi'] if enabled else None,
                                fg='white' if enabled else C['muted'], enabled=enabled)

    def _log(self, msg):
        def do():
            self.log.configure(state='normal')
            self.log.insert('end', msg.rstrip() + '\n')
            self.log.see('end')
            self.log.configure(state='disabled')
        self.root.after(0, do)

    # ---- checking ----------------------------------------------------------
    def check(self):
        for w in self.content.winfo_children():
            w.destroy()
        self.actions = []
        self._set_header('checking', 'Checking for updates…', 'Contacting PyPI and GitHub')
        self._set_busy(True)
        threading.Thread(target=self._check_worker, daemon=True).start()

    def _check_worker(self):
        self._log(f'Checking PyPI, GitHub and git repos in {self.repos_dir}...')
        try:
            res = check_all(self.repos_dir)
        except Exception as e:
            res = {'packages': [], 'repos': [], 'avail_pip': [], 'avail_git': [],
                   'warnings': [f'Update check failed: {e}']}
        for r in res['repos']:
            if r['error']:
                self._log(f"{r['name']}: {r['error']}")
        for w in res['warnings']:
            self._log(w)
        self._log('Check complete.\n')
        self.root.after(0, self._show, res)

    def _show(self, res):
        for w in res['warnings']:
            self._banner(w)

        pip = res['packages']
        pip.sort(key=lambda p: (p['latest'] is None, p['name'].lower()))
        n_pip = sum(1 for p in pip if p['latest'])
        card = self._section('INSTALLED PACKAGES', len(pip), _plural(n_pip, 'update') if n_pip else None)
        if not pip:
            self._empty_row(card, 'No Stretch 4 packages installed.')
        for p in pip:
            if p['latest']:
                self._row(card, 'py', p['name'],
                          [(p['version'], C['muted']), ('  →  ', C['muted']), (p['latest'], C['green'])],
                          action=('Install', [sys.executable, '-m', 'pip', 'install', '-U', p['name']], None),
                          is_update=True)
            elif p['local']:
                where = p['local'].replace(os.path.expanduser('~'), '~', 1)
                self._row(card, 'py', p['name'], [(p['version'], C['muted'])],
                          desc=f'Installed from {where} — update it with Git Pull in Installed Repos.',
                          pill='editable')
            else:
                self._row(card, 'py', p['name'], [(p['version'], C['muted'])], pill='up')

        repos = res['repos']
        repos.sort(key=lambda r: (r['status'] != 'behind', r['name'].lower()))
        n_git = sum(1 for r in repos if r['status'] == 'behind')
        card = self._section('INSTALLED REPOS', len(repos), _plural(n_git, 'update') if n_git else None,
                             self.repos_dir)
        if not repos:
            self._empty_row(card, f'No git repos in {self.repos_dir}.')
        for r in repos:
            tag = 'Installed as a package' if r.get('pip_pkg') else None
            if r['status'] == 'behind':
                self._row(card, 'git', r['name'], [(r['branch'], C['muted'])], tag=tag,
                          badge=_plural(r['behind'], 'commit') + ' behind',
                          action=('Git Pull', ['git', 'pull', '--ff-only'], r['path']), is_update=True)
            else:
                pill = {'ok': 'up', 'error': 'error', 'no-upstream': 'local', 'detached': 'detached'}[r['status']]
                self._row(card, 'git', r['name'], [(r['branch'] if r['status'] != 'detached' else '', C['muted'])],
                          tag=tag, pill=pill)

        avail_pip, avail_git = res['avail_pip'], res['avail_git']
        if avail_pip or avail_git:
            tk.Label(self.content, text='Available from Hello Robot', font=self.f_title, bg=C['bg'],
                     fg=C['text'], anchor='w').pack(fill='x', pady=(18, 0))
            tk.Label(self.content, text='Public Stretch 4 packages and repos you don’t have yet.',
                     font=self.f_sub, bg=C['bg'], fg=C['muted'], anchor='w').pack(fill='x')
        if avail_pip:
            card = self._section('PYPI PACKAGES', len(avail_pip))
            for p in avail_pip:
                self._row(card, 'py', p['name'], [(p['version'], C['muted'])], desc=p['summary'],
                          action=('Install', [sys.executable, '-m', 'pip', 'install', p['name']], None))
        if avail_git:
            card = self._section('GITHUB REPOS', len(avail_git))
            for r in avail_git:
                dest = os.path.join(self.repos_dir, r['name'])
                self._row(card, 'git', r['name'], [], desc=r['description'],
                          action=('Clone', ['git', 'clone', r['url'], dest], None))

        n = n_pip + n_git
        self._set_busy(False)
        if n == 0:
            self._set_header('ok', 'Your software is up to date',
                             f'{_plural(len(pip), "package")} and {_plural(len(repos), "repo")} checked.')
        else:
            parts = [_plural(x, w) for x, w in ((n_pip, 'package'), (n_git, 'repo')) if x]
            self._set_header('updates', f'{_plural(n, "update")} available',
                             'Updates are available for ' + ' and '.join(parts) + '.')

    # ---- widgets -----------------------------------------------------------
    def _banner(self, text):
        f = tk.Frame(self.content, bg=C['amber_bg'], highlightthickness=1, highlightbackground='#e9d48a')
        f.pack(fill='x', pady=(0, 10))
        tk.Label(f, text='⚠  ' + text, bg=C['amber_bg'], fg=C['amber'], font=self.f_body,
                 anchor='w', justify='left', wraplength=680, padx=12, pady=8).pack(fill='x')

    def _section(self, text, count, note=None, path=None):
        f = tk.Frame(self.content, bg=C['bg'])
        f.pack(fill='x', pady=(12, 6))
        tk.Label(f, text=f'{text}  ·  {count}', font=self.f_small, bg=C['bg'], fg=C['muted']).pack(side='left')
        if note:
            tk.Label(f, text=note, font=self.f_small, bg=C['accent'], fg='white', padx=7).pack(side='left', padx=8)
        if path:
            path = path.replace(os.path.expanduser('~'), '~', 1)
            if len(path) > 45:
                path = '…' + path[-44:]
            tk.Label(f, text=path, font=self.f_body, bg=C['bg'], fg=C['muted']).pack(side='right')
        card = tk.Frame(self.content, bg=C['card'], highlightthickness=1, highlightbackground=C['border'])
        card.pack(fill='x')
        card.rows = 0
        return card

    def _empty_row(self, card, text):
        tk.Label(card, text=text, font=self.f_body, bg=C['card'], fg=C['muted'],
                 anchor='w', padx=14, pady=12).pack(fill='x')

    def _row(self, card, kind, name, detail, desc='', badge=None, tag=None, pill=None, action=None,
             is_update=False):
        if card.rows:
            tk.Frame(card, bg=C['border'], height=1).pack(fill='x', padx=12)
        card.rows += 1
        row = tk.Frame(card, bg=C['card'], padx=12, pady=9)
        row.pack(fill='x')

        tk.Label(row, text='Py' if kind == 'py' else 'git', width=4, pady=5, fg='white',
                 bg=C['chip_py'] if kind == 'py' else C['chip_git'], font=self.f_small).pack(side='left', anchor='n')

        right = tk.Frame(row, bg=C['card'])
        right.pack(side='right')
        text = tk.Frame(row, bg=C['card'], padx=12)
        text.pack(side='left', fill='x', expand=True)
        tk.Label(text, text=name, font=self.f_name, bg=C['card'], fg=C['text'], anchor='w').pack(fill='x')
        line = tk.Frame(text, bg=C['card'])
        for t, color in detail:
            if t:
                tk.Label(line, text=t, font=self.f_body, bg=C['card'], fg=color).pack(side='left')
        if tag:
            tk.Label(line, text=tag, font=self.f_small, bg=C['blue_bg'], fg=C['blue'],
                     padx=8, pady=1).pack(side='left', padx=(10, 0))
        if badge:
            tk.Label(line, text=badge, font=self.f_small, bg=C['amber_bg'], fg=C['amber'],
                     padx=8, pady=1).pack(side='left', padx=(10, 0))
        if line.winfo_children():
            line.pack(fill='x')
        if desc:
            tk.Label(text, text=desc, font=self.f_body, bg=C['card'], fg=C['muted'], anchor='w',
                     justify='left', wraplength=500).pack(fill='x')
        error = tk.Label(text, font=self.f_body, bg=C['card'], fg=C['red'], anchor='w',
                         justify='left', wraplength=500)

        if pill:
            t, fg, bg = {'up':       ('✓ Up to date', C['green'], C['green_bg']),
                         'error':    ("Couldn't check", C['red'], C['red_bg']),
                         'local':    ('No upstream', C['muted'], C['grey_bg']),
                         'editable': ('Local install', C['blue'], C['blue_bg']),
                         'detached': ('Detached HEAD', C['muted'], C['grey_bg'])}[pill]
            tk.Label(right, text=t, font=self.f_small, fg=fg, bg=bg, padx=10, pady=4).pack()
            return

        label, cmd, cwd = action
        primary = C['blue'] if is_update else C['card']
        btn = FlatButton(right, label, None, bg=primary, fg='white' if is_update else C['blue'],
                         hover=C['blue_hi'] if is_update else C['grey_bg'], font=self.f_btn, width=9)
        if not is_update:
            btn.configure(highlightthickness=1, highlightbackground=C['blue'])
        btn.pack()

        def run(done=None):
            self.actions[:] = [a for a in self.actions if a[0] is not btn]
            self._refresh_update_all()
            self._run_action(btn, error, label, cmd, cwd, done)

        btn._command = run
        if is_update:
            self.actions.append((btn, run))

    # ---- updating ----------------------------------------------------------
    def _run_action(self, btn, error, label, cmd, cwd, done=None):
        btn.set(text='Working…', bg=C['border'], fg=C['muted'], enabled=False)
        error.pack_forget()

        def worker():
            if label == 'Clone':
                os.makedirs(self.repos_dir, exist_ok=True)
            self._log(f'$ {" ".join(cmd)}' + (f'   (in {cwd})' if cwd else ''))
            try:
                res = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                                     env=ENV, stdin=subprocess.DEVNULL, timeout=1800)
                ok, out = res.returncode == 0, (res.stdout + res.stderr).strip()
            except Exception as e:
                ok, out = False, str(e)
            self._log(out + '\n')

            def finish():
                if ok:
                    btn.set(text='✓ Done', bg=C['green_bg'], fg=C['green'], enabled=False)
                    btn.configure(highlightthickness=0)
                else:
                    btn.set(text='Retry', bg=C['red'], fg='white', hover=C['red_hi'], enabled=True)
                    btn.configure(highlightthickness=0)
                    last = out.splitlines()[-1] if out else 'Unknown error'
                    error.configure(text=f'{label} failed: {last}')
                    error.pack(fill='x', pady=(4, 0))
                if done:
                    done()
            self.root.after(0, finish)
        threading.Thread(target=worker, daemon=True).start()

    def update_all(self):
        """Run every pending update one at a time, then re-check."""
        pending = [run for _, run in self.actions]
        self._set_header('working', 'Installing updates…', 'This may take a few minutes.')
        self._set_busy(True)

        def next_action():
            if pending:
                pending.pop(0)(next_action)
            else:
                self.check()
        next_action()


def main():
    parser = argparse.ArgumentParser(description='GUI to update Stretch 4 pip packages and git repos')
    parser.add_argument('--repos', default=os.path.expanduser('~/repos'),
                        help='Folder containing git repos (default: ~/repos)')
    args = parser.parse_args()
    root = tk.Tk()
    UpdaterApp(root, args.repos)
    root.mainloop()


if __name__ == '__main__':
    main()
