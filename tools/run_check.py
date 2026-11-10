# -*- coding: utf-8 -*-
"""Run a check in Autodesk Fusion and keep its result with the Fusion version and the date.

    python tools/run_check.py gearpair_check            -> projects/gear_pair/results/gearpair_check.txt
    python tools/run_check.py controls/planetary_ring_phase   a control: it must FAIL its clash check
                                    -> projects/planetary_gearbox/results/controls_planetary_ring_phase.txt
    python tools/run_check.py --all                     every check of every project but extras/, by name
    python tools/run_check.py --cleanup                 close, without saving, the designs the checks opened

A check is found by its name in whichever project keeps it (<project>/checks/<name>.py; a control, named
controls/<name>, in <project>/controls/), and its result goes to that project's results/. A path from the
companion's folder works too: projects/fit_pair/controls/fit_pair_nominal.

Fusion must be running with its MCP server on (Preferences > General > API > "Fusion MCP Server (runs locally on this
device)"); tools/fusion_mcp.py finds its port (Chapter 3). A check is a Fusion script with run(_context); the text
__COMPANION__ in it is replaced by this repository's folder before it is sent, so the checks work wherever the
repository is cloned. Every design a check opens is marked 'mdfc' and only those are closed by --cleanup.
"""
import io
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MCP = os.path.join(HERE, 'fusion_mcp.py')
TMP = os.path.join(os.environ.get('TEMP', ROOT), 'mdfc_check_run.py')
sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP  # noqa: E402

VERSION_PROBE = '''import adsk.core
def run(_context):
    print(adsk.core.Application.get().version)
'''

CLEANUP = '''import adsk.core, adsk.fusion
def run(_context):
    app = adsk.core.Application.get()
    shut = []
    for i in reversed(range(app.documents.count)):
        d = app.documents.item(i)
        mark = None
        for group in ('mdfc', 'rdm_t5'):
            try:
                mark = mark or d.attributes.itemByName(group, 'run')
            except Exception:
                pass
        if mark is not None:
            shut.append(d.name)
            d.close(False)
    print('closed without saving: %d design(s); still open: %s' % (len(shut), [app.documents.item(i).name for i in range(app.documents.count)]))
'''


def send(source):
    io.open(TMP, 'w', encoding='utf-8').write(source)
    r = subprocess.run([sys.executable, MCP, 'script', TMP], capture_output=True, encoding='utf-8', errors='replace', timeout=3600)
    out = r.stdout + r.stderr
    try:
        d = json.loads(out[out.index('{'):])
        return d.get('message') or d.get('error') or out
    except Exception:
        return out


def find(name):
    """(the check's file, the name of its result file): a check by its name, a control as controls/<name>, or a path
    from the companion's folder."""
    if name.endswith('.py'):
        name = name[:-3]
    direct = os.path.join(ROOT, name + '.py')
    if os.path.isfile(direct):
        path = direct
    else:
        sub, base = ('controls', name.split('/', 1)[1]) if name.startswith('controls/') else ('checks', name)
        hits = [os.path.join(p, sub, base + '.py') for p in CP.places() if os.path.isfile(os.path.join(p, sub, base + '.py'))]
        if not hits:
            raise SystemExit('No %s called %s in any project.' % (sub[:-1], base))
        path = hits[0]
    stem = os.path.basename(path)[:-3]
    return path, ('controls_' + stem if os.path.basename(os.path.dirname(path)) == 'controls' else stem)


def result_path(name):
    """Where the result of the check or control called name is kept: its own project's results/."""
    path, stem = find(name)
    return os.path.join(os.path.dirname(os.path.dirname(path)), 'results', stem + '.txt')


def run_check(name):
    path, stem = find(name)
    src = io.open(path, encoding='utf-8').read().replace('__COMPANION__', ROOT.replace('\\', '/'))
    version = send(VERSION_PROBE).strip()
    t0 = time.time()
    rep = send(src)
    head = '# %s\n# Autodesk Fusion %s, run %s, %.0f s\n\n' % (name, version, time.strftime('%Y-%m-%d %H:%M'), time.time() - t0)
    out = result_path(name)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    io.open(out, 'w', encoding='utf-8', newline='\n').write(head + rep)
    return head + rep


def all_checks():
    """Every check of every project but extras/, by name."""
    return sorted(f[:-3] for p in CP.places() if os.path.basename(p) != 'extras' and os.path.isdir(os.path.join(p, 'checks'))
                  for f in os.listdir(os.path.join(p, 'checks')) if f.endswith('.py'))


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == '--cleanup':
        print(send(CLEANUP))
    elif a[0] == '--all':
        for n in all_checks():
            rep = run_check(n)
            print('%-34s %s' % (n, 'ALL PASS' if 'ALL PASS' in rep else 'see ' + os.path.relpath(result_path(n), ROOT)))
            print(send(CLEANUP))
    else:
        print(run_check(a[0]))
