# -*- coding: utf-8 -*-
"""Run a check in Autodesk Fusion and keep its result with the Fusion version and the date.

    python tools/run_check.py gearpair_check            -> results/gearpair_check.txt
    python tools/run_check.py controls/planetary_ring_phase   a control: it must FAIL its clash check
    python tools/run_check.py --all                     every check in checks/, in order
    python tools/run_check.py --cleanup                 close, without saving, the designs the checks opened

Fusion must be running with its MCP server on (Preferences > General > API > "Fusion MCP Server (runs locally on this
device)"); tools/fusion_mcp.py finds its port (Chapter 3). A check is a Fusion script with run(_context); the text
__COMPANION__ in it is replaced
by this repository's folder before it is sent, so the checks work wherever the repository is cloned. Every design a
check opens is marked 'mdfc' and only those are closed by --cleanup.
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


def run_check(name):
    path = os.path.join(ROOT, name + '.py') if '/' in name else os.path.join(ROOT, 'checks', name + '.py')
    src = io.open(path, encoding='utf-8').read().replace('__COMPANION__', ROOT.replace('\\', '/'))
    version = send(VERSION_PROBE).strip()
    t0 = time.time()
    rep = send(src)
    head = '# %s\n# Autodesk Fusion %s, run %s, %.0f s\n\n' % (name, version, time.strftime('%Y-%m-%d %H:%M'), time.time() - t0)
    os.makedirs(os.path.join(ROOT, 'results'), exist_ok=True)
    io.open(os.path.join(ROOT, 'results', name.replace('/', '_') + '.txt'), 'w', encoding='utf-8', newline='\n').write(head + rep)
    return head + rep


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    if a[0] == '--cleanup':
        print(send(CLEANUP))
    elif a[0] == '--all':
        names = sorted(f[:-3] for f in os.listdir(os.path.join(ROOT, 'checks')) if f.endswith('.py'))
        for n in names:
            rep = run_check(n)
            print('%-34s %s' % (n, 'ALL PASS' if 'ALL PASS' in rep else 'see results/%s.txt' % n))
            print(send(CLEANUP))
    else:
        print(run_check(a[0]))
