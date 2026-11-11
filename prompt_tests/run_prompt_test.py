# -*- coding: utf-8 -*-
"""Run a project's check on the script written from its prompt alone (prompt_tests/<Name>/<Name>.py) instead of the
companion's own script, and keep the result beside it.

    python prompt_tests/run_prompt_test.py KeyedShaft        -> prompt_tests/KeyedShaft/check_result.txt

The check is the project's own, unchanged but for the path of the script it loads. Fusion must be running with its MCP
server on, and no other check running.
"""
import glob
import io
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'tools'))
sys.argv, args = [sys.argv[0]], sys.argv[1:]
import run_check as R  # noqa: E402


def check_for(name):
    """The check that loads the companion's <Name>.py (its PATH line)."""
    want = '/%s/%s.py' % (name, name)
    hits = []
    for p in R.CP.places():
        for f in glob.glob(os.path.join(p, 'checks', '*.py')):
            src = io.open(f, encoding='utf-8').read()
            if want in src and ('PATH = ' in src):
                line = [l for l in src.split('\n') if l.startswith('PATH = ')]
                if line and want in line[0]:
                    hits.append((f, line[0]))
    if not hits:
        raise SystemExit('no check loads %s.py' % name)
    return hits[0]


def main(name):
    check, line = check_for(name)
    new = os.path.join(HERE, name, name + '.py')
    if not os.path.isfile(new):
        raise SystemExit('not written yet: ' + new)
    src = io.open(check, encoding='utf-8').read()
    quote = line[len('PATH = ')]
    src = src.replace(line, 'PATH = %s__COMPANION__/prompt_tests/%s/%s.py%s' % (quote, name, name, quote), 1)
    src = src.replace('__COMPANION__', ROOT.replace('\\', '/'))
    version = R.send(R.VERSION_PROBE).strip()
    t0 = time.time()
    rep = R.send(src)
    head = '# %s on prompt_tests/%s/%s.py\n# Autodesk Fusion %s, run %s, %.0f s\n\n' % (
        os.path.relpath(check, ROOT).replace('\\', '/'), name, name, version, time.strftime('%Y-%m-%d %H:%M'), time.time() - t0)
    io.open(os.path.join(HERE, name, 'check_result.txt'), 'w', encoding='utf-8', newline='\n').write(head + rep)
    print(head + rep)
    R.send(R.CLEANUP)


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    main(args[0])
