# -*- coding: utf-8 -*-
"""Check a prompt for the five parts of Chapter 4, before it is sent.

    python tools/prompt_lint.py scripts/FirstAssembly/PROMPT.txt
    python tools/prompt_lint.py --all            every scripts/*/PROMPT.txt

What it checks (it cannot tell whether the design is right, only whether the prompt leaves things to guess):
  1. the goal names the deliverable: a Fusion script that, when run, builds ...
  2. an INPUTS block in which every input has a unit (or is a count) and a default
  3. a DERIVED block: what is computed from the inputs
  4. a HARD CONSTRAINTS block, with at least one refusal ("refuse")
  5. a FUSION CONVENTIONS block naming the document type, the units and the joint kind
  6. an ACCEPTANCE CRITERIA block with numbers worked out in advance
"""
import glob
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLOCKS = ['INPUTS', 'DERIVED', 'HARD CONSTRAINTS', 'FUSION CONVENTIONS', 'ACCEPTANCE CRITERIA']


def blocks(text):
    """Split the prompt at its block headings (a line starting with the heading in capitals)."""
    out, cur = {'GOAL': []}, 'GOAL'
    for line in text.split('\n'):
        head = next((b for b in BLOCKS if line.strip().upper().startswith(b)), None)
        if head:
            cur = head
            out[cur] = [line]
        else:
            out.setdefault(cur, []).append(line)
    return {k: '\n'.join(v) for k, v in out.items()}


def lint(path):
    text = io.open(path, encoding='utf-8').read()
    b = blocks(text)
    res = []
    res.append(('goal names the deliverable', bool(re.search(r'script that,? when run,? builds', b['GOAL'], re.I))))
    for name in BLOCKS:
        res.append(('%s block present' % name.lower(), name in b))
    items = []                                  # an input may run on to the next, indented line
    for l in b.get('INPUTS', '').split('\n'):
        if l.strip().startswith('-'):
            items.append(l.strip())
        elif items and l.startswith('  ') and l.strip():
            items[-1] += ' ' + l.strip()
    unit = r'\((mm|deg|rpm|N|N\.m|N·m|kg|s|%|MPa|GPa|h|um)\)|teeth|count|number of|\b[zZN]\w*,'
    choice = r'class|one of|condition|\bor\b'   # a choice needs no unit
    unitless = [l for l in items if not re.search(unit, l) and not re.search(choice, l)]
    nodefault = [l for l in items if not re.search(r',\s*[-+\w.]+(\s+\w+)*\s*$', l)]
    res.append(('every input has a unit or is a count (%d inputs)' % len(items), bool(items) and not unitless))
    res.append(('every input has a default', bool(items) and not nodefault))
    res.append(('a hard constraint refuses bad input', 'refuse' in b.get('HARD CONSTRAINTS', '').lower()))
    conv = b.get('FUSION CONVENTIONS', '')
    res.append(('conventions: document type', 'FusionDesignDocumentType' in conv))
    res.append(('conventions: units (cm)', bool(re.search(r'\bcm\b', conv))))
    res.append(('conventions: As-built joints', bool(re.search(r'as-?built', conv, re.I))))
    acc = b.get('ACCEPTANCE CRITERIA', '')
    res.append(('acceptance criteria give numbers', len(re.findall(r'\d+(?:\.\d+)?', acc)) >= 3))
    res.append(('acceptance criteria include a refusal', 'refused' in acc.lower()))
    return res, unitless + ['(no default) ' + l for l in nodefault]


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    paths = sorted(glob.glob(os.path.join(ROOT, 'scripts', '*', 'PROMPT.txt'))) if sys.argv[1:] == ['--all'] else sys.argv[1:]
    bad = 0
    for p in paths:
        res, notes = lint(p)
        miss = [n for n, good in res if not good]
        bad += bool(miss)
        print('%-44s %2d of %d %s' % (os.path.relpath(p, ROOT), len(res) - len(miss), len(res), 'ok' if not miss else 'MISSING: ' + '; '.join(miss)))
        for n in notes:
            print('      ' + n)
    sys.exit(1 if bad else 0)
