# -*- coding: utf-8 -*-
"""Check a prompt for the five parts of Chapter 4, before it is sent.

    python tools/prompt_lint.py projects/first_assembly/FirstAssembly/PROMPT.txt
    python tools/prompt_lint.py --all            every PROMPT.txt in projects/, design_office/ and common/

What it checks (it cannot tell whether the design is right, only whether the prompt leaves things to guess):
  1. the goal names the deliverable: a Fusion script that, when run, builds ...
  2. an INPUTS block in which every input has a unit (or is a count) and a default
  3. a DERIVED block: what is computed from the inputs
  4. a HARD CONSTRAINTS block, with at least one refusal ("refuse")
  5. a FUSION CONVENTIONS block naming the document type, the units and the joint kind
  6. an ACCEPTANCE CRITERIA block with numbers worked out in advance
  7. the prompt stands on its own: it cites no chapter, part, section, table or figure of a book (a named source's
     table, such as Shigley's Table 7-1, is fine)
  8. every file it names is named by its place: projects/<project>/<Name>/<Name>.py, design_office/lib/x.py
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
    unit = r'\((mm|deg|rpm|N|N\.m|N·m|kg|s|%|MPa|GPa|bar|h|um|kW|W)\)|teeth|count|number of|\b[zZN]\w*,'
    choice = r'class|one of|condition|\bor\b|\bratio\b|\bfactor\b'   # a choice, a ratio or a factor needs no unit
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
    refs = book_refs(text)
    res.append(('stands on its own: no chapter, part or table of a book the model does not have', not refs))
    loose = unplaced(text, os.path.basename(os.path.dirname(os.path.abspath(path))))
    res.append(('every file named by its place (project/folder/file)', not loose))
    return res, unitless + ['(no default) ' + l for l in nodefault] + ['(cites the book) ' + r for r in refs] + \
        ['(no place given) ' + f for f in loose]


# A prompt is read by a model that has never seen the book it was printed in: "as in Chapter 12" tells it nothing.
# A book's own chapter, part, section, table or figure is refused; one of a named source (Shigley's Table 7-1,
# SKF's table 1, ISO 286-2) is kept, the source named within the same line.
BOOK_REF = re.compile(r"\b([Cc]hapters?\b|Volumes?\b|Vol\.|Appendix|Parts? \d+|Sections? \d+|Table \d+\.\d+|Figure \d+|"
                      r"Fig\. \d+|Eqs?\. \d+\.\d+|this book|the book)")
SOURCE = re.compile(r"Shigley|Budynas|Nisbett|Childs|SKF|ISO \d|DIN \d|AGMA|Norton|Juvinall|Roark|Peterson|Boothroyd|"
                    r"igus|HIWIN|maxon|Festo|Nabtesco|Beckhoff|Harmonic Drive|\(\d{4}\)|, \d{4}\b|\d{4}, ")


def book_refs(text):
    out = []
    for line in text.split('\n'):
        for m in BOOK_REF.finditer(line):
            near = line[max(0, m.start() - 30):m.start()]                                # the source named before it
            if not m.group(0).lower().startswith(('chapter', 'volume', 'vol.')):
                near += line[m.end():m.end() + 25]          # a table or figure may name its source after it too
            if not SOURCE.search(near):
                out.append(line.strip()[:90])
    return out


# Every file a prompt names is named by its place in the project tree, at least once: a script as
# <project>/<Name>/<Name>.py, a library or a result as <project>/lib/x.py, <project>/results/x. The prompt's own
# script needs no place: it is the folder the prompt is kept in.
def unplaced(text, own):
    out = []
    for m in re.finditer(r"(?<![\w/.])(lib|results|checks|controls|captures)/[\w*<>%.\-]+", text):
        out.append(m.group(0))
    for n in sorted(set(re.findall(r"(?<![/\w])([A-Z][A-Za-z0-9]+)\.py\b", text))):
        if n != own and not re.search(r"[\w.]/%s/%s\.py\b" % (n, n), text):
            out.append(n + '.py')
    return out


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    paths = sorted(glob.glob(os.path.join(ROOT, 'projects', '*', '*', 'PROMPT.txt')) + glob.glob(os.path.join(ROOT, 'design_office', '*', 'PROMPT.txt'))
                   + glob.glob(os.path.join(ROOT, 'common', '*', 'PROMPT.txt'))) if sys.argv[1:] == ['--all'] else sys.argv[1:]
    bad = 0
    for p in paths:
        res, notes = lint(p)
        miss = [n for n, good in res if not good]
        bad += bool(miss)
        print('%-56s %2d of %d %s' % (os.path.relpath(p, ROOT), len(res) - len(miss), len(res), 'ok' if not miss else 'MISSING: ' + '; '.join(miss)))
        for n in notes:
            print('      ' + n)
    sys.exit(1 if bad else 0)
