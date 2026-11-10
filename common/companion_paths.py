# -*- coding: utf-8 -*-
"""companion_paths.py: where everything in the companion is kept, so that no script, check or tool has to know.

The companion is kept as a design office keeps its work: one folder for each machine and one for each exercise, every
file of the job inside it.

    projects/<machine>/        one machine or one exercise: a folder for each of its scripts (the script, its
                               manifest and the prompt it was written from), lib/ (its design calculations and the
                               weights Fusion measured), checks/, controls/, captures/ (the pictures), results/,
                               references/ (the catalogues and standards it was sized from, listed with their links)
                               and a README listing them
    design_office/             the work done on every machine: part numbers, materials and fits, the range sweep,
                               masses, studies, stack-ups, drawings, bills of materials, assembly, making
    common/                    what every project uses: this file, the parts library (lib/) and BallScrewAxis
    extras/                    scripts kept for reference that no chapter uses
    tools/                     running checks, captures and the prompt checker

A script finds this file by walking up from its own folder to the first one that holds common/companion_paths.py,
puts common/ on sys.path and imports it:

    import companion_paths as CP     # every lib/ folder is now on sys.path, so `import scara_design` works
    CP.ROOT                          the companion's folder
    CP.script('BagGripper')          .../projects/palletizer/BagGripper/BagGripper.py
    CP.project('BagGripper')         .../projects/palletizer
    CP.result('bom.json')            the file of that name in whichever project's results/ holds it
    CP.lib_file('named_materials.json')   the file of that name in whichever lib/ holds it
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOPS = ('common', 'design_office', 'extras')


def places():
    """Every project folder: common, design_office, extras and each folder under projects/."""
    out = [os.path.join(ROOT, t) for t in TOPS]
    pj = os.path.join(ROOT, 'projects')
    if os.path.isdir(pj):
        out += [os.path.join(pj, d) for d in sorted(os.listdir(pj)) if os.path.isdir(os.path.join(pj, d))]
    return [p for p in out if os.path.isdir(p)]


LIBS = [os.path.join(p, 'lib') for p in places() if os.path.isdir(os.path.join(p, 'lib'))]
for _d in reversed(LIBS):
    if _d not in sys.path:
        sys.path.insert(0, _d)


def scripts():
    """{name: path} of every script: a folder <Name> holding <Name>.py, in any project."""
    out = {}
    for p in places():
        for d in os.listdir(p):
            f = os.path.join(p, d, d + '.py')
            if os.path.isfile(f):
                out[d] = f
    return out


def script(name):
    """The path of the script <name>.py, wherever its project keeps it."""
    s = scripts()
    if name not in s:
        raise KeyError('No script called %s in %s' % (name, ROOT))
    return s[name]


def project(name):
    """The project folder that keeps the script <name>."""
    return os.path.dirname(os.path.dirname(script(name)))


def _find(sub, parts):
    hits = [os.path.join(p, sub, *parts) for p in places() if os.path.exists(os.path.join(p, sub, *parts))]
    if not hits:
        raise FileNotFoundError('No %s/%s in any project of %s' % (sub, '/'.join(parts), ROOT))
    return hits[0]


def result(*parts):
    """A file (or folder) kept in some project's results/, found by its name: result('bom', 'axle.csv')."""
    return _find('results', parts)


def lib_file(*parts):
    """A file kept in some project's lib/, found by its name."""
    return _find('lib', parts)


def check(name):
    """The path of the check, control or capture called <name>: checks/<name>.py, controls/<name>.py or
    captures/<name>.py in whichever project keeps it."""
    for p in places():
        for sub in ('checks', 'controls', 'captures'):
            f = os.path.join(p, sub, name + '.py')
            if os.path.isfile(f):
                return f
    raise KeyError('No check, control or capture called %s in %s' % (name, ROOT))
