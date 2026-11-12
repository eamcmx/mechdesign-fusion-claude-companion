# -*- coding: utf-8 -*-
"""What a project's check looks for in the script it runs: the names a script written from the prompt must use.

    python tools/check_names.py RearAxleCarrier          the names, as the prompt's line for the check
    python tools/check_names.py --all                    every prompt, and whether its PROMPT.txt names them all

A check drives a script through a few conventions (CHECK_INTERFACE.md at the companion's root) and a few names of its
own: build()'s keyword arguments, which are also the dialog's input ids; the joints and components it looks up; the
module-level functions and values it calls or reads; what the script keeps in _last; and the words a refusal must
say. This tool reads them out of the check, and out of the companion's own script for build()'s arguments, so that a
prompt can give them and a reader's model need not see the check.
"""
import glob
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP  # noqa: E402

STANDARD = {'VERSION', 'build', 'run', '_Created', '_mdfc_message', '_last', '__file__'}


def check_for(name):
    """The check whose PATH line loads the companion's <Name>.py."""
    want = '/%s/%s.py' % (name, name)
    for p in CP.places():
        for f in sorted(glob.glob(os.path.join(p, 'checks', '*.py'))):
            src = io.open(f, encoding='utf-8').read()
            line = [l for l in src.split('\n') if l.startswith('PATH = ')]
            if line and want in line[0]:
                return f
    return None


def script_for(name):
    for p in CP.places():
        f = os.path.join(p, name, name + '.py')
        if os.path.isfile(f):
            return f
    return None


def build_args(script):
    """build()'s keyword arguments, in order, from the companion's script."""
    src = io.open(script, encoding='utf-8').read()
    m = re.search(r'^def build\((.*?)\):', src, re.S | re.M)
    if not m:
        return []
    out = []
    for a in re.split(r',\s*(?![^()]*\))', m.group(1)):
        a = a.strip()
        if a and not a.startswith('*'):
            out.append(a.split('=')[0].strip())
    return out


def dialog_ids(script):
    src = io.open(script, encoding='utf-8').read()
    ids = re.findall(r"\.add\w*?Input\(\s*'([A-Za-z_0-9]+)'", src)
    m = re.search(r'^FIELDS = \[(.*?)\]\s*$', src, re.S | re.M)
    if m:
        ids += re.findall(r"\(\s*'([A-Za-z_0-9]+)'", m.group(1))
    seen = []
    for i in ids:
        if i not in seen:
            seen.append(i)
    return seen


def aliases(script):
    """The libraries the companion's script imports under a short name: {alias: module}."""
    src = io.open(script, encoding='utf-8').read()
    return {a: m for m, a in re.findall(r'^\s*import ([A-Za-z_0-9]+) as ([A-Za-z_0-9]+)', src, re.M)}


def names(name):
    chk, scr = check_for(name), script_for(name)
    if not chk or not scr:
        return None
    src = io.open(chk, encoding='utf-8').read()
    out = dict(check=os.path.relpath(chk, ROOT).replace('\\', '/'), build=build_args(scr), ids=dialog_ids(scr))
    out['joints'] = sorted(set(re.findall(r"""fc\.joint\(\s*['"]([^'"]+)['"]""", src)))
    out['occs'] = sorted(set(re.findall(r"""fc\.occ\(\s*['"]([^'"]+)['"]""", src)))
    ns = sorted(set(re.findall(r"""ns\[['"]([A-Za-z_0-9]+)['"]\]""", src)) - STANDARD)
    al = aliases(scr)
    out['libs'] = ['%s imported as %s' % (al[x], x) for x in ns if x in al]
    out['module'] = [x for x in ns if x not in al]
    out['calls'] = set(re.findall(r"""ns\[['"]([A-Za-z_0-9]+)['"]\]\(""", src))
    last = set(re.findall(r"""ns\[['"]_last['"]\]\[['"]([A-Za-z_0-9]+)['"]\]""", src))
    last |= set(re.findall(r"""ns\[['"]_last['"]\]\.get\(['"]([A-Za-z_0-9]+)['"]""", src))
    out['last'] = sorted(last)
    out['uses_last'] = "ns['_last']" in src or 'ns["_last"]' in src
    out['refusals'] = []
    for m in re.finditer(r"""fc\.refused\(\s*ns\s*,\s*(['"])(.+?)\1\s*(?:,\s*(.*?))?\)\s*$""", src, re.M):
        out['refusals'].append((m.group(2), (m.group(3) or '').strip()))
    vals = re.findall(r"run_dialog\(\s*ns\s*,\s*\{(.*?)\}", src, re.S)
    out['typed'] = sorted(set(k for v in vals for k in re.findall(r"""['"]([A-Za-z_0-9]+)['"]\s*:""", v)))
    return out


def readable(name):
    """A name the check builds with % (one per side or per part): 'Bolt %d' reads 'Bolt <n>'."""
    return name.replace('%d', '<n>').replace('%s', '<side>')


def unique(seq):
    out = []
    for x in seq:
        if x not in out:
            out.append(x)
    return out


def line(n):
    """The prompt's line for the check, in the prompt's own plain words."""
    args = ', '.join(n['build'])
    if not n['build']:
        parts = ['build()']
    elif set(n['ids']) <= set(n['build']):
        parts = ["build(%s), the dialog's input ids the same" % args]
    else:
        parts = ["build(%s); the dialog's input ids %s" % (args, ', '.join(n['ids']))]
    if n['joints']:
        parts.append('the joint%s %s' % ('s' if len(n['joints']) > 1 else '',
                                         ', '.join("'%s'" % readable(j) for j in n['joints'])))
    if n['occs']:
        parts.append('the component%s %s' % ('s' if len(n['occs']) > 1 else '',
                                             ', '.join("'%s'" % readable(o) for o in n['occs'])))
    mod = [m + '()' if m in n['calls'] else m for m in n['module']] + n['libs']
    if mod:
        parts.append('at module level %s' % ', '.join(mod))
    if n['last']:
        parts.append('_last holding %s' % ', '.join("'%s'" % k for k in n['last']))
    words = unique(w for w, _ in n['refusals'])
    if words:
        parts.append('refusals that say %s' % ', '.join("'%s'" % w for w in words))
    return ("- The companion's check, %s, drives the script as CHECK_INTERFACE.md says (give it to the model for its "
            "names, never for a design fact): %s." % (n['check'], '; '.join(parts)))


def wanted(n):
    """What a prompt must carry word for word for its check."""
    want = list(n['build']) + [readable(x) for x in n['joints'] + n['occs']] + n['module'] + n['libs'] + n['last']
    return unique(want + [w for w, _ in n['refusals']] + [n['check'], 'CHECK_INTERFACE.md'])


def missing(name, n, text=None):
    """The check's names that its PROMPT.txt does not carry, word for word."""
    if text is None:
        p = os.path.join(os.path.dirname(script_for(name)), 'PROMPT.txt')
        text = io.open(p, encoding='utf-8').read()
    text = re.sub(r'\s+', ' ', text)
    return [w for w in wanted(n) if re.sub(r'\s+', ' ', w) not in text]


def prompts():
    """The prompts the books print: every PROMPT.txt in projects/, design_office/ and common/."""
    out = []
    for top in ('projects/*/*', 'design_office/*', 'common/*'):
        for f in sorted(glob.glob(os.path.join(ROOT, top, 'PROMPT.txt'))):
            out.append(os.path.basename(os.path.dirname(f)))
    return sorted(out)


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    if sys.argv[1:] == ['--all']:
        bad = 0
        for nm in prompts():
            n = names(nm)
            if n is None:
                print('%-22s no check loads it' % nm)
                continue
            miss = missing(nm, n)
            bad += bool(miss)
            print('%-22s %s' % (nm, 'ok' if not miss else 'MISSING: ' + '; '.join(miss)))
        sys.exit(1 if bad else 0)
    for nm in sys.argv[1:]:
        n = names(nm)
        print(line(n) if n else '%s: no check loads it' % nm)
