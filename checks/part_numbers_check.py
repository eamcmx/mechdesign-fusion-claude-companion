# PartNumbers (Chapter 49) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the bag gripper built by its own script and numbered; every component with a body numbered, or with a bought part,
# or one of the gripper's non-parts; one number for one kind; Chapter 49's counts; the root P13-000; a refusal.
import re, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import detail_inventory as D
importlib.reload(D)

PATH = '__COMPANION__/scripts/PartNumbers/PartNumbers.py'
SCHEME = re.compile(r'^([WFAP]\d\d-\d{3}|S-\d{4})$')


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    nmap = D.number_map('bag')
    inv = D.inventory('bag')
    comps = {}
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count:
            comps[c.entityToken] = (c, o.fullPathName)
    print('  components with a body: %d' % len(comps))
    numbered, part_of, non_parts, wrong = [], [], [], []
    by_num, by_kind = {}, {}
    for c, path in comps.values():
        kind = D.kind_of(c.name, nmap)
        if kind is None:
            (non_parts if path.startswith('Bag') or path.startswith('Station') else wrong).append(c.name)
        elif nmap[kind][0] is None:
            part_of.append(c.name) if c.description.startswith('with the ') else wrong.append(c.name)
        elif SCHEME.match(c.partNumber) and c.description == nmap[kind][1]:
            numbered.append(c.name)
            by_num.setdefault(c.partNumber, set()).add(kind)
            by_kind.setdefault(kind, set()).add(c.partNumber)
        else:
            wrong.append(c.name)
    fc.ok(not wrong, 'every component numbered, with a bought part, or not a part: %d, %d, %d; wrong %s'
          % (len(numbered), len(part_of), len(non_parts), sorted(wrong)[:6]))
    fc.ok(all(len(v) == 1 for v in by_num.values()) and all(len(v) == 1 for v in by_kind.values()),
          'one number for one kind: %d numbers for %d kinds' % (len(by_num), len(by_kind)))
    made = sorted(n for n in by_num if not n.startswith('S-'))
    bought = sorted(n for n in by_num if n.startswith('S-'))
    fc.ok(len(made) == inv['made_kinds'] == 13 and made[0] == 'P13-001' and made[-1] == 'P13-013',
          'the made kinds: %d, %s to %s' % (len(made), made[0] if made else '-', made[-1] if made else '-'))
    fc.ok(len(bought) == inv['bought_kinds'] == 8, 'the bought kinds: %d (%s)' % (len(bought), ', '.join(bought)))
    fc.ok(len(part_of) == 2 and all(n.startswith('Piston rod') for n in part_of), 'with a cylinder: %s' % sorted(part_of))
    fc.ok(root.partNumber == 'P13-000', 'the root: %s, %s' % (root.partNumber, root.description))
    fc.ok(bool(non_parts), 'left unnumbered, not parts: %s' % sorted(set(non_parts))[:6])
    fc.refused(ns, 'No assembly', key='crane')
    fc.verdict()
