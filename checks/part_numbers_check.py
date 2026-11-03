# PartNumbers (Chapter 49) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the bag gripper built by its own script and numbered; every component with a body numbered, or with a bought part,
# or one of the gripper's non-parts; one number for one kind; Chapter 49's counts; the root P13-000; then the
# compliance unit numbered after the eight, its new designations on the next free serials; a refusal.
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
    # the compliance unit, numbered after the eight: its made parts P14, its new designations the next free serials
    print('  ' + ns['build']('unit').splitlines()[0])
    root = fc.design().rootComponent
    nmap = D.number_map('unit')
    nums, wrong = {}, []
    for o in root.allOccurrences:
        c = o.component
        if not c.bRepBodies.count:
            continue
        kind = D.kind_of(c.name, nmap)
        if kind is None or not SCHEME.match(c.partNumber) or c.description != nmap[kind][1]:
            wrong.append(c.name)
        else:
            nums.setdefault(c.partNumber, set()).add(kind)
    made = sorted(n for n in nums if not n.startswith('S-'))
    new = sorted(n for n in nums if n.startswith('S-') and n > 'S-0100')
    fc.ok(not wrong and root.partNumber == 'P14-000' and made == ['P14-%03d' % k for k in range(1, 7)]
          and len(nums) - len(made) == 10 and new == ['S-%04d' % k for k in range(101, 107)],
          'the compliance unit: root %s; made %s to %s; %d bought, the new ones %s to %s; wrong %s'
          % (root.partNumber, made[0] if made else '-', made[-1] if made else '-', len(nums) - len(made),
             new[0] if new else '-', new[-1] if new else '-', sorted(wrong)[:4]))
    fc.ok(len(nums['S-0102']) == 2 and len(nums['S-0106']) == 1,
          'two names, one part: %s as S-0102; the unit\'s spring S-0106, not the vacuum gripper\'s'
          % sorted(nums['S-0102']))
    fc.refused(ns, 'No assembly', key='crane')
    fc.verdict()
