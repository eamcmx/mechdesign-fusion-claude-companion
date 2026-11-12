# =============================================================================
# MassProperties.py  -  Chapter 57 of "Mechanical Design in Autodesk Fusion with Claude": the masses of the detailed
# assemblies. It builds one of the nine machines through RangeSweep.py's machine() (each with its own detail script,
# the bag gripper with its stops) and weighs it with Fusion's own physical properties: the whole, every group (a
# top-level occurrence and all the parts under it) and, for the robot set to q = 0, its eleven moving bodies in the
# form of projects/palletizer/lib/palletizer_weighed.json, so that Chapter 43's cycle (palletizer_links.py) can be run
# again on the detailed robot. Each machine's result goes to design_office/results/mass_<machine>.json.
# Written from design_office/MassProperties/PROMPT.txt alone (a prompt test).
#
#   INPUTS        the dialog: the machine (robot by default)
#   DERIVED       RangeSweep.py's machine(): the build; the materials the detail scripts give the parts
#   HARD RULES    RangeSweep.py loaded, not copied; Fusion's own physical properties of each part, moments about the
#                 world's axes moved to the centre by the parallel-axis rule; duplicate group names numbered
#   CONVENTIONS   each machine's own new design document, made by its own script; kg, mm, kg m2 (cm, kg cm2 inside);
#                 no joint added; nothing moved but the robot, by its own set_pose()
#   ACCEPTANCE    the message; the check: each machine weighed, its groups summing to its total, its file written,
#                 the robot's 11 bodies; a machine called crane refused
# =============================================================================
import importlib.util
import json
import math
import os
import re
import sys
import time
import traceback
import types

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # noqa: E402  every lib/ folder on sys.path; CP.script(name)

VERSION = 'MassProperties 1.0 (prompt test, 2026-11-12)'
MACHINES = ('robot', 'wrist', 'flap', 'axle', 'bag', 'clamp', 'vacuum', 'changer', 'unit')
RESULTS = os.path.join(CP.ROOT, 'design_office', 'results')
EVENT_ID = 'mdfcMassPropertiesBuild'
_last = {}

# The robot's eleven moving bodies, in palletizer_weighed.json's order and with its 'move' codes: (name, move, pivot,
# kind). kind 'q2': c along z, Ia about Z, It about Y, In about X; 'q3': c along x, Ia about X, It about Y, In about Z;
# 'plane': off = the centre's (x, z) from the pivot, and Izz. All moments about the body's own centre.
ROBOT = (('turret', 'turret', 'AXZ', 'plane'),
         ('lower arm', 'q2@A', 'A', 'q2'),
         ('crank and counterweight', 'q3@A', 'A', 'q3'),
         ('parallel rod', 'trC', 'C', 'q2'),
         ('levelling rod 1', 'q2@E', 'E', 'q2'),
         ('triangle', 'trB', 'B', 'plane'),
         ('upper arm', 'q3@B', 'B', 'q3'),
         ('levelling rod 2', 'q3@G', 'G', 'q3'),
         ('wrist housing', 'trW', 'W', 'plane'),
         ('tool flange', 'trW', 'W', 'plane'),
         ('balancer', 'turret', 'AXZ', 'plane'))
FLANGE_PARTS = ('rv-80n output j4', 'tool flange')     # taken out of the group Wrist: they spin with J4
BALANCER_WORDS = ('balancer', 'barrel', 'gas spring')


def _range_sweep():
    """RangeSweep.py, loaded from its own place (not copied), fresh on every build."""
    spec = importlib.util.spec_from_file_location('mdfc_mp57_RangeSweep', CP.script('RangeSweep'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---------- Fusion's physical properties ----------
def _pp(entity, accuracy=None):
    acc = adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy if accuracy is None else accuracy
    try:
        return entity.getPhysicalProperties(acc)
    except Exception:
        return entity.physicalProperties


def _read(pp):
    """(kg, centre in cm, moments of inertia about the world's x, y and z axes in kg cm2)."""
    m = pp.mass
    if not m or m <= 0.0:
        return 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    c = pp.centerOfMass
    r = pp.getXYZMomentsOfInertia()            # (ok, xx, yy, zz, xy, yz, xz)
    return m, (c.x, c.y, c.z), (r[1], r[2], r[3])


def _part(occ):
    """One part's own mass, centre and world-axis moments: the occurrence's physical properties, or, for a part that
    also holds other parts, the sum of its own bodies."""
    if occ.childOccurrences.count == 0:
        return _read(_pp(occ))
    m, s, I = 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    for b in occ.bRepBodies:
        bm, bc, bI = _read(_pp(b))
        m += bm
        for k in range(3):
            s[k] += bm * bc[k]
            I[k] += bI[k]
    c = tuple(s[k] / m for k in range(3)) if m > 0 else (0.0, 0.0, 0.0)
    return m, c, tuple(I)


def _d2(c):
    """Squared distances (cm2) of a point from the world's x, y and z axes."""
    return (c[1] ** 2 + c[2] ** 2, c[0] ** 2 + c[2] ** 2, c[0] ** 2 + c[1] ** 2)


def _about_origin(leaves):
    """True when Fusion's XYZ moments are about the world's axes, as the prompt has them: then no part's moment is
    less than m d2. If most parts far from an axis break that, the values were about each part's centre."""
    far = bad = 0
    for L in leaves:
        m, I = L['m'], L['I']
        if m <= 0.0:
            continue
        d2 = _d2(L['c'])
        if max(m * d for d in d2) < 1e-6:
            continue
        far += 1
        if any(I[k] < 0.999 * m * d2[k] - 1e-6 for k in range(3)):
            bad += 1
    return not (far and bad > 0.25 * far)


def _sum(recs):
    """A set of parts: mass (kg), centre (mm) and moments of inertia about the centre along the world's axes (kg m2),
    the parts' world-axis moments summed and moved to the centre by the parallel-axis rule."""
    m = sum(r['m'] for r in recs)
    if m <= 0.0:
        return dict(m=0.0, c=(0.0, 0.0, 0.0), I=(0.0, 0.0, 0.0), parts=len(recs))
    c = [sum(r['m'] * r['c'][k] for r in recs) / m for k in range(3)]
    Io = [sum(r['I'][k] for r in recs) for k in range(3)]
    d2 = _d2(c)
    Ic = [Io[k] - m * d2[k] for k in range(3)]
    return dict(m=m, c=tuple(10.0 * v for v in c), I=tuple(1e-4 * v for v in Ic), parts=len(recs))


def _materials(occ):
    names = set()
    try:
        for b in occ.bRepBodies:
            if b.material:
                names.add(b.material.name)
    except Exception:
        pass
    return names


# ---------- the groups ----------
def _group_names(tops):
    """Each top-level occurrence's component name; a name that comes twice is numbered (Name 1, Name 2, ...), and a
    numbered name never takes one that another group already has, so that no group is lost from the sum."""
    raw = [o.component.name for o in tops]
    count = {}
    for n in raw:
        count[n] = count.get(n, 0) + 1
    used = {n for n in raw if count[n] == 1}
    seen, names = {}, []
    for n in raw:
        if count[n] == 1:
            names.append(n)
            continue
        seen[n] = seen.get(n, 0) + 1
        nm = '%s %d' % (n, seen[n])
        while nm in used:
            nm += "'"
        used.add(nm)
        names.append(nm)
    return raw, names


def weigh(des):
    """Every part that has bodies weighed; the groups and the whole as sums of their parts."""
    root = des.rootComponent
    tops = list(root.occurrences)
    raw, names = _group_names(tops)
    index = {o.name: i for i, o in enumerate(tops)}
    groups = [dict(idx=i, name=names[i], raw=raw[i], recs=[], materials=set()) for i in range(len(tops))]
    leaves = []
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        segs = o.fullPathName.split('+')
        gi = index.get(segs[0])
        if gi is None:                                  # never expected: a part outside every top-level occurrence
            groups.append(dict(idx=len(groups), name='(outside) ' + segs[0], raw=segs[0], recs=[], materials=set()))
            index[segs[0]] = gi = len(groups) - 1
        m, c, I = _part(o)
        L = dict(name=o.component.name, segs=[s.rsplit(':', 1)[0] for s in segs], g=gi, m=m, c=c, I=I)
        leaves.append(L)
        groups[gi]['recs'].append(L)
        groups[gi]['materials'] |= _materials(o)
    if root.bRepBodies.count:                           # bodies in the root itself: a group of their own
        g = dict(idx=len(groups), name='(root bodies)', raw='(root bodies)', recs=[], materials=set())
        groups.append(g)
        for b in root.bRepBodies:
            m, c, I = _read(_pp(b))
            L = dict(name=b.name, segs=[b.name], g=g['idx'], m=m, c=c, I=I)
            leaves.append(L)
            g['recs'].append(L)
    origin = _about_origin(leaves)
    if not origin:                                      # moments came about each part's centre: move them to the axes
        for L in leaves:
            d2 = _d2(L['c'])
            L['I'] = tuple(L['I'][k] + L['m'] * d2[k] for k in range(3))
    for g in groups:
        g['sum'] = _sum(g['recs'])
    whole = None
    try:
        wm, wc, _ = _read(_pp(root, adsk.fusion.CalculationAccuracy.LowCalculationAccuracy))
        whole = dict(m=wm, c=[10.0 * v for v in wc])
    except Exception:
        pass
    return dict(leaves=leaves, groups=groups, total=_sum(leaves), whole=whole,
                moments='about the world axes, moved to the centre' if origin else
                        'about each part centre (Fusion), moved to the world axes and back to each centre')


# ---------- the robot's moving bodies ----------
def _norm(s):
    return ' '.join(s.lower().replace('leveling', 'levelling').split())


def _xz(v):
    """A point as (x, z) mm: a Point3D (cm), an (x, z) pair or an (x, y, z) triple (mm)."""
    if v is None:
        return None
    if hasattr(v, 'x') and hasattr(v, 'z') and not isinstance(v, (tuple, list)):
        p = (float(v.x) * 10.0, float(v.z) * 10.0)
    else:
        v = list(v)
        if len(v) == 2:
            p = (float(v[0]), float(v[1]))
        elif len(v) >= 3:
            p = (float(v[0]), float(v[2]))
        else:
            return None
    return p if all(math.isfinite(x) for x in p) else None


def _xz_of(H, k):
    try:
        if isinstance(H, dict):
            v = H.get(k, H.get(k.lower()))
            if v is None and (k + 'X') in H and (k + 'Z') in H:
                v = (H[k + 'X'], H[k + 'Z'])
        else:
            v = getattr(H, k, None)
        return _xz(v)
    except Exception:
        return None


def _modules(mod):
    """Where the robot's HOME points may be kept: its own script, the modules it imports, the palletizer's libraries."""
    out = [mod] if mod is not None else []
    for v in (list(vars(mod).values()) if mod is not None else []):
        if isinstance(v, types.ModuleType) and v not in out and not v.__name__.startswith('adsk'):
            out.append(v)
    for n, v in list(sys.modules.items()):
        if n.startswith('palletizer') and isinstance(v, types.ModuleType) and v not in out:
            out.append(v)
    return out


def _home(mod):
    mods = _modules(mod)
    for M in mods:
        for nm in ('HOME', 'HOME_POINTS', 'HOME_PTS'):
            H = getattr(M, nm, None)
            if H is None:
                continue
            if callable(H):
                try:
                    H = H()
                except Exception:
                    continue
            if _xz_of(H, 'A') is not None:
                return H, '%s.%s' % (getattr(M, '__name__', '?'), nm)
    lst = getattr(mod, '_last', None)
    if isinstance(lst, dict):
        for nm in ('HOME', 'home', 'points', 'pts'):
            H = lst.get(nm)
            if H is not None and _xz_of(H, 'A') is not None:
                return H, "_last['%s']" % nm
    for M in mods:                                      # last resort: the concept's points at q = 0
        f = getattr(M, 'pts', None)
        if callable(f):
            try:
                H = f(0.0, 0.0)
            except Exception:
                continue
            if _xz_of(H, 'A') is not None:
                return H, '%s.pts(0, 0)' % getattr(M, '__name__', '?')
    return None, 'none found'


def _num(mods, name):
    for M in mods:
        v = getattr(M, name, None)
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            return float(v), getattr(M, '__name__', '?')
    return None, None


def _pivots(mod):
    H, src = _home(mod)
    piv = {k: (_xz_of(H, k) if H is not None else None) for k in 'ABCEGW'}
    mods = _modules(mod)
    ax, sx = _num(mods, 'AX')
    az, sz = _num(mods, 'AZ')
    if ax is not None and az is not None:
        piv['AXZ'], src_t = (ax, az), '%s.AX, %s.AZ' % (sx, sz)
    elif isinstance(H, dict) and isinstance(H.get('AX'), (int, float)) and isinstance(H.get('AZ'), (int, float)):
        piv['AXZ'], src_t = (float(H['AX']), float(H['AZ'])), src + "['AX'], ['AZ']"
    else:
        piv['AXZ'], src_t = piv['A'], src + "['A'] (no AX, AZ found)"
    return piv, dict(points=src, turret=src_t)


def _body_of(raw):
    """The moving body a robot group belongs to, by the group's (component's) name; None for the base."""
    n = _norm(raw)
    if n == 'wrist' or n.startswith('wrist '):
        return 'wrist'
    if n.startswith(FLANGE_PARTS):
        return 'tool flange'
    if any(w in n for w in BALANCER_WORDS):
        return 'balancer'
    if n.startswith('turret'):
        return 'turret'
    if n.startswith('lower arm'):
        return 'lower arm'
    if n.startswith('crank') or n.startswith('counterweight'):
        return 'crank and counterweight'
    if n.startswith('parallel rod'):
        return 'parallel rod'
    if n.startswith('levelling rod'):
        return 'levelling rod'
    if 'triangle' in n:
        return 'triangle'
    if n.startswith('upper arm'):
        return 'upper arm'
    return None


def robot_bodies(mod, w):
    """The eleven moving bodies at q = 0, in palletizer_weighed.json's form, and the base apart."""
    owner = {}
    for g in w['groups']:
        b = _body_of(g['raw'])
        if b == 'levelling rod':
            mm = re.match(r'levelling rod\s*([12])\b', _norm(g['raw']))
            if mm:
                b = 'levelling rod ' + mm.group(1)
            else:                                       # by its line at q = 0: rod 1 stands up, rod 2 lies along x
                I = g['sum']['I']
                b = 'levelling rod 1' if I[2] <= I[0] else 'levelling rod 2'
        owner[g['idx']] = b
    members = {r[0]: [] for r in ROBOT}
    base = []
    which = {}
    for L in w['leaves']:
        b = owner.get(L['g'])
        if b == 'wrist':
            below = [_norm(s) for s in L['segs'][1:]] + [_norm(L['name'])]
            b = 'tool flange' if any(s.startswith(FLANGE_PARTS) for s in below) else 'wrist housing'
        which[id(L)] = b
    if not any(b == 'balancer' for b in which.values()):
        # no balancer group: its barrels and rods kept inside the turret's group or a group left to the base
        for L in w['leaves']:
            if which[id(L)] in (None, 'turret') and \
                    any(k in _norm(s) for s in L['segs'] + [L['name']] for k in BALANCER_WORDS):
                which[id(L)] = 'balancer'
    for L in w['leaves']:
        b = which[id(L)]
        (members[b] if b else base).append(L)
    piv, src = _pivots(mod)
    gname = {g['idx']: g['name'] for g in w['groups']}
    bodies, detail = [], {}
    for name, move, pv, kind in ROBOT:
        s = _sum(members[name])
        x, y, z = s['c']
        Ixx, Iyy, Izz = s['I']
        P = piv.get(pv) if s['m'] > 0.0 else None       # no parts found: no position either
        d = dict(name=name, m=round(s['m'], 3), move=move, y=round(y, 1))
        if kind == 'q2':
            d.update(c=round(z - P[1], 1) if P else None, Ia=round(Izz, 4), It=round(Iyy, 4), In=round(Ixx, 4))
        elif kind == 'q3':
            d.update(c=round(x - P[0], 1) if P else None, Ia=round(Ixx, 4), It=round(Iyy, 4), In=round(Izz, 4))
        else:
            d.update(off=[round(x - P[0], 1), round(z - P[1], 1)] if P else None, Izz=round(Izz, 4))
        if name == 'tool flange':
            d['spin'] = True
        bodies.append(d)
        detail[name] = dict(parts=s['parts'], centre=[round(v, 2) for v in s['c']],
                            I=[round(v, 5) for v in s['I']], pivot=pv, pivot_xz=list(P) if P else None,
                            groups=sorted({gname[L['g']] for L in members[name]}))
    b = _sum(base)
    return dict(base=round(b['m'], 3), bodies=bodies, bodies_detail=detail,
                base_groups=sorted({gname[L['g']] for L in base}), pivots={k: list(v) if v else None
                                                                            for k, v in piv.items()},
                pivots_from=src)


# ---------- the build ----------
def build(key='robot'):
    if key not in MACHINES:
        raise ValueError('No machine called %s: Chapter 57 weighs %s.' % (key, ', '.join(MACHINES)))
    t0 = time.time()
    RS = _range_sweep()
    msg, _setter = RS.machine(key)                      # the machine's own document, by its own script
    first = msg.split('\n')[0]
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    mod = RS._mods.get(key)
    if key == 'robot':                                  # q = 0, by the robot's own set_pose()
        G = mod._last['groups']
        mod.set_pose(G, 0.0, 0.0, 0.0)
        adsk.doEvents()
    w = weigh(des)
    tot = w['total']
    groups = {}
    for g in w['groups']:
        s = g['sum']
        groups[g['name']] = dict(m=s['m'], c=list(s['c']), I=list(s['I']), parts=s['parts'],
                                 materials=sorted(g['materials']))
    res = dict(machine=key, version=VERSION, machine_line=first,
               total=dict(m=tot['m'], c=tuple(tot['c']), I=list(tot['I']), parts=tot['parts']),
               groups=groups, fusion_whole=w['whole'], moments=w['moments'],
               units='kg, mm, kg m2 (moments about the centre along the world axes)')
    if key == 'robot':
        res.update(robot_bodies(mod, w))
        res['robot_version'] = getattr(mod, 'VERSION', None)
        res['pose'] = 'q = 0: set_pose(G, 0, 0, 0)'
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, 'mass_%s.json' % key)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=1)
    res['seconds'] = time.time() - t0
    _last.clear()
    _last.update(res)

    gs = sum(g['m'] for g in groups.values())
    line = ('Mass properties of the %s: %d parts, %.3f kg, centre (%.1f, %.1f, %.1f) mm; %d groups summing to %.3f kg.'
            % (key, tot['parts'], tot['m'], tot['c'][0], tot['c'][1], tot['c'][2], len(groups), gs))
    order = sorted(groups.items(), key=lambda kv: -kv[1]['m'])
    glist = ', '.join('%s %.3f kg' % (n, g['m']) for n, g in order[:25])
    if len(order) > 25:
        glist += ' and %d more' % (len(order) - 25)
    out = [line, 'Groups: %s.' % glist]
    if w['whole']:
        out.append('Fusion weighs the whole design at %.3f kg.' % w['whole']['m'])
    if key == 'robot':
        out.append('The robot at q = 0, its 11 moving bodies (palletizer_weighed.json form): %s; the base apart, %.3f kg. '
                   'Pivots from %s.' % (', '.join('%s %.1f kg' % (b['name'], b['m']) for b in res['bodies']),
                                        res['base'], res['pivots_from']['points']))
    out.append('Written to %s.' % os.path.relpath(path, CP.ROOT).replace('\\', '/'))
    out.append(first)
    return '\n'.join(out) + '\n\n[%s]' % VERSION


# ---------- the command dialog; the build runs from a custom event, outside the command ----------
_handlers = []


class _Built(adsk.core.CustomEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.additionalInfo or 'robot'))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('MassProperties failed:\n%s' % traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            key = args.command.commandInputs.itemById('machine').selectedItem.name
            adsk.core.Application.get().fireCustomEvent(EVENT_ID, key)
        except Exception:
            ui.messageBox('MassProperties failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('machine', 'Machine', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in MACHINES:
            dd.listItems.add(k, k == 'robot')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        try:
            app.unregisterCustomEvent(EVENT_ID)
        except Exception:
            pass
        ev = app.registerCustomEvent(EVENT_ID)
        hb = _Built()
        ev.add(hb)
        _handlers.append(hb)
        cid = 'mdfcMassProperties'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Mass properties',
                                                         'Chapter 57: the masses of the detailed assemblies')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('MassProperties failed:\n%s' % traceback.format_exc())


def stop(_context):
    try:
        adsk.core.Application.get().unregisterCustomEvent(EVENT_ID)
    except Exception:
        pass
