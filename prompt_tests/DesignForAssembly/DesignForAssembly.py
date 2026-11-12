# =============================================================================
# DesignForAssembly.py  -  Chapter 65 of "Mechanical Design in Autodesk Fusion with Claude": design for assembly.
# It builds the detailed bag gripper with its stops through RangeSweep.py's machine('bag'), puts every piece of its
# bill through Boothroyd and Dewhurst's test for unnecessary parts (lib/dfma.py, each question answered for a future
# redesign), writes each piece's answer on its occurrence and reads the count back from the model, then makes the
# three fixings the test found missing: the tines joined to their tubes; the end blocks joined to the end plates, the
# eight M8 x 30 between them removed and their holes filled, two M8 x 20 through the beam's side walls into each
# spigot; two M8 x 30 from inside each end plate into its cylinder bracket. The new screws are checked for
# interference, the bill is counted again, and everything goes to design_office/results/dfa.json.
# Written from design_office/DesignForAssembly/PROMPT.txt.
#
#   INPUTS        the dialog: the machine (only the bag gripper, bag, has its answers)
#   DERIVED       RangeSweep.machine('bag'); bom_detail (the bill); dfma (the answers, times, the three unfixed kinds);
#                 bag_gripper.LAY (the places of the fixings)
#   HARD RULES    the gripper built by RangeSweep.py, loaded; the counts read back from the attributes; no joints
#   CONVENTIONS   RangeSweep's new design document; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the check: answers read back, the fixings, the bill after, no interference, the mass
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import json
import os
import re
import sys
import time
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # noqa: E402  every lib/ folder on sys.path

import bag_gripper as B                                 # noqa: E402
import bom_detail as BD                                 # noqa: E402
import dfma as DF                                       # noqa: E402
import mdfc_parts as mp                                 # noqa: E402
importlib.reload(DF)
importlib.reload(BD)

VERSION = 'DesignForAssembly 1.0 (2026-11-12)'
ANSWERED = ('bag',)                    # the machines lib/dfma.py answers the test for
KEY = 'bag'
GRP = 'mdfc_dfa'                       # the attributes' group
RESULT = os.path.join(CP.ROOT, 'design_office', 'results', 'dfa.json')
ANSWERED_FOR = 'a future redesign (Dewhurst & Meeker, 2014, p. 5)'
_last = {}
_handlers = []

# ---------- the places of the fixings, typed from the gripper's layout (bag_gripper.LAY) and the prompt ----------
L = B.LAY
PLATE_X = L['plate_x']                                  # 410, 425: the +x end plate's inner and outer faces
BLOCK_X = (L['beam_x'] - L['block'], L['beam_x'])      # 380, 410: the end block in the beam's end, now the spigot
SPIGOT_X = L['beam_x'] - L['block'] / 2.0              # 395: the spigot's mid-depth
BEAM_ZMID = (L['beam'][0] + L['beam'][1]) / 2.0        # -66: the beam's mid-height
WALL_Y = (L['beam_y'] - L['beam_t'], L['beam_y'])      # 74, 80: a side wall's inner and outer faces
OLD_HOLES = ((50.0, -40.0), (-50.0, -40.0), (50.0, -92.0), (-50.0, -92.0))   # the M8 x 30's (y, z) in each plate
M8 = 8.0
SPIGOT_L, SPIGOT_TAP = 20.0, 20.0                       # ISO 4762 M8 x 20; tapped 20 deep from the wall's outer face
BRACKET_L, BRACKET_TAP = 30.0, 16.0                     # ISO 4762 M8 x 30; tapped 16 deep in the bracket
BRACKET_DY, BRACKET_PITCH = 42.5, 30.0                  # from the cylinder's pin along y; apart up the arm
PIN = B.cylinder_geometry()['pin']                     # (y, z) of the jaw cylinder's rear pin at the +x end
CLEAR_R = 4.5                                           # an M8 clearance hole, 9 mm, as RangeSweep's stop brackets
TAP_R = M8 / 2.0 + 0.001                                # a tapped hole drawn at the thread's size, as mp.nut's bore
COLOURS = dict(blue=((25, 85, 200), 'Paint - Enamel Glossy (Blue)'),
               red=((205, 35, 35), 'Paint - Enamel Glossy (Red)'),
               yellow=((245, 200, 20), 'Paint - Enamel Glossy (Yellow)'))
_OFF = [0.0, 0.0, 0.0]                                  # the gripper's frame against the world, from the beam's box
_NUM_CACHE = {}
_SUP = {}


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_dfa65_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---------- small helpers ----------
def _items(coll):
    return [coll.item(i) for i in range(coll.count)]


def _P(x, y, z):
    """A point of the gripper's layout in the world."""
    return (x + _OFF[0], y + _OFF[1], z + _OFF[2])


def _num(short):
    """The bill's number of a line of dfma.BAG, by its short name."""
    for l in DF.BAG:
        if l[2] == short:
            return l[0]
    raise KeyError('dfma.BAG has no line called %r' % short)


def _tube_number():
    """The weldment's tube, through which its pieces are answered (bom_detail.DETAIL)."""
    for start, num, desc, kind, ch in BD.DETAIL[KEY]:
        if start == 'Tube':
            return num
    raise KeyError('bom_detail names no tube for the bag gripper')


def _sup():
    if not _SUP:
        try:
            _SUP.update(BD.superseded())
        except Exception:
            pass
    return _SUP


def _number_of(name):
    """The bill's number an occurrence of this name takes (superseded numbers merged), None for a load, '?' when
    bom_detail cannot number it."""
    if name not in _NUM_CACHE:
        try:
            ln = BD.line_of(KEY, name)
            _NUM_CACHE[name] = None if ln is None else (_sup().get(ln[0], ln[0]) if ln[2] == 'bought' else ln[0])
        except KeyError:
            _NUM_CACHE[name] = '?'
    return _NUM_CACHE[name]


def _world(o):
    """An occurrence's transform to the world, through its parents (each native transform is to its parent)."""
    nat = o.nativeObject if o.nativeObject else o
    m = nat.transform2.copy()
    if o.assemblyContext:
        m.transformBy(_world(o.assemblyContext))
    return m


def _matname(m):
    try:
        return m.name if m else ''
    except Exception:
        return ''


def _mass_of(body):
    try:
        return body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.MediumCalculationAccuracy).mass
    except Exception:
        return body.physicalProperties.mass


def _bodies_world(o):
    """Each body of an occurrence as a temporary body in the world, with what it is made of."""
    W = _world(o)
    out = []
    for b in _items(o.component.bRepBodies):
        tb = mp.tbm().copy(b)
        mp.tbm().transform(tb, W)
        try:
            ap = b.appearance
        except Exception:
            ap = None
        out.append(dict(tb=tb, name=b.name, material=b.material, mat=_matname(b.material), appearance=ap,
                        volume=b.volume, attrs=[(a.groupName, a.name, a.value) for a in _items(b.attributes)]))
    return out


def _bbox(tbs):
    lo, hi = [1e18] * 3, [-1e18] * 3
    for tb in tbs:
        bb = tb.boundingBox
        for i, ax in enumerate('xyz'):
            lo[i] = min(lo[i], getattr(bb.minPoint, ax) * 10.0)
            hi[i] = max(hi[i], getattr(bb.maxPoint, ax) * 10.0)
    return lo, hi


def _occ(root, name):
    for o in root.allOccurrences:
        if o.component.name == name:
            return o
    raise RuntimeError('no occurrence of a component called %r' % name)


def _boolean(a, b, kind):
    try:
        return bool(mp.tbm().booleanOperation(a, b, kind))
    except Exception:
        return False


def _union(a, b):
    return _boolean(a, b, adsk.fusion.BooleanTypes.UnionBooleanType)


def _cut(a, b):
    return _boolean(a, b, adsk.fusion.BooleanTypes.DifferenceBooleanType)


def _frac_inside(tb, p0, p1, r):
    """How much of the cylinder p0-p1, radius r, lies inside the body (1.0: all of it)."""
    c = mp.cyl(p0, p1, r)
    v = c.volume
    t = mp.tbm().copy(tb)
    if not _boolean(t, c, adsk.fusion.BooleanTypes.IntersectionBooleanType) or v <= 0:
        return 0.0
    return t.volume / v


def _hole_radii(tbs, y, z):
    """The radii (mm) of the cylindrical faces along x through (y, z): the holes of the old M8 x 30."""
    out = []
    for tb in tbs:
        for f in tb.faces:
            c = adsk.core.Cylinder.cast(f.geometry)
            if not c:
                continue
            n = c.axis.length
            if n <= 0 or abs(c.axis.x) / n < 0.9999:
                continue
            if abs(c.origin.y * 10.0 - y) < 0.05 and abs(c.origin.z * 10.0 - z) < 0.05:
                out.append(round(c.radius * 10.0, 4))
    return sorted(set(out))


# ---------- the census and the bill (bom_detail) ----------
def _census(root):
    """Every occurrence of the gripper that has a body, the loads (the bag, the station's rollers) left out:
    (occurrences, [(name, mass kg, materials, volume cm3)], the names bom_detail cannot number)."""
    occs, census, unnumbered = [], [], []
    for o in root.allOccurrences:
        comp = o.component
        if comp.bRepBodies.count == 0:
            continue
        if any(p.startswith(BD.LOADS) for p in o.fullPathName.split('+')):
            continue
        num = _number_of(comp.name)
        if num is None and comp.name.startswith(BD.LOADS):
            continue
        if num == '?':
            unnumbered.append(comp.name)
            continue
        bodies = _items(comp.bRepBodies)
        census.append((comp.name, sum(_mass_of(b) for b in bodies),
                       ', '.join(sorted({_matname(b.material) for b in bodies if b.material})),
                       sum(b.volume for b in bodies)))
        occs.append(o)
    return occs, census, unnumbered


def _bill(census):
    b = BD.bom(KEY, census)
    try:
        return BD.finish(b)
    except Exception as e:                              # the makers' masses unread: merge the numbers only
        sup = _sup()
        merged, lines = {}, []
        for g in b['lines']:
            if g['kind'] == 'bought' and g['number'] in sup:
                g = dict(g, number=sup[g['number']])
            k = g['number']
            if k is not None and g['kind'] == 'bought' and k in merged:
                m = merged[k]
                m['qty'] += g['qty']
                m['mass'] += g['mass']
                m['with'] = m.get('with', []) + g.get('with', [])
                for kk, v in g['names'].items():
                    m['names'].setdefault(kk, []).extend(v)
                continue
            if k is not None:
                merged[k] = g
            lines.append(g)
        b['lines'] = lines
        b['finish_error'] = str(e)
        return b


def _line_names(g):
    out = []
    for lst in g['names'].values():
        for n, _piece in lst:
            if n not in out:
                out.append(n)
    return out


def _pieces_of(g, lines, tube):
    """The occurrence names that stand for the pieces of a first-level line: every one of a made part, one body kind
    of each set of a bought part (a bearing's outer rings, not its balls as well), a weldment's tubes."""
    if g['number'] in BD.WELDMENTS:
        ins = [x for x in lines if x['kind'] == 'in ' + g['number'] and x['number']]
        carrier = [x for x in ins if x['number'] == tube] or [x for x in ins if x['qty'] == g['qty']]
        return sorted(_line_names(carrier[0])) if carrier else []
    if g['kind'] == 'bought':
        out = []
        for setn in sorted(g['names']):
            kinds = {}
            for n, piece in g['names'][setn]:
                kinds.setdefault(piece, [])
                if n not in kinds[piece]:
                    kinds[piece].append(n)
            best = max(sorted(kinds), key=lambda p: (len(kinds[p]), p == ''))
            out += sorted(kinds[best])
        return out
    return sorted(_line_names(g))


def _by_number(lines):
    """{number: [occurrence names]} over every line (first level, a weldment's pieces, the bodies with a bought part)."""
    out = {}
    for g in lines:
        if g['number']:
            v = out.setdefault(g['number'], [])
            for n in _line_names(g):
                if n not in v:
                    v.append(n)
        for w in g.get('with', []):
            out.setdefault(w['parent'], []).append(w['name'])
    return out


def _pieces_in_bill(lines):
    return sum(g['qty'] for g in BD.top(lines) if g['number'] and g['kind'] in ('made', 'bought'))


# ---------- the answers on the occurrences, and read back ----------
def _answer(nat, line, essential, extra=None):
    num, n, short, grp, ess, q, why, op, basis = line
    h = DF.TIMES[op]
    vals = dict(number=num, name=short, group=grp, essential='1' if essential else '0',
                question=(q if essential and q else 'none'), why=why, op=op, time='%.4f' % DF.op_time(op),
                handling='%s %.2f s' % (h[0], h[1]), insertion='%s %.2f s' % (h[2], h[3]) if h[2] else 'none 0.00 s',
                basis=basis, answered=ANSWERED_FOR)
    vals.update(extra or {})
    for k, v in vals.items():
        nat.attributes.add(GRP, k, str(v))


def _tag(bill, occ_of):
    """Every piece of every first-level line its answer, as attributes of its occurrence."""
    answers = {l[0]: l for l in DF.BAG}
    tube = _tube_number()
    lines = bill['lines']
    tagged, rows, unanswered, short = 0, [], [], []
    seen = set()
    for g in BD.top(lines):
        num = g['number']
        if not num or g['kind'] not in ('made', 'bought'):
            continue
        if num not in answers:
            unanswered.append(num)
            continue
        seen.add(num)
        ln = answers[num]
        names = _pieces_of(g, lines, tube)
        if len(names) != g['qty'] or g['qty'] != ln[1]:
            short.append(dict(number=num, pieces=len(names), bill=g['qty'], test=ln[1]))
        k = 0
        for nm in names:
            for o in occ_of.get(nm, []):
                nat = o.nativeObject if o.nativeObject else o
                extra = {'through': 'its tube'} if num in BD.WELDMENTS else None
                _answer(nat, ln, k < ln[4], extra)
                rows.append(dict(occurrence=nm, number=num, essential=k < ln[4]))
                k += 1
                tagged += 1
    missing = [l[0] for l in DF.BAG if l[0] not in seen]
    return dict(tagged=tagged, rows=rows, unanswered=unanswered, missing=missing, mismatched=short)


def _read_back(root):
    """The test as the model holds it: each occurrence's attributes, counted."""
    pieces = []
    for o in root.allOccurrences:
        nat = o.nativeObject if o.nativeObject else o
        at = nat.attributes
        a = at.itemByName(GRP, 'number')
        if a is None:
            continue

        def get(k, d=''):
            x = at.itemByName(GRP, k)
            return x.value if x else d
        pieces.append(dict(occurrence=o.component.name, number=a.value, essential=int(get('essential', '0')),
                           question=get('question', 'none'), op=get('op'), time=float(get('time', '0')),
                           new=get('new', '')))
    N = len(pieces)
    Nmin = sum(p['essential'] for p in pieces)
    by_q, per = {}, {}
    for p in pieces:
        per[p['number']] = per.get(p['number'], 0) + 1
        if p['essential'] and p['question'] != 'none':
            by_q[p['question']] = by_q.get(p['question'], 0) + 1
    t = sum(p['time'] for p in pieces)
    return dict(N=N, Nmin=Nmin, by_q=by_q, t=t, E=DF.T0 * Nmin / t if t else 0.0, lines=sorted(per), per_line=per,
                pieces=pieces)


# ---------- rebuilding a part with new bodies ----------
def _add_body(comp, tb):
    if comp.parentDesign.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        comp.bRepBodies.add(tb, bf)
        bf.finishEdit()
        return bf.bodies.item(bf.bodies.count - 1)
    return comp.bRepBodies.add(tb)


def _put_bodies(comp, bodies, Wi):
    """World bodies into a component whose world transform is the inverse of Wi."""
    des = comp.parentDesign
    for b in bodies:
        tb = b['tb']
        mp.tbm().transform(tb, Wi)
        nb = _add_body(comp, tb)
        try:
            nb.name = b['name']
        except Exception:
            pass
        done = False
        if b.get('material'):
            try:
                nb.material = b['material']
                done = True
            except Exception:
                pass
        if not done and b.get('mat'):
            m = des.materials.itemByName(b['mat'])
            if m:
                nb.material = m
            else:
                mp.set_material(nb, b['mat'])
        if b.get('appearance'):
            try:
                nb.appearance = b['appearance']
            except Exception:
                pass
        for g, n, v in b.get('attrs', []):
            nb.attributes.add(g, n, v)


def _recreate(root, name, bodies):
    """The part `name` rebuilt in its place with the given world bodies: same parent, transform, name, description,
    part number and attributes (its occurrence deleted and made again: no body is left behind)."""
    o = _occ(root, name)
    if o.childOccurrences.count:
        raise RuntimeError('%s holds other parts and cannot be rebuilt' % name)
    nat = o.nativeObject if o.nativeObject else o
    par = o.assemblyContext
    pcomp = par.component if par else root
    T = nat.transform2.copy()
    Wi = _world(o)
    Wi.invert()
    comp = o.component
    keep = {}
    for k in ('description', 'partNumber'):
        try:
            keep[k] = getattr(comp, k)
        except Exception:
            pass
    oattrs = [(a.groupName, a.name, a.value) for a in _items(nat.attributes)]
    cattrs = [(a.groupName, a.name, a.value) for a in _items(comp.attributes)]
    try:
        grounded = nat.isGroundToParent
    except Exception:
        grounded = None
    nat.deleteMe()
    new = pcomp.occurrences.addNewComponent(T)
    new.component.name = name
    for k, v in keep.items():
        try:
            setattr(new.component, k, v)
        except Exception:
            pass
    if pcomp != root and grounded is not None:
        try:
            new.isGroundToParent = grounded
        except Exception:
            pass
    _put_bodies(new.component, bodies, Wi)
    for g, n, v in oattrs:
        new.attributes.add(g, n, v)
    for g, n, v in cattrs:
        new.component.attributes.add(g, n, v)
    return new.component.name


def _delete(root, name):
    o = _occ(root, name)
    (o.nativeObject if o.nativeObject else o).deleteMe()


def _add_part(root, pcomp, Wp, name, body):
    """A new part in pcomp (whose world transform is Wp), its body given in the world."""
    new = pcomp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    new.component.name = name
    if pcomp != root:
        try:
            new.isGroundToParent = True
        except Exception:
            pass
    Wi = Wp.copy()
    Wi.invert()
    _put_bodies(new.component, [body], Wi)
    return new


def _stems(names):
    """The starts of the existing names, the one ending in the size (M8x20) first, then the longest first."""
    out = []
    for n in names:
        words = re.sub(r' \(\d+\)$', '', n).split()
        for k in range(len(words), 1, -1):
            s = re.sub(r'\s+\d+$', '', ' '.join(words[:k])).strip()
            if s and s not in out:
                out.append(s)
    return sorted(out, key=lambda s: (not re.search(r'M\d+x\d+$', s), -len(s)))


def _new_name(des, number, existing, tag, k, used):
    """A name for a new screw that bom_detail numbers as `number`: the existing screws' name with the fixing's tag,
    else their numbering carried on, else an existing name (Fusion then adds its own count)."""
    stems = _stems(existing)
    idx = [int(m.group(1)) for m in (re.search(r'(\d+)$', re.sub(r' \(\d+\)$', '', n)) for n in existing) if m]
    n0 = max(idx) if idx else 0
    cands = ['%s %s %d' % (s, tag, k) for s in stems] + ['%s %d' % (s, n0 + k) for s in stems]
    taken = {c.name for c in des.allComponents} | set(used)
    for c in cands:
        if c not in taken and _number_of(c) == number:
            return c
    return existing[0] if existing else '%s %d' % (tag, k)


def _join(main_tb, main_mat, others):
    """Others unioned into the main body where they are of its material; the rest kept as bodies beside it."""
    extras = []
    for b in others:
        if b['mat'] == main_mat:
            trial = mp.tbm().copy(main_tb)
            if _union(trial, b['tb']):
                main_tb = trial
                continue
        extras.append(b)
    return main_tb, extras


def _main(bodies):
    i = max(range(len(bodies)), key=lambda k: bodies[k]['volume'])
    return bodies[i], [b for k, b in enumerate(bodies) if k != i]


# ---------- the three fixings ----------
def _bracket_place(br_tb, pl_tb, s):
    """Where the two M8 x 30 go into one cylinder bracket's root: 42.5 from the cylinder's pin along y, 30 apart up
    the arm. Which side of the pin the root lies on, and the pair's height, the prompt leaves open: the typed places
    are tried first (the root beyond the pin, then before it, the pair centred on the pin's height), then the same
    sides centred on the root's height as the bracket is drawn; the first whose tapped holes lie wholly in the bracket
    and whose clearance holes lie wholly in the plate is taken."""
    py, pz = s * PIN[0], PIN[1]
    sg = 1.0 if py > 0 else -1.0
    cands = [('typed: the root beyond the pin, the pair at the pin\'s height', py + sg * BRACKET_DY, pz),
             ('typed: the root inside the pin, the pair at the pin\'s height', py - sg * BRACKET_DY, pz)]
    x0, x1 = sorted((s * PLATE_X[1] + _OFF[0], s * (PLATE_X[1] + 2.0) + _OFF[0]))
    sec = mp.tbm().copy(br_tb)
    root_bb = None
    if _boolean(sec, mp.box(x0, x1, -5000.0, 5000.0, -5000.0, 5000.0),
                adsk.fusion.BooleanTypes.IntersectionBooleanType) and sec.volume > 1e-6:
        lo, hi = _bbox([sec])
        root_bb = ([lo[i] - _OFF[i] for i in range(3)], [hi[i] - _OFF[i] for i in range(3)])
        zr = (root_bb[0][2] + root_bb[1][2]) / 2.0
        yr = (root_bb[0][1] + root_bb[1][1]) / 2.0
        cands += [('the root beyond the pin, the pair at the root\'s mid-height', py + sg * BRACKET_DY, zr),
                  ('the root inside the pin, the pair at the root\'s mid-height', py - sg * BRACKET_DY, zr),
                  ('the root\'s middle as drawn', yr, zr)]
    tried = []
    for label, y, zc in cands:
        zs = (zc + BRACKET_PITCH / 2.0, zc - BRACKET_PITCH / 2.0)
        f_br = [_frac_inside(br_tb, _P(s * PLATE_X[1], y, z), _P(s * (PLATE_X[1] + BRACKET_TAP), y, z), M8 / 2.0)
                for z in zs]
        f_pl = [_frac_inside(pl_tb, _P(s * PLATE_X[0], y, z), _P(s * PLATE_X[1], y, z), CLEAR_R) for z in zs]
        row = dict(label=label, y=y, z=list(zs), in_bracket=f_br, in_plate=f_pl)
        tried.append(row)
        if min(f_br) > 0.999 and min(f_pl) > 0.999:
            return dict(row, ok=True, tried=tried, root=root_bb)
    return dict(tried[0], ok=False, tried=tried, root=root_bb)


def _fix(des, root, bill):
    TINE, BLOCK, BRACKET = [u[0] for u in DF.UNFIXED]
    PLATE, BEAM = _num('end plate'), _num('beam')
    M830, M820 = _num('cap screw M8 x 30'), _num('cap screw M8 x 20')
    TUBE = _tube_number()
    count = {l[0]: l[1] for l in DF.BAG}
    nn = _by_number(bill['lines'])
    notes = []

    def names(num, k):
        v = sorted(nn.get(num, []))
        if len(v) != k:
            notes.append('the bill has %d of %s, the fixing expects %d' % (len(v), num, k))
        return v

    def grab(n):
        o = _occ(root, n)
        bw = _bodies_world(o)
        lo, hi = _bbox([b['tb'] for b in bw])
        par = o.assemblyContext
        return dict(name=n, bodies=bw, lo=lo, hi=hi, c=[(lo[i] + hi[i]) / 2.0 for i in range(3)],
                    parent=par.component if par else root,
                    Wp=_world(par) if par else adsk.core.Matrix3D.create())

    # the gripper's frame: the beam's box against the layout (its middle in x and y, its top on the adapter)
    beam = grab(names(BEAM, 1)[0])
    off = [beam['c'][0], beam['c'][1], beam['hi'][2] - L['beam'][0]]
    _OFF[:] = off if max(abs(v) for v in off) >= 0.5 else [0.0, 0.0, 0.0]
    size = [beam['hi'][i] - beam['lo'][i] for i in range(3)]
    size_exp = [2 * L['beam_x'], 2 * L['beam_y'], L['beam'][0] - L['beam'][1]]
    if max(abs(size[i] - size_exp[i]) for i in range(3)) > 0.5:
        notes.append('the beam measures %s mm, the layout %s' % ([round(v, 2) for v in size], size_exp))
    if any(_OFF):
        notes.append('the gripper stands %s mm from the world origin; the places are moved with it'
                     % [round(v, 2) for v in _OFF])
    side = lambda g: 1 if g['c'][0] - _OFF[0] > 0 else -1

    # 1. the tines of each jaw joined to its tube
    TB = [grab(n) for n in names(TUBE, count[_num('jaw weldment')])]
    TI = [grab(n) for n in names(TINE, count[TINE])]
    of_tube = {t['name']: [] for t in TB}
    for ti in TI:
        best = min(TB, key=lambda t: sum((t['c'][i] - ti['c'][i]) ** 2 for i in range(3)))
        of_tube[best['name']].append(ti)
    tube_new, joins = {}, []
    for t in TB:
        main, rest = _main(t['bodies'])
        tb, extra = _join(mp.tbm().copy(main['tb']), main['mat'], [b for ti in of_tube[t['name']] for b in ti['bodies']])
        tube_new[t['name']] = [dict(main, tb=tb)] + rest + extra
        joins.append(dict(tube=t['name'], tines=sorted(ti['name'] for ti in of_tube[t['name']]),
                          kept_as_bodies=[b['name'] for b in extra]))

    # the parts at each end: end plate, end block, the old M8 x 30, the cylinder bracket
    PL = [grab(n) for n in names(PLATE, count[PLATE])]
    BL = [grab(n) for n in names(BLOCK, count[BLOCK])]
    BR = [grab(n) for n in names(BRACKET, count[BRACKET])]
    S30 = [grab(n) for n in names(M830, count[M830])]
    S20 = [grab(n) for n in sorted(nn.get(M820, []))]
    mat20 = S20[0]['bodies'][0] if S20 else dict(material=None, mat='Steel')
    mat30 = S30[0]['bodies'][0] if S30 else mat20
    ends = {}
    for s in (1, -1):
        e = dict(plate=[p for p in PL if side(p) == s], blocks=[b for b in BL if side(b) == s],
                 brackets=[b for b in BR if side(b) == s], screws=[m for m in S30 if side(m) == s])
        if len(e['plate']) != 1:
            notes.append('%d end plates found at the %s end' % (len(e['plate']), '+x' if s > 0 else '-x'))
        ends[s] = e

    # 3b first: where the bracket screws go (the plate's clearance holes need them)
    places = {}
    for s, e in ends.items():
        if not e['plate']:
            continue
        pm, _r = _main(e['plate'][0]['bodies'])
        places[s] = []
        for br in e['brackets']:
            bm, _r2 = _main(br['bodies'])
            pl = _bracket_place(bm['tb'], pm['tb'], s)
            pl.update(bracket=br['name'], end=s)
            places[s].append(pl)
            if not pl['ok']:
                notes.append('%s: no tried place holds both tapped holes in the bracket and both clearance holes in '
                             'the plate; the first typed place is used' % br['name'])

    # 2. each end block joined to its end plate, the old holes filled, the spigot tapped, the bracket holes cleared
    plate_new, filled, old_at = {}, [], []
    for s, e in ends.items():
        if not e['plate']:
            continue
        p = e['plate'][0]
        main, rest = _main(p['bodies'])
        blk_bodies = [b for x in e['blocks'] for b in x['bodies']]
        tb, extra = _join(mp.tbm().copy(main['tb']), main['mat'], blk_bodies)
        for m in e['screws']:
            old_at.append(dict(screw=m['name'], end=s, y=round(m['c'][1] - _OFF[1], 2), z=round(m['c'][2] - _OFF[2], 2)))
        x_fill = (s * PLATE_X[1], s * (BLOCK_X[0] if not extra else PLATE_X[0]))
        for y, z in OLD_HOLES:
            q = _P(0.0, y, z)
            radii = _hole_radii([main['tb']] + [b['tb'] for b in blk_bodies], q[1], q[2])
            if not radii:
                notes.append('no hole found in %s at y %+.0f, z %.0f: the fill adds nothing there' % (p['name'], y, z))
            r = min(max(radii + [CLEAR_R]) + 0.05, 9.0)
            ok = _union(tb, mp.cyl(_P(x_fill[0], y, z), _P(x_fill[1], y, z), r))
            for b in extra:                                 # a block of another material keeps its own body
                _union(b['tb'], mp.cyl(_P(s * BLOCK_X[1], y, z), _P(s * BLOCK_X[0], y, z), r))
            filled.append(dict(end=s, y=y, z=z, holes_found=radii, fill_r=r, ok=ok))
        for w in (1, -1):                                   # the spigot tapped 20 deep from the wall's outer face
            c = lambda: mp.cyl(_P(s * SPIGOT_X, w * (WALL_Y[1] + 1.0), BEAM_ZMID),
                               _P(s * SPIGOT_X, w * (WALL_Y[1] - SPIGOT_TAP), BEAM_ZMID), TAP_R)
            if not _cut(tb, c()):
                notes.append('the spigot tap at the %+d end, y %+d failed' % (s, w))
            for b in extra:
                _cut(b['tb'], c())
        for pl in places.get(s, []):                       # the plate's clearance holes for the bracket screws
            for z in pl['z']:
                if not _cut(tb, mp.cyl(_P(s * (PLATE_X[0] - 1.0), pl['y'], z), _P(s * (PLATE_X[1] + 1.0), pl['y'], z),
                                       CLEAR_R)):
                    notes.append('a clearance hole in %s failed' % p['name'])
        plate_new[p['name']] = [dict(main, tb=tb)] + rest + extra

    # 3a. the beam's side walls: clearance holes for the spigot screws
    bm, brest = _main(beam['bodies'])
    btb = mp.tbm().copy(bm['tb'])
    for s in (1, -1):
        for w in (1, -1):
            if not _cut(btb, mp.cyl(_P(s * SPIGOT_X, w * (WALL_Y[1] + 1.0), BEAM_ZMID),
                                    _P(s * SPIGOT_X, w * (WALL_Y[0] - 1.0), BEAM_ZMID), CLEAR_R)):
                notes.append('a clearance hole in the beam failed')
    beam_new = [dict(bm, tb=btb)] + brest

    # 3b. the brackets tapped 16 deep from the plate's face
    bracket_new = {}
    for s, pls in places.items():
        for pl in pls:
            br = [b for b in BR if b['name'] == pl['bracket']][0]
            m, rest = _main(br['bodies'])
            tb = mp.tbm().copy(m['tb'])
            for z in pl['z']:
                if not _cut(tb, mp.cyl(_P(s * (PLATE_X[1] - 1.0), pl['y'], z),
                                       _P(s * (PLATE_X[1] + BRACKET_TAP), pl['y'], z), TAP_R)):
                    notes.append('a tapped hole in %s failed' % br['name'])
            bracket_new[br['name']] = [dict(m, tb=tb)] + rest

    # the new screws, in the world
    screws = []
    for s in (1, -1):
        for w in (1, -1):
            at, ax = _P(s * SPIGOT_X, w * WALL_Y[1], BEAM_ZMID), (0.0, -float(w), 0.0)
            screws.append(dict(number=M820, tag='spigot', L=SPIGOT_L, at=at, axis=ax, tb=mp.cap_screw(M8, SPIGOT_L, at, ax),
                               parent=beam['parent'], Wp=beam['Wp'], like=mat20,
                               why='holds the spigot in the beam: through the side wall, tapped 20 deep'))
    for s, pls in places.items():
        for pl in pls:
            br = [b for b in BR if b['name'] == pl['bracket']][0]
            for z in pl['z']:
                at, ax = _P(s * PLATE_X[0], pl['y'], z), (float(s), 0.0, 0.0)
                screws.append(dict(number=M830, tag='bracket', L=BRACKET_L, at=at, axis=ax,
                                   tb=mp.cap_screw(M8, BRACKET_L, at, ax), parent=br['parent'], Wp=br['Wp'], like=mat30,
                                   why='holds the cylinder bracket from inside the end plate, tapped 16 deep'))

    # ---- into the model ----
    n_tines = n_blocks = 0
    rebuilt = []
    for t in TB:
        rebuilt.append(_recreate(root, t['name'], tube_new[t['name']]))
        for ti in of_tube[t['name']]:
            _delete(root, ti['name'])
            n_tines += 1
    removed = []
    for s, e in ends.items():
        if not e['plate']:
            continue
        rebuilt.append(_recreate(root, e['plate'][0]['name'], plate_new[e['plate'][0]['name']]))
        for b in e['blocks']:
            _delete(root, b['name'])
            n_blocks += 1
    for m in S30:                                       # the eight M8 x 30 between plates and blocks
        _delete(root, m['name'])
        removed.append(m['name'])
    rebuilt.append(_recreate(root, beam['name'], beam_new))
    for nm, bodies in bracket_new.items():
        rebuilt.append(_recreate(root, nm, bodies))
    added, added_rows = [], []
    answers = {l[0]: l for l in DF.BAG}
    k_tag = {}
    for sc in screws:
        k_tag[sc['tag']] = k_tag.get(sc['tag'], 0) + 1
        existing = [n for n in nn.get(sc['number'], [])] or [m['name'] for m in (S20 if sc['number'] == M820 else S30)]
        name = _new_name(des, sc['number'], existing, sc['tag'], k_tag[sc['tag']], added)
        occ = _add_part(root, sc['parent'], sc['Wp'], name,
                        dict(tb=sc['tb'], name=name, material=sc['like'].get('material'), mat=sc['like'].get('mat', 'Steel'),
                             appearance=None, attrs=[]))
        name = occ.component.name
        ln = answers[sc['number']]
        _answer(occ, ln, False, dict(new='1', why=sc['why']))
        added.append(name)
        added_rows.append(dict(name=name, number=_number_of(name), L=sc['L'], at=[round(v, 3) for v in sc['at']],
                               axis=list(sc['axis'])))
    return dict(tines=n_tines, blocks=n_blocks, removed=sorted(removed), added=added, added_rows=added_rows,
                joins=joins, filled=filled, old_screws_at=old_at, bracket_places=[
                    dict((k, v) for k, v in pl.items() if k != 'root') for s in places for pl in places[s]],
                rebuilt=rebuilt, offset=list(_OFF), notes=notes,
                places=dict(spigot_x=SPIGOT_X, beam_z=BEAM_ZMID, wall_y=list(WALL_Y), spigot_tap=SPIGOT_TAP,
                            plate_x=list(PLATE_X), block_x=list(BLOCK_X), pin=list(PIN), pin_x=L['cyl_x'],
                            bracket_dy=BRACKET_DY, bracket_pitch=BRACKET_PITCH, bracket_tap=BRACKET_TAP,
                            clearance_d=2 * CLEAR_R, old_holes=[list(h) for h in OLD_HOLES]))


# ---------- interference, colours ----------
def _owner(e):
    o = adsk.fusion.Occurrence.cast(e)
    if o:
        return o.component.name
    b = adsk.fusion.BRepBody.cast(e)
    if b:
        return b.assemblyContext.component.name if b.assemblyContext else b.parentComponent.name
    return getattr(e, 'name', '?')


def _interferences(des, occs):
    col = adsk.core.ObjectCollection.create()
    for o in occs:
        col.add(o)
    inp = des.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = des.analyzeInterference(inp)
    out = []
    if res:
        for r in res:
            try:
                v = round(r.interferenceBody.volume * 1000.0, 3)
            except Exception:
                v = None
            out.append((_owner(r.entityOne), _owner(r.entityTwo), v))
    return out


def _appearance(des, key):
    rgb, base_name = COLOURS[key]
    nm = 'DFA %s' % key
    ap = des.appearances.itemByName(nm)
    if ap:
        return ap
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
    base = None
    for n in (base_name, 'Paint - Enamel Glossy (White)', 'Plastic - Glossy (White)'):
        try:
            base = lib.appearances.itemByName(n)
        except Exception:
            base = None
        if base:
            break
    if base is None:
        for i in range(min(lib.appearances.count, 300)):
            a = lib.appearances.item(i)
            if any(adsk.core.ColorProperty.cast(p) for p in _items(a.appearanceProperties)):
                base = a
                break
    ap = des.appearances.addByCopy(base, nm)
    col = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
    props = [p for p in _items(ap.appearanceProperties) if adsk.core.ColorProperty.cast(p)]
    main = [p for p in props if p.id in ('opaque_albedo', 'generic_diffuse', 'metal_f0', 'layered_diffuse',
                                          'transparent_color', 'surface_albedo')] or props[:1]
    for p in main:
        try:
            adsk.core.ColorProperty.cast(p).value = col
        except Exception:
            pass
    return ap


def _colour(des, root, bill, occ_of, added):
    """Blue the essential, red the candidates, yellow the kinds the test found with no fixing and the new screws."""
    answers = {l[0]: l for l in DF.BAG}
    unfixed = {u[0] for u in DF.UNFIXED}
    aps = {}
    for k in COLOURS:
        try:
            aps[k] = _appearance(des, k)
        except Exception:
            aps[k] = None
    done = {}

    def paint(nm, parent):
        ln = answers.get(parent)
        for o in occ_of.get(nm, []):
            nat = o.nativeObject if o.nativeObject else o
            a = nat.attributes.itemByName(GRP, 'essential')
            if parent in unfixed or nm in added:
                key = 'yellow'
            elif a is not None:
                key = 'blue' if a.value == '1' else 'red'
            else:
                key = 'blue' if ln and ln[4] > 0 else 'red'
            done[key] = done.get(key, 0) + 1
            if aps.get(key) is None:
                continue
            try:
                o.appearance = aps[key]
            except Exception:
                try:
                    nat.appearance = aps[key]
                except Exception:
                    pass
    for g in bill['lines']:
        parent = g['kind'][3:] if g['kind'].startswith(('in ', 'with ')) else g['number']
        if g['kind'].startswith('with '):
            parent = g['kind'][5:]
        for nm in _line_names(g):
            paint(nm, parent)
        for w in g.get('with', []):
            paint(w['name'], w['parent'])
    return done


# ---------- the whole job ----------
def build(key='bag'):
    if key not in ANSWERED:
        raise ValueError('No answers for %s: the test for unnecessary parts is answered only for the bag gripper '
                         '(%s).' % (key, ', '.join(ANSWERED)))
    t0 = time.time()
    rs = _load('RangeSweep')
    msg0, _setter = rs.machine(key)                     # the detailed bag gripper with its stops, in its new design
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent

    # the bill as the model has it, each piece answered on its occurrence, the count read back
    occs0, census0, unnumbered0 = _census(root)
    bill0 = _bill(census0)
    occ_of = {}
    for o, c in zip(occs0, census0):
        occ_of.setdefault(c[0], []).append(o)
    tag = _tag(bill0, occ_of)
    before = _read_back(root)
    mass_before = sum(c[1] for c in census0)
    lib = DF.bag_test()

    # the three fixings
    fixed = _fix(des, root, bill0)

    # the bill again
    occs1, census1, unnumbered1 = _census(root)
    bill1 = _bill(census1)
    lines_after = {}
    for g in BD.top(bill1['lines']):
        if g['number'] and g['kind'] in ('made', 'bought'):
            lines_after[g['number']] = lines_after.get(g['number'], 0) + g['qty']
    pieces_after = _pieces_in_bill(bill1['lines'])
    after = _read_back(root)
    mass_after = sum(c[1] for c in census1)

    # the new screws against every other piece of the gripper (the bag and the station are not in the census)
    hits = _interferences(des, occs1)
    new = set(fixed['added'])
    inter = [h for h in hits if h[0] in new or h[1] in new]

    occ_of1 = {}
    for o, c in zip(occs1, census1):
        occ_of1.setdefault(c[0], []).append(o)
    try:
        painted = _colour(des, root, bill1, occ_of1, new)
    except Exception as e:
        painted = 'not coloured: %s' % e
    answers = [dict(number=l[0], pieces=l[1], name=l[2], group=l[3], essential=l[4], question=l[5], why=l[6],
                    op=l[7], time=DF.op_time(l[7]), codes=list(DF.TIMES[l[7]][:4]), basis=l[8]) for l in DF.BAG]

    redesign = DF.redesign()
    calc = {}
    for nm, f in (('tine_weld', DF.tine_weld), ('end_spigot', DF.end_spigot), ('bracket_screws', DF.bracket_screws)):
        try:
            calc[nm] = f()
        except Exception as e:
            calc[nm] = 'not computed: %s' % e

    r = dict(version=VERSION, machine=key, answered_for=ANSWERED_FOR,
             before=dict((k, v) for k, v in before.items() if k != 'pieces'), pieces=before['pieces'],
             tagged=tag['tagged'], lines_before=before['lines'], unanswered=tag['unanswered'], missing=tag['missing'],
             mismatched=tag['mismatched'], pieces_in_bill_before=_pieces_in_bill(bill0['lines']),
             lib=dict(N=lib['N'], Nmin=lib['Nmin'], by_q=lib['by_q'], t=lib['t'], E=lib['E'], by_group=lib['by_group']),
             answers=answers, unfixed=[list(u) for u in DF.UNFIXED], fixed=fixed, lines_after=lines_after,
             pieces_after=pieces_after,
             after=dict((k, v) for k, v in after.items() if k != 'pieces'),
             redesign=dict(N=redesign['N'], Nmin=redesign['Nmin'], t=redesign['t'], E=redesign['E']),
             interference=inter, interferences_all=len(hits), interferences_list=hits[:50],
             mass_before=mass_before, mass_after=mass_after, painted=painted,
             unnumbered=sorted(set(unnumbered0)), calculations=calc, seconds=time.time() - t0,
             built=msg0.split('\n')[0] if msg0 else '')
    try:
        os.makedirs(os.path.dirname(RESULT), exist_ok=True)
        with open(RESULT, 'w', encoding='utf-8') as f:
            json.dump(r, f, indent=1, default=lambda o: list(o) if isinstance(o, (set, tuple)) else str(o))
        r['written'] = RESULT
    except OSError as e:
        r['written'] = 'not written: %s' % e
    _last.clear()
    _last.update(r)

    b = before
    q = b['by_q']
    msg = ('Design for assembly of the bag gripper, each question answered for %s.\n'
           'Pieces answered on their occurrences: %d (%d lines of the bill); essential, read back: %d (base %d, '
           'move %d, separate %d); candidates for elimination: %d. Assembly time %.1f s (Matuszek et al., 2021, '
           'Table 2); E = 3 x %d / %.1f s = %.4f.\n'
           'Fixings: %d tines joined to their tubes; %d end blocks joined to the end plates, the %d M8 x 30 between '
           'them removed (%s) and their holes filled; added %d ISO 4762 M8 x 20 through the beam\'s side walls into '
           'the spigots and %d ISO 4762 M8 x 30 from inside the end plates into the cylinder brackets: %s.\n'
           'After the fixings: %d pieces in the bill, M8 x 20 %s, M8 x 30 %s, %s.\n'
           'New screws\' interference: %s (%d interferences among all the gripper\'s pieces). '
           'Mass %.3f kg before, %.3f kg after.\n'
           'Blue: essential; red: candidates; yellow: the kinds that had no fixing and the new screws.%s\n\n[%s]'
           % (ANSWERED_FOR, b['N'], len(b['lines']), b['Nmin'], q.get('base', 0), q.get('move', 0),
              q.get('separate', 0), b['N'] - b['Nmin'], b['t'], b['Nmin'], b['t'], b['E'],
              fixed['tines'], fixed['blocks'], len(fixed['removed']), ', '.join(fixed['removed']),
              sum(1 for x in fixed['added_rows'] if x['L'] == SPIGOT_L),
              sum(1 for x in fixed['added_rows'] if x['L'] == BRACKET_L), ', '.join(fixed['added']),
              pieces_after, lines_after.get(_num('cap screw M8 x 20')), lines_after.get(_num('cap screw M8 x 30')),
              'no tine or end block line' if not ({u[0] for u in DF.UNFIXED[:2]} & set(lines_after))
              else 'a tine or end block line left',
              'none' if not inter else '; '.join('%s / %s %s mm3' % h for h in inter), len(hits),
              mass_before, mass_after,
              ('\nNotes: ' + '; '.join(fixed['notes'])) if fixed['notes'] else '', VERSION))
    return msg


# ---------- the command dialog ----------
class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            key = args.command.commandInputs.itemById('machine').selectedItem.name
            ui.messageBox(build(key))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('DesignForAssembly failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('machine', 'Machine',
                                                       adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in ANSWERED:
            dd.listItems.add(k, k == 'bag')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcDesignForAssembly'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Design for assembly',
                                                         'Chapter 65: the test for unnecessary parts and the fixings')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('DesignForAssembly failed:\n%s' % traceback.format_exc())
