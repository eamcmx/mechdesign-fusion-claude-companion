# =============================================================================
# PalletizerDetail.py  -  Chapter 55 of "Mechanical Design in Autodesk Fusion with Claude": one of the palletizer's
# assemblies in detail (the bag gripper, the robot, the compliance unit or the vacuum gripper). Each is built by its
# own script, the functions that draw the detailed parts replaced before the build; then MaterialsFits.py's apply()
# gives the made parts their materials and the fitting faces their classes and roughness by rule, and the decisions of
# lib/palletizer_detail.py that differ from the rules are given by hand after the census.
# Written from PROMPT.txt (a prompt test).
#
#   INPUTS        the dialog: the assembly (bag, robot, unit or vacuum), the plugs' fillet weld leg (mm)
#   DERIVED       palletizer_detail: eyes(), SRB, robot_bearings(), jaw_ends(), bag_6206(), plug_weld(), gland(),
#                 gallery_plug(), vacuum_joint(), sleeve_fixing(), HIWIN; MaterialsFits.apply()
#   HARD RULES    a weld leg under Table 9-6's minimum for the 4 mm wall or over the wall, and an assembly that is
#                 not one of the four, refused
#   CONVENTIONS   each assembly's own document, groups and transforms; no joints; a face split by a construction
#                 plane with splitFaceFeatures
#   ACCEPTANCE    the message: the assembly, the faces given by hand, the descriptions, the version tag
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP        # noqa: E402  every lib/ folder on sys.path; CP.script(name) finds a script
import materials_fits as MF         # noqa: E402
import palletizer_detail as XP      # noqa: E402
import mdfc_parts as mp             # noqa: E402
for _m in (MF, XP, mp):
    importlib.reload(_m)                        # Fusion keeps old modules

VERSION = 'PalletizerDetail 1.0 (2026-11-12)'
CM = 0.1
KEYS = ('bag', 'robot', 'unit', 'vacuum')
TITLES = {'bag': 'bag gripper', 'robot': 'robot', 'unit': 'compliance unit', 'vacuum': 'vacuum gripper'}
SET_IN = 4.0                 # mm, the plugs set into the tube's ends (the prompt)
SHORT_SHOULDER = 36.0        # mm, the short stubs' shoulder (the prompt)
BOSS_LIP = XP.BOSS['lip']    # mm, the lip in the end plates' bosses, 56 (palletizer_detail)
GROUND = 'ground after welding'
TUBE_ZONES = {'boss': ('s6', 0.8), 'below': ('g6', 0.4)}      # the compliance unit's tube, by hand (the prompt)
M4_CLEAR = 4.5               # mm, the sleeves' flange holes for the M4: ISO 273 medium (the prompt gives none)
M4_ANGLES = (90.0, 210.0, 330.0)   # deg, the three M4 on their circle (the prompt gives no orientation)


# ---------- loading the scripts the prompt names ----------
def _load(name):
    """A fresh copy of the companion's script <name>, loaded (not copied) from where companion_paths finds it."""
    spec = importlib.util.spec_from_file_location('mdfc_pd_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------- bodies, world mm ----------
def xcyl(x0, x1, r, y, z):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def xring(x0, x1, ro, ri, y, z):
    return mp.tube((x0, y, z), (x1, y, z), ro, ri)


def bbox_mm(tb):
    b = tb.boundingBox
    return ((b.minPoint.x * 10, b.minPoint.y * 10, b.minPoint.z * 10),
            (b.maxPoint.x * 10, b.maxPoint.y * 10, b.maxPoint.z * 10))


def centre(tb):
    lo, hi = bbox_mm(tb)
    return [(lo[i] + hi[i]) / 2.0 for i in range(3)]


def xbox(xa, xb, yc, zc, h):
    """A box from xa to xb (either order), h either side of the axis (yc, zc)."""
    x0, x1 = sorted((xa, xb))
    return mp.box(x0, x1, yc - h, yc + h, zc - h, zc + h)


def half_space(p0, n, w, S=400.0):
    """A box with one face on the plane through p0 square to the unit vector n, reaching S behind it: near p0, the
    half space n.(p - p0) <= 0. w is a unit vector in the plane."""
    c = [p0[i] - n[i] * S / 2.0 for i in range(3)]
    ob = adsk.core.OrientedBoundingBox3D.create(mp.P3(*[v * CM for v in c]), mp.V3(*n), mp.V3(*w),
                                                S * CM, S * CM, S * CM)
    return mp.tbm().createBox(ob)


def weld_body(sx, xf, h, yc, zc, a):
    """A fillet weld of leg h all round the inside of a square tube (a from its axis to the inner wall), in the corner
    between the wall and the plug's face at |x| = xf, on the side sx of the gripper: a ring of right triangles, one leg
    on the wall from xf to xf + h, the other on the plug's face from the wall in by h."""
    outer = xbox(sx * xf, sx * (xf + h + 1.0), yc, zc, a)
    core = xbox(sx * (xf - 1.0), sx * (xf + h + 2.0), yc, zc, a + 2.0)
    k = 1.0 / math.sqrt(2.0)
    for ny, nz in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        n = (-sx * k, ny * k, nz * k)
        w = (0.0, 0.0, 1.0) if ny else (0.0, 1.0, 0.0)
        p0 = (sx * xf, yc + ny * (a - h), zc + nz * (a - h))
        mp.intersect(core, half_space(p0, n, w))
    return mp.cut(outer, core)


# ---------- faces after the build ----------
def _design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def _leaves(root):
    return [o for o in root.allOccurrences if o.component.bRepBodies.count]


def _named(ls, start):
    return [o for o in ls if o.component.name.startswith(start)]


def _cyl_faces(o, r):
    """The component's own cylindrical faces of radius r (mm), with their geometry, in the component's frame (the
    frame each part was drawn in, the world at rest)."""
    out = []
    for b in o.component.bRepBodies:
        for f in b.faces:
            c = adsk.core.Cylinder.cast(f.geometry)
            if c is not None and abs(c.radius * 10.0 - r) < 1e-3:
                out.append((f, c))
    return out


def _span(f, a):
    b = f.boundingBox
    return sorted((getattr(b.minPoint, a) * 10.0, getattr(b.maxPoint, a) * 10.0))


def _proxy(f, occ):
    try:
        return f.createForAssemblyContext(occ)
    except Exception:
        return None


def _attr(ent, name):
    if ent is None:
        return None
    try:
        a = ent.attributes.itemByName('mdfc', name)
        return a.value if a else None
    except Exception:
        return None


def _put(f, occ, name, val):
    """The attribute ('mdfc', name) of the face set to val, on the face and on its occurrence's proxy."""
    done = False
    for ent in (f, _proxy(f, occ)):
        if ent is None:
            continue
        try:
            a = ent.attributes.itemByName('mdfc', name)
            if a is None:
                ent.attributes.add('mdfc', name, val)
            elif a.value != val:
                a.value = val
            done = True
        except Exception:
            pass
    if not done:
        raise RuntimeError('could not mark a face of %s' % occ.component.name)


def _refit(f, occ, cls, ra_default):
    """A face's class changed by hand, its roughness kept as the census gave it (or ra_default if it gave none)."""
    old = _attr(_proxy(f, occ), 'fit') or _attr(f, 'fit')
    tag, ra = 'Ra', ra_default
    if old:
        w = old.split(' + ')[0].split()
        if len(w) == 3:
            try:
                tag, ra = w[1], float(w[2])
            except ValueError:
                pass
    val = '%s %s %.1f' % (cls, tag, ra)
    _put(f, occ, 'fit', val)
    return old, val


def _colour(des, MFS, f, joint):
    try:
        f.appearance = MFS.appearance(des, MFS.COLOURS[joint])
    except Exception:
        pass


def _material(name):
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    try:
        return lib.materials.itemByName(name)
    except RuntimeError:
        return None


def _to_native(occ, p):
    """A world point (mm) of a part being drawn into its component's own frame, as mdfc_parts.add_body moves it."""
    m = occ.transform2.copy()
    m.invert()
    q = mp.P3(*[v * CM for v in p])
    q.transformBy(m)
    return (q.x * 10.0, q.y * 10.0, q.z * 10.0)


def _on_axis(f, c, pa, pb, tol=0.5):
    """Whether a cylindrical face lies on the axis pa-pb (mm, the component's frame) and overlaps that stretch."""
    L = math.sqrt(sum((pb[i] - pa[i]) ** 2 for i in range(3)))
    u = [(pb[i] - pa[i]) / L for i in range(3)]
    ax = c.axis
    n = ax.length
    if abs(abs((ax.x * u[0] + ax.y * u[1] + ax.z * u[2]) / n) - 1.0) > 1e-6:
        return False
    o = (c.origin.x * 10.0, c.origin.y * 10.0, c.origin.z * 10.0)
    w = [o[i] - pa[i] for i in range(3)]
    t = sum(w[i] * u[i] for i in range(3))
    if math.sqrt(max(0.0, sum(v * v for v in w) - t * t)) > tol:
        return False
    b = f.boundingBox
    lo = [b.minPoint.x * 10.0, b.minPoint.y * 10.0, b.minPoint.z * 10.0]
    hi = [b.maxPoint.x * 10.0, b.maxPoint.y * 10.0, b.maxPoint.z * 10.0]
    ts = sorted(sum(((lo, hi)[k][i] - pa[i]) * u[i] for i in range(3)) for k in (0, 1))
    return ts[0] < L - tol and ts[1] > tol


def _parent(occ):
    """The component that holds the occurrence."""
    try:
        p = occ.sourceComponent
        if p is not None and p.name != occ.component.name:
            return p
    except Exception:
        pass
    des = occ.component.parentDesign
    for c in des.allComponents:
        for i in range(c.occurrences.count):
            if c.occurrences.item(i).component.name == occ.component.name:
                return c
    return des.rootComponent


def _plug_weld(h):
    """plug_weld() for a leg of h: the library's weld group with its leg set to h for the call."""
    keep = XP.WELD['h']
    XP.WELD['h'] = float(h)
    try:
        return XP.plug_weld()
    finally:
        XP.WELD['h'] = keep


def refuse(key, weld):
    if key not in KEYS:
        raise ValueError('No assembly called %s: the four are %s.' % (key, ', '.join(KEYS)))
    w = XP.plug_weld()
    if weld < w['h_min'] - 1e-9:
        raise ValueError('A fillet weld leg of %.1f mm is under the minimum of Shigley\'s Table 9-6, %.1f mm, for the '
                         'tube\'s %.0f mm wall.' % (weld, w['h_min'], w['h_max']))
    if weld > w['h_max'] + 1e-9:
        raise ValueError('A fillet weld leg of %.1f mm is over the tube\'s %.0f mm wall.' % (weld, w['h_max']))


def _message(key, detail, mf_msg, hand, desc, notes=()):
    return ('Palletizer detail, the %s.\n%s\n\nMaterials and fits by rule (MaterialsFits.apply): %s\n\n'
            'Given by hand after the census: %s.\nDescriptions: %s.%s\n\n[%s]'
            % (TITLES[key], detail, mf_msg.split('\n\n')[0], '; '.join(hand) or 'nothing',
               '; '.join(desc) or 'none given by hand', ('\n' + ' '.join(notes)) if notes else '', VERSION))


# =====================================================================================================================
# the bag gripper: StandardParts.py's build, the end plates' bosses, the short stubs, the plugs and their welds
# =====================================================================================================================
def _tube_in(g):
    """From the jaw tube's axis to its inner wall, mm (the tube's section as StandardParts reads it)."""
    try:
        return g.bg.TUBE['b'] / 2.0 - g.bg.TUBE['t']
    except Exception:
        return XP.WELD['b'] / 2.0


def _set_in(sh, tx, yc, zc, a, d_round, xf):
    """A stub's plug moved SET_IN into the tube: from the plug's new face out to the tube's end only the stub's round
    of d_round is left, and the plug's square section starts that much further in. Returns the plug's length."""
    lo, hi = bbox_mm(sh)
    sx = 1.0 if lo[0] + hi[0] > 0 else -1.0
    inner = min(abs(lo[0]), abs(hi[0]))
    plug = tx - inner if 0.0 < tx - inner < 40.0 else 8.0
    ring = xbox(sx * xf, sx * (tx + 1.0), yc, zc, a + 1.0)
    mp.cut(ring, xcyl(sx * (xf - 1.0), sx * (tx + 2.0), d_round / 2.0, yc, zc))
    mp.cut(sh, ring)
    mp.union(sh, xbox(sx * (xf - plug), sx * xf, yc, zc, a))
    return plug


def _build_bag(weld):
    E = XP.jaw_ends()
    W = _plug_weld(weld)
    B6 = XP.bag_6206()
    cls6206 = XP.nearest_zero_class(B6['least'], B6['most'])
    brg = XP.DGBB['6206']
    d, D = brg['d'], brg['D']
    x0, x1 = E['locating']['lip'][1], E['locating']['groove'][0]           # the 6206's faces, |x|
    xf = W['x_weld']                                                        # the plug's face, |x|
    a472, _, _, s472, g472, _, _ = E['din472']
    a471, _, _, s471, g471, _, _ = E['din471']
    x_end = E['boss'][1]
    ctx = dict(bores=[], clips=0, short=0, long=0, welds=0, plates=0, no_boss=0)

    def end_plate(tb):
        """The end plate's two bearing bores, found on its body: each boss lengthened to x_end, each bore turned to a
        lip, the seat, a DIN 472 groove and the seat's diameter again out to the boss's end; the locating boss holds
        the outer ring between the lip and the circlip, the floating one leaves it free either way."""
        lo, hi = bbox_mm(tb)
        sx = 1.0 if lo[0] + hi[0] > 0 else -1.0
        cyls = []
        for i in range(tb.faces.count):
            f = tb.faces.item(i)
            c = adsk.core.Cylinder.cast(f.geometry)
            if c is None:
                continue
            n = c.axis.length
            if n == 0 or abs(abs(c.axis.x / n) - 1.0) > 1e-6:
                continue
            xs = sorted(abs(v) * 10.0 for v in (f.boundingBox.minPoint.x, f.boundingBox.maxPoint.x))
            cyls.append((c.radius * 10.0, c.origin.y * 10.0, c.origin.z * 10.0, xs))
        axes = {}
        for r, y, z, xs in cyls:
            if abs(r - D / 2.0) < 0.01:
                axes.setdefault((round(y, 1), round(z, 1)), (y, z))
        recs = []
        for yb, zb in axes.values():
            boss = [c for c in cyls if abs(c[1] - yb) < 0.05 and abs(c[2] - zb) < 0.05 and g472 / 2.0 + 1.0 < c[0] < 150.0]
            locating = sx * yb > 0                # the +y jaw located at +x, the -y jaw (turned) at -x
            end = E['locating'] if locating else E['floating']
            lip0, seat0 = end['lip']
            gr0, gr1 = end['groove']
            if boss:
                boss.sort(key=lambda c: (-c[3][1], c[0]))
                R, x_out = boss[0][0], boss[0][3][1]
                if x_out < x_end - 1e-6:
                    mp.union(tb, xcyl(sx * (x_out - 0.01), sx * x_end, R, yb, zb))
                r_fill = min(g472 / 2.0 + 1.0, R - 0.5)
            else:
                R, r_fill = None, g472 / 2.0 + 1.0
                ctx['no_boss'] += 1
            mp.union(tb, xcyl(sx * lip0, sx * x_end, r_fill, yb, zb))
            mp.cut(tb, xcyl(sx * (lip0 - 1.0), sx * (x_end + 1.0), BOSS_LIP / 2.0, yb, zb))
            mp.cut(tb, xcyl(sx * seat0, sx * (x_end + 1.0), D / 2.0, yb, zb))
            mp.cut(tb, xcyl(sx * gr0, sx * gr1, g472 / 2.0, yb, zb))
            recs.append(dict(y=yb, z=zb, sx=sx, locating=locating, seat=(seat0, gr0), groove=(gr0, gr1), R=R))
        return tb, recs

    def detail_gripper(g):
        plain_jaw, plain_build = g.jaw_bodies, g.build
        a = _tube_in(g)
        short = {}

        def short_stub(tb0):
            """The short stub: its plug 8 long (as drawn) set 4 mm in, the shoulder 36 to the bearing, the seat, the
            DIN 471 groove and the end past it."""
            lo, hi = bbox_mm(tb0)
            sx = 1.0 if lo[0] + hi[0] > 0 else -1.0
            ys, zs = (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0
            inner = min(abs(lo[0]), abs(hi[0]))
            tx = g.L['tube_x']
            plug = tx - inner if 0.0 < tx - inner < 40.0 else 8.0
            b = xbox(sx * (xf - plug), sx * xf, ys, zs, a)
            mp.union(b, xcyl(sx * (xf - 0.01), sx * x0, SHORT_SHOULDER / 2.0, ys, zs))
            mp.union(b, xcyl(sx * (x0 - 0.01), sx * E['short']['end'], d / 2.0, ys, zs))
            gr0, gr1 = E['short']['groove']
            mp.cut(b, xring(sx * gr0, sx * gr1, d / 2.0 + 1.0, E['short']['d2'] / 2.0, ys, zs))
            short.update(sx=sx, y=ys, z=zs)
            return b

        def jaw_bodies():
            out, tube = [], None
            for nm, tb, mat in plain_jaw():
                if nm.startswith('Stub shaft') and 'short' in nm:
                    tb = short_stub(tb)
                    ctx['short'] += 1
                elif nm.startswith('Tube') and tube is None:
                    tube = tb
                out.append((nm, tb, mat))
            if tube is not None:
                _, yc, zc = centre(tube)
            else:
                yc, zc = g.YH, g.ZH
            for k, sx in enumerate((1.0, -1.0)):
                out.append(('Fillet weld %d' % (k + 1), weld_body(sx, xf, weld, yc, zc, a), 'Steel'))
                ctx['welds'] += 1
            if short:
                sx = short['sx']
                gr0 = E['short']['groove'][0]
                out.append(('Circlip DIN 471 %gx%g' % (d, s471),
                            xring(sx * gr0, sx * (gr0 + s471), (d + 2.0 * a471) / 2.0, g471 / 2.0, short['y'], short['z']),
                            'Steel'))
            return out

        def build(*args, **kw):
            plain_add = mp.add_body

            def add_body(occ, tb, name, material=None):
                if not occ.component.name.startswith('End plate'):
                    return plain_add(occ, tb, name, material)
                tb, recs = end_plate(tb)
                body = plain_add(occ, tb, name, material)
                ctx['plates'] += 1
                parent = _parent(occ)
                for r in recs:
                    r['plate'] = occ.component.name
                    r['pa'] = _to_native(occ, (r['sx'] * r['seat'][0], r['y'], r['z']))
                    r['pb'] = _to_native(occ, (r['sx'] * r['seat'][1], r['y'], r['z']))
                    ctx['bores'].append(r)
                    sx, gr0 = r['sx'], r['groove'][0]
                    clip = xring(sx * gr0, sx * (gr0 + s472), g472 / 2.0, (D - 2.0 * a472) / 2.0, r['y'], r['z'])
                    ctx['clips'] += 1
                    c = mp.new_occ(parent, 'Circlip DIN 472 %gx%g %d' % (D, s472, ctx['clips']))
                    plain_add(c, clip, c.component.name, 'Steel')
                return body

            mp.add_body = add_body
            try:
                return plain_build(*args, **kw)
            finally:
                mp.add_body = plain_add

        g.jaw_bodies, g.build = jaw_bodies, build

    SP = _load('StandardParts')
    plain_load, plain_detail = SP._load, SP.detail_bodies

    def load(name):
        mod = plain_load(name)
        if name == 'BagGripper':
            detail_gripper(mod)
        return mod

    def detail_bodies(g, shoulder, key_len):
        sh, lv, extra, info = plain_detail(g, shoulder, key_len)
        _set_in(sh, g.L['tube_x'], g.YH, g.ZH, _tube_in(g), shoulder, xf)
        ctx['long'] += 1
        return sh, lv, extra, info

    SP._load, SP.detail_bodies = load, detail_bodies
    SP.build()
    des = _design()
    root = des.rootComponent
    MFS = _load('MaterialsFits')
    mf_msg = MFS.apply('bag')
    # ---- by hand after the census ----
    ls = _leaves(root)
    ra_h = MF.HOUSING['stationary outer'][1]
    hand, desc, notes = [], [], []
    n_h7, census_loc = 0, set()
    for o in _named(ls, 'End plate'):
        for f, c in _cyl_faces(o, D / 2.0):
            for r in ctx['bores']:
                if r['plate'] == o.component.name and _on_axis(f, c, r['pa'], r['pb']):
                    if r['locating']:
                        census_loc.add(_attr(_proxy(f, o), 'fit') or _attr(f, 'fit'))
                    else:
                        _refit(f, o, 'H7', ra_h)
                        _colour(des, MFS, f, 'bearing seat')
                        n_h7 += 1
    hand.append('H7 in the %d floating bosses\' seats (the locating ones by rule: %s)'
                % (n_h7, ', '.join(sorted(str(v) for v in census_loc)) or 'none found'))
    steel = _material(MF.FUSION['steel 1020'])
    jaw = [o for o in ls if o.component.name.startswith(('Tube', 'Stub shaft', 'Fillet weld'))]
    if steel is None:
        notes.append('Fusion\'s library has no %s: the jaws keep the rule\'s material.' % MF.FUSION['steel 1020'])
    else:
        for o in jaw:
            for b in o.component.bRepBodies:
                b.material = steel
    hand.append('the jaws\' %d tubes, %d stubs and %d welds in steel 1020 (%s)'
                % (len(_named(ls, 'Tube')), len(_named(ls, 'Stub shaft')), len(_named(ls, 'Fillet weld')),
                   MF.FUSION['steel 1020']))
    n_gr = 0
    for o in _named(ls, 'Stub shaft'):
        for f, c in _cyl_faces(o, d / 2.0):
            xa, xb = sorted(abs(v) for v in _span(f, 'x'))
            if xa < x1 - 0.5 and xb > x0 + 0.5:
                _put(f, o, 'process', GROUND)
                n_gr += 1
    hand.append('%d stub seats marked %s' % (n_gr, GROUND))
    dsg = 'SKF 6206-2RS1/%s' % cls6206
    nb = 0
    for o in _named(ls, 'Bearing 6206'):
        o.component.description = dsg
        nb += 1
    desc.append('%s on the %d bearings (the class nearest SKF\'s virtual zero over the hall)' % (dsg, nb))
    # where the driven stubs are against the locating bosses
    drv = []
    for o in _named(ls, 'Stub shaft long'):
        b = o.boundingBox
        drv.append((b.minPoint.x + b.maxPoint.x) * ((b.minPoint.y + b.maxPoint.y)) > 0)
    if drv and not all(drv):
        notes.append('Note: a driven (long) stub sits at a floating end; the bosses follow "the +y jaw located at +x".')
    if ctx['plates'] != 2 or len(ctx['bores']) != 4:
        notes.append('Note: %d end plates and %d bores were detailed (2 and 4 meant).' % (ctx['plates'], len(ctx['bores'])))
    if ctx['no_boss']:
        notes.append('Note: %d bosses had no outside found on the plate; drawn at the groove plus 1 mm.' % ctx['no_boss'])
    if ctx['short'] == 0 or ctx['long'] == 0:
        notes.append('Note: %d short and %d long stubs were detailed.' % (ctx['short'], ctx['long']))
    detail = ('In each end plate the locating boss holds its 6206\'s outer ring between a lip %g across and a DIN 472 %g '
              'circlip (groove %g across), the floating boss leaves it %g mm either way; the bosses out to %g mm; '
              '%d DIN 472 %g x %g and %d DIN 471 %g x %g circlips; the short stubs with a shoulder %g and a groove %g '
              'to %.1f mm; the plugs %g mm into the tube\'s ends, %d fillet welds of %.1f mm from %g to %g mm '
              '(Table 9-6: %.1f to %.1f mm); in the brake stop n = %.1f on the weld metal and %.1f on the base metal, '
              'in the cycle nf = %.1f.'
              % (BOSS_LIP, D, g472, XP.BOSS['play'], x_end, len(_named(ls, 'Circlip DIN 472')), D, s472,
                 len(_named(ls, 'Circlip DIN 471')), d, s471, SHORT_SHOULDER, E['short']['d2'], E['short']['end'],
                 SET_IN, len(_named(ls, 'Fillet weld')), weld, xf, xf + weld, W['h_min'], W['h_max'], W['n_weld'],
                 W['n_base'], W['nf']))
    _last.update(key='bag', weld=weld, mod=SP, ctx=ctx, hand=hand, desc=desc)
    return _message('bag', detail, mf_msg, hand, desc, notes)


# =====================================================================================================================
# the robot: PalletizerRobot.py's build, the rods' eyes and their pins
# =====================================================================================================================
def _eye_body(e, x, z, ro):
    """A rod's eye along y: bored to the bearing's outside, the shoulder (lip) on the pin's root side, the DIN 472
    groove on the free side and the bore again out to the eye's face."""
    br = XP.SRB[e['bearing']]
    s = e['side']
    y0, y1 = e['eye']
    face_free = y1 if s > 0 else y0
    b = mp.cyl((x, y0, z), (x, y1, z), ro)
    mp.cut(b, mp.cyl((x, y0 - 1.0, z), (x, y1 + 1.0, z), e['lip_d'] / 2.0))
    mp.cut(b, mp.cyl((x, e['ring'][0], z), (x, face_free + s * 1.0, z), br['D'] / 2.0))
    mp.cut(b, mp.cyl((x, e['groove'][0], z), (x, e['groove'][1], z), e['groove_d'] / 2.0))
    return b


def _pin_body(e, x, z):
    """A pin along y: the collar from its link's face to the inner ring, the seat through the ring to 0.5 short of its
    face, the end screw's tapped hole."""
    br = XP.SRB[e['bearing']]
    s = e['side']
    root, r_root = e['collar']
    pin = mp.cyl((x, root, z), (x, r_root, z), e['collar_d'] / 2.0)
    mp.union(pin, mp.cyl((x, r_root - s * 0.01, z), (x, e['pin_end'], z), br['d'] / 2.0))
    sd, sl = e['screw']
    tip = e['washer'][1] - s * sl
    mp.cut(pin, mp.cyl((x, e['pin_end'] + s * 1.0, z), (x, tip - s * 1.0, z), sd / 2.0))
    return pin


def _build_robot():
    RB = _load('PalletizerRobot')
    eyes = XP.eyes()
    by_eye = {e['name'] + ' eye': e for e in eyes}
    by_pin = {'Pin ' + e['pin']: e for e in eyes}
    plain = RB.part
    made = dict(eyes=0, pins=0)

    def part(g, name, tb, material='Steel'):
        e = by_eye.get(name)
        if e is not None:
            lo, hi = bbox_mm(tb)
            x, z = (lo[0] + hi[0]) / 2.0, (lo[2] + hi[2]) / 2.0
            o = plain(g, name, _eye_body(e, x, z, (hi[0] - lo[0]) / 2.0), material)
            cs = e['circlip'][0]
            y0 = e['groove'][0]
            plain(g, 'Circlip DIN 472 %gx%g %s' % (XP.SRB[e['bearing']]['D'], cs, e['name']),
                  mp.tube((x, y0, z), (x, y0 + e['side'] * cs, z), e['groove_d'] / 2.0, e['clip_inner'] / 2.0))
            made['eyes'] += 1
            return o
        e = by_pin.get(name)
        if e is not None:
            x, _, z = centre(tb)
            o = plain(g, name, _pin_body(e, x, z), material)
            s = e['side']
            w0, w1 = e['washer']
            hole = XP.END_WASHER[e['bearing']][2]
            plain(g, 'End washer pin %s' % e['pin'], mp.tube((x, w0, z), (x, w1, z), e['washer_d'] / 2.0, hole / 2.0))
            sd, sl = e['screw']
            plain(g, 'Cap screw M%dx%d pin %s' % (sd, sl, e['pin']), mp.cap_screw(float(sd), float(sl), (x, w1, z), (0, -s, 0)))
            made['pins'] += 1
            return o
        return plain(g, name, tb, material)

    RB.part = part
    RB.build()
    des = _design()
    root = des.rootComponent
    MFS = _load('MaterialsFits')
    mf_msg = MFS.apply('robot')
    # ---- by hand after the census ----
    ls = _leaves(root)
    ra_h = MF.HOUSING['stationary outer'][1]
    hand, desc, notes = [], [], []
    n_h7 = 0
    for e in eyes:
        Dd = XP.SRB[e['bearing']]['D']
        ra_, rb_ = sorted(e['ring'])
        for o in _named(ls, e['name'] + ' eye'):
            for f, c in _cyl_faces(o, Dd / 2.0):
                ya, yb = _span(f, 'y')
                if ya < rb_ - 0.5 and yb > ra_ + 0.5:
                    _refit(f, o, 'H7', ra_h)
                    _colour(des, MFS, f, 'bearing seat')
                    n_h7 += 1
    hand.append('H7 in the %d rods\' eyes\' seats (the load stays along the rod); the triangle\'s seat K7 by rule' % n_h7)
    rows = XP.robot_bearings()
    by_row = {r['pin']: r for r in rows}
    for o in _named(ls, 'Bearing '):
        n = o.component.name
        row = None
        for e in eyes:
            if n == 'Bearing %s %s' % (e['bearing'], e['name']):
                row = by_row.get(e['pin'])
        if row is None:
            row = next((r for r in rows if not r['rod'] and n.startswith('Bearing %s ' % r['bearing'])), None)
        if row is None:
            continue
        if row['needs'] is None:
            notes.append('Note: no clearance class leaves %s at %s a clearance.' % (row['bearing'], row['pin']))
            continue
        dsg = 'SKF %s%s' % (row['bearing'], '' if row['needs'] == 'Normal' else '/' + row['needs'])
        o.component.description = dsg
        desc.append('%s: %s' % (n, dsg))
    if made['eyes'] != 6 or made['pins'] != 6:
        notes.append('Note: %d eyes and %d pins were detailed (6 and 6 meant).' % (made['eyes'], made['pins']))
    detail = ('The six eyes bored to their bearings\' outside, with a shoulder of %g (22207 E) or %g (22213 E) on the '
              'pin\'s root side and a DIN 472 groove and circlip on the free side; the six pins with a collar of %g or %g '
              'from the link to the inner ring, the seat through the ring, the end 0.5 mm short, an end washer and an '
              'ISO 4762 screw (M%d x %d, M%d x %d).'
              % ((XP.EYE_LIP['22207 E'], XP.EYE_LIP['22213 E'], XP.PIN_COLLAR['22207 E'], XP.PIN_COLLAR['22213 E'])
                 + tuple(XP.END_SCREW['22207 E']) + tuple(XP.END_SCREW['22213 E'])))
    _last.update(key='robot', weld=None, mod=RB, hand=hand, desc=desc)
    return _message('robot', detail, mf_msg, hand, desc, notes)


# =====================================================================================================================
# the compliance unit: ComplianceUnit.py's build, the O-ring's gland, the gallery plugs, the tube's two zones
# =====================================================================================================================
def _build_unit():
    CU = _load('ComplianceUnit')
    G, P = XP.gland(), XP.gallery_plug()
    rod = CU.AIR['tube'][0]
    L_or = math.pi * XP.GLAND['W'] ** 2 / 4.0 / XP.GLAND['E']      # the squeezed section keeps the free one's area
    v2 = G['v2']
    plain_z, plain_part, plain_gal = CU.zcyl, CU.part, CU.galleries
    made = dict(groove=0, oring=0, plugs=0)

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        if abs(r - v2['groove_d'] / 2.0) < 1e-9 and abs(abs(z1 - z0) - v2['width']) < 1e-9 and x == 0 and y == 0:
            zc = (z0 + z1) / 2.0                                      # Volume 2's groove: ERIKS's instead
            made['groove'] += 1
            return plain_z(zc - G['width'] / 2.0, zc + G['width'] / 2.0, G['groove_d'] / 2.0, x, y)
        return plain_z(z0, z1, r, x, y)

    def part(g, name, tb, material='Steel'):
        if name.startswith('O-ring'):
            x, y, zc = centre(tb)
            tb = mp.tube((x, y, zc - L_or / 2.0), (x, y, zc + L_or / 2.0), G['groove_d'] / 2.0, rod / 2.0)
            made['oring'] += 1
        return plain_part(g, name, tb, material)

    def along(p, e, t):
        return [p[i] + e[i] * t for i in range(3)]

    def galleries(b, z, x_edge, y_edge, z_face, plugs, g, tag):
        plain_gal(b, z, x_edge, y_edge, z_face, False, g, tag)
        if not plugs:
            return
        ends = [((x_edge, ys * CU.YA, z), (-1.0, 0.0, 0.0), k + 1) for k, ys in enumerate((1, -1))]
        ends.append(((0.0, -y_edge, z), (0.0, 1.0, 0.0), 3))
        for p, e, k in ends:
            mp.cut(b, mp.cyl(along(p, e, -1.0), along(p, e, P['cb_t']), P['cb_d'] / 2.0))
            mp.cut(b, mp.cyl(along(p, e, P['cb_t'] - 0.01), along(p, e, P['cb_t'] + P['tapped']), P['d'] / 2.0))
            plug = mp.cyl(along(p, e, P['cb_t'] - P['k']), along(p, e, P['cb_t']), P['dk'] / 2.0)
            mp.union(plug, mp.cyl(along(p, e, P['cb_t'] - 0.01), along(p, e, P['cb_t'] + P['i']), P['d'] / 2.0))
            plain_part(g, 'Plug %s %d' % (tag, k), plug)
            made['plugs'] += 1

    CU.zcyl, CU.part, CU.galleries = zcyl, part, galleries
    CU.build()
    des = _design()
    root = des.rootComponent
    ls = _leaves(root)
    # ---- before the census: the tube's outside split where it leaves its boss ----
    z_split = CU.Z_TOP - CU.AIR['boss'][1]
    tube = _named(ls, 'Tube 20x16')[0]
    comp = tube.component
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, adsk.core.ValueInput.createByReal(_to_native(tube, (0, 0, z_split))[2] * CM))
    plane = comp.constructionPlanes.add(ci)
    col = adsk.core.ObjectCollection.create()
    for f, c in _cyl_faces(tube, rod / 2.0):
        col.add(f)
    sf = comp.features.splitFaceFeatures
    sf.add(sf.createInput(col, plane, True))
    MFS = _load('MaterialsFits')
    mf_msg = MFS.apply('unit')
    # ---- by hand after the census ----
    ls = _leaves(root)
    hand, desc, notes = [], [], []
    zones = []
    for o in _named(ls, 'Tube 20x16'):
        for f, c in _cyl_faces(o, rod / 2.0):
            za, zb = _span(f, 'z')
            where = 'boss' if (za + zb) / 2.0 > z_split else 'below'
            cls, ra = TUBE_ZONES[where]
            _put(f, o, 'fit', '%s Ra %.1f' % (cls, ra))
            _colour(des, MFS, f, 'pressed' if where == 'boss' else 'sliding')
            zones.append('%s Ra %.1f %s' % (cls, ra, 'in the boss' if where == 'boss' else 'below it'))
    hand.append('the tube\'s outside in %d zones: %s' % (len(zones), ', '.join(sorted(zones))))
    hw = XP.HIWIN['unit']
    for o in ls:
        n = o.component.name
        if n.startswith('Block HGH25CA'):
            o.component.description = 'HIWIN ' + hw['block']
        elif n.startswith('Rail HGR25'):
            o.component.description = 'HIWIN ' + hw['rail']
    desc.append('%d blocks HIWIN %s, %d rails HIWIN %s' % (len(_named(ls, 'Block HGH25CA')), hw['block'],
                                                        len(_named(ls, 'Rail HGR25')), hw['rail']))
    if made['groove'] != 1 or made['oring'] != 1 or made['plugs'] != 6:
        notes.append('Note: %d grooves, %d O-rings and %d plugs were detailed (1, 1 and 6 meant).'
                     % (made['groove'], made['oring'], made['plugs']))
    detail = ('The O-ring\'s groove %.2f across and %.2f wide (ERIKS\'s dynamic table, squeeze %.1f %%, %.1f to %.1f %% at '
              'the tolerances), the O-ring drawn squeezed to %.2f x %.2f; six DIN 908 M10 x %g plugs, their collars in '
              'counterbores %g across and %g deep, tapped %g; the tube\'s outside split at z = %.1f mm where it leaves '
              'its boss.'
              % (G['groove_d'], G['width'], G['squeeze'], G['squeeze_min'], G['squeeze_max'], XP.GLAND['E'], L_or,
                 P['p'], P['cb_d'], P['cb_t'], P['tapped'], z_split))
    _last.update(key='unit', weld=None, mod=CU, hand=hand, desc=desc)
    return _message('unit', detail, mf_msg, hand, desc, notes)


# =====================================================================================================================
# the vacuum gripper: VacuumGripper.py's build, the plate's through bolts, the sleeves' flanges
# =====================================================================================================================
def _build_vacuum():
    VG = _load('VacuumGripper')
    J, SF, SL, V = XP.vacuum_joint(), XP.sleeve_fixing(), XP.SLEEVE, XP.VAC
    L = VG.L
    ps = L['plate_screw']
    zt, zb = L['beam']
    t = L['beam_t']
    p0, p1 = L['plate']
    s0 = L['sleeve'][0]
    wd, wo, wt = V['washer']
    old_screw = 'Cap screw M%dx' % int(round(ps['d']))          # the plate's screws, tapped into it (Chapter 46)
    plain_z, plain_part = VG.zcyl, VG.part
    made = dict(bolts=0, washers=0, nuts=0, sleeves=0, m4=0, holes=0)

    def at_screw(x, y):
        return any(abs(x - xs) < 1e-6 for xs in ps['x']) and abs(abs(y) - L['beam_y']) < 1e-6

    def holes(x, y):
        r = SL['pcd'] / 2.0
        return [(x + r * math.cos(math.radians(a)), y + r * math.sin(math.radians(a))) for a in M4_ANGLES]

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        if at_screw(x, y):
            if abs(r - ps['d'] / 2.0) < 1e-9 and abs(z0 - (zb + t - ps['L'] - 0.5)) < 1e-6 and abs(z1 - (p0 + 1.0)) < 1e-6:
                made['holes'] += 1                                   # the plate's tapped hole: 6.6 through
                return plain_z(p1 - 1.0, p0 + 1.0, V['hole'] / 2.0, x, y)
            if abs(r - ps['clear_r']) < 1e-9 and abs(z0 - (zb - 1.0)) < 1e-6 and abs(z1 - (zb + t + 1.0)) < 1e-6:
                return plain_z(z0, z1, V['hole'] / 2.0, x, y)       # the beam's wall: 6.6 through
        return plain_z(z0, z1, r, x, y)

    def part(g, name, tb, material='Steel'):
        if name.startswith(old_screw):
            x, y, _ = centre(tb)
            if at_screw(x, y):
                k = int(name.split()[-1])
                zw = zb + t                                          # the beam's bottom wall, inside
                plain_part(g, 'Washer ISO 7089 6 %d' % (2 * k - 1), mp.tube((x, y, zw), (x, y, zw + wt), wo / 2.0, wd / 2.0))
                o = plain_part(g, 'Cap screw M6x%d %d' % (J['L'], k),
                               mp.cap_screw(V['d'], float(J['L']), (x, y, zw + wt), (0, 0, -1)))
                plain_part(g, 'Washer ISO 7089 6 %d' % (2 * k), mp.tube((x, y, p1 - wt), (x, y, p1), wo / 2.0, wd / 2.0))
                plain_part(g, 'Nut ISO 4032 M6 %d' % k, mp.nut(V['d'], (x, y, p1 - wt), (0, 0, -1), m=V['nut_m']))
                made['bolts'] += 1
                made['washers'] += 2
                made['nuts'] += 1
                return o
        if name == 'Plate':
            for _, x, y in VG.cups_xy():
                for xs, ys in holes(x, y):
                    mp.cut(tb, mp.cyl((xs, ys, s0 - SF['thread']), (xs, ys, s0 + 1.0), SL['d'] / 2.0))
            return plain_part(g, name, tb, material)
        if name.startswith('Sleeve '):
            x, y, _ = centre(tb)
            mp.union(tb, mp.tube((x, y, s0), (x, y, s0 + SL['flange_t']), SL['flange_d'] / 2.0, L['sleeve_r'][1]))
            for xs, ys in holes(x, y):
                mp.cut(tb, mp.cyl((xs, ys, s0 - 1.0), (xs, ys, s0 + SL['flange_t'] + 1.0), M4_CLEAR / 2.0))
            o = plain_part(g, name, tb, material)
            for xs, ys in holes(x, y):
                made['m4'] += 1
                plain_part(g, 'Cap screw M%dx%d %d' % (SL['d'], SL['L'], made['m4']),
                           mp.cap_screw(SL['d'], SL['L'], (xs, ys, s0 + SL['flange_t']), (0, 0, -1)))
            made['sleeves'] += 1
            return o
        return plain_part(g, name, tb, material)

    VG.zcyl, VG.part = zcyl, part
    VG.build()
    MFS = _load('MaterialsFits')
    mf_msg = MFS.apply('vacuum')
    notes = []
    if made['bolts'] != J['screws'] or made['holes'] != J['screws'] or made['sleeves'] != SF['cups']:
        notes.append('Note: %d bolts, %d plate holes and %d sleeves were detailed (%d, %d and %d meant).'
                     % (made['bolts'], made['holes'], made['sleeves'], J['screws'], J['screws'], SF['cups']))
    detail = ('The plate held by %d ISO 4762 M6 x %d through bolts (grip %.1f, L > l + H = %.1f), %d ISO 7089 washers '
              'under the heads and the nuts, %d ISO 4032 nuts, the plate\'s holes %g through, preload %.0f N at %.1f N m; '
              '%d sleeves on foot flanges %g across and %g thick, %d ISO 4762 M%d x %d on a circle of %g tapped %g mm '
              'into the plate (preload %.0f times the stop\'s pull).'
              % (made['bolts'], J['L'], J['grip'], J['need'], made['washers'], made['nuts'], V['hole'], J['Fi'], J['T'],
                 made['sleeves'], SL['flange_d'], SL['flange_t'], made['m4'], SL['d'], SL['L'], SL['pcd'],
                 SF['thread'], SF['ratio']))
    _last.update(key='vacuum', weld=None, mod=VG, hand=[], desc=[])
    return _message('vacuum', detail, mf_msg, [], [], notes)


# ---------- the build ----------
_last = {}


def build(key='bag', weld=None):
    weld = XP.WELD['h'] if weld is None else float(weld)
    refuse(key, weld)
    if key == 'bag':
        return _build_bag(weld)
    if key == 'robot':
        return _build_robot()
    if key == 'unit':
        return _build_unit()
    return _build_vacuum()


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            key = ins.itemById('assembly').selectedItem.name
            ui.messageBox(build(key, ins.itemById('weld').value), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        dd = ins.addDropDownCommandInput('assembly', 'The assembly', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in KEYS:
            dd.listItems.add(k, k == 'bag')
        ins.addValueInput('weld', 'The plugs\' fillet weld leg (mm)', '', adsk.core.ValueInput.createByReal(XP.WELD['h']))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'PalletizerDetailCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Palletizer detail', 'Chapter 55: the palletizer in detail')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PalletizerDetail could not start:\n' + traceback.format_exc())
