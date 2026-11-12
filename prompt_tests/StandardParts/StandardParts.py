# =============================================================================
# StandardParts.py  -  Chapter 50 of "Mechanical Design in Autodesk Fusion with Claude": the bag gripper with its
# bought parts described by their full designations and its driven stub shafts detailed with their standard parts.
# The gripper is built by its own script (BagGripper.py's build()) and numbered by PartNumbers.py's number(); before
# the build three of BagGripper's functions are replaced: the jaw's bodies, the tube that draws the bearings and the
# cylinder that draws the dowel and its hole. The sizes are lib/standard_parts.py's. Written from PROMPT.txt.
#
#   INPUTS        the dialog: the shaft's shoulder (mm), the key's length (mm)
#   DERIVED       standard_parts: stub_detail(), bearing_6206(), lever_key(), dowel_adapter(), decide(),
#                 detail_numbers(); detail_inventory: number_map(), kind_of(); PartNumbers.number()
#   HARD RULES    build() refuses a shoulder outside SKF's band, a key shorter than crushing in the brake stop allows
#                 and a key length DIN 6885-1 does not make at 8 mm wide
#   CONVENTIONS   the gripper's own: its new design, its groups placed by transforms, no joints; the numbers and
#                 descriptions in Component.partNumber and Component.description; mm here, cm inside mdfc_parts
#   ACCEPTANCE    the message; 82 parts; every bought part described; the stack closed; the refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
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
import companion_paths as CP   # noqa: E402  every lib/ folder on sys.path; CP.script(name) finds a script
import detail_inventory as D   # noqa: E402
import standard_parts as S     # noqa: E402
import mdfc_parts as mp        # noqa: E402
importlib.reload(D)                             # Fusion keeps old modules
importlib.reload(S)
importlib.reload(mp)

VERSION = 'StandardParts 1.0 (2026-11-11)'
CM = 0.1

SPACER = (36.0, 30.0)     # the spacer, outside and inside (mm)
BOSS_OVER = 5.0           # the lever's boss 5 mm longer than the key
KEY_IN = 2.5              # the key 2.5 mm inside each end of the boss
DOWEL_D = 10.0            # the adapter's dowel, ISO 8734 10 x 22
TAP_BEYOND = 0.5          # the tapped hole runs 0.5 d past the end screw's tip (the prompt gives no depth)
TOL = 1e-6


def _load(name):
    """A companion script loaded as a module, as it stands on disk."""
    spec = importlib.util.spec_from_file_location('mdfc_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def xcyl(x0, x1, r, y, z):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def xslot(x0, x1, w, z0, z1, y):
    """A form-A outline along x, w wide with round ends, from z0 up to z1: the key or the keyseat it sits in."""
    r = w / 2.0
    b = mp.box(x0 + r, x1 - r, y - r, y + r, z0, z1)
    return mp.union(b, mp.cyl((x0 + r, y, z0), (x0 + r, y, z1), r), mp.cyl((x1 - r, y, z0), (x1 - r, y, z1), r))


# ---------- the detail: stub_detail() for this shoulder and this key ----------
def detail(shoulder, key_len):
    """The driven stub shaft's detail, after the refusals."""
    dt = S.stub_detail()
    brg, key, dw = S.bearing_6206(), S.lever_key(), S.dowel_adapter()
    lo, hi = brg['skf']['da_min'], brg['skf']['da_max']
    if not lo - TOL <= shoulder <= hi + TOL:
        raise ValueError('A shoulder of %.1f mm: SKF gives the 6206-2RS1\'s shaft abutment from da min %.1f to da max '
                         '%.1f mm.' % (shoulder, lo, hi))
    if key_len < key['l_crush'] - TOL:
        raise ValueError('A key %.1f mm long would be crushed in the brake stop: the hub\'s side of the key, h - t1 = '
                         '%.0f mm high, needs at least %.2f mm at a design factor of %.1f.'
                         % (key_len, key['h'] - key['t1'], key['l_crush'], S.N_KEY))
    if not any(abs(key_len - x) < TOL for x in key['Ls']):
        raise ValueError('A key %.1f mm long: DIN 6885-1 makes keys %.0f mm wide only in the lengths %s mm.'
                         % (key_len, key['b'], ', '.join('%g' % x for x in key['Ls'])))
    x1 = dt['x_lever'][1]
    boss = key_len + BOSS_OVER
    hub = (x1 - boss, x1)
    if hub[0] <= dt['x_brg'][1] + TOL:
        raise ValueError('A key %.0f mm long: its boss, %.0f mm, would reach the bearing at x %.0f and leave no room '
                         'for the spacer.' % (key_len, boss, dt['x_brg'][1]))
    P = dict(dt)
    P.update(shoulder=float(shoulder), key_L=float(key_len), boss=boss, hub=hub,
             key=(hub[0] + KEY_IN, hub[1] - KEY_IN), spacer=(dt['x_brg'][1], hub[0]),
             key_name='Key %dx%dx%d' % (dt['b'], dt['h'], key_len), screw_name='Cap screw M%dx%d' % dt['screw'],
             tap=dt['thread'] + TAP_BEYOND * dt['screw'][0], dowel=dw, key_data=key, skf=brg['skf'])
    return P


def new_numbers(P):
    """detail_numbers(), the key's name and description at the key's length."""
    out = {}
    for name, (num, desc) in S.detail_numbers().items():
        if name.startswith('Key '):
            out[P['key_name']] = (num, 'DIN 6885-1 A %d x %d x %d' % (P['b'], P['h'], P['key_L']))
        else:
            out[name] = (num, desc)
    return out


# ---------- the bodies the replacements draw (the +y jaw, closed, world mm) ----------
def stub_shaft(tb, P, yh, zh):
    """The driven stub shaft on the gripper's own end cap: the shoulder from the tube's end to the bearing, the seat to
    1 mm inside the hub's face, a form-A keyseat t1 deep on top, a tapped hole for the end screw."""
    d, xt, xb0, end = P['d'], P['x_tube'], P['x_brg'][0], P['end']
    mp.cut(tb, xcyl(xt + 1.0, P['x_lever'][1] + 1.0, d / 2 + 1.0, yh, zh))     # the concept's plain stub
    mp.union(tb, xcyl(xt - 1.0, xb0, P['shoulder'] / 2, yh, zh))                # the shoulder
    mp.union(tb, xcyl(xb0 - 1.0, end, d / 2, yh, zh))                            # the seat
    k0, k1 = P['key']
    mp.cut(tb, xslot(k0, k1, P['b'], zh + d / 2 - P['t1'], zh + d / 2 + 1.0, yh))
    sd = P['screw'][0]
    mp.cut(tb, xcyl(end - P['tap'], end + 1.0, sd / 2.0, yh, zh))               # tapped, at the nominal diameter
    return tb


def lever(P, mod, yh, zh):
    """The gripper's lever with its hub lengthened into the boss, bored and keywayed t2 deep through."""
    x0, x1 = P['x_lever']
    h0 = P['hub'][0]
    r = mod.L['lever']
    d = P['d']
    lv = xcyl(h0, x1, P['hub_r'], yh, zh)                                        # the boss
    mp.union(lv, mp.box(x0, x1, yh - 20.0, yh + 20.0, zh, zh + r))               # the arm and its eye, as the gripper's
    mp.union(lv, xcyl(x0, x1, 20.0, yh, zh + r))
    mp.cut(lv, xcyl(h0 - 1.0, x1 + 1.0, d / 2, yh, zh))
    mp.cut(lv, mp.box(h0 - 1.0, x1 + 1.0, yh - P['b'] / 2, yh + P['b'] / 2, zh, zh + d / 2 + P['t2']))
    mp.union(lv, xcyl(x1 - 0.01, x1 + 20.0, mod.CYL['EK'] / 2, yh, zh + r))      # the pin, as the gripper's
    return lv


def detail_bodies(P, yh, zh):
    """The parts the detail adds on the driven end: the spacer, the key, the end washer and the end screw."""
    d = P['d']
    x1 = P['x_lever'][1]
    s0, s1 = P['spacer']
    k0, k1 = P['key']
    w = P['washer']
    zk = zh + d / 2 - P['t1']
    sd, sl = P['screw']
    return [('Spacer', mp.tube((s0, yh, zh), (s1, yh, zh), SPACER[0] / 2, SPACER[1] / 2), 'Steel'),
            (P['key_name'], xslot(k0, k1, P['b'], zk, zk + P['h'], yh), 'Steel'),
            ('End washer', mp.tube((x1, yh, zh), (x1 + w['t'], yh, zh), w['D'] / 2, w['hole'] / 2), 'Steel'),
            (P['screw_name'], mp.cap_screw(float(sd), float(sl), (x1 + w['t'], yh, zh), (-1.0, 0.0, 0.0)), 'Steel')]


def bearing_body(x0, y, z, P):
    """A 6206-2RS1 at SKF's d, D and B, its seals between d1 and D2 set in from the rings' faces."""
    d, Do, B, d1, D2 = P['bore']
    si = P['set_in']
    b = mp.tube((x0, y, z), (x0 + B, y, z), Do / 2, d / 2)
    mp.cut(b, mp.tube((x0 - 1.0, y, z), (x0 + si, y, z), D2 / 2, d1 / 2))
    mp.cut(b, mp.tube((x0 + B - si, y, z), (x0 + B + 1.0, y, z), D2 / 2, d1 / 2))
    return b


# ---------- the three replacements ----------
def replace(mod, P):
    """BagGripper's jaw_bodies, xtube and zcyl replaced: each draws the detail for the calls it means and passes every
    other through to the gripper's own."""
    own_jaw, own_xtube, own_zcyl = mod.jaw_bodies, mod.xtube, mod.zcyl
    yh, zh = mod.YH, mod.ZH
    xb = mod.L['bearing_x']
    ro, ri = mod.bg.BRG[30][0] / 2, mod.L['stub_d'] / 2
    dw = P['dowel']

    def jaw_bodies():
        out = []
        for nm, tb, mat in own_jaw():
            if nm == 'Stub shaft long':
                tb = stub_shaft(tb, P, yh, zh)
            elif nm == 'Lever':
                tb = lever(P, mod, yh, zh)
            out.append((nm, tb, mat))
        return out + detail_bodies(P, yh, zh)

    def xtube(x0, x1, r_out, r_in, y, z):
        if (abs(x0 - xb[0]) < TOL and abs(x1 - xb[1]) < TOL and abs(r_out - ro) < TOL and abs(r_in - ri) < TOL):
            return bearing_body(x0, y, z, P)
        return own_xtube(x0, x1, r_out, r_in, y, z)

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        if abs(z0 + dw['in_adapter0']) < TOL and abs(z1 - dw['in_tool']) < TOL and abs(r - DOWEL_D / 2) < TOL:
            return own_zcyl(-dw['in_adapter'], dw['in_tool'], r, x, y)        # the dowel 22 long, and its hole
        return own_zcyl(z0, z1, r, x, y)

    mod.jaw_bodies, mod.xtube, mod.zcyl = jaw_bodies, xtube, zcyl


# ---------- the numbers and the descriptions ----------
def describe(root, PN, new):
    """PartNumbers' number(), then the detail's new kinds numbered and every bought part given its full designation."""
    nmap = D.number_map('bag')
    done, part_of, left = PN.number(root, 'bag')
    added, bought, seen = [], {}, set()
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count == 0 or c.entityToken in seen:
            continue
        seen.add(c.entityToken)
        k = D.kind_of(c.name, new)
        if k:
            c.partNumber, c.description = new[k]
            added.append(c.name)
        elif c.partNumber.startswith('S-'):
            c.description = S.decide(nmap[D.kind_of(c.name, nmap)][1])['full']
        if c.partNumber.startswith('S-'):
            bought.setdefault(c.partNumber, [c.description, 0])[1] += 1
    left = sorted({n for n in left if D.kind_of(n, new) is None})
    return dict(added=added, bought=bought, left=left, part_of=part_of)


def measured_shoulder(root, P):
    """The shoulder's diameter read off jaw A's driven stub shaft: its cylinder from the tube's end to the bearing."""
    for o in root.allOccurrences:
        if o.component.name != 'Stub shaft long A' or o.bRepBodies.count == 0:
            continue
        for f in o.bRepBodies.item(0).faces:
            g = f.geometry
            if isinstance(g, adsk.core.Cylinder) and abs(abs(g.axis.x) - 1.0) < TOL:
                bb = f.boundingBox
                if abs(bb.minPoint.x / CM - P['x_tube']) < 0.02 and abs(bb.maxPoint.x / CM - P['x_brg'][0]) < 0.02:
                    return 2 * g.radius / CM
    return None


_last = {}


def build(shoulder=36.0, key_len=25.0):
    P = detail(shoulder, key_len)
    PN = _load('PartNumbers')
    mod = _load('BagGripper')
    replace(mod, P)
    mod.build()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    new = new_numbers(P)
    res = describe(root, PN, new)
    _last.update(groups=mod._last['groups'], g=mod, added=res['added'], detail=P, new=new, bought=res['bought'],
                 left=res['left'])
    pl = mod.parts_list(root)
    n_parts = sum(pl.values())
    names = {'%s %s' % (n, j) for n in ('Spacer', P['key_name'], 'End washer', P['screw_name']) for j in 'AB'}
    n_new = sum(1 for o in root.allOccurrences if o.component.name in names)
    try:
        sh = measured_shoulder(root, P)
    except Exception:
        sh = None
    d, Do, B, d1, D2 = P['bore']
    w, kd, dw, skf = P['washer'], P['key_data'], P['dowel'], P['skf']
    screw_full = S.decide(P['screw_name'])['full']
    lines = [
        'Bag gripper detailed with its standard parts: %d parts, the gripper\'s %d and %d more on the jaws\' driven '
        'ends.' % (n_parts, n_parts - n_new, n_new),
        'Shoulder: %.1f mm across (on the model: %s), from the tube\'s end at x %.0f to the bearing at x %.0f; SKF\'s '
        'band %.1f to %.1f mm (da min to da max).' % (P['shoulder'], '%.3f' % sh if sh else 'not found',
                                                      P['x_tube'], P['x_brg'][0], skf['da_min'], skf['da_max']),
        'Bearings: four %s, d %.0f, D %.0f, B %.0f, the seals between d1 %.2f and D2 %.2f set in %.1f mm from the '
        'rings\' faces.' % (S.decide('Bearing 6206')['full'], d, Do, B, d1, D2, P['set_in']),
        'Shaft: the %.0f mm seat from x %.0f to x %.0f, 1 mm inside the hub\'s face; a form-A keyseat %.0f wide, t1 '
        '%.1f deep on top; a tapped hole M%d, %.0f deep.' % (d, P['x_brg'][0], P['end'], P['b'], P['t1'],
                                                             P['screw'][0], P['tap']),
        'Spacer: %.0f outside, %.0f inside, %.0f long, from the inner ring at x %.0f to the boss at x %.0f.'
        % (SPACER[0], SPACER[1], P['spacer'][1] - P['spacer'][0], P['spacer'][0], P['spacer'][1]),
        'Hub: the lever\'s hub lengthened into a boss %.0f long (x %.0f to %.0f), %.0f across, bored %.0f and keywayed '
        't2 %.1f deep through.' % (P['boss'], P['hub'][0], P['hub'][1], 2 * P['hub_r'], d, P['t2']),
        'Key: %s (b %.0f, h %.0f), x %.1f to %.1f, %.1f mm inside each end of the boss; crushing in the brake stop '
        'needs %.2f mm, shear %.2f.' % (new[P['key_name']][1], P['b'], P['h'], P['key'][0], P['key'][1], KEY_IN,
                                        kd['l_crush'], kd['l_shear']),
        'End washer: %.0f x %.0f with a %.0f mm hole, %.0f mm clear of the shaft\'s end. End screw: %s, %.0f mm under '
        'the head, %.0f mm of thread in the shaft against 1.5 d = %.0f.'
        % (w['D'], w['t'], w['hole'], P['x_lever'][1] - P['end'], screw_full, P['under_head'], P['thread'],
           P['thread_need']),
        'Dowel: %s, drawn %.0f long, %.0f mm in the adapter (%.0f thick) and %.0f in the tool plate.'
        % (S.decide('Dowel pin 10x17')['full'], dw['L'], dw['in_adapter'], dw['t_adapter'], dw['in_tool']),
        'Bought components described: %d components, %d numbers: %s.'
        % (sum(v[1] for v in res['bought'].values()), len(res['bought']),
           '; '.join('%s %s (%d)' % (k, v[0], v[1]) for k, v in sorted(res['bought'].items()))),
        'New numbers, alike on both jaws: %s.' % ', '.join('%s %s' % new[k] for k in sorted(new)),
        'Left unnumbered: %s.' % (', '.join(res['left']) or 'none'),
        '[%s]' % VERSION,
    ]
    return '\n'.join(lines)


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('shoulder', 'The shaft\'s shoulder (mm)', 36.0), ('key_len', 'The key\'s length (mm)', 25.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value for k, _, _ in FIELDS}
            ui.messageBox(build(**vals), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, dflt in FIELDS:
            ins.addValueInput(key, label, '', adsk.core.ValueInput.createByReal(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'StandardPartsCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Standard parts',
                                                         'Chapter 50: the bag gripper with its standard parts')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StandardParts could not start:\n' + traceback.format_exc())
