# =============================================================================
# StandardParts.py  -  Chapter 50 of "Mechanical Design in Autodesk Fusion with Claude": the bag gripper with its bought
# parts described by their full designations and its driven stub shafts detailed with their standard parts. The
# gripper is built by its own Volume 2 script and numbered by Chapter 49's; this script only changes what Chapter 50
# decided: the shaft's shoulder at SKF's abutment, the 6206-2RS1 drawn to SKF's figures, the lever's key and its seats,
# a spacer, the lever's hub lengthened into a boss, an end washer and screw, and the adapter's dowel at a made length.
# Written from PROMPT.txt beside it; checked by checks/standard_parts_check.py.
#
#   INPUTS        the dialog: the shaft's shoulder (mm), the key's length (mm)
#   DERIVED       standard_parts: stub_detail(), detail_numbers(), decide(), dowel_adapter(); detail_inventory
#   HARD RULES    a shoulder outside SKF's band, a key shorter than crushing allows or not made, refused
#   CONVENTIONS   the gripper's own; both jaws detailed alike, the -y one the +y one turned; no joint, no move
#   ACCEPTANCE    the message; the check: descriptions, the shoulder, the key in its hub, the stack, no interference
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

SCRIPTS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import detail_inventory as D   # noqa: E402
import standard_parts as S     # noqa: E402
import mdfc_parts as mp        # noqa: E402
importlib.reload(D)
importlib.reload(S)
importlib.reload(mp)

VERSION = 'StandardParts 1.0 (2026-11-03)'
DT = S.stub_detail()
DW = S.dowel_adapter()


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def refuse(shoulder, key_len):
    if not DT['da_min'] <= shoulder <= DT['da_max']:
        raise ValueError('A shoulder of %.1f mm: SKF\'s 6206-2RS1 asks for %.1f to %.1f mm.'
                         % (shoulder, DT['da_min'], DT['da_max']))
    if key_len < DT['key_need'] - 1e-9:
        raise ValueError('A key %.0f mm long: in the brake stop it is crushed unless it is at least %.2f mm.'
                         % (key_len, DT['key_need']))
    made = S.commercial('DIN_6885-1')[str(int(DT['b']))]
    if key_len not in made:
        raise ValueError('A key %.0f mm long: DIN 6885-1 makes keys %d wide in %s mm.'
                         % (key_len, DT['b'], ', '.join('%g' % x for x in sorted(made) if x <= 50)))


# ---------- the detail, for the +y jaw at rest (the -y jaw's is the same turned) ----------
def xslot(x0, x1, w, z0, z1, y, zc=0.0):
    """A round-ended slot along x, w wide, from z0 to z1 (z0 < z1), its ends' centres w/2 inside x0 and x1."""
    r = w / 2.0
    s = mp.box(x0 + r, x1 - r, y - r, y + r, z0, z1)
    for xc in (x0 + r, x1 - r):
        mp.union(s, mp.cyl((xc, y, z0), (xc, y, z1), r))
    return s


def detail_bodies(g, shoulder, key_len):
    """The driven stub shaft, the lever, and the parts the detail adds, from the gripper's own jaw (g: its module)."""
    L, yh, zh = g.L, g.YH, g.ZH
    r = DT['d'] / 2.0
    hb = g.bg.TUBE['b'] / 2
    tt = g.bg.TUBE['t']
    tx = L['tube_x']
    boss = key_len + 5.0
    hub = (DT['x_lever'][1] - boss, DT['x_lever'][1])
    key = (hub[0] + 2.5, hub[1] - 2.5)
    d_sh = shoulder
    # the shaft: the end cap in the tube, the shoulder to the bearing, the seat through the bearing and the hub to
    # 1 mm inside the hub's face, the keyseat t1 deep on top, the tapped hole for the end screw
    sh = mp.box(tx - 8.0, tx, yh - hb + tt, yh + hb - tt, zh - hb + tt, zh + hb - tt)
    mp.union(sh, g.xcyl(tx - 8.0, DT['end'], r, yh, zh))
    mp.union(sh, g.xcyl(tx - 0.01, DT['x_brg'][0], d_sh / 2.0, yh, zh))
    mp.cut(sh, xslot(key[0], key[1], DT['b'], zh + r - DT['t1'], zh + r + 1.0, yh))
    sd, sl = DT['screw']
    mp.cut(sh, g.xcyl(DT['end'] - (sl - DT['under_head']) - 1.0, DT['end'] + 1.0, sd / 2.0, yh, zh))
    # the lever: its hub lengthened into the boss, bored and keywayed t2 deep through
    x0, x1 = DT['x_lever']
    rl = L['lever']
    lv = g.xcyl(hub[0], x1, DT['hub_r'], yh, zh)
    mp.union(lv, mp.box(x0, x1, yh - 20.0, yh + 20.0, zh, zh + rl))
    mp.union(lv, g.xcyl(x0, x1, 20.0, yh, zh + rl))
    mp.cut(lv, g.xcyl(hub[0] - 1.0, x1 + 1.0, r, yh, zh))
    mp.cut(lv, mp.box(hub[0] - 1.0, x1 + 1.0, yh - DT['b'] / 2, yh + DT['b'] / 2, zh + r - 1.0, zh + r + DT['t2']))
    mp.union(lv, g.xcyl(x1 - 0.01, x1 + 20.0, g.CYL['EK'] / 2, yh, zh + rl))
    # the spacer, the key, the end washer, the end screw
    sp = g.xtube_plain(DT['spacer'][0], hub[0], d_sh / 2.0, r, yh, zh)
    ky = xslot(key[0], key[1], DT['b'], zh + r - DT['t1'], zh + r - DT['t1'] + DT['h'], yh)
    w = DT['washer']
    ew = g.xtube_plain(x1, x1 + w['t'], w['D'] / 2.0, w['hole'] / 2.0, yh, zh)
    sc = mp.cap_screw(float(sd), float(sl), (x1 + w['t'], yh, zh), (-1, 0, 0))
    kname = 'Key %dx%dx%d' % (DT['b'], DT['h'], key_len)
    return sh, lv, [('Spacer', sp, 'Steel'), (kname, ky, 'Steel'), ('End washer', ew, 'Steel'),
                    ('Cap screw M%dx%d' % (sd, sl), sc, 'Steel')], dict(hub=hub, key=key, boss=boss, kname=kname)


def skf_bearing(x0, x1, y, z):
    """The 6206-2RS1 drawn to SKF's figures: d, D, B, and the seals between the inner ring's shoulder d1 and the outer
    ring's recess D2, their faces set in from the rings'."""
    d, Dd, B, d1, D2 = DT['bore']
    b = mp.tube((x0, y, z), (x1, y, z), Dd / 2.0, d / 2.0)
    for a, c in ((x0 - 1.0, x0 + DT['set_in']), (x1 - DT['set_in'], x1 + 1.0)):
        mp.cut(b, mp.tube((a, y, z), (c, y, z), D2 / 2.0, d1 / 2.0))
    return b


_last = {}


def build(shoulder=None, key_len=None):
    shoulder = DT['shoulder'] if shoulder is None else shoulder
    key_len = DT['key_L'] if key_len is None else key_len
    refuse(shoulder, key_len)
    g = _load('BagGripper')
    pn = _load('PartNumbers')
    # the gripper's own parts, changed where Chapter 50 decided: the jaws' driven ends, the bearings, the dowel
    plain_jaw, plain_xtube, plain_zcyl = g.jaw_bodies, g.xtube, g.zcyl
    g.xtube_plain = plain_xtube
    sh, lv, extra, info = detail_bodies(g, shoulder, key_len)

    def jaw_bodies():
        out = []
        for nm, tb, mat in plain_jaw():
            if nm in ('Stub shaft long', 'Lever'):
                tb = mp.tbm().copy(sh if nm == 'Stub shaft long' else lv)
            out.append((nm, tb, mat))
        return out + [(nm, mp.tbm().copy(tb), mat) for nm, tb, mat in extra]

    def xtube(x0, x1, ro, ri, y, z):
        if (x0, x1) == tuple(g.L['bearing_x']) and abs(2 * ro - DT['bore'][1]) < 1e-9:
            return skf_bearing(x0, x1, y, z)
        return plain_xtube(x0, x1, ro, ri, y, z)

    def zcyl(z0, z1, r, x=0.0, y=0.0):
        if (z0, z1, r, x, y) == (-DW['in_adapter0'], DW['in_tool'], 5.0, 80.0, 0.0):
            return plain_zcyl(-DW['in_adapter'], DW['in_tool'], r, x, y)       # the dowel and its hole
        return plain_zcyl(z0, z1, r, x, y)

    g.jaw_bodies, g.xtube, g.zcyl = jaw_bodies, xtube, zcyl
    g.build()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    # Chapter 49's numbers, then the detail's new kinds, then every bought part's full designation
    done, part_of, left = pn.number(root, 'bag')
    new = S.detail_numbers()
    known = D.number_map('bag')
    added, described = [], 0
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count == 0:
            continue
        kind = D.kind_of(c.name, new)
        if kind is not None:
            c.partNumber, c.description = new[kind]
            added.append(c.name)
            continue
        kind = D.kind_of(c.name, known)
        if kind is not None and c.partNumber.startswith('S-'):
            c.description = S.decide(known[kind][1])['full']
            described += 1
    _last.update(g=g, groups=g._last['groups'], info=info, shoulder=shoulder, key_len=key_len, added=added)
    nums = sorted({v[0] for v in new.values()})
    return ('Bag gripper detailed: the driven stub shafts with a %.1f mm shoulder (SKF: %.1f to %.1f), SKF 6206-2RS1, '
            'a spacer of %.0f, the lever\'s hub %.0f long with DIN 6885-1 A %d x %d x %.0f, an end washer %.0f x %.0f and '
            'ISO 4762 M%d x %d; the dowel ISO 8734 10 x %.0f, %.0f mm in the adapter. %d bought components described by '
            'their full designations; new parts %s.\n\n[%s]'
            % (shoulder, DT['da_min'], DT['da_max'], info['hub'][0] - DT['spacer'][0], info['boss'], DT['b'], DT['h'],
               key_len, DT['washer']['D'], DT['washer']['t'], DT['screw'][0], DT['screw'][1], DW['L'],
               DW['in_adapter'], described, ', '.join(nums), VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(ins.itemById('shoulder').value, ins.itemById('key').value), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('shoulder', 'The shaft\'s shoulder (mm)', '', adsk.core.ValueInput.createByReal(float(DT['shoulder'])))
        ins.addValueInput('key', 'The key\'s length (mm)', '', adsk.core.ValueInput.createByReal(DT['key_L']))
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
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Standard parts', 'Chapter 50: the bag gripper\'s standard parts')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StandardParts could not start:\n' + traceback.format_exc())
