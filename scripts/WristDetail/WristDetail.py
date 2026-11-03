# =============================================================================
# WristDetail.py  -  Chapter 52 of "Mechanical Design in Autodesk Fusion with Claude": the robotic wrist of Chapter 40
# in detail. It builds Chapter 40's wrist with WristIntegration.py, loaded, not copied, with the bought products drawn
# to their makers' figures (lib/wrist_detail.py: the Beckhoff AM8111 with its brake, the PLE040 gear unit at 15:1,
# the Senring slip ring with its flange), the bearings' abutments brought inside SKF's bands, the stop disc's key the
# 5 x 5 x 10 that is made, and then gives every made part its material and every fitting face its class and roughness
# as Chapter 51's MaterialsFits.py does, the front side-gear bearings' housings to their own load class.
# Written from PROMPT.txt beside it; checked by checks/wrist_detail_check.py.
#
#   INPUTS        the dialog: the hard stop's contact angle (128 deg), as Chapter 40's
#   DERIVED       wrist_detail: products(), abutments(), load_classes(); wrist_design: layout(), LAY40
#   HARD RULES    Chapter 40's refusals; nothing of WristIntegration's or MaterialsFits' code copied
#   CONVENTIONS   as WristIntegration.py; the changed parts keep their groups and their names, the key renamed
#   ACCEPTANCE    the message; the check: the products' sizes, the abutments, the key, the faces' classes, no
#                 interference at rest and over the range
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
SCRIPTS = os.path.dirname(HERE)
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_wd52_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


WI = _load('WristIntegration')          # loads WristDifferential and reloads wrist_design
MF = _load('MaterialsFits')
import wrist_design as wd               # noqa: E402  (the module WI and WD use, after their reloads)
import wrist_detail as WDET             # noqa: E402
importlib.reload(WDET)
WD = WI.WD

VERSION = 'WristDetail 1.0 (2026-11-03)'
_last = {}
pose = WI.pose           # the joints as Chapter 40 drives them


# ---------- the products, drawn to their makers' figures ----------
def gearmotor_product(L):
    """The motor and its gear unit as their makers draw them: the unit's square 40 flange, 5 thick, at |y| = gh_face,
    its round body 40 across to the adapter, the adapter square 40, 28.5 long; the AM8111 square 40, 129 long with its
    brake, and its connector."""
    G, Mo = WDET.GEARHEAD, WDET.MOTOR
    xm, zm = L['x_m'], L['z_m']
    F = Mo['flange'] / 2
    y = L['gh_face']
    fl = WD.union(WD.box(xm - F, xm + F, y - 5, y, zm - F, zm + F), WD.ycyl(y - 0.1, y + 2.0, G['spigot'] / 2, xm, zm))
    WD.cut(fl, WD.ycyl(y - 6, y + 3, G['shaft'] / 2 + 0.2, xm, zm))
    for dx in (-16, 16):
        for dz in (-16, 16):
            WD.cut(fl, WD.ycyl(y - 6, y + 1, 2.0, xm + dx, zm + dz))
    y_ad = y - G['body']
    gh = WD.ycyl(y_ad, y - 5, G['D'] / 2, xm, zm)
    y_m = y_ad - G['adapter']
    ad = WD.box(xm - F, xm + F, y_m, y_ad, zm - F, zm + F)
    y_e = y_m - Mo['k']
    mo = WD.box(xm - F, xm + F, y_e, y_m, zm - F, zm + F)
    cn = WD.box(xm - 6, xm + 6, y_e + 6, y_e + 18, zm + F, zm + F + 6)
    return [(fl, 'gearhead flange', 'alu'), (gh, 'planetary gearhead PLE040 15:1', 'dark'),
            (ad, 'motor adapter', 'dark'), (mo, 'servo motor AM8111 with brake', 'stator'), (cn, 'connector', 'black')]


def motor_output_product(L):
    """Chapter 39's 18-groove pulley and two set screws on the PLE040's smooth 12 mm shaft, 26 long."""
    items = _orig['motor_output'](L)
    xm, zm = L['x_m'], L['z_m']
    r = WDET.GEARHEAD['shaft'] / 2
    s0 = L['gh_face'] - 5.0
    s1 = L['gh_face'] + WDET.GEARHEAD['shaft_L']
    out = []
    for tb, name, look in items:
        if name.startswith('pulley'):
            WD.cut(tb, WD.ycyl(s0 - 1, s1 + 1, r, xm, zm))
        if name.startswith('gearhead output shaft'):
            tb, name = WD.ycyl(s0, s1, r, xm, zm), 'gearhead output shaft 12'
        if name.startswith('set screw'):
            continue
        out.append((tb, name, look))
    ys = (L['m_hub'][0] + L['m_hub'][1]) / 2
    out.append((WD.cyl((xm, ys, zm + r), (xm, ys, zm + r + 4.0), 2.0), 'set screw M4', 'screw'))
    out.append((WD.cyl((xm + r, ys, zm), (xm + r + 4.0, ys, zm), 2.0), 'set screw M4', 'screw'))
    return out


def slip_parts_product(K):
    """Chapter 40's rotor flange; the SNM022A-06's stator, 22 across and 19.5 long with its 44.5 mm flange at the top,
    against the bracket's tab; the tab alone, the bar and the pin no longer needed, the stator held by its flange."""
    S = WDET.SLIPRING
    rotor, _, _, tab_old = _orig['slip_parts'](K)
    top = K['rotor'][0]                                 # the rotor flange's underside: the tab's underside too
    stator = WD.union(WD.zcyl(top - S['L'], top - 2.0 + 0.01, S['D'] / 2),
                      WD.cut(WD.zcyl(top - 2.0, top, S['flange'] / 2), WD.zcyl(top - 3.0, top + 1.0, 8.0)))
    WD.cut(stator, WD.zcyl(top - 3.0, top + 1.0, 1.7, 17.5, 9.5))       # the bracket screw through the flange
    tab = WD.box(13.0, 19.0, 6.0, 17.0, -25.5, -22.5)
    WD.cut(tab, WD.zcyl(-26.0, -22.0, 1.7, 17.5, 9.5))
    pin = None                                         # no pin: part_detail() leaves it out of the stator's part
    return rotor, stator, pin, tab


def motor_bracket_product(L):
    """Chapter 39's bracket with its middle slot opened from 24 to 27 mm across, so that the PLE040's 26 h7 spigot
    passes it with 0.5 mm a side over the slot's 3 mm of travel for the belt's tension; its four slots for the unit's
    screws as before, at the pattern Chapter 39 drew (the maker's drawing gives the unit's own)."""
    b = _orig['motor_bracket'](L)
    xm, zm = L['x_m'], L['z_m']
    y0, y1 = L['bracket']
    r, dz = (WDET.GEARHEAD['spigot'] + 1.0) / 2, 3.0
    WD.cut(b, WD.union(WD.ycyl(y0 - 1, y1 + 1, r, xm, zm - dz), WD.ycyl(y0 - 1, y1 + 1, r, xm, zm + dz),
                       WD.box(xm - r, xm + r, y0 - 1, y1 + 1, zm - dz, zm + dz)))
    return b


# ---------- the abutments: the collar and the spacers S4 and S5 at 18 mm ----------
def part_detail(self, parent, key, name, items, M=None):
    """The slip ring's stator made without Chapter 40's pin, which its flange makes needless."""
    if key == 'stator':
        items = [it for it in items if it[1] != 'pin']
    return _orig['part'](self, parent, key, name, items, M)


def frame_detail(*args, **kw):
    """The bracket's screw, from under the slip ring's flange now, 2 mm lower, and 10 long (Builder.screw below)."""
    if kw.get('T') == (17.5, 9.5, -25.5):
        kw['T'] = (17.5, 9.5, -25.5 - 2.0)
        _flag['bracket'] = True
    return _orig['frame'](*args, **kw)


def screw_detail(self, parent, d, L, M, *a, **kw):
    if _flag.pop('bracket', False) and (d, L) == (3, 8):
        L = 10
    return _orig['screw'](self, parent, d, L, M, *a, **kw)


_flag = {}


def ring_detail(z0, z1, ri, ro, x=0.0, y=0.0):
    if (ri, ro) == (7.5, 9.5):                          # spacers S4 and S5, 19 mm: over the 6002's da max
        ro = WDET.DETAIL_ABUT / 2
    return _orig['ring'](z0, z1, ri, ro, x, y)


def zcyl_detail(z0, z1, r, x=0.0, y=0.0):
    L = _lay
    if r == 10.0 and (z0, z1) == tuple(L['collar']):    # the roll shaft's collar, 20 mm, between the two 6002s
        r = WDET.DETAIL_ABUT / 2
    return _orig['zcyl'](z0, z1, r, x, y)


_orig = {}
_lay = {}


def _patch():
    _orig.update(ring=WD.ring, zcyl=WD.zcyl, gearmotor=WD.gearmotor, motor_output=WD.motor_output,
                 slip_parts=WI.slip_parts, frame=WD.frame, screw=WD.Builder.screw, motor_bracket=WD.motor_bracket,
                 part=WD.Builder.part)
    WD.Builder.part = part_detail
    WD.ring, WD.zcyl, WD.frame, WD.Builder.screw = ring_detail, zcyl_detail, frame_detail, screw_detail
    WD.motor_bracket = motor_bracket_product
    WD.gearmotor, WD.motor_output = gearmotor_product, motor_output_product
    WI.slip_parts = slip_parts_product


def _unpatch():
    WD.ring, WD.zcyl, WD.frame, WD.Builder.screw = _orig['ring'], _orig['zcyl'], _orig['frame'], _orig['screw']
    WD.motor_bracket = _orig['motor_bracket']
    WD.Builder.part = _orig['part']
    WD.gearmotor, WD.motor_output = _orig['gearmotor'], _orig['motor_output']
    WI.slip_parts = _orig['slip_parts']


def lay_detail():
    """Chapter 40's layout with the stop disc 2 mm longer outward for the 10 mm key: the disc 71 to 82, the key 72 to
    82, the trunnion's end at 82; the slip ring's stator from its flange down 19.5."""
    K = dict(wd.LAY40)
    K.update(disc=(71.0, 82.0), key=(72.0, 82.0), tr_end=82.0,
             stator=(K['rotor'][0] - WDET.SLIPRING['L'], K['rotor'][0]))
    return K


def build(contact=128.0):
    app = adsk.core.Application.get()
    saved_lay, saved_key = dict(wd.LAY40), dict(wd.KEY40)
    with WDET.products():
        _lay.clear()
        _lay.update(wd.layout())
        wd.LAY40.update(lay_detail())
        wd.KEY40.update(L=10.0)
        _patch()
        try:
            msg40 = WI.build(contact)
        finally:
            _unpatch()
            wd.LAY40.clear()
            wd.LAY40.update(saved_lay)
            wd.KEY40.clear()
            wd.KEY40.update(saved_key)
        des = adsk.fusion.Design.cast(app.activeProduct)
        root = des.rootComponent
        renamed = 0
        for o in list(root.allOccurrences):
            c = o.component
            if c.name == 'Key DIN 6885 A 5x5x8':
                c.name = 'Key DIN 6885 A 5x5x10'
                renamed += 1
        L = wd.layout()
        numbers = WDET.numbers()[1]
    msg51 = MF.apply('wrist')
    front = front_bearings(root, L)
    _last.update(L=L, numbers=numbers, front=front, mf=dict(MF._last), renamed=renamed)
    a = numbers
    return ('Wrist in detail: the AM8111 with its brake and the PLE040 at 15:1 drawn to their makers\' figures, each '
            'motor\'s end %.1f mm off the middle; the SNM022A-06 with its 44.5 mm flange; the collar and spacers S4 and '
            'S5 at %.0f mm; the key 5 x 5 x 10.\nMotor %.3f N m rms of %.2f rated, %.3f peak of %.2f; the stop by %.2f deg; '
            'play at the tool %.3f mm; the wrist %.3f kg.\nFront side-gear housings %s: %d faces.\n%s\n\n[%s]'
            % (a['m_end'], WDET.DETAIL_ABUT, a['motors']['rms'], WDET.MOTOR['rated'], a['motors']['peak'],
               WDET.MOTOR['peak'], a['stops']['stop_at'], a['play']['at_tool'], a['mass']['m'], FRONT_CLS, len(front),
               msg51.split('\n')[0], VERSION))


FRONT_CLS = 'M7'      # SKF table 4: rotating outer ring load, light (P <= 0.05 C), the front 61902s (lib/wrist_detail)


def front_bearings(root, L):
    """The side gears' bores for the front 61902s (|y| from zff to zff + 7, 28 across): their load is light, so the
    housing is M7, not Chapter 51's N7 for a normal load; the attribute and the colour stay those of a bearing seat."""
    lo, hi = L['B1']
    out = []
    for o in root.allOccurrences:
        if not o.component.name.startswith('Side bevel gear'):
            continue
        for b in o.bRepBodies:
            for f in b.faces:
                g = f.geometry
                if isinstance(g, adsk.core.Cylinder) and abs(g.radius - 1.4) < 1e-4:
                    bb = f.boundingBox
                    ya, yb = sorted((abs(bb.minPoint.y * 10), abs(bb.maxPoint.y * 10)))
                    if abs(ya - lo) < 0.05 and abs(yb - hi) < 0.05:
                        f.attributes.add('mdfc', 'fit', '%s Ra 3.2' % FRONT_CLS)
                        out.append(f)
    return out


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            v = args.command.commandInputs.itemById('contact').value
            ui.messageBox(build(v), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        cmd.commandInputs.addValueInput('contact', 'Hard stop contact angle (deg)', '',
                                        adsk.core.ValueInput.createByReal(128.0))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    ui = adsk.core.Application.get().userInterface
    try:
        cid = 'WristDetailCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Wrist in detail', 'Chapter 52: the wrist in detail')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('WristDetail could not start:\n' + traceback.format_exc())
