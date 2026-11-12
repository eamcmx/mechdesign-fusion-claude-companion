# =============================================================================
# WristIntegration.py  -  Chapter 40 of "Mechanical Design in Autodesk Fusion with Claude": the pitch-roll wrist of
# Chapter 39 made ready for its arm. WristDifferential.py is loaded (not copied) and builds the wrist in a new design;
# then, with the joints at rest, the parts that change are deleted from their groups and the new ones added to the
# groups they move with:
#   Roll output   the roll shaft bored 5 mm through for the tool's wires; the tool flange drawn to
#                 ISO 9409-1-31.5-4-M5 and held by a DIN 471 ring on the shaft (the end washer and the M6 screw
#                 go); the slip ring's rotor on the shaft's lower end
#   Carrier       the slip ring's stator under the carrier, held from turning by a pin in a bracket screwed to the
#                 carrier block (which gets the screw's tapped hole); on the left (+y) side the trunnion longer, a
#                 spacer S5, a stop disc keyed to it with a lug on +z, the end washer and screw moved out to its end
#   Frame         on the left side the fixed bearing cap replaced by a stop cap (a block on -z) with four
#                 countersunk screws and two buffer pads
#
#   INPUTS        the dialog: the hard stop's contact angle (deg), 128
#   DERIVED       wrist_design.stops40() for that contact: the brakes' stopping angle and the block's half angle;
#                 FLANGE40, BORE40, SLIPRING, STOP40, KEY40 and LAY40: the flange's interface, the bore, the slip
#                 ring, the stop's radii and every new axial position; hollow_roll_shaft() and backlash40()
#   HARD RULES    build() refuses a contact the brakes would not stop the carrier short of, or one that leaves the
#                 block less than 10 deg either side of -z
#   CONVENTIONS   WristDifferential.py's axes, groups, As-built joints and motion links; the pitch's limits opened
#                 to +-140 deg so that the stops can be reached; every body keeps the material it had before the
#                 deletions. Stop parts are drawn in WristDifferential's side frame (local z = +y, local x = +z):
#                 an angle there runs from +z towards +x, which is the way a positive pitch turns the carrier
#   ACCEPTANCE    the message; 97 components in 43 kinds; the links; no interference at rest, at the poses and over
#                 +-127 deg of pitch; at +-129 deg the lug inside a pad and nothing else; the flange's faces; two
#                 refusals
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
import companion_paths as CP    # noqa: E402  every lib/ folder on sys.path; CP.script(name) finds a script
import wrist_design as wd       # noqa: E402
importlib.reload(wd)

VERSION = 'WristIntegration 1.0 (2026-11-11)'
CONTACT = 128.0                 # deg, the dialog's default
PITCH_OPEN = 140.0              # deg either way: the pitch joint's limits, so that the stops can be reached
BLOCK_MIN = 10.0                # deg, the least the block may keep either side of -z
FLANGE_NAME = 'Tool flange ISO 9409-1-31.5-4-M5'
# the slip ring's bracket (assumed, mm, world at rest): a 3 mm tab under the carrier block's bottom face, outside
# the lower 6002 (which stands below that face), a pad in the -x +y corner with one M3 screw and an arm along +y; a
# leg down from the arm beside the stator, carrying the pin that enters the stator from +y
BRACKET = dict(pad=((-19.0, -12.5), (11.5, 19.0)), arm=((-12.5, 4.0), (16.5, 19.0)), leg=((-4.0, 4.0), (16.5, 19.0)),
               t=3.0, leg_bottom=-42.0, screw=(-15.75, 15.25), pin_d=3.0, pin_in=2.0)
WD = None                       # WristDifferential.py, loaded by build()


def wrist_differential():
    """WristDifferential.py loaded from its own folder, afresh each time (Fusion keeps modules between runs)."""
    global WD
    spec = importlib.util.spec_from_file_location('WristDifferential', CP.script('WristDifferential'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    WD = mod
    return mod


# ---------- the design and its rules ----------
def stop_numbers(contact):
    """stops40() for this contact angle. It reads STOP40, which holds 128 deg, so the contact is set for the call."""
    kept = wd.STOP40
    try:
        wd.STOP40 = dict(kept, contact=float(contact))
        return wd.stops40()
    finally:
        wd.STOP40 = kept


def design(contact=CONTACT):
    c = float(contact)
    o = dict(contact=c)
    o['L'] = wd.layout()
    o['lay'] = wd.LAY40
    o['stop'] = s = stop_numbers(c)
    lh = wd.STOP40['lug'] / 2.0
    # angles from +z towards +x: the lug's leading edge meets the pad's face at the contact; the pad's back face is
    # the block's face
    o['ang'] = dict(lug=lh, pad=c + lh, block=180.0 - s['block_half'])
    o['shaft'] = wd.hollow_roll_shaft()
    o['solid'] = wd.roll_shaft()
    o['play'] = wd.backlash40()
    return o


def refuse(o):
    s, c = o['stop'], o['contact']
    short = []
    if s['stop_at'] >= c - 1e-9:
        short.append('the brakes stop the carrier only at %.2f deg (the switch at %.0f deg, %.2f deg in their %.0f ms '
                     'delay, %.2f deg braking), not short of the contact'
                     % (s['stop_at'], wd.STOP40['switch'], s['coast'], wd.STOP40['delay'] * 1000, s['brake']))
    if s['block_half'] < BLOCK_MIN - 1e-9:
        short.append('it leaves the block too short: %.2f deg either side of -z, where it needs %.0f'
                     % (s['block_half'], BLOCK_MIN))
    if short:
        raise ValueError('A hard stop with its contact at %.1f deg is refused: %s.' % (c, '; '.join(short)))


# ---------- temporary bodies (mm), WristDifferential's helpers ----------
def _half(th, side, z0, z1, S=100.0):
    """The half-space on one side of the plane through the local z axis at angle th (deg, from local +x towards +y):
    side +1 where the angle grows from it, -1 where it falls; z0..z1 along z."""
    t = math.radians(th)
    u = (math.cos(t), math.sin(t), 0.0)
    n = (-math.sin(t), math.cos(t), 0.0)
    c = (side * S * n[0], side * S * n[1], (z0 + z1) / 2.0)
    obb = adsk.core.OrientedBoundingBox3D.create(WD.P(*c), WD.V(*u), WD.V(*n), 2 * S * WD.MM, 2 * S * WD.MM,
                                                 (abs(z1 - z0) + 2.0) * WD.MM)
    return WD.tbm().createBox(obb)


def sector(z0, z1, r0, r1, th0, th1):
    """A ring sector about the local z axis, r0..r1, from th0 to th1 (deg, less than 180 apart), z0..z1."""
    b = WD.zcyl(z0, z1, r1)
    if r0 > 0:
        WD.cut(b, WD.zcyl(z0 - 1, z1 + 1, r0))
    return WD.inter(b, _half(th0, 1, z0, z1), _half(th1, -1, z0, z1))


def ring_groove(lay):
    """The groove for the flange's DIN 471-15 ring: LAY40's ring2 taken as the ring's own faces, 0.1 either side."""
    r0, r1 = lay['ring2']
    return r0 - 0.1, r1 + 0.1


def bored_shaft(L, lay):
    """The roll shaft as Chapter 39's, longer to LAY40's shaft_top, its end left plain (no screw), a second ring's
    groove under the flange's ring and the 5 mm bore through."""
    top = lay['shaft_top']
    s = WD.union(WD.zcyl(L['shaft_bot'], L['collar'][0] + 0.1, 7.5), WD.zcyl(L['collar'][0], L['collar'][1], 10.0),
                 WD.zcyl(L['collar'][1] - 0.1, top, 7.5))
    WD.cut(s, WD.ring(L['ring471'][0], L['ring471'][1], 7.15, 9.0))
    g0, g1 = ring_groove(lay)
    WD.cut(s, WD.ring(g0, g1, 7.15, 9.0))
    k0, k1 = L['key']
    WD.cut(s, WD.box(4.5, 10, -2.5, 2.5, k0 + 2.5, k1), WD.xcyl(4.5, 10, 2.5, 0, k0 + 2.5))
    WD.cut(s, WD.zcyl(L['shaft_bot'] - 1, top + 1, wd.BORE40 / 2))
    return s


def tool_flange40(L, lay):
    """ISO 9409-1-31.5-4-M5: 45 mm, 11 thick on the gear's hub; four M5 on 31.5, the 5 H7 pin hole on +X between
    two of them, the 20 H7 recess from the face carried down to the DIN 471 ring that holds the flange (a hollow
    centre no larger than the centring diameter); the bore 15 and the keyway below the ring."""
    F = wd.FLANGE40
    f0, f1 = L['flange']
    sh = lay['ring2'][0]                                   # the shoulder the ring bears on
    f = WD.zcyl(f0, f1, F['D'] / 2)
    WD.cut(f, WD.zcyl(f0 - 1, sh, 7.5), WD.box(6.0, 9.8, -2.5, 2.5, f0 - 1, sh), WD.zcyl(sh, f1 + 1, F['d2'] / 2))
    for a in (45, 135, 225, 315):
        x, y = WD.polar(F['d1'] / 2, a)
        WD.cut(f, WD.zcyl(f1 - 8, f1 + 1, F['thread'] / 2, x, y))      # M5 tapped, 8 deep as in Chapter 39
    x, y = WD.polar(F['d1'] / 2, 0)
    WD.cut(f, WD.zcyl(f1 - F['pin_depth'], f1 + 1, F['pin'] / 2, x, y))
    return f


def rotor(lay):
    z0, z1 = lay['rotor']
    return WD.cut(WD.zcyl(z0, z1, wd.SLIPRING['flange'][0]), WD.zcyl(z0 - 1, z1 + 1, wd.BORE40 / 2))


def stator(lay):
    """The capsule, D x L, with the hole the bracket's pin enters from +y (clear all round) and the wires' way out
    below."""
    z0, z1 = lay['stator']
    r = wd.SLIPRING['D'] / 2
    s = WD.zcyl(z0, z1, r)
    pr = BRACKET['pin_d'] / 2
    WD.cut(s, WD.ycyl(r - BRACKET['pin_in'] - 1.0, r + 0.5, pr + 0.15, 0.0, lay['pin_z']))
    WD.cut(s, WD.zcyl(z0 - 1, z0 + 6, wd.BORE40 / 2))
    return s


def slip_ring_bracket(lay, z_face):
    """The tab (corner pad and arm) under the carrier block's bottom face at z_face, the leg down from the arm
    beside the stator and the pin pressed into it, pointing at the roll axis."""
    B = BRACKET
    zt = z_face - B['t']
    (px, py), (ax, ay), (lx, ly) = B['pad'], B['arm'], B['leg']
    b = WD.union(WD.box(px[0], px[1], py[0], py[1], zt, z_face), WD.box(ax[0], ax[1], ay[0], ay[1], zt, z_face),
                 WD.box(lx[0], lx[1], ly[0], ly[1], B['leg_bottom'], zt + 0.01))
    r = wd.SLIPRING['D'] / 2
    WD.union(b, WD.ycyl(r - B['pin_in'], ly[0] + 0.5, B['pin_d'] / 2, 0.0, lay['pin_z']))
    sx, sy = B['screw']
    WD.cut(b, WD.zcyl(zt - 1, z_face + 1, 1.7, sx, sy))
    return b


def carrier40(L):
    """Chapter 39's carrier block with the bracket's M3 tapped hole in its bottom face. The face is taken from the
    block as drawn (its 45 deg cone ends it above the layout's z_cb); returns the block and the face's height."""
    c = WD.carrier_block(L, L['g'])
    z_face = c.boundingBox.minPoint.z / WD.MM
    sx, sy = BRACKET['screw']
    WD.cut(c, WD.zcyl(z_face - 1, z_face + 7, 1.5, sx, sy))              # M3 tapped (drawn at the nominal size)
    return c, z_face


def stop_trunnion(L, lay):
    """Chapter 39's trunnion, its end at LAY40's tr_end, with the keyseat for KEY40 under the lug (local +x = +z)."""
    t = WD.trunnion(dict(L, tr_end=lay['tr_end']))
    k = wd.KEY40
    k0, k1 = lay['key']
    x0, r = 7.5 - k['t1'], k['b'] / 2
    WD.cut(t, WD.box(x0, 10.0, -r, r, k0 + r, k1 + 1), WD.xcyl(x0, 10.0, r, 0, k0 + r))
    return t


def key_5x5x8(lay):
    k = wd.KEY40
    k0, k1 = lay['key']
    x0, r = 7.5 - k['t1'], k['b'] / 2
    return WD.union(WD.box(x0, x0 + k['h'], -r, r, k0 + r, k1), WD.xcyl(x0, x0 + k['h'], r, 0, k0 + r))


def stop_disc(lay, ang):
    """The hub and the lug (STOP40's 25 deg, from LAY40's lug_r to its tip) on +z, bored 15 with the keyway."""
    d0, d1 = lay['disc']
    r_hub, r_tip = lay['lug_r']
    b = WD.union(WD.zcyl(d0, d1, r_hub), sector(d0, d1, 0.0, r_tip, -ang['lug'], ang['lug']))
    WD.cut(b, WD.zcyl(d0 - 1, d1 + 1, 7.5), WD.box(6.0, 9.8, -2.5, 2.5, d0 - 1, d1 + 1))
    return b


def stop_cap(L, lay, ang):
    """The fixed cap's spigot ring (it still locates the frame bearing), a plate with a lobe under the block and the
    pads, the block on -z between the pads, a bore for the trunnion and the spacer, four countersinks for M4."""
    s = wd.STOP40
    y3, T = L['B3'][1], L['cheek'][1]
    p0, p1 = lay['cap_plate']
    b0, b1 = lay['block']
    c = WD.union(WD.ring(y3, T + 0.1, 13.0, 16.0), WD.zcyl(p0, p1, 22.0),
                 sector(p0, p1, 0.0, s['r_out'], ang['pad'] - 3.0, 360.0 - ang['pad'] + 3.0),
                 sector(b0 - 0.1, b1, s['r_in'], s['r_out'], ang['block'], 360.0 - ang['block']))
    WD.cut(c, WD.zcyl(y3 - 1, b0 + 1, 11.0))
    hh = (2.0 * 4 - 4) / 2.0                               # ISO 10642 M4: head 8, its cone 2 deep
    for a in (45, 135, 225, 315):
        x, y = WD.polar(18, a)
        WD.cut(c, WD.zcyl(p0 - 1, p1 + 1, 2.25, x, y), WD.zcone(p1 - hh, 2.25, p1 + 0.5, 4.75, x, y))
    return c


def buffer_pad(lay, ang):
    """STOP40's pad: r_in to r_out, the block's width along the axis, from the contact face to the block's face."""
    s = wd.STOP40
    b0, b1 = lay['block']
    return sector(b0, b1, s['r_in'], s['r_out'], ang['pad'], ang['block'])


def side_turned(deg):
    """The +y side frame turned by deg about its own z (the pitch axis)."""
    X, Y, Z = WD.SIDE[1]
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return WD.frame(tuple(c * X[i] + s * Y[i] for i in range(3)), tuple(-s * X[i] + c * Y[i] for i in range(3)), Z)


# ---------- deleting and adding ----------
def occurrences(comp, name, plus_y=None):
    """The occurrences of a group's part called `name`; plus_y True keeps those on the +y side (local z along +y)."""
    out = []
    for i in range(comp.occurrences.count):
        oc = comp.occurrences.item(i)
        if oc.component.name != name:
            continue
        if plus_y is not None and (oc.transform2.getCell(1, 2) > 0.5) != plus_y:
            continue
        out.append(oc)
    return out


GONE = [('roll', 'Roll shaft', None, 1), ('roll', FLANGE_NAME, None, 1), ('roll', 'End washer 18x6.4x2', None, 1),
        ('roll', 'Low-head screw DIN 7984 M6x12', None, 1),
        ('carrier', 'Carrier block', None, 1), ('carrier', 'Trunnion', True, 1),
        ('carrier', 'End washer 18x5.5x2.5', True, 1), ('carrier', 'Cap screw ISO 4762 M5x12', True, 1),
        ('frame', 'Bearing cap fixed', None, 1), ('frame', 'Cap screw ISO 4762 M4x12', True, 4)]
REPLACED = (FLANGE_NAME, 'Carrier block')         # new parts under the old names: the old ones renamed first


def looks(des):
    out = {}
    for comp in des.allComponents:
        for i in range(comp.bRepBodies.count):
            b = comp.bRepBodies.item(i)
            try:
                m, a = b.material, b.appearance
                out[(comp.name, b.name)] = (m, m.name if m else '', a, a.name if a else '')
            except Exception:
                pass
    return out


def _assign(body, attr, first, second):
    for v in (first, second):
        if v is None:
            continue
        try:
            setattr(body, attr, v)
            return True
        except Exception:
            pass
    return False


def keep_looks(app, des, before, made):
    """Every body of a part that stays gets back the material (and the appearance) it had before the deletions."""
    n = 0
    for comp in des.allComponents:
        if comp.name in made:
            continue
        for i in range(comp.bRepBodies.count):
            b = comp.bRepBodies.item(i)
            k = (comp.name, b.name)
            if k not in before:
                continue
            m, mn, a, an = before[k]
            if mn and (b.material is None or b.material.name != mn):
                n += _assign(b, 'material', m, WD._find(app, 'm', mn))
            if an and (b.appearance is None or b.appearance.name != an):
                _assign(b, 'appearance', a, WD._find(app, 'a', an))
    return n


def integrate(app, des, o):
    W = WD
    L, lay, ang = o['L'], o['lay'], o['ang']
    occ = W._last['occ']
    G = dict(roll=occ['roll'].component, carrier=occ['carrier'].component, frame=occ['frame'].component)
    before = looks(des)
    # --- the parts that change, deleted from their groups (the joints at rest)
    doomed = []
    for grp, name, plus_y, n in GONE:
        found = occurrences(G[grp], name, plus_y)
        if len(found) != n:
            raise RuntimeError('expected %d of %s in %s, found %d' % (n, name, G[grp].name, len(found)))
        doomed += found
    for oc in doomed:
        if oc.component.name in REPLACED:
            oc.component.name = oc.component.name + ' (Chapter 39)'
    for oc in doomed:
        if not oc.deleteMe():
            raise RuntimeError('could not delete an occurrence of %s' % oc.component.name)
    washer = occurrences(G['carrier'], 'End washer 18x5.5x2.5')[0].component
    m5 = occurrences(G['carrier'], 'Cap screw ISO 4762 M5x12')[0].component
    # --- the new ones, each added to the group it moves with
    B = W.Builder(app, des.rootComponent)
    R, C, F = G['roll'], G['carrier'], G['frame']
    r2 = lay['ring2']
    B.part(R, 'shaft40', 'Roll shaft, bored', [(bored_shaft(L, lay), 'roll shaft', 'steel')], W.ident())
    B.part(R, 'flange40', FLANGE_NAME, [(tool_flange40(L, lay), 'tool flange', 'stainless')], W.ident())
    B.part(R, 'ring40', 'Retaining ring DIN 471-15, flange', [(WD.ring(r2[0], r2[1], 7.15, 9.6), 'ring', 'screw')],
           W.ident())
    B.part(R, 'rotor', 'Slip ring rotor', [(rotor(lay), 'slip ring rotor', 'alu')], W.ident())
    block, z_face = carrier40(L)
    B.part(C, 'carrier40', 'Carrier block', [(block, 'carrier block', 'carrier')], W.ident())
    B.part(C, 'trunnion40', 'Trunnion with stop end', [(stop_trunnion(L, lay), 'trunnion', 'steel')], W.side(1))
    B.part(C, 's5', 'Spacer S5', [(WD.ring(lay['S5'][0], lay['S5'][1], 7.5, 9.0), 'spacer', 'steel')], W.side(1))
    B.part(C, 'disc', 'Stop disc', [(stop_disc(lay, ang), 'stop disc', 'steel')], W.side(1))
    B.part(C, 'key8', 'Key DIN 6885 A 5x5x8', [(key_5x5x8(lay), 'key', 'steel')], W.side(1))
    B.inst(C, washer, W.side(1, 0, 0, lay['tr_end'] - L['tr_end']))          # Chapter 39's washer, at the new end
    B.inst(C, m5, W.side(1, 0, 0, lay['tr_end'] + 2.5))                       # and its screw
    B.part(C, 'stator', 'Slip ring stator', [(stator(lay), 'slip ring stator', 'black')], W.ident())
    B.part(C, 'srbracket', 'Slip ring bracket', [(slip_ring_bracket(lay, z_face), 'slip ring bracket', 'alu')],
           W.ident())
    sx, sy = BRACKET['screw']
    B.screw(C, 3, 8, W.frame(*W.DOWN, T=(sx, sy, z_face - BRACKET['t'])))      # 3 in the tab, 5 in the block
    B.part(F, 'stopcap', 'Stop cap', [(stop_cap(L, lay, ang), 'stop cap', 'frame')], W.side(1))
    for a in (45, 135, 225, 315):
        x, y = W.polar(18, a)
        B.screw(F, 4, 12, W.side(1, x, y, lay['cap_plate'][1]), kind='csk')
    B.part(F, 'pad', 'Buffer pad', [(buffer_pad(lay, ang), 'pad', 'belt')], W.side(1))
    B.inst(F, B.parts['pad'], side_turned(-(ang['pad'] + ang['block'])))     # its mirror image on the -x side
    for comp in (R, C, F):
        W.ground_children(comp)
    made = set(REPLACED) | set(c.name for c in B.parts.values())
    return keep_looks(app, des, before, made)


def parts_list(root):
    return (WD or wrist_differential()).parts_list(root)


_last = {}


def pose(pitch, roll):
    """Pitch and roll in degrees, through WristDifferential's joints and links."""
    WD.pose(pitch, roll)


def build(contact=CONTACT):
    o = design(contact)
    refuse(o)
    W = wrist_differential()
    W.build()                                       # Chapter 39's wrist, in a new design
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    W.pose(0.0, 0.0)
    integrate(app, des, o)
    lim = W._last['joints']['pitch'].jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = math.radians(-PITCH_OPEN)
    lim.isMaximumValueEnabled = True
    lim.maximumValue = math.radians(PITCH_OPEN)
    _last.clear()
    _last.update(W._last)
    _last['o40'] = o
    pose(0.0, 0.0)
    app.activeViewport.fit()
    pl = parts_list(root)
    s, sh, so, py = o['stop'], o['shaft'], o['solid'], o['play']
    S, F, SR = wd.STOP40, wd.FLANGE40, wd.SLIPRING
    rows = dict((n, mm) for n, _, mm in py['rows'])
    return ('Pitch-roll wrist made ready for its arm.\n'
            'Tool flange ISO 9409-1-31.5-4-M5: four M5 on %.1f, a %.0f H7 pin hole on +X, the %.0f H7 recess carried '
            'down to a DIN 471-15 ring at %.2f mm that holds it; no screw in the shaft\'s end.\n'
            'Roll shaft bored %.0f mm through: the payload on it at %.1f Hz (%.1f solid; %.0f wanted). Slip ring: %d '
            'circuits of %.0f A to %.0f rev/min, the rotor on the shaft\'s lower end, the stator under the carrier.\n'
            'Pitch stops, contact at %.1f deg: soft limit %.0f deg, switch %.0f deg; %.2f deg coasting in the brakes\' '
            '%.0f ms and %.2f deg braking stop the carrier at %.2f deg. Block %.2f deg either side of -z, pads %.2f deg '
            '(%.0f mm at r %.0f). The hard stop alone: %.3f J, a buffer of %.0f N m/rad, %.1f N m and %.0f N at its '
            'peak (%.2f MPa on the pad), the key %.0f MPa in shear and %.0f MPa on its sides, the trunnion %.1f MPa; '
            'the lug stops at %.2f deg.\n'
            'Play at the payload\'s centre, %.0f mm from the pitch axis: gearheads %.3f, belt %.3f, bevel set %.3f mm, '
            '%.2f mm in all (%.2f with 3 arcmin gearheads).\n'
            'Pitch limits opened to +-%.0f deg.\n\nParts: %d components in %d kinds; %s.\n\n[%s]'
            % (F['d1'], F['pin'], F['d2'], o['lay']['ring2'][0],
               wd.BORE40, sh['f'], so['f'], wd.brief()['f_min'], SR['circuits'], SR['amps'], SR['rpm'],
               o['contact'], S['soft'], S['switch'], s['coast'], S['delay'] * 1000, s['brake'], s['stop_at'],
               s['block_half'], s['pad_deg'], S['pad'], S['r'], s['E'], s['k'], s['T_pk'], s['F'], s['p'],
               s['tau_key'], s['sig_key'], s['tau_tr'], s['end'],
               py['r_p'], rows['gearhead'], rows['belt'], rows['bevel set'], py['at_tool'], py['precise'],
               PITCH_OPEN, sum(pl.values()), len(pl), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())),
               VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('contact', 'Hard stop\'s contact angle (deg)', CONTACT)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(**{k: ins.itemById(k).value for k, _, _ in FIELDS}), VERSION)
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
        cid = 'WristIntegrationCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Wrist ready for its arm',
                                                         'The pitch-roll wrist of Chapter 40: tool flange, slip ring, '
                                                         'hard stops')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('WristIntegration could not start:\n' + traceback.format_exc())
