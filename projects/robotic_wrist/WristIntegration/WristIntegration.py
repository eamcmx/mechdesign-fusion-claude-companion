# =============================================================================
# WristIntegration.py  -  Chapter 40 of "Mechanical Design in Autodesk Fusion with Claude": the pitch-roll wrist of
# Chapter 39 made ready for its arm. It builds Chapter 39's wrist with WristDifferential.py, loaded, not copied, and
# then changes four things: the tool flange is drawn to ISO 9409-1-31.5-4-M5 and held by a retaining ring, the roll
# shaft is bored for the tool's wires, a slip ring under the carrier carries them across the roll, and the left
# trunnion carries a stop disc that meets a block on a new cap, with two buffer pads, at 128 deg of pitch. Every size
# comes from lib/wrist_design.py (layout() and LAY40). Written from PROMPT.txt beside it; checked by
# checks/wrist_integration_check.py.
#
#   INPUTS        the dialog: the hard stop's contact angle (128 deg)
#   DERIVED       wrist_design.stops40(): the block's half angle from the contact angle, the lug and the pads
#   HARD RULES    build() refuses a contact angle the brakes would not stop short of, or one that leaves no block
#   CONVENTIONS   as WristDifferential.py; the new parts go into the groups they move with, at rest
#   ACCEPTANCE    the message; no interference at rest and over the range; the lug on the pads at the contact angle;
#                 the masses measured
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
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import wrist_design as wd       # noqa: E402
importlib.reload(wd)


def _load(name, script):
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


WD = _load('mdfc_wrist_differential', 'WristDifferential')

VERSION = 'WristIntegration 1.0 (2026-11-02)'


def design(contact=128.0):
    wd.STOP40['contact'] = float(contact)
    o = dict(contact=float(contact))
    o['stop'] = wd.stops40()
    o['L'] = wd.layout()
    o['shaft'] = wd.hollow_roll_shaft()
    o['play'] = wd.backlash40()
    return o


def refuse(o):
    s = o['stop']
    short = []
    if s['stop_at'] >= o['contact']:
        short.append('the brakes stop the carrier at %.1f deg, at or past the contact at %.1f' % (s['stop_at'], o['contact']))
    if s['block_half'] < 10.0:
        short.append('a block of %.1f deg either side, too short to hold its pads' % s['block_half'])
    if short:
        raise ValueError('This stop falls short: %s.' % '; '.join(short))


# ---------- new parts, drawn with WristDifferential's helpers (mm) ----------
def sector(z0, z1, r0, r1, a_lo, a_hi):
    """A ring sector from angle a_lo to a_hi (deg, a_hi - a_lo under 180) about local z."""
    ring = WD.ring(z0, z1, r0, r1)
    h1 = WD.rotz(WD.box(-200, 200, 0, 200, z0 - 1, z1 + 1), a_lo)
    h2 = WD.rotz(WD.box(-200, 200, 0, 200, z0 - 1, z1 + 1), a_hi - 180.0)
    return WD.inter(ring, h1, h2)


def trunnion_stop(L, K):
    """The left trunnion, longer, with a keyseat for the stop disc on local +x (world +z) and an M5 end."""
    y0, y1 = L['y_fl']
    end = K['tr_end']
    t = WD.union(WD.zcyl(L['tr_spigot'], y0 + 0.1, 7.5), WD.zcyl(y0, y1, 15.0), WD.zcyl(y1 - 0.1, end, 7.5))
    for ang in (45, 135, 225, 315):
        x, y = WD.polar(10.0, ang)
        WD.cut(t, WD.zcyl(y0 - 1, y1 + 1, 1.7, x, y), WD.zcone(y1 - 1.5, 1.7, y1 + 0.01, 3.1, x, y))
    WD.cut(t, WD.zcyl(end - 10, end + 1, 2.5))
    k0, k1 = K['key']
    WD.cut(t, WD.box(4.5, 10, -2.5, 2.5, k0, k1 + 1))
    return t


def stop_disc(K):
    z0, z1 = K['disc']
    r0, r1 = K['lug_r']
    half = wd.STOP40['lug'] / 2
    d = WD.union(WD.ring(z0, z1, 7.5, 12.0), sector(z0, z1, r0, r1, -half, half))
    WD.cut(d, WD.box(6.0, 9.8, -2.5, 2.5, z0 - 1, z1 + 1))
    return d


def stop_cap(L, K, s):
    """The new fixed cap: its spigot clamps the 6002's outer ring as the old one did; a plate 72 mm across, held by
    four countersunk M4; the block on its face, centred on local -x (world -z), its faces at 180 +- block_half."""
    y3 = L['B3'][1]
    T = L['cheek'][1]
    p0, p1 = K['cap_plate']
    c = WD.union(WD.ring(y3, T + 0.1, 13.0, 16.0), WD.ring(p0, p1, 10.0, wd.STOP40['r_out']))
    b0, b1 = K['block']
    WD.union(c, sector(p1 - 0.1, b1, wd.STOP40['r_in'], wd.STOP40['r_out'], 180 - s['block_half'], 180 + s['block_half']))
    for a in (45, 135, 225, 315):
        x, y = WD.polar(18, a)
        WD.cut(c, WD.zcyl(p0 - 1, p1 + 1, 2.25, x, y), WD.zcone(p1 - 2.2, 2.25, p1 + 0.01, 4.45, x, y))
    return c


def pads(K, s):
    b0, b1 = K['block']
    lo = 180 - s['block_half']
    hi = 180 + s['block_half']
    r0, r1 = wd.STOP40['r_in'], wd.STOP40['r_out']
    return (sector(b0, b1, r0, r1, lo - s['pad_deg'], lo), sector(b0, b1, r0, r1, hi, hi + s['pad_deg']))


def roll_shaft_hollow(L, K):
    s = WD.union(WD.zcyl(L['shaft_bot'], L['collar'][0] + 0.1, 7.5), WD.zcyl(L['collar'][0], L['collar'][1], 10.0),
                 WD.zcyl(L['collar'][1] - 0.1, K['shaft_top'], 7.5))
    WD.cut(s, WD.ring(L['ring471'][0], L['ring471'][1], 7.15, 9.0))
    r0, r1 = K['ring2']
    WD.cut(s, WD.ring(r0 - 0.05, r1 + 0.05, 7.15, 9.0))
    k0, k1 = L['key']
    WD.cut(s, WD.box(4.5, 10, -2.5, 2.5, k0 + 2.5, k1), WD.xcyl(4.5, 10, 2.5, 0, k0 + 2.5))
    WD.cut(s, WD.zcyl(L['shaft_bot'] - 1, K['shaft_top'] + 1, wd.BORE40 / 2))
    return s


def flange_iso(L):
    """ISO 9409-1-31.5-4-M5: four M5 on 31.5, a 5 H7 pin hole on +x between two of them, the 20 H7 centring recess,
    3 deep for a 2.5 mm spigot, carried on down to the retaining ring; the shaft's bore and keyway below."""
    F = wd.FLANGE40
    f0, f1 = L['flange']
    top = L['shaft_top']
    f = WD.zcyl(f0, f1, F['D'] / 2)
    WD.cut(f, WD.zcyl(f0 - 1, top, 7.5), WD.zcyl(top - 0.01, f1 + 1, F['d2'] / 2), WD.box(6.0, 9.8, -2.5, 2.5, f0 - 1, top))
    r = F['d1'] / 2
    for a in (45, 135, 225, 315):
        x, y = WD.polar(r, a)
        WD.cut(f, WD.zcyl(f1 - 8, f1 + 1, F['thread'] / 2, x, y))
    WD.cut(f, WD.zcyl(f1 - F['pin_depth'], f1 + 1, F['pin'] / 2, r, 0.0))
    return f


def slip_parts(K):
    """The slip ring's rotor flange (on the roll shaft's lower end), its stator, the stator's pin and the bracket
    that stops the stator turning: a tab under the carrier and a bar with a hole for the pin."""
    r0, r1 = K['rotor']
    rotor = WD.cut(WD.zcyl(r0, r1, 7.5), WD.zcyl(r0 - 1, r1 + 1, wd.BORE40 / 2))
    s0, s1 = K['stator']
    stator = WD.zcyl(s0, s1, wd.SLIPRING['D'] / 2)
    zp = K['pin_z']
    c45 = math.sqrt(0.5)
    pin = WD.cyl((11.0 * c45, 11.0 * c45, zp), (21.0 * c45, 21.0 * c45, zp), 1.5)
    tab = WD.box(13.0, 19.0, 6.0, 17.0, -25.5, -22.5)
    bar = WD.box(13.0, 17.0, 13.0, 17.0, -44.0, -25.5 + 0.01)
    WD.union(tab, bar)
    WD.cut(tab, WD.cyl((8.0 * c45, 8.0 * c45, zp), (40.0 * c45, 40.0 * c45, zp), 1.8))
    WD.cut(tab, WD.zcyl(-26.0, -22.0, 1.7, 17.5, 9.5))
    return rotor, stator, pin, tab


# ---------- building ----------
def _side(o):
    """+1 for a part on the left (+y), -1 on the right: from the part's own placement, its local z along +-y for the
    side-axis parts, else its origin. (After a deletion Fusion reports empty bounding boxes until it recomputes.)"""
    m = o.transform2
    zy = m.getCell(1, 2)
    if abs(zy) > 0.5:
        return 1 if zy > 0 else -1
    return 1 if m.translation.y > 0 else -1


def _record_looks(design):
    rec = {}
    for comp in design.allComponents:
        for body in comp.bRepBodies:
            try:
                rec[(comp.name, body.name)] = (body.material.name if body.material else None,
                                               body.appearance.name if body.appearance else None)
            except Exception:
                pass
    return rec


def _restore_looks(app, design, rec):
    """Deleting parts can delete a design material that other bodies shared (Chapter 38): give every body back the
    material and the look it had."""
    for comp in design.allComponents:
        for body in comp.bRepBodies:
            m, a = rec.get((comp.name, body.name), (None, None))
            if m and (body.material is None or body.material.name != m):
                mm = WD._find(app, 'm', m)
                if mm:
                    try:
                        body.material = mm
                    except Exception:
                        pass
            if a and (body.appearance is None or body.appearance.name != a):
                aa = WD._find(app, 'a', a)
                if aa:
                    try:
                        body.appearance = aa
                    except Exception:
                        pass


def cut_into(group_occ, name, tb):
    """Cut a temporary body (world mm, the group at rest at identity) out of a part already built."""
    for o in group_occ.childOccurrences:
        if o.component.name == name:
            comp = o.component
            m = o.transform2.copy()
            m.invert()
            WD.tbm().transform(tb, m)
            target = comp.bRepBodies.item(0)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            comp.bRepBodies.add(tb, bf)
            bf.finishEdit()
            tools = adsk.core.ObjectCollection.create()
            tools.add(bf.bodies.item(0))
            ci = comp.features.combineFeatures.createInput(target, tools)
            ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            comp.features.combineFeatures.add(ci)
            return True
    raise RuntimeError('no part named %r in %s' % (name, group_occ.name))


def _group(root, name):
    for o in root.occurrences:
        if o.component.name == name:
            return o
    raise RuntimeError('no group named %r' % name)


def _delete(root, group, rules):
    """Delete a group's parts by (name, side) rules, side None for both. The group is read again from the root and
    every part to go is found before any is deleted: after an edit, an occurrence read earlier may be stale."""
    g = _group(root, group)
    doomed = [(o.nativeObject or o) for o in g.childOccurrences
              if any(o.component.name == n and (sd_ is None or _side(o) == sd_) for n, sd_ in rules)]
    for o in doomed:
        o.deleteMe()
    return len(doomed)


def build(contact=128.0):
    o = design(contact)
    refuse(o)
    msg39 = WD.build()
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    occ = WD._last['occ']
    L, K, s = o['L'], wd.LAY40, o['stop']
    rec = _record_looks(des)
    # the joints back to rest before anything is added
    WD.pose(0.0, 0.0)
    gone = 0
    gone += _delete(root, 'Roll output', [(n, None) for n in ('Roll shaft', 'Tool flange ISO 9409-1-31.5-4-M5',
                                                              'Low-head screw DIN 7984 M6x12', 'End washer 18x6.4x2')])
    gone += _delete(root, 'Carrier', [(n, 1) for n in ('Trunnion', 'End washer 18x5.5x2.5', 'Cap screw ISO 4762 M5x12')])
    gone += _delete(root, 'Frame', [('Bearing cap fixed', None), ('Cap screw ISO 4762 M4x12', 1)])
    for k, nm in (('frame', 'Frame'), ('carrier', 'Carrier'), ('roll', 'Roll output')):
        occ[k] = _group(root, nm)
    _restore_looks(app, des, rec)
    B = WD.Builder(app, root)
    # the roll output: the bored shaft, the flange to the standard, the second ring, the slip ring's rotor
    rc = occ['roll'].component
    B.part(rc, 'shaft40', 'Roll shaft, bored', [(roll_shaft_hollow(L, K), 'roll shaft', 'steel')], WD.ident())
    B.part(rc, 'flange40', 'Tool flange ISO 9409-1-31.5-4-M5', [(flange_iso(L), 'tool flange', 'stainless')], WD.ident())
    r0, r1 = K['ring2']
    B.part(rc, 'ring471b', 'Retaining ring DIN 471-15, flange', [(WD.ring(r0, r1, 7.15, 9.6), 'ring', 'screw')], WD.ident())
    rotor, stator, pin, tab = slip_parts(K)
    B.part(rc, 'rotor', 'Slip ring rotor', [(rotor, 'rotor flange', 'alu')], WD.ident())
    # the carrier: the slip ring's stator and its bracket; the left trunnion with its stop disc
    cc = occ['carrier'].component
    B.part(cc, 'stator', 'Slip ring stator', [(stator, 'stator', 'black'), (pin, 'pin', 'steel')], WD.ident())
    B.part(cc, 'sbracket', 'Slip ring bracket', [(tab, 'bracket', 'alu')], WD.ident())
    B.screw(cc, 3, 8, WD.frame(*WD.DOWN, T=(17.5, 9.5, -25.5)))
    B.part(cc, 'tstop', 'Trunnion with stop end', [(trunnion_stop(L, K), 'trunnion', 'steel')], WD.side(1))
    B.part(cc, 's5', 'Spacer S5', [(WD.ring(K['S5'][0], K['S5'][1], 7.5, 9.5), 'spacer', 'steel')], WD.side(1))
    B.part(cc, 'disc', 'Stop disc', [(stop_disc(K), 'stop disc', 'steel')], WD.side(1))
    k0, k1 = K['key']
    B.part(cc, 'key8', 'Key DIN 6885 A 5x5x8', [(WD.box(4.5, 9.5, -2.5, 2.5, k0, k1), 'key', 'steel')], WD.side(1))
    B.part(cc, 'washer5b', 'End washer 18x5.5x2.5', [(WD.ring(K['tr_end'], K['tr_end'] + 2.5, 2.75, 9.0), 'washer', 'steel')], WD.side(1))
    B.screw(cc, 5, 12, WD.side(1, 0, 0, K['tr_end'] + 2.5))
    # the frame: the stop cap, its countersunk screws and the two pads
    fc_ = occ['frame'].component
    B.part(fc_, 'stopcap', 'Stop cap', [(stop_cap(L, K, s), 'stop cap', 'frame')], WD.side(1))
    for a in (45, 135, 225, 315):
        x, y = WD.polar(18, a)
        B.screw(fc_, 4, 12, WD.side(1, x, y, K['cap_plate'][1]), kind='csk')
    pa, pb = pads(K, s)
    B.part(fc_, 'padA', 'Buffer pad', [(pa, 'pad', 'seal')], WD.side(1))
    B.part(fc_, 'padB', 'Buffer pad', [(pb, 'pad', 'seal')], WD.side(1))
    # the carrier's tapped hole for the bracket's screw
    cut_into(occ['carrier'], 'Carrier block', WD.zcyl(-23.0, -16.5, 1.5, 17.5, 9.5))
    for sub in (occ['frame'], occ['carrier'], occ['roll']):
        WD.ground_children(sub.component)
    lim = WD._last['joints']['pitch'].jointMotion.rotationLimits
    lim.minimumValue = math.radians(-140.0)
    lim.maximumValue = math.radians(140.0)
    app.activeViewport.fit()
    pl = WD.parts_list(root)
    st, sh, pp = o['stop'], o['shaft'], o['play']
    return ('Wrist integrated: %d of Chapter 39\'s parts replaced.\nStops: switch at %.0f deg, the brakes stop by %.1f deg, '
            'contact at %.0f deg; the hard stop alone %.1f N m, %.0f N on a pad. Roll shaft bored %.0f mm: %.1f Hz.\n'
            'Play at the payload: %.2f mm.\n\nParts: %d components; %s.\n\n[%s]'
            % (gone, wd.STOP40['switch'], st['stop_at'], o['contact'], st['T_pk'], st['F'], wd.BORE40, sh['f'],
               pp['at_tool'], sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


def pose(pitch, roll):
    WD.pose(pitch, roll)


_last = WD._last
parts_list = WD.parts_list

# ---------- the command dialog ----------
_handlers = []
FIELDS = [('contact', 'Hard stop contact (deg)', 128.0)]


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
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Pitch-roll wrist, integrated', 'The wrist of Chapter 40')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('WristIntegration could not start:\n' + traceback.format_exc())
