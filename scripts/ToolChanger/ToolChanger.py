# =============================================================================
# ToolChanger.py  -  Chapter 44 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's tool flange
# and its tool changer. The robot's plate to ISO 9409-1-160-6-M10 under J4's RV-80N; the changer's master and its
# cover on it by six M10 cap screws, with a double-acting piston whose rod passes a gland to the two-angle cam that
# drives six segments out into the tool plate's ring; the tool plate with the same ISO pattern underneath for the
# gripper. The sizes are lib/tool_changer.py's. Written from PROMPT.txt beside it; checked by
# checks/tool_changer_check.py.
#
#   INPUTS        the dialog: the cam's locking angle (deg), the supply's least pressure (bar), locked or not
#   DERIVED       tool_changer: ISO9409 (Table 1), CHANGER (the lock's sizes), GEOM (the lock's geometry)
#   HARD RULES    build() refuses a locking angle the friction does not hold (alpha >= atan 0.10) and a pressure
#                 whose clamp falls under 1.5 times the cycle's need
#   CONVENTIONS   the interface's axis along Z, the robot's face at z = 0, +Xm along X; groups Robot plate
#                 (grounded), Master, Piston, Segment 1 to 6, Tool plate; set_state() places the moving ones
#   ACCEPTANCE    the message; the check: no interference locked and free, the lock holding, the tool plate free to
#                 come off, the ISO pattern measured, the groups weighed
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import tool_changer as tc     # noqa: E402
import mdfc_parts as mp       # noqa: E402
importlib.reload(tc)
importlib.reload(mp)

VERSION = 'ToolChanger 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

C = tc.CHANGER
GM = tc.GEOM
ISO = tc.ISO9409[C['iso']]
D1, D2, D3, D5 = C['iso'], ISO['d2'], ISO['d3'], ISO['d5']
R1 = D1 / 2.0
TH = tc.THREAD[ISO['d4']][0]
ANG = tc.iso_holes(D1)                    # the threaded holes' angles from +Xm
PORTS = (60.0, 120.0, 240.0, 300.0)       # the services' four ports on the same circle, between the screws
TRAVEL = 3.0                              # the segments' radial travel, mm
STROKE = GM['stroke']                     # the piston's travel, mm (20: the cam's 5 deg zone wholly above the segments)
Z_FACE, Z_TOOL = GM['z_face'], GM['z_tool']
SEG_Z0, SEG_Z1 = GM['z_seg']
T5 = math.tan(math.radians(C['alpha']))
T15 = math.tan(math.radians(C['beta']))


# ---------- bodies (mm) ----------
def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def zcone(z0, r0, z1, r1):
    return mp.tbm().createCylinderOrCone(P3(0, 0, z0 * CM), r0 * CM, P3(0, 0, z1 * CM), r1 * CM)


def at(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def halfspace(theta, side):
    """A large box on one side of the vertical plane through the axis at angle theta (side +1: counterclockwise)."""
    d = (math.cos(math.radians(theta)), math.sin(math.radians(theta)), 0.0)
    n = (-d[1] * side, d[0] * side, 0.0)
    c = P3(n[0] * 200 * CM, n[1] * 200 * CM, -60 * CM)
    ob = adsk.core.OrientedBoundingBox3D.create(c, V3(*d), V3(*n), 800 * CM, 400 * CM, 400 * CM)
    return mp.tbm().createBox(ob)


def sector(body, mid, half):
    mp.intersect(body, halfspace(mid - half, 1))
    mp.intersect(body, halfspace(mid + half, -1))
    return body


def slab(body, mid, width):
    """Keep what lies within width/2 of the radial line at angle mid, on the line's outer side: a straight guide."""
    d = (math.cos(math.radians(mid)), math.sin(math.radians(mid)), 0.0)
    n = (-d[1], d[0], 0.0)
    c = P3(d[0] * 100 * CM, d[1] * 100 * CM, -60 * CM)
    ob = adsk.core.OrientedBoundingBox3D.create(c, V3(*d), V3(*n), 200 * CM, width * CM, 400 * CM)
    return mp.intersect(body, mp.tbm().createBox(ob))


def group(root, name, grounded=False):
    o = mp.new_occ(root, name)
    o.isGroundToParent = False
    if grounded:
        o.isGrounded = True
    return o


def part(g, name, tb, material='Steel'):
    o = mp.new_occ(g.component, name)
    mp.add_body(o, tb, name, material)
    return o


# ---------- the parts ----------
def robot_plate(root):
    g = group(root, 'Robot plate', True)
    b = zcyl(0.0, 25.0, D2 / 2.0)
    mp.cut(b, zcyl(-1.0, ISO['t2'], D3 / 2.0))                          # the centring recess d3 H7
    x, y = at(R1, 0.0)
    mp.cut(b, zcyl(-1.0, ISO['t1'] + 2.0, D5 / 2.0, x, y))              # the pin hole d5 H7 on +Xm
    for a in ANG:
        x, y = at(R1, a)
        mp.cut(b, zcyl(-1.0, 20.0, TH / 2.0, x, y))                    # the threaded holes, 20 deep
    part(g, 'Robot plate ISO 9409-1-160-6-M10', b)
    return g


def master(root):
    """The master's body from the cover down to the face, with its spigot; the piston's bore over the gland, the
    cam's bore under it, the segments' windows, the screws' pockets, the ports and pins; the two air ports."""
    g = group(root, 'Master', True)
    zc0 = GM['z_cover'][0]
    zg0, zg1 = GM['z_gland']
    b = zcyl(Z_FACE, zc0, D2 / 2.0)
    mp.union(b, zcyl(GM['z_spigot'], Z_FACE, GM['R_spigot']))          # the lock's spigot into the tool plate
    mp.cut(b, zcyl(GM['z_bore'][0], zc0 + 1.0, GM['R_piston']))         # the piston's bore, -51 up to the cover
    mp.cut(b, zcyl(GM['z_spigot'] - 1.0, zg0, GM['R_seg_in'] + 0.1))   # the cam's bore, up to the gland
    mp.cut(b, zcyl(zg0 - 1.0, zg1 + 1.0, GM['R_rod'] + 0.1))           # the gland's hole for the rod
    for a, z in zip(GM['a_ports'], GM['z_ports']):                      # the air: release under the piston, lock over it
        x0, y0 = at(GM['R_piston'] - 1.0, a)
        x1, y1 = at(D2 / 2.0 + 1.0, a)
        mp.cut(b, mp.cyl((x0, y0, z), (x1, y1, z), GM['port_d'] / 2.0))
    for k in range(C['n_seg']):                                         # the segments' windows
        w = zcyl(SEG_Z0 - 0.2, SEG_Z1 + 0.2, GM['R_spigot'] + 1.0)
        mp.cut(w, zcyl(SEG_Z0 - 1.0, SEG_Z1 + 1.0, GM['R_seg_in'] - 5.0))
        mp.cut(b, slab(w, 60.0 * k, C['seg_w'] * 1000.0 + 0.4))
    for a in ANG:                                                       # the screws' pockets and clearance holes
        x, y = at(R1, a)
        mp.cut(b, zcyl(Z_FACE - 1.0, -20.0, 9.0, x, y))
        mp.cut(b, zcyl(-21.0, 1.0, (TH + 1.0) / 2.0, x, y))
    for a in PORTS:                                                     # the services' ports
        x, y = at(R1, a)
        mp.cut(b, zcyl(Z_FACE - 1.0, 1.0, 5.0, x, y))
    x, y = at(R1, 0.0)
    mp.cut(b, zcyl(-8.0, 1.0, D5 / 2.0, x, y))                          # the robot's locating pin
    for a in (0.0, 180.0):                                              # the tool plate's two locating pins
        x, y = at(R1, a)
        mp.cut(b, zcyl(Z_FACE - 1.0, Z_FACE + 12.0, 6.0, x, y))
    relief = zcyl(Z_FACE - 0.01, Z_FACE + 0.2, C['face_Ri'] * 1000.0)   # the face carries only the ring 90 to 100
    mp.cut(relief, zcyl(Z_FACE - 1.0, Z_FACE + 1.0, GM['R_spigot']))
    mp.cut(b, relief)
    part(g, 'Master body', b)
    # the cover closes the piston's bore and carries the spigot into the robot's recess; the six screws clamp it
    cv = zcyl(zc0, 0.0, D2 / 2.0)
    mp.union(cv, zcyl(0.0, 9.0, D3 / 2.0 - 0.01))
    for a in ANG:
        x, y = at(R1, a)
        mp.cut(cv, zcyl(zc0 - 1.0, 1.0, (TH + 1.0) / 2.0, x, y))
    for a in PORTS:
        x, y = at(R1, a)
        mp.cut(cv, zcyl(zc0 - 1.0, 1.0, 5.0, x, y))
    x, y = at(R1, 0.0)
    mp.cut(cv, zcyl(-8.0, 1.0, D5 / 2.0, x, y))
    part(g, 'Master cover', cv)
    for k, a in enumerate(ANG):
        x, y = at(R1, a)
        s = zcyl(-30.0, -20.0, 8.0, x, y)
        mp.union(s, zcyl(-20.0, 15.0, TH / 2.0, x, y))
        part(g, 'Cap screw M10x35 %d' % (k + 1), s)
    x, y = at(R1, 0.0)
    part(g, 'Pin 10 robot', zcyl(-8.0, ISO['t1'] + 2.0, D5 / 2.0, x, y))
    for k, a in enumerate((0.0, 180.0)):
        x, y = at(R1, a)
        part(g, 'Pin 12 tool %d' % (k + 1), zcyl(Z_FACE - 12.0, Z_FACE + 12.0, 6.0, x, y))
    return g


def piston(root):
    """The piston at its locked place, its face 4 mm over the gland, its rod down through the gland to the cam; the
    cam, screwed on the rod's end from below: the 5 deg locking zone over the segments' height, the 30 deg lead
    below it."""
    g = group(root, 'Piston')
    zp = GM['z_piston']                                                   # -47
    b = zcyl(zp, zp + GM['piston_h'], GM['R_piston'] - 0.05)
    mp.union(b, zcyl(SEG_Z1, zp + 0.01, GM['R_rod']))
    part(g, 'Piston with rod', b)
    r_low = GM['R_seg_in'] - (SEG_Z1 - SEG_Z0) * T5                     # 48.6 at the segments' foot
    cam = zcone(SEG_Z0, r_low, SEG_Z1, GM['R_seg_in'])
    z_bot = GM['z_spigot'] + 0.5
    mp.union(cam, zcone(z_bot, r_low - (SEG_Z0 - z_bot) * math.tan(math.radians(C['alpha_free'])), SEG_Z0, r_low))
    part(g, 'Cam', cam)
    return g


def segment(root, k):
    """Segment k at its locked place: its inner face on the cam's 5 deg cone, its outer face at 68 with the 15 deg
    face the ring's lip bears on."""
    g = group(root, 'Segment %d' % (k + 1))
    r_low = GM['R_seg_in'] - (SEG_Z1 - SEG_Z0) * T5
    zc = GM['z_chamfer']
    r_top = GM['R_seg_out'] - (SEG_Z1 - zc) / T15                       # where the 15 deg face meets the top
    b = zcyl(SEG_Z0, zc, GM['R_seg_out'])
    mp.union(b, zcone(zc, GM['R_seg_out'], SEG_Z1, r_top))
    mp.cut(b, zcone(SEG_Z0 - 0.001, r_low - 0.001 * T5, SEG_Z1 + 0.001, GM['R_seg_in'] + 0.001 * T5))
    slab(b, 60.0 * k, C['seg_w'] * 1000.0)
    part(g, 'Segment %d' % (k + 1), b)
    return g


def tool_plate(root):
    g = group(root, 'Tool plate')
    b = zcyl(Z_TOOL, Z_FACE, D2 / 2.0)
    zc = GM['z_chamfer']
    z_out = zc - (GM['R_groove'] - GM['R_seg_out']) * T15               # the lip's face at the groove's wall
    z_in = zc + (GM['R_seg_out'] - GM['R_bore']) * T15                  # ... at the bore
    mp.cut(b, zcyl(z_in - 0.001, Z_FACE + 1.0, GM['R_bore']))           # the bore through the lip
    mp.cut(b, zcone(z_out, GM['R_groove'], z_in, GM['R_bore']))          # the lip's 15 deg face
    mp.cut(b, zcyl(GM['z_spigot'] - 1.0, z_out + 0.001, GM['R_groove']))  # the groove the segments go into
    mp.cut(b, zcyl(Z_TOOL - 1.0, Z_TOOL + ISO['t2'], D3 / 2.0))          # the gripper's centring recess
    x, y = at(R1, 0.0)
    mp.cut(b, zcyl(Z_TOOL - 1.0, Z_TOOL + ISO['t1'] + 2.0, D5 / 2.0, x, y))
    for a in ANG:
        x, y = at(R1, a)
        mp.cut(b, zcyl(Z_TOOL - 1.0, Z_TOOL + 20.0, TH / 2.0, x, y))
    for a in PORTS:
        x, y = at(R1, a)
        mp.cut(b, zcyl(Z_TOOL - 1.0, Z_FACE + 1.0, 5.0, x, y))
    for a in (0.0, 180.0):
        x, y = at(R1, a)
        mp.cut(b, zcyl(Z_FACE - 12.0, Z_FACE + 1.0, 6.0, x, y))
    part(g, 'Tool plate ISO 9409-1-160-6-M10', b)
    return g


# ---------- the states ----------
def place(occ, dx=0.0, dy=0.0, dz=0.0):
    m = adsk.core.Matrix3D.create()
    m.translation = V3(dx * CM, dy * CM, dz * CM)
    occ.transform2 = m


def set_state(G_, locked=True, tool_down=0.0):
    """Locked: the piston down, the segments out. Free: the piston up by its stroke, each segment in by its travel.
    tool_down lowers the tool plate (mm) to test whether it holds or comes off."""
    place(G_['Piston'], dz=0.0 if locked else STROKE)
    for k in range(C['n_seg']):
        dx, dy = at(0.0 if locked else -TRAVEL, 60.0 * k)
        place(G_['Segment %d' % (k + 1)], dx, dy)
    place(G_['Tool plate'], dz=-tool_down)
    adsk.doEvents()


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        nm = o.component.name
        if nm.endswith(')') and ' (' in nm:                             # Fusion's suffix for a name already taken
            nm = nm[:nm.rindex(' (')]
        wds = nm.split()
        if wds[-1].isdigit() and not o.component.name.startswith('Robot plate') and not o.component.name.startswith('Tool plate'):
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


def design(alpha=None, p_bar=None):
    c = dict(C)
    if alpha is not None:
        c['alpha'] = float(alpha)
    if p_bar is not None:
        c['p_min'] = float(p_bar) * 1e5
    return c


def refuse(c):
    lim = math.degrees(math.atan(tc.MU_MIN))
    if c['alpha'] >= lim:
        raise ValueError('A locking angle of %.1f deg is not held by friction as low as %.2f (atan = %.2f deg): the '
                         'lock would let go without air.' % (c['alpha'], tc.MU_MIN, lim))
    Fp = math.pi / 4 * c['D_piston'] ** 2 * c['p_min']
    Fc = tc.lock_clamp(Fp + c['spring'], c['alpha'], c['beta'])
    need = 1.5 * max(tc.face_clamp_needed(x['Fa'], x['M'], c['face_Ri'], c['face_Ro'])[0]
                     for x in tc.cycle_loads(depth=c['depth'], n=240))
    if Fc < need:
        raise ValueError('At %.1f bar the clamp is %.1f kN, under 1.5 times the cycle\'s need (%.1f kN).'
                         % (c['p_min'] / 1e5, Fc / 1e3, need / 1e3))
    return Fc, need


_last = {}


def build(alpha=None, p_bar=None, locked=True):
    c = design(alpha, p_bar)
    Fc, need = refuse(c)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    G_ = {'Robot plate': robot_plate(root), 'Master': master(root), 'Piston': piston(root)}
    for k in range(c['n_seg']):
        G_['Segment %d' % (k + 1)] = segment(root, k)
    G_['Tool plate'] = tool_plate(root)
    for k, col in (('Robot plate', 'Paint - Enamel Glossy (Blue)'), ('Master', 'Paint - Enamel Glossy (Grey)'),
                   ('Piston', 'Paint - Enamel Glossy (Red)'), ('Tool plate', 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(des, G_[k], col)
    set_state(G_, bool(locked))
    _last.update(groups=G_, c=c)
    app.activeViewport.fit()
    n = parts_list(root)
    return ('Tool changer built, %s.\nRobot side and tool side ISO 9409-1-%d-%d-%s; master on the robot plate by %d '
            'M10 x 35 class 10.9.\nLock: piston %d mm at %.1f bar, air to lock and to free; cam %.1f deg (holds itself without air), '
            'ring face %.0f deg; %d segments; clamp %.1f kN against the cycle\'s %.1f kN (1.5 x its need).\n\nParts: %d '
            'components; %s.\n\n[%s]'
            % ('locked' if locked else 'free', D1, ISO['n'], ISO['d4'], ISO['n'], round(c['D_piston'] * 1000),
               c['p_min'] / 1e5, c['alpha'], c['beta'], c['n_seg'], Fc / 1e3, need / 1e3,
               sum(n.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(n.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('alpha', 'The cam\'s locking angle (deg)', C['alpha']), ('p_bar', 'The supply\'s least pressure (bar)',
          C['p_min'] / 1e5)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value for k, _, _ in FIELDS}
            vals['locked'] = ins.itemById('locked').value
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
            ins.addValueInput(key, label, '', VI(dflt))
        ins.addBoolValueInput('locked', 'Locked', True, '', True)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'ToolChangerCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Tool changer', 'The palletizer\'s tool changer of Chapter 44')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('ToolChanger could not start:\n' + traceback.format_exc())
