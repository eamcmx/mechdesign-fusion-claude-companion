# -*- coding: utf-8 -*-
"""ScaraJointModules.py: the SCARA's two arm joints, J1 and J2, as two modules side by side (Volume 2, Chapter 36).

Each module: a servo motor with its holding brake, square flange under a module plate on four bolts; its shaft up
through the plate into the wave generator of a Harmonic Drive CSG-2UH housed unit, ratio 50, whose output flange
carries the next arm. J1 is a CSG-40-50-2UH (or the size typed in the dialog), J2 a CSG-20-50-2UH. The output turns
one fiftieth of the motor's shaft, the other way, since the circular spline is fixed in the housing.

Every number of the sizing comes from projects/scara_robot/lib/scara_design.py (modules(), CSG, MOTOR); every size
of a unit's outline from the catalogue's outlines kept there (A, B, E, O, S, b, c holes of d). Inside the unit the
parts are drawn at concept level, in proportion to the unit's size. Millimetres in this file; the API works in cm.

Axes: z up along each module's axis, z = 0 on the housing's input face; J1 at the origin, J2 400 mm along x.
"""
import importlib
import math
import os
import re
import sys
import traceback

import adsk.core
import adsk.fusion


def _common_dir():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py in any folder above %s' % __file__)
        d = up


_COMMON = _common_dir()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP          # noqa: E402,F401  puts every project's lib/ on sys.path
import scara_design as sd             # noqa: E402
import mdfc_parts as MP               # noqa: E402
importlib.reload(sd)                  # Fusion keeps modules between runs: pick up edits
importlib.reload(MP)

VERSION = 'ScaraJointModules PT-1 (prompt test, 2026-11-11)'
CMD_ID = 'mdfc_ScaraJointModules_PT'
CM = 0.1                              # mm to the API's cm

RATIO = 50
SIZES1 = (20, 32, 40)                 # the J1 unit the dialog accepts
SIZE2 = 20                            # the J2 unit
X2 = 400.0                            # mm, the J2 module along x
PLATE_T = {1: 12.0, 2: 8.0}           # mm, the module plates, as scara_design.heights() lays them (Chapter 38)
HOLE_TO_BOLT = {5.5: 5, 6.6: 6, 9.0: 8}   # Table 29's case-bolt hole d: the bolt it clears (ISO 273, medium)
CLEAR = {5: 5.5, 6: 6.6, 8: 9.0}      # the clearance hole of a bolt (the same series)
AT = {5: 14.2, 6: 20.1, 8: 36.6}      # mm2, the bolts' tensile stress areas, as scara_design.modules() takes them
MOTOR_BOLT = {1: 8, 2: 5}             # the motors' flange bolts: M8 on J1's 130 flange, M5 on J2's 60 (parts list)
SHAFT_D = {1: 22.0, 2: 14.0}          # mm, the motors' shafts (assumed: the usual for a 130 and a 60 flange)

_last = {}                            # what the last build made: the joints, the numbers, the parts
_handlers = []


class Refused(ValueError):
    """A unit the catalogue's limits or the brief's life rule out."""


# ============================================================================================================
# The numbers: Chapter 36's selection for the J1 unit asked for, J2 on the CSG-20
# ============================================================================================================
def _j1_rows(b):
    """J1's load pattern as scara_design.modules() makes it: the standard cycle's horizontal leg there and back,
    the rest of the cycle at rest after each leg."""
    A = (b['x0'], -b['across'] / 2)
    B = (b['x0'], b['across'] / 2)
    rest = (b['cycle'] - 2 * b['t_h']) / 2
    return sd.leg_pattern(b, A, B, b['m_rated'], b['t_h'], 1, rest=rest), A


def design(size1=40):
    """Chapter 36's sizing for a J1 unit of size1 and the J2 CSG-20: the units' selections against the catalogue,
    the motors, the output bearings, the case bolts, the natural frequencies; and what rules the J1 unit out."""
    size1 = int(round(float(size1)))
    if size1 not in SIZES1:
        raise Refused('the J1 unit must be a CSG 20, 32 or 40, not %s' % size1)
    b = sd.brief()
    o = sd.modules(b)
    r1, A = _j1_rows(b)
    sel1 = sd.hd_select(r1, size1)
    mot1 = sd.motor(r1, size1, 1)
    u1, u2 = sd.CSG[size1], sd.CSG[SIZE2]
    sel2, mot2 = o['j2'][SIZE2], o['j2_motor']
    short = []
    for what, v, lim, unit in (('repeated peak torque', sel1['Tpk'], u1['Trep'], 'N m'),
                               ('average torque', sel1['Tav'], u1['Tav'], 'N m'),
                               ('largest input speed', sel1['ni_max'], u1['nmax'], 'rev/min')):
        if v > lim:
            short.append('its %s %s %s is over its limit %s %s by %s %s (%.1f %%)'
                         % (what, _n(v, 1), unit, _n(lim), unit, _n(v - lim, 1), unit, 100 * (v - lim) / lim))
    if sel1['Lh'] < b['life_h']:
        short.append("its life %s h is under the brief's %s h by %s h"
                     % (_n(sel1['Lh']), _n(b['life_h']), _n(b['life_h'] - sel1['Lh'])))
    bolt1 = HOLE_TO_BOLT[float(u1['d'])]
    bolt2 = HOLE_TO_BOLT[float(u2['d'])]
    p = sd.inertia(b, b['m_rated'])
    qa = sd.ik(b, *A)
    M11 = sd.dynamics(p, qa[1], 0, 0, 0, 0)[2]['M11']
    return dict(size1=size1, size2=SIZE2, b=b, sel1=sel1, sel2=sel2, mot1=mot1, mot2=mot2, u1=u1, u2=u2,
                short=short, bear1=sd.j1_bearing(b, size=size1), bear2=o['j2_bearing'],
                bolts1=sd.case_bolts(size1, At=AT[bolt1]), bolts2=sd.case_bolts(SIZE2, At=AT[bolt2]),
                bolt1=bolt1, bolt2=bolt2,
                f1=math.sqrt(u1['K1'] / M11) / (2 * math.pi), f2=o['f2'])


def _n(x, nd=0):
    """A number with its thousands set apart by a space: 23 984."""
    return '{:,.{}f}'.format(x, nd).replace(',', ' ')


# ============================================================================================================
# The module's layout (mm), from the catalogue's outline and, inside the unit, proportions of its size
# ============================================================================================================
def layout(size, joint):
    u = sd.CSG[size]
    mo = sd.MOTOR[joint]
    L = dict(size=size, joint=joint)
    A, B, E, O, S = (float(u[k]) for k in ('A', 'B', 'E', 'O', 'S'))
    L.update(A=A, B=B, E=E, O=O, S=S, bc=float(u['b']), nc=int(u['nc']), dh=float(u['d']))
    Mc = HOLE_TO_BOLT[float(u['d'])]
    # the cross roller bearing on its pitch circle dp: rollers Dw across, Lw long, in a square groove at 45 deg
    rp = u['dp'] * 1000.0 / 2.0
    Dw = 0.10 * 2 * rp
    Lw = 0.90 * Dw
    er = Dw / math.sqrt(2.0)                 # a roller's reach from its centre along r and z
    c = 0.6                                  # the groove's clearance on its half-diagonal
    H = er + c                               # the groove's half-diagonal
    land = max(2.0, 0.2 * Dw)
    gr = 0.4                                 # the radial gap between the housing's bore and the hub
    g = 0.5                                  # the output flange's face above the housing's top
    zr = E - H - land                        # the rollers' plane
    z_bz = zr - H - land                     # the bottom of the bearing's bore in the housing
    z_hb = z_bz - 1.0                        # the hub's end, on the flexspline's diaphragm
    # the strain wave gear: the circular spline's pitch radius from the size (tenths of an inch of pitch diameter)
    rg = size * 2.54 / 2.0
    Dg = 2 * rg
    rf_o = rg - 0.3                          # the flexspline's teeth
    Rcs = rg + 0.3                           # the circular spline's teeth, in the housing
    Rcav = Rcs + 1.0                         # the housing's bore between the circular spline and the bearing
    tw = 0.035 * Dg                          # the flexspline's wall at its teeth
    rf_i = rf_o - tw
    t_d = 0.03 * Dg                          # its diaphragm
    z_o = 1.0                                # the cup's open end above the input face
    w_t = 0.20 * Dg                          # the teeth's width
    z_cs = z_o + w_t + 1.0                   # the circular spline's end
    a_wg = rf_i - 0.5                        # the wave generator: its ellipse inside the cup, 2 % of D out of round
    b_wg = a_wg - 0.02 * Dg
    z_w0, z_w1 = z_o + 0.5, z_o + w_t - 0.5
    rs = SHAFT_D[joint] / 2.0
    # the plate, the motor and its brake
    tp = PLATE_T[joint]
    F, D, Lmot = float(mo['flange']), float(mo['D']), float(mo['L'])
    tf = 0.10 * F
    zf0 = -tp - tf                           # the motor flange's underside
    Lb = 0.28 * Lmot                         # the brake, under the motor
    neck = 3.0
    Lm = Lmot - Lb - neck
    Mm = MOTOR_BOLT[joint]
    e = F / 2 - 0.11 * F                     # the flange's bolts, on its diagonals
    le_c = min(1.25 * Mc, tp - 2.0)          # the bolts' engagement in the plate
    le_m = min(1.25 * Mm, tp - 2.0)
    P = max(A, F) + 20.0
    # the rollers: as many as an even count allows, each clear of the next by 10 % of its angle
    phi = math.asin(Dw / 2 / (rp - er))
    n_r = int(math.floor(2 * math.pi / (2.2 * phi) / 2)) * 2
    L.update(Mc=Mc, rp=rp, Dw=Dw, Lw=Lw, er=er, H=H, gr=gr, g=g, zr=zr, z_bz=z_bz, z_hb=z_hb, rg=rg, rf_o=rf_o,
             Rcs=Rcs, Rcav=Rcav, rf_i=rf_i, t_d=t_d, z_o=z_o, w_t=w_t, z_cs=z_cs, a_wg=a_wg, b_wg=b_wg, z_w0=z_w0,
             z_w1=z_w1, rs=rs, z_st=z_w1 - 1.0, z_sb=zf0 - 0.4 * Lm, tp=tp, F=F, D=D, Lmot=Lmot, tf=tf, zf0=zf0,
             Lb=Lb, neck=neck, Lm=Lm, Mm=Mm, e=e, le_c=le_c, le_m=le_m, Lc=E + le_c, Lmb=tf + le_m, P=P,
             n_r=n_r, rh=rp - gr)
    t0 = math.pi / L['nc']                   # the case bolts half a pitch off x
    L['case_xy'] = [(L['bc'] / 2 * math.cos(t0 + 2 * math.pi * k / L['nc']),
                     L['bc'] / 2 * math.sin(t0 + 2 * math.pi * k / L['nc'])) for k in range(L['nc'])]
    L['motor_xy'] = [(sx * e, sy * e) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    _fits(L)
    return L


def _fits(L):
    """The proportions above must leave every part clear of the next; say which does not."""
    rules = [
        ('the output flange clears the case bolts\' heads', L['O'] / 2 + 0.5 < L['bc'] / 2 - 0.75 * L['Mc']),
        ('the case bolts\' heads sit on the housing', L['bc'] / 2 + 0.75 * L['Mc'] <= L['A'] / 2),
        ('the case bolts clear the housing\'s bores', L['bc'] / 2 - L['dh'] / 2 > max(L['Rcav'], L['rp'] + L['H']) + 1),
        ('the circular spline ends under the bearing', L['z_cs'] < L['z_bz'] - 0.5),
        ('the wave generator ends under the diaphragm', L['z_w1'] + 0.5 < L['z_hb'] - L['t_d']),
        ('the hub\'s groove clears its bore', L['rh'] - L['H'] > L['S'] / 2 + 1),
        ('the motor bolts\' heads clear the motor', L['e'] * math.sqrt(2) - 0.75 * L['Mm'] > L['D'] / 2 + 0.5),
        ('the motor bolts\' heads are on the flange', L['e'] + 0.75 * L['Mm'] < L['F'] / 2),
        ('the tapped holes stay in the plate', max(L['le_c'], L['le_m']) + 1 < L['tp']),
        ('the shaft fits the wave generator', L['b_wg'] > L['rs'] + 2),
        ('the wave generator turns inside the cup', L['a_wg'] < L['rf_i']),
        ('the flexspline turns inside the circular spline', L['rf_o'] < L['Rcs']),
        ('at least 12 rollers', L['n_r'] >= 12),
    ]
    bad = [r for r, good in rules if not good]
    if bad:
        raise RuntimeError('CSG-%d, J%d: the layout does not fit: %s' % (L['size'], L['joint'], '; '.join(bad)))


# ============================================================================================================
# The bodies, as temporary bodies in the module's frame (mm)
# ============================================================================================================
def _cone(z0, r0, z1, r1):
    return MP.tbm().createCylinderOrCone(MP.P3(0, 0, z0 * CM), r0 * CM, MP.P3(0, 0, z1 * CM), r1 * CM)


def _zc(z0, z1, r, x=0.0, y=0.0):
    return MP.cyl((x, y, z0), (x, y, z1), r)


def _housing(L):
    """The housing with the circular spline: A across, E high from the input face; the circular spline's teeth at
    its input end, the outer half of the rollers' square groove at its output end, the c case-bolt holes of d on
    the circle b."""
    rp, H, zr, d = L['rp'], L['H'], L['zr'], 0.5
    h = _zc(0.0, L['E'], L['A'] / 2)
    MP.cut(h, _zc(-1.0, L['z_bz'], L['Rcs']))
    MP.cut(h, _zc(L['z_cs'], L['z_bz'], L['Rcav']))
    MP.cut(h, _zc(L['z_bz'] - 0.5, L['E'] + 1.0, rp + L['gr']))
    MP.cut(h, _cone(zr - H - d, rp - d, zr, rp + H), _cone(zr, rp + H, zr + H + d, rp - d))
    for x, y in L['case_xy']:
        MP.cut(h, _zc(-1.0, L['E'] + 1.0, L['dh'] / 2, x, y))
    return h


def _rollers(L):
    """One row of cross rollers on the pitch circle, their axes alternately at +45 and -45 deg."""
    out = []
    s = math.sqrt(0.5)
    for k in range(L['n_r']):
        t = 2 * math.pi * k / L['n_r']
        er_ = (math.cos(t), math.sin(t), 0.0)
        sg = 1.0 if k % 2 == 0 else -1.0
        a = (s * er_[0], s * er_[1], sg * s)
        c = (L['rp'] * er_[0], L['rp'] * er_[1], L['zr'])
        h = L['Lw'] / 2
        out.append(MP.cyl([c[i] - a[i] * h for i in range(3)], [c[i] + a[i] * h for i in range(3)], L['Dw'] / 2))
    return out


def _flange(L):
    """The output flange: its face O across reaching B, its bore S; its hub down inside the rollers, with the inner
    half of their groove, to the flexspline's diaphragm."""
    rp, H, zr, d = L['rp'], L['H'], L['zr'], 0.5
    f = _zc(L['z_hb'], L['E'] + L['g'] + 1.0, L['rh'])
    MP.union(f, _zc(L['E'] + L['g'], L['B'], L['O'] / 2))
    MP.cut(f, _zc(L['z_hb'] - 1.0, L['B'] + 1.0, L['S'] / 2))
    groove = _zc(zr - H, zr + H, rp + d)
    MP.cut(groove, _cone(zr - H - d, rp + d, zr, rp - H), _cone(zr, rp - H, zr + H + d, rp + d))
    MP.cut(f, groove)
    return f


def _flexspline(L):
    """The flexspline's cup under the hub, open towards the input face, its teeth at the open end."""
    fs = _zc(L['z_o'], L['z_hb'], L['rf_o'])
    MP.cut(fs, _zc(L['z_o'] - 1.0, L['z_hb'] - L['t_d'], L['rf_i']))
    return fs


def _plate(L):
    """The module plate under the housing: the shaft's hole, the case bolts' tapped holes from the top, the motor
    bolts' from below (the threads drawn as plain holes)."""
    P, tp = L['P'], L['tp']
    pl = MP.box(-P / 2, P / 2, -P / 2, P / 2, -tp, 0.0)
    MP.cut(pl, _zc(-tp - 1.0, 1.0, L['rs'] + 1.5))
    for x, y in L['case_xy']:
        MP.cut(pl, _zc(-(L['le_c'] + 1.0), 1.0, L['Mc'] / 2 + 0.05, x, y))
    for x, y in L['motor_xy']:
        MP.cut(pl, _zc(-tp - 1.0, -tp + L['le_m'] + 1.0, L['Mm'] / 2 + 0.05, x, y))
    return pl


def _motor(L):
    """The servo motor: its square flange under the plate, the motor below it and its brake below the motor; the
    rotor's bore that takes the shaft."""
    F, D, tp, zf0 = L['F'], L['D'], L['tp'], L['zf0']
    m = MP.box(-F / 2, F / 2, -F / 2, F / 2, zf0, -tp)
    z_m = zf0 - L['Lm']
    MP.union(m, _zc(z_m, zf0 + 0.5, D / 2))
    MP.union(m, _zc(z_m - L['neck'] - 0.5, z_m + 0.5, 0.42 * D))
    MP.union(m, _zc(zf0 - L['Lmot'], z_m - L['neck'], 0.47 * D))
    MP.cut(m, _zc(L['z_sb'] - 1.0, -tp + 1.0, L['rs'] + 0.5))
    for x, y in L['motor_xy']:
        MP.cut(m, _zc(zf0 - 1.0, -tp + 1.0, CLEAR[L['Mm']] / 2, x, y))
    return m


def _shaft(L):
    """The motor's shaft, from inside the motor up through the plate into the wave generator, and no further."""
    return _zc(L['z_sb'], L['z_st'], L['rs'])


# ============================================================================================================
# Fusion: components, bodies, the wave generator's sketch, the joints
# ============================================================================================================
def _add_bodies(occ, tbs, name, material):
    """Several temporary bodies (module frame) into one component, in one base feature."""
    m = occ.transform2.copy()
    m.invert()
    comp = occ.component
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    for tb in tbs:
        MP.tbm().transform(tb, m)
        comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    for i in range(bf.bodies.count):
        body = bf.bodies.item(i)
        body.name = '%s %d' % (name, i + 1)
        MP.set_material(body, material)


def _copies(comp, name, xy, make, material):
    """One component drawn at the first place, then an occurrence of it at each of the others."""
    x0, y0 = xy[0]
    first = MP.new_occ(comp, name, (x0, y0, 0.0))
    MP.add_body(first, make(x0, y0), name, material)
    for x, y in xy[1:]:
        mat = adsk.core.Matrix3D.create()
        mat.translation = MP.V3(x * CM, y * CM, 0.0)
        occ = comp.occurrences.addExistingComponent(first.component, mat)
        occ.isGroundToParent = True
    return first


def _wave_generator(occ, L):
    """The elliptical wave generator, sketched and extruded in its own component (the temporary-body manager makes
    no ellipse): the ellipse a by b, the shaft's bore."""
    comp = occ.component
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.xYConstructionPlane, MP.VI(L['z_w0'] * CM))
    plane = comp.constructionPlanes.add(ci)
    sk = comp.sketches.add(plane)
    sk.name = 'Wave generator'
    z = L['z_w0'] * CM
    ctr = sk.modelToSketchSpace(MP.P3(0, 0, z))
    major = sk.modelToSketchSpace(MP.P3(L['a_wg'] * CM, 0, z))
    minor = sk.modelToSketchSpace(MP.P3(0, L['b_wg'] * CM, z))
    sk.sketchCurves.sketchEllipses.add(ctr, major, minor)
    sk.sketchCurves.sketchCircles.addByCenterRadius(ctr, L['rs'] * CM)
    prof = None
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        if p.profileLoops.count == 2:
            prof = p
    if prof is None:
        raise RuntimeError('the wave generator\'s sketch gave no ring between its ellipse and its bore')
    ext = comp.features.extrudeFeatures.addSimple(prof, MP.VI((L['z_w1'] - L['z_w0']) * CM),
                                                   adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    body = ext.bodies.item(0)
    body.name = 'Wave generator'
    MP.set_material(body, 'Steel')
    return body


def _module(root, joint, size, x):
    """One module: three subassemblies (stator, output, input) in a module component, the as-built revolute joints
    of its output and its input about its axis, and the motion link between them."""
    L = layout(size, joint)
    tag = 'J%d' % joint
    mod = MP.new_occ(root, '%s module' % tag, (x, 0.0, 0.0))
    mod.isGrounded = True
    mc = mod.component
    st = MP.new_occ(mc, 'Stator %s' % tag)
    ou = MP.new_occ(mc, 'Output %s' % tag)
    inp = MP.new_occ(mc, 'Input %s' % tag)
    st.isGroundToParent = True
    ou.isGroundToParent = False
    inp.isGroundToParent = False
    sc, oc, ic = st.component, ou.component, inp.component

    # the stator: the housing, the rollers, the plate, the motor and the bolts
    MP.add_body(MP.new_occ(sc, 'CSG-%d-50-2UH housing' % size), _housing(L), 'Housing and circular spline', 'Steel')
    _add_bodies(MP.new_occ(sc, 'Cross rollers %d' % size), _rollers(L), 'Roller', 'Steel')
    MP.add_body(MP.new_occ(sc, 'Module plate %s' % tag), _plate(L), 'Module plate', 'Aluminum 6061')
    MP.add_body(MP.new_occ(sc, 'Servo motor %s' % tag), _motor(L), 'Motor and brake', 'Steel')
    _copies(sc, 'Case bolt M%d' % L['Mc'], L['case_xy'],
            lambda px, py: MP.cap_screw(L['Mc'], L['Lc'], (px, py, L['E']), (0, 0, -1)), 'Steel')
    _copies(sc, 'Motor bolt M%d' % L['Mm'], L['motor_xy'],
            lambda px, py: MP.cap_screw(L['Mm'], L['Lmb'], (px, py, L['zf0']), (0, 0, 1)), 'Steel')
    # the output: the flange and the flexspline
    MP.add_body(MP.new_occ(oc, 'Output flange %d' % size), _flange(L), 'Output flange', 'Steel')
    MP.add_body(MP.new_occ(oc, 'Flexspline %d' % size), _flexspline(L), 'Flexspline', 'Steel')
    # the input: the motor's shaft and the wave generator
    MP.add_body(MP.new_occ(ic, 'Motor shaft %s' % tag), _shaft(L), 'Motor shaft', 'Steel')
    _wave_generator(MP.new_occ(ic, 'Wave generator %d' % size), L)

    # as-built revolute joints about the module's axis (+z through its origin), in the root, on proxies
    def axis_point():
        geo = None
        try:
            geo = adsk.fusion.JointGeometry.createByPoint(mc.originConstructionPoint.createForAssemblyContext(mod))
        except Exception:
            geo = None
        if geo is None:                       # a point of a root sketch on the module's axis instead
            sk = root.sketches.add(root.xYConstructionPlane)
            sk.name = '%s axis' % tag
            geo = adsk.fusion.JointGeometry.createByPoint(sk.sketchPoints.add(MP.P3(x * CM, 0, 0)))
        return geo

    def revolute(moving, name):
        geo = axis_point()
        ji = root.asBuiltJoints.createInput(moving.createForAssemblyContext(mod), st.createForAssemblyContext(mod), geo)
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
        j = root.asBuiltJoints.add(ji)
        j.name = name
        return j
    j_out = revolute(ou, '%s output' % tag)
    j_in = revolute(inp, '%s input' % tag)
    # the motion link: 50 turns in to one out (one turn in to 7.2 deg out), the output the other way
    mli = root.motionLinks.createInput(j_in, j_out)
    try:
        mli.motionOne = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
        mli.motionTwo = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
    except Exception:
        pass                                  # a revolute joint has one motion: the default is that one
    mli.valueOne = adsk.core.ValueInput.createByString('360 deg')
    mli.valueTwo = adsk.core.ValueInput.createByString('%g deg' % (360.0 / RATIO))
    mli.isReversed = True
    link = root.motionLinks.add(mli)
    link.name = '%s %d to 1' % (tag, RATIO)
    # both joints turn about the module's +z: the signs of their axes
    return (j_out, 1, j_in, 1), L


def parts_list(root):
    """{name: count} of every part (an occurrence whose component holds bodies), the unit's size dropped from the
    names of its inner parts."""
    out = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            k = re.sub(r' \d+$', '', o.component.name)
            out[k] = out.get(k, 0) + 1
    return dict(sorted(out.items()))


def build(size1=40):
    """Refuse a J1 unit that falls short; otherwise a new design with the two modules. Returns the message."""
    d = design(size1)
    if d['short']:
        raise Refused('CSG-%d at J1 falls short: %s.' % (d['size1'], '; '.join(d['short'])))
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    joints, lay = {}, {}
    for joint, size, x in ((1, d['size1'], 0.0), (2, SIZE2, X2)):
        joints[joint], lay[joint] = _module(root, joint, size, x)
    parts = parts_list(root)
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    global _last
    _last = dict(doc=doc, design=d, joints=joints, layout=lay, parts=parts)
    return _message(d, parts)


def _message(d, parts):
    s1, s2, u1, u2, m1, m2 = d['sel1'], d['sel2'], d['u1'], d['u2'], d['mot1'], d['mot2']
    b1, b2, c1, c2 = d['bear1'], d['bear2'], d['bolts1'], d['bolts2']
    lines = [
        'SCARA joint modules J1 and J2, %s' % VERSION,
        '',
        'J1  CSG-%d-50-2UH: average torque %.1f N m (limit %g), repeated peak %.1f N m (limit %g), life %s h '
        "(the brief's %s h), largest input speed %.0f rev/min (limit %g)"
        % (d['size1'], s1['Tav'], u1['Tav'], s1['Tpk'], u1['Trep'], _n(s1['Lh']), _n(d['b']['life_h']),
           s1['ni_max'], u1['nmax']),
        '    motor through the ratio %d, the efficiency %.2f and the inertia: %.2f N m peak, %.2f N m RMS'
        % (RATIO, sd.ETA_50, m1['peak'], m1['rms']),
        'J2  CSG-%d-50-2UH: average torque %.1f N m (limit %g), repeated peak %.1f N m (limit %g), life %s h on '
        'radial moves, largest input speed %.0f rev/min (limit %g)'
        % (d['size2'], s2['Tav'], u2['Tav'], s2['Tpk'], u2['Trep'], _n(s2['Lh']), s2['ni_max'], u2['nmax']),
        '    motor: %.2f N m peak, %.2f N m RMS' % (m2['peak'], m2['rms']),
        'Output bearings: J1 static safety %.1f, life %s h oscillating over the cycle; J2 static safety %.1f'
        % (b1['fs'], _n(b1['Loc']), b2['fs']),
        'Case bolts, class 12.9: J1 %d x M%d on %g mm hold %s N m by friction (%.1f x the momentary %g N m); '
        'J2 %d x M%d on %g mm, %s N m (%.1f x %g N m)'
        % (u1['nc'], d['bolt1'], u1['b'], _n(c1['T']), c1['n'], u1['Tmom'],
           u2['nc'], d['bolt2'], u2['b'], _n(c2['T']), c2['n'], u2['Tmom']),
        'Natural frequencies from K1: J1 %.1f Hz, J2 %.1f Hz' % (d['f1'], d['f2']),
        '',
        'Parts, %d components in %d kinds:' % (sum(parts.values()), len(parts)),
    ]
    lines += ['    %s  %d' % (k, v) for k, v in parts.items()]
    return '\n'.join(lines)


# ============================================================================================================
# The dialog
# ============================================================================================================
class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            size1 = args.command.commandInputs.itemById('size1').value
            ui.messageBox(build(size1=size1))
        except Refused as e:
            ui.messageBox('Refused: %s' % e)
        except Exception:
            ui.messageBox('ScaraJointModules failed:\n%s' % traceback.format_exc())


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if __name__ != 'mdfc_check':          # a check runs the dialog inside its own script
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('size1', 'J1 unit, CSG size', '', adsk.core.ValueInput.createByString('40'))
        ins.addTextBoxCommandInput('note', '', 'J1: CSG 20, 32 or 40. J2 is a CSG-20. Both ratio 50.', 2, True)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        hd = _Destroy()
        cmd.destroy.add(hd)
        _handlers.append(hd)


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'SCARA joint modules',
                                                       'The J1 and J2 strain wave modules, side by side')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('ScaraJointModules failed:\n%s' % traceback.format_exc())


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
