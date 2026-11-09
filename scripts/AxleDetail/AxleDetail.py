# =============================================================================
# AxleDetail.py  -  Chapter 54 of "Mechanical Design in Autodesk Fusion with Claude": the rear axle of Chapter 31 in
# detail. It builds Chapter 31's axle with RearAxle.py, loaded, not copied, with the tapered bearings' abutments
# brought inside SKF's bands (the pinion nose's middle bore stepped for both cups, the companion flange's hub and the
# adjusters relieved, the case's shoulders lowered), the spacer between the pinion's cones from face to face, the
# cups' bores closed to the cups, the half shafts' splines to ISO 14's 8 x 36 x 40 x 7, and then gives every made part
# its material and every fitting face its class and roughness as Chapter 51's MaterialsFits.py does, the splines and
# the adjusters' threads by this chapter's own rules.
# Written from PROMPT.txt beside it; checked by checks/axle_detail_check.py.
#
#   INPUTS        the dialog: the adjusters' thread pitch (2 mm)
#   DERIVED       axle_detail: abutments(), spacer(), load_classes(), splines(), adjuster(); axle_design: layout
#   HARD RULES    build() refuses a pitch at which one thread would not carry the case bearing's axial load at the
#                 peak with a factor of 2 in shear, and anything Chapter 31 refuses; nothing of RearAxle's, RearAxleCarrier's,
#                 AxleHousing's or MaterialsFits' code copied
#   CONVENTIONS   as RearAxle.py; the changed parts keep their names
#   ACCEPTANCE    the message; the check: the bores, the abutments, the spacer touching both cones, the splines, the
#                 faces' classes, no interference at rest and driving, the refusal
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
SCRIPTS = os.path.dirname(HERE)
LIB = os.path.join(os.path.dirname(SCRIPTS), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_ad54_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


RA = _load('RearAxle')                  # loads RearAxleCarrier and AxleHousing and reloads axle_design
MF = _load('MaterialsFits')
import axle_design as ad                # noqa: E402  (the module RA and its parts use, after their reloads)
import mdfc_parts as mp                 # noqa: E402
import axle_detail as XD                # noqa: E402
importlib.reload(XD)
CAR, HSG = RA.CAR, RA.HSG

VERSION = 'AxleDetail 1.0 (2026-11-03)'
CM = 0.1
VI = adsk.core.ValueInput.createByReal
_last = {}
A_, B_, C_ = ad.SKF_TRB['32309'], ad.SKF_TRB['32308'], ad.SKF_TRB['32213']
YC = CAR.HUB_Y + C_[2]                   # the case cups' back faces, |y|
THREAD = 'M120 x 2 %s'                   # the adjusters' thread, 6H in the carrier and the caps, 6g on the adjusters
SPLINE_SHAFT = 'f7 a11 d10'              # ISO 14 sliding fit, the half shafts in the side gears (d, D, B)
SPLINE_HUB = 'H7 H10 H9'
SPLINE_FIXED = 'h7 a11 h10'              # the pinion in the companion flange


def near(a, b, tol=1e-6):
    return abs(a - b) < tol


def pts(p0, p1):
    return [round(v, 3) for v in list(p0) + list(p1)]


def cyl_detail(p0, p1, r):
    """The cylinders Chapter 29's carrier cuts and joins that the detail changes, each recognised by its radius and its
    ends: the cups' bores to the cups, the nose's middle bore stepped. The case's shoulders are not changed here: their
    cylinder lies inside the case's sphere, so relieve_case() cuts a ring from each end after the build."""
    L = XD.carrier_layout()
    on_x = p0[1] == p0[2] == p1[1] == p1[2] == 0
    on_y = p0[0] == p0[2] == p1[0] == p1[2] == 0
    if on_x and near(r, A_[1] / 2 + 0.05) and near(p1[0], L['cupA'][1], 1e-3):
        r = A_[1] / 2
    elif on_x and near(r, B_[1] / 2 + 0.05) and near(p0[0], L['cupB'][0], 1e-3):
        r = B_[1] / 2
    elif on_x and near(r, 46.5) and near(p0[0], L['cupA'][1] - 0.01, 1e-3):
        a = _orig['cyl'](p0, (L['step'], 0, 0), DIA['middle_A'] / 2)
        mp.union(a, _orig['cyl']((L['step'] - 0.01, 0, 0), p1, DIA['middle_B'] / 2))
        return a
    elif on_y and near(r, C_[1] / 2 + 0.05):
        r = C_[1] / 2
    return _orig['cyl'](p0, p1, r)


def tube_detail(p0, p1, r_out, r_in):
    """The tubes the detail changes: the spacer from cone face to cone face, the companion flange's hub relieved where
    it meets cone B, the adjusters relieved where they meet the cups."""
    L = XD.carrier_layout()
    on_x = p0[1] == p0[2] == p1[1] == p1[2] == 0
    on_y = p0[0] == p0[2] == p1[0] == p1[2] == 0
    if on_x and near(r_out, 28.0) and near(p0[0], L['spacer_v2'][0], 1e-3):
        return _orig['tube']((L['spacer'][0], 0, 0), (L['spacer'][1], 0, 0), r_out, r_in)
    if on_x and near(r_out, 30.0) and near(r_in, 15.0) and near(p0[0], L['xB'], 1e-3):
        h = XD.RELIEF['flange']
        t = _orig['tube'](p0, (p0[0] + h, 0, 0), DIA['hub_relief'] / 2, r_in)
        mp.union(t, _orig['tube']((p0[0] + h - 0.01, 0, 0), p1, r_out, r_in))
        return t
    if on_y and near(r_out, C_[1] / 2) and near(r_in, 36.0) and near(abs(p0[1]), YC):
        s = 1 if p0[1] > 0 else -1
        h = XD.RELIEF['adjuster']
        t = _orig['tube'](p0, (0, s * (YC + h), 0), r_out, DIA['adjuster_relief'] / 2)
        mp.union(t, _orig['tube']((0, s * (YC + h - 0.01), 0), p1, r_out, r_in))
        return t
    return _orig['tube'](p0, p1, r_out, r_in)


def spline_engagement_detail(*a, **kw):
    kw.setdefault('d_minor', XD.SHAFT[0])
    kw.setdefault('d_major', XD.SHAFT[1])
    return _orig['spline_engagement'](*a, **kw)


DIA = {k: new for (what, brg, kind, v2, new), k in zip(XD.ABUTMENTS, (
    'shim', 'spacer_A', 'middle_A', 'bore_A', 'spacer_B', 'hub_relief', 'middle_B', 'bore_B', 'case_shoulder',
    'adjuster_relief'))}
_orig = {}


def _patch():
    _orig.update(cyl=mp.cyl, tube=mp.tube, spline_engagement=ad.spline_engagement, car_spline=CAR.SHAFT_SPLINE,
                 hsg_spline=HSG.SPLINE)
    mp.cyl, mp.tube, ad.spline_engagement = cyl_detail, tube_detail, spline_engagement_detail
    CAR.SHAFT_SPLINE = HSG.SPLINE = XD.SHAFT


def _unpatch():
    mp.cyl, mp.tube, ad.spline_engagement = _orig['cyl'], _orig['tube'], _orig['spline_engagement']
    CAR.SHAFT_SPLINE, HSG.SPLINE = _orig['car_spline'], _orig['hsg_spline']


def design(pitch=2.0):
    XD.ADJ['p'] = pitch
    return XD.adjuster()


SY_ADJ = 530.0                           # the adjusters' 1045, cold drawn (Chapter 51's steel; Shigley Table A-20)


def refuse(pitch, a):
    allow = 0.577 * SY_ADJ / 2.0
    if a['tau_one'] > allow:
        raise ValueError('The adjusters\' thread falls short: at a pitch of %.1f mm one thread carries the case bearing\'s %.1f kN '
                         'at %.1f MPa in shear, over the %.1f a factor of 2 allows.' % (pitch, a['Fa'] / 1000,
                                                                                     a['tau_one'], allow))


# ---------- after the build: the bores split where the adjusters' threads begin, and the faces by hand ----------
def split_at_y(occ, y_abs, radius):
    comp = occ.component
    for s in (1, -1):
        y = s * y_abs
        faces = adsk.core.ObjectCollection.create()
        for b in comp.bRepBodies:
            for f in b.faces:
                g = f.geometry
                bb = f.boundingBox
                if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, radius, 1e-3) \
                        and bb.minPoint.y * 10 < y - 0.01 and bb.maxPoint.y * 10 > y + 0.01:
                    faces.add(f)
        if faces.count == 0:
            continue
        pi_ = comp.constructionPlanes.createInput()
        pi_.setByOffset(comp.xZConstructionPlane, VI(y * CM))
        pl = comp.constructionPlanes.add(pi_)
        pl.isLightBulbOn = False
        comp.features.splitFaceFeatures.add(comp.features.splitFaceFeatures.createInput(faces, pl, True))


def relieve_case(des):
    """The case's ends, where the cones' back faces sit: Chapter 29's sphere and shoulder leave them 80 across, over
    SKF's da max of 76, so a ring from 75 to 82 is cut 2 mm deep at each end, and the cone sits on 75."""
    for o in des.rootComponent.allOccurrences:
        if o.component.name != 'Differential case':
            continue
        h = XD.RELIEF['case']
        for s in (1, -1):
            t = _orig_tube((0, s * (CAR.HUB_Y - h), 0), (0, s * (CAR.HUB_Y + 0.01), 0), 41.0, DIA['case_shoulder'] / 2)
            m = o.transform2.copy()
            p = o.assemblyContext
            while p:
                m.transformBy(p.transform2)
                p = p.assemblyContext
            m.invert()
            mp.tbm().transform(t, m)
            comp = o.component
            body = comp.bRepBodies.item(0)
            bf = comp.features.baseFeatures.add()
            bf.startEdit()
            comp.bRepBodies.add(t, bf)
            bf.finishEdit()
            tools = adsk.core.ObjectCollection.create()
            tools.add(bf.bodies.item(0))
            ci = comp.features.combineFeatures.createInput(body, tools)
            ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
            comp.features.combineFeatures.add(ci)
        return o


def _orig_tube(p0, p1, r_out, r_in):
    return (_orig.get('tube') or mp.tube)(p0, p1, r_out, r_in)


def mark(f, value, kind, des, new):
    if not f.attributes.itemByName('mdfc', 'fit'):
        new.append(f)
    f.attributes.add('mdfc', 'fit', value)
    f.appearance = MF.appearance(des, MF.COLOURS[kind])


def by_hand(des):
    """The adjusters' threads, 6g on the adjusters and 6H in the carrier's and the caps' bores outboard of the cups;
    ISO 14's classes on the half shafts' splines and the side gears' splined bores (sliding), and on the pinion's and
    the companion flange's (fixed). The census meets a spline as many short cylinders of its tips and a thread as a
    cylinder, so the rules of Chapter 51 cannot give either its class."""
    root = des.rootComponent
    new, changed = [], []
    for o in root.allOccurrences:
        nm = o.component.name
        for b in o.bRepBodies:
            for f in b.faces:
                g = f.geometry
                bb = f.boundingBox
                if not isinstance(g, (adsk.core.Cylinder, adsk.core.Plane)):
                    continue
                cyl = isinstance(g, adsk.core.Cylinder)
                r = g.radius * 10 if cyl else None
                if nm.startswith('Adjuster') and cyl and near(r, C_[1] / 2, 1e-3):
                    mark(f, THREAD % '6g', 'sliding', des, new)
                elif (nm == 'Carrier casting' or nm.startswith('Bearing cap')) and cyl and near(r, C_[1] / 2, 1e-3) \
                        and min(abs(bb.minPoint.y), abs(bb.maxPoint.y)) * 10 > YC - 0.01:
                    mark(f, THREAD % '6H', 'sliding', des, new)
                else:
                    continue
                changed.append(f)
    sp = {'Half shaft': (XD.SHAFT, SPLINE_SHAFT), 'Side gear': (XD.SHAFT, SPLINE_HUB),
          'Pinion shaft': (XD.PINION, SPLINE_FIXED), 'Companion flange': (XD.PINION, SPLINE_HUB)}
    for o in root.allOccurrences:
        nm = o.component.name
        key = next((k for k in sp if nm.startswith(k)), None)
        if not key:
            continue
        (dmin, dmaj, n, w), cls = sp[key]
        c_d, c_D, c_B = cls.split()
        ax = (1.0, 0.0, 0.0) if key in ('Pinion shaft', 'Companion flange') else (0.0, 1.0, 0.0)
        for b in o.bRepBodies:
            for f in b.faces:
                g = f.geometry
                if isinstance(g, adsk.core.Cylinder) and near(g.radius * 10, dmin / 2, 0.15):
                    mark(f, '%s Ra 1.6' % c_d, 'keyed', des, new)
                elif isinstance(g, adsk.core.Plane) and _at(f, dmaj / 2, ax):
                    mark(f, '%s Ra 3.2' % c_D, 'keyed', des, new)
                elif isinstance(g, adsk.core.Plane) and _at(f, w / 2, ax):
                    mark(f, '%s Ra 1.6' % c_B, 'keyed', des, new)
                else:
                    continue
                changed.append(f)
    return changed, new


def _at(f, dist, ax):
    """A plane parallel to the spline's axis (which passes through the origin) at `dist` from it, to within the 0.1
    by which a hub's slots are drawn over the shaft's teeth: at half the width a flank, at the major radius a tooth's
    top or a slot's bottom."""
    g = f.geometry
    o = g.origin
    nv = g.normal
    along = abs(nv.x * ax[0] + nv.y * ax[1] + nv.z * ax[2])
    return along < 1e-6 and abs(abs(o.x * nv.x + o.y * nv.y + o.z * nv.z) * 10 - dist) < 0.15


def build(pitch=2.0):
    a = design(pitch)
    refuse(pitch, a)
    app = adsk.core.Application.get()
    _patch()
    try:
        msg31 = RA.build()
    finally:
        _unpatch()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    for o in root.allOccurrences:
        if o.component.name == 'Carrier casting' or o.component.name.startswith('Bearing cap'):
            split_at_y(o, YC, C_[1] / 2)
    relieve_case(des)
    msg51 = MF.apply('axle')
    changed, new = by_hand(des)
    sp, s = XD.spacer(), XD.splines()
    _last.update(adjuster=a, spacer=sp, splines=s, changed=changed, new=new, mf=dict(MF._last))
    return ('Rear axle in detail: the nose\'s middle bore %.0f at cup A and %.0f at cup B, the flange\'s hub %.0f where it '
            'meets cone B, the case\'s shoulders %.0f, the adjusters %.0f across at the cups; the spacer %.1f mm from cone to '
            'cone; the cups\' bores %.0f, %.0f and %.0f.\nSplines %d x %.0f x %.0f x %.0f (ISO 14), torsion factor %.2f; '
            'adjusters M%.0f x %.1f, minor %.2f, %.0f threads.\nBy hand: %d faces, %d of them new.\n%s\n\n[%s]'
            % (DIA['middle_A'], DIA['middle_B'], DIA['hub_relief'], DIA['case_shoulder'], DIA['adjuster_relief'],
               sp['L'], A_[1], B_[1], C_[1], XD.SHAFT[2], XD.SHAFT[0], XD.SHAFT[1], XD.SHAFT[3], s['n'],
               XD.ADJ['d'], pitch, a['dr'], a['nt'], len(changed), len(new), msg51.split('\n')[0], VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('pitch', 'Adjusters\' thread pitch (mm)', 2.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            vals = {k: float(inputs.itemById(k).value) for k, _, _ in FIELDS}
            ui.messageBox(build(**vals))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('AxleDetail failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        for k, label, v in FIELDS:
            cmd.commandInputs.addValueInput(k, label, '', adsk.core.ValueInput.createByReal(v))


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcAxleDetail'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Rear axle in detail', 'Chapter 54: the rear axle in detail')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('AxleDetail failed:\n%s' % traceback.format_exc())
