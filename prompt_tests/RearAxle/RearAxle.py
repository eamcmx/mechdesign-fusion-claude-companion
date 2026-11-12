# =============================================================================
# RearAxle.py  -  the rear axle of a light truck put together from its two subsystems: the carrier of
# RearAxleCarrier.py bolted into the banjo housing of AxleHousing.py, each half shaft splined into its side gear.
# Both scripts are loaded (not copied) and their own builders draw every part; this script adds the joints and the
# Motion Links that make the axle turn as one machine, and the check of the splines' engagement in the side gears
# (lib/axle_design.spline_engagement). Written from PROMPT.txt; checked by checks/rear_axle_check.py.
#
#   INPUTS        no dialog: each subsystem keeps its own defaults and refusals; Y_TIP of AxleHousing.py (where the
#                 half shafts' splines start, 35 mm from the axle's middle) is read each time the design is worked
#   DERIVED       RearAxleCarrier.design() and layout(), AxleHousing.design(); the splines engaged from the shaft's
#                 tip to the end of the side gear's hub (HUB_Y + T of the case's bearing), their pressure at the
#                 design torque of one half shaft and their factor against the shaft's yield
#   HARD RULES    refuse() refuses what either subsystem refuses, and splines that reach less than the length needed
#   CONVENTIONS   the axle along Y, forward +X, up +Z, Y to the left (ISO 8855), the axle's middle at the origin, L at
#                 +Y and R at -Y; six subassemblies: Housing (grounded), Carrier, Pinion assembly, Differential
#                 assembly, Wheel end L, Wheel end R
#   ACCEPTANCE    the message; 106 components in 43 kinds; no interference at rest, driving straight or in a turn
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import inspect
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
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script


def load_script(name):
    """A script of the companion loaded as a module from its own file (each load reads the file afresh)."""
    path = CP.script(name)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CAR = load_script('RearAxleCarrier')    # the carrier: design(), layout(), refuse() and its builders
HSG = load_script('AxleHousing')        # the housing and the wheel ends: design(), refuse() and their builders
import axle_design as ad        # noqa: E402   (reloaded by the two scripts above)
import mdfc_parts as mp         # noqa: E402

VERSION = 'RearAxle 1.0 (2026-11-11)'
VS = adsk.core.ValueInput.createByString


# ---------- the design ----------
def design():
    """Both subsystems' designs, and the splines of the half shafts in the side gears."""
    car = CAR.design()
    hsg = HSG.design()
    C = ad.SKF_TRB[CAR.CASE_BRG]
    y_hub = CAR.HUB_Y + C[2]                    # the side gear's splined hub ends at the case bearing's outer face
    T = car['d']['T_shaft'] * 1000.0            # the design torque of one half shaft, N mm
    d_minor, d_major, n, _w = HSG.SPLINE
    Sy = inspect.signature(ad.wheel_end).parameters['Sy'].default     # the half shaft's yield, as it was sized
    sp = ad.spline_engagement(HSG.Y_TIP, HSG.SPLINE_END, y_hub, T, d_minor=d_minor, d_major=d_major, n=n, Sy=Sy)
    sp.update(y_tip=HSG.Y_TIP, y_end=HSG.SPLINE_END, y_hub=y_hub, T=T, Sy=Sy, d_minor=d_minor, d_major=d_major, z=n)
    return dict(car=car, L=CAR.layout(car), hsg=hsg, sp=sp)


def refuse(D):
    """What either subsystem refuses, then splines too short in the side gears. Returns the ground clearance."""
    clr = CAR.refuse(D['car'])
    HSG.refuse(D['hsg'])
    sp = D['sp']
    if sp['l'] <= 0:
        raise ValueError('The splines start at %.1f mm, past the end of the side gear\'s hub at %.2f mm: they do not '
                         'reach into the side gears; %.1f mm is needed.' % (sp['y_tip'], sp['y_hub'], sp['l_need']))
    if sp['l'] < sp['l_need']:
        raise ValueError('The splines reach %.2f mm into the side gears (from %.1f mm to the hub\'s end at %.2f mm), '
                         'less than the %.1f mm needed.' % (sp['l'], sp['y_tip'], sp['y_hub'], sp['l_need']))
    return clr


# ---------- joints ----------
def _spin(r0, r1):
    """Twice the sine of the turn from r0 to r1, along its axis, in the world."""
    Dm = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    return (Dm[2][1] - Dm[1][2], Dm[0][2] - Dm[2][0], Dm[1][0] - Dm[0][1])


def revolute_plus(root, moving, fixed, axis, name):
    """The carrier's revolute_about(), with the joint's positive turn about +axis: if Fusion's joint turns the part
    the other way, the joint is made again with its two occurrences the other way round."""
    j, s = CAR.revolute_about(root, moving, fixed, axis, name)
    if s > 0:
        return j, s
    j.deleteMe()
    J = adsk.fusion.JointDirections
    for direction in (J.ZAxisJointDirection, J.XAxisJointDirection, J.YAxisJointDirection):
        geo = adsk.fusion.JointGeometry.createByPoint(CAR.origin_point(moving))
        inp = root.asBuiltJoints.createInput(fixed, moving, geo)
        inp.setAsRevoluteJointMotion(direction)
        jj = root.asBuiltJoints.add(inp)
        r0 = CAR.rotation(moving)
        jj.jointMotion.rotationValue = 0.2
        r1 = CAR.rotation(moving)
        jj.jointMotion.rotationValue = 0.0
        if sum(a * b for a, b in zip(_spin(r0, r1), axis)) > 0.1:
            jj.name = name
            return jj, 1
        jj.deleteMe()
    return CAR.revolute_about(root, moving, fixed, axis, name)       # as Fusion gives it; the sign says which way


def rigid(root, one, two, name):
    """An As-built rigid joint: `one` held where it was built on `two`."""
    try:
        inp = root.asBuiltJoints.createInput(one, two, None)
        inp.setAsRigidJointMotion()
        j = root.asBuiltJoints.add(inp)
    except Exception:
        geo = adsk.fusion.JointGeometry.createByPoint(CAR.origin_point(one))
        inp = root.asBuiltJoints.createInput(one, two, geo)
        inp.setAsRigidJointMotion()
        j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _native(j):
    try:
        n = j.nativeObject
    except Exception:
        n = None
    return j if n is None else n


def motion_link(root, one, two, v_one, v_two, reverse, name):
    """A Motion Link: `one` turns v_one degrees while `two` turns v_two, the same way unless `reverse`. A joint made in
    the root between two parts of one subassembly is kept by that subassembly, so the link is made there first."""
    n1, n2 = _native(one), _native(two)
    try:
        c1, c2 = n1.parentComponent, n2.parentComponent
    except Exception:
        c1 = c2 = None
    tries = []
    if c1 is not None and c2 is not None and c1 == c2 and c1 != root:
        tries.append(lambda: (c1, n1, n2))

        def in_root():
            o = root.allOccurrencesByComponent(c1).item(0)
            return root, n1.createForAssemblyContext(o), n2.createForAssemblyContext(o)
        tries.append(in_root)
    tries.append(lambda: (root, one, two))
    errors = []
    for make in tries:
        try:
            comp, a, b = make()
            li = comp.motionLinks.createInput(a, b)
            li.valueOne = VS('%d deg' % v_one)
            li.valueTwo = VS('%d deg' % v_two)
            li.isReversed = bool(reverse)
            m = comp.motionLinks.add(li)
            m.name = name
            return m
        except Exception as e:
            errors.append(str(e))
    raise RuntimeError('the Motion Link %s could not be made: %s' % (name, '; '.join(errors)))


# ---------- the axle ----------
def build():
    D = design()
    clr = refuse(D)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    Dc, L, Dh = D['car'], D['L'], D['hsg']
    # the six subassemblies, each drawn by its own subsystem's builder, where that subsystem puts it: both put the
    # axle's middle at the origin, L at +Y, and the carrier's plate on the neck's flange at x = 165
    hs, _ = HSG.housing(root, Dh)
    hs.isGrounded = True
    car, _ = CAR.carrier(root, Dc, L)
    pin, _ = CAR.pinion_assembly(root, Dc, L)
    dif, _, movers, _ = CAR.differential_assembly(root, Dc, L)
    ends = {}
    for s in (1, -1):
        e, _ = HSG.wheel_end_asm(root, Dh, s)
        ends['L' if s > 0 else 'R'] = e
    # the carrier on the housing; the pinion and the case on the carrier, linked by the final drive
    rigid(root, car, hs, 'Carrier on the housing')
    jp, sp_ = revolute_plus(root, pin, car, (1, 0, 0), 'Pinion turns')
    jc, sc_ = revolute_plus(root, dif, car, (0, 1, 0), 'Case turns')
    g = Dc['g']
    k = math.gcd(g['z1'], g['z2'])
    motion_link(root, jp, jc, 360 * g['z2'] // k, 360 * g['z1'] // k, (sp_ * sc_) < 0,      # the case about +Y when
                'Final drive %d:%d' % (g['z1'], g['z2']))                                 # the pinion turns about +X
    # the spiders and the side gears in the case, made in the root between proxies
    case_occ = [o for o in dif.component.occurrences if o.component.name == 'Differential case'][0]
    case_px = case_occ.createForAssemblyContext(dif)
    J = {}
    for nm, (occ, axis) in movers.items():
        J[nm] = CAR.revolute_about(root, occ.createForAssemblyContext(dif), case_px, axis, nm + ' turns in the case')
    # each half shaft rigid in its side gear: its wheel end, in which the half shaft is grounded, held on the gear
    for tag, e in ends.items():
        gear_px = movers['Side gear %s' % tag][0].createForAssemblyContext(dif)
        rigid(root, e, gear_px, 'Half shaft %s in its side gear' % tag)
    # the differential: each spider turns z_s for z_p of a side gear, the other way as seen along their own axes
    # (both axes pointing out of the case's centre); side gear L to every spider, spider 1 to side gear R
    x = Dc['diff']
    kd = math.gcd(x['z_s'], x['z_p'])
    t_s, t_p = 360 * x['z_s'] // kd, 360 * x['z_p'] // kd       # degrees of a spider, of a side gear
    jL, sL = J['Side gear L']
    for i in range(1, x['N'] + 1):
        jS, sS = J['Spider %d' % i]
        motion_link(root, jL, jS, t_p, t_s, (sL * sS) > 0, 'Side gear L to spider %d' % i)
    jS1, sS1 = J['Spider 1']
    jR, sR = J['Side gear R']
    motion_link(root, jS1, jR, t_s, t_p, (sS1 * sR) > 0, 'Spider 1 to side gear R')
    for occ, c in ((hs, 'Paint - Enamel Glossy (Grey)'), (car, 'Paint - Enamel Glossy (Grey)'),
                   (pin, 'Paint - Enamel Glossy (Blue)'), (dif, 'Paint - Enamel Glossy (Yellow)'),
                   (ends['L'], 'Paint - Enamel Glossy (Blue)'), (ends['R'], 'Paint - Enamel Glossy (Blue)')):
        mp.colour(des, occ, c)
    app.activeViewport.fit()
    pl = CAR.parts_list(root)
    pk, d, sp = Dc['peak'], Dc['d'], D['sp']
    return ('Rear axle built: the carrier of RearAxleCarrier bolted into the housing of AxleHousing on its %d studs at '
            'x = %.0f mm, each half shaft splined into its side gear.\n'
            'Final drive: spiral bevel %d:%d (%.4f), module %.2f, face %.0f mm, crown wheel %.1f mm across, ground '
            'clearance %.1f mm, pinion bending factor %.3f at the design torque; the case turns %d times for %d turns '
            'of the pinion, about +Y when the pinion turns about +X.\n'
            'Differential: %d spiders of %d teeth on side gears of %d, module %.1f; each spider turns %d for %d of a '
            'side gear, the other way; in the tightest turn the outer wheel runs %.4f times as fast as the inner.\n'
            'Splines %d x %.0f x %.0f, from %.1f mm to %.1f mm, in side gears whose hubs end at %.2f mm: %.2f mm '
            'engaged, %.1f mm needed; %.1f MPa on the flanks at %.1f N m with half the teeth carrying it, factor %.2f '
            'against the shaft\'s yield of %.0f MPa.\n\n'
            'Parts: %d components in %d kinds; %s.\n\n[%s]'
            % (len(ad.joint_bolts()), HSG.NECK_X[1],
               g['z1'], g['z2'], g['z2'] / g['z1'], g['m'], g['F'], g['da2'], clr, pk['SF1'], g['z1'] // k, g['z2'] // k,
               x['N'], x['z_p'], x['z_s'], x['m'], x['z_s'] // kd, x['z_p'] // kd, d['ratio_turn'],
               sp['z'], sp['d_minor'], sp['d_major'], sp['y_tip'], sp['y_end'], sp['y_hub'], sp['l'], sp['l_need'],
               sp['p'], sp['T'] / 1000.0, sp['n'], sp['Sy'],
               sum(pl.values()), len(pl), ', '.join('%d %s' % (v, kk) for kk, v in sorted(pl.items())), VERSION))


def run(_context):
    ui = adsk.core.Application.get().userInterface
    try:
        ui.messageBox(build(), VERSION)
    except ValueError as e:
        ui.messageBox(str(e), VERSION)
    except Exception:
        ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)
