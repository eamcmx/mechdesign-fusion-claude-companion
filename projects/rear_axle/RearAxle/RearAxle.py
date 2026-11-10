# =============================================================================
# RearAxle.py  -  Chapter 31 of "Mechanical Design in Autodesk Fusion with Claude": the rear axle put together. The
# carrier of RearAxleCarrier.py (Chapter 29) is bolted into the housing of AxleHousing.py (Chapter 30) and each half
# shaft is splined into its side gear, so that the axle turns as one machine: the pinion drives the case through the
# spiral bevel pair, the case drives the wheels through the spiders and the side gears, and in a turn the side gears
# let the outer wheel run faster than the inner one. The two subsystems' scripts are loaded and their own builders
# used; nothing is drawn twice. Written from PROMPT.txt beside it; checked by checks/rear_axle_check.py.
#
#   INPUTS        none: each subsystem keeps its own defaults and its own refusals
#   DERIVED       RearAxleCarrier.design() and layout(); AxleHousing.design()
#   HARD RULES    the subsystems' own; the splines' engagement in the side gears (lib/axle_design.spline_engagement)
#   CONVENTIONS   the axle along Y, forward +X, up +Z (ISO 8855: Y to the left), the axle's middle at the origin;
#                 five subassemblies (housing, carrier, pinion, differential, two wheel ends), every part a component
#   ACCEPTANCE    the message; no interference; the axle turning straight and in a turn
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
import axle_design as ad        # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(ad)
importlib.reload(mp)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CAR = _load('RearAxleCarrier')
HSG = _load('AxleHousing')
VERSION = 'RearAxle 1.0 (2026-11-01)'
VS = adsk.core.ValueInput.createByString


def point_at_origin(occ):
    comp = occ.component
    sk = comp.sketches.add(comp.xZConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0)).createForAssemblyContext(occ)


def rigid(root, moving, fixed, name):
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(point_at_origin(moving)))
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def link(root, j1, s1, n1, j2, s2, n2, same_sense, name):
    """A motion link: joint one turns n1 while joint two turns n2. same_sense says whether the two parts turn the same
    way about the axes their joints were made about; s1 and s2 are the joints' own signs on those axes."""
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * n1))
    li.valueTwo = VS('%d deg' % (360 * n2))
    li.isReversed = ((1 if same_sense else -1) * s1 * s2) < 0
    m = root.motionLinks.add(li)
    m.name = name
    return m


def design():
    D = CAR.design()
    Dz = HSG.design()
    x = D['diff']
    sp = ad.spline_engagement(HSG.Y_TIP, HSG.SPLINE_END, CAR.HUB_Y + ad.SKF_TRB[CAR.CASE_BRG][2],
                              ad.duty(ad.brief())['T_shaft'] * 1000.0)
    return dict(D=D, Dz=Dz, sp=sp, z_s=x['z_s'], z_p=x['z_p'], N=x['N'])


def refuse(A):
    CAR.refuse(A['D'])
    HSG.refuse(A['Dz'])
    sp = A['sp']
    if sp['l'] < sp['l_need']:
        raise ValueError('The half shafts\' splines reach %.2f mm into the side gears; they need %.1f mm.'
                         % (sp['l'], sp['l_need']))


def build():
    A = design()
    refuse(A)
    D, Dz = A['D'], A['Dz']
    L = CAR.layout(D)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    hs, _ = HSG.housing(root, Dz)
    hs.isGrounded = True
    car, _ = CAR.carrier(root, D, L)
    rigid(root, car, hs, 'Carrier bolted to the housing')
    pin, _ = CAR.pinion_assembly(root, D, L)
    dif, _, movers, _ = CAR.differential_assembly(root, D, L)
    jp, sp_ = CAR.revolute_about(root, pin, car, (1, 0, 0), 'Pinion turns')
    jc, sc_ = CAR.revolute_about(root, dif, car, (0, 1, 0), 'Case turns')
    g = D['g']
    k = math.gcd(g['z1'], g['z2'])
    link(root, jp, sp_, g['z2'] // k, jc, sc_, g['z1'] // k, True, 'Final drive %d:%d' % (g['z1'], g['z2']))
    case_px = [o for o in dif.component.occurrences if o.component.name == 'Differential case'][0].createForAssemblyContext(dif)
    J = {}
    for name, (occ, axis) in movers.items():
        J[name] = CAR.revolute_about(root, occ.createForAssemblyContext(dif), case_px, axis, name + ' turns in the case')
    # the differential: each spider rolls on both side gears; seen along their own axes (from the middle outwards),
    # a spider turns the other way to a side gear, z_s/z_p as fast; the two side gears turn the same way about their
    # own axes, so opposite ways about Y
    zs, zp = A['z_s'], A['z_p']
    kk = math.gcd(zs, zp)
    jL, sL = J['Side gear L']
    jR, sR = J['Side gear R']
    j1, s1 = J['Spider 1']
    link(root, jL, sL, zp // kk, j1, s1, zs // kk, False, 'Side gear L and spider 1')
    link(root, j1, s1, zs // kk, jR, sR, zp // kk, False, 'Spider 1 and side gear R')
    for i in range(2, A['N'] + 1):
        ji, si = J['Spider %d' % i]
        link(root, jL, sL, zp // kk, ji, si, zs // kk, False, 'Side gear L and spider %d' % i)
    # the wheel ends: each half shaft splined into its side gear (L at +y, R at -y, as in both subsystems)
    ends = {}
    for s, tag in ((1, 'L'), (-1, 'R')):
        e = HSG.wheel_end_asm(root, Dz, s)[0]
        gear = [o for o in dif.component.occurrences if o.component.name == 'Side gear %s' % tag][0]
        rigid(root, e, gear.createForAssemblyContext(dif), 'Half shaft %s in side gear %s' % (tag, tag))
        ends[tag] = e
    for occ, c in ((hs, 'Paint - Enamel Glossy (Grey)'), (car, 'Paint - Enamel Glossy (Grey)'),
                   (pin, 'Paint - Enamel Glossy (Blue)'), (dif, 'Paint - Enamel Glossy (Yellow)'),
                   (ends['L'], 'Paint - Enamel Glossy (Blue)'), (ends['R'], 'Paint - Enamel Glossy (Blue)')):
        mp.colour(design_, occ, c)
    app.activeViewport.fit()
    pl = CAR.parts_list(root)
    sp = A['sp']
    return ('Rear axle built: the carrier of Chapter 29 bolted into the housing of Chapter 30, the half shafts splined '
            'into the side gears.\n'
            'Final drive %d:%d; differential %d spiders of %d teeth, side gears of %d.\n'
            'Splines %d x %.0f x %.0f engaged %.2f mm in each side gear (%.1f mm needed): %.1f MPa on the flanks, '
            'factor %.2f.\n\nParts: %d components in %d kinds; %s.\n\n[%s]'
            % (g['z1'], g['z2'], A['N'], zp, zs, HSG.SPLINE[2], HSG.SPLINE[0], HSG.SPLINE[1], sp['l'], sp['l_need'],
               sp['p'], sp['n'], sum(pl.values()), len(pl), ', '.join('%d %s' % (v, kk_) for kk_, v in sorted(pl.items())),
               VERSION))


def run(_context):
    ui = adsk.core.Application.get().userInterface
    try:
        ui.messageBox(build(), VERSION)
    except ValueError as e:
        ui.messageBox(str(e), VERSION)
    except Exception:
        ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)
