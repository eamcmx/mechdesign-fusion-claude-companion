# =============================================================================
# FlapActuator.py  -  Chapter 31 of "Mechanical Design in Autodesk Fusion with Claude": a trailing-edge flap driven
# by a ball screw. The screw turns, its nut runs aft along it, and a link from the nut to the flap's horn swings the
# flap down about its hinge: a slider-crank. Written from PROMPT.txt beside it; checked by checks/flap_check.py.
#
#   INPUTS        the dialog: lead L, screw diameter d0, ball friction mu, horn r, horn angle phi0, nut line zs,
#                 link l, flap chord c and span s, approach speed V
#   DERIVED       screw(): lead angle, efficiency driving and back-driving; drive(): the nut's travel to 40 deg,
#                 the screw's turns, the smallest transmission angle, the hinge moment, the nut's force and the
#                 screw's torque (both ways) at each flap angle
#   HARD RULES    build() refuses a transmission angle below 40 deg, a nut that does not move aft steadily, and a
#                 self-locking screw (a ball screw must back-drive and needs a brake)
#   CONVENTIONS   x aft, z up, the flap's hinge at the origin; bodies from TemporaryBRepManager; As-built Joints
#                 closing the loop nut-link-flap; one Motion Link screw to nut at its lead
#   ACCEPTANCE    the message; the nut at the lead per turn and the flap at the angle worked out here at every
#                 step; no interference; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'FlapActuator 1.0 (2026-11-01)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
FULL = 40.0                                     # deg of flap
ALPHA_B = math.radians(45.0)                    # the balls' contact angle
RHO, CN, XCP = 1.225, 1.2, 0.45                 # air density, normal-force coefficient at full flap, its point
MU_MIN = 40.0
Y_LINK = (17.0, 27.0)


def screw(L, d0, mu):
    lam = math.atan(L / (math.pi * d0))
    ca = math.cos(ALPHA_B)
    eta = (ca - mu * math.tan(lam)) / (ca + mu / math.tan(lam))           # driving the load
    eta_b = (ca - mu / math.tan(lam)) / (ca + mu * math.tan(lam))         # the load driving the screw
    return dict(lam=lam, eta=eta, eta_b=eta_b, mu_lock=math.tan(lam) * ca)


def horn_tip(r, phi0, d):
    """The horn's tip with the flap down by d: phi measured from +z towards +x (aft)."""
    return (r * math.sin(phi0 + d), r * math.cos(phi0 + d))


def nut_x(r, phi0, zs, l, d):
    """The nut's pin, forward of the horn's tip on the line z = zs, for the flap down by d."""
    P = horn_tip(r, phi0, d)
    if abs(zs - P[1]) >= l:
        raise ValueError('A link of %.1f mm cannot reach from the nut line to the horn (%.1f mm apart).'
                         % (l, abs(zs - P[1])))
    return P[0] - math.sqrt(l * l - (zs - P[1]) ** 2)


def drive(L, d0, mu, r, phi0_deg, zs, l, c, s, V):
    sc = screw(L, d0, mu)
    phi0 = math.radians(phi0_deg)
    q = 0.5 * RHO * V * V
    M_full = q * (c / 1000.0) * (s / 1000.0) * CN * XCP * (c / 1000.0)        # N m at full flap
    rows, mu_min, steady = [], 180.0, True
    for k in range(0, int(FULL * 2) + 1):
        d = math.radians(k / 2.0)
        x = nut_x(r, phi0, zs, l, d)
        e = 1e-6
        dxdd = (nut_x(r, phi0, zs, l, d + e) - nut_x(r, phi0, zs, l, d - e)) / (2 * e)     # mm per rad
        if dxdd <= 0:
            steady = False
        P = horn_tip(r, phi0, d)
        v1, v2 = (P[0] - x, P[1] - zs), P
        mu_min = min(mu_min, math.degrees(math.acos(min(1.0, abs(v1[0] * v2[0] + v1[1] * v2[1])
                                                          / (math.hypot(*v1) * math.hypot(*v2))))))
        M = M_full * (k / 2.0) / FULL                                             # hinge moment, linear in d
        F = M / (dxdd / 1000.0) if dxdd > 0 else float('inf')                     # N, by virtual work
        T = F * L / 1000.0 / (2 * math.pi * sc['eta'])                            # N m to drive
        Tb = F * L / 1000.0 * sc['eta_b'] / (2 * math.pi)                         # N m the load puts back
        rows.append((k / 2.0, x, dxdd, M, F, T, Tb))
    x0 = rows[0][1]
    travel = rows[-1][1] - x0
    worst = max(rows, key=lambda r_: r_[4])
    return dict(sc=sc, q=q, M_full=M_full, rows=rows, x0=x0, travel=travel, turns=travel / L, mu_min=mu_min,
                steady=steady, F_max=worst[4], d_Fmax=worst[0], T_max=worst[5], Tb_max=worst[6], phi0=phi0)


def flap_angle(r, phi0, zs, l, xn):
    """Inverse: the flap's angle for a nut at xn, by bisection on 0 to the full angle."""
    lo, hi = 0.0, math.radians(FULL + 5.0)
    for _ in range(100):
        mid = (lo + hi) / 2
        if nut_x(r, phi0, zs, l, mid) < xn:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# ---------- Fusion building blocks (mm at the interface; the mechanism's plane is xz) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cyl(p0, p1, r):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r * CM, P3(*[v * CM for v in p1]), r * CM)


def box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    return tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM,
                                                                   (y1 - y0) * CM, (z1 - z0) * CM))


def pin(p, y0, y1, r):
    return cyl((p[0], y0, p[1]), (p[0], y1, p[1]), r)


def bar(p, q, w, ys):
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    ux, uz = (q[0] - p[0]) / L, (q[1] - p[1]) / L
    c = P3((p[0] + q[0]) / 2 * CM, (ys[0] + ys[1]) / 2 * CM, (p[1] + q[1]) / 2 * CM)
    body = tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(ux, 0, uz), V3(0, 1, 0), L * CM,
                                                                  (ys[1] - ys[0]) * CM, w * CM))
    union(body, pin(p, ys[0], ys[1], w / 2), pin(q, ys[0], ys[1], w / 2))
    return body


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def cut(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.DifferenceBooleanType, *bs)


def add_body(comp, tb, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def new_part(root, name):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def joint_at(root, moving, fixed, p_xyz, axis, kind, name):
    """An As-built joint at a world point: revolute about the world `axis` or slider along it; the joint directions
    are tried and the one that moves the part about (along) `axis` is kept (Chapter 17). Returns (joint, sign)."""
    J = adsk.fusion.JointDirections
    sk = moving.component.sketches.add(moving.component.xZConstructionPlane)
    sk.isVisible = False
    pt = sk.sketchPoints.add(sk.modelToSketchSpace(P3(*[v * CM for v in p_xyz]))).createForAssemblyContext(moving)
    for direction in (J.XAxisJointDirection, J.YAxisJointDirection, J.ZAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        if kind == 'slider':
            inp.setAsSliderJointMotion(direction)
        else:
            inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        if kind == 'slider':
            t0 = moving.transform2.translation
            j.jointMotion.slideValue = 0.05
            t1 = moving.transform2.translation
            j.jointMotion.slideValue = 0.0
            w = (t1.x - t0.x, t1.y - t0.y, t1.z - t0.z)
            n = math.sqrt(sum(v * v for v in w)) or 1.0
            w = tuple(v / n for v in w)
        else:
            r0 = rotation(moving)
            j.jointMotion.rotationValue = 0.05
            r1 = rotation(moving)
            j.jointMotion.rotationValue = 0.0
            D = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
            w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        dot = sum(w[i] * axis[i] for i in range(3))
        off = sum(w[i] * w[i] for i in range(3)) - dot * dot
        if abs(dot) > 0.02 and off < 1e-6:
            j.name = name
            return j, (1 if dot > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction moves %s about %s' % (moving.name, axis))


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the flap ----------
def build(L=5.0, d0=16.0, mu=0.005, r=100.0, phi0=-45.0, zs=60.0, l=150.0, c=400.0, s=1500.0, V=40.0):
    g = drive(L, d0, mu, r, phi0, zs, l, c, s, V)
    if not g['steady']:
        raise ValueError('The nut does not move aft all the way to %.0f deg: the link would pass its dead centre.'
                         % FULL)
    if g['mu_min'] < MU_MIN:
        raise ValueError('The link and the horn come within %.2f deg of a straight line (transmission angle): '
                         'below %.0f deg.' % (g['mu_min'], MU_MIN))
    if mu >= g['sc']['mu_lock']:
        raise ValueError('With a friction coefficient of %.3f the screw would lock itself (at %.4f): a ball screw '
                         'is chosen to run free, with a brake.' % (mu, g['sc']['mu_lock']))
    phi = g['phi0']
    x0 = g['x0']
    P0 = horn_tip(r, phi, 0.0)
    N0 = (x0, zs)
    x_end = x0 + g['travel']

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    half = s / 2.0
    wg = new_part(root, 'Wing')
    spar_x = x0 - 60.0
    wb = box(spar_x - 40, spar_x, -half - 30, half + 30, -40, 140)                       # the rear spar
    for side in (1, -1):                                                                # hinge brackets
        y0, y1 = sorted((side * (half + 5), side * (half + 25)))
        union(wb, box(spar_x - 1, 25, y0, y1, -25, 25))
    union(wb, pin((0.0, 0.0), -half - 25, half + 25, 8.0))                              # the hinge pin
    add_body(wg.component, wb, 'Wing')
    wg.isGrounded = True

    fl = new_part(root, 'Flap')
    fb = box(10, c, -half, half, -15, 15)                                               # the flap's slab
    union(fb, pin((0.0, 0.0), -half, half, 20.0))                                       # its nose round the hinge
    union(fb, bar((0.0, 0.0), P0, 24.0, (-10.0, 10.0)), pin(P0, 10.0 - 0.001, Y_LINK[1], 6.0))   # horn and pin
    cut(fb, pin((0.0, 0.0), -half - 1, half + 1, 8.2))
    add_body(fl.component, fb, 'Flap')

    sw = new_part(root, 'Screw')
    add_body(sw.component, cyl((spar_x + 0.5, 0, zs), (x_end + 25, 0, zs), d0 / 2), 'Screw')

    nt = new_part(root, 'Nut')
    nb = box(x0 - 20, x0 + 20, -15, 15, zs - 16, zs + 16)
    union(nb, pin(N0, 15.0 - 0.001, Y_LINK[1], 6.0))
    cut(nb, cyl((x0 - 21, 0, zs), (x0 + 21, 0, zs), d0 / 2 + 0.2))
    add_body(nt.component, nb, 'Nut')

    lk = new_part(root, 'Link')
    lb = bar(N0, P0, 20.0, Y_LINK)
    cut(lb, pin(N0, Y_LINK[0] - 1, Y_LINK[1] + 1, 6.2), pin(P0, Y_LINK[0] - 1, Y_LINK[1] + 1, 6.2))
    add_body(lk.component, lb, 'Link')

    js, s_s = joint_at(root, sw, wg, (spar_x, 0, zs), (1, 0, 0), 'revolute', 'Screw turns')
    jn, s_n = joint_at(root, nt, wg, (x0, 0, zs), (1, 0, 0), 'slider', 'Nut runs')
    joint_at(root, lk, nt, (N0[0], 0, N0[1]), (0, 1, 0), 'revolute', 'Link on nut')
    joint_at(root, fl, wg, (0, 0, 0), (0, 1, 0), 'revolute', 'Flap on hinge')
    joint_at(root, lk, fl, (P0[0], 0, P0[1]), (0, 1, 0), 'revolute', 'Link on horn')          # closes the loop
    li = root.motionLinks.createInput(js, jn)
    li.valueOne = VS('360 deg')                                    # one turn of the screw ...
    li.valueTwo = VI(L * CM)                                       # ... moves the nut one lead
    li.isReversed = s_s * s_n > 0                                  # right-hand: a turn about +x moves the nut to -x
    root.motionLinks.add(li).name = 'Ball screw'

    colour(design, wg, 'Paint - Enamel Glossy (Grey)')
    colour(design, fl, 'Paint - Enamel Glossy (Blue)')
    colour(design, nt, 'Paint - Enamel Glossy (Yellow)')
    colour(design, lk, 'Paint - Enamel Glossy (Red)')
    app.activeViewport.fit()

    sc = g['sc']
    return ('Flap actuator built: ball screw %.0f x %.0f (lead angle %.3f deg), horn %.0f mm at %.0f deg, link %.0f '
            'mm, nut line %.0f mm above the hinge.\n\n'
            'Nut travel to %.0f deg of flap: %.3f mm, %.3f turns of the screw; smallest transmission angle %.2f deg.\n'
            'Efficiency %.4f driving, %.4f back-driving (self-locking only above mu = %.4f): the screw needs a brake.\n'
            'At %.0f m/s (q = %.1f Pa) the hinge moment reaches %.2f N m; the largest nut force is %.1f N at %.1f deg, '
            'which takes %.4f N m at the screw and puts %.4f N m back on it.\n\n'
            'Drive Joints on "Screw turns": the nut, the link and the flap follow.\n\n[%s]'
            % (d0, L, math.degrees(sc['lam']), r, phi0, l, zs, FULL, g['travel'], g['turns'], g['mu_min'], sc['eta'],
               sc['eta_b'], sc['mu_lock'], V, g['q'], g['M_full'], g['F_max'], g['d_Fmax'], g['T_max'], g['Tb_max'],
               VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('L', 'Lead', 'mm', 5.0), ('d0', 'Screw diameter', 'mm', 16.0), ('r', 'Horn radius', 'mm', 100.0),
          ('phi0', 'Horn angle, flap up', 'deg', -45.0), ('zs', 'Nut line above the hinge', 'mm', 60.0),
          ('l', 'Link', 'mm', 150.0), ('c', 'Flap chord', 'mm', 400.0), ('s', 'Flap span', 'mm', 1500.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            v = {}
            for key, _, unit, _ in FIELDS:
                x = ins.itemById(key).value
                v[key] = x / CM if unit == 'mm' else math.degrees(x)
            ui.messageBox(build(**v), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, unit, d in FIELDS:
            ins.addValueInput(key, label, unit, VI(d * CM if unit == 'mm' else math.radians(d)))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'FlapActuatorCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap actuator', 'A flap driven by a ball screw')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Flap actuator could not start:\n' + traceback.format_exc())
