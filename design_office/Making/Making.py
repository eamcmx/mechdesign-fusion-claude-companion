# =============================================================================
# Making.py  -  Chapter 66 of "Mechanical Design in Autodesk Fusion with Claude": making the parts. It takes a made
# part out of its machine as Part 10 left it and programs it in Fusion's Manufacture workspace: the setups, the
# operations with tools from Fusion's sample libraries, the toolpaths generated, each setup's machining time, and the
# NC program from Fusion's generic Fanuc posts.
#   bracket   the bag gripper's opening stop bracket P13-018 (Chapter 62: the arm turned, the root R2.5), milled from
#             a 1045 block in four setups: the bolting face and its two holes; the top, the part bolted to a
#             fixture plate through its own holes (face, adaptive roughing, the walls the top can reach); the lug's
#             M30 x 1.5 on an angle plate, and from that end the arm under the lug, which the top cannot reach (its
#             walls and root R2.5 by a 5 mm cutter, its end wall and that root by a 10 mm cutter of corner radius 2.5);
#             the arm under the lug from the lug's other end
#   spacer    the rear axle's pinion spacer A01-016 (Chapter 61: ground to each axle), turned from 60 x 40 tube,
#             0.8 mm long for the grinding
#   tine      the bag gripper's tine P13-012, a flat L cut from 10 mm plate: its inside corner rounded, the sixteen
#             nested on one plate by Fusion's Arrange (2D true shape), cut by the sample laser
#   census()  every made kind of the nine machines measured in Fusion (its faces, their kinds, axes and areas, its
#             smallest box), for lib/making.py to sort by the process that makes it
# Checked by checks/making_check.py.
#
#   INPUTS        the dialog: the part
#   DERIVED       lugs_brackets (the bracket's volume), drawings_detail (the spacer's grinding allowance), making (the
#                 tine's corner, the plate and the spacing), the machines through RangeSweep.py, the change through
#                 StopBracket.py, the bills of materials through BillOfMaterials.py
#   HARD RULES    every part copied from its machine's model; the tools from Fusion's sample libraries (one cutter
#                 copied and given the root's corner radius, one given 5 mm); nothing saved but the NC programs and
#                 the census
#   CONVENTIONS   a new design document per part; mm, the API's cm inside
#   ACCEPTANCE    the message: the setups, their operations and toolpaths, the times, the NC files
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import glob
import importlib
import importlib.util
import io
import json
import math
import os
import sys
import time
import traceback

import adsk.cam
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
import drawings_detail as DD     # noqa: E402
import making as MK              # noqa: E402

VERSION = 'Making 1.2 (2026-11-12)'
PARTS = ('bracket', 'spacer', 'tine')
NUMBERS = dict(bracket=(661, 662, 663, 664), spacer=(665,), tine=(667,))     # the NC programs' numbers, Fanuc's 1-9999
UNDER_LUG = 13.0                   # mm from the lug's end face: half its 25, and 0.5 over, cut from each end
NC = os.path.join(PROJECT, 'results', 'nc')
FEED_SCALE, RAPID, TOOL_CHANGE = 100.0, 5000.0, 10.0      # % of the programmed feed, mm/min rapid, s a tool change
_last = {}
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_mk66_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def post(name):
    """A post processor from Fusion's own install (the newest copy)."""
    root = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'Autodesk', 'webdeploy', 'production')
    found = glob.glob(os.path.join(root, '*', 'Applications', 'CAM360', 'Data', 'Posts', name + '.cps'))
    if not found:
        raise RuntimeError('no post %s.cps in Fusion\'s install' % name)
    return max(found, key=os.path.getmtime).replace('\\', '/')


def library(name):
    lm = adsk.cam.CAMManager.get().libraryManager.toolLibraries
    return lm.toolLibraryAtURL(adsk.core.URL.create('systemlibraryroot://Samples/%s.json' % name))


def tool(lib, desc=None, kind=None, d=None):
    for i in range(lib.count):
        t = lib.item(i)
        p = t.parameters
        if desc is not None and p.itemByName('tool_description').value.value.strip() == desc:
            return t
        if kind is not None and p.itemByName('tool_type').value.value == kind and \
                (d is None or abs(p.itemByName('tool_diameter').value.value * 10 - d) < 1e-6):
            return t
    raise RuntimeError('no tool %s in the library' % (desc or '%s %g' % (kind, d)))


def setp(params, name, expr, notes):
    try:
        params.itemByName(name).expression = expr
    except Exception as e:
        near = [params.item(i).name for i in range(params.count) if name[:5].lower() in params.item(i).name.lower()]
        notes.append('%s = %s refused (%s); near: %s' % (name, expr, str(e)[:40], ', '.join(near[:12])))


def operation(setup, strategy, t, params=(), select=None, notes=None):
    oi = setup.operations.createInput(strategy)
    oi.tool = t
    for n, v in params:
        setp(oi.parameters, n, v, notes)
    op = setup.operations.add(oi)
    if select:
        select(op)
    return op


def generate(cam, op, limit=600.0):
    f = cam.generateToolpath(op)
    t0 = time.time()
    while not f.isGenerationCompleted and time.time() - t0 < limit:
        adsk.doEvents()
        time.sleep(0.2)
    return f.isGenerationCompleted


# ---------- the parts, out of their machines ----------
def take(machine, part, change=None):
    """The part's body copied from its machine, world mm, as a temporary body; the machine closed unsaved."""
    app = adsk.core.Application.get()
    rsm = _load('RangeSweep')
    rsm.machine(machine)
    mdoc = app.activeDocument
    try:
        mdoc.attributes.add('mdfc', 'run', 'Making')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    occ = [o for o in des.rootComponent.allOccurrences if o.component.name == part and o.bRepBodies.count][0]
    if change:
        change(occ)
    body = occ.bRepBodies.item(0)
    tb = adsk.fusion.TemporaryBRepManager.get().copy(body)
    mat = body.material.name if body.material else None
    mdoc.close(False)
    return tb, mat


def into_frame(tb, rows):
    """Turn the body by the rows of a rotation (local x, y, z in world terms), then put its box's middle on z and its
    lowest point on z = 0."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    m = adsk.core.Matrix3D.create()
    (a, b, c), (d, e, f), (g, h, i) = rows
    m.setWithArray([a, b, c, 0, d, e, f, 0, g, h, i, 0, 0, 0, 0, 1])
    tbm.transform(tb, m)
    bb = tb.boundingBox
    t = adsk.core.Matrix3D.create()
    t.translation = V3(-(bb.minPoint.x + bb.maxPoint.x) / 2, -(bb.minPoint.y + bb.maxPoint.y) / 2, -bb.minPoint.z)
    tbm.transform(tb, t)
    return tb


def new_part_document(tb, mat, name):
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        doc.attributes.add('mdfc', 'run', 'Making')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    bf = root.features.baseFeatures.add()
    bf.startEdit()
    root.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = root.bRepBodies.item(0)
    body.name = name
    lib = app.materialLibraries.itemByName('Fusion Material Library')
    m = lib.materials.itemByName(mat) if mat else None
    if m is not None:
        body.material = m
    app.userInterface.workspaces.itemById('CAMEnvironment').activate()
    cam = adsk.cam.CAM.cast(doc.products.itemByProductType('CAMProductType'))
    return doc, body, cam


def faces(body, kind, r=None):
    st = dict(cyl=adsk.core.SurfaceTypes.CylinderSurfaceType, plane=adsk.core.SurfaceTypes.PlaneSurfaceType)[kind]
    return [f for f in body.faces if f.geometry.surfaceType == st and (r is None or abs(f.geometry.radius * 10 - r) < 1e-6)]


# ---------- the opening bracket, milled ----------
def bracket(notes):
    sb = _load('StopBracket')
    tb, mat = take('bag', 'Stop bracket open 1', lambda occ: sb.change(occ, 1, 2.5))
    # its own frame: the bolting face on z = 0, the arm up along z (world x), x along the foot's edge (world y)
    into_frame(tb, ((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    doc, body, cam = new_part_document(tb, mat, 'P13-018 Stop bracket, opening')
    holes = faces(body, 'cyl', 4.5)
    bore = faces(body, 'cyl', 15.0)
    arm = []
    for f in faces(body, 'plane'):
        ok, n = f.evaluator.getNormalAtPoint(f.pointOnFace)
        if abs(n.z) < 1e-6 and 12.0 < f.centroid.z * 10 < 40.0:
            arm.append(f)
    root_edges = [e for f in arm for e in f.edges
                  if all(abs(p.z * 10 - (12.0 + 2.5)) < 0.05 for p in (e.startVertex.geometry, e.endVertex.geometry))]
    ml, hl = library('Milling Tools (Metric)'), library('Hole Making Tools (Metric)')
    face_mill, flat12 = tool(ml, '50mm Face Mill'), tool(ml, '12mm Flat Endmill')
    flat10, thread_mill = tool(ml, '10mm Flat Endmill'), tool(ml, 'M10-1.5 Thread Mill')
    drill9 = tool(hl, kind='drill', d=9.0)
    # the root's cutter: the library's 10 mm bull nose, its corner radius made the drawing's 2.5
    bull = adsk.cam.Tool.createFromJson(tool(ml, '10mm Bullnose Endmill').toJson())
    setp(bull.parameters, 'tool_cornerRadius', '2.5 mm', notes)
    setp(bull.parameters, 'tool_description', "'10mm Bullnose R2.5'", notes)

    def new_setup(name, flip=False, axis=None):
        si = cam.setups.createInput(adsk.cam.OperationTypes.MillingOperation)
        si.models = [body]
        s = cam.setups.add(si)
        s.name = name
        for n, v in (('job_stockOffsetMode', "'simple'"), ('job_stockOffsetSides', '2.5 mm'), ('job_stockOffsetTop', '3 mm'),
                     ('job_stockOffsetBottom', '3 mm')):
            setp(s.parameters, n, v, notes)
        if flip:
            setp(s.parameters, 'wcs_orientation_flipZ', 'true', notes)
        if axis is not None:
            setp(s.parameters, 'wcs_orientation_mode', "'axesZX'", notes)
            s.parameters.itemByName('wcs_orientation_axisZ').value.value = [axis]
        return s
    a = new_setup('1 Bolting face and holes', flip=True)
    operation(a, 'face', face_mill, notes=notes)

    def sel_holes(op):
        op.parameters.itemByName('holeFaces').value.value = holes
    operation(a, 'drill', drill9, select=sel_holes, notes=notes)
    b = new_setup('2 Top, bolted to a fixture plate')
    operation(b, 'face', face_mill, notes=notes)
    operation(b, 'adaptive', flat12, [('stockToLeave', '0.3 mm')], notes=notes)
    # the walls the top can reach: the foot's, the lug's ends. The arm and its root R2.5 lie under the lug seen from
    # here (24 inside the lug's 25 along its axis, 24 inside its 48 across), so no cutter reaches them from the top
    operation(b, 'contour_new', flat12, [('maximumStepdown', '4 mm')], notes=notes)
    c = new_setup('3 The lug, M30 x 1.5', axis=bore[0])

    def sel_bore(op):
        op.parameters.itemByName('circularFaces').value.value = bore
    # the bore to the thread's minor diameter, 30 - 1.0825 x 1.5 = 28.38: 0.81 radial stock left on the 30 drawn
    operation(c, 'bore', flat10, [('stockToLeave', '0.81 mm')], sel_bore, notes)
    operation(c, 'thread', thread_mill, [('threadPitch', '1.5 mm')], sel_bore, notes)
    # the lug's round, a wall seen down its axis: its 25 mm length only, from the lug's end face down
    lugop = operation(c, 'contour_new', flat12, [('maximumStepdown', '4 mm')], notes=notes)
    bh = lugop.parameters.itemByName('bottomHeight_mode')
    default = bh.expression
    for v in ("'from surface top'", "'from stock top'", "'from top'"):
        try:
            bh.expression = v
            setp(lugop.parameters, 'bottomHeight_offset', '-25.5 mm', notes)
            break
        except Exception:
            continue
    notes.append('the lug contour: bottom height %s (default %s), %s' % (bh.expression, default,
                                                                          lugop.parameters.itemByName('bottomHeight_offset').expression))

    # the arm under the lug, cut along the lug's axis, half its length from each end: roughed; its walls along the
    # axis and their root by a 5 mm cutter, whose radius is the root's 2.5; its end wall and that root by the bull nose
    flat5 = adsk.cam.Tool.createFromJson(flat10.toJson())
    setp(flat5.parameters, 'tool_diameter', '5 mm', notes)
    setp(flat5.parameters, 'tool_description', "'5mm Flat Endmill'", notes)

    def half_depth(op, label):
        bh_ = op.parameters.itemByName('bottomHeight_mode')
        for v, extra in (("'from model top'", 0.0), ("'from surface top'", 0.0), ("'from stock top'", 3.0),
                         ("'from top'", 3.0)):
            try:
                bh_.expression = v
                setp(op.parameters, 'bottomHeight_offset', '%.1f mm' % -(UNDER_LUG + extra), notes)
                break
            except Exception:
                continue
        notes.append('%s: bottom height %s, %s' % (label, bh_.expression, op.parameters.itemByName('bottomHeight_offset').expression))

    def under_lug(s, side):
        for strategy, t, prm, what in (('adaptive', flat10, [('stockToLeave', '0.3 mm')], 'roughed'),
                                       ('contour_new', flat5, [('maximumStepdown', '2 mm')], 'walls and root along the axis'),
                                       ('parallel', bull, [], 'the end wall and its root')):
            half_depth(operation(s, strategy, t, prm, notes=notes), 'under the lug from its %s end, %s' % (side, what))
    under_lug(c, 'first')
    d = new_setup('4 The arm under the lug, from its other end', flip=True, axis=bore[0])
    under_lug(d, 'other')
    return doc, cam, [a, b, c, d], 'fanuc', dict(holes=len(holes), bore=len(bore), root_edges=len(root_edges),
                                               volume=body.volume * 1000.0, material=body.material.name if body.material else None)


# ---------- the pinion spacer, turned ----------
def spacer(notes):
    tb, mat = take('axle', 'Spacer')
    # its axis along z: the axis of its largest cylindrical face, x any direction square to it
    cyl = max((f.geometry for f in tb.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType),
              key=lambda g: g.radius)
    z = cyl.axis.copy()
    z.normalize()
    x = V3(1, 0, 0) if abs(z.x) < 0.9 else V3(0, 1, 0)
    x = x.crossProduct(z).crossProduct(z)
    x.normalize()
    y = z.crossProduct(x)
    into_frame(tb, ((x.x, x.y, x.z), (y.x, y.y, y.z), (z.x, z.y, z.z)))
    doc, body, cam = new_part_document(tb, mat, 'A01-016 Spacer, pinion')
    sp = DD.spacer()
    tl = library('Turning Tools (Metric)')
    rough, finish, boring = tool(tl, 'CNMT Right Hand'), tool(tl, 'VNMT Right Hand'), tool(tl, 'ID Boring')
    parting = tool(tl, 'OD Grooving')
    si = cam.setups.createInput(adsk.cam.OperationTypes.TurningOperation)
    si.models = [body]
    s = cam.setups.add(si)
    s.name = 'Turned from 60 x 40 tube'
    for n, v in (('job_stockMode', "'fixedtube'"), ('job_stockDiameter', '60 mm'), ('job_stockDiameterInner', '40 mm'),
                 ('job_stockLength', '%.1f mm' % (sp['stock'] + 10.0))):
        setp(s.parameters, n, v, notes)
    leave = '%.2f mm' % ((sp['stock'] - sp['L']) / 2)             # each face long by half the grinding allowance
    fo = operation(s, 'turning_face', rough, [('frontHeight_offset', leave)], notes=notes)
    operation(s, 'turning_profile_roughing', rough, notes=notes)
    operation(s, 'turning_profile_finishing', finish, notes=notes)
    operation(s, 'turning_profile_roughing', boring, notes=notes)
    po = operation(s, 'turning_part', parting, [('backHeight_offset', '-' + leave)], notes=notes)
    return doc, cam, [s], 'fanuc turning', dict(volume=body.volume * 1000.0, length=(body.boundingBox.maxPoint.z - body.boundingBox.minPoint.z) * 10,
                                               material=body.material.name if body.material else None, stock=sp['stock'],
                                               leave=(sp['stock'] - sp['L']) / 2,
                                               front=fo.parameters.itemByName('frontHeight_offset').value.value * 10.0,
                                               back=po.parameters.itemByName('backHeight_offset').value.value * 10.0)


# ---------- the tine, cut from plate ----------
def inside_corner(body):
    """The L's inside corner: the straight edges along z whose ends lie inside the body's box in x and y (the five
    outer corners sit on the box)."""
    bb = body.boundingBox
    out = []
    for e in body.edges:
        if e.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            continue
        a, b = e.startVertex.geometry, e.endVertex.geometry
        if abs(a.x - b.x) > 1e-6 or abs(a.y - b.y) > 1e-6:
            continue
        if bb.minPoint.x + 1e-3 < a.x < bb.maxPoint.x - 1e-3 and bb.minPoint.y + 1e-3 < a.y < bb.maxPoint.y - 1e-3:
            out.append(e)
    return out


def top_face(body):
    return max((f for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType
                and abs(f.centroid.z - body.boundingBox.maxPoint.z) < 1e-6), key=lambda f: f.area)


def tine(notes):
    tn = MK.TINE
    tb, mat = take('bag', 'Tine 1 A')
    # its own frame: the L in x-y (x along the foot, world y; y up the leg, world z), the plate's thickness on z
    into_frame(tb, ((0, 1, 0), (0, 0, 1), (1, 0, 0)))
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        doc.attributes.add('mdfc', 'run', 'Making')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = 'P13-012 Tine'
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = comp.bRepBodies.item(0)
    lib = app.materialLibraries.itemByName('Fusion Material Library')
    m = lib.materials.itemByName(mat) if mat else None
    if m is not None:
        body.material = m
    sharp = dict(volume=body.volume * 1000.0, area=top_face(body).area * 100.0)
    corner = inside_corner(body)
    if tn['r'] > 0 and len(corner) == 1:
        fi = comp.features.filletFeatures.createInput()
        ec = adsk.core.ObjectCollection.create()
        ec.add(corner[0])
        fi.edgeSetInputs.addConstantRadiusEdgeSet(ec, adsk.core.ValueInput.createByString('%g mm' % tn['r']), True)
        comp.features.filletFeatures.add(fi)
    body = comp.bRepBodies.item(0)
    top = top_face(body)
    outline = sum(e.length for e in top.loops.item(0).edges) * 10.0 if top.loops.count == 1 else None
    rounded = [f for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType]
    # the sixteen: copies of the one component, arranged by Fusion on a plate
    occs = [occ] + [root.occurrences.addExistingComponent(comp, adsk.core.Matrix3D.create()) for _ in range(tn['n'] - 1)]
    ai = root.features.arrangeFeatures.createInput(adsk.fusion.ArrangeSolverTypes.Arrange2DTrueShapeSolverType)
    for o in occs:
        ai.arrangeComponents.add(o)
    env = ai.setPlaneEnvelope(root.xYConstructionPlane, adsk.core.ValueInput.createByString('%g mm' % tn['plate'][0]),
                              adsk.core.ValueInput.createByString('%g mm' % tn['plate'][1]))
    env.objectSpacing = adsk.core.ValueInput.createByString('%g mm' % tn['spacing'])
    env.frameWidth = adsk.core.ValueInput.createByString('%g mm' % tn['frame'])
    arr = root.features.arrangeFeatures.add(ai)
    stats = json.loads(arr.arrangeStatistics)['statistics']
    copies = [o for o in root.allOccurrences if o.assemblyContext is not None and o.bRepBodies.count]
    cut = [o.bRepBodies.item(0) for o in copies]
    lo = [min(b.boundingBox.minPoint.x for b in cut), min(b.boundingBox.minPoint.y for b in cut)]
    hi = [max(b.boundingBox.maxPoint.x for b in cut), max(b.boundingBox.maxPoint.y for b in cut)]
    turns = sorted({round(math.degrees(math.atan2(o.transform2.getCell(1, 0), o.transform2.getCell(0, 0)))) % 360 for o in copies})
    app.userInterface.workspaces.itemById('CAMEnvironment').activate()
    cam = adsk.cam.CAM.cast(doc.products.itemByProductType('CAMProductType'))
    si = cam.setups.createInput(adsk.cam.OperationTypes.JetOperation)
    si.models = cut
    s = cam.setups.add(si)
    s.name = 'The sixteen on one plate'
    for n, v in (('job_stockOffsetMode', "'simple'"), ('job_stockOffsetSides', '%g mm' % tn['frame']),
                 ('job_stockOffsetTop', '0 mm'), ('job_stockOffsetBottom', '0 mm')):
        setp(s.parameters, n, v, notes)
    laser = tool(library('Cutting Tools (Metric)'), kind='laser cutter')

    def sel_outlines(op):
        cp = op.parameters.itemByName('contours').value
        sels = cp.getCurveSelections()
        fc = sels.createNewFaceContourSelection()
        fc.inputGeometry = [top_face(b) for b in cut]
        cp.applyCurveSelections(sels)
    operation(s, 'jet2d', laser, select=sel_outlines, notes=notes)
    tp = laser.parameters
    return doc, cam, [s], 'jet', dict(
        volume=body.volume * 1000.0, area=top.area * 100.0, outline=outline, sharp=sharp, corner_edges=len(corner),
        rounded=[f.geometry.radius * 10.0 for f in rounded], material=body.material.name if body.material else None,
        thickness=(body.boundingBox.maxPoint.z - body.boundingBox.minPoint.z) * 10.0, arranged=len(copies),
        nest=[(hi[0] - lo[0]) * 10.0, (hi[1] - lo[1]) * 10.0], nest_lo=[lo[0] * 10.0, lo[1] * 10.0], turns=turns,
        efficiency=stats['Actual Efficiency']['value'], unarranged=stats['Components Unarranged']['value'],
        kerf=tp.itemByName('tool_kerfWidth').value.value * 10.0, feed=tp.itemByName('tool_feedCutting').value.value,   # mm/min (the NC's F)
        pierce=tp.itemByName('tool_pierceTime').value.value)


# ---------- the census: every made kind, measured ----------
SURF = {adsk.core.SurfaceTypes.PlaneSurfaceType: 'plane', adsk.core.SurfaceTypes.CylinderSurfaceType: 'cylinder',
        adsk.core.SurfaceTypes.ConeSurfaceType: 'cone', adsk.core.SurfaceTypes.SphereSurfaceType: 'sphere',
        adsk.core.SurfaceTypes.TorusSurfaceType: 'torus', adsk.core.SurfaceTypes.EllipticalCylinderSurfaceType: 'elliptical',
        adsk.core.SurfaceTypes.EllipticalConeSurfaceType: 'elliptical', adsk.core.SurfaceTypes.NurbsSurfaceType: 'nurbs'}


def _v(p, k=1.0):
    return [round(p.x * k, 5), round(p.y * k, 5), round(p.z * k, 5)]


def body_facts(body):
    """A body's faces (kind, area mm2; a plane's normal and a point on it; a surface of revolution's axis, a point on
    it and its radius), its volume and its smallest box."""
    fs = []
    for f in body.faces:
        g = f.geometry
        d = dict(t=SURF.get(g.surfaceType, 'other'), a=round(f.area * 100.0, 4))
        if d['t'] == 'plane':
            d['n'], d['o'] = _v(g.normal), _v(g.origin, 10.0)
        elif d['t'] in ('cylinder', 'cone', 'torus'):
            d['ax'], d['o'] = _v(g.axis), _v(g.origin, 10.0)
            d['r'] = round((g.radius if d['t'] == 'cylinder' else getattr(g, 'majorRadius', getattr(g, 'radius', 0.0))) * 10.0, 4)
        elif d['t'] == 'sphere':
            d['o'], d['r'] = _v(g.origin, 10.0), round(g.radius * 10.0, 4)
        fs.append(d)
    out = dict(volume=round(body.volume * 1000.0, 3), area=round(body.area * 100.0, 3), faces=fs)
    try:
        ob = body.orientedMinimumBoundingBox
        out['box'] = sorted([ob.length * 10.0, ob.width * 10.0, ob.height * 10.0])
        dirs = sorted([(ob.length, ob.lengthDirection), (ob.width, ob.widthDirection), (ob.height, ob.heightDirection)], key=lambda x: x[0])
        out['box_dirs'] = [_v(d) for _, d in dirs]
        out['box_centre'] = _v(ob.centerPoint, 10.0)
    except Exception as e:
        bb = body.boundingBox
        out['box'] = sorted([(bb.maxPoint.x - bb.minPoint.x) * 10, (bb.maxPoint.y - bb.minPoint.y) * 10, (bb.maxPoint.z - bb.minPoint.z) * 10])
        out['box_note'] = 'axis-aligned box: %s' % str(e)[:80]
    return out


def census():
    """Each machine built as Part 10 left it; each made line of its bill (BillOfMaterials.py's census) measured on the
    first piece Fusion holds of it; a weldment by its pieces. Written to results/making_census.json."""
    app = adsk.core.Application.get()
    rsm, bom = _load('RangeSweep'), _load('BillOfMaterials')
    kinds = []
    for key in bom.MACHINES:
        rsm.machine(key)
        doc = app.activeDocument
        try:
            doc.attributes.add('mdfc', 'run', 'Making')
        except Exception:
            pass
        des = adsk.fusion.Design.cast(app.activeProduct)
        rows, occ = bom.census(des)
        b = bom.BD.finish(bom.BD.bom(key, rows))
        inside = {}
        for g in b['lines']:
            if g['kind'].startswith('in ') and g['names']:
                inside.setdefault(g['kind'][3:], []).append(g)
        for g in b['lines']:
            if g['kind'] != 'made':
                continue
            pieces = [g] if g['names'] else inside.get(g['number'], [])
            bodies = []
            for p in pieces:
                name = sorted(n for v in p['names'].values() for n, _ in v)[0]
                o = occ[name][0]
                for i in range(o.bRepBodies.count):
                    bodies.append(dict(piece=p['description'], number=p['number'], **body_facts(o.bRepBodies.item(i))))
            kinds.append(dict(machine=key, number=g['number'], description=g['description'], qty=g['qty'],
                              material=bom.material(key, g),
                              weldment=not g['names'], bodies=bodies))
        doc.close(False)
    path = os.path.join(PROJECT, 'results', 'making_census.json')
    json.dump(dict(version=VERSION, kinds=kinds), io.open(path, 'w', encoding='utf-8'), indent=0)
    _last.clear()
    _last.update(census=len(kinds), path=path)
    return 'Census: %d made kinds measured, %d bodies, results/making_census.json.\n\n[%s]' % (
        len(kinds), sum(len(k['bodies']) for k in kinds), VERSION)


def build(which='bracket'):
    if which == 'census':
        return census()
    if which not in PARTS:
        raise ValueError('No part called %s: Chapter 66 makes %s.' % (which, ', '.join(PARTS)))
    notes = []
    doc, cam, setups, post_name, info = dict(bracket=bracket, spacer=spacer, tine=tine)[which](notes)
    rows = []
    os.makedirs(NC, exist_ok=True)
    cps = post(post_name)
    total = 0.0
    for k, s in enumerate(setups):
        ops = []
        for i in range(s.operations.count):
            op = s.operations.item(i)
            try:
                ok = generate(cam, op)
            except Exception as e:
                ok = False
                notes.append('%s: %s' % (op.name, str(e)[:80]))
            t = cam.getMachiningTime(op, FEED_SCALE, RAPID / 10.0, TOOL_CHANGE).machiningTime if op.hasToolpath else None
            tp = op.tool.parameters
            msg = ''
            if op.hasWarning:
                try:
                    msg = op.warning
                except Exception:
                    msg = '?'
            ops.append(dict(name=op.name, strategy=op.strategy, toolpath=bool(op.hasToolpath), warning=bool(op.hasWarning), message=msg,
                            tool=tp.itemByName('tool_description').value.value.strip(),
                            d=tp.itemByName('tool_diameter').value.value * 10, seconds=t))
        mt = cam.getMachiningTime(s, FEED_SCALE, RAPID / 10.0, TOOL_CHANGE)
        number = str(NUMBERS[which][k])
        nc = None
        try:
            pi = adsk.cam.PostProcessInput.create(number, cps, NC, adsk.cam.PostOutputUnitOptions.MillimetersOutput)
            pi.isOpenInEditor = False
            cam.postProcess(s, pi)
            for ext in ('.nc', '.NC', '.txt'):
                p = os.path.join(NC, number + ext)
                if os.path.exists(p):
                    nc = p
        except Exception as e:
            notes.append('post %s: %s' % (s.name, str(e)[:120]))
        lines = sum(1 for _ in open(nc, encoding='latin-1')) if nc else 0
        rows.append(dict(setup=s.name, ops=ops, seconds=mt.machiningTime, feed=mt.feedDistance * 10,
                         rapid=mt.rapidDistance * 10, tool_changes=mt.toolChangeCount, nc=os.path.basename(nc) if nc else None,
                         nc_lines=lines))
        total += mt.machiningTime
    out = dict(part=which, info=info, setups=rows, total=total, post=os.path.basename(cps), notes=notes, version=VERSION)
    json.dump(out, io.open(os.path.join(PROJECT, 'results', 'making_%s.json' % which), 'w', encoding='utf-8'), indent=1, default=str)
    _last.clear()
    _last.update(out)
    lines = ['%s: %s; %.0f s; %s (%d lines)' % (r['setup'], ', '.join('%s %s' % (o['name'], 'ok' if o['toolpath'] else 'NO TOOLPATH')
                                                                      for o in r['ops']), r['seconds'], r['nc'], r['nc_lines'])
             for r in rows]
    return ('Making %s: %d setups, %.1f min of cutting.\n%s%s\n\n[%s]' % (
        which, len(rows), total / 60.0, '\n'.join(lines), ('\nnotes: ' + '; '.join(notes)) if notes else '', VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.command.commandInputs.itemById('part').selectedItem.name))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('Making failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('part', 'Part', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k, p in enumerate(PARTS + ('census',)):
            dd.listItems.add(p, k == 0)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcMaking'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Making', 'Chapter 66: a made part programmed in Manufacture')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Making failed:\n%s' % traceback.format_exc())
