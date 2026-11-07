# =============================================================================
# Making.py  -  Chapter 66 of "Mechanical Design in Autodesk Fusion with Claude": making the parts. It takes a made
# part out of its machine as Part 10 left it and programs it in Fusion's Manufacture workspace: the setups, the
# operations with tools from Fusion's sample libraries, the toolpaths generated, each setup's machining time, and the
# NC program from Fusion's generic Fanuc posts.
#   bracket   the bag gripper's opening stop bracket P13-018 (Chapter 62: the arm turned, the root R2.5), milled from
#             a 1045 block in three setups: the bolting face and its two holes; the top, the part bolted to a
#             fixture plate through its own holes (face, adaptive roughing, the arm's root with a 10 mm cutter of
#             corner radius 2.5, the lug finished, the outline); the lug's M30 x 1.5, on an angle plate
#   spacer    the rear axle's pinion spacer A01-016 (Chapter 61: ground to each axle), turned from 60 x 40 tube,
#             0.8 mm long for the grinding
# Checked by checks/making_check.py.
#
#   INPUTS        the dialog: the part
#   DERIVED       lugs_brackets (the bracket's volume), drawings_detail (the spacer's grinding allowance), the machines
#                 through RangeSweep.py, the change through StopBracket.py
#   HARD RULES    every part copied from its machine's model; the tools from Fusion's sample libraries (one cutter
#                 copied and given the root's corner radius); nothing saved but the NC programs
#   CONVENTIONS   a new design document per part; mm, the API's cm inside
#   ACCEPTANCE    the message: the setups, their operations and toolpaths, the times, the NC files
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import glob
import importlib
import importlib.util
import math
import os
import sys
import time
import traceback

import adsk.cam
import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
COMP = os.path.dirname(SCRIPTS)
LIB = os.path.join(COMP, 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import drawings_detail as DD     # noqa: E402

VERSION = 'Making 1.0 (2026-11-07)'
PARTS = ('bracket', 'spacer')
NC = os.path.join(COMP, 'results', 'nc')
FEED_SCALE, RAPID, TOOL_CHANGE = 100.0, 5000.0, 10.0      # % of the programmed feed, mm/min rapid, s a tool change
_last = {}
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_mk66_' + name, os.path.join(SCRIPTS, name, name + '.py'))
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
                abs(p.itemByName('tool_diameter').value.value * 10 - d) < 1e-6:
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

    def sel_root(op):
        cp = op.parameters.itemByName('contours').value
        sels = cp.getCurveSelections()
        ch = sels.createNewChainSelection()
        ch.inputGeometry = root_edges
        cp.applyCurveSelections(sels)
    operation(b, 'contour_new', flat12, [('maximumStepdown', '4 mm')], notes=notes)   # the walls: foot, arm, the lug's ends
    operation(b, 'contour2d', bull, [('bottomHeight_offset', '-2.5 mm')], sel_root, notes)
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
    return doc, cam, [a, b, c], 'fanuc', dict(holes=len(holes), bore=len(bore), root_edges=len(root_edges),
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


def build(which='bracket'):
    if which not in PARTS:
        raise ValueError('No part called %s: Chapter 66 makes %s.' % (which, ', '.join(PARTS)))
    notes = []
    doc, cam, setups, post_name, info = dict(bracket=bracket, spacer=spacer)[which](notes)
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
        number = '%d%d' % (66, k + 1) if which == 'bracket' else '%d%d' % (66, 5 + k)
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
    out = dict(part=which, info=info, setups=rows, total=total, post=os.path.basename(cps), notes=notes)
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
        for k, p in enumerate(PARTS):
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
