# =============================================================================
# PartDrawings.py  -  Chapter 63 of "Mechanical Design in Autodesk Fusion with Claude": a drawing for one part of each
# machine, the part whose drawing Part 10 changed most. It builds each machine through RangeSweep.py, takes the part's
# body out into a design of its own, set in the part's own frame (z along its largest cylinder's axis, x towards its
# centre of mass), carries into the model what Part 10 decided for it (the opening stop bracket's root radius), saves
# the design to the cloud folder the drawing needs, and asks Fusion for an ISO drawing of it. On each sheet it then
# puts the datums, the geometric tolerances and the dimensions Part 10 decided, and a table of the requirements no
# symbol of the API carries, and exports the sheet as a PDF. The results go to results/part_drawings.json.
# Written from PROMPT.txt beside it; checked by checks/part_drawings_check.py.
#
#   INPUTS        the dialog: the part (all by default)
#   DERIVED       lib/drawings_detail.py: the parts, their numbers and every value a sheet carries
#   HARD RULES    nothing of the machines' scripts copied; built through RangeSweep.machine(); every design saved to
#                 the folder the dialog names, never over another file
#   CONVENTIONS   ISO drawings, first-angle projection, millimetres; the API's cm inside the model, mm on the sheet
#   ACCEPTANCE    the message; the check: each sheet's views, dimensions, symbols and table read back from Fusion
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import json
import math
import os
import sys
import time
import traceback

import adsk.core
import adsk.fusion
import adsk.drawing

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script

import drawings_detail as DD     # noqa: E402
importlib.reload(DD)

VERSION = 'PartDrawings 1.2 (2026-11-09)'
D = adsk.drawing
_last = {}


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_pd63_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _mark(doc):
    try:
        doc.attributes.add('mdfc', 'run', 'check')
    except Exception:
        pass


# ---------- the part's own frame ----------
def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(v):
    n = math.sqrt(_dot(v, v))
    return [x / n for x in v]


def frame(body):
    """z along the axis of the body's largest cylindrical face, x from that axis towards the centre of mass (or the
    world's x, y or z when the centre lies on the axis), the origin on the axis at the body's lowest point along z.
    Returns (origin, x, y, z) in cm, world coordinates."""
    cyl = None
    for f in body.faces:
        g = f.geometry
        if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType and (cyl is None or g.radius > cyl.radius):
            cyl = g
    if cyl is None:
        raise RuntimeError('%s has no cylindrical face to take its axis from' % body.name)
    z = _unit([cyl.axis.x, cyl.axis.y, cyl.axis.z])
    o0 = [cyl.origin.x, cyl.origin.y, cyl.origin.z]
    c = body.physicalProperties.centerOfMass
    v = _sub([c.x, c.y, c.z], o0)
    v = [v[i] - _dot(v, z) * z[i] for i in range(3)]
    if math.sqrt(_dot(v, v)) < 1e-4:
        for w in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            v = [w[i] - _dot(w, z) * z[i] for i in range(3)]
            if math.sqrt(_dot(v, v)) > 0.5:
                break
    x = _unit(v)
    y = _cross(z, x)
    zs = [_dot(_sub([p.geometry.x, p.geometry.y, p.geometry.z], o0), z) for p in body.vertices]
    zmin = min(zs) if zs else 0.0
    if not zs:
        bb = body.boundingBox
        zmin = min(_dot(_sub([p.x, p.y, p.z], o0), z) for p in (bb.minPoint, bb.maxPoint))
    origin = [o0[i] + zmin * z[i] for i in range(3)]
    return origin, x, y, z


def to_local(origin, x, y, z):
    """The matrix taking world coordinates into the part's frame."""
    m = adsk.core.Matrix3D.create()
    rows = (x, y, z)
    t = [-_dot(r, origin) for r in rows]
    m.setWithArray([x[0], x[1], x[2], t[0], y[0], y[1], y[2], t[1], z[0], z[1], z[2], t[2], 0, 0, 0, 1])
    return m


def _apply(m, p):
    pt = adsk.core.Point3D.create(p[0] / 10.0, p[1] / 10.0, p[2] / 10.0)
    pt.transformBy(m)
    return [pt.x * 10.0, pt.y * 10.0, pt.z * 10.0]


# ---------- the model's changes the drawing carries ----------
def _near(v, p, tol=1e-3):
    g = v.geometry
    return abs(g.x * 10 - p[0]) < tol and abs(g.y * 10 - p[1]) < tol and abs(g.z * 10 - p[2]) < tol


def root_fillet(root, body, m, spec):
    """The fillet Chapter 60 put on the drawing, on the edges round the root whose world corners the spec gives."""
    corners = [_apply(m, p) for p in spec['corners']]
    edges = adsk.core.ObjectCollection.create()
    for e in body.edges:
        if any(_near(e.startVertex, a) for a in corners) and any(_near(e.endVertex, b) for b in corners):
            edges.add(e)
    if edges.count != len(spec['corners']):
        raise RuntimeError('the root: %d edges found, %d expected' % (edges.count, len(spec['corners'])))
    fi = root.features.filletFeatures.createInput()
    fi.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(spec['r'] / 10.0), True)
    root.features.filletFeatures.add(fi)
    return dict(edges=edges.count, r=spec['r'])


CHANGES = {'root_fillet': root_fillet}


def turn_arm(tb, spec):
    """Chapter 62's change, made on the part's copy while it is still in the machine's world: the opening bracket's arm
    turned 45 deg about its own axis, by StopBracket's own function."""
    _load('StopBracket').turn_arm(tb, spec.get('side', 1))
    return dict(turn=DD.bracket()['turn'])


BEFORE = {'turn_arm': turn_arm}        # changes made on the copy in world coordinates, before it is moved into its frame


def before(tb, spec):
    return {ch['kind']: BEFORE[ch['kind']](tb, ch) for ch in spec.get('changes', ()) if ch['kind'] in BEFORE}


def apply_changes(root, body, m, spec):
    """Every change Part 10 decided for the part, carried into its body before it is drawn: on its own sheet and in
    an assembly alike."""
    done = {}
    for ch in spec.get('changes', ()):
        if ch['kind'] in BEFORE:
            continue
        done[ch['kind']] = CHANGES[ch['kind']](root, body, m, ch)
    return done


# ---------- the cloud ----------
def folder(path):
    app = adsk.core.Application.get()
    f = app.data.activeProject.rootFolder
    for name in path:
        nxt = f.dataFolders.itemByName(name)
        f = nxt if nxt is not None else f.dataFolders.add(name)
    return f


def _names(f):
    return [f.dataFiles.item(i).name for i in range(f.dataFiles.count)]


def wait_saved(doc, limit=180.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        adsk.doEvents()
        time.sleep(0.3)
        if doc.isSaved and doc.dataFile is not None:
            return time.time() - t0
    raise RuntimeError('%s not saved after %.0f s' % (doc.name, limit))


def create_drawing(data_file, limit=180.0, assembly=False):
    """Fusion's automatic ISO drawing of a saved design; a design saved a moment ago may not be ready, so asked again
    every 10 s. An assembly gets its main sheet with Fusion's parts list and no sheet for each component."""
    dm = D.DrawingManager.get()
    t0, err = time.time(), None
    while time.time() - t0 < limit:
        inp = dm.createDrawingInput(data_file, D.DrawingCreationModes.AutomaticDrawingCreationMode)
        inp.standard = D.DrawingStandardTypes.ISODrawingStandardType
        inp.units = D.DrawingUnitTypes.MillimeterDrawingUnitType
        inp.sheetSize = D.SheetSizes.A3ISOSheetSize
        if assembly:
            gp = inp.automationPreferences.globalPreferences
            gp.isComponentSheetGenerated = False
            gp.isMainAssemblySheetGenerated = True
            ma = inp.automationPreferences.mainAssemblyPreferences
            ma.isoViewSheetPreferences.isSheetCreated = True
            ma.isoViewSheetPreferences.isPartsListIncluded = True
        try:
            df = dm.createDrawing(inp)
            if df is not None:
                return df, time.time() - t0
        except Exception as e:
            err = e
        for _ in range(40):
            adsk.doEvents()
            time.sleep(0.25)
    raise RuntimeError('no drawing of %s after %.0f s: %s' % (data_file.name, limit, err))


def open_drawing(df):
    app = adsk.core.Application.get()
    doc = app.documents.open(df, True)
    for _ in range(40):
        adsk.doEvents()
        time.sleep(0.25)
    ui = app.userInterface
    cmd = ui.activeCommand
    if cmd and cmd != 'SelectCommand':
        ui.terminateActiveCommand()      # the automatic drawing leaves its auto-dimension dialog open
        adsk.doEvents()
    dd = D.DrawingDocument.cast(doc)
    sheet = dd.drawing.activeSheet       # the dialog's preview dimensions leave the sheet a moment after it closes
    last, same = -1, 0
    for _ in range(120):
        adsk.doEvents()
        time.sleep(0.25)
        n = sheet.drawingDimensions.count
        same = same + 1 if n == last else 0
        last = n
        if same >= 8:
            break
    _mark(doc)
    return dd, cmd


# ---------- the sheet ----------
def curves(view):
    out = []
    vc = view.viewCurves
    for i in range(vc.count):
        c = vc.item(i)
        s = c.startPoint.coordinate if c.startPoint else None
        e = c.endPoint.coordinate if c.endPoint else None
        k = c.centerPoint.coordinate if c.centerPoint else None
        r = math.hypot(s.x - k.x, s.y - k.y) if (s and k) else None
        out.append(dict(c=c, type=c.type, s=(s.x, s.y) if s else None, e=(e.x, e.y) if e else None,
                        k=(k.x, k.y) if k else None, r=r))
    return out


def P2(x, y):
    return D.DrawingPoint.create(adsk.core.Point2D.create(x, y))


def lines(view, min_len=0.5):
    """The view's straight curves with their length and direction (degrees, 0 to 180)."""
    out = []
    for c in curves(view):
        if c['type'] == D.ViewCurveTypes.LineViewCurveType and c['s'] and c['e']:
            dx, dy = c['e'][0] - c['s'][0], c['e'][1] - c['s'][1]
            n = math.hypot(dx, dy)
            if n >= min_len:
                c.update(len=n, ang=math.degrees(math.atan2(dy, dx)) % 180.0,
                         mid=((c['s'][0] + c['e'][0]) / 2, (c['s'][1] + c['e'][1]) / 2))
                out.append(c)
    return out


def rounds(view):
    """The view's arcs and circles with their radius on the sheet."""
    return [c for c in curves(view) if c['type'] in (D.ViewCurveTypes.ArcViewCurveType, D.ViewCurveTypes.CircleViewCurveType)
            and c['r'] is not None]


def box(view):
    cs = curves(view)
    xs = [p[0] for c in cs for p in (c['s'], c['e']) if p]
    ys = [p[1] for c in cs for p in (c['s'], c['e']) if p]
    return min(xs), min(ys), max(xs), max(ys)


def dia(sheet, c, at):
    return sheet.drawingDimensions.addDiameterDimension(c['c'], P2(*at))


def datum(sheet, c, label, at, on=None):
    """A datum identifier on curve c, its leader from the point on (the curve's middle by default) to its box at."""
    syms = sheet.drawingSymbols
    di = syms.createDatumIdentifierInput()
    di.viewCurve = c['c']
    p = on or c.get('mid') or c['s']
    di.leaderPoints = [P2(*p), P2(*at)]
    di.placementPoint = P2(*at)
    di.identifier = label
    return syms.addDatumIdentifier(di)


def frame_on(sheet, c, char, tol, datums, at, on=None, diameter=False, mmc=False):
    """A feature control frame on curve c: the characteristic, the tolerance (with a diameter sign and the maximum
    material modifier if asked) and up to two datums."""
    syms = sheet.drawingSymbols
    fi = syms.createFeatureControlFrameInput()
    fi.viewCurve = c['c']
    p = on or c.get('mid') or c['s']
    fi.leaderPoints = [P2(*p), P2(*at)]
    fi.placementPoint = P2(*at)
    fr = fi.firstFrame
    fr.geometricCharacteristic = char
    t = fr.primaryTolerance
    if diameter:
        t.append(D.SymbolPaletteTypes.DiameterSymbolPaletteType)
    t.appendText(tol)
    if mmc:
        t.append(D.SymbolPaletteTypes.MaximumMaterialRequirementSymbolPaletteType)
    fr.primaryTolerance = t
    for k, lab in enumerate(datums[:2]):
        attr = 'primaryDatum' if k == 0 else 'secondaryDatum'
        d_ = getattr(fr, attr)
        d_.appendText(lab)
        setattr(fr, attr, d_)
    return syms.addFeatureControlFrame(fi)


def requirements(sheet, rows, title='REQUIREMENTS'):
    """The table of what no symbol of the API carries: a title row, then (item, requirement, from) rows."""
    ct = sheet.customTables
    ti = ct.createInput()
    ti.rowCount = len(rows) + 1
    ti.columnCount = 3
    t = ct.add(ti)
    t.setCellData(0, 0, title)
    for r, row in enumerate(rows):
        for k, text in enumerate(row):
            t.setCellData(r + 1, k, text)
    return t


def readback(sheet):
    views = []
    for j in range(sheet.views.count):
        v = sheet.views.item(j)
        cs = curves(v)
        xs = [p[0] for c in cs for p in (c['s'], c['e']) if p]
        ys = [p[1] for c in cs for p in (c['s'], c['e']) if p]
        views.append(dict(type=v.type, n=len(cs), box=[min(xs), min(ys), max(xs), max(ys)] if xs else None,
                          kinds={str(t): sum(1 for c in cs if c['type'] == t) for t in sorted(set(c['type'] for c in cs))},
                          arcs=sorted(round(c['r'], 3) for c in cs if c['r'] is not None)))
    dims = sheet.drawingDimensions
    syms = sheet.drawingSymbols
    tables = []
    ct = sheet.customTables
    for i in range(ct.count):
        t = ct.item(i)
        tables.append([[t.getCellData(r, k) for k in range(t.columnCount)] for r in range(t.rowCount)])
    return dict(sheet=sheet.name, size=[sheet.width, sheet.height], views=views,
                dims=[dims.item(i).type for i in range(dims.count)], symbols=syms.count, tables=tables)


def export(dd, path):
    em = dd.drawing.exportManager
    o = em.createPDFExportOptions(path)
    o.openPDF = False
    return em.execute(o)


# ---------- one part ----------
def _fit(face):
    for f in (face, getattr(face, 'nativeObject', None)):
        if f is not None:
            a = f.attributes.itemByName('mdfc', 'fit')
            if a is not None:
                return a.value
    return None


def fits(body):
    """The classes and roughnesses Chapter 51's MaterialsFits.py left on the body's faces, read, not decided again."""
    out = []
    for i, f in enumerate(body.faces):
        v = _fit(f)
        if v is None:
            continue
        g = f.geometry
        cyl = g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType
        out.append(dict(face=i, value=v, kind='cylinder' if cyl else 'plane' if g.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType else 'other',
                        d=round(2 * g.radius * 10, 3) if cyl else None, area=round(f.area * 100, 2)))
    return out


def radii(body):
    """Each cylindrical face's radius in the model, mm, with how many faces have it."""
    out = {}
    for f in body.faces:
        g = f.geometry
        if g.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
            r = round(g.radius * 10, 3)
            out[r] = out.get(r, 0) + 1
    return out


def take_part(spec, RS):
    """Build the part's machine, copy the part's body into the part's frame and close the machine. A machine built
    without Chapter 51's materials and fits gets them first, through MaterialsFits.apply()."""
    app = adsk.core.Application.get()
    msg, _ = RS.machine(spec['machine'])
    mdoc = app.activeDocument
    _mark(mdoc)
    if spec.get('apply51'):
        _load('MaterialsFits').apply(spec['machine'])
    des = adsk.fusion.Design.cast(app.activeProduct)
    occs = [o for o in des.rootComponent.allOccurrences if o.component.name == spec['part'] and o.bRepBodies.count]
    if not occs:
        raise RuntimeError('no part called %s in the %s' % (spec['part'], spec['machine']))
    body = occs[0].bRepBodies.item(0)
    fr = frame(body)
    m = to_local(*fr)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    tb = tbm.copy(body)
    pre = before(tb, spec)
    tbm.transform(tb, m)
    mat = body.material.name if body.material else None
    big = max(f.geometry.radius for f in body.faces if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType)
    return dict(tb=tb, m=m, frame=fr, material=mat, machine_doc=mdoc, fits=fits(body), R=big * 10.0, pre=pre)


def make_part(spec, RS, where):
    app = adsk.core.Application.get()
    got = take_part(spec, RS)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    _mark(doc)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = des.rootComponent
    body = root.bRepBodies.add(got['tb'])
    body.name = spec['title']
    done = dict(got['pre'])
    done.update(apply_changes(root, body, got['m'], spec))
    body = root.bRepBodies.item(0)
    lib = app.materialLibraries.itemByName('Fusion Material Library')
    mat = lib.materials.itemByName(got['material']) if got['material'] else None
    if mat is not None:
        body.material = mat
    root.partNumber = spec['number']
    root.description = spec['title']
    bb = body.boundingBox
    box = [round(v * 10, 3) for v in (bb.minPoint.x, bb.minPoint.y, bb.minPoint.z, bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z)]
    name = '%s %s' % (spec['number'], spec['title'])
    if name in _names(where):
        raise RuntimeError('%s is already in the folder: a drawing is never saved over another file' % name)
    doc.saveAs(name, where, 'Volume 3, Chapter 63: %s' % spec['title'], '')
    t_save = wait_saved(doc)
    try:
        got['machine_doc'].close(False)
    except Exception:
        pass
    return dict(doc=doc, name=name, box=box, material=got['material'], changes=done, t_save=t_save,
                volume=body.volume * 1000.0, fits=got['fits'], R=got['R'], radii=radii(body))


def make_assembly(spec, RS, where):
    """An assembly of its own for the drawing: each kind of the spec's items one component with its part number and
    designation, each piece an occurrence of it where the machine has it, all in the frame of the spec's first part."""
    app = adsk.core.Application.get()
    msg, _ = RS.machine(spec['machine'])
    mdoc = app.activeDocument
    _mark(mdoc)
    des = adsk.fusion.Design.cast(app.activeProduct)
    leaves = [o for o in des.rootComponent.allOccurrences if o.component.bRepBodies.count]
    picked = DD.assembly_items(spec, [(o.component.name, _centre(o)) for o in leaves])
    first = [o for o in leaves if o.component.name == picked[0]['names'][0]][0]
    m = to_local(*frame(first.bRepBodies.item(0)))
    tbm = adsk.fusion.TemporaryBRepManager.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    _mark(doc)
    d2 = adsk.fusion.Design.cast(app.activeProduct)
    d2.designType = adsk.fusion.DesignTypes.DirectDesignType
    root = d2.rootComponent
    root.partNumber, root.description = spec['number'], spec['title']
    lib = app.materialLibraries.itemByName('Fusion Material Library')
    drawn = {p['number']: p for p in DD.PARTS}          # a part with a sheet of its own goes in as that sheet draws it
    made, done = [], {}
    for item in picked:
        occ0 = None
        for k, nm in enumerate(item['names']):
            src = [o for o in leaves if o.component.name == nm][0]
            body = src.bRepBodies.item(0)
            tb = tbm.copy(body)
            if item['number'] in drawn:
                done.update(before(tb, drawn[item['number']]))
            tbm.transform(tb, m)
            if occ0 is None:
                occ0 = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                comp = occ0.component
                comp.name = (item['number'] + ' ' + item['title']).strip()
                comp.partNumber, comp.description = item['number'], item['designation']
                b = comp.bRepBodies.add(tb)
                if item['number'] in drawn:
                    done.update(apply_changes(comp, b, m, drawn[item['number']]))
                    b = comp.bRepBodies.item(0)
                mat = lib.materials.itemByName(body.material.name) if body.material else None
                if mat is not None:
                    b.material = mat
                c0 = tb.boundingBox
                c0 = [(c0.minPoint.x + c0.maxPoint.x) / 2, (c0.minPoint.y + c0.maxPoint.y) / 2, (c0.minPoint.z + c0.maxPoint.z) / 2]
            else:                                       # the same kind again: an occurrence moved to the piece's place
                c1 = tb.boundingBox
                c1 = [(c1.minPoint.x + c1.maxPoint.x) / 2, (c1.minPoint.y + c1.maxPoint.y) / 2, (c1.minPoint.z + c1.maxPoint.z) / 2]
                t = adsk.core.Matrix3D.create()
                t.translation = adsk.core.Vector3D.create(*[c1[i] - c0[i] for i in range(3)])
                root.occurrences.addExistingComponent(comp, t)
        made.append(dict(number=item['number'], title=item['title'], n=len(item['names'])))
    name = '%s %s' % (spec['number'], spec['title'])
    if name in _names(where):
        raise RuntimeError('%s is already in the folder: a drawing is never saved over another file' % name)
    doc.saveAs(name, where, 'Volume 3, Chapter 63: %s' % spec['title'], '')
    t_save = wait_saved(doc)
    try:
        mdoc.close(False)
    except Exception:
        pass
    return dict(doc=doc, name=name, box=None, material=None, changes=done, t_save=t_save, volume=None, fits=[], R=None,
                radii={}, items=made)


def _centre(o):
    """The occurrence's centre in the machine, mm, from its body (an occurrence's own box comes back empty for the
    stops' pieces)."""
    b = o.bRepBodies.item(0).boundingBox if o.bRepBodies.count else o.boundingBox
    return [(b.minPoint.x + b.maxPoint.x) * 5, (b.minPoint.y + b.maxPoint.y) * 5, (b.minPoint.z + b.maxPoint.z) * 5]


def draw(spec, where, outdir, RS, annotate=True):
    t0 = time.time()
    part = make_assembly(spec, RS, where) if spec.get('assembly') else make_part(spec, RS, where)
    df, t_draw = create_drawing(part['doc'].dataFile, assembly=bool(spec.get('assembly')))
    dd, cmd = open_drawing(df)
    sheet = dd.drawing.activeSheet
    rounds_all = [c['r'] for j in range(sheet.views.count) for c in rounds(sheet.views.item(j))]
    scale = max(rounds_all) / part['R'] if (rounds_all and part['R']) else None
    added = {}
    n0, m0 = sheet.drawingDimensions.count, sheet.drawingSymbols.count
    ctx = dict(s=scale, fits=part['fits'], radii=part['radii'], material=part['material'], rows=[])
    if annotate and spec['key'] in ANNOTATE:
        added = ANNOTATE[spec['key']](sheet, spec, ctx)
    if spec['key'] in TIDY:
        tidied = sheet.tidyUp                 # Fusion's own rearranging (a property: reading it tidies), so the table
        #                                       leaves the views clear
    for _ in range(20):
        adsk.doEvents()
    read = dict(dims=sheet.drawingDimensions.count - n0, symbols=sheet.drawingSymbols.count - m0)
    pdf = os.path.join(outdir, '%s.pdf' % spec['number'])
    ok = export(dd, pdf)
    rb = readback(sheet)
    return dict(key=spec['key'], number=spec['number'], title=spec['title'], design=part['name'], box=part['box'],
                volume=part['volume'], material=part['material'], changes=part['changes'], drawing=df.name,
                fits=part['fits'], model=dict(radii={str(k): v for k, v in part['radii'].items()}), scale=scale,
                items=part.get('items'), sheets=dd.drawing.sheets.count,
                left_open=cmd, pdf=os.path.basename(pdf) if ok else None, added=added, read=read, rows=ctx['rows'], sheet=rb,
                seconds=time.time() - t0)


# ---------- each sheet's symbols, its curves found by their geometry ----------
GC = D.GeometricCharacteristicTypes
# What the API was seen to place without stopping Fusion's drawing command (Fusion 2706.1.17, tested 4 Nov 2026 one
# call at a time): diameter and linear dimensions, a position frame with the diameter sign and (M), a parallelism frame,
# a datum identifier on a line whose middle no other entity passes through, and the requirement table. The API picks a
# curve at its middle. A profile frame stops the command (its keyword refused). A datum on the spacer's end face stops
# it whatever the leader: with the axis's centre line through the face's middle Fusion asks 'Next/Accept'; on a sheet
# made without centre lines it waits for a leader point the API does not send, or attaches early and refuses the next
# point. So the spacer's datum and its parallelism stay rows of its table, and so do the segment's profiles.
ENABLED = {'datum', 'table', 'dims', 'frames'}


def _views(sheet):
    return [sheet.views.item(j) for j in range(sheet.views.count)]


def _table(sheet, spec, ctx):
    import materials_fits as M
    mat = M.material(spec['machine'], spec['kind'], 'Aluminum' if 'Alum' in (ctx.get('material') or '') else 'Steel')
    rows = DD.rows(spec['key'], ctx['fits'], mat)
    requirements(sheet, rows)
    ctx['rows'] = rows
    return rows


def _close(a, b, tol):
    return abs(a - b) < tol


def sheet_bracket(sheet, spec, ctx):
    """The opening bracket: the holes' size and their position from datum A, the foot's bolting face."""
    s, b = ctx['s'], DD.bracket()
    face = side = None
    for v in _views(sheet):
        holes = [c for c in rounds(v) if _close(c['r'], b['H'] / 2 * s, 0.05)]
        if len(holes) == 2:
            face, hs = v, sorted(holes, key=lambda c: c['k'][1])
        elif any(_close(c['len'], 70.0 * s, 0.1) for c in lines(v)):
            side = v
    lo, hi = hs
    x0, y0, x1, y1 = box(face)
    if 'dims' in ENABLED:
        dia(sheet, hi, (hi['k'][0] + 18 * s, y1 + 14))
        # the pitch between the centres (an aligned dimension takes points of the view's curves, not of the sheet),
        # its line 17 mm off the centres' line on the side away from the lug: past the lug's edge, inside the foot
        dx, dy = hi['k'][0] - lo['k'][0], hi['k'][1] - lo['k'][1]
        n = math.hypot(dx, dy)
        mx, my = (lo['k'][0] + hi['k'][0]) / 2, (lo['k'][1] + hi['k'][1]) / 2
        sheet.drawingDimensions.addLinearDimension(lo['c'].centerPoint, hi['c'].centerPoint,
                                                   P2(mx + 17 * s * dy / n, my - 17 * s * dx / n), True)
    if 'frames' in ENABLED:
        frame_on(sheet, lo, GC.PositionGeometricCharacteristicType, '%.1f' % b['t'], ['A'], (x0 - 4, y0 - 24),
                 on=lo['s'], diameter=True, mmc=True)
    sx0, sy0, sx1, sy1 = box(side)
    cands = [c for c in lines(side) if _close(c['len'], 70.0 * s, 0.1)]
    rc = rounds(side)
    cx = sum(c['k'][0] for c in rc) / len(rc) if rc else (sx0 + sx1) / 2
    bolting = max(cands, key=lambda c: abs(c['mid'][0] - cx))          # the face farthest from the lug
    datum(sheet, bolting, 'A', (bolting['mid'][0] + 14, sy0 - 12))
    rows = _table(sheet, spec, ctx)
    return dict(dims=2 if 'dims' in ENABLED else 0, datums=1, frames=1 if 'frames' in ENABLED else 0, rows=len(rows))


def sheet_segment(sheet, spec, ctx):
    """The segment: datum A its top face, datum B its 15 deg face; the profiles of 0.02 of the 15 deg face from A and
    the 5 deg face from B go into the table (see below)."""
    s, g = ctx['s'], DD.segment()
    for v in _views(sheet):
        ls = lines(v)
        f15 = [c for c in ls if 12.0 < abs(c['ang'] - 90.0) < 18.0]
        if f15:
            break
    f15 = max(f15, key=lambda c: c['len'])
    # the 5 deg face's silhouette in the same view, 3 to 8 deg off the view's long axis: its ends find the top face
    five = []
    for c in curves(v):
        if c['s'] and c['e'] and c['type'] != D.ViewCurveTypes.ArcViewCurveType:
            dx, dy = c['e'][0] - c['s'][0], c['e'][1] - c['s'][1]
            if math.hypot(dx, dy) > 10 * s and 3.0 < math.degrees(math.atan2(abs(dy), abs(dx))) < 8.0:
                five.append(c)
    sil = max(five, key=lambda c: math.hypot(c['e'][0] - c['s'][0], c['e'][1] - c['s'][1]))
    ends = [sil['s'], sil['e']]
    tops = [c for c in lines(v) if abs(c['ang'] - 90.0) < 1.0 and any(_close(c['s'][0], p[0], 0.05) and
            min(abs(c['s'][1] - p[1]), abs(c['e'][1] - p[1])) < 0.05 for p in ends)]
    top = min(tops, key=lambda c: c['len'])                              # the short one: the top face, not the foot
    x0, y0, x1, y1 = box(v)
    datum(sheet, top, 'A', (x1 + 16, top['mid'][1]))
    datum(sheet, f15, 'B', (x1 + 16, f15['mid'][1] + 14))
    # Fusion 2706.1.17 cannot take a profile frame from the API: its drawing command rejects the characteristic's
    # keyword and waits for input that never comes (FusionDrawing.log, -AMFCFRAME; AcCore log 'Invalid option
    # keyword'). The two profiles go into the table, against the datums the sheet carries.
    rows = _table(sheet, spec, ctx)
    return dict(dims=0, datums=2, frames=0, rows=len(rows))


def sheet_spacer(sheet, spec, ctx):
    """The spacer: datum A the face on cone A, the other face parallel to it within IT5; the bore's size."""
    s, sp = ctx['s'], DD.spacer()
    for v in _views(sheet):
        rs = rounds(v)
        bore = [c for c in rs if c['type'] == D.ViewCurveTypes.CircleViewCurveType and _close(c['r'], sp['d_in'] / 2 * s, 0.05)]
        if bore:
            if 'dims' in ENABLED:
                dia(sheet, bore[0], (bore[0]['k'][0] + 0.75 * bore[0]['r'], bore[0]['k'][1] - 0.2 * bore[0]['r']))
        elif 'end faces' in ENABLED:
            ends = [c for c in curves(v) if c['s'] and c['e'] and _close(c['s'][0], c['e'][0], 1e-3)
                    and abs(c['s'][1] - c['e'][1]) > sp['d_out'] * s * 0.9]
            if len(ends) == 2:
                left, right = sorted(ends, key=lambda c: c['s'][0])
                x0, y0, x1, y1 = box(v)
                for c in (left, right):
                    c['mid'] = (c['s'][0], (c['s'][1] + c['e'][1]) / 2)
                datum(sheet, left, 'A', (x0 - 16, y1 + 12), on=(left['s'][0], y1 - 6 * s))
                frame_on(sheet, right, GC.ParallelismGeometricCharacteristicType, '%.3f' % sp['parallel'], ['A'],
                         (x1 + 10, y1 + 14), on=(right['s'][0], y1 - 6 * s))
    rows = _table(sheet, spec, ctx)
    on = 'end faces' in ENABLED
    return dict(dims=1 if 'dims' in ENABLED else 0, datums=1 if on else 0, frames=1 if on else 0, rows=len(rows))


def sheet_fitting(sheet, spec, ctx):
    """The spar fitting: the bores' size, where they show round, and the bolt holes'."""
    s, f = ctx['s'], DD.fitting()
    n = 0
    for v in _views(sheet):
        for c in rounds(v):
            if c['type'] == D.ViewCurveTypes.CircleViewCurveType and _close(c['r'], f['D'] / 2 * s, 0.05):
                if 'dims' in ENABLED:
                    dia(sheet, c, (c['k'][0] + 1.6 * c['r'], c['k'][1] + 1.6 * c['r']))
                    n += 1
                break
        if n:
            break
    holes = []
    for v in _views(sheet):
        holes = [c for c in rounds(v) if c['type'] == D.ViewCurveTypes.CircleViewCurveType and c['r'] < 6 * s]
        if len(holes) >= 4:
            h = max(holes, key=lambda c: (c['k'][1], -c['k'][0]))
            if 'dims' in ENABLED:
                dia(sheet, h, (h['k'][0] + 14, h['k'][1] + 18))
                n += 1
            break
    rows = _table(sheet, spec, ctx)
    return dict(dims=n, datums=0, frames=0, rows=len(rows))


def sheet_disc(sheet, spec, ctx):
    """The stop disc: the bore's size, where its fit is read; nothing else beyond the general tolerance."""
    s = ctx['s']
    bores = [f for f in ctx['fits'] if f['kind'] == 'cylinder' and f['value'][:1].isupper()]
    n = 0
    if bores:
        d = bores[0]['d']
        for v in _views(sheet):
            cs = [c for c in rounds(v) if _close(c['r'], d / 2 * s, 0.05)]
            if cs:
                c = cs[0]
                if 'dims' in ENABLED:
                    dia(sheet, c, (c['k'][0] - 1.2 * c['r'], c['k'][1] - 2.2 * c['r']))
                    n = 1
                break
    rows = _table(sheet, spec, ctx)
    return dict(dims=n, datums=0, frames=0, rows=len(rows))


def sheet_stop(sheet, spec, ctx):
    """The opening stop put together: Fusion's parts list on the sheet, and the table of what the fitter must do."""
    rows = DD.stop_rows()
    requirements(sheet, rows, title='ASSEMBLY')
    ctx['rows'] = rows
    return dict(dims=0, datums=0, frames=0, rows=len(rows))


# the sheets on which Fusion puts the requirement table over a view (the API has no place for a table): tidied
TIDY = ('segment', 'spacer', 'disc')
ANNOTATE = dict(stop=sheet_stop, bracket=sheet_bracket, segment=sheet_segment, spacer=sheet_spacer, fitting=sheet_fitting, disc=sheet_disc)
_D = 'dims' in ENABLED
WANT = dict(bracket=dict(dims=2 if _D else 0, datums=1, frames=1 if 'frames' in ENABLED else 0), segment=dict(datums=2),
            spacer=dict(dims=1 if _D else 0, datums=1 if 'end faces' in ENABLED else 0, frames=1 if 'end faces' in ENABLED else 0),
            fitting=dict(dims=2 if _D else 0), disc=dict(dims=1 if _D else 0), stop=dict())


def build(which='all', where=None, annotate=True):
    where = tuple(where or DD.FOLDER)
    keys = [p['key'] for p in DD.PARTS]
    if which != 'all' and which not in keys:
        raise ValueError('No part called %s: Chapter 63 draws %s.' % (which, ', '.join(keys)))
    RS = _load('RangeSweep')
    f = folder(where)
    outdir = os.path.join(PROJECT, 'results', 'drawings')
    os.makedirs(outdir, exist_ok=True)
    out = []
    for spec in DD.PARTS:
        if which in ('all', spec['key']):
            out.append(draw(spec, f, outdir, RS, annotate))
    json.dump(out, open(os.path.join(PROJECT, 'results', 'part_drawings.json'), 'w'), indent=1, default=str)
    _last.clear()
    _last.update(parts=out, folder='/'.join(where), names=_names(f))
    lines = ['%s %s: %d views, %d dimensions, %d symbols, %s' % (o['number'], o['title'], len(o['sheet']['views']),
             len(o['sheet']['dims']), o['sheet']['symbols'], o['pdf']) for o in out]
    return 'Part drawings: %d made in %s.\n%s\n\n[%s]' % (len(out), '/'.join(where), '\n'.join(lines), VERSION)


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.command.commandInputs.itemById('key').selectedItem.name))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('PartDrawings failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('key', 'Part', adsk.core.DropDownStyles.TextListDropDownStyle)
        dd.listItems.add('all', True)
        for p in DD.PARTS:
            dd.listItems.add(p['key'], False)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcPartDrawings'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Part drawings', 'Chapter 63: a drawing for one part of each machine')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PartDrawings failed:\n%s' % traceback.format_exc())
