# =============================================================================
# Making.py  -  Chapter 66 of "Mechanical Design in Autodesk Fusion with Claude": making the parts. For one made part
# it takes the part out of its machine in its final detailed form, puts it in a design of its own and programs it in
# Fusion's Manufacture workspace: the setups and their stock, the operations with tools from Fusion's sample
# libraries, every toolpath generated, each setup's machining time and its NC program from Fusion's generic posts.
# Asked for the census, it builds each of the nine machines and measures every made kind of its bill (the faces, the
# volume, the smallest box) for lib/making.py's sort by process.
# Written from design_office/Making/PROMPT.txt alone (a prompt test).
#
#   INPUTS        the dialog: the part (bracket by default; spacer, tine or census)
#   DERIVED       RangeSweep (the machines), StopBracket (the arm turned), BillOfMaterials (the bills, the pieces);
#                 lugs_brackets, drawings_detail, making (the hand values); the sample tool libraries; the posts
#   HARD RULES    nothing of the machines' scripts copied; nothing saved but the census, each part's results and
#                 its NC programs
#   CONVENTIONS   a new parametric design for the part, the body in a base feature; lengths mm, the API's cm inside
#   ACCEPTANCE    the message: each setup's operations, its time, its NC file and its lines; the version tag
# =============================================================================
import glob
import importlib
import importlib.util
import io
import json
import math
import os
import re
import sys
import time
import traceback

import adsk.core
import adsk.fusion
import adsk.cam

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script

import mdfc_parts as mp          # noqa: E402
import making as MK              # noqa: E402
importlib.reload(MK)
import lugs_brackets as LB       # noqa: E402
importlib.reload(LB)
import drawings_detail as DD     # noqa: E402
importlib.reload(DD)

VERSION = 'Making 1.0 prompt test (2026-11-12)'
PARTS = ('bracket', 'spacer', 'tine')
CHOICES = PARTS + ('census',)
RESULTS = os.path.join(CP.ROOT, 'design_office', 'results')
NC_DIR = os.path.join(RESULTS, 'nc')
CENSUS = os.path.join(RESULTS, 'making_census.json')
RAPID = 5000.0                   # mm/min, the rapid moves for the machining time
TOOL_CHANGE = 10.0               # s for each tool change
PROGRAMS = dict(bracket=(6601, 6602, 6603), spacer=(6611,), tine=(6621,))
SAMPLE_LIBS = ('Milling Tools (Metric)', 'Hole Making Tools (Metric)', 'Turning Tools (Metric)', 'Cutting Tools (Metric)')
_last = {}
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VS = adsk.core.ValueInput.createByString
VI = adsk.core.ValueInput.createByReal


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_mk66_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class _Log(list):
    def __call__(self, text):
        self.append(text)


# ---------- small geometry helpers (mm) ----------
def _mm(p):
    return (p.x * 10.0, p.y * 10.0, p.z * 10.0)


def _unit(v):
    n = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) or 1.0
    return (v.x / n, v.y / n, v.z / n)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    n = math.sqrt(_dot(a, a)) or 1.0
    return (a[0] / n, a[1] / n, a[2] / n)


def _perp(e):
    a = (1.0, 0.0, 0.0) if abs(e[0]) < 0.9 else (0.0, 1.0, 0.0)
    return _norm(_cross(e, a))


ST = adsk.core.SurfaceTypes


def _is(f, kind):
    return f.geometry.surfaceType == kind


def _frame(origin_mm, x, y, z):
    """The matrix that takes a body from the world into the frame (origin, x, y, z): the frame's inverse."""
    m = adsk.core.Matrix3D.create()
    o = origin_mm
    m.setWithCoordinateSystem(P3(o[0] / 10.0, o[1] / 10.0, o[2] / 10.0), V3(*x), V3(*y), V3(*z))
    m.invert()
    return m


# ---------- the machine and its bill ----------
def _bill(bom, key, des):
    """The machine's bill as BillOfMaterials makes it: its lines, each made line's material, and the occurrences by
    the component's name."""
    rows, occ = bom.census(des)
    b = bom.BD.finish(bom.BD.bom(key, rows))
    for g in b['lines']:
        try:
            g['material'] = bom.material(key, g)
        except Exception:
            g['material'] = ', '.join(g.get('mats') or [])
    return b, occ


def _names(g):
    nm = g.get('names') or {}
    if isinstance(nm, dict):
        return sorted(n for v in nm.values() for n, _p in v)
    return sorted(nm)


def _pieces(g, lines):
    """The pieces a made line is measured on: its first component; a weldment's first component of each of its
    pieces' lines (the lines 'in <its number>'), in the order of its parts."""
    names = _names(g)
    if names:
        return False, [(None, names[0])]
    subs = [l for l in lines if l['kind'] == 'in %s' % g['number']]
    order = list(g.get('parts') or [])
    subs.sort(key=lambda l: order.index(l['number']) if l['number'] in order else len(order))
    return True, [(l['number'] or l['description'], _names(l)[0]) for l in subs if _names(l)]


def _machine(rs, bom, key, number):
    """Build the machine through RangeSweep, find the made line `number` in its bill and the first component that is
    one of its pieces."""
    app = adsk.core.Application.get()
    msg, _setter = rs.machine(key)
    doc = app.activeDocument
    try:
        doc.attributes.add('mdfc', 'run', 'Making')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    b, occ = _bill(bom, key, des)
    line = [g for g in b['lines'] if g['number'] == number and g['kind'] == 'made']
    if not line:
        raise RuntimeError('%s: no made line %s in its bill' % (key, number))
    _w, pieces = _pieces(line[0], b['lines'])
    o = occ[pieces[0][1]][0]
    return doc, des, line[0], o


def _world_copy(occ):
    """The occurrence's first body as a temporary body in world coordinates, and its material's name."""
    body = occ.component.bRepBodies.item(0)
    tb = mp.tbm().copy(body)
    mp.tbm().transform(tb, mp.world_matrix(occ))
    return tb, (body.material.name if body.material else None)


def _new_design():
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        doc.attributes.add('mdfc', 'run', 'Making')
    except Exception:
        pass
    return doc, des


def _material(body, name):
    if not name:
        return
    libs = adsk.core.Application.get().materialLibraries
    for i in range(libs.count):
        lib = libs.item(i)
        try:
            for j in range(lib.materials.count):
                m = lib.materials.item(j)
                if m.name == name:
                    body.material = m
                    return
        except Exception:
            continue


def _base_body(comp, tb, name, material):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    _material(body, material)
    return body


# ---------- the Manufacture workspace ----------
def _cam(doc):
    ui = adsk.core.Application.get().userInterface
    ui.workspaces.itemById('CAMEnvironment').activate()
    return adsk.cam.CAM.cast(doc.products.itemByProductType('CAMProductType'))


def _pnames(params):
    try:
        return [params.item(i).name for i in range(params.count)]
    except Exception:
        return [p.name for p in params]


def _param(params, name):
    try:
        return params.itemByName(name)
    except Exception:
        return None


def _set(params, names, expr, log, what=''):
    """Set the first of these parameters that exists, by its expression; note it when none does."""
    for n in ((names,) if isinstance(names, str) else names):
        p = _param(params, n)
        if p is not None:
            try:
                p.expression = expr
                return n
            except Exception as e:
                log('%s: %s = %s refused (%s)' % (what, n, expr, e))
    log('%s: no parameter %s' % (what, '/'.join((names,) if isinstance(names, str) else names)))
    return None


def _like(params, groups, without=()):
    """The first parameter whose name holds every word of one of the groups (any case), none of `without`."""
    names = _pnames(params)
    for g in groups:
        for n in names:
            nl = n.lower()
            if all(w in nl for w in g) and not any(w in nl for w in without):
                return n
    return None


def _set_like(params, groups, without, expr, log, what):
    """Set the first parameter, among those whose names hold every word of a group, that takes the expression."""
    tried = []
    for g in groups:
        for n in _pnames(params):
            nl = n.lower()
            if n in tried or not all(w in nl for w in g) or any(w in nl for w in without):
                continue
            tried.append(n)
            try:
                _param(params, n).expression = expr
                return n
            except Exception:
                continue
    log('%s: no parameter for %s took %s (tried %s)' % (what, groups[0], expr, ', '.join(tried) or 'none'))
    return None


def _value_mm(params, name):
    p = _param(params, name)
    try:
        return p.value.value * 10.0
    except Exception:
        return None


def _stock_to_leave(params, mm, log, what, axial=True):
    done = []
    for n in _pnames(params):
        nl = n.lower()
        if 'stocktoleave' not in nl:
            continue
        p = _param(params, n)
        try:
            if nl.startswith('use'):
                p.expression = 'true'
                done.append(n)
            elif axial or not ('axial' in nl or 'vertical' in nl):
                p.expression = '%g mm' % mm
                done.append(n)
        except Exception as e:
            log('%s: %s refused (%s)' % (what, n, e))
    if not done:
        log('%s: no stock-to-leave parameter' % what)
    return done


def _choose(params, words, prefer, log, what):
    """A choice parameter whose name holds one of `words`, set to its first choice that holds one of `prefer`."""
    for n in _pnames(params):
        if not any(w in n.lower() for w in words):
            continue
        p = _param(params, n)
        cv = adsk.cam.ChoiceParameterValue.cast(p.value) if p is not None else None
        if cv is None:
            continue
        try:
            r = cv.getChoices()
            names, values = (r[1], r[2]) if len(r) == 3 else (r[0], r[1])
        except Exception:
            continue
        for nm, val in zip(names, values):
            if any(w in (str(nm) + ' ' + str(val)).lower() for w in prefer):
                try:
                    p.expression = "'%s'" % val
                    return n, val
                except Exception as e:
                    log('%s: %s = %s refused (%s)' % (what, n, val, e))
    log('%s: no choice among %s for %s' % (what, words, prefer))
    return None


# ---------- tools from Fusion's sample libraries ----------
_TOOLS = {}


def _library(name):
    tl = adsk.cam.CAMManager.get().libraryManager.toolLibraries
    for u in ('systemlibraryroot://Samples/%s.json' % name, 'systemlibraryroot://%s.json' % name):
        try:
            lib = tl.toolLibraryAtURL(adsk.core.URL.create(u))
            if lib is not None and lib.count:
                return lib
        except Exception:
            pass
    from urllib.parse import unquote
    stack = [tl.urlByLocation(adsk.cam.LibraryLocations.Fusion360LibraryLocation)]
    while stack:
        f = stack.pop()
        for a in tl.childAssetURLs(f):
            if unquote(a.toString()).rstrip('/').endswith('/' + name + '.json') or unquote(a.toString()).endswith(name):
                return tl.toolLibraryAtURL(a)
        stack.extend(tl.childFolderURLs(f))
    raise RuntimeError("no sample tool library '%s' in Fusion's libraries" % name)


def _lib_tools(name):
    if name not in _TOOLS:
        lib = _library(name)
        _TOOLS[name] = [lib.item(i) for i in range(lib.count)]
    return _TOOLS[name]


def _tv(t, name):
    p = t.parameters.itemByName(name)
    try:
        return None if p is None else p.value.value
    except Exception:
        return None


def _ttype(t):
    return str(_tv(t, 'tool_type') or '')


def _dia(t):
    return (_tv(t, 'tool_diameter') or 0.0) * 10.0


def _reach(t):
    return max((_tv(t, 'tool_bodyLength') or 0.0), (_tv(t, 'tool_fluteLength') or 0.0)) * 10.0


def _pick(libs, types, d=None, lo=0.0, hi=1e9, what='a tool'):
    """The sample tool of one of these types (its exact name) between lo and hi mm across, the nearest to d, the
    longest reach first."""
    found = [(t, lib) for lib in libs for t in _lib_tools(lib) if _ttype(t) in types and lo <= _dia(t) <= hi]
    if not found:
        raise RuntimeError('no %s (%s, %g-%g mm) in %s' % (what, '/'.join(types), lo, hi, ', '.join(libs)))
    found.sort(key=lambda x: (abs(_dia(x[0]) - d) if d is not None else 0.0, -_reach(x[0])))
    return found[0]


def _copy(t):
    return adsk.cam.Tool.createFromJson(t.toJson())


class _Numbers(object):
    """Each setup's tools numbered 1, 2, ... in order of use, the same tool keeping its number."""

    def __init__(self):
        self.seen = {}

    def __call__(self, base, tag=None):
        k = tag or id(base)
        if k not in self.seen:
            c = _copy(base)
            p = c.parameters.itemByName('tool_number')
            if p is not None:
                p.value.value = len(self.seen) + 1
            self.seen[k] = c
        return self.seen[k]


def _tool_text(t):
    try:
        cr = (_tv(t, 'tool_cornerRadius') or 0.0) * 10.0
        n = _tv(t, 'tool_number')
        return 'T%s %s D%g%s' % (n, _ttype(t), round(_dia(t), 3), (' CR%g' % round(cr, 3)) if cr else '')
    except Exception:
        return '?'


# ---------- setups, operations, toolpaths, times, posts ----------
def _setup(cam, kind, models, name, stock, extra, log):
    si = cam.setups.createInput(kind)
    si.models = models
    mode, vals = stock
    si.stockMode = mode
    for k, v in vals:
        _set(si.parameters, k, v, log, name)
    for k, v in extra:
        if isinstance(v, list):
            p = _param(si.parameters, k)
            if p is None:
                log('%s: no parameter %s' % (name, k))
            else:
                p.value.value = v
        else:
            _set(si.parameters, k, v, log, name)
    s = cam.setups.add(si)
    s.name = name
    return s


def _op(setup, strategy, tool, name, params, log):
    inp = setup.operations.createInput(strategy)
    inp.tool = tool
    inp.displayName = name
    for k, v in params:
        _set(inp.parameters, k, v, log, name)
    return setup.operations.add(inp)


def _geometry(op, names, entities, log):
    for n in names:
        p = _param(op.parameters, n)
        if p is not None:
            p.value.value = list(entities)
            return n
    log('%s: no geometry parameter %s' % (op.name, '/'.join(names)))
    return None


def _contours(op, entities, log, faces=False):
    """The 2D contours: a chain of edges, or the outlines of planar faces."""
    p = _param(op.parameters, 'contours')
    if p is None:
        log('%s: no contours parameter' % op.name)
        return
    v = p.value
    sels = v.getCurveSelections()
    s = sels.createNewFaceContourSelection() if faces else sels.createNewChainSelection()
    s.inputGeometry = list(entities)
    if not faces and hasattr(s, 'sideType'):
        try:
            s.sideType = adsk.cam.SideTypes.AlwaysOutsideSideType
        except Exception as e:
            log('%s: side refused (%s)' % (op.name, e))
    v.applyCurveSelections(sels)


def _op_row(op):
    msgs = []
    for a in ('warning', 'error', 'warningMessage', 'errorMessage'):
        try:
            v = getattr(op, a, None)
        except Exception:
            v = None
        if isinstance(v, str) and v.strip():
            msgs.append(v.strip())
    tool = None
    try:
        tool = op.tool
    except Exception:
        pass
    return dict(name=op.name, strategy=op.strategy, tool=_tool_text(tool) if tool else None,
                toolpath=bool(op.hasToolpath), warning=bool(op.hasWarning), error=bool(op.hasError),
                message=' | '.join(msgs))


def _generate(cam, op, log):
    try:
        fut = cam.generateToolpath(op)
        t0 = time.time()
        while not fut.isGenerationCompleted and time.time() - t0 < 3600.0:
            adsk.doEvents()
            time.sleep(0.05)
    except Exception as e:
        log('%s: generation failed (%s)' % (op.name, e))


def _machining_time(cam, setup, log):
    """Fusion's machining time at the programmed feeds (100 %), rapids at RAPID mm/min, TOOL_CHANGE s a change.
    The API takes the rapid feed in its own units (cm/s); the time is read again if Fusion's rapid time shows
    another unit."""
    rapid = RAPID / 600.0
    scale = 100.0
    mt = cam.getMachiningTime(setup, scale, rapid, TOOL_CHANGE)
    rd, rt = getattr(mt, 'rapidDistance', 0.0), getattr(mt, 'totalRapidTime', 0.0)
    if rd > 1e-3 and rt > 1e-9 and abs(rd / rt / rapid - 1.0) > 0.02:
        log('%s: rapid read as %.4g cm/s, asked %.4g: read again' % (setup.name, rd / rt, rapid))
        rapid = rapid * rapid / (rd / rt)
        mt = cam.getMachiningTime(setup, scale, rapid, TOOL_CHANGE)
    fd, ft = getattr(mt, 'feedDistance', 0.0), getattr(mt, 'totalFeedTime', 0.0)
    if ft > 1e-9 and fd * 10.0 / (ft / 60.0) > 50000.0:
        log('%s: feeds averaged %.0f mm/min at a scale of 100: the scale taken as a factor' % (setup.name, fd * 600.0 / ft))
        scale = 1.0
        mt = cam.getMachiningTime(setup, scale, rapid, TOOL_CHANGE)
    return dict(seconds=mt.machiningTime, feed=getattr(mt, 'feedDistance', 0.0) * 10.0,
                rapid=getattr(mt, 'rapidDistance', 0.0) * 10.0, feed_time=getattr(mt, 'totalFeedTime', 0.0),
                rapid_time=getattr(mt, 'totalRapidTime', 0.0), tool_changes=getattr(mt, 'toolChangeCount', 0))


def _post_path(cam, name):
    la = os.environ.get('LOCALAPPDATA', '')
    hits = glob.glob(os.path.join(la, 'Autodesk', 'webdeploy', 'production', '*', 'Applications', 'CAM360', 'Data',
                                  'Posts', name))
    if hits:
        return max(hits, key=lambda p: (os.path.getmtime(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.dirname(p)))))), os.path.getmtime(p)))
    try:
        p = os.path.join(cam.genericPostFolder, name)
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    raise RuntimeError('no post %s in Fusion\'s install' % name)


def _post(cam, setup, post, number):
    os.makedirs(NC_DIR, exist_ok=True)
    before = {f: os.path.getmtime(os.path.join(NC_DIR, f)) for f in os.listdir(NC_DIR)}
    pin = adsk.cam.PostProcessInput.create(str(number), _post_path(cam, post), NC_DIR,
                                           adsk.cam.PostOutputUnitOptions.MillimetersOutput)
    pin.isOpenInEditor = False
    cam.postProcess(setup, pin)
    new = [f for f in os.listdir(NC_DIR) if f.split('.')[0] == str(number)
           and (f not in before or os.path.getmtime(os.path.join(NC_DIR, f)) > before[f])]
    if not new:
        return None, 0
    f = max(new, key=lambda x: os.path.getsize(os.path.join(NC_DIR, x)))
    with io.open(os.path.join(NC_DIR, f), encoding='latin-1') as fh:
        n = sum(1 for _ in fh)
    return f, n


def _program(cam, setup, ops, post, number, log):
    for op in ops:
        _generate(cam, op, log)
    rows = [_op_row(op) for op in ops]
    t = _machining_time(cam, setup, log)
    nc, lines = None, 0
    try:
        nc, lines = _post(cam, setup, post, number)
    except Exception as e:
        log('%s: post failed (%s)' % (setup.name, e))
    out = dict(setup=setup.name, ops=rows, nc=nc, nc_lines=lines, post=post)
    out.update(t)
    return out


def _mill_stock(sides, top, bottom):
    return (adsk.cam.SetupStockModes.RelativeBoxStock,
            [('job_stockOffsetMode', "'simple'"), ('job_stockOffsetSides', '%g mm' % sides),
             ('job_stockOffsetTop', '%g mm' % top), ('job_stockOffsetBottom', '%g mm' % bottom)])


# ---------- the bracket ----------
def bracket_part(log):
    """The opening stop bracket P13-018 out of the bag gripper with Chapter 62's change, in its own frame: the bolting
    face on z = 0, the arm up z, the foot's edges along x and y, its centre on the arm's axis."""
    rs, sb, bom = _load('RangeSweep'), _load('StopBracket'), _load('BillOfMaterials')
    d = DD.bracket()
    arm = LB.arm(True)
    mdoc, des, line, occ = _machine(rs, bom, 'bag', 'P13-018')
    sb.change(occ, 1, d['r_root'])                               # the arm turned 45 deg, its root rounded
    tb, mat = _world_copy(occ)
    x0 = arm['x'][0] - d['foot']                                 # the bolting face, on the end plate
    cy, cz = arm['centre']
    m = adsk.core.Matrix3D.create()
    m.setWithArray([0, 1, 0, -cy / 10.0, 0, 0, 1, -cz / 10.0, 1, 0, 0, -x0 / 10.0, 0, 0, 0, 1])
    mp.tbm().transform(tb, m)
    mdoc.close(False)
    doc, des = _new_design()
    body = _base_body(des.rootComponent, tb, 'Stop bracket, opening P13-018', mat)
    return doc, des, body, line, mat


def _axes(faces):
    """The distinct axes of these cylindrical faces (a hole split in two faces is one hole)."""
    out = []
    for f in faces:
        g = f.geometry
        o, ax = _mm(g.origin), _unit(g.axis)
        new = True
        for p, a in out:
            d = [o[k] - p[k] for k in range(3)]
            off = math.sqrt(max(0.0, _dot(d, d) - _dot(d, a) ** 2))
            if abs(abs(_dot(ax, a)) - 1.0) < 1e-6 and off < 1e-3:
                new = False
        if new:
            out.append((o, ax))
    return out


def build_bracket(log):
    d = DD.bracket()
    arm = LB.arm(True)
    D, P = [float(v) for v in d['thread'].replace('M', '').split(' x ')]
    minor = D - 1.0825 * P                                       # ISO 965's basic minor diameter of the M30 x 1.5
    r_root = d['r_root']
    doc, des, body, line, mat = bracket_part(log)
    cyl = [f for f in body.faces if _is(f, ST.CylinderSurfaceType)]
    holes = [f for f in cyl if abs(f.geometry.radius * 10 - d['H'] / 2) < 1e-3]
    bores = [f for f in cyl if abs(f.geometry.radius * 10 - D / 2) < 1e-3]
    z_root = d['foot'] + r_root

    def root_edge(e):
        if e.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            return False
        if abs(e.startVertex.geometry.z * 10 - z_root) > 1e-3 or abs(e.endVertex.geometry.z * 10 - z_root) > 1e-3:
            return False
        return any(_is(f, ST.CylinderSurfaceType) and abs(f.geometry.radius * 10 - r_root) < 1e-4 for f in e.faces)
    roots = [e for e in body.edges if root_edge(e)]
    info = dict(volume=body.volume * 1000.0, hand_volume=LB.volume(True, r_root), holes=len(_axes(holes)),
                bore=len(_axes(bores)), root_edges=len(roots), material=body.material.name if body.material else None,
                minor=minor, turn=LB.TURN, r_root=r_root)
    cam = _cam(doc)
    mill, hole = SAMPLE_LIBS[0], SAMPLE_LIBS[1]
    face_mill = _pick((mill,), ('face mill',), 50.0, 30.0, 100.0, 'face mill')[0]
    drill = _pick((hole, mill), ('drill',), d['H'], 0.0, 100.0, 'drill')[0]
    if abs(_dia(drill) - d['H']) > 1e-6:                         # no 9 mm drill in the sample: one copied to 9
        log('the %g mm drill copied from a %.2f mm one' % (d['H'], _dia(drill)))
        drill = _copy(drill)
        drill.parameters.itemByName('tool_diameter').value.value = d['H'] / 10.0
    rough = _pick((mill,), ('flat end mill',), 12.0, 8.0, 16.0, 'roughing end mill')[0]
    finish = _pick((mill,), ('flat end mill',), 10.0, 6.0, 12.0, 'finishing end mill')[0]
    base10 = None
    for types in (('bull nose end mill',), ('flat end mill',)):
        try:
            base10 = _pick((mill,), types, 10.0, 10.0 - 1e-6, 10.0 + 1e-6, '10 mm cutter')[0]
            break
        except RuntimeError:
            continue
    if base10 is None:
        base10 = _pick((mill,), ('end mill',), 10.0, 0.0, 100.0, '10 mm cutter')[0]
    root_tool = _copy(base10)                                    # the sample has no 10 mm bull nose with R2.5
    root_tool.parameters.itemByName('tool_diameter').value.value = 1.0
    if 'bull nose' not in _ttype(root_tool):
        try:
            root_tool.parameters.itemByName('tool_type').value.value = 'bull nose end mill'
        except Exception as e:
            log('the root cutter keeps its type %s (%s)' % (_ttype(root_tool), e))
    root_tool.parameters.itemByName('tool_cornerRadius').value.value = r_root / 10.0
    borer = _pick((mill,), ('flat end mill',), 16.0, 8.0, minor - 4.0, 'boring end mill')[0]
    thread = _pick((mill, hole), ('thread mill',), 16.0, 4.0, minor - 4.0, 'thread mill')[0]

    stock = _mill_stock(2.5, 3.0, 3.0)
    setups = []
    # (1) from under the bolting face: the face faced, the two holes drilled
    n1 = _Numbers()
    s1 = _setup(cam, adsk.cam.OperationTypes.MillingOperation, [body], '1 Bolting face: faced, 2 x 9 drilled', stock,
                [('wcs_orientation_flipZ', 'true')], log)
    o11 = _op(s1, 'face', n1(face_mill), 'Face the bolting face', [], log)
    o12 = _op(s1, 'drill', n1(drill), 'Drill 2 x %g through' % d['H'],
              [('holeMode', "'selection-faces'"), ('drillTipThroughBottom', 'true'), ('breakThroughDepth', '1 mm')], log)
    _geometry(o12, ('holeFaces', 'faces'), holes, log)
    setups.append((s1, [o11, o12]))
    # (2) bolted to a fixture plate through its own holes: the top, the roughing, the walls, the arm's root
    n2 = _Numbers()
    s2 = _setup(cam, adsk.cam.OperationTypes.MillingOperation, [body],
                '2 Top: bolted to a fixture plate through its own holes', stock, [], log)
    o21 = _op(s2, 'face', n2(face_mill), 'Face the top', [], log)
    o22 = _op(s2, 'adaptive', n2(rough), 'Adaptive roughing, 0.3 mm left', [], log)
    _stock_to_leave(o22.parameters, 0.3, log, o22.name)
    o23 = _op(s2, 'contour_new', n2(finish), 'Walls finished, 3D contour', [], log)
    o24 = _op(s2, 'contour2d', n2(root_tool, 'root'), "Arm's root, 2D contour, R%g" % r_root,
              [('bottomHeight_mode', "'from surface bottom'"), ('bottomHeight_offset', '%g mm' % d['foot']),
               ('doMultipleDepths', 'false')], log)
    _contours(o24, roots, log)
    setups.append((s2, [o21, o22, o23, o24]))
    # (3) down the lug's axis: the bore to the minor diameter, the thread, the lug's round over its length
    n3 = _Numbers()
    s3 = _setup(cam, adsk.cam.OperationTypes.MillingOperation, [body], "3 Down the lug's axis: bore, thread, round",
                stock, [('wcs_orientation_mode', "'axesZX'"), ('wcs_orientation_axisZ', bores[:1])], log)
    o31 = _op(s3, 'bore', n3(borer), 'Bore to %.3f (the minor diameter)' % minor, [], log)
    _geometry(o31, ('circularFaceSelections', 'holeFaces', 'faces'), bores, log)
    _stock_to_leave(o31.parameters, (D - minor) / 2.0, log, o31.name, axial=False)
    o32 = _op(s3, 'thread', n3(thread), 'Thread %s, pitch %g' % (d['thread'], P),
              [('threadPitch', '%g mm' % P)], log)
    _geometry(o32, ('circularFaceSelections', 'holeFaces', 'faces'), bores, log)
    # the lug's round: a 3D contour between the lug's two ends, the heights taken from the model's top along the
    # setup's tool axis
    n_ax = (arm['e1'][0], arm['e1'][1], 0.0)                      # the lug's axis in the part's frame
    c = (0.0, 0.0, d['axis_out'])
    zax = n_ax
    try:
        w = s3.workCoordinateSystem
        zax = (w.getCell(0, 2), w.getCell(1, 2), w.getCell(2, 2))
    except Exception as e:
        log('setup 3: its tool axis not read (%s), taken along the lug' % e)
    top = max(_dot(_mm(v.geometry), zax) for v in body.vertices)
    mid = _dot(c, zax)
    half = d['lug_len'] / 2.0 * abs(_dot(zax, n_ax))
    o33 = _op(s3, 'contour_new', n3(finish), "Lug's round finished over its %g mm" % d['lug_len'],
              [('topHeight_mode', "'from surface top'"), ('topHeight_offset', '%.4f mm' % (mid + half - top)),
               ('bottomHeight_mode', "'from surface top'"), ('bottomHeight_offset', '%.4f mm' % (mid - half - top))],
              log)
    setups.append((s3, [o31, o32, o33]))
    out = [_program(cam, s, ops, 'fanuc.cps', PROGRAMS['bracket'][i], log) for i, (s, ops) in enumerate(setups)]
    return info, out


# ---------- the spacer ----------
def build_spacer(log):
    rs, bom = _load('RangeSweep'), _load('BillOfMaterials')
    sp = DD.spacer()
    leave = (sp['stock'] - sp['L']) / 2.0                        # half the grinding allowance at each end
    mdoc, des, line, occ = _machine(rs, bom, 'axle', 'A01-016')
    tb, mat = _world_copy(occ)
    cyl = [f.geometry for f in tb.faces if _is(f, ST.CylinderSurfaceType)]
    g = max(cyl, key=lambda c: c.radius)
    ax, o = _unit(g.axis), _mm(g.origin)
    ends = [_dot([q - p for q, p in zip(_mm(f.geometry.origin), o)], ax) for f in tb.faces
            if _is(f, ST.PlaneSurfaceType) and abs(abs(_dot(_unit(f.geometry.normal), ax)) - 1) < 1e-6]
    ends += [_dot([q - p for q, p in zip(_mm(v.geometry), o)], ax) for v in tb.vertices]
    o0 = [o[i] + ax[i] * min(ends) for i in range(3)]
    x = _perp(ax)
    mp.tbm().transform(tb, _frame(o0, x, _cross(ax, x), ax))     # its axis along z, its back face on z = 0
    mdoc.close(False)
    doc, des = _new_design()
    body = _base_body(des.rootComponent, tb, 'Spacer, pinion A01-016', mat)
    zs = [f.geometry.origin.z * 10 for f in body.faces if _is(f, ST.PlaneSurfaceType)
          and abs(abs(_unit(f.geometry.normal)[2]) - 1) < 1e-6]                 # its two end faces
    radii = sorted(f.geometry.radius * 10 for f in body.faces if _is(f, ST.CylinderSurfaceType))
    info = dict(length=max(zs) - min(zs), d_out=2 * radii[-1], d_in=2 * radii[0], leave=leave, stock=sp['stock'],
                range=sp['range'], grind=sp['grind'], L=sp['L'], material=body.material.name if body.material else None)
    cam = _cam(doc)
    turn = SAMPLE_LIBS[2]
    od = [t for t in _lib_tools(turn) if 'turning' in _ttype(t) and 'general' in _ttype(t)]
    if not od:
        raise RuntimeError("no 'turning general' tool in %s" % turn)
    bars = [t for t in _lib_tools(turn) if 'boring' in _ttype(t)]
    parts = [t for t in _lib_tools(turn) if 'groov' in _ttype(t) or 'part' in _ttype(t)]
    if not bars or not parts:
        raise RuntimeError('no boring bar or parting tool in %s' % turn)
    front = 2.0                                                  # the tube's end ahead of the part, faced off
    length = sp['L'] + front + 40.0                              # the part, its facing, the part-off and the chuck
    tube = getattr(adsk.cam.SetupStockModes, 'FixedTubeStock', None)
    vals = []
    if tube is None:
        log('no fixed tube stock in this Fusion: a relative tube')
        tube = adsk.cam.SetupStockModes.RelativeTubeStock
    n = _Numbers()
    si = cam.setups.createInput(adsk.cam.OperationTypes.TurningOperation)
    si.models = [body]
    si.stockMode = tube
    ps = si.parameters
    for groups, without, v in (([('stock', 'outer', 'diameter'), ('stock', 'tube', 'outer'), ('stock', 'fixed', 'diameter'),
                                  ('stock', 'diameter')], ('inner', 'round', 'offset', 'mode'), '60 mm'),
                               ([('stock', 'inner', 'diameter'), ('stock', 'inner')], ('round', 'offset', 'mode'), '40 mm'),
                               ([('stock', 'fixed', 'length'), ('stock', 'length')], ('round', 'offset', 'mode'),
                                '%g mm' % length),
                               ([('stock', 'front', 'offset'), ('stock', 'front')], ('round', 'mode'), '%g mm' % front)):
        k = _set_like(ps, groups, without, v, log, 'spacer stock')
        if k:
            vals.append((k, v))
    s = cam.setups.add(si)
    s.name = '1 Turned from 60 x 40 tube'
    info['stock_set'] = vals
    info['stock_params'] = []
    for k in _pnames(s.parameters):
        if k.lower().startswith('job_stock'):
            try:
                info['stock_params'].append((k, _param(s.parameters, k).expression))
            except Exception:
                info['stock_params'].append((k, None))
    of = _op(s, 'turning_face', n(od[0]), 'Face, %.2f mm long' % leave, [], log)
    k = _set_like(of.parameters, [('front', 'offset')], ('clearance', 'retract', 'radial', 'mode'),
                  '%g mm' % leave, log, of.name)
    if k:
        info['front'] = _value_mm(of.parameters, k)
        info['front_param'] = k
    else:
        k = _like(of.parameters, [('axial', 'stocktoleave')], ('use',))
        _stock_to_leave(of.parameters, leave, log, of.name)
        info['front'] = _value_mm(of.parameters, k) if k else None
        info['front_param'] = k
    orr = _op(s, 'turning_profile_roughing', n(od[0]), 'Rough the outside', [], log)
    ofi = _op(s, 'turning_profile_finishing', n(od[-1], 'od finish'), 'Finish the outside', [], log)
    obr = _op(s, 'turning_profile_roughing', n(bars[0]), 'Rough the bore', [], log)
    info['bore_mode'] = _choose(obr.parameters, ('mode',), ('inner', 'inside'), log, obr.name)
    opo = _op(s, 'turning_part', n(parts[0]), 'Part off, %.2f mm long' % leave, [], log)
    k = _set_like(opo.parameters, [('back', 'offset'), ('part', 'offset')],
                  ('clearance', 'retract', 'radial', 'chamfer', 'feed', 'height', 'mode'), '%g mm' % -leave, log,
                  opo.name)
    if k:
        info['back'] = _value_mm(opo.parameters, k)
        info['back_param'] = k
    else:
        log('%s: no offset for the part-off' % opo.name)
        info['back'] = None
    info['op_params'] = {o.name: [x for x in _pnames(o.parameters) if any(w in x.lower() for w in ('offset', 'front', 'back', 'mode'))]
                         for o in (of, opo, obr)}
    out = [_program(cam, s, [of, orr, ofi, obr, opo], 'fanuc turning.cps', PROGRAMS['spacer'][0], log)]
    return info, out


# ---------- the tine ----------
def _vbox(body):
    ps = [_mm(v.geometry) for v in body.vertices]
    return [min(p[i] for p in ps) for i in range(3)], [max(p[i] for p in ps) for i in range(3)]


def _inside_corner(body):
    """The one straight edge along z whose ends lie inside the body's box in x and y: the L's inside corner."""
    lo, hi = _vbox(body)
    out = []
    for e in body.edges:
        if e.geometry.curveType != adsk.core.Curve3DTypes.Line3DCurveType:
            continue
        a, b = _mm(e.startVertex.geometry), _mm(e.endVertex.geometry)
        if abs(a[0] - b[0]) > 1e-4 or abs(a[1] - b[1]) > 1e-4:
            continue
        if lo[0] + 1e-3 < a[0] < hi[0] - 1e-3 and lo[1] + 1e-3 < a[1] < hi[1] - 1e-3:
            out.append(e)
    return out


def _enum(prefix, word_sets):
    for cname in dir(adsk.fusion):
        if not cname.startswith(prefix):
            continue
        E = getattr(adsk.fusion, cname)
        for m in dir(E):
            if m.startswith('_'):
                continue
            ml = m.lower()
            if any(all(w in ml for w in ws) for ws in word_sets):
                v = getattr(E, m)
                if isinstance(v, int):
                    return cname, m, v
    return None


def _try(calls, log, what):
    errs = []
    for label, f in calls:
        try:
            r = f()
            return label, r
        except Exception as e:
            errs.append('%s: %s' % (label, str(e).splitlines()[0] if str(e) else type(e).__name__))
    log('%s failed: %s' % (what, '; '.join(errs)))
    return None, None


def _get(ob, a):
    try:
        return getattr(ob, a)
    except Exception:
        return None


def _set_length(obj, attr, mm, log):
    cur = _get(obj, attr)
    for label, f in (('expression', lambda: setattr(cur, 'expression', '%g mm' % mm)),
                     ('value input', lambda: setattr(obj, attr, VS('%g mm' % mm))),
                     ('cm', lambda: setattr(obj, attr, mm / 10.0))):
        if label == 'expression' and not hasattr(cur, 'expression'):
            continue
        try:
            f()
            return label
        except Exception:
            continue
    log('arrange: %s not set' % attr)
    return None


def _arrange(des, occs, log):
    """Fusion's Arrange, 2D true shape: the tines on the plate, the spacing between them and the frame at its edge.
    The API's names are found on the objects themselves; every step is noted."""
    root = des.rootComponent
    W, H = MK.TINE['plate']
    sk = root.sketches.add(root.xYConstructionPlane)
    sk.name = 'Plate %g x %g' % (W, H)
    sk.sketchCurves.sketchLines.addTwoPointRectangle(P3(0, 0, 0), P3(W / 10.0, H / 10.0, 0))
    prof = sk.profiles.item(0)
    af = root.features.arrangeFeatures
    note = dict(features=[a for a in dir(af) if not a.startswith('_')])
    solver = _enum('Arrange', (('trueshape',), ('true', 'shape')))
    note['solver'] = solver[:2] if solver else None
    coll = adsk.core.ObjectCollection.create()
    for o in occs:
        coll.add(o)
    calls = []
    if solver:
        calls += [('createInput(solver)', lambda: af.createInput(solver[2])),
                  ('createInput(objects, solver)', lambda: af.createInput(coll, solver[2]))]
    calls += [('createInput()', lambda: af.createInput()), ('createInput(objects)', lambda: af.createInput(coll))]
    how, inp = _try(calls, log, 'arrange input')
    if inp is None:
        raise RuntimeError('no arrange input: ' + log[-1])
    note['input'] = how
    subs = [inp] + [_get(inp, a) for a in dir(inp) if not a.startswith('_') and
                    any(w in a.lower() for w in ('definition', 'envelope', 'setting', 'option'))
                    and _get(inp, a) is not None and not callable(_get(inp, a))]
    note['input_attrs'] = [a for a in dir(inp) if not a.startswith('_')]
    # the solver, where the input carries it
    for ob in subs:
        for a in dir(ob):
            if solver and 'solver' in a.lower() and not callable(_get(ob, a)):
                try:
                    setattr(ob, a, solver[2])
                    note['solver_set'] = a
                except Exception:
                    pass
    # the objects
    placed = None
    for ob in subs:
        for a in dir(ob):
            al = a.lower()
            if a.startswith('_') or not any(w in al for w in ('compon', 'object', 'occurrence')):
                continue
            v = _get(ob, a)
            if v is not None and hasattr(v, 'add') and not callable(v):
                how, r = _try([('%s.add(occ)' % a, lambda v=v: [v.add(o) for o in occs]),
                               ('%s.add(occ, 1)' % a, lambda v=v: [v.add(o, 1) for o in occs])], log, 'arrange objects')
                if how:
                    placed = how
                    break
            elif callable(v) and al.startswith(('add', 'set')):
                how, r = _try([('%s(collection)' % a, lambda v=v: v(coll)), ('%s(list)' % a, lambda v=v: v(list(occs))),
                               ('%s(occ)' % a, lambda v=v: [v(o) for o in occs])], log, 'arrange objects')
                if how:
                    placed = how
                    break
        if placed:
            break
    note['objects'] = placed
    # the envelope: the plate's profile, or the XY plane with its size
    env = None
    for ob in subs:
        for a in dir(ob):
            al = a.lower()
            v = _get(ob, a)
            if 'envelope' in al and callable(v) and al.startswith(('set', 'create', 'add')):
                how, r = _try([('%s(profile)' % a, lambda v=v: v(prof)),
                               ('%s(plane, 2000 mm, 1000 mm)' % a, lambda v=v: v(root.xYConstructionPlane, VS('%g mm' % W), VS('%g mm' % H))),
                               ('%s(plane, cm)' % a, lambda v=v: v(root.xYConstructionPlane, W / 10.0, H / 10.0))], log,
                              'arrange envelope')
                if how:
                    env = how
                    break
        if env:
            break
    if env is None:                                              # or an envelope property that takes the profile
        for ob in subs:
            for a in dir(ob):
                if 'envelope' in a.lower() and not a.startswith('_') and not callable(_get(ob, a)):
                    how, r = _try([('%s = profile' % a, lambda ob=ob, a=a: setattr(ob, a, prof))], log, 'arrange envelope')
                    if how:
                        env = how
                        break
            if env:
                break
    note['envelope'] = env
    # the spacing between the parts and the frame at the plate's edge
    for ob in subs:
        for a in dir(ob):
            al = a.lower()
            if a.startswith('_') or callable(_get(ob, a)):
                continue
            if 'spacing' in al and 'envelope' not in al:
                note['spacing'] = (a, _set_length(ob, a, MK.TINE['spacing'], log))
            elif 'frame' in al:
                note['frame'] = (a, _set_length(ob, a, MK.TINE['frame'], log))
    feat = af.add(inp)
    adsk.doEvents()
    note['feature_attrs'] = [a for a in dir(feat) if not a.startswith('_')]
    got = {}
    for a in note['feature_attrs']:
        al = a.lower()
        if any(w in al for w in ('arranged', 'efficien', 'utiliz')):
            v = _get(feat, a)
            if v is not None and not callable(v):
                got[a] = v if isinstance(v, (int, float)) else (v.count if hasattr(v, 'count') else str(v))
    note['result'] = got
    return feat, note


def _layout_by_hand(occs, w, h, log):
    """Only when Fusion's Arrange cannot be driven: the tines put in rows by the script, so that the rest of the
    program can still be made. Reported as not arranged by Fusion."""
    s, fr = MK.TINE['spacing'], MK.TINE['frame']
    per = int((MK.TINE['plate'][0] - 2 * fr + s) // (w + s))
    for k, o in enumerate(occs):
        try:
            o.isGrounded = False
        except Exception:
            pass
        m = adsk.core.Matrix3D.create()
        m.translation = V3((fr + (k % per) * (w + s)) / 10.0, (fr + (k // per) * (h + s)) / 10.0, 0)
        o.transform2 = m
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    if des.snapshots.hasPendingSnapshot:
        des.snapshots.add()


def _off_plate(k):
    """Where the k-th tine waits before it is nested: in a row below the plate, clear of it."""
    m = adsk.core.Matrix3D.create()
    m.translation = V3(k * 30.0, -150.0, 0.0)
    return m


def build_tine(log):
    rs, bom = _load('RangeSweep'), _load('BillOfMaterials')
    h0, h = MK.tine(0.0), MK.tine()
    mdoc, des, line, occ = _machine(rs, bom, 'bag', 'P13-012')
    tb, mat = _world_copy(occ)
    planes = sorted([f for f in tb.faces if _is(f, ST.PlaneSurfaceType)], key=lambda f: -f.area)
    zt = _unit(planes[0].geometry.normal)                       # the L's flat faces: their normal is its thickness
    side = [f for f in planes if abs(_dot(_unit(f.geometry.normal), zt)) < 1e-6]
    xt = _unit(side[0].geometry.normal)
    yt = _cross(zt, xt)
    mp.tbm().transform(tb, _frame((0.0, 0.0, 0.0), xt, yt, zt))
    lo, _hi = _vbox(tb)
    m = adsk.core.Matrix3D.create()
    m.translation = V3(-lo[0] / 10.0, -lo[1] / 10.0, -lo[2] / 10.0)
    mp.tbm().transform(tb, m)                                    # its box from the origin
    mdoc.close(False)
    doc, des = _new_design()
    root = des.rootComponent
    o1 = root.occurrences.addNewComponent(_off_plate(0))
    comp = o1.component
    comp.name = 'Tine P13-012'
    body = _base_body(comp, tb, 'Tine P13-012', mat)
    if not body.material or body.material.name != 'Steel AISI 1020 107 HR':
        _material(body, 'Steel AISI 1020 107 HR')              # 1020, Fusion's name for it (making.py's RHO_1020)
    sharp = dict(volume=body.volume * 1000.0, faces=body.faces.count)
    corner = _inside_corner(body)
    if len(corner) == 1:
        coll = adsk.core.ObjectCollection.create()
        coll.add(corner[0])
        fi = comp.features.filletFeatures.createInput()
        fi.addConstantRadiusEdgeSet(coll, VI(h['r'] / 10.0), True)
        comp.features.filletFeatures.add(fi)
    body = comp.bRepBodies.item(0)
    lo, hi = _vbox(body)
    top = [f for f in body.faces if _is(f, ST.PlaneSurfaceType) and abs(abs(_unit(f.geometry.normal)[2]) - 1) < 1e-6
           and abs(f.geometry.origin.z * 10 - hi[2]) < 1e-4]
    info = dict(sharp=sharp, volume=body.volume * 1000.0, thickness=hi[2] - lo[2],
                material=body.material.name if body.material else None, corner_edges=len(corner),
                rounded=[f.geometry.radius * 10 for f in body.faces if _is(f, ST.CylinderSurfaceType)],
                outline=sum(e.length * 10 for e in top[0].edges) if top else None, area=top[0].area * 100 if top else None,
                outline_box=(hi[0] - lo[0], hi[1] - lo[1]), clear=h['clear'], round_from=h['round_from'],
                hand=dict(sharp=h0['V'], rounded=h['V'], outline=h['P']))
    # sixteen occurrences, nested on the plate by Fusion's Arrange
    occs = [o1] + [root.occurrences.addExistingComponent(comp, _off_plate(k)) for k in range(1, MK.TINE['n'])]
    for o in occs:
        try:
            o.isGrounded = False
        except Exception:
            pass
    if des.snapshots.hasPendingSnapshot:
        des.snapshots.add()
    arranged_by = 'Fusion'
    try:
        feat, note = _arrange(des, occs, log)
    except Exception as e:
        note = dict(error=str(e), trace=traceback.format_exc().splitlines()[-3:])
        arranged_by = 'the script (Arrange failed: %s)' % str(e).splitlines()[0][:200]
        _layout_by_hand(occs, info['outline_box'][0], info['outline_box'][1], log)
    adsk.doEvents()
    W, H = MK.TINE['plate']
    boxes, turns, on = [], [], []
    for o in occs:
        a, b = _vbox(o.bRepBodies.item(0))
        t = o.transform2
        if a[0] >= -1e-3 and a[1] >= -1e-3 and b[0] <= W + 1e-3 and b[1] <= H + 1e-3:   # on the plate
            on.append(o)
            boxes.append((a, b))
            turns.append(round(math.degrees(math.atan2(t.getCell(1, 0), t.getCell(0, 0))), 3))
    inside = len(on)
    if not boxes:
        boxes = [_vbox(o.bRepBodies.item(0)) for o in occs]
    x0, y0 = min(a[0] for a, b in boxes), min(a[1] for a, b in boxes)
    x1, y1 = max(b[0] for a, b in boxes), max(b[1] for a, b in boxes)
    area = 0.0
    for o in on:                                               # each tine's top face, as Fusion measures it
        pb = o.bRepBodies.item(0)
        zt_o = _vbox(pb)[1][2]
        area += sum(f.area * 100 for f in pb.faces if _is(f, ST.PlaneSurfaceType)
                    and abs(abs(_unit(f.geometry.normal)[2]) - 1) < 1e-6 and abs(f.geometry.origin.z * 10 - zt_o) < 1e-4)
    res = note.get('result', {}) if isinstance(note, dict) else {}
    eff = None
    for k, v in res.items():
        if ('efficien' in k.lower() or 'utiliz' in k.lower()) and isinstance(v, (int, float)):
            eff = v / 100.0 if v > 1.0 else v
    fusion = arranged_by == 'Fusion'
    info.update(nest=(x1 - x0, y1 - y0), nest_lo=(x0, y0), turns=sorted(set(turns)), arranged=inside if fusion else 0,
                unarranged=(MK.TINE['n'] - inside) if fusion else MK.TINE['n'],
                efficiency=(eff if eff is not None else area / (W * H)) if fusion else 0.0,
                efficiency_from=('the arrange feature' if eff is not None else 'the tines\' faces as Fusion measures them')
                if fusion else None, arranged_by=arranged_by, arrange=note)
    # one cutting setup over the nested copies, one 2D profile with the sample laser over their top faces
    cam = _cam(doc)
    cut = SAMPLE_LIBS[3]
    lasers = [t for t in _lib_tools(cut) if 'laser' in _ttype(t)]
    if not lasers:
        raise RuntimeError('no laser in %s' % cut)
    laser = _Numbers()(lasers[0])
    feed = _feed(laser, 'tool_feedCutting')
    pierce = _tv(laser, 'tool_pierceTime')
    info['feed'] = feed
    info['pierce'] = pierce if isinstance(pierce, (int, float)) else 0.0
    tops = []
    for o in occs:
        pb = o.bRepBodies.item(0)
        zt_o = _vbox(pb)[1][2]
        tops += [f for f in pb.faces if _is(f, ST.PlaneSurfaceType) and abs(abs(_unit(f.geometry.normal)[2]) - 1) < 1e-6
                 and abs(f.geometry.origin.z * 10 - zt_o) < 1e-4]
    s = _setup(cam, adsk.cam.OperationTypes.JetOperation, [o.bRepBodies.item(0) for o in occs],
               '1 Cut from %g mm plate' % MK.TINE['t'], _mill_stock(MK.TINE['frame'], 0.0, 0.0), [], log)
    op = _op(s, 'jet2d', laser, '2D profile, %d tines' % len(occs), [], log)
    _contours(op, tops, log, faces=True)
    out = [_program(cam, s, [op], 'jet.cps', PROGRAMS['tine'][0], log)]
    if info['feed'] is None and out[0]['nc']:
        txt = io.open(os.path.join(NC_DIR, out[0]['nc']), encoding='latin-1').read()
        fs = sorted(set(re.findall(r'F(\d+\.?\d*)', txt)))
        info['feed'] = float(fs[0]) if len(fs) == 1 else None
        log('the laser\'s feed read from its program: %s' % fs)
    return info, out


def _feed(tool, name):
    """A feed in mm/min, from its expression ('1000 mm/min')."""
    p = tool.parameters.itemByName(name)
    if p is None:
        return None
    e = str(getattr(p, 'expression', '') or '')
    m = re.match(r'^\s*([\d.]+)\s*(mm/min|m/min|in/min|ipm|mm/s)?\s*$', e)
    if not m:
        return None
    return float(m.group(1)) * {None: 1.0, 'mm/min': 1.0, 'm/min': 1000.0, 'in/min': 25.4, 'ipm': 25.4,
                                'mm/s': 60.0}[m.group(2)]


# ---------- the census ----------
_KIND = {ST.PlaneSurfaceType: 'plane', ST.CylinderSurfaceType: 'cylinder', ST.ConeSurfaceType: 'cone',
         ST.SphereSurfaceType: 'sphere', ST.TorusSurfaceType: 'torus', ST.NurbsSurfaceType: 'nurbs',
         ST.EllipticalCylinderSurfaceType: 'elliptical cylinder', ST.EllipticalConeSurfaceType: 'elliptical cone'}


def _face(f):
    g = f.geometry
    t = _KIND.get(g.surfaceType, 'other')
    out = dict(t=t, a=f.area * 100.0)
    if t == 'plane':
        out.update(n=_unit(g.normal), o=_mm(g.origin))
    elif t == 'cylinder':
        out.update(ax=_unit(g.axis), o=_mm(g.origin), r=g.radius * 10.0)
    elif t == 'cone':
        out.update(ax=_unit(g.axis), o=_mm(g.origin), r=g.radius * 10.0, half=g.halfAngle)
    elif t == 'torus':
        out.update(ax=_unit(g.axis), o=_mm(g.origin), r=g.majorRadius * 10.0, r2=g.minorRadius * 10.0)
    elif t == 'sphere':
        out.update(o=_mm(g.origin), r=g.radius * 10.0)
    return out


def _smallest_box(body, faces):
    """The smallest box round the body: Fusion's minimum box against boxes along the body's own directions (its
    planes' normals and its axes), the least volume kept; sides ascending with their directions, mm."""
    mm_ = adsk.core.Application.get().measureManager
    cands = []
    try:
        ob = body.orientedMinimumBoundingBox
        if ob is not None:
            cands.append(('minimum', ob))
    except Exception:
        pass
    dirs = []
    for f in sorted(faces, key=lambda f: -f['a']):
        d = f.get('n') or f.get('ax')
        if d and not any(abs(abs(_dot(d, e)) - 1) < 1e-6 for e in dirs):
            dirs.append(d)
    frames = [((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))]
    for d in dirs[:6]:
        e = [x for x in dirs if abs(_dot(x, d)) < 1e-6]
        frames.append((d, e[0] if e else _perp(d)))
    for d1, d2 in frames:
        try:
            cands.append(('along', mm_.getOrientedBoundingBox(body, V3(*d1), V3(*d2))))
        except Exception:
            pass
    if not cands:
        b = body.boundingBox
        s = [(b.maxPoint.x - b.minPoint.x) * 10, (b.maxPoint.y - b.minPoint.y) * 10, (b.maxPoint.z - b.minPoint.z) * 10]
        c = _mm(P3((b.maxPoint.x + b.minPoint.x) / 2, (b.maxPoint.y + b.minPoint.y) / 2, (b.maxPoint.z + b.minPoint.z) / 2))
        sides = sorted(zip(s, [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]))
        return [x for x, _ in sides], [d for _, d in sides], c, 'world'
    src, ob = min(cands, key=lambda c: c[1].length * c[1].width * c[1].height)
    sides = sorted([(ob.length * 10, _unit(ob.lengthDirection)), (ob.width * 10, _unit(ob.widthDirection)),
                    (ob.height * 10, _unit(ob.heightDirection))], key=lambda x: x[0])
    return [x for x, _ in sides], [d for _, d in sides], _mm(ob.centerPoint), src


def _measure(body):
    faces = [_face(f) for f in body.faces]
    box, dirs, c, src = _smallest_box(body, faces)
    return dict(faces=faces, volume=body.volume * 1000.0, box=box, box_dirs=dirs, box_centre=c, box_from=src)


def build_census(log):
    app = adsk.core.Application.get()
    rs, bom = _load('RangeSweep'), _load('BillOfMaterials')
    kinds, machines = [], []
    for key in bom.MACHINES:
        t0 = time.time()
        rs.machine(key)
        doc = app.activeDocument
        try:
            doc.attributes.add('mdfc', 'run', 'Making')
        except Exception:
            pass
        des = adsk.fusion.Design.cast(app.activeProduct)
        b, occ = _bill(bom, key, des)
        made = 0
        for g in b['lines']:
            if g['kind'] != 'made':
                continue
            weld, pieces = _pieces(g, b['lines'])
            bodies = []
            for label, name in pieces:
                if name not in occ:
                    log('%s %s: no component %s' % (key, g['number'], name))
                    continue
                for bd in occ[name][0].bRepBodies:
                    m = _measure(bd)
                    m.update(piece=label, component=name)
                    bodies.append(m)
            kinds.append(dict(machine=key, number=g['number'], description=g['description'], qty=g['qty'],
                              material=g.get('material'), weldment=weld, pieces=[n for _l, n in pieces], bodies=bodies))
            made += 1
            adsk.doEvents()
        machines.append(dict(key=key, made_kinds=made, seconds=time.time() - t0))
        doc.close(False)
        adsk.doEvents()
    os.makedirs(RESULTS, exist_ok=True)
    with io.open(CENSUS, 'w', encoding='utf-8') as f:
        json.dump(dict(version=VERSION, units='mm, mm2, mm3', machines=machines, kinds=kinds), f, indent=0, default=str)
    cs = MK.census(CENSUS)
    return dict(kinds=cs['kinds'], pieces=cs['pieces'], kg=cs['kg'], machines=machines,
                by={k: {n: dict(kinds=g['kinds'], pieces=g['pieces']) for n, g in v.items()} for k, v in cs['by'].items()},
                known={r['number']: (r['route'], r['shape'], r['bodies']) for r in cs['rows']
                       if r['number'] in ('A01-016', 'P13-012', 'P13-018', 'P13-016', 'A01-004', 'A01-007')})


# ---------- the build ----------
def _min(s):
    return '%.1f min' % (s / 60.0)


def build(part='bracket'):
    part = str(part).strip().lower()
    if part not in CHOICES:
        raise ValueError('No part called %s: Making programs the %s, or takes the census.' % (part, ', '.join(PARTS)))
    _TOOLS.clear()
    log = _Log()
    t0 = time.time()
    if part == 'census':
        c = build_census(log)
        _last.clear()
        _last.update(part='census', path=CENSUS, log=list(log), seconds=time.time() - t0, **c)
        return ('Census of the made kinds: %d kinds in %d pieces, %.1f kg, measured in the nine machines and written to '
                'design_office/results/making_census.json.\nBy route: %s\nBy shape: %s\nKnown: %s%s\n\n[%s]' % (
                    c['kinds'], c['pieces'], c['kg'],
                    '; '.join('%s %d' % (k, g['kinds']) for k, g in sorted(c['by']['route'].items())),
                    '; '.join('%s %d' % (k, g['kinds']) for k, g in sorted(c['by']['shape'].items())),
                    '; '.join('%s %s, %s (%d bodies)' % (n, r, s, b) for n, (r, s, b) in sorted(c['known'].items())),
                    ('\nNotes: ' + '; '.join(log[:12])) if log else '', VERSION))
    info, setups = dict(bracket=build_bracket, spacer=build_spacer, tine=build_tine)[part](log)
    total = sum(st['seconds'] for st in setups)
    _last.clear()
    _last.update(part=part, info=info, setups=setups, total=total, log=list(log), seconds=time.time() - t0,
                 version=VERSION)
    os.makedirs(RESULTS, exist_ok=True)
    with io.open(os.path.join(RESULTS, 'making_%s.json' % part), 'w', encoding='utf-8') as f:
        json.dump(dict(_last), f, indent=1, default=str)
    head = dict(
        bracket=lambda i: ('The opening stop bracket P13-018, its arm turned %g deg and its root R%g: %.1f mm3 (by hand '
                           '%.1f), %d holes, %d bore, %d root edges; %s.' % (i['turn'], i['r_root'], i['volume'],
                                                                             i['hand_volume'], i['holes'], i['bore'],
                                                                             i['root_edges'], i['material'])),
        spacer=lambda i: ('The pinion spacer A01-016: %.2f mm long, %g / %g across; faced %s mm and parted %s mm long '
                          'each, so it comes off at %.1f for grinding to %.2f-%.2f.' % (
                              i['length'], round(i['d_out'], 3), round(i['d_in'], 3), i.get('front'), i.get('back'),
                              i['stock'], i['range'][0], i['range'][1])),
        tine=lambda i: ('The tine P13-012: %.1f mm3 sharp, %.1f with its corner R%g, outline %.3f mm, %g thick, %s; '
                        '%d of %d nested by %s in %.1f x %.1f, %.2f %% of the plate; laser at %s mm/min.' % (
                            i['sharp']['volume'], i['volume'], MK.TINE['r'], i['outline'] or 0, i['thickness'],
                            i['material'], i['arranged'], MK.TINE['n'], i['arranged_by'], i['nest'][0], i['nest'][1],
                            100 * (i['efficiency'] or 0), i['feed'])))[part](info)
    rows = []
    for st in setups:
        rows.append('%s: %s; %s (%.0f mm cut, %.0f mm rapid); %s, %d lines' % (
            st['setup'], ', '.join('%s [%s]%s' % (o['name'], o['tool'], '' if o['toolpath'] and not o['warning'] else
                                                 (' WARNING %s' % o['message'] if o['toolpath'] else ' NO TOOLPATH'))
                                   for o in st['ops']),
            _min(st['seconds']), st['feed'], st['rapid'], st['nc'] or 'NO NC FILE', st['nc_lines']))
    return '%s\n%s\nIn all %s of machining at the programmed feeds.%s\n\n[%s]' % (
        head, '\n'.join(rows), _min(total), ('\nNotes: ' + '; '.join(log[:12])) if log else '', VERSION)


# ---------- the command dialog ----------
_handlers = []
EVENT = 'mdfcMakingBuild'


class _Build(adsk.core.CustomEventHandler):
    """The build, outside the command: Fusion refuses a new document inside a command's events."""

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.additionalInfo))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('Making failed:\n%s' % traceback.format_exc())


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            part = args.command.commandInputs.itemById('part').selectedItem.name
            adsk.core.Application.get().fireCustomEvent(EVENT, part)
        except Exception:
            ui.messageBox('Making failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('part', 'Part', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in CHOICES:
            dd.listItems.add(k, k == 'bracket')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        try:
            app.unregisterCustomEvent(EVENT)
        except Exception:
            pass
        ev = app.registerCustomEvent(EVENT)
        hb = _Build()
        ev.add(hb)
        _handlers.append(hb)
        cid = 'mdfcMaking'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Making', 'Chapter 66: the program for one made part')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Making failed:\n%s' % traceback.format_exc())
