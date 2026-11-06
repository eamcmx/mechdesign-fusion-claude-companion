# =============================================================================
# StudyParts.py  -  Chapters 58, 59 and 62 of "Mechanical Design in Autodesk Fusion with Claude": the parts the
# Simulation studies are run on, each in its own new design, one body, its material set. Fusion's Simulation has no
# API in this Fusion, so the study itself (its constraints, loads and mesh) is set up in the Simulation window as the
# chapters list; this script makes the part and prints the faces the study picks.
#   cantilever      Chapter 58's calibration: 200 x 20 x 10 steel, fixed at x = 0, 100 N at the tip down z
#   keyseat         Chapter 58: the bag gripper's long stub, 30 across, with Chapter 60's end-mill keyseat 8 x 4 x 25
#                   and its bottom corners at r/d = 0.02 (0.6 mm), twisted by the cylinder's 95.3 N m
#   tine            Chapter 59: one tine of the bag gripper's jaw, its leg's top welded to the tube
#   bracket_sharp   Chapter 62: the opening stop bracket's 24 mm square arm on its foot, the root sharp
#   bracket_r25     Chapter 62: the same with the drawing's 2.5 mm root fillet
#   lug             Chapter 62: one of the flap's spar-fitting lugs, 12 thick, 55 wide, its eye 35 across
# Checked by the chapters' own checks against the results Fusion writes (tools/sim_results.py).
#
#   INPUTS        the dialog: the part
#   DERIVED       lib/bag_gripper.py (the tine), lib/range_sweep.py (the bracket), lib/fatigue_detail.py (the
#                 keyseat), lib/flap_design.py (the lug)
#   HARD RULES    one body in the root component; nothing saved by the script
#   CONVENTIONS   a new design document; mm, the API's cm inside
#   ACCEPTANCE    the message: the part's volume and the faces to pick
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
import mdfc_parts as mp          # noqa: E402

VERSION = 'StudyParts 1.0 (2026-11-06)'
PARTS = ('cantilever', 'keyseat', 'tine', 'bracket_sharp', 'bracket_r25', 'lug')
_last = {}


def _material(name):
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    for n in name:
        m = lib.materials.itemByName(n)
        if m:
            return m
    raise RuntimeError('none of the materials %s in the library' % (name,))


STEEL = ('Steel',)
AL7075 = ('Aluminum 7075-T6', 'Aluminum 7075', 'Aluminum')


def _face_at(body, p, normal=None, tol=0.05):
    """The face of the body through the point p (mm) whose normal there is the one given, if given."""
    pt = adsk.core.Point3D.create(p[0] / 10.0, p[1] / 10.0, p[2] / 10.0)
    found = body.parentComponent.findBRepUsingPoint(pt, adsk.fusion.BRepEntityTypes.BRepFaceEntityType, tol / 10.0, False)
    for f in found:
        if f.body != body:
            continue
        if normal is not None:
            ok, n = f.evaluator.getNormalAtPoint(pt)
            if not ok or n.x * normal[0] + n.y * normal[1] + n.z * normal[2] < 0.99:
                continue
        return f
    return None


def _fillet(body, edges, r):
    coll = adsk.core.ObjectCollection.create()
    for e in edges:
        coll.add(e)
    root = body.parentComponent
    fi = root.features.filletFeatures.createInput()
    fi.addConstantRadiusEdgeSet(coll, adsk.core.ValueInput.createByReal(r / 10.0), True)
    root.features.filletFeatures.add(fi)


def _edges_where(body, test):
    out = []
    for e in body.edges:
        a, b = e.startVertex.geometry, e.endVertex.geometry
        if test([a.x * 10, a.y * 10, a.z * 10], [b.x * 10, b.y * 10, b.z * 10], e):
            out.append(e)
    return out


def geometry(which):
    """The part as a temporary body (mm), its material, and the faces the study picks: name -> (point, normal)."""
    if which == 'cantilever':
        tb = mp.box(0.0, 200.0, -10.0, 10.0, 0.0, 10.0)
        return tb, STEEL, dict(fixed=((0.0, 0.0, 5.0), (-1, 0, 0)), load=((200.0, 0.0, 5.0), (1, 0, 0)))
    if which == 'keyseat':
        tb = mp.cyl([0.0, 0.0, 0.0], [150.0, 0.0, 0.0], 15.0)
        slot = mp.box(66.5, 83.5, -4.0, 4.0, 11.0, 20.0)
        mp.union(slot, mp.cyl([66.5, 0.0, 11.0], [66.5, 0.0, 20.0], 4.0), mp.cyl([83.5, 0.0, 11.0], [83.5, 0.0, 20.0], 4.0))
        mp.cut(tb, slot)
        return tb, STEEL, dict(fixed=((0.0, 0.0, 0.0), (-1, 0, 0)), load=((150.0, 0.0, 0.0), (1, 0, 0)),
                               bottom=((75.0, 0.0, 11.0), (0, 0, 1)))
    if which == 'tine':
        tb = mp.box(-5.0, 5.0, 270.0, 295.0, -480.0, -230.0)
        mp.union(tb, mp.box(-5.0, 5.0, 40.0, 295.0, -505.0, -480.0))
        return tb, STEEL, dict(fixed=((0.0, 282.5, -230.0), (0, 0, 1)))
    if which.startswith('bracket'):
        tb = mp.box(-12.0, 0.0, -35.0, 35.0, -35.0, 35.0)
        mp.union(tb, mp.box(0.0, 48.0, -12.0, 12.0, -12.0, 12.0))
        return tb, STEEL, dict(fixed=((-12.0, 0.0, 0.0), (-1, 0, 0)), load=((48.0, 0.0, 0.0), (1, 0, 0)))
    if which == 'lug':
        tb = mp.box(-100.0, 0.0, -27.5, 27.5, 0.0, 12.0)
        mp.union(tb, mp.cyl([0.0, 0.0, 0.0], [0.0, 0.0, 12.0], 27.5))
        mp.cut(tb, mp.cyl([0.0, 0.0, -1.0], [0.0, 0.0, 13.0], 17.5))
        return tb, AL7075, dict(fixed=((-100.0, 0.0, 6.0), (-1, 0, 0)), load=((17.5, 0.0, 6.0), (-1, 0, 0)))
    raise ValueError('No part %s: the parts are %s.' % (which, ', '.join(PARTS)))


def build(which='cantilever'):
    if which not in PARTS:
        raise ValueError('No part %s: the parts are %s.' % (which, ', '.join(PARTS)))
    app = adsk.core.Application.get()
    tb, mat, picks = geometry(which)
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    try:
        doc.attributes.add('mdfc', 'run', 'StudyParts')
    except Exception:
        pass
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    bf = root.features.baseFeatures.add()
    bf.startEdit()
    root.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = root.bRepBodies.item(root.bRepBodies.count - 1)
    body.name = which
    if which == 'keyseat':                    # the keyseat's bottom corners, r/d = 0.02
        floor = _face_at(body, (75.0, 0.0, 11.0), (0, 0, 1))
        _fillet(body, list(floor.edges), 0.6)
        body = root.bRepBodies.item(root.bRepBodies.count - 1)
    if which == 'bracket_r25':                # the root all round the arm
        _fillet(body, _edges_where(body, lambda a, b, e: abs(a[0]) < 1e-6 and abs(b[0]) < 1e-6 and
                                   max(abs(a[1]), abs(a[2])) < 12.5 and max(abs(b[1]), abs(b[2])) < 12.5), 2.5)
        body = root.bRepBodies.item(root.bRepBodies.count - 1)
    body.material = _material(mat)
    vol = body.volume * 1000.0
    found = {k: _face_at(body, p, n) is not None for k, (p, n) in picks.items()}
    out = dict(part=which, volume=vol, material=body.material.name, faces=body.faces.count, picks=picks, found=found,
               mass=body.physicalProperties.mass)
    _last.clear()
    _last.update(out)
    return ('Study part %s: %.1f mm3, %s, %d faces, %.4f kg; faces to pick %s\n\n[%s]' % (
        which, vol, body.material.name, body.faces.count, body.physicalProperties.mass,
        ', '.join('%s %s' % (k, 'found' if v else 'MISSING') for k, v in found.items()), VERSION))


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
            ui.messageBox('StudyParts failed:\n%s' % traceback.format_exc())


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
        cid = 'mdfcStudyParts'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Study parts', 'Chapters 58, 59 and 62: the parts studied')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('StudyParts failed:\n%s' % traceback.format_exc())
