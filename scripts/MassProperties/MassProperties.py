# =============================================================================
# MassProperties.py  -  Chapter 57 of "Mechanical Design in Autodesk Fusion with Claude": the detailed machines
# weighed. It builds one of the nine machines as Part 9 detailed it (through RangeSweep.py's builders, loaded, not
# copied), its parts in Chapter 51's library materials, and weighs every group: its mass, its centre and its moments
# of inertia about its centre; for the robot also each moving body in the form lib/palletizer_links.py reads, so that
# Chapter 43's cycle can be run again on the detailed robot. The results go to results/mass_<machine>.json.
# Written from PROMPT.txt beside it; checked by checks/mass_properties_check.py.
#
#   INPUTS        the dialog: the machine (robot by default)
#   DERIVED       RangeSweep.machine() for the build; palletizer_links for the robot's bodies
#   HARD RULES    nothing of the machines' scripts copied
#   CONVENTIONS   each machine's own new design document; masses in kg, lengths in mm, inertia in kg m2
#   ACCEPTANCE    the message; the check: each machine weighed, the robot's bodies written
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import json
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
COMP = os.path.dirname(SCRIPTS)
LIB = os.path.join(COMP, 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_mp57_' + name, os.path.join(SCRIPTS, name, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


VERSION = 'MassProperties 1.1 (2026-11-03)'
MACHINES = ('robot', 'wrist', 'flap', 'axle', 'bag', 'clamp', 'vacuum', 'changer', 'unit')
_last = {}


def props(occs):
    """The occurrences' mass (kg), centre (mm, world) and moments of inertia about that centre (kg m2, world axes)."""
    m = sum(o.physicalProperties.mass for o in occs)
    if m <= 0:
        return 0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    c = [sum(o.physicalProperties.mass * getattr(o.physicalProperties.centerOfMass, a) * 10.0 for o in occs) / m
         for a in 'xyz']
    I = [0.0] * 3
    for o in occs:
        r = o.physicalProperties.getXYZMomentsOfInertia()           # kg cm2 about the world's origin
        for i in range(3):
            I[i] += r[i + 1]
    cm = [v / 1000.0 for v in c]
    return m, c, [I[0] * 1e-4 - m * (cm[1] ** 2 + cm[2] ** 2), I[1] * 1e-4 - m * (cm[0] ** 2 + cm[2] ** 2),
                  I[2] * 1e-4 - m * (cm[0] ** 2 + cm[1] ** 2)]


def leaves_under(occ):
    out = []
    stack = [occ]
    while stack:
        o = stack.pop()
        if o.component.bRepBodies.count:
            out.append(o)
        stack.extend(list(o.childOccurrences))
    return out


def robot_bodies(mod):
    """The robot's moving bodies at q = 0 in lib/palletizer_weighed.json's form (Chapter 43's check weighed the
    concept so): mass, layer y, the centre against the body's pivot and the moments of inertia its motion needs."""
    G, H = mod._last['groups'], mod.HOME
    AX, AZ = mod.AX, mod.AZ
    mod.set_pose(G, 0.0, 0.0, 0.0)
    spin = ('RV-80N output J4', 'Tool flange')
    out = []

    def add(name, occs, move, **kw):
        m, c, I = props(occs)
        bd = dict(name=name, m=round(m, 3), move=move, y=round(c[1], 1))
        if move in ('turret', 'trB', 'trW'):
            ref = {'turret': (AX, AZ), 'trB': H['B'], 'trW': H['W']}[move]
            bd.update(off=(round(c[0] - ref[0], 1), round(c[2] - ref[1], 1)), Izz=round(I[2], 3))
        elif move in ('q2@A', 'q2@E', 'trC'):
            piv = {'q2@A': H['A'], 'q2@E': H['E'], 'trC': H['C']}[move]
            bd.update(c=round(c[2] - piv[1], 1), Ia=round(I[2], 3), It=round(I[1], 3), In=round(I[0], 3))
        else:
            piv = {'q3@A': H['A'], 'q3@B': H['B'], 'q3@G': H['G']}[move]
            bd.update(c=round(c[0] - piv[0], 1), Ia=round(I[0], 3), It=round(I[1], 3), In=round(I[2], 3))
        bd.update(kw)
        out.append(bd)
    kids = lambda g: leaves_under(G[g])
    add('turret', kids('Turret'), 'turret')
    add('lower arm', kids('Lower arm'), 'q2@A')
    add('crank and counterweight', kids('Crank'), 'q3@A')
    add('parallel rod', kids('Parallel rod'), 'trC')
    add('levelling rod 1', kids('Levelling rod 1'), 'q2@E')
    add('triangle', kids('Triangle'), 'trB')
    add('upper arm', kids('Upper arm'), 'q3@B')
    add('levelling rod 2', kids('Levelling rod 2'), 'q3@G')
    add('wrist housing', [o for o in kids('Wrist') if o.component.name not in spin], 'trW')
    add('tool flange', [o for o in kids('Wrist') if o.component.name in spin], 'trW', spin=True)
    add('balancer', [o for b, r in G['balancer'] for o in (b, r)], 'turret')
    base = sum(o.physicalProperties.mass for o in kids('Base'))
    return out, base


def build(key='robot'):
    if key not in MACHINES:
        raise ValueError('No machine called %s: Chapter 57 weighs %s.' % (key, ', '.join(MACHINES)))
    RS = _load('RangeSweep')
    msg, setter = RS.machine(key)
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    groups = {}
    for top in root.occurrences:
        ls = leaves_under(top)
        if not ls:
            continue
        m, c, I = props(ls)
        name, k = top.component.name, 2
        while name in groups:                       # two groups of one component, the wrist's two input drives:
            name = '%s (%d)' % (top.component.name, k)   # each kept under its own key
            k += 1
        groups[name] = dict(m=round(m, 4), c=[round(v, 2) for v in c], I=[round(v, 6) for v in I], parts=len(ls))
    every = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    m, c, I = props(every)
    out = dict(version=VERSION, machine=key, total=dict(m=round(m, 4), c=[round(v, 2) for v in c], I=[round(v, 6) for v in I],
                                                         parts=len(every)), groups=groups)
    if key == 'robot':
        mod = RS._mods.get('robot')
        if mod is not None:
            out['bodies'], out['base'] = robot_bodies(mod)
    json.dump(out, open(os.path.join(COMP, 'results', 'mass_%s.json' % key), 'w'), indent=1)
    _last.clear()
    _last.update(out)
    return ('Weighed: the %s, %d parts, %.2f kg, centre (%.1f, %.1f, %.1f) mm; %d groups.\n%s\n\n[%s]'
            % (key, len(every), m, c[0], c[1], c[2], len(groups), msg.split('\n')[0], VERSION))


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
            ui.messageBox('MassProperties failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('key', 'Machine', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in MACHINES:
            dd.listItems.add(k, k == 'robot')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcMassProperties'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Mass properties', 'Chapter 57: the detailed machines weighed')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('MassProperties failed:\n%s' % traceback.format_exc())
