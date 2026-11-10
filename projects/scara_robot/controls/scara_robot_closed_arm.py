# CONTROL for the prompt line "a window through the first arm at J2 for the J2 motor": the first arm is drawn as a
# closed box, as a beam is first sketched. The J2 module's motor hangs from its plate through where the window should
# be, so the interference check at rest must find it inside the arm's walls and fail.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/scara_robot/ScaraRobot/ScaraRobot.py'
OLD = "    mp.cut(bx, mp.box(L1 - F2 / 2 - 3, L1 + F2 / 2 + 3, -F2 / 2 - 3, F2 / 2 + 3, B1 - 1, B1 + h + 1))"
NEW = "    pass"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build'](45)
    fc.mark()
    root = fc.design().rootComponent
    meet = [o for o in root.allOccurrences if o.component.bRepBodies.count and o.component.name in
            ('First arm beam', 'Servo motor J2', 'Motor bolt M5', 'Module plate J2', 'Motor shaft J2')]
    c = fc.interference(meet)
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    print('CONTROL: the first arm without its window; the interference check MUST fail')
