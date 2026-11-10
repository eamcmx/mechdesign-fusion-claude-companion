# CONTROL for the prompt line "The worm right-handed; the wheel right-handed too (crossed shafts: the same hand)": the
# wheel is made left-handed, still with a gap facing the thread at the mid-plane. Expected: the wheel's teeth lean the
# wrong way, 1.5 deg out of line with the thread across the face width, and the interference check finds the worm and
# the wheel overlapping away from the mid-plane.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/worm_wheel/WormWheel/WormWheel.py'
EDITS = [("phase = -math.pi / 2 - math.pi / z2 - theta / 2", "phase = -math.pi / 2 - math.pi / z2 + theta / 2"),
         ("sweep = twisted(c, wheel_body, pts, 0.0, b, theta)", "sweep = twisted(c, wheel_body, pts, 0.0, b, -theta)")]


def run(_context):
    ns = fc.load(PATH, EDITS)
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  %d overlaps: %s' % (len(c), c))
    fc.ok(not c, 'no interference at rest')
    print('CONTROL: the wheel left-handed; the check above MUST fail')
