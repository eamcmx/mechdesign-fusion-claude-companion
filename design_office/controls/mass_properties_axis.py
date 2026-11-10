# CONTROL for the prompt's hard constraint "the moments of inertia about the world's axes moved to the centre by the
# parallel-axis rule": the moments left about the world's origin. The whole check runs on it. Expected: the masses and
# the groups' sums unchanged, and the robot bodies' line on their moments of inertia fails, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/design_office/checks/mass_properties_check.py'
OLD = ("    return m, c, [I[0] * 1e-4 - m * (cm[1] ** 2 + cm[2] ** 2), I[1] * 1e-4 - m * (cm[0] ** 2 + cm[2] ** 2),\n"
       "                  I[2] * 1e-4 - m * (cm[0] ** 2 + cm[1] ** 2)]")
NEW = "    return m, c, [I[0] * 1e-4, I[1] * 1e-4, I[2] * 1e-4]"


def run(_context):
    comp = '__COMPANION__'
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    out_old = "'mass_%s.json' % key"
    out_new = "'mass_control_%s.json' % key"             # the control's masses apart from the chapter's
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r), (%r, %r)])\n" % (OLD, NEW, out_old, out_new))
    src = src.replace("os.path.exists('%s/design_office/results/mass_%%s.json' %% key)" % comp, 'True')
    env = {'__name__': 'mass_properties_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the moments of inertia left about the origin; the bodies\' inertia line above MUST fail, and it alone')
