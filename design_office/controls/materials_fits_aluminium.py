# CONTROL for the rule in lib/materials_fits.py's decide() that an aluminium housing takes J7 where SKF's table gives
# H7: the script run with that line removed from the rules it reads, the check's own reading of the rules removed
# alike. The whole check runs on it. Expected: the end plates' seats come out H7, so the line for the end plates fails,
# and every other line passes, the faces still carrying what the (changed) rules decide.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/design_office/checks/materials_fits_check.py'
OLD = "importlib.reload(M)\n"
NEW = ("importlib.reload(M)\n"
       "exec(compile(open(M.__file__, encoding='utf-8').read().replace(\"h_cls = 'J7'\", \"pass\"), M.__file__, 'exec'), "
       "M.__dict__)\n")


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    env = {'__name__': 'materials_fits_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the aluminium housing\'s J7 removed; the end plates\' line above MUST fail, and it alone')
