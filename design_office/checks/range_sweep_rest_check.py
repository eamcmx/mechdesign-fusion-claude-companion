# RangeSweep (Chapter 56), its second half: the clamp gripper, the vacuum gripper, the changer and the compliance unit,
# and the refusal of an unknown machine; checks/range_sweep_check.py run on ORDER[6:] instead of ORDER[:6], its results
# merged into results/range_sweep.json. The two halves are run apart so that neither keeps Fusion busy for more than
# half an hour.
CHECK = '__COMPANION__/design_office/checks/range_sweep_check.py'


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    assert src.count("KEYS = ORDER[:6]") == 1
    src = src.replace("KEYS = ORDER[:6]", "KEYS = ORDER[6:]")
    env = {'__name__': 'range_sweep_rest'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
