# Pictures for Chapter 52: the wrist in detail, assembled, at rest: each motor with its brake and gear unit drawn to
# the makers' figures, through the far cheek's window. Taken from the standard views capture.py names.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)


def run(_context):
    ns = fc.load('__COMPANION__/scripts/WristDetail/WristDetail.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
