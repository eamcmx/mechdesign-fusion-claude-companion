# A first build of RearAxleCarrier (Chapter 29), straight from build(), with the time each stage takes and the
# traceback if a stage fails: the step before the full check, while the script is being made to work.
import sys, importlib, time, traceback
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/RearAxleCarrier/RearAxleCarrier.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    t0 = time.time()
    try:
        msg = ns['build']()
        print(msg)
        fc.mark()
        print('  built in %.0f s' % (time.time() - t0))
    except Exception:
        print('  FAILED after %.0f s' % (time.time() - t0))
        print(traceback.format_exc())
        try:
            fc.mark()
        except Exception:
            pass
