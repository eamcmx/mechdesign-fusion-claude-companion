# Picture for Chapter 66: the sixteen tines as Fusion's Arrange nested them on the plate, with the laser's path, built by
# Making.py's tine part; the sixteen occurrences Arrange copied from are hidden, the nested copies shown, seen from
# above in the Manufacture workspace.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion


def run(_context):
    ns = fc.load('__COMPANION__/scripts/Making/Making.py')
    print(ns['build']('tine').splitlines()[0])
    fc.mark()
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeDocument.products.itemByProductType('DesignProductType'))
    root = des.rootComponent
    hidden = 0
    for i in range(root.occurrences.count):
        o = root.occurrences.item(i)
        if o.component.name == 'P13-012 Tine':
            o.isLightBulbOn = False
            hidden += 1
    print('hid %d originals; the nested copies shown' % hidden)
