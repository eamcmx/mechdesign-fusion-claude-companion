# -*- coding: utf-8 -*-
"""Fusion Simulation's results as it writes them to disk, read without Fusion (Fusion 2706, November 2026).

Simulation has no API in this Fusion (adsk.sim's Simulations.get returns None), so a study is set up in the window
and solved in the cloud, and its results are read here from the folder Fusion keeps for each solve:

    %LOCALAPPDATA%/Autodesk/Autodesk Fusion 360/<user>/W.login/Sim/<design>/SimResultsGuid/<guid>/
        summary.xml                      the minimum and maximum of every result (UTF-16, SI units)
        resultsAttribs.xml               the setup the solver ran: material, mesh, loads
        volume/resultManifest.xml        the results' files, body by body, and the load cases or modes
        volume/bodymesh<b>.msh_light     uint32 0, uint32 n, then n nodes as float32 x, y, z in metres, then elements
        volume/bodyresults<b>_<p>_<c>.res_light
                                         15 bytes (byte 7 the values per node, bytes 11-14 n), then n rows of float32

Result ids (found on a cantilever whose answers are known, 6 November 2026): 1 von Mises (Pa); 2 displacement
(x, y, z in m; summary ids 3, 4, 5 its parts, 2 its size); 6, 7, 8 the principal stresses 1, 2, 3; 9, 10, 11 the
normal stresses xx, yy, zz; 12, 13, 14 the shear stresses; 57 the safety factor; 58 to 63 the strains; 123 the
reaction force at each node (x, y, z in N), zero at every node not held.

    python tools/sim_results.py [results folder]       the newest solve if no folder is given: its summary
"""
import glob
import io
import os
import re
import struct
import sys

import numpy as np

SIM = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'Autodesk', 'Autodesk Fusion 360')
NAMES = {1: 'von Mises', 2: 'displacement', 3: 'dx', 4: 'dy', 5: 'dz', 6: 'principal 1', 7: 'principal 2',
         8: 'principal 3', 9: 'xx', 10: 'yy', 11: 'zz', 57: 'safety factor'}


def folders():
    """Every solve's results folder, the newest first."""
    out = glob.glob(os.path.join(SIM, '*', 'W.login', 'Sim', '*', 'SimResultsGuid', '*'))
    out = [f for f in out if os.path.exists(os.path.join(f, 'summary.xml'))]
    return sorted(out, key=lambda f: os.path.getmtime(os.path.join(f, 'summary.xml')), reverse=True)


def _xml(path):
    raw = open(path, 'rb').read()
    return raw.decode('utf-16') if raw[:2] in (b'\xff\xfe', b'\xfe\xff') else raw.decode('utf-8', 'replace')


def summary(folder):
    """{id: (min, max)} from summary.xml."""
    s = _xml(os.path.join(folder, 'summary.xml'))
    return {int(i): (float(lo), float(hi)) for i, hi, lo in
            re.findall(r'resultComponent id="(\d+)" maxValue="([^"]+)" minValue="([^"]+)"', s)}


def attribs(folder):
    return _xml(os.path.join(folder, 'resultsAttribs.xml'))


def convergence(folder):
    """An adaptive solve's passes, from convergenceData.xml: [{result id: value}] per pass, the first the mesh as set
    up; [] if the study was not refined."""
    p = os.path.join(folder, 'convergenceData.xml')
    if not os.path.exists(p):
        return []
    s = _xml(p)
    m = re.search(r'ResultComponentID values="([^"]+)"', s)
    if not m:
        return []
    ids = [int(v) for v in m.group(1).split()]
    out = []
    for row in re.findall(r'<rowData values="([^"]+)"', s):
        vals = [float(v) for v in row.split()]
        out.append({i: v for i, v in zip(ids, vals) if i})
    return out


def domain(folder):
    """The load cases or modes: [(id, value)] (a modal study's values are its frequencies)."""
    s = _xml(os.path.join(folder, 'volume', 'resultManifest.xml'))
    return [(int(i), float(v)) for i, v in re.findall(r'<DomainPoint Domain="\d+" ID="(\d+)" Value="([^"]+)"', s)]


def nodes(folder, body=0):
    with open(os.path.join(folder, 'volume', 'bodymesh%d.msh_light' % body), 'rb') as f:
        head = f.read(8)
        n = struct.unpack('<II', head)[1]
        return np.frombuffer(f.read(12 * n), dtype='<f4').reshape(n, 3).astype(float)


def values(folder, cid, body=0, point=0):
    """One result at every node: an (n,) or (n, k) array, SI units."""
    path = os.path.join(folder, 'volume', 'bodyresults%d_%d_%d.res_light' % (body, point, cid))
    raw = open(path, 'rb').read()
    k, n = raw[7], struct.unpack('<I', raw[11:15])[0]
    a = np.frombuffer(raw[15:15 + 4 * k * n], dtype='<f4').astype(float)
    return a if k == 1 else a.reshape(n, k)


def bodies(folder):
    return sorted(int(m) for m in re.findall(r'bodymesh(\d+)\.msh_light', ' '.join(os.listdir(os.path.join(folder, 'volume')))))


def probe(folder, cid, box, body=0, point=0):
    """The result at the nodes inside box = ((x0, x1), (y0, y1), (z0, z1)) in mm: (count, mean, min, max)."""
    xyz = nodes(folder, body) * 1000.0
    v = values(folder, cid, body, point)
    s = np.ones(len(xyz), bool)
    for i, (lo, hi) in enumerate(box):
        s &= (xyz[:, i] >= lo) & (xyz[:, i] <= hi)
    if not s.any():
        return 0, None, None, None
    w = v[s]
    return int(s.sum()), w.mean(axis=0), w.min(axis=0), w.max(axis=0)


def at(folder, cid, p, box, axis=0, body=0, point=0, sub=None):
    """The result at the point p (mm), from the nodes inside box fitted by a straight line along axis (0 x, 1 y, 2 z):
    for a coarse mesh, whose nodes need not lie on p. Returns (value, nodes used)."""
    xyz = nodes(folder, body) * 1000.0
    v = values(folder, cid, body, point)
    if sub is not None:
        v = v[:, sub]
    s = np.ones(len(xyz), bool)
    for i, (lo, hi) in enumerate(box):
        s &= (xyz[:, i] >= lo) & (xyz[:, i] <= hi)
    if s.sum() < 2:
        return None, int(s.sum())
    t, w = xyz[s, axis], v[s]
    if np.ptp(t) < 1e-9:
        return float(w.mean()), int(s.sum())
    b, a = np.polyfit(t, w, 1)
    return float(a + b * p[axis]), int(s.sum())


def reactions(folder, box, about=(0.0, 0.0, 0.0), body=0):
    """The reaction force (N) and moment (N mm, about the point given in mm) the constrained nodes inside box carry:
    result 123 holds each node's reaction, zero at every node not held."""
    xyz = nodes(folder, body) * 1000.0
    v = values(folder, 123, body)
    s = np.ones(len(xyz), bool)
    for i, (lo, hi) in enumerate(box):
        s &= (xyz[:, i] >= lo) & (xyz[:, i] <= hi)
    r = xyz[s] - np.array(about)
    return v[s].sum(axis=0), np.cross(r, v[s]).sum(axis=0), int(s.sum())


def peak(folder, cid, body=0, point=0):
    """The node where the result is largest: (value, x, y, z in mm)."""
    xyz = nodes(folder, body) * 1000.0
    v = values(folder, cid, body, point)
    i = int(np.argmax(v))
    return v[i], xyz[i, 0], xyz[i, 1], xyz[i, 2]


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    f = sys.argv[1] if len(sys.argv) > 1 else folders()[0]
    print(f)
    s = summary(f)
    for i in sorted(s):
        if i in NAMES:
            print('%3d %-14s %14.6g %14.6g' % (i, NAMES[i], s[i][0], s[i][1]))
    print('bodies', bodies(f), 'domain', domain(f))
    print('nodes', len(nodes(f)))
