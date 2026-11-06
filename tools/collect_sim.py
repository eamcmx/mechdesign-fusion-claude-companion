# -*- coding: utf-8 -*-
"""The Simulation studies of Chapters 58, 59 and 62 as Fusion solved them (6 November 2026), read from the result
folders Fusion writes (tools/sim_results.py) into results/sim_studies.json: each solve's design, study, mesh, the
numbers the chapters set against their hand calculations, and the folder it came from.

    python tools/collect_sim.py
"""
import io
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sim_results as S  # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), 'results', 'sim_studies.json')

# result folder -> (design in Admin Project / MechDesignFusionClaude Vol3 / Chapter 58 dev, study, mesh as set up)
SOLVES = {
    'b6e2bab0-5775-45f2-a92b-48e298c65bbb': ('Ch58 test cantilever', 'Study 1 - Static Stress', '4 mm'),
    '714a7421-0e98-43c5-8870-fb74abdb6813': ('Ch58 test cantilever', 'Study 1 - Static Stress', '2 mm'),
    'ae401cba-959f-4f46-8316-081ce3cc2e55': ('Ch58 test cantilever', 'Study 1 - Static Stress', '1 mm'),
    '3ae9e6d4-a1be-4440-919e-a360820fad18': ('Ch58 test cantilever', 'Study 2 - Modal Frequencies', '2 mm'),
    'd86bfd63-cf80-421b-b8bc-23fb413255ff': ('Ch58 keyseat', 'Study 1 - Static Stress', '3 mm, turn angle 30 deg, minimum 10 %'),
    'eb65df8a-bbac-4d7a-817e-0251c42ab613': ('Ch58 keyseat', 'Study 1 - Static Stress', '3 mm; floor and fillets 0.3 mm'),
    '54acb8ac-e64b-455a-9f47-e2bb3d04e61c': ('Ch58 keyseat', 'Study 1 - Static Stress', '3 mm; floor and fillets 0.2 mm'),
    'be10300e-ecda-431a-a5b1-0d60bfcc5935': ('Ch59 tine', 'Study 1 - Modal Frequencies', '2 mm'),
    '4ef07d8c-6fb3-4192-9c2b-38ec5469d5d9': ('Ch62 bracket sharp', 'Study 1 - Static Stress', '2 mm, adaptive 3 passes'),
    'b81c055e-fe8e-4baa-9733-e26dee04b05c': ('Ch62 bracket r2.5', 'Study 1 - Static Stress', '2 mm, adaptive 3 passes'),
    'c8b13c98-4d3b-40d2-9504-875dad92c19f': ('Ch62 lug', 'Study 1 - Static Stress', '2 mm, adaptive 3 passes'),
}
EXTRA = {          # the bracket pushed square to a face instead of along the diagonal
    'c9d46a96-7f49-4094-926a-053ada48f511': ('Ch62 bracket r2.5', 'Study 2 - Static Stress, push along -y', '2 mm, adaptive 3 passes'),
}


def folder_of(guid):
    for f in S.folders():
        if f.rstrip('\\/').endswith(guid):
            return f
    return None


def base(f):
    s = S.summary(f)
    out = dict(nodes=len(S.nodes(f)), folder=f.split('W.login')[-1].replace('\\', '/'))
    if 1 in s:
        out.update(vm_max=s[1][1] / 1e6, disp_max=s[2][1] * 1e3)
    c = S.convergence(f)
    if c:
        out['passes_vm'] = [r[1] / 1e6 for r in c]
    m = S._xml(os.path.join(f, 'volume', 'resultManifest.xml'))
    if 'SimFrequency' in m:
        rows = re.findall(r'DomainPoint Domain="0" ID="(\d+)" Value="([^"]+)">\s*<Attribute Name="MassParticipationFactorX" '
                          r'Value="([^"]+)"/>\s*<Attribute Name="MassParticipationFactorY" Value="([^"]+)"/>\s*'
                          r'<Attribute Name="MassParticipationFactorZ" Value="([^"]+)"/>', m)
        out['modes'] = [dict(f=float(v), mx=float(a), my=float(b), mz=float(c)) for _, v, a, b, c in rows]
    return out


def cantilever(f):
    o = {}
    for X in (20, 50, 100, 150):
        v, n = S.at(f, 9, (X, 0, 10), ((X - 2.6, X + 2.6), (-10, 10), (9.99, 10.01)))
        o['sxx_top_%d' % X] = v / 1e6
    o['tip_dz'] = S.probe(f, 2, ((199.99, 200.01), (-10, 10), (0, 10)))[1][2] * 1e3
    v, x, y, z = S.peak(f, 1)
    o['peak_at'] = [x, y, z]
    return o


def keyseat(f):
    xyz = S.nodes(f) * 1000
    tm = (S.values(f, 6) - S.values(f, 8)) / 2
    ks = (xyz[:, 0] > 55) & (xyz[:, 0] < 95) & (xyz[:, 2] > 8)
    i = np.where(ks)[0][np.argmax(tm[ks])]
    r = np.hypot(xyz[:, 1], xyz[:, 2])
    sel = (abs(r - 15) < 0.01) & (((xyz[:, 0] > 20) & (xyz[:, 0] < 40)) | ((xyz[:, 0] > 110) & (xyz[:, 0] < 130)))
    return dict(tau_keyseat=tm[i] / 1e6, tau_keyseat_at=list(xyz[i]), tau_nominal=float(tm[sel].mean()) / 1e6,
                Kts=float(tm[i] / tm[sel].mean()))


def bracket(f):
    xyz = S.nodes(f) * 1000
    vm, sxx = S.values(f, 1), S.values(f, 9)
    i = int(np.argmax(vm))
    o = dict(peak_at=list(xyz[i]))
    for X in (12.0, 24.0):
        for tag, sy, sz in (('pp', 1, 1), ('mm', -1, -1), ('mp', -1, 1), ('pm', 1, -1), ('p0', 1, 0), ('m0', -1, 0)):
            sel = (abs(xyz[:, 0] - X) < 1.2) & (abs(xyz[:, 1] - 12 * sy) < 0.01) & (abs(xyz[:, 2] - 12 * sz) < (0.01 if sz else 12))
            if sel.sum() > 1:
                b, a = np.polyfit(xyz[sel, 0], sxx[sel], 1)
                o['sxx_%s_%d' % (tag, X)] = float(a + b * X) / 1e6
    return o


def lug(f):
    xyz = S.nodes(f) * 1000
    s1, sxx = S.values(f, 6), S.values(f, 9)
    k = int(np.argmax(s1))
    sec = (abs(xyz[:, 0]) < 0.6) & (abs(xyz[:, 1]) >= 17.5) & (abs(xyz[:, 1]) <= 27.5)
    shank = abs(xyz[:, 0] + 60) < 0.6
    return dict(s1_max=s1[k] / 1e6, s1_at=list(xyz[k]), net_sxx_mean=float(sxx[sec].mean()) / 1e6,
                shank_sxx_mean=float(sxx[shank].mean()) / 1e6)


def collect():
    out = []
    for guid, (design, study, mesh) in list(SOLVES.items()) + list(EXTRA.items()):
        f = folder_of(guid)
        if not f:
            out.append(dict(guid=guid, design=design, study=study, mesh=mesh, missing=True))
            continue
        row = dict(guid=guid, design=design, study=study, mesh=mesh, **base(f))
        if 'modes' not in row:
            kind = {'Ch58 test cantilever': cantilever, 'Ch58 keyseat': keyseat, 'Ch62 lug': lug}.get(design, bracket)
            row.update(kind(f))
        out.append(row)
    return out


if __name__ == '__main__':
    rows = collect()
    io.open(OUT, 'w', encoding='utf-8').write(json.dumps(rows, indent=1, default=float))
    for r in rows:
        print('%-22s %-28s %-40s %s' % (r['design'], r['study'], r['mesh'], 'MISSING' if r.get('missing') else
                                       r.get('vm_max', r.get('modes', [{}])[0].get('f'))))
    print('written', OUT)
