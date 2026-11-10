# -*- coding: utf-8 -*-
"""The Simulation studies of Chapters 58, 59 and 62 as Fusion solved them (6 November 2026), read from the result
folders Fusion writes (tools/sim_results.py) into design_office/results/sim_studies.json: each solve's design, study, mesh, the
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

OUT = os.path.join(os.path.dirname(HERE), 'design_office', 'results', 'sim_studies.json')

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
    F, M, n = S.reactions(f, ((-0.001, 0.001), (-11, 11), (-1, 11)))
    o['reaction_F'], o['reaction_M'] = list(F), list(M)
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
    F, M, n = S.reactions(f, ((-0.001, 0.001), (-16, 16), (-16, 16)))
    return dict(tau_keyseat=tm[i] / 1e6, tau_keyseat_at=list(xyz[i]), tau_nominal=float(tm[sel].mean()) / 1e6,
                Kts=float(tm[i] / tm[sel].mean()), reaction_F=list(F), reaction_M=list(M))


def bracket(f):
    xyz = S.nodes(f) * 1000
    vm, sxx = S.values(f, 1), S.values(f, 9)
    i = int(np.argmax(vm))
    o = dict(peak_at=list(xyz[i]))
    # up a tension corner (the push along -y, or along the diagonal): the largest s1 in each half millimetre
    s1 = S.values(f, 6) / 1e6
    cor = (xyz[:, 1] > 11.0) & (abs(xyz[:, 2]) > 11.0) & (np.maximum(abs(xyz[:, 1]), abs(xyz[:, 2])) < 15.5)
    o['corner_line'] = corner_line(xyz[:, 0], s1, cor)
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


def corner_line(h, s1, sel, top=40.0, step=0.5):
    out = []
    for k in range(int(top / step)):
        m = sel & (h >= k * step) & (h < (k + 1) * step)
        if m.any():
            out.append([round((k + 0.5) * step, 3), round(float(s1[m].max()), 3)])
    return out


def stop_real(f):
    """The opening stop on its end plate (StudyAssemblies 'stop'): for each load case the bracket's root, its largest
    first principal stress at the two tension corners and the axial stress on the tension face 12 mm up the arm, in
    the arm's own frame: its axis the foot's middle, its faces square and along the absorber's push."""
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'common'))
    import companion_paths  # noqa: E402  every lib/ folder on sys.path
    import range_sweep as RS
    o = RS.stop_layout()['open']
    foot = o['foot']
    c = np.array([0.0, 0.5 * sum(foot['y']), 0.5 * sum(foot['z'])])
    x0 = foot['x'][1]
    n1 = -np.array([0.0, o['axis'][0], o['axis'][1]])            # the tension face's outward normal, against the push
    n2 = np.array([0.0, n1[2], -n1[1]])
    xyz = S.nodes(f, 0) * 1000
    a, b = (xyz - c) @ n1, (xyz - c) @ n2
    root = (xyz[:, 0] > x0) & (xyz[:, 0] < x0 + 5) & (np.maximum(abs(a), abs(b)) < 15.5) & (a > 10)
    out = dict(cases=[])
    for p, _v in S.domain(f):
        s1, sxx = S.values(f, 6, 0, p) / 1e6, S.values(f, 9, 0, p) / 1e6
        row = {}
        for tag, side in (('corner_minus', b < 0), ('corner_plus', b > 0)):
            sel = root & side
            i = np.where(sel)[0][np.argmax(s1[sel])]
            row[tag] = float(s1[i])
            row[tag + '_at'] = [float(xyz[i, 0] - x0), float(a[i]), float(b[i])]
        X = x0 + 12.0
        sel = (abs(xyz[:, 0] - X) < 2.6) & (a > 11.95) & (abs(b) < 6)
        k = np.polyfit(xyz[sel, 0], sxx[sel], 1)
        row['face_12'] = float(np.polyval(k, X))
        # up the worst tension corner: the largest s1 in each half millimetre of height above the foot's face
        cor = (a > 11.0) & (b < -11.0) & (np.maximum(abs(a), abs(b)) < 15.5)
        row['corner_line'] = corner_line(xyz[:, 0] - x0, s1, cor)
        out['cases'].append(row)
    return out


def hinge_real(f):
    """The flap's right hinge bracket on its rib (StudyAssemblies 'hinge'; the bracket is body 0, x 1094 to 1106, the
    rib body 1 on its face x = 1106): where the rib's two held edges pass the load on, the bracket's body between the
    eye and the lap across its thickness and its depth, and the eye's bore at mid-thickness and on its two faces."""
    sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'common'))
    import companion_paths  # noqa: E402  every lib/ folder on sys.path
    import flap_design as FD
    d = FD.system_design()
    sy = FD.SYS
    out = dict(hand=dict(R=d['R_hinge'], axial=d['bracket_axial'], perp=d['bracket_perp'], M=d['bracket_M'],
                         sigma=d['bracket_sigma'], n=d['bracket_n'], net=d['bracket_lug']['net']))
    xr = S.nodes(f, 1) * 1000
    r = S.values(f, 123, 1, 0)
    held = np.abs(r).sum(1) > 1e-9
    out['R_upper_edge'] = r[held & (xr[:, 2] > 50)].sum(0).tolist()
    out['R_lower_edge'] = r[held & (xr[:, 2] < 0)].sum(0).tolist()
    x = S.nodes(f, 0) * 1000
    s1 = S.values(f, 6, 0, 0) / 1e6
    ya = 0.5 * (sy['bracket_bolts'][0][0] + sy['bracket_bolts'][1][0])
    za = 0.5 * (sy['bracket_bolts'][0][1] + sy['bracket_bolts'][1][1])
    e = np.array([ya, za]) / np.hypot(ya, za)                   # from the eye's centre to the bolts' middle
    n = np.array([-e[1], e[0]])
    s, t = x[:, 1:3] @ e, x[:, 1:3] @ n
    holes = np.zeros(len(x), bool)
    for by, bz in sy['bracket_bolts']:
        holes |= np.hypot(x[:, 1] - by, x[:, 2] - bz) < 14.0
    outer, rib, mid = abs(x[:, 0] - 1094.0) < 0.05, abs(x[:, 0] - 1106.0) < 0.05, abs(x[:, 0] - 1100.0) < 1.0
    out['sections'] = []
    for sv in (45.0, 60.0, 75.0, 90.0):
        sel = (abs(s - sv) < 0.75) & ~holes
        out['sections'].append(dict(s=sv, depth=float(t[sel].max() - t[sel].min()), outer=float(s1[sel & outer].mean()),
                                    rib=float(s1[sel & rib].mean()), mid=float(s1[sel & mid].mean()),
                                    max=float(s1[sel].max())))
    span = (s >= 45.0) & (s <= 90.0) & ~holes                    # the free span, 45 to 90 mm out of the eye
    k = np.where(span)[0][np.argmax(s1[span])]
    out['span_max'] = dict(s1=float(s1[k]), s=float(s[k]), t=float(t[k]), x=float(x[k, 0]))
    P = -0.5 * np.array(d['ult']['R'])                          # the flap's pull on this bracket, in (y, z)
    u = P / np.linalg.norm(P)
    w = np.array([-u[1], u[0]])
    rr = np.hypot(x[:, 1], x[:, 2])
    ang = np.degrees(np.arctan2(x[:, 1:3] @ w, x[:, 1:3] @ u))
    bore = abs(rr - 17.5) < 0.05
    for tag, sel in (('eye_mid', bore & mid), ('eye_outer', bore & outer), ('eye_rib', bore & rib)):
        k = np.where(sel)[0][np.argmax(s1[sel])]
        out[tag] = dict(s1=float(s1[k]), angle=float(abs(ang[k])))
    net = (abs(x[:, 1:3] @ u) < 0.6) & (rr >= 17.5) & (rr <= 27.6) & mid
    out['net_mid_mean'] = float(np.mean([s1[net & (rr >= a0) & (rr < a0 + 1.0)].mean() for a0 in np.arange(17.5, 27.5, 1.0)]))
    return out


READERS = dict(stop=stop_real, hinge=hinge_real)
HINGE_MESH = '2 mm'
REAL = {          # the real parts with their neighbours (StudyAssemblies), cloud folder Chapter 62 dev
    '0f326502-9158-4ced-90f4-153ee534727e': ('Ch62 stop real parts', 'Study 1 - Static Stress, 2180 N',
                                             '6 mm, local 1.0 mm on the root', 'stop'),
    '6f00fa22-9ee5-4272-9259-bdc8fd4bd4ee': ('Ch62 stop real parts', 'Study 2 - Static Stress, 2180 N and 4000 N',
                                             '4 mm, local 1.0 mm on the root', 'stop'),
    'b9819aa8-3d21-4e08-bcd8-40e1e1af66e7': ('Ch62 hinge real parts', 'Study 1 - Static Stress, first solve: the ring bonded in the eye',
                                             HINGE_MESH, 'hinge'),
    '95a19e6d-b465-4c95-b6cb-2fce153981c0': ('Ch62 hinge real parts', 'Study 1 - Static Stress, second solve: the ring in Rough contact',
                                             HINGE_MESH, 'hinge'),
}


def collect():
    out = []
    for guid, (design, study, mesh, kind) in REAL.items():
        f = folder_of(guid)
        if not f:
            out.append(dict(guid=guid, design=design, study=study, mesh=mesh, missing=True))
            continue
        out.append(dict(guid=guid, design=design, study=study, mesh=mesh, nodes=len(S.nodes(f)),
                        folder=f.split('W.login')[-1].replace('\\', '/'), **{kind: READERS[kind](f)}))
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
