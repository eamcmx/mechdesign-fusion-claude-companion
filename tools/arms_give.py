# -*- coding: utf-8 -*-
"""Chapter 59's study of the palletizer's arms sideways, read from the result files Fusion wrote: the tool flange's
outer face moves across the arms' plane under the 1000 N, and the parts along the way say where that give comes from.
Writes results/sim_arms.json (or the file named after the guid), which fig_ch59.py and the verifier read.

    python tools/arms_give.py <the study's result guid> [<out.json>]

Where the give comes from: for each part along the load path (the fork's plates, the lower arm's box, the elbow's hub
and pin, the upper arm's cheeks, its box, the flange), a rigid motion is fitted to the part's nodes by least squares,
u = t + w x (x - c), and the flange's outer face would move t + w x (f - c) across the plane if everything beyond that
part were rigid. The same running total by hand integrates palletizer_links.lateral()'s M^2/EI + T^2/GJ up to the same
place. The result files list the bodies in the order StudyAssemblies made them: its ARMS, then the study's own plates.
"""
import io
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'lib'))
import sim_results as S          # noqa: E402
import palletizer_design as PD   # noqa: E402
import palletizer_links as PL    # noqa: E402
import fea_studies as FS         # noqa: E402

OUT = os.path.join(os.path.dirname(HERE), 'results', 'sim_arms.json')
F = 1000.0                        # N across the plane, on the flange's outer face
# StudyAssemblies' ARMS, in its order: the body index in the result files
ARMS = ('Fork plate 1', 'Fork plate 2', 'Hub boss J2 side', 'Hub boss J3 side', 'Balancer cross pin Q', 'Lower arm box',
        'Elbow hub', 'Elbow pin',
        'Upper arm cheek 1', 'Upper arm cheek 2', 'Bearing 32213 elbow 1', 'Bearing 32213 elbow 2', 'Yoke bulkhead',
        'Upper arm box', 'Front fork cheek 1', 'Front fork cheek 2', 'Bearing 32213 wrist 1', 'Bearing 32213 wrist 2',
        'Torque tube', 'Rear lever', 'Pin D',
        'Wrist pin', 'Wrist hub', 'Wrist housing', 'RV-80N case J4', 'RV-80N output J4', 'Tool flange')
IX = {n: i for i, n in enumerate(ARMS)}


def body_near(f, test):
    for b in S.bodies(f):
        x = S.nodes(f, b) * 1000
        if test(x.min(0), x.max(0)):
            return b, x
    raise RuntimeError('no body found')


def rigid(x, u):
    """The rigid motion (t, w) nearest the nodes' displacements, about their centroid c, and its rms misfit."""
    c = x.mean(0)
    r = x - c
    M = np.zeros((3 * len(x), 6))
    M[0::3, 0] = M[1::3, 1] = M[2::3, 2] = 1.0
    M[0::3, 4], M[0::3, 5] = r[:, 2], -r[:, 1]
    M[1::3, 5], M[1::3, 3] = r[:, 0], -r[:, 2]
    M[2::3, 3], M[2::3, 4] = r[:, 1], -r[:, 0]
    sol = np.linalg.lstsq(M, u.reshape(-1), rcond=None)[0]
    return c, sol[:3], sol[3:], float(np.sqrt(np.mean((M @ sol - u.reshape(-1)) ** 2)))


def hand_upto(q2, q3, at, stops):
    """palletizer_links.lateral()'s integral for 1 N at `at`, summed from the shoulder up to each stop (arm, s in m),
    in mm for each kN."""
    pp = PL.pose_points(q2, q3)
    fr = PL.arm_frames(q2, q3)
    out = []
    for arm_stop, s_stop in stops:
        c = 0.0
        for arm, root, Lm in (('lower', pp['A'], PD.LAYOUT['L2'] / 1000), ('upper', pp['B'], PD.LAYOUT['L3'] / 1000)):
            ax, nn = fr[arm]
            n = 200                                   # lateral()'s own steps, so the last total is its c
            for k in range(n):
                s = (k + 0.5) / n * Lm
                if arm == arm_stop and s > s_stop:
                    break
                if arm == 'lower' and s < PL.S_HUB:
                    continue
                sec = PL.SECTIONS['fork'] if (arm == 'lower' and PL.S_HUB <= s < PL.S_FORK) else PL.SECTIONS[arm]
                Sx, Sz = root[0] + s * ax[0], root[1] + s * ax[2]
                M = (-(at[1] - Sz), 0.0, at[0] - Sx)
                T, Ml = PL.dot(M, ax), PL.dot(M, nn)
                c += (Ml * Ml / (PL.E_STEEL * sec['Ilat']) + T * T / (PL.G_STEEL * sec['J'])) * Lm / n
            if arm == arm_stop:
                break
        out.append(c * 1e6)
    return out


def read(guid):
    f = [g for g in S.folders() if g.rstrip('\\/').endswith(guid)][0]
    hi = PD.waypoints(PD.brief())['hi']
    p = PD.pts(hi['q2'], hi['q3'])
    fx, fz = p['fl']
    nb = len(S.bodies(f))
    X = {b: S.nodes(f, b) * 1000 for b in range(nb)}
    U = {b: S.values(f, 2, b) * 1000 for b in range(nb)}       # mm
    x = X[IX['Tool flange']]
    if max(abs(x[:, 0].min() - (fx - 125)), abs(x[:, 2].min() - (fz - 30)), abs(x[:, 2].max() - fz)) > 0.5:
        raise RuntimeError('body %d is not the tool flange: the bodies are not in StudyAssemblies\' order' % IX['Tool flange'])
    u = U[IX['Tool flange']]
    face = abs(x[:, 2] - (fz - 30)) < 0.01
    c = np.hypot(x[:, 0] - fx, x[:, 1]) < 3.0
    out = dict(guid=guid, nodes=int(sum(len(v) for v in X.values())), bodies=nb, flange_face_uy=float(u[face, 1].mean()),
               flange_centre_uy=float(u[face & c, 1].mean()), flange_face_n=int(face.sum()))
    # the supports' reactions, N, summed over every node held, and the solver's element count from its log
    out['reaction'] = [float(v) for v in sum(S.values(f, 123, b, 0).sum(0) for b in range(nb))]
    log = io.open(os.path.join(f, 'gewiz.out'), encoding='utf-8', errors='replace').read()
    m = re.search(r'ASSEMBLY TIME FOR\s+(\d+)\s+ELEMENTS', log)
    out['elements'] = int(m.group(1)) if m else None
    give = out['flange_face_uy'] / F * 1000.0            # mm for each kN
    # where it comes from: the parts along the path, and the same places by hand
    A = np.array([p['A'][0], 0.0, p['A'][1]])
    B = np.array([p['B'][0], 0.0, p['B'][1]])
    W = np.array([p['W'][0], 0.0, p['W'][1]])
    fpt = np.array([fx, 0.0, fz - 30.0])
    a = (B - A) / np.linalg.norm(B - A)
    au = (W - B) / np.linalg.norm(W - B)
    Lu = np.linalg.norm(W - B)

    def nodes(names, axis=None, root=None, s0=None, s1=None):
        xs = np.vstack([X[IX[n]] for n in names])
        us = np.vstack([U[IX[n]] for n in names])
        if axis is not None:
            d = (xs - root) @ axis
            m = (d >= s0) & (d <= s1)
            xs, us = xs[m], us[m]
        return xs, us
    places = [('the fork at the hub\'s bosses', nodes(('Fork plate 1', 'Fork plate 2'), a, A, 160.0, 200.0), ('lower', 0.18)),
              ('the fork where it enters the box', nodes(('Fork plate 1', 'Fork plate 2'), a, A, 640.0, 680.0), ('lower', 0.66)),
              ('the lower arm\'s box at its end', nodes(('Lower arm box',), a, A, 1100.0, 2000.0), ('lower', 1.3)),
              ('the elbow\'s hub', nodes(('Elbow hub',)), ('lower', 1.3)),
              ('the upper arm\'s cheeks', nodes(('Upper arm cheek 1', 'Upper arm cheek 2')), ('upper', 0.0)),
              ('the upper arm\'s box at its end', nodes(('Upper arm box',), au, B, Lu - 170.0, Lu - 120.0), ('upper', (Lu - 120.0) / 1000)),
              ('the tool flange', (x, u), ('upper', 9.0))]
    hand = hand_upto(hi['q2'], hi['q3'], (fx / 1000.0, (fz - 30) / 1000.0), [h for _n, _x, h in places])
    chain = []
    for (name, (xs, us), _h), hv in zip(places, hand):
        cc, t, w, fit = rigid(xs, us)
        chain.append(dict(place=name, study=float((t + np.cross(w, fpt - cc))[1]), hand=hv, twist=float(w @ a), fit=fit,
                          n=int(len(xs))))
    a_out = FS.arms(at=(fx / 1000.0, (fz - 30) / 1000.0))
    L = a_out['lateral']
    k = L['R'] ** 2 / (give * 1e-6)
    K, I1, It = L['K_drive'], L['I1'], L['I_tip']
    Ir = I1 - It
    aa, bb_, cc_ = Ir * It, -(Ir * k + It * (K + k)), K * k
    f1 = math.sqrt((-bb_ - math.sqrt(bb_ * bb_ - 4 * aa * cc_)) / (2 * aa)) / (2 * math.pi)
    out.update(give=give, give_hand=a_out['give'], ratio=give / a_out['give'], k_arm=k, f1=f1, f1_hand=L['f1'],
               give_max=a_out['give_max'], margin=a_out['give_max'] / give, chain=chain)
    return out


if __name__ == '__main__':
    o = read(sys.argv[1])
    path = sys.argv[2] if len(sys.argv) > 2 else OUT
    io.open(path, 'w', encoding='utf-8').write(json.dumps(o, indent=1))
    print(json.dumps(o, indent=1))
