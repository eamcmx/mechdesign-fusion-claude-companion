"""Chapter 62: the opening stop bracket and the flap's spar-fitting lug, the hand calculation against Fusion's finite
elements, and the change the bracket takes from it.

The opening bracket P13-018 (Chapter 56): a foot 70 x 70 x 12 on the end plate, an arm 24 square rising 28 mm from it
and a lug round the DYSC-25's thread, the absorber's axis 48 mm above the foot's face. The absorber lies at 45 deg to
the end plate's edges (it follows the bar's motion at the open stop), the arm's sides along the plate's edges, so the
push bends the arm about a diagonal of its square. Fusion's study of the arm with its root rounded to 2.5 mm puts the
peak at Kt 2.51 against Table 7-1's 1.7, which takes Chapter 60's fatigue factor from 2.68 to 1.90. The same study with
the push square to a face, which is the arm turned 45 deg about its own axis on the same foot, gives a section modulus
sqrt(2) larger and a peak 31 % lower. So the arm turns: its sides square to the absorber's axis and along the bolts'
line, the foot, its bolts, the lug and the absorber where they were.

The spar fitting F01-022 (Chapter 53): a lug 12 thick, 55 wide, its eye a half disc R 27.5 round a 35 bore that holds
the GE 20 ES's outer ring, pulled by half the actuator's ultimate push; Shigley's Fig. A-15-12, drawn for a
square-ended plate, against Fusion's study of the round eye.

FEA: the peaks and the section stresses Fusion wrote for each study, read from its result files at every node by
tools/sim_results.py and collected by tools/collect_sim.py into results/sim_studies.json (solved 6 November 2026,
Fusion 2706.1.17; the parts from scripts/StudyParts).

Sources: Budynas & Nisbett (2015), Shigley 10th: Table 7-1 (p. 365), Eq. 6-34 (p. 303), Eq. 6-41 (p. 315), Fig. A-15-12
(p. 1037); Fasten.it's table of DIN EN ISO 4762 (the M8 head's dk max 13, its key's s 6).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import fea_studies as FS          # noqa: E402
import fatigue_detail as FD       # noqa: E402
import range_sweep as RS          # noqa: E402

TURN = 45.0                       # deg: the arm turned about its own axis, from Chapter 56's square to the plate

# Fusion's results (results/sim_studies.json): the largest von Mises in each adaptive pass, the largest first principal
# stress at the root (s1), MPa, and the axial stress at the arm's corners and faces 12 and 24 mm up from the root
FEA = dict(
    sharp=dict(guid='4ef07d8c', passes=(130.35362, 193.38589, 260.5529, 366.40679)),
    diagonal=dict(guid='b81c055e', passes=(129.20862, 158.87468, 155.1566, 161.0693), s1=168.013, corner_12=47.109,
                  corner_24=31.919, face_24=16.369),
    square=dict(guid='c9d46a96', passes=(90.99033, 111.80748, 111.5374), s1=114.083, face_12=34.072, face_24=22.725,
                corner_24=22.592),
    lug=dict(guid='c8b13c98', passes=(184.14517, 186.1512, 187.09889), s1=192.273, net=44.377, shank=15.536),
)
# the opening stop on its end plate (StudyAssemblies 'stop': the bracket with its arm turned, the DYSC in its thread,
# the lock nut, the end plate on its four frame holes, two M8 bolt connectors at 15 921 N): the largest first principal
# stress at the root's two tension corners and the face's axial stress 12 mm up, at the design's push and at the DYSC's
# rating (Study 2, 4 mm, 1.0 mm on the root's fillets) and at the design's push alone (Study 1, 6 mm)
REAL = dict(stop=dict(guid=('0f326502', '6f00fa22'), F=(2180.0, 4000.0), peak=(148.614304, 271.30016),
                      other=(112.189736, 212.518912), face_12=(33.52959203693101, 62.30554151500411), peak_study1=149.095856,
                      up=1.92))


def _v(a, b, s=1.0):
    return (a[0] + s * b[0], a[1] + s * b[1])


def arm(turned=True):
    """The opening bracket's arm at jaw A's end, world mm: its x range, its centre in (y, z), the unit directions of
    its sides in (y, z) and its four root corners on the foot's face. Chapter 56's arm has its sides along y and z;
    turned, along the absorber's axis n and the bolts' line u."""
    o = RS.stop_layout()['open']
    c = o['centre']
    a = o['arm']['y'][1] - o['arm']['y'][0]
    if turned:
        e1, e2 = tuple(o['axis']), tuple(o['u'])
    else:
        e1, e2 = (1.0, 0.0), (0.0, 1.0)
    root = o['foot']['x'][1]
    corners = []
    for s1 in (-1, 1):
        for s2 in (-1, 1):
            p = _v(_v(c, e1, s1 * a / 2), e2, s2 * a / 2)
            corners.append((root, p[0], p[1]))
    return dict(x=(root, o['arm']['x'][1]), centre=c, a=a, e1=e1, e2=e2, corners=corners, turned=turned,
                angle=math.degrees(math.atan2(e1[1], e1[0])) % 90.0)


def clearance(turned=True, r=2.5):
    """From each M8's head to the arm's root, mm: the bolts 25 mm either side of the lug's middle along u. The arm
    reaches half its side along u when turned, half its diagonal when not; its root's fillet r reaches on along the
    foot's face to its toe. Fusion mitres the fillet's four runs where they meet at the arm's edges, so the toe is the
    arm's square grown by r with sharp corners: r further along a side, r sqrt 2 further along a diagonal. The head
    ISO 4762's dk max 13; the model draws it 1.5 d, 12."""
    o = RS.stop_layout()['open']
    c = o['centre']
    b = o['bolts'][0]
    d = math.hypot(b[0] - c[0], b[1] - c[1])
    a = arm(turned)['a']
    reach = a / 2 if turned else a / math.sqrt(2.0)
    toe = a / 2 + r if turned else (a / 2 + r) * math.sqrt(2.0)
    dk = RS.BRACKET['head'][0]
    edge, edge_drawn = d - dk / 2, d - 0.75 * RS.BRACKET['bolt']
    return dict(d=d, reach=reach, dk=dk, edge=edge, gap=edge - reach, drawn=edge_drawn - reach, r=r, toe_reach=toe,
                toe=edge - toe, toe_drawn=edge_drawn - toe, r_max=edge - a / 2)


def bracket():
    """The arm's root, Chapter 60's way, with the peak from Fusion in place of Table 7-1's first estimate: nominal
    sigma = M / W at the foot's face, W = a^3 / (6 sqrt 2) about the diagonal and a^3 / 6 square to a face; Kt the
    study's peak over it; q at the 2.5 root (Eq. 6-34), Kf = 1 + q (Kt - 1), repeated bending sa = sm = Kf sigma / 2,
    Goodman."""
    k = FS.BRACKET
    st = RS.bracket_strength()['open']
    fr = FD.bracket_roots()['open']
    Sut = FD.STEEL_1045_HR['Sut']
    a, h, F = k['a'], k['h'], k['F']
    M = F * h
    W = dict(diagonal=a ** 3 / (6 * math.sqrt(2)), square=a ** 3 / 6)
    s = {n: M / w for n, w in W.items()}
    # the concentration is on the largest first principal stress at the root, as a factor on a nominal stress is; the
    # passes' von Mises only shows whether the mesh has settled
    peak = dict(diagonal=FEA['diagonal']['s1'], square=FEA['square']['s1'])
    q = fr['drawn']['q']
    rows = {}
    for tag, Kt, sig in (('ch60', 1.7, s['diagonal']), ('diagonal', peak['diagonal'] / s['diagonal'], s['diagonal']),
                         ('square', peak['square'] / s['square'], s['square'])):
        Kf = 1 + q * (Kt - 1)
        sa = Kf * sig / 2.0
        n = 1.0 / (sa / fr['Se'] + sa / Sut)
        rows[tag] = dict(Kt=Kt, Kf=Kf, sigma=sig, peak=Kt * sig, sa=sa, n=n, n_4000=n * F / RS.DYSC[25]['F_stop'])
    # on its end plate: the same chain on the real assembly's peak at the design's push; its rating case solved too
    rs = REAL['stop']
    Kt = rs['peak'][0] / s['square']
    Kf = 1 + q * (Kt - 1)
    sa = Kf * s['square'] / 2.0
    n = 1.0 / (sa / fr['Se'] + sa / Sut)
    k = (rs['peak'][1] - rs['peak'][0]) / (rs['F'][1] - rs['F'][0])
    rows['real'] = dict(Kt=Kt, Kf=Kf, sigma=s['square'], peak=rs['peak'][0], sa=sa, n=n,
                        n_4000=n * F / RS.DYSC[25]['F_stop'], k=k, preload=rs['peak'][0] - k * rs['F'][0],
                        linear=rs['peak'][1] / rs['peak'][0], over_rigid=rs['peak'][0] / peak['square'])
    # the beam's own stress up the arm: at a corner for the diagonal, on a face for the square push
    up = dict(diagonal={x: F * (h - x) / W['diagonal'] for x in (12.0, 24.0)},
              square={x: F * (h - x) / W['square'] for x in (12.0, 24.0)})
    sharp = FEA['sharp']['passes']
    return dict(M=M, W=W, sigma=s, ratio=W['square'] / W['diagonal'], q=q, Se=fr['Se'], Sut=Sut, rows=rows, up=up,
                drop=1 - peak['square'] / peak['diagonal'], sharp=sharp, sharp_growth=sharp[-1] / sharp[0],
                ch56_sigma=st['sigma'], fr=fr)


def _overlap(turned, n=20000):
    """The arm's top inside the lug, mm^3: the lug a cylinder r 24 round the absorber's axis 20 mm above the arm's top
    face, 25 long along it; across the axis (w) the lug's underside lies sqrt(r^2 - w^2) - 20 into the arm. Turned,
    the arm's square spans 24 along the axis, inside the lug's 25; on its diagonal, its section is a diamond whose
    length along the axis is 2 (a / sqrt 2 - |w|), cut at the lug's ends."""
    o = RS.stop_layout()['open']
    r = o['lug']['r']
    gap = RS.AXIS_X - o['arm']['x'][1]
    half_lug = (o['lug']['t'][1] - o['lug']['t'][0]) / 2
    a = arm(turned)['a']
    wmax = a / 2 if turned else min(a / math.sqrt(2), math.sqrt(r * r - gap * gap))
    v = 0.0
    dw = 2 * wmax / n
    for i in range(n):
        w = -wmax + (i + 0.5) * dw
        depth = math.sqrt(r * r - w * w) - gap
        if depth <= 0:
            continue
        L = a if turned else 2 * min(half_lug, a / math.sqrt(2) - abs(w))
        v += depth * L * dw
    return v


def volume(turned=True, r=2.5):
    """The opening bracket, mm^3: foot, arm and lug, less the arm's top inside the lug, the lug's M30 bore and the two
    M8 clearance holes, with the root's fillet r as Fusion makes it (fillet())."""
    o = RS.stop_layout()['open']
    f, ar, lg = o['foot'], o['arm'], o['lug']
    foot = (f['x'][1] - f['x'][0]) * (f['y'][1] - f['y'][0]) * (f['z'][1] - f['z'][0])
    a = ar['y'][1] - ar['y'][0]
    arm_v = a * a * (ar['x'][1] - ar['x'][0])
    L = lg['t'][1] - lg['t'][0]
    lug_v = math.pi * lg['r'] ** 2 * L
    bore = math.pi * (RS.DYSC[25]['D1'] / 2.0) ** 2 * L
    holes = 2 * math.pi * 4.5 ** 2 * (f['x'][1] - f['x'][0])
    v = foot + arm_v + lug_v - _overlap(turned) - bore - holes
    if r:
        v += fillet(r, a)
    return v


def fillet(r, a=24.0, mitred=True):
    """The root's fillet round a square arm of side a, mm^3: four straight runs of the spandrel, (1 - pi/4) r^2 over
    a, and the corners at the arm's four edges. Fusion mitres the runs there: two prisms of the spandrel meeting on
    the diagonal, each corner holding the material under min(h(r1), h(r2)), h the spandrel's height at r1 and r2 from
    the two faces, which integrates to (5/3 - pi/2) r^3. fea_studies.bracket_volume took the spandrel turned a
    quarter round instead (Pappus), mitred=False here."""
    A, cbar = FS._spandrel(r)
    corner = (5.0 / 3.0 - math.pi / 2) * r ** 3 if mitred else A * (math.pi / 2) * cbar
    return 4 * A * a + 4 * corner


def lug():
    """The spar fitting's eye: the net section's stress sigma_0 = P / ((w - d) t), Fig. A-15-12's Kt for the square
    end, the study's largest principal stress over sigma_0; the shank's P / (w t); against Fusion's 7075-T6 yield."""
    k = FS.LUG
    h = FS.lug()
    Kt_fea = FEA['lug']['s1'] / h['sigma0']
    Sy = 503.0                   # MPa, the yield of Fusion's 'Aluminum 7075' (read back from resultsAttribs)
    Sy34 = 542.0                 # MPa, 7075-T6 as Chapter 34 took it (Table A-22, p. 1051)
    return dict(sigma0=h['sigma0'], shank=k['P'] / (k['w'] * k['t']), Kt_chart=k['Kt'], peak_chart=h['peak'],
                Kt_fea=Kt_fea, ratio=Kt_fea / k['Kt'], n=Sy / FEA['lug']['s1'], Sy=Sy, d_over_w=h['d_over_w'],
                h_over_w=h['h_over_w'], P=k['P'], Sy34=Sy34, n34=Sy34 / FEA['lug']['s1'],
                n_net34=Sy34 / h['sigma0'], bearing=h['bearing'],
                net_err=FEA['lug']['net'] / h['sigma0'] - 1, shank_err=FEA['lug']['shank'] / (k['P'] / (k['w'] * k['t'])) - 1)


def web_patch(margin=200.0):
    """The rear spar web the spar fitting's study takes, mm^3: the web's full depth at the spar, between the lower skin
    and 2 mm above the upper, 15 thick, for the base plate's 180 and margin either side, less the four M8 holes."""
    import flap_design as FL
    S = FL.SYS
    sp = S['spar']
    x = S['base_x'] + margin
    H = FL.skin_up(sp[0]) + 2.0 - FL.skin_low(sp[0])
    t = sp[1] - sp[0]
    return 2 * x * t * H - len(S['bolts']) * math.pi * 4.05 ** 2 * t


def closing():
    """The closing bracket's lug root, which no study took: Chapter 60's factor with Table 7-1's 1.7, and with the
    opening arm's factor on its end plate, the largest any study found, as a bound."""
    fr = FD.bracket_roots()['closed']
    Sut = FD.STEEL_1045_HR['Sut']
    Kt = REAL['stop']['peak'][0] / (FS.BRACKET['F'] * FS.BRACKET['h'] / (FS.BRACKET['a'] ** 3 / 6))
    Kf = 1 + fr['drawn']['q'] * (Kt - 1)
    sa = Kf * fr['sigma'] / 2
    n = 1.0 / (sa / fr['Se'] + sa / Sut)
    push = RS.jaw_stops()['closed']['F']
    return dict(sigma=fr['sigma'], n_ch60=fr['drawn']['n'], Kt=Kt, Kf=Kf, n=n, n_4000=n * push / RS.DYSC[25]['F_stop'])


if __name__ == '__main__':
    import pprint
    pprint.pprint(arm(True))
    pprint.pprint(arm(False))
    print('clearance', clearance(True), clearance(False))
    b = bracket()
    pprint.pprint({k: v for k, v in b.items() if k != 'fr'})
    print('volume turned', volume(True), 'diagonal', volume(False), 'sharp diagonal', volume(False, None),
          'overlaps', _overlap(True), _overlap(False))
    pprint.pprint(lug())
    pprint.pprint(closing())
