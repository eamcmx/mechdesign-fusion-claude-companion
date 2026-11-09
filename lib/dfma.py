"""Chapter 65: design for manufacture and assembly. The bag gripper's 114 pieces through Boothroyd and Dewhurst's test
for unnecessary parts, their assembly time, the design efficiency, the three kinds the test found with no fixing and the
fixings it prefers, the vacuum gripper's sleeves screwed into the plate, and how many fastener designations the nine
machines share.

Sources (all freely available, Mechanical Design References/Web pages/DFMA_open_access_sources_2026-11-04.md):
  Dewhurst & Meeker (2014), p. 3: the test for unnecessary parts, four questions: is the part required by the
    function; must it move relative to the others; must it be of a different material; is it necessary for
    assembly, disassembly or service. All "no": a candidate for elimination. Answered for a future redesign (p. 5).
  Range (2013), Eq. 1: E = N t0 / t, t0 = 3 s, N the functionally essential parts (fasteners removed).
  Matuszek et al. (2021), Table 2: Boothroyd-Dewhurst handling and insertion codes and times for a gearbox's
    operations; Table 3: the three criteria per part (relative movement, another material, separation of parts).
Shigley = Budynas & Nisbett (2015): Tables 9-1 and 9-2 (weld groups), 9-4 (AISC allowables), 9-5 (Kfs), 9-6 (minimum
fillet size); Table 8-7 (bolt lengths); Section 8-? as Volume 1 (preload 0.75 At Sp, T = K Fi d with K = 0.2).
Lengths mm, forces N, moments N mm unless named."""
import json
import math
import os

import bag_gripper as B
import palletizer_detail as PD

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(os.path.dirname(HERE), 'results')

T0 = 3.0                                    # s, Range (2013) Eq. 1; Matuszek et al. (2021) Eq. 4, ta = 3 s

# Matuszek et al. (2021), Table 2: (handling code, s, insertion code, s, their operation). The bag gripper's operations
# take the nearest of theirs; where none is near, the mapping is an assumption and is marked so (BASIS below).
TIMES = {
    'base':  ('30', 1.95, '00', 1.50, 'main housing, picked up and placed'),
    'place': ('30', 1.95, None, 0.00, 'cover, picked up (held afterwards by its screws)'),
    'press': ('00', 1.13, '31', 5.00, 'bearing pressed into the housing'),
    'shaft': ('30', 1.95, '51', 9.00, 'shaft sub-assembly pressed into its bearing'),
    'slide': ('00', 1.13, '01', 2.50, 'spacer onto the shaft sub-assembly'),
    'washer': ('05', 1.84, '06', 5.50, 'spring washer onto a screw'),
    'screw': ('10', 1.50, '38', 6.00, 'screw with washer tightened, cover to housing'),
}


def op_time(op):
    h = TIMES[op]
    return h[1] + h[3]


# The test, answered for the bag gripper as Chapters 47 and 55 left it, for a future redesign (Dewhurst & Meeker,
# p. 5). Per line: (number, pieces, short name, group, essential pieces, the question that keeps them, why, operation,
# basis of the operation's time). Questions: 'move' (moves relative to the parts already assembled), 'material'
# (must be of a different material), 'separate' (necessary for assembly or service), 'base' (the first part: nothing
# to combine it with), None (a candidate: every answer no). group: 'made', 'fastener', 'bought'.
BAG = [
    ('P13-001', 1, 'adapter plate', 'made', 1, 'base', 'the first part; the gripper is built on it', 'base', 'theirs'),
    ('S-0052', 1, 'dowel pin 10 x 22', 'fastener', 0, None, 'locates the adapter; a spigot or key machined on the '
     'adapter could', 'press', 'assumed'),
    ('S-0032', 6, 'cap screw M10 x 25', 'fastener', 0, None, 'joins two parts that do not move', 'screw', 'theirs'),
    ('P13-002', 1, 'beam', 'made', 0, None, 'no motion against the adapter, the same alloy, not needed apart',
     'place', 'theirs'),
    ('S-0031', 6, 'cap screw M8 x 20', 'fastener', 0, None, 'four join beam and adapter, two hold the end washers',
     'screw', 'theirs'),
    ('P13-004', 2, 'end block', 'made', 0, None, 'no motion against the beam or the end plate, the same alloy',
     'slide', 'assumed'),
    ('P13-005', 2, 'end plate', 'made', 1, 'separate', 'one end plate must come off for a jaw to go in; the other '
     'could be part of the beam', 'place', 'theirs'),
    ('S-0039', 8, 'cap screw M8 x 30', 'fastener', 0, None, 'join end plates and end blocks', 'screw', 'theirs'),
    ('S-0016', 4, 'bearing 6206-2RS1/C4', 'bought', 4, 'move', 'its rings turn against each other', 'press', 'theirs'),
    ('S-0110', 4, 'circlip DIN 472 62 x 2', 'fastener', 0, None, 'holds a bearing in its bore', 'washer', 'assumed'),
    ('P13-016', 2, 'jaw weldment', 'made', 2, 'move', 'turns against the end plates', 'shaft', 'theirs'),
    ('P13-012', 16, 'tine', 'made', 0, None, 'no motion against its jaw, a weldable steel like the tube',
     'place', 'assumed'),
    ('P13-015', 2, 'spacer', 'made', 0, None, 'locates a bearing on its stub', 'slide', 'theirs'),
    ('S-0107', 2, 'key DIN 6885-1 A 8 x 7 x 25', 'fastener', 0, None, 'joins lever and stub', 'slide', 'assumed'),
    ('P13-006', 2, 'lever', 'made', 2, 'separate', 'the stub carries the tube on one side of its bearing and the '
     'lever on the other; made one, the end plate could not go on', 'slide', 'assumed'),
    ('P13-014', 2, 'end washer', 'made', 0, None, 'clamps the lever on its stub', 'place', 'assumed'),
    ('S-0111', 2, 'circlip DIN 471 30 x 1.5', 'fastener', 0, None, 'holds the short stub\'s bearing', 'washer',
     'assumed'),
    ('P13-003', 2, 'cylinder bracket', 'made', 0, None, 'no motion against the end plate; the steel is a choice, '
     'not a need', 'place', 'assumed'),
    ('S-0050', 2, 'cylinder DSBC-80-125-C', 'bought', 2, 'move', 'its rod moves', 'place', 'assumed'),
    ('P13-009', 2, 'rod end', 'made', 0, None, 'moves against the lever but not against its rod: it could be the '
     'rod\'s own eye', 'screw', 'assumed'),
    ('S-0049', 2, 'cylinder DSBC-32-50', 'bought', 2, 'move', 'its rod moves', 'place', 'assumed'),
    ('S-0035', 8, 'cap screw M6 x 16', 'fastener', 0, None, 'join the press cylinders and the beam', 'screw',
     'theirs'),
    ('P13-008', 2, 'press rod', 'made', 0, None, 'moves with the press plate, not against it', 'screw', 'assumed'),
    ('P13-007', 1, 'press plate', 'made', 1, 'move', 'moves against the beam', 'place', 'theirs'),
    ('P13-017', 2, 'stop bracket, closing', 'made', 0, None, 'no motion against the end plate', 'place', 'assumed'),
    ('P13-018', 2, 'stop bracket, opening', 'made', 0, None, 'no motion against the end plate', 'place', 'assumed'),
    ('S-0120', 4, 'shock absorber DYSC-25-25', 'bought', 4, 'move', 'its plunger moves', 'screw', 'assumed'),
    ('S-0121', 8, 'cap screw M8 x 40', 'fastener', 0, None, 'hold the stop brackets', 'screw', 'theirs'),
    ('S-0122', 8, 'washer ISO 7089 8', 'fastener', 0, None, 'under the stop brackets\' nuts', 'washer', 'theirs'),
    ('S-0123', 8, 'nut ISO 4032 M8', 'fastener', 0, None, 'on the stop brackets\' bolts', 'screw', 'assumed'),
]


def bag_test(lines=BAG):
    """Pieces, the theoretical minimum, the assembly time and the design efficiency; per group and per question."""
    N = sum(l[1] for l in lines)
    Nmin = sum(l[4] for l in lines)
    t = sum(l[1] * op_time(l[7]) for l in lines)
    by_group, by_q, by_op, assumed = {}, {}, {}, 0.0
    for num, n, name, grp, ess, q, why, op, basis in lines:
        g = by_group.setdefault(grp, [0, 0])
        g[0] += n
        g[1] += ess
        if ess:
            by_q[q] = by_q.get(q, 0) + ess
        by_op[op] = by_op.get(op, 0.0) + n * op_time(op)
        if basis == 'assumed':
            assumed += n * op_time(op)
    fast = by_group['fastener'][0]
    return dict(N=N, Nmin=Nmin, t=t, E=T0 * Nmin / t, by_group=by_group, by_q=by_q, by_op=by_op, assumed=assumed,
                share_assumed=assumed / t, fasteners=fast, t_fasteners=sum(l[1] * op_time(l[7]) for l in lines
                                                                           if l[3] == 'fastener'))


# ---- the three kinds the test found with no fixing (no fastener, weld, thread or fit names them in Volumes 2 and 3)
# and the fixing the test prefers: one that adds no part where it can
UNFIXED = [('P13-012', 16, 'tines', 'welded to the tube in the jaw weldment P13-016'),
           ('P13-004', 2, 'end blocks', 'made one with the end plates, held in the beam by screws through its side walls'),
           ('P13-003', 2, 'cylinder brackets', 'screwed to the end plates from inside')]


def tine_weld():
    """Each tine's top on the tube's underside, a fillet all round the leg's 10 x 25 section (Shigley Section 9-3):
    Table 9-1 A = 1.414 h (b + d); Table 9-2 Iu = d^2 (3 b + d) / 6, bending about the jaw's axis, d = 25 the leg's
    width across it. Loads in the brake stop (Chapter 47's accelerations): the foot's share of the bag Fv at its centre,
    137.5 from the leg's mid-line; the bag's push Fh at its mid-height, 170 under the leg's top; and the tine's own
    weight and inertia at its centre (Chapter 47's tine). Both moments open the jaw and add. h = 4: Table 9-6 asks
    3/16 in for the 10 mm tine, not more than the 4 mm tube wall. Allowables 0.30 Sut of E60xx and 0.40 Sy of the
    1020 (Table 9-4), the cycle Gerber with Kfs = 2.7 (Table 9-5) as Chapter 55's plug weld."""
    d_ = B.design()
    acc = B.accelerations(d_['payload'])
    L, bag = B.LAY, B.BAG
    t = d_['tine']
    n = len(L['tine_x'])
    yl = sum(L['leg_y']) / 2.0                                 # 282.5, the leg's mid-line
    a_foot = (yl - bag['w'] / 2) + (bag['w'] / 2 - L['tip_y']) / 2   # 137.5 to the foot's share of the bag
    h_leg = L['leg_top'] - (L['bag_top'] - bag['h'] / 2)        # 170
    ty, tz = yl - t['yc'], L['leg_top'] - t['zc']              # the tine's centre from the weld: 58.07 in, 194.43 down
    b, d, h = float(L['tine_t']), float(L['leg_y'][1] - L['leg_y'][0]), 4.0
    A = 1.414 * h * (b + d)
    Iu = d * d * (3 * b + d) / 6.0
    I = 0.707 * h * Iu

    def case(av, ah):
        Fv, Fh = bag['m'] / (2 * n) * av, bag['m'] / n * ah
        Wv, Wh = t['m'] * av, t['m'] * ah
        M = Fv * a_foot + Fh * h_leg + Wv * ty + Wh * tz
        V = math.hypot(Fv + Wv, Fh + Wh)
        tV, tM = V / A, M * (d / 2.0) / I
        return dict(Fv=Fv, Fh=Fh, Wv=Wv, Wh=Wh, M=M / 1000.0, V=V, tV=tV, tM=tM, tau=math.hypot(tV, tM))

    st = case(acc['stop']['a_v'], acc['stop']['a_h'])
    cy = case(acc['cycle']['a_v'], acc['cycle']['a_h'])
    weld_allow = 0.30 * PD.WELD['E60']['Sut']
    base_allow = 0.40 * PD.STEEL_1020['HR']['Sy']
    Sut = PD.STEEL_1020['HR']['Sut']
    ka = 272.0 * Sut ** -0.995
    Sse = ka * 1.0 * 0.59 * 0.5 * Sut
    Ssu = 0.67 * Sut
    ta = tm = 2.7 * cy['tau'] / 2.0
    nf = 0.5 * (Ssu / tm) ** 2 * (ta / Sse) * (-1.0 + math.sqrt(1.0 + (2.0 * tm * Sse / (Ssu * ta)) ** 2))
    leg_stop = st['M'] * 1000.0 / (b * d * d / 6.0)            # the leg's own bending at its top with both moments
    return dict(b=b, d=d, h=h, A=A, Iu=Iu, I=I, a_foot=a_foot, h_leg=h_leg, ty=ty, tz=tz, stop=st, cycle=cy,
                weld_allow=weld_allow, n_weld=weld_allow / st['tau'], base_allow=base_allow,
                tau_base=0.707 * st['tau'], n_base=base_allow / (0.707 * st['tau']), Sse=Sse, Ssu=Ssu, ta=ta, tm=tm,
                nf=nf, leg_stop=leg_stop, leg_ch47=d_['tines']['s_leg'], n_leg=PD.STEEL_1020['HR']['Sy'] / leg_stop,
                weld_length=2 * (b + d), welds=n * 2)


def end_spigot():
    """The end block made one with the end plate: its spigot, 148 x 88 x 30, fills the beam's end as the block did,
    and the eight M8 x 30 between plate and block go. What the end plate hands the beam: the jaw's opening moment
    through the cylinder's bracket, about the beam's own axis, carried by the spigot's corners on the beam's walls; the
    bag's push on the end wall in a stop along the beam, pulling the end plate off. Two ISO 4762 M8 x 20 through the
    beam's side walls into the spigot hold it in: Shigley's L > h + 1.5 d with h = 6, the wall (Chapter 50's rule)."""
    d_ = B.design()
    acc = B.accelerations(d_['payload'])
    L = B.LAY
    w_in, h_in = 2 * (L['beam_y'] - L['beam_t']), (L['beam'][0] - L['beam'][1]) - 2 * L['beam_t']   # 148, 88
    T = d_['jaw_stop']['M']                                    # N m, one jaw's cylinder on each end plate
    F_couple = T / (w_in / 1000.0)                             # the two long walls' reaction pair
    p_couple = F_couple / (L['block'] * L['beam_t'])           # spread over the spigot's 30 on a wall's 6 (bearing)
    F_pull = B.BAG['m'] * acc['stop']['a_h']                   # the bag on one end wall, a stop along the beam
    n, d = 2, 8.0
    L_need = L['beam_t'] + 1.5 * d
    At, Sp = 36.6, 580.0                                       # M8 8.8: below M16, 580 MPa (ISO 898-1)
    Fi = 0.75 * At * Sp
    shear = F_pull / n / (math.pi * d * d / 4)
    return dict(w_in=w_in, h_in=h_in, depth=L['block'], T=T, F_couple=F_couple, p_couple=p_couple, F_pull=F_pull,
                n=n, d=d, L_need=L_need, L=20.0, Fi=Fi, shear=shear, removed=8, added=2 * n)


def bracket_screws():
    """The cylinder's rear bracket screwed to the end plate from inside: two ISO 4762 M8 through the 15 mm plate into
    the steel arm's root, tapped there, 30 apart up the arm. The cylinder lies across the gripper, so its force on
    the bracket's pin runs across too, 85 from the plate's face: a moment about the vertical that tips the arm about
    its root's edge, the screws 17.5 from it. The worst force is the cylinder's own push held closed against the stop
    (Festo: 3016 N advancing at 6 bar, Chapter 47); the brake stop's 1754 N is smaller. Preload 0.75 At Sp, the joint's
    stiffness constant C = 0.25 (an assumption, as for a short steel-aluminium joint), separation n0 = Fi / (P (1 - C)).
    Length: L > h + 1.5 d with h = 15, the plate."""
    d_ = B.design()
    L = B.LAY
    F = float(B.DSBC[B.CYL['bore']]['F_adv'])
    F_stop = d_['bearing']['F_c']
    arm = L['cyl_x'] - L['plate_x'][1]                         # 85
    root_w = 35.0                                              # the arm's width across, its root on the plate
    e = root_w / 2.0                                           # the screws on its mid-line, from the tipping edge
    M = F * arm
    n, d = 2, 8.0
    P = M / e / n                                              # each screw's share of the tipping
    At, Sp = 36.6, 580.0                                       # M8 8.8, as above
    Fi = 0.75 * At * Sp
    C = 0.25
    n0 = Fi / (P * (1 - C))
    Fb = Fi + C * P
    L_need = (L['plate_x'][1] - L['plate_x'][0]) + 1.5 * d
    shear = F / n / (math.pi * d * d / 4)
    T = 0.2 * Fi * d / 1000.0
    return dict(F=F, F_stop=F_stop, arm=arm, M=M / 1000.0, e=e, n=n, d=d, P=P, Fi=Fi, C=C, n0=n0, Fb=Fb,
                n_proof=At * Sp / Fb, L_need=L_need, L=30.0, shear=shear, T=T)


def redesign(lines=BAG):
    """The bag gripper after the three fixings: the tines in the weldment (16 pieces fewer, no new part), the end
    blocks in the end plates (2 blocks and 8 M8 x 30 fewer, 4 M8 x 20 more), the brackets screwed (4 M8 x 30 more).
    The weldment's operation stays; the new screws take the screws' time."""
    sp, br = end_spigot(), bracket_screws()
    new = [list(l) for l in lines if l[0] not in ('P13-012', 'P13-004', 'S-0039')]
    for l in new:
        if l[0] == 'S-0031':
            l[1] += sp['added']                                # the spigots' M8 x 20, the same designation
    new.append(['S-0039', br['n'] * 2, 'cap screw M8 x 30', 'fastener', 0, None, 'hold the cylinder brackets',
                'screw', 'theirs'])
    r = bag_test([tuple(l) for l in new])
    return dict(lines=new, **r)


# ---- the vacuum gripper's 36 sleeves: today a foot flange 48 across and three M4 x 12 each (Chapter 55); screwed
# into the plate instead, a fine thread on a spigot round the plunger's bore, no screw
def sleeve_thread():
    """A sleeve screwed into the 10 mm plate on an M24 x 1.5 spigot (the plunger's bore 16.2 through it), a shoulder
    30 across on the plate: tightened to T, K = 0.2 (Volume 1), its preload Fi = T / (K d) presses the shoulder's
    ring; the plunger's moment at the foot, 2.20 N m in the stop (Chapter 55), opens the ring's edge when
    M = Fi (R^2 + r^2) / (4 R) (a ring under a uniform pressure and a linear one). The time: Chapter 55's sleeve is
    placed and held by three screws; the screwed sleeve is one screw's operation."""
    s = PD.sleeve_fixing()
    M = s['M'] if 'M' in s else 2.203
    d, T, K = 24.0, 8.0, 0.2
    Fi = T * 1000.0 / (K * d)
    R, r = 15.0, 12.0
    M_open = Fi * (R * R + r * r) / (4 * R) / 1000.0
    n = 36
    before = n * (op_time('place') + 3 * op_time('screw'))
    after = n * op_time('screw')
    return dict(d=d, pitch=1.5, T=T, K=K, Fi=Fi, R=R, r=r, M=M, M_open=M_open, n_open=M_open / M, n=n,
                screws_removed=3 * n, t_before=before, t_after=after, saved=before - after)


# ---- the nine machines' fasteners: how many designations, how many shared between machines (Matuszek et al.'s
# unification index: unified components over all, here the designations used by more than one machine)
FASTENER = ('ISO 4762', 'ISO 4017', 'ISO 4032', 'ISO 7089', 'ISO 10642', 'ISO 8734', 'DIN 471', 'DIN 472',
            'DIN 6885', 'DIN 908', 'DIN 934')


def fleet_fasteners(bom=None):
    if bom is None:
        bom = json.load(open(os.path.join(RESULTS, 'bom.json'), encoding='utf-8'))
    used = {}
    pieces = 0
    for m in bom['machines']:
        for g in m['lines']:
            if g['kind'] != 'bought' or not g['number']:
                continue
            if any(g['description'].startswith(f) for f in FASTENER):
                used.setdefault(g['number'], dict(description=g['description'], machines=set(), qty=0))
                used[g['number']]['machines'].add(m['key'])
                used[g['number']]['qty'] += g['qty']
                pieces += g['qty']
    shared = [k for k, v in used.items() if len(v['machines']) > 1]
    return dict(designations=len(used), shared=len(shared), pieces=pieces, index=len(shared) / len(used),
                used={k: dict(description=v['description'], machines=sorted(v['machines']), qty=v['qty'])
                      for k, v in sorted(used.items())})


def everything():
    return dict(test=bag_test(), weld=tine_weld(), spigot=end_spigot(), bracket=bracket_screws(), after=redesign(),
                sleeve=sleeve_thread(), fleet=fleet_fasteners())


if __name__ == '__main__':
    e = everything()
    for k in ('test', 'after'):
        v = e[k]
        print(k, {a: (round(b, 4) if isinstance(b, float) else b) for a, b in v.items() if a != 'lines'})
    for k in ('weld', 'spigot', 'bracket', 'sleeve'):
        print(k, json.dumps(e[k], default=lambda o: round(o, 4) if isinstance(o, float) else str(o))[:1500])
    f = e['fleet']
    print('fleet', f['designations'], f['shared'], f['pieces'], round(f['index'], 4))
