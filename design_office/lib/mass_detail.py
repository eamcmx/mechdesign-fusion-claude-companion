"""Chapter 57: the detailed machines weighed against the masses their design calculations used. Volume 2 weighed each
concept in Fusion with generic densities; Part 9 gave every part its library material and added the parts the detail
needed. Here the detailed machines' masses (results/mass_<machine>.json, written by design_office/MassProperties) are set
against the concepts' (projects/palletizer/lib/*_weighed.json and the design modules' own figures), and the calculations the masses drive
are run again: the robot's cycle (projects/palletizer/lib/palletizer_links.rework), the end-of-arm tooling against the brief's allowance,
the wrist's pitching group against Chapter 40's measured figures.

Sources: projects/palletizer/lib/palletizer_weighed.json (Chapter 43's check), projects/palletizer/lib/*_gripper_weighed.json, tool_changer_weighed.json,
compliance_unit_weighed.json (Chapters 44 to 48), projects/robotic_wrist/lib/wrist_design.MEAS40 (Chapter 40), palletizer_design.brief().
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(os.path.dirname(HERE), 'results')
try:
    import companion_paths as CP                 # on sys.path wherever a library is used: a script, a check, the book
except ImportError:                              # this file run on its own: common/ is beside design_office/
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'common'))
    import companion_paths as CP
CONCEPT = dict(bag='bag_gripper_weighed.json', vacuum='vacuum_gripper_weighed.json', clamp='clamp_gripper_weighed.json',
               changer='tool_changer_weighed.json', unit='compliance_unit_weighed.json')


def detail(key):
    p = os.path.join(RES, 'mass_%s.json' % key)
    return json.load(open(p, encoding='utf-8')) if os.path.exists(p) else None


def concept_total(key):
    """A concept's total mass as its own check weighed it, kg."""
    w = json.load(open(CP.lib_file(CONCEPT[key]), encoding='utf-8'))
    if 'total' in w and 'm' in w['total']:
        return w['total']['m']
    return sum(v['m'] for k, v in w.items() if isinstance(v, dict) and 'm' in v)


def robot_bodies():
    import palletizer_links as L
    c = L.weighed_bodies()
    d = detail('robot')
    return c, (d or {}).get('bodies')


def robot_compare():
    """Each moving body's mass, concept and detail, and the change."""
    c, d = robot_bodies()
    if d is None:
        return None
    dd = {b['name']: b for b in d}
    rows = [dict(name=b['name'], concept=b['m'], detail=dd[b['name']]['m'],
                 change=(dd[b['name']]['m'] - b['m']) / b['m'] * 100.0) for b in c if b['name'] in dd]
    return rows


def robot_rework():
    """Chapter 43's cycle on the concept's bodies and on the detail's: each joint's peak and rms torque, each reducer's
    check, J4's."""
    import palletizer_links as L
    import palletizer_drives as DR
    c, d = robot_bodies()
    out = {}
    for tag, bl in (('concept', c), ('detail', d)):
        if bl is None:
            continue
        # the modules keep the cycle they last ran, keyed only by whether bodies were given: cleared for each set
        L._FULL.clear()
        L._CYC.clear()
        DR._RUN.clear()
        r = L.rework(bl)
        out[tag] = dict(peak=r['peak'], rms=r['rms'], red=r['red'], j4=r['j4'])
    return out



def empty_share():
    """Each joint's peak torque on the cycle's empty legs (the gripper alone, the brief's allowance) against its peak on
    the loaded legs, with the detailed bodies; J2 with the balancer, as the rework takes it."""
    import palletizer_links as L
    import palletizer_drives as DR
    _, d = robot_bodies()
    if d is None:
        return None
    L._FULL.clear()
    L._CYC.clear()
    DR._RUN.clear()
    r = L.full_rows(1200, d)
    with L.bodies_in(d):
        DR._RUN.clear()
        rows2 = DR.rows_balanced(lambda q: DR.gas_torque(q), r['rows'])
    out = {}
    for j in range(4):
        rr = rows2 if j == 1 else r['rows']
        e = max(abs(x[4][j]) for x in rr if not x[5])
        f = max(abs(x[4][j]) for x in rr if x[5])
        out[j + 1] = dict(empty=e, loaded=f, share=e / f)
    L._FULL.clear()
    L._CYC.clear()
    DR._RUN.clear()
    return out


def eoat():
    """The end-of-arm tooling on J4's flange: the changer, the unit and each gripper, concept and detail, against the
    brief's allowance for the gripper."""
    import palletizer_design as P
    b = P.brief()
    rows = {}
    for k in ('changer', 'unit', 'bag', 'vacuum', 'clamp'):
        d = detail(k)
        rows[k] = dict(concept=concept_total(k), detail=d['total']['m'] if d else None)
    return dict(rows=rows, allowance=b['gripper'], payload=b['payload'])


def unit_springs():
    """The compliance unit's springs against the detailed grippers: Chapter 48 set their preload, at Chapter 44's 1.25,
    to hold the carriage on its stops when a brake stop throws the load up, from the mass below the carriage with the
    concept's gripper. The detailed gripper changes that mass by its own change; the stop's upward acceleration is kept.
    The springs as fitted give their installed force; the change in length a larger preload would need, at their
    rate, against their solid length."""
    import compliance_unit as CU
    d = CU.design()
    co = d['coil']
    fitted = co['F1'] * CU.SPRING['n']
    out = {}
    for k in ('vacuum', 'bag'):
        g = d['grippers'][k]
        det = detail(k)
        dm = (det['total']['m'] - concept_total(k)) if det else None
        a = g['lift']['a_v']
        m_new = g['m'] + dm if dm is not None else None
        F_new = CU.S_STOP * m_new * max(0.0, -a) if m_new is not None else None
        out[k] = dict(m=g['m'], dm=dm, m_new=m_new, a=a, F_pre=g['springs']['F_pre'], F_new=F_new)
    worst = max((v for v in out.values() if v['F_new'] is not None), key=lambda v: v['F_new'], default=None)
    if worst:
        dF = worst['F_new'] / CU.SPRING['n'] - co['F1']
        extra = dF / co['k']
        out['fitted'] = dict(F=fitted, F1=co['F1'], k=co['k'], L1=co['L1'], L2=co['L2'], Ls=co['Ls'], dF_each=dF,
                             shorten=extra, L2_new=co['L2'] - extra, solid=co['L2'] - extra <= co['Ls'] * 1.15)
    return out


def wrist_again():
    """The wrist as Fusion weighs it detailed, against Chapter 40's measured groups and Chapter 52's estimate with its
    products: the drive's loads (Chapter 39's equations of motion) with the detailed roll output and carrier, and the
    base plate's loads on the arm with the detailed mass and centre, both with Chapter 52's products in."""
    import wrist_design as W
    import wrist_detail as WD
    d = detail('wrist')
    if d is None:
        return None
    g = d['groups']
    ro, ca = g['Roll output'], g['Carrier']
    M = dict(roll=ro['m'], roll_z=ro['c'][2], roll_Iz=ro['I'][2], roll_Ix=ro['I'][0],
             carrier=ca['m'], carrier_z=ca['c'][2], carrier_Iy=ca['I'][1])
    with WD.products():
        before = W.loads40()
        after = W.loads(M=M)
        est = W.wrist_mass40()
        L = W.layout()
        b = W.brief()
        o = W.loads(b)
        ga = o['g_eff']
        zb = -L['base'][0]
        h_p = (zb + L['h_f'] + b['e_p']) / 1000.0
        h_w = (d['total']['c'][2] - L['base'][0]) / 1000.0
        m_w = d['total']['m']
        M_arm = b['m_p'] * ga * h_p + m_w * ga * h_w
        arm_before = W.arm_loads40(b)
    return dict(M=M, T_in=(before['T_in'], after['T_in']), Tp=(before['Tp'], after['Tp']), Tr=(before['Tr'], after['Tr']),
                m=(est['m'], m_w), z=(est['z'], d['total']['c'][2]), h_w=(est['above_base'] / 1000.0, h_w),
                M_arm=(arm_before['M'], M_arm), T_narrow=(arm_before['T_screw_narrow'], M_arm / 0.020 / 2),
                Fi_M6=arm_before['Fi_M6'], meas40=dict(W.MEAS40))


# ---- the tooling on J4 as detailed ------------------------------------------------------------------------------------
LOADS = dict(bag=('Bag', 'Station'), clamp=('Pack',), vacuum=('Pack',), changer=('Robot plate',))
# the changer's robot plate is the robot's side of the coupling: Chapter 47's "payload below the robot plate" leaves it


def alone(key):
    """A machine as detailed without what it carries or stands on: its mass, its centre's depth (mm, the machine's own
    frame, negative down) and its moment of inertia about its own vertical axis (kg m2), the groups' own about their
    centres moved there by the parallel-axis rule."""
    d = detail(key)
    if d is None:
        return None
    gs = {k: v for k, v in d['groups'].items() if not k.startswith(LOADS.get(key, ()))}
    m = sum(v['m'] for v in gs.values())
    z = sum(v['m'] * v['c'][2] for v in gs.values()) / m
    izz = sum(v['I'][2] + v['m'] * ((v['c'][0] / 1000.0) ** 2 + (v['c'][1] / 1000.0) ** 2) for v in gs.values())
    return dict(m=m, z=z, izz=izz, groups=gs)


class detailed:
    """Chapters 44 to 48's calculations with the detailed tooling: the grippers' weighed totals, the changer's groups and
    the compliance unit's two groups as Fusion weighs them detailed, in place of the concepts'; restored on leaving."""

    def __enter__(self):
        import bag_gripper as BG
        import vacuum_gripper as VG
        import clamp_gripper as CG
        import compliance_unit as CU
        self.m = dict(BG=BG, VG=VG, CG=CG, CU=CU)
        self.saved = dict(bg=BG.design, vg=VG.design, gw=CG.GW, mg=CG.M_GRIPPER, cw=CG.CW, cu=CU.weighed)
        tot = {k: alone(k) for k in ('bag', 'vacuum', 'clamp')}
        ch = detail('changer')['groups']
        cw = dict(note='detailed, Chapter 57')
        for k in ('Master', 'Piston', 'Tool plate'):
            cw[k] = dict(m=ch[k]['m'], z=ch[k]['c'][2])
        seg = [v for k, v in ch.items() if k.startswith('Segment')]
        cw['Segments'] = dict(m=sum(v['m'] for v in seg), z=sum(v['m'] * v['c'][2] for v in seg) / sum(v['m'] for v in seg))

        def wrap(fn, t):
            def design(*a, **kw):
                d = fn(*a, **kw)
                d = dict(d)
                w = dict(d.get('weighed') or {})
                w['total'] = dict(w.get('total') or {}, m=t['m'], z=t['z'], izz=t['izz'])
                d['weighed'] = w
                return d
            return design
        BG.design = wrap(self.saved['bg'], tot['bag'])
        VG.design = wrap(self.saved['vg'], tot['vacuum'])
        CG.GW = dict(self.saved['gw'] or {}, total=dict((self.saved['gw'] or {}).get('total', {}), m=tot['clamp']['m'],
                                                         z=tot['clamp']['z'], izz=tot['clamp']['izz']))
        CG.M_GRIPPER = tot['clamp']['m']
        CG.CW = cw
        u = detail('unit')['groups']
        H = CU.height()['H']
        plain = self.saved['cu']

        def weighed():
            w = plain()
            mb, zb = u['Body']['m'], -u['Body']['c'][2]
            mc, zc = u['Carriage']['m'], -u['Carriage']['c'][2]
            w.update(m_unit=mb + mc, f_unit=(mb * zb + mc * zc) / (mb + mc) / H, m_car=mc, f_car=zc / H, weighed=True)
            return w
        CU.weighed = weighed
        return self

    def __exit__(self, *exc):
        m, s_ = self.m, self.saved
        m['BG'].design, m['VG'].design = s_['bg'], s_['vg']
        m['CG'].GW, m['CG'].M_GRIPPER, m['CG'].CW = s_['gw'], s_['mg'], s_['cw']
        m['CU'].weighed = s_['cu']
        return False


def tooling():
    """The payload below the robot plate for each gripper, concept and detailed, against Chapter 41's envelope (450 kg,
    its centre 250 mm under the face, 78 kg m2 about J4's axis), each with the compliance unit under the changer; and
    the compliance unit's design again: its springs' preload for the bag and the vacuum gripper."""
    import bag_gripper as BG
    import vacuum_gripper as VG
    import clamp_gripper as CG
    import compliance_unit as CU

    def run():
        envs = dict(bag=BG.envelope(), vacuum=VG.envelope(), clamp=CG.envelope())
        H = CU.height()['H']
        w = CU.weighed()
        out = {}
        for k, e in envs.items():
            rows = CU.rows_with_unit(e, H)
            M = sum(r[1] for r in rows)
            mz = sum(r[1] * r[2] for r in rows)
            out[k] = dict(m=M, mz=mz, z=mz / M, I=e['I'], m_gripper=[r[1] for r in e['rows'] if r[0] == 'gripper'][0])
        d = CU.design()
        out['springs'] = {k: dict(m=v['m'], F_pre=v['springs']['F_pre'], a=v['lift']['a_v']) for k, v in d['grippers'].items()}
        out['F_pre'] = d['F_pre']
        out['coil'] = {k: d['coil'][k] for k in ('d', 'D', 'F1', 'F2', 'k', 'L1', 'L2', 'Ls', 'n', 'tau', 'Ssy')}
        out['unit'] = w
        return out
    concept = run()
    with detailed():
        det = run()
    return dict(concept=concept, detail=det, envelope=dict(m=450.0, mz=450.0 * 0.250, I=78.0))


# ---- the bought parts as Fusion weighs them drawn, against their makers' masses --------------------------------------
# the makers' masses: SKF's product pages (Web pages/SKF_6002-2RSH_2026-11-02.md, SKF_6206-2RS1_2026-11-03.md,
# SKF_22213E_2026-11-02.md, SKF_22207E_2026-11-02.md, SKF_61836_2026-11-02.md, SKF_61856MA_2026-11-02.md, product net weight), Festo's DYSC catalogue (Festo_DYSC_shock_absorbers_2026-11-03.md, product weight, DYSC-25), and
# Chapter 52's motor with its brake and gear unit (wrist_detail.MOTOR, GEARHEAD), and Nabtesco's RV-N catalogue
# (Nabtesco_RV-N_catalogue_CAT180410_2026-11-03.pdf, the rating table, printed p. 9: weight 9.3 kg for the RV-80N, 57.2
# for the RV-500N, 102.0 for the RV-700N); a reducer drawn as its case and its output, the two added
PROBES = dict(wrist='mass_wrist_parts.txt', bag='mass_bag_parts.txt', vacuum='mass_vacuum_parts.txt',
              robot='mass_robot_parts.txt')                                                         # results/


def probed(key):
    """Each part of a probed machine: name -> (mass kg, volume cm3), from its probe's printout."""
    import re
    txt = open(os.path.join(RES, PROBES[key]), encoding='utf-8').read()
    out = {}
    for m, name, vol in re.findall(r'^\s*([0-9.]+) kg  (.{50}) .{28} +([0-9.]+) cm3$', txt, re.M):
        out.setdefault(name.strip(), (float(m), float(vol)))
    return out


def bought():
    import wrist_detail as WD
    w, b, r = probed('wrist'), probed('bag'), probed('robot')
    rows = [('the wrist\'s motor unit', 'wrist', 'Servo motor with brake and 15:1 gearhead',
             WD.MOTOR['mass'] + WD.GEARHEAD['mass'], 'Chapter 52', 2),
            ('SKF 6002-2RSH', 'wrist', 'Bearing 6002-2RSH', 0.0298, 'SKF', 4),
            ('Festo DYSC-25-25-Y1F', 'bag', 'Shock absorber DYSC-25-25-Y1F open 1', 0.575, 'Festo', 4),
            ('SKF 6206-2RS1', 'bag', 'Bearing 6206 1', 0.2, 'SKF', 4),
            ('SKF 61856 MA', 'robot', 'Bearing 61856 MA lower arm', 7.34, 'SKF', 1),
            ('SKF 61836', 'robot', 'Bearing 61836 crank', 1.72, 'SKF', 1),
            ('SKF 22213 E', 'robot', 'Bearing 22213 E Parallel rod end 1', 1.52, 'SKF', 2),
            ('SKF 22207 E', 'robot', 'Bearing 22207 E triangle', 0.438, 'SKF', 5),
            ('Nabtesco RV-700N, J1', 'robot', ('RV-700N case J1', 'RV-700N output J1'), 102.0, 'Nabtesco', 1),
            ('Nabtesco RV-500N, J2', 'robot', ('RV-500N case J2', 'RV-500N output J2'), 57.2, 'Nabtesco', 1),
            ('Nabtesco RV-80N, J4', 'robot', ('RV-80N case J4', 'RV-80N output J4'), 9.3, 'Nabtesco', 1)]
    out = []
    for lab, mach, name, maker, src, n in rows:
        src_ = dict(wrist=w, bag=b, robot=r)[mach]
        names = name if isinstance(name, tuple) else (name,)
        m, vol = sum(src_[x][0] for x in names), sum(src_[x][1] for x in names)
        out.append(dict(part=lab, machine=mach, drawn=m, vol=vol, maker=maker, src=src, n=n, diff=(m - maker) / maker,
                        rho_drawn=m / vol * 1000.0, rho_real=maker / vol * 1000.0))
    return out


def bag_stops():
    """Chapter 56's stops as Fusion weighs them drawn, from the bag gripper's probe."""
    b = probed('bag')
    pick = lambda pre: sum(v[0] for k, v in b.items() if k.startswith(pre))
    return dict(absorbers=pick('Shock absorber DYSC'), closed=pick('Stop bracket closed'), open=pick('Stop bracket open'),
                small=pick('Lock nut DYSC') + pick('Cap screw ISO 4762 M8x40 stop') + pick('Nut ISO 4032 M8 stop') +
                pick('Washer ISO 7089 8 stop'), each_absorber=b['Shock absorber DYSC-25-25-Y1F open 1'][0],
                each_closed=b['Stop bracket closed 1'][0], each_open=b['Stop bracket open 1'][0])


def vacuum_fasteners():
    """Chapter 55's fasteners in the vacuum gripper as Fusion weighs them: the plate's through bolts with their washers
    and nuts, and the sleeves' M4 cap screws."""
    v = probed('vacuum')
    pick = lambda pre: sum(m for k, (m, _) in v.items() if k.startswith(pre))
    count = lambda pre: sum(1 for k in v if k.startswith(pre))
    return dict(through=pick('Cap screw M6x25') + pick('Washer ISO 7089 6') + pick('Nut ISO 4032 M6'),
                m4=pick('Cap screw M4x12'), n_m6=count('Cap screw M6x25'), n_m4_names=count('Cap screw M4x12'))
