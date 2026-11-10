# -*- coding: utf-8 -*-
"""wrist_detail.py: Chapter 52 of Volume 3, the robotic wrist in detail. Volume 2's wrist (lib/wrist_design.py,
Chapters 39 and 40) assumed its bought products; this module puts the makers' figures in their place, works the
wrist's numbers again with them, and checks the bearings' abutments and load classes against SKF's pages.

Sources (all in the references folder, Web pages/):
- Beckhoff AM8111 servomotor and AG2250 (Neugart PLE040) gear unit: Beckhoff_AM8111_PLE040_2026-11-03.md.
- Senring SNM022A-06 slip ring, sold as SparkFun ROB-13064: SparkFun_slip_ring_6_wire_2A_2026-11-03.md.
- SKF 6002-2RSH and 61902-2RS1: SKF_6002-2RSH_61902-2RS1_2026-11-03.md; SKF's seat tables as Chapter 51 read them.

    python wrist_detail.py      -> the products against the assumptions, the numbers again, the abutments
"""
import contextlib
import copy
import io
import math
import os
import re

import wrist_design as W

REFS = os.path.join(os.path.expanduser('~'), 'Documents', 'Mechanical Design References', 'Web pages')

# ---- the products (one maker's motor and gear unit, so that the unit's adapter fits the motor's flange) -------------
MOTOR = dict(name='Beckhoff AM8111 with holding brake, 48 V DC', rated=0.19, standstill=0.20, peak=0.68,
             n_rated=4000.0, nmax=10000.0, Jm=0.0521e-4, kT=0.07, I_peak=8.6, k=129.0, flange=40.0, spigot=30.0,
             shaft=8.0, shaft_L=25.0, brake=0.6, brake_apply=0.008, brake_release=0.014, mass=0.74)
GEARHEAD = dict(name='Beckhoff AG2250-PLE040, two-stage, i = 15', i=15.0, T2N=18.0, T2B=29.0, T2Stop=36.0,
                eta=(0.97, 0.86), J1=(0.015e-4, 0.026e-4), backlash=19.0, mass=0.45, L=106.5, adapter=28.5,
                body=52.0, shaft_L=26.0, shaft=12.0, spigot=26.0, D=40.0, Fr=(200.0, 20000.0), Fr_max=240.0,
                n1N=5000.0, n1max=18000.0)
SLIPRING = dict(name='Senring SNM022A-06 (SparkFun ROB-13064)', D=22.0, L=19.5, flange=44.5, circuits=6, amps=2.0,
                rpm=250.0, volts=210.0)
DETAIL_ABUT = 18.0          # mm: the roll shaft's collar and spacers S4 and S5 cut to 18 at the 6002s' faces


def skf_page():
    """The two bearings' figures from the saved SKF page: {designation: {item: value}}."""
    s = io.open(os.path.join(REFS, 'SKF_6002-2RSH_61902-2RS1_2026-11-03.md'), encoding='utf-8').read()
    out = {}
    for block in s.split('\n## ')[1:]:
        name = block.split(',')[0].strip()
        rows = {}
        for ln in block.split('\n'):
            m = re.match(r'\| ([^|]+?) \| ([^|]+?) \|$', ln)
            if m and not m.group(1).startswith(('Item', '---')):
                v = re.search(r'-?\d+(?:\.\d+)?', m.group(2))
                rows[m.group(1).split(',')[0].strip()] = float(v.group(0)) if v else m.group(2)
        out[name] = rows
    return out


@contextlib.contextmanager
def products():
    """wrist_design's assumed products replaced by the makers' figures for the time of a calculation (or a build):
    the motor's torques, speed, inertia, brake and its delay, mass and length; the gear unit's ratio, efficiency (its
    lower figure), inertia (its higher), backlash, mass, lengths and shaft; the slip ring's size and speed."""
    saved = {k: copy.deepcopy(getattr(W, k)) for k in ('MOTOR39', 'GEARHEAD', 'SLIPRING', 'STOP40', 'BACKLASH40')}
    try:
        W.MOTOR39.update(rated=MOTOR['rated'], peak=MOTOR['peak'], n_rated=MOTOR['n_rated'], nmax=MOTOR['nmax'],
                         Jm=MOTOR['Jm'], flange=MOTOR['flange'], L_m=MOTOR['k'] - 25.0, L_br=25.0, brake=MOTOR['brake'],
                         mass=MOTOR['mass'])
        W.GEARHEAD.update(i=GEARHEAD['i'], eta=GEARHEAD['eta'][1], J_in=GEARHEAD['J1'][1],
                          L=GEARHEAD['body'] - 5.0 + GEARHEAD['adapter'], D=GEARHEAD['D'], shaft=GEARHEAD['shaft'],
                          shaft_L=GEARHEAD['shaft_L'], mass=GEARHEAD['mass'])
        W.SLIPRING.update(D=SLIPRING['D'], L=SLIPRING['L'], circuits=SLIPRING['circuits'], amps=SLIPRING['amps'],
                          rpm=SLIPRING['rpm'], flange=((SLIPRING['flange'] - SLIPRING['D']) / 2, 2.0))
        W.STOP40.update(delay=MOTOR['brake_apply'])
        W.BACKLASH40.update(gearhead_arcmin=GEARHEAD['backlash'])
        yield
    finally:
        for k, v in saved.items():
            getattr(W, k).clear()
            getattr(W, k).update(v)


def numbers():
    """Chapters 39 and 40's numbers, as assumed and with the products."""
    def take():
        m, s, b, w, a = W.motors(), W.stops40(), W.backlash40(), W.wrist_mass40(), W.arm_loads40()
        L = W.layout()
        return dict(motors=m, stops=s, play=b, mass=w, arm=a, m_end=L['m_end'], mv=W.moves())
    before = take()
    with products():
        after = take()
    return before, after


def drive_checks(after=None):
    """The gear unit and the motor against their makers' limits, with the products: the unit's output torque in the
    cycle (the motor's peak times the ratio and the efficiency, its higher figure), at the drive's current limit and
    in a braked stop (both brakes' torque through the unit); its radial load from the belt (both spans' installation
    tension) against 200 N for 20 000 h; the speeds; the slip ring's speed against the roll's."""
    after = after or numbers()[1]
    m = after['motors']
    i, eta_hi = GEARHEAD['i'], GEARHEAD['eta'][0]
    T_cycle = m['peak'] * i * eta_hi
    T_limit = W.LIMIT * i * eta_hi
    T_brake = MOTOR['brake'] * i
    F_belt = 2 * W.T8_5M15                          # both spans at Gates' general installation tension (Chapter 39)
    roll_rpm = after['mv']['wr'] * 60 / (2 * math.pi)
    return dict(T_cycle=T_cycle, T_limit=T_limit, T_brake=T_brake, F_belt=F_belt, n_motor=m['nmax'],
                rms_ratio=m['rms'] / MOTOR['rated'], peak_ratio=m['peak'] / MOTOR['peak'],
                brake_ratio=MOTOR['brake'] / m['brake_need'], roll_rpm=roll_rpm,
                I_limit=W.LIMIT / MOTOR['kT'])


# ---- the detail's names: the key Chapter 50 changed keeps its number; the bracket's screw, 2 mm longer for the slip
# ring's flange under the tab, is a new designation and takes the next free serial after Chapter 50's key
def detail_numbers(nmap=None):
    import detail_inventory as D
    nmap = nmap or D.number_map('wrist')
    return {'Key DIN 6885 A 5x5x10': (nmap['Key DIN 6885 A 5x5x8'][0], 'DIN 6885-1 A 5 x 5 x 10'),
            'Cap screw ISO 4762 M3x10': ('S-0108', 'ISO 4762 M3 x 10 - 8.8 - zinc-plated')}


# ---- the bearings: abutments and load classes ------------------------------------------------------------------------
# the concept's shoulders and spacers at each bearing (WristDifferential.py and WristIntegration.py), mm across
ABUTMENTS = [('roll shaft collar', '6002-2RSH', 'shaft', 20.0), ('spacer S4', '6002-2RSH', 'shaft', 19.0),
             ('spacer S5', '6002-2RSH', 'shaft', 19.0), ('spacer S3', '6002-2RSH', 'shaft', 18.0),
             ('spacer S1', '61902-2RS1', 'shaft', 18.0), ('spacer S2', '61902-2RS1', 'shaft', 18.0),
             ('spacer S3', '61902-2RS1', 'shaft', 18.0), ('cheek lip', '6002-2RSH', 'housing', 26.0),
             ('side gear shoulder', '61902-2RS1', 'housing', 25.0)]


def abutments():
    """Each shoulder against SKF's band: a shaft's between da min and da max, a housing's under Da max; the detail's
    diameter where the concept's is outside."""
    sk = skf_page()
    out = []
    for what, brg, side, d in ABUTMENTS:
        p = sk[brg]
        if side == 'shaft':
            lo, hi = p['da min'], p['da max']
            ok = lo <= d <= hi
            new = d if ok else DETAIL_ABUT
        else:
            lo, hi = None, p['Da max']
            ok = d <= hi
            new = d
        out.append(dict(what=what, bearing=brg, side=side, d=d, lo=lo, hi=hi, ok=ok, new=new))
    return out


def load_classes():
    """Each bearing position's equivalent load against SKF's C (Chapter 39's loads, the radial one governing) and the
    class SKF's tables give it: light up to 0.05 C, normal to 0.1 C, and over 0.1 C said so."""
    sk = skf_page()
    r, s, f = W.roll_bearings(), W.side_bearings(), W.frame_bearings()
    rows = [('roll shaft, upper (fixed)', '6002-2RSH', r['R4']), ('roll shaft, lower (floating)', '6002-2RSH', r['R5']),
            ('side gear, front', '61902-2RS1', s['R1']), ('side gear, in the pulley', '61902-2RS1', s['R2']),
            ('frame, in each cheek', '6002-2RSH', f['R3'])]
    out = []
    for pos, brg, P in rows:
        C = sk[brg]['C'] * 1000.0
        q = P / C
        cls = 'light' if q <= 0.05 else ('normal' if q <= 0.1 else 'over normal')
        out.append(dict(position=pos, bearing=brg, P=P, C=C, ratio=q, cls=cls))
    return out


def housing_classes():
    """SKF's table 4 on load_classes(): the front side gears' 61902s carry a rotating outer ring load, M7 when it is
    light and N7 when it is heavier; the roll shaft's lower 6002, the floating one, a load of indeterminate direction
    with its outer ring free to slide, J7 when light or normal and K7 above."""
    lc = {r['position']: r['cls'] for r in load_classes()}
    return dict(front='M7' if lc['side gear, front'] == 'light' else 'N7',
                floating='J7' if lc['roll shaft, lower (floating)'] in ('light', 'normal') else 'K7')


if __name__ == '__main__':
    b, a = numbers()
    for k in ('peak', 'rms', 'nmax', 'brake_need'):
        print('motor %-10s %.3f -> %.3f' % (k, b['motors'][k], a['motors'][k]))
    for k in ('coast', 'brake', 'stop_at'):
        print('stop  %-10s %.2f -> %.2f' % (k, b['stops'][k], a['stops'][k]))
    print('play at the tool %.3f -> %.3f mm' % (b['play']['at_tool'], a['play']['at_tool']))
    print('wrist %.3f -> %.3f kg; arm moment %.2f -> %.2f N m; motor end %.1f -> %.1f' % (
        b['mass']['m'], a['mass']['m'], b['arm']['M'], a['arm']['M'], b['m_end'], a['m_end']))
    print(drive_checks(a))
    for x in abutments():
        print(x)
    for x in load_classes():
        print(x)
