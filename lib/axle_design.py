# =============================================================================
# axle_design.py  -  the rear axle of Chapters 28 to 31 (Volume 2): a live beam axle for a light pickup truck.
# No Fusion here: the numbers that decide the axle, worked in the order of the chapters, so that the scripts and
# their checks can call them. Every value marked "assumed" in brief() is a judgement of the brief, not a measurement.
#
#   brief()          the vehicle and the brief (Chapter 28, Table "the brief")
#   tyre(b)          the tyre's free radius from its size code 265/65 R17
#   duty(b)          the axle's duty: weight on the axles, the traction limit, the engine limit, the final-drive
#                    ratio from the top speed, the start on a grade, speeds, the turn, the life in revolutions
#   load_cases(b)    the four load cases on the wheels: bump, cornering, braking, full traction
#   envelope(b)      the ground clearance and the largest crown wheel it leaves room for
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math

G = 9.81                                                  # m/s^2


def brief(m=3100.0, m_r=1850.0, L=3100.0, h=800.0, track=1560.0, tyre_w=265.0, tyre_ar=65.0, rim_in=17.0,
          r_roll=375.0, r_stat=365.0, T_e=450.0, n_Pmax=3600.0, P_max=150.0, n_e_max=4000.0, i_1=4.70, i_6=0.75,
          eta_gb=0.95, eta_fd=0.96, mu=0.9, mu_y=0.8, f_r=0.015, v_max=170.0, grade=0.30, bump=3.0,
          clearance=210.0, R_turn=5.5, life_km=300000.0, z_p=12, z_c=47):
    """The brief. Masses in kg, lengths in mm, torque in N m, power in kW, speeds in rpm or km/h.
    m, m_r      gross vehicle mass and the gross load on the rear axle (assumed)
    L, h        wheelbase and the laden centre of gravity's height (assumed)
    r_roll      the tyre's rolling radius, r_stat its static loaded radius (assumed, a few per cent under the free one)
    T_e ...     the engine: 450 N m, 150 kW at 3600 rpm, 4000 rpm at most (assumed)
    i_1, i_6    the gearbox's first and sixth gears; eta_gb, eta_fd the gearbox's and the final drive's efficiency
    mu, mu_y    tyre-road friction, driving and cornering (dry asphalt, assumed); f_r rolling resistance (assumed)
    v_max       top speed (km/h) at the engine's maximum-power speed in sixth
    grade       the steepest start, laden, as a rise per unit run
    bump        the vertical load factor of a pothole (assumed)
    clearance   the least ground clearance under the axle, laden (mm)
    R_turn      the radius of the tightest turn, at the middle of the rear axle (m, assumed)
    life_km     the life of the axle (km, assumed)
    z_p, z_c    the pinion's and the crown wheel's teeth (chosen in duty())"""
    return dict(m=m, m_r=m_r, L=L, h=h, track=track, tyre_w=tyre_w, tyre_ar=tyre_ar, rim_in=rim_in, r_roll=r_roll,
                r_stat=r_stat, T_e=T_e, n_Pmax=n_Pmax, P_max=P_max, n_e_max=n_e_max, i_1=i_1, i_6=i_6,
                eta_gb=eta_gb, eta_fd=eta_fd, mu=mu, mu_y=mu_y, f_r=f_r, v_max=v_max, grade=grade, bump=bump,
                clearance=clearance, R_turn=R_turn, life_km=life_km, z_p=z_p, z_c=z_c)


def tyre(b):
    """The tyre 265/65 R17: section width 265 mm, sidewall 65 % of it, on a rim of 17 inches."""
    t = dict(rim=b['rim_in'] * 25.4, wall=b['tyre_ar'] / 100.0 * b['tyre_w'])
    t['D'] = t['rim'] + 2 * t['wall']
    t['r0'] = t['D'] / 2
    t['roll_pct'] = 100.0 * (1 - b['r_roll'] / t['r0'])
    return t


def duty(b):
    d = {}
    W = b['m'] * G                                         # N
    d['W'] = W
    d['W_r'] = b['m_r'] * G                                # static rear axle load, N
    d['W_f'] = W - d['W_r']
    d['a'] = b['L'] * b['m_r'] / b['m']                    # front axle to the centre of gravity, mm
    d['b'] = b['L'] - d['a']                               # centre of gravity to the rear axle, mm
    r = b['r_roll'] / 1000.0
    # the traction limit: F = mu (W a/L + F h/L)  ->  F = mu W (a/L) / (1 - mu h/L)
    d['aL'], d['hL'] = d['a'] / b['L'], b['h'] / b['L']
    d['F_trac'] = b['mu'] * W * d['aL'] / (1 - b['mu'] * d['hL'])
    d['W_r_dyn'] = d['W_r'] + d['F_trac'] * d['hL']        # the rear axle's load while it pulls its hardest
    d['T_trac'] = d['F_trac'] * r                          # N m, at the crown wheel (both wheels)
    # the final drive: the top speed at the maximum-power speed in sixth
    d['n_prop_top'] = b['n_Pmax'] / b['i_6']               # rpm
    v = b['v_max'] / 3.6                                   # m/s
    d['omega_w_top'] = v / r                               # rad/s
    d['n_w_top'] = d['omega_w_top'] * 60 / (2 * math.pi)   # rpm
    d['i0_max'] = d['n_prop_top'] / d['n_w_top']           # the largest ratio that still reaches v_max
    d['i0'] = b['z_c'] / b['z_p']
    d['v_top'] = d['n_prop_top'] / d['i0'] * 2 * math.pi / 60 * r * 3.6     # km/h actually reached
    # the engine limit, first gear, full torque
    d['T_pinion'] = b['T_e'] * b['i_1'] * b['eta_gb']      # N m into the pinion
    d['T_eng'] = d['T_pinion'] * d['i0'] * b['eta_fd']     # N m at the crown wheel
    d['T_design'] = min(d['T_eng'], d['T_trac'])
    d['T_shaft'] = d['T_design'] / 2                       # each half shaft (open differential)
    d['F_avail'] = d['T_eng'] / r                          # N at the road, first gear
    # the start on the grade: weight's component down the slope and rolling resistance
    th = math.atan(b['grade'])
    d['theta'] = math.degrees(th)
    d['F_grade'] = W * (math.sin(th) + b['f_r'] * math.cos(th))
    # on the grade the rear axle carries more: (W cos th a + W sin th h)/L at rest
    d['W_r_grade'] = W * (math.cos(th) * d['aL'] + math.sin(th) * d['hL'])
    d['mu_needed'] = d['F_grade'] / d['W_r_grade']
    # speeds
    d['n_pinion_max'] = b['n_e_max'] / b['i_6']            # rpm, the propeller shaft at the engine's top speed
    d['n_case_max'] = d['n_pinion_max'] / d['i0']
    # the tightest turn: the wheels on R -/+ track/2, the case at the mean
    R, t2 = b['R_turn'], b['track'] / 2000.0
    d['ratio_turn'] = (R + t2) / (R - t2)                  # outer to inner wheel speed
    d['rel_turn'] = t2 / R                                 # each side gear's speed in the case, per unit case speed
    # life
    d['rev_wheel'] = b['life_km'] * 1000.0 / (2 * math.pi * r)
    d['rev_pinion'] = d['rev_wheel'] * d['i0']
    return d


def load_cases(b):
    """The loads on one rear wheel at the road, N: vertical V, lateral S (along the axle), longitudinal X."""
    d = duty(b)
    t = b['track'] / 1000.0
    h = b['h'] / 1000.0
    Ws = d['W_r'] / 2                                      # static, per wheel
    c = {'static': dict(V=Ws, S=0.0, X=0.0)}
    c['bump'] = dict(V=b['bump'] * Ws, S=0.0, X=0.0)
    # cornering at mu_y: the rear axle's load moves to the outer wheel by W_r mu_y h / t
    dW = d['W_r'] * b['mu_y'] * h / t
    c['corner_out'] = dict(V=Ws + dW, S=b['mu_y'] * (Ws + dW), X=0.0)
    c['corner_in'] = dict(V=Ws - dW, S=b['mu_y'] * (Ws - dW), X=0.0)
    c['dW_corner'] = dW
    # braking at mu: the load moves forward; the rear wheels brake with mu times what they still carry
    W_r_brake = (d['W_r'] - b['mu'] * d['W'] * d['hL'])
    c['brake'] = dict(V=W_r_brake / 2, S=0.0, X=b['mu'] * W_r_brake / 2)
    c['W_r_brake'] = W_r_brake
    c['trac'] = dict(V=d['W_r_dyn'] / 2, S=0.0, X=d['F_trac'] / 2)
    return c


def envelope(b, wall=8.0, gap=6.0):
    """The ground clearance leaves the housing r_stat - clearance below the axle's centre; less the housing's wall
    and the gap to the crown wheel's teeth (assumed), that is the largest crown-wheel radius."""
    e = dict(below=b['r_stat'] - b['clearance'])
    e['r_crown_max'] = e['below'] - wall - gap
    e['d_crown_max'] = 2 * e['r_crown_max']
    return e


if __name__ == '__main__':
    b = brief()
    for k, v in tyre(b).items():
        print('tyre', k, round(v, 4))
    for k, v in duty(b).items():
        print('duty', k, round(v, 4))
    for k, v in load_cases(b).items():
        print('load', k, v if isinstance(v, dict) else round(v, 3))
    print('envelope', envelope(b))
