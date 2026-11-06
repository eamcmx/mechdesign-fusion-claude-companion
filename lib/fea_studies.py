"""Chapters 58, 59 and 62: the hand calculations the Simulation studies are set against, part by part, in the units
of the book (mm, N, MPa; Hz). The studies' parts come from scripts/StudyParts; their results are read from the files
Fusion writes (tools/sim_results.py) into results/sim_*.json.

Sources (Budynas & Nisbett, 2015, Shigley's Mechanical Engineering Design, 10th ed.):
  Table A-9 (p. 1023): a cantilever with an end load: sigma = M c / I, y_max = F l^3 / (3 E I)
  Table 7-1 (p. 365): first-iteration Kt and Kts: end-mill keyseat (r/d 0.02) 2.14 in bending, 3.0 in torsion;
    shoulder fillet, sharp (r/d 0.02) 2.7, well rounded (r/d 0.1) 1.7 in bending
  Fig. A-15-12 (p. 1037): a plate loaded in tension by a pin through a hole, sigma_0 = F / A, A = (w - d) t
  Eq. 3-37 (p. 116): tau_max = 16 T / (pi d^3) for a round bar in torsion
  Eq. 3-40 and Table 3-1 (pp. 118-119): a rectangle in torsion, J = beta a b^3
Fusion's material library, 'Steel': E 210 GPa, nu 0.3, rho 7850 kg/m^3 (read back from the solver's resultsAttribs).
The beam's natural frequencies: Euler-Bernoulli, f_n = (beta_n L)^2 / (2 pi L^2) sqrt(E I / (rho A)), beta_n L =
1.8751, 4.6941, 7.8548 for a cantilever; a free-fixed bar in torsion f = (2n - 1) / (4 L) sqrt(G J / (rho I_p)).
"""
import math

E_ST, NU_ST, RHO_ST = 210000.0, 0.3, 7850e-9          # MPa, -, kg/mm^3 (Fusion's 'Steel')
BETA_L = (1.8751, 4.6941, 7.8548)


# ---- Chapter 58: the calibration cantilever -----------------------------------------------------------------------------
CANT = dict(L=200.0, b=20.0, h=10.0, P=100.0)          # mm, mm, mm (the depth along the load), N


def cantilever(x=None):
    """The bending stress at the top face at x from the fixed end, and the tip's deflection."""
    c = CANT
    I = c['b'] * c['h'] ** 3 / 12.0
    out = dict(I=I, delta=c['P'] * c['L'] ** 3 / (3 * E_ST * I))
    xs = x if x is not None else (0.0, 20.0, 50.0, 100.0, 150.0)
    out['sigma'] = {xx: c['P'] * (c['L'] - xx) * (c['h'] / 2) / I for xx in xs}
    return out


def cantilever_modes():
    """Bending in each plane (thin: about the 10 mm depth; stiff: the 20 mm), torsion and the axial mode, Hz."""
    c = CANT
    L, b, h = c['L'] * 1e-3, c['b'] * 1e-3, c['h'] * 1e-3
    E, rho, G = E_ST * 1e6, RHO_ST * 1e9, E_ST * 1e6 / (2 * (1 + NU_ST))
    A = b * h
    out = {}
    for tag, I in (('thin', b * h ** 3 / 12), ('stiff', h * b ** 3 / 12)):
        k = math.sqrt(E * I / (rho * A * L ** 4))
        out[tag] = [bl ** 2 / (2 * math.pi) * k for bl in BETA_L]
    J = torsion_J(c['b'], c['h']) * 1e-12
    Ip = A * (b * b + h * h) / 12
    out['torsion'] = math.sqrt(G * J / (rho * Ip)) / (4 * L)
    out['axial'] = math.sqrt(E / rho) / (4 * L)
    return out


def torsion_J(a, b):
    """J of an a x b rectangle (a >= b), mm^4: J = a b^3 (1/3 - 0.21 (b/a)(1 - (b/a)^4 / 12)) (Roark), the series
    Shigley's Table 3-1 tabulates."""
    a, b = max(a, b), min(a, b)
    r = b / a
    return a * b ** 3 * (1.0 / 3 - 0.21 * r * (1 - r ** 4 / 12))


# ---- Chapter 58: the long stub's keyseat in torsion -------------------------------------------------------------------
KEYSEAT = dict(d=30.0, T=95.29510623542795, b=8.0, t1=4.0, L=25.0, r=0.6, Kts=3.0)   # Chapter 60's numbers


def keyseat():
    k = KEYSEAT
    tau = 16 * k['T'] * 1000.0 / (math.pi * k['d'] ** 3)
    return dict(tau=tau, vm_nominal=math.sqrt(3) * tau, tau_peak=k['Kts'] * tau, r_over_d=k['r'] / k['d'])


# ---- Chapter 59: a tine ---------------------------------------------------------------------------------------------------
TINE = dict(leg=(250.0, 25.0, 10.0), foot=(255.0, 25.0, 10.0), offset=(-115.0, -12.5))   # mm: leg L x a x b, foot
#   Ly x Lz x Lx, the foot's centre of mass from the leg's lower end (y, z)


def tine_modes():
    """The leg as a cantilever (its consistent mass), the foot a rigid body at its end: in the leg's plane two degrees
    of freedom (the end's sideways move and its slope); out of it three (the move, the slope, the leg's twist), Hz."""
    import numpy as np
    L, a, b = [v * 1e-3 for v in TINE['leg']]
    Ly, Lz, Lx = [v * 1e-3 for v in TINE['foot']]
    ry, rz = [v * 1e-3 for v in TINE['offset']]
    E, rho = E_ST * 1e6, RHO_ST * 1e9
    G = E / (2 * (1 + NU_ST))
    A = a * b
    ml, mf = rho * A * L, rho * Lx * Ly * Lz
    Ixx, Iyy, Izz = mf * (Ly ** 2 + Lz ** 2) / 12, mf * (Lx ** 2 + Lz ** 2) / 12, mf * (Lx ** 2 + Ly ** 2) / 12

    def kb(I):
        return E * I / L ** 3 * np.array([[12, -6 * L], [-6 * L, 4 * L * L]])
    Mb = ml / 420 * np.array([[156, -22 * L], [-22 * L, 4 * L * L]])

    def freqs(K, M):
        return sorted(math.sqrt(v) / (2 * math.pi) for v in np.linalg.eigvals(np.linalg.solve(M, K)).real)
    K = kb(b * a ** 3 / 12)
    M = Mb.copy()
    c = np.array([1.0, -rz])
    M += mf * np.outer(c, c)
    M[1, 1] += mf * ry ** 2 + Ixx
    inplane = freqs(K, M)
    J = torsion_J(a * 1e3, b * 1e3) * 1e-12
    Ip = A * (a * a + b * b) / 12
    K = np.zeros((3, 3))
    K[:2, :2] = kb(a * b ** 3 / 12)
    K[2, 2] = G * J / L
    M = np.zeros((3, 3))
    M[:2, :2] = Mb
    c = np.array([1.0, -rz, -ry])
    M += mf * np.outer(c, c)
    M[1, 1] += Iyy
    M[2, 2] += Izz + rho * Ip * L / 3
    outplane = freqs(K, M)
    leg_alone = 1.8751 ** 2 / (2 * math.pi) * math.sqrt(E * a * b ** 3 / 12 / (rho * A * L ** 4))
    return dict(inplane=inplane, outplane=outplane, m_leg=ml, m_foot=mf, leg_alone=leg_alone)


# ---- Chapter 62: the opening stop bracket's root, and a spar-fitting lug ---------------------------------------------------
BRACKET = dict(a=24.0, h=48.0, F=2179.963049455729, r=2.5)     # Chapter 56: the arm's side, the push's arm, the push


def bracket():
    k = BRACKET
    W = k['a'] ** 3 / (6 * math.sqrt(2))
    s = k['F'] * k['h'] / W
    return dict(W=W, sigma=s, drawn=1.7 * s, sharp=2.7 * s, r_over_d=k['r'] / k['a'])


LUG = dict(P=20848.421817117036 / 2, d=35.0, w=55.0, t=12.0, h=27.5, Kt=2.3)   # Chapter 34: half the ultimate push


def lug():
    k = LUG
    s0 = k['P'] / ((k['w'] - k['d']) * k['t'])
    return dict(sigma0=s0, bearing=k['P'] / (k['d'] * k['t']), d_over_w=k['d'] / k['w'], h_over_w=k['h'] / k['w'],
                peak=k['Kt'] * s0)


def _spandrel(r):
    """A concave fillet's cross-section, a square r x r less a quarter disc: its area and its centroid's distance
    from either face."""
    A = (1 - math.pi / 4) * r * r
    return A, r * (10 - 3 * math.pi) / (12 - 3 * math.pi)


def keyseat_volume(n=4000):
    """The keyseat part, mm^3: the 30 x 150 stub less the end-mill keyseat (a slot 8 x 25 with round ends, its floor
    11 mm above the axis, open to the round surface), the floor's fillet r 0.6 added back (Pappus on its spandrel)."""
    k = KEYSEAT
    R, half, rr = k['d'] / 2, k['b'] / 2, k['b'] / 2
    zf = R - k['t1']
    straight = k['L'] - k['b']                          # 17: between the round ends' centres
    cut = 0.0
    dy = 2 * half / n
    for i in range(n):                                  # across the slot: the slot's length at y, its depth to the round
        y = -half + (i + 0.5) * dy
        length = straight + 2 * math.sqrt(rr * rr - y * y)
        cut += length * (math.sqrt(R * R - y * y) - zf) * dy
    A, c = _spandrel(k['r'])
    fil = A * (2 * straight + 2 * math.pi * (rr - c))
    return math.pi * R * R * 150.0 - cut + fil


def bracket_volume(r=None):
    """The bracket part, mm^3: the foot 12 x 70 x 70 and the arm 24 x 24 x 48, with the root fillet r all round the
    arm (four straight runs of 24 and, at the arm's four edges, the spandrel turned through a quarter round)."""
    v = 12 * 70 * 70 + 24 * 24 * 48.0
    if r:
        A, c = _spandrel(r)
        v += A * 4 * 24 + 4 * A * (math.pi / 2) * c
    return v


VOLUMES = dict(cantilever=200 * 20 * 10.0, tine=250 * 25 * 10 + 255 * 25 * 10.0, bracket_sharp=bracket_volume(),
               bracket_r25=bracket_volume(BRACKET['r']),
               lug=100 * 55 * 12 + math.pi * 27.5 ** 2 / 2 * 12 - math.pi * 17.5 ** 2 * 12)
VOLUMES['keyseat'] = keyseat_volume()


if __name__ == '__main__':
    print(cantilever())
    print(cantilever_modes())
    print(keyseat())
    print(tine_modes())
    print(bracket())
    print(lug())
