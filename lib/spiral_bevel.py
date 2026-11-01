# =============================================================================
# spiral_bevel.py  -  the teeth of a spiral bevel pair at concept level (Volume 2, Chapter 29). No Fusion here.
#
# A gear's own frame: the cone's apex at the origin, its axis +z, the heel at the outer cone distance Ae.
# A tooth is lofted through transverse sections at cone distances from the toe (Ae - F) to the heel (Ae). Each
# section is Tredgold's virtual spur tooth on the back cone (Volume 1, Chapter 17), drawn with the transverse
# module m_et R/Ae and the transverse pressure angle atan(tan(alpha_n)/cos(beta)), as for a helical gear's transverse
# section (Volume 1, Chapter 14). The section is turned about the axis by
#     phi(R) = -/+ (tan(beta) / sin(delta)) ln(R / Rm),
# the turn that keeps the tooth on a logarithmic spiral of constant spiral angle beta on the crown gear: a point that
# moves by d theta on the crown gear moves by d theta / sin(delta) on the gear's own pitch cone. The two members carry
# opposite signs, opposite hands; with hand = +1 the pinion is left-handed and the gear right-handed (Childs, 2014,
# Section 10.2: right-hand teeth incline away from the axis clockwise, seen from the small end).
# Real spiral bevel teeth are cut on a circular arc, whose spiral angle varies along the face; the logarithmic
# spiral is the concept-level stand-in with the mean spiral angle everywhere.
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math


def inv(a):
    return math.tan(a) - a


def member(z, z_mate, m_et, F, beta=35.0, alpha_n=20.0, backlash=0.2, pinion=True, hand=+1):
    """One member of the pair: its cones, its virtual tooth at the heel and its spiral."""
    g = dict(z=z, z_mate=z_mate, m=m_et, F=F, beta=math.radians(beta), alpha_n=math.radians(alpha_n))
    g['delta'] = math.atan(z / z_mate)
    g['Ae'] = 0.5 * m_et * math.hypot(z, z_mate)
    g['Ai'] = g['Ae'] - F
    g['Rm'] = g['Ae'] - F / 2.0
    g['k'] = g['Ai'] / g['Ae']                            # the toe as a fraction of the heel
    g['alpha_t'] = math.atan(math.tan(g['alpha_n']) / math.cos(g['beta']))
    g['ha'] = m_et * math.cos(g['beta'])                  # at the heel
    g['hf'] = 1.25 * g['ha']
    # the virtual (back-cone) spur gear at the heel
    g['zv'] = z / math.cos(g['delta'])
    g['rv'] = g['zv'] * m_et / 2.0
    g['rbv'] = g['rv'] * math.cos(g['alpha_t'])
    g['rav'] = g['rv'] + g['ha']
    g['rfv'] = g['rv'] - g['hf']
    g['half_v'] = math.pi / (2 * g['zv']) - backlash / 2.0 / g['rv'] / 2.0   # half the tooth's angle, backlash shared
    g['sign'] = -hand if pinion else hand
    g['pinion'] = pinion
    return g


def phi(g, R):
    """The turn of the tooth's section at cone distance R about the gear's axis, radians."""
    return g['sign'] * math.tan(g['beta']) / math.sin(g['delta']) * math.log(R / g['Rm'])


def tooth_uv(g, n_flank=7, n_tip=3, extra=1.0):
    """The virtual tooth at the heel, (u, v): u across the tooth, v outward from the virtual pitch circle; closed,
    its foot `extra` mm below the root so that the loft overlaps the blank."""
    rv, rb, ra, rf, a = g['rv'], g['rbv'], g['rav'], g['rfv'], g['alpha_t']
    ph = lambda rho: g['half_v'] + inv(a) - inv(math.acos(min(1.0, rb / rho)))
    lo = max(rf, rb)
    right = []
    for i in range(n_flank + 1):
        rho = lo + (ra - lo) * i / n_flank
        p = ph(rho)
        right.append((rho * math.sin(p), rho * math.cos(p) - rv))
    pt = ph(ra)
    tip = [(ra * math.sin(pt - 2 * pt * i / (n_tip + 1)), ra * math.cos(pt - 2 * pt * i / (n_tip + 1)) - rv)
           for i in range(1, n_tip + 1)]
    left = [(-u, v) for (u, v) in reversed(right)]
    p0, rb_ = ph(lo), rf - extra
    bottom = [(-rb_ * math.sin(p0), rb_ * math.cos(p0) - rv), (rb_ * math.sin(p0), rb_ * math.cos(p0) - rv)]
    return right + tip + left + bottom


def section(g, s, phase=0.0, uv=None):
    """The tooth's section at R = s Ae in the gear's own frame: the heel section on its back cone, scaled by s
    towards the apex and turned by phi(R) + phase about z."""
    d = g['delta']
    sd, cd = math.sin(d), math.cos(d)
    R = s * g['Ae']
    ang = phi(g, R) + phase
    c, sn = math.cos(ang), math.sin(ang)
    out = []
    for (u, v) in (uv or tooth_uv(g)):
        x, y, z = g['Ae'] * sd + v * cd, u, g['Ae'] * cd - v * sd       # the heel's back-cone plane
        x, y, z = s * x, s * y, s * z
        out.append((x * c - y * sn, x * sn + y * c, z))
    return out


def scales(g, n=7):
    return [g['k'] + (1 - g['k']) * i / (n - 1) for i in range(n)]


def pitch_point(g, R, phase=0.0):
    """Where tooth 0's middle crosses the pitch cone at cone distance R, in the gear's own frame."""
    ang = phi(g, R) + phase
    rr = R * math.sin(g['delta'])
    return (rr * math.cos(ang), rr * math.sin(ang), R * math.cos(g['delta']))


# The pair placed as in the axle: the common apex at the origin, the crown wheel's axis along -Y (it lies on the left
# of the pinion's axis), the pinion's axis along +X (towards the front). Each gear's own x axis points to the line of
# contact, so tooth 0 of the pinion and the gap of the crown wheel meet there at the mean cone distance.
PINION_FRAME = ((0.0, -1.0, 0.0), (0.0, 0.0, -1.0), (1.0, 0.0, 0.0))   # own x, y, z in world
CROWN_FRAME = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))


def to_world(frame, p):
    ex, ey, ez = frame
    return tuple(p[0] * ex[i] + p[1] * ey[i] + p[2] * ez[i] for i in range(3))


def rolled_mismatch(gp, gc, n=11):
    """Roll the pair until tooth 0's section at cone distance R reaches the line of contact on the pinion (a turn of
    -phi_p(R)); the crown wheel turns with it by -(z_p/z_c) times that, as rolling pitch cones do. Where the hands are
    right, the crown wheel's tooth 0 section at R arrives there too: the largest distance between the two pitch points,
    over the face, is then zero. Turns are about each gear's own z axis."""
    worst = 0.0
    for i in range(n):
        R = gp['Ai'] + (gp['Ae'] - gp['Ai']) * i / (n - 1)
        t1 = -phi(gp, R)
        t2 = -gp['z'] / gc['z'] * t1
        a = to_world(PINION_FRAME, pitch_point(gp, R, t1))
        b = to_world(CROWN_FRAME, pitch_point(gc, R, t2))
        worst = max(worst, math.dist(a, b))
    return worst


if __name__ == '__main__':
    gp = member(12, 47, 6.5, 52.0, pinion=True)
    gc = member(47, 12, 6.5, 52.0, pinion=False)
    print('pinion delta %.4f deg, crown %.4f; Ae %.3f, Rm %.3f, k %.4f' % (math.degrees(gp['delta']),
          math.degrees(gc['delta']), gp['Ae'], gp['Rm'], gp['k']))
    print('alpha_t %.3f deg; pinion zv %.3f; transverse undercut limit %.3f' % (
        math.degrees(gp['alpha_t']), gp['zv'], 2 * math.cos(gp['beta']) / math.sin(gp['alpha_t']) ** 2))
    for g, name in ((gp, 'pinion'), (gc, 'crown')):
        print(name, 'phi toe %.3f deg, heel %.3f deg' % (math.degrees(phi(g, g['Ai'])), math.degrees(phi(g, g['Ae']))))
    print('rolled pitch-point mismatch, opposite hands: %.2e mm' % rolled_mismatch(gp, gc))
    gc_same = member(47, 12, 6.5, 52.0, pinion=False, hand=-1)
    print('rolled pitch-point mismatch, same hands (wrong): %.3f mm' % rolled_mismatch(gp, gc_same))
