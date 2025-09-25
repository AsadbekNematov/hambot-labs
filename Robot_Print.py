import math

#Constants
R_WHEEL_M    = 0.045    
AXLE_LEN_M   = 0.184    
FT2M         = 0.3048

#Waypoints 
WPTS = [
    (  2.0,  -2.0,  math.pi    ),   # P0
    ( -1.5,  -2.0,  math.pi    ),   # P1
    ( -2.0,  -1.5,  math.pi/2  ),   # P2
    ( -2.0,  -0.5,  math.pi/2  ),   # P3
    ( -1.0,  -0.5,  3*math.pi/2),   # P4
    ( -0.5,  -1.0,  7*math.pi/4),   # P5
    (  2.0,  -1.0,  0.0        ),   # P6
    (  2.0,   0.0,  math.pi/2  ),   # P7
    (  0.0,   0.0,  math.pi    ),   # P8
    (  0.0,   1.0,  math.pi/2  ),   # P9
    ( -2.0,   1.0,  math.pi    ),   # P10
    ( -1.0,   2.0,  0.0        ),   # P11
    (  1.5,   2.0,  0.0        ),   # P12
    (  2.5,   2.0,  0.0        ),   # P13
]

#Helper Functions 
def straight_distance(i, j, v):
    x1, y1, _ = WPTS[i]
    x2, y2, _ = WPTS[j]
    d_ft = math.hypot(x2 - x1, y2 - y1)
    d_m = d_ft * FT2M
    t = d_m / v
    return d_m, t

def arc_distance(R_m, Theta, v):
    D = R_m * Theta
    t = D / v
    return D, t

#Segment Functions 
def p0_to_p1():
    v = 0.5
    d, t = straight_distance(0, 1, v)
    print(f"P0 to P1: d={d:.3f} m, v={v:.2f} m/s, t={t:.2f} s")
    return d, t

def p1_to_p2():
    dx_ft = WPTS[2][0] - WPTS[1][0]
    dy_ft = WPTS[2][1] - WPTS[1][1]
    R_ft = math.hypot(dx_ft, dy_ft) / math.sqrt(2.0)
    R_m = R_ft * FT2M
    Theta = math.pi/2
    v = 0.4
    d, t = arc_distance(R_m, Theta, v)
    print(f"P1 to P2: R={R_m:.3f} m, Θ=90°, v={v:.2f}, D={d:.3f}, t={t:.2f}")
    return d, t

def p2_to_p3():
    v = 0.5
    d, t = straight_distance(2, 3, v)
    print(f"P2 to P3: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p3_to_p4():
    R_ft = 0.5
    R_m = R_ft * FT2M
    Theta = math.pi
    v = 0.4
    d, t = arc_distance(R_m, Theta, v)
    print(f"P3 to P4: R={R_m:.3f}, Θ=180°, v={v:.2f}, D={d:.3f}, t={t:.2f}")
    return d, t

def p4_to_p5():
    v = 0.5
    d, t = straight_distance(4, 5, v)
    print(f"P4 to P5: d={d:.3f} m, v={v:.2f}, t={t:.2f} (ignores turn time)")
    return d, t

def p5_to_p6():
    v = 0.5
    d, t = straight_distance(5, 6, v)
    print(f"P5 to P6: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p6_to_p7():
    v = 0.5
    d, t = straight_distance(6, 7, v)
    print(f"P6 to P7: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p7_to_p8():
    v = 0.5
    d, t = straight_distance(7, 8, v)
    print(f"P7 to P8: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p8_to_p9():
    v = 0.5
    d, t = straight_distance(8, 9, v)
    print(f"P8 to P9: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p9_to_p10():
    v = 0.5
    d, t = straight_distance(9, 10, v)
    print(f"P9 to P10: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p10_to_p11():
    dx_ft = WPTS[11][0] - WPTS[10][0]
    dy_ft = WPTS[11][1] - WPTS[10][1]
    R_ft = math.hypot(dx_ft, dy_ft) / math.sqrt(2.0)
    R_m = R_ft * FT2M
    Theta = math.pi/2
    v = 0.4
    d, t = arc_distance(R_m, Theta, v)
    print(f"P10 to P11: R={R_m:.3f}, Θ=90°, v={v:.2f}, D={d:.3f}, t={t:.2f}")
    return d, t

def p11_to_p12():
    v = 0.5
    d, t = straight_distance(11, 12, v)
    print(f"P11 to P12: d={d:.3f} m, v={v:.2f}, t={t:.2f}")
    return d, t

def p12_to_p13():
    VR = 0.24
    VL = 0.80
    T  = 0.5
    L  = AXLE_LEN_M

    v = 0.5 * (VR + VL)
    w = (VR - VL) / L
    R = v / w
    theta = w * T
    D = v * T

    print("P12 to P13:")
    print(f"VR={VR:.2f}, VL={VL:.2f}")
    print(f"v={v:.3f}, w={w:.3f}, R={R:.3f}")
    print(f"θ={math.degrees(theta):.2f}°, D={D:.3f}, T={T:.2f}")
    return D, T

def main():
    total_d = 0.0
    total_t = 0.0

    for function in [p0_to_p1, p1_to_p2, p2_to_p3, p3_to_p4,
               p4_to_p5, p5_to_p6, p6_to_p7, p7_to_p8,
               p8_to_p9, p9_to_p10, p10_to_p11, p11_to_p12, p12_to_p13]:
        d, t = function()
        total_d += d
        total_t += t

    print(f"Total Path Length = {total_d:.3f} m")
    print(f"Total Travel Time = {total_t:.2f} s")

if __name__ == "__main__":
    main()
