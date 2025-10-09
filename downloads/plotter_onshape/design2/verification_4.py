import cmath
import math

# --- Solution 1 ---
def t1_sol1(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol1(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

# --- Solution 2 ---
def t1_sol2(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol2(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

# --- Solution 3 ---
def t1_sol3(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol3(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

# --- Solution 4 ---
def t1_sol4(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol4(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

x, y = 400, 430

solutions = [
    (t1_sol1, t2_sol1),
    (t1_sol2, t2_sol2),
    (t1_sol3, t2_sol3),
    (t1_sol4, t2_sol4)
]

for i, (f_t1, f_t2) in enumerate(solutions, 1):
    t1 = f_t1(x, y)
    t2 = f_t2(x, y)
    print(f"Solution {i}:")
    print(f"  t1 (rad): {t1}")
    print(f"  t1 (deg): {math.degrees(t1.real):.4f}")
    print(f"  t2 (rad): {t2}")
    print(f"  t2 (deg): {math.degrees(t2.real):.4f}")
    print(f"  t1 imaginary part: {t1.imag:.4e}, t2 imaginary part: {t2.imag:.4e}")
    print("-"*30)
