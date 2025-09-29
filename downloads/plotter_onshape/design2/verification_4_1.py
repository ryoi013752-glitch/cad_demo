import cmath
import math

# 你的四組 solution (複製自你的定義)
def t1_sol1(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol1(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

def t1_sol2(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol2(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

def t1_sol3(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol3(x, y): 
    return 2*cmath.atan((640*y - cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

def t1_sol4(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 - 2*x**2*y**2 + 409600*x**2 - y**4 + 409600*y**2))/(x**2 + 640*x + y**2))
def t2_sol4(x, y): 
    return 2*cmath.atan((640*y + cmath.sqrt(-x**4 + 1600*x**3 - 2*x**2*y**2 - 550400*x**2 + 1600*x*y**2 - 71680000*x - y**4 + 89600*y**2 + 39936000000))/(x**2 - 1440*x + y**2 + 416000))

x, y = 430, 400

solutions = [
    (t1_sol1, t2_sol1),
    (t1_sol2, t2_sol2),
    (t1_sol3, t2_sol3),
    (t1_sol4, t2_sol4)
]

for i, (f_t1, f_t2) in enumerate(solutions, 1):
    t1 = f_t1(x, y).real
    t2 = f_t2(x, y).real
    t1_deg = math.degrees(t1)
    t2_deg = math.degrees(t2)
    print(f"Solution {i}: t1 = {t1_deg:.4f}°, t2 = {t2_deg:.4f}°")
    print(f"  Both < 90° ? {'Yes' if 0 <= t1_deg < 90 and 0 <= t2_deg < 90 else 'No'}")
    print('-'*30)
