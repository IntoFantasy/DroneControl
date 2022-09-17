import math
import sympy

x, y = sympy.symbols('x, y')
x2, y2 = -0.4999999999999998, 0.8660254037844387
x3, y3 = 0.17364817766692997, -0.9848077530122081
R = 1
answer = sympy.nonlinsolve([sympy.Eq(((x-x3) ** 2 + (y-y3) ** 2+x**2+y**2 - R**2) /
                                     (2 * sympy.sqrt((x-x3) ** 2 + (y-y3) ** 2) * sympy.sqrt(x**2+y**2)), 0.866),
                            sympy.Eq(((x - x2) ** 2 + (y - y2) ** 2 + x ** 2 + y ** 2 - R ** 2) /
                                     (2 * sympy.sqrt((x - x2) ** 2 + (y - y2) ** 2) * sympy.sqrt(x ** 2 + y ** 2)),
                                     )], [x, y])
print(answer)
