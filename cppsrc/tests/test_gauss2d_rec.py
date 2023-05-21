from scipy.stats import multivariate_normal
from scipy.integrate import quad, dblquad
import scipy.special as special
import numpy as np
import math
import quadpy

scheme = quadpy.t2.get_good_scheme(10)
def I(mu_x, mu_y, sigma, xa, ya, sz):
    rv = multivariate_normal([mu_x, mu_y], [[sigma, 0], [0, sigma]])
    f = lambda y, x: rv.pdf([x,y])
    return dblquad(f, xa, xa + sz, ya, ya + sz)

def I1(xa, ya):
    return special.erfc(xa/math.sqrt(2)) * special.erfc(ya/math.sqrt(2)) / 4

def I2(mu_x, mu_y, sigma, xa, ya, sz):
    xap = (xa - mu_x) / sigma
    yap = (ya - mu_y) / sigma
    szp = sz / sigma
    return I1(xap, yap) - I1(xap + szp, yap) - I1(xap, yap + szp) + I1(xap + szp, yap + szp)

mu_x_ = 1
mu_y_ = 0.5
sigma_ = 3
xa_ = 1
ya_ = 0
sz_ = np.inf

print(I(mu_x_, mu_y_, sigma_, xa_, ya_, sz_)[0])
print(I(mu_x_, mu_y_, sigma_, xa_, ya_, sz_)[1])
print(I2(mu_x_, mu_y_, sigma_, xa_, ya_, sz_))
