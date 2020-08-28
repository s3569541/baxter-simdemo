#!/usr/bin/python

# Cuboid
# https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors

# Adapt cuboid for cube, where s(ide) == d == h == w == 0.05 and m==0.01, and therefore all nonzero (diagonal) elements == 1/12 * m * (2*s*s)

# side 4cm
s = 0.04
# mass 10g
m = 0.01

print (2*m*s*s) / 12
