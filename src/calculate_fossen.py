#! /bin/usr/python3

import math

r = 0.1111
h = 0.59274

a = h / 2
b = r
m = 23.9

e = 1 - math.pow(b/a,2)

one_minus_e_squared_over_e_to_the_three = (1 - math.pow(e,2))/math.pow(e,3)
one_plus_e_one_minus_e = (1 + e) / (1 - e)
ln = math.log(one_plus_e_one_minus_e)
a0 = 2*one_minus_e_squared_over_e_to_the_three * (ln/2 -e)

one_over_e_squared = 1 / math.pow(e,2)
b0 = one_over_e_squared - one_minus_e_squared_over_e_to_the_three*ln/2


xu = -a0/(2-a0) * m 
yu = -b0 / (2- b0) *m
zw = yu
kp = 0
b_squared_minus_a_squared = math.pow(b,2) - math.pow(a,2)
b_squared_plus_a_squared = math.pow(b,2) + math.pow(a,2)

nr = (-m/5)*( math.pow(b_squared_minus_a_squared,2) * (a0 - b0) ) / (2*(b_squared_minus_a_squared) + b_squared_plus_a_squared*(b0-a0))
mq = nr


r_squared = math.pow(r,2)
h_squared = math.pow(h,2)
ixx = m * r_squared / 2
iyy = m * (3 * r_squared + h_squared) / 12
izz = iyy

print("Moment of inertia matrix:\n")
print("ixx: ", ixx)
print("iyy: ", iyy)
print("izz:, ", izz)


print("\nAdded mass values:\n")
print("XdotU: ", xu)
print("\nYdotV: ", yu)
print("\nZdotW: ", zw)
print("\nKdotP : ", 0)
print("\nNdotR: ", nr)
print("\nMdotQ: ", mq)