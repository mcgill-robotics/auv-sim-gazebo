#! /bin/usr/python3

import math

a = 0.5588
b = 0.10478
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

print("Xu : ", xu)
print("\nyv: ", yu)
print("\nzw: ", zw)
print("\nkp : ", 0)
print("\nnr: ", nr)
print("\nmq: ", mq)