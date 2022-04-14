# from sympy import *

# Robot Chassis Parameters
l = 370       #hip to hip length of the robot
b = 210.1     #hip to hip breadth of the robot
h = 44        #height of the robot

## Leg Type 1: Rear

'''
Variable name convention as follows:

The first number represents the length and the second number represents the Leg Type

l11 is the hip to knee length of Leg Type 1
l21 is the knee to ankle length of Leg Type 1

l22 is the knee to ankle length of Leg Type 2
and so on...

'''

# Defining lengths and offsets
l11 = 160      #hip to knee length
# l21 = 160      #knee to ankle length
l21 = 173.5    #knee to point contact
l3 = 39        #ankle to toe length
d1 = 37        #hip offset
d2 = 12.95     #knee offset

'''
Variable name convention as follows:

The first number represents the angle and the second number represents the Leg #

theta11 is the hip rotation angle of Leg 1
theta21 is the knee roation angle of Leg 1
theta31 is the ankle roation angle of Leg 1

theta14 is the hip rotation angle of Leg 4
and so on...

'''

# theta11, alpha11, theta21, alpha21, theta31, alpha31 = symbols("theta11 alpha11 theta21 alpha21 theta31 alpha31")
# theta14, alpha14, theta24, alpha24, theta34, alpha34 = symbols("theta14 alpha14 theta24 alpha24 theta34 alpha34")


## Leg Type 2: Front

# Defining lengths and offsets
l12 = 160      #hip to knee length
l22 = 173.5    #knee to point contact

# theta12, alpha12, theta22, alpha22 = symbols("theta12 alpha12 theta22 alpha22")
# theta13, alpha13, theta23, alpha23 = symbols("theta13 alpha13 theta23 alpha23")
