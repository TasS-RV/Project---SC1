import numpy as np
from numpy.linalg import inv

#Grip-limited scenario
F_grip = []
#Tilt telt scenario
F_tilt = [0, 3031, 1750, 0]
#Dive - hard braking
F_dive = []
#Bump hit
F_bump = []


#Define Suspension dimensions
r = 266.7
a = 120
b = 110
c = 70
d = 77.48

#External reaction forces - through rigid wishbone + hib + rim/tire body

#*Reaction_Fsus = [F2y, F3y, F4y, F3x, F4x, F4z] #6 variable unknown function - use for mapping forces from "resolve" function output

F_A = ([0,0,0, (r+a)/r,(r-b)/r,0],
[c ,0 ,0 , d, d, 0],
[1, 1, 1, 0, 0, 0],
[0 ,0 ,0 ,0 ,0 ,1],
[0,0,0, -a/r, b/r, 0],
[-(r+a)/d, -(r+a)/d, -(r-b)/d, 0, 0, 1])

F_Ainv = inv(F_A)
#*Reaction_Fsus = np.dot(F_Ainv, F_ext)

def resolve(forces_external, F_Ainv):
    F1x = forces_external[0]
    F0y = forces_external[1]
    F0z = forces_external[2]
    F0x = forces_external[3]
    #Capture external loads for varying driving situations
    F_ext = [-1*F1x, 0, -1*F0y, -1*F0z, -F0x, 0]
    Reaction_Fsus  = np.dot(F_Ainv, F_ext)
    return Reaction_Fsus

load_models = {"brake_dive": F_dive, "tilt_test": F_tilt , "max_lateral": F_grip, "Bump_load": F_bump}
#Test_case with trivial lateral load transfer and tilt test case
test_load_models = {"tilt": F_tilt}

for key in test_load_models:
    print("Load Case:{}, Forces Matrix: {}".format(key, resolve(test_load_models[key], F_Ainv)))
    #Captures the matrix of F1x, F0x, F0z, F0y matrix inversion is applied to resolve for unknown upright rigid body loads

    
