import sympy as sp
from numpy import pi
from forward_kinematics import UR5ForwardKinematics


FK = UR5ForwardKinematics()
# The th_2 value causes problems
arm_angle = [0.6816, 0.7316, 0.6816, 0.7316, 0.6816, 0.7316]
arm_angle = [0.7853, 0.7316, 0, 0, 0, 0]
# arm_angle = [0.2474, 0.9689, 0.2474, 0.9689, 0.2474, 0.9689]
# arm_angle = [0.2474, pi/4.0, 0.2474, 0.9689, 0.2474, 0.9689]

vector = FK.return_vector_to_reference_frame(FK.B02, arm_angle)
sp.pprint(vector)