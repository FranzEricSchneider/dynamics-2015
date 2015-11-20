import sympy as sp
from forward_kinematics import UR5ForwardKinematics

FK = UR5ForwardKinematics()
arm_angle = [0.2474, 0.9689, 0.2474, 0.9689, 0.2474, 0.9689]

print(FK.B06[1, 1])

# vector = FK.return_vector_to_reference_frame(FK.B02, arm_angle)
# sp.pprint(vector)