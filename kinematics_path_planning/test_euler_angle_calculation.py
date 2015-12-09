from effector_pose import EffectorPose
from forward_kinematics import UR5ForwardKinematics
from inverse_kinematics import InverseKinematics

FK = UR5ForwardKinematics()
IK = InverseKinematics()
ep = EffectorPose(0.4, 0, 0,
                  0.75, -1.1415, -0.111111)
joints = IK.calculate_joints(ep)
angles = FK.effectorEulerAngles(joints[0])
print('Phi')
print(angles[0])
print('Theta')
print(angles[1])
print('Psi')
print(angles[2])
