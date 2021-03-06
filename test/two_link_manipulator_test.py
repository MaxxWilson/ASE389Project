import os
import sys
cwd = os.getcwd()
sys.path.append(cwd)
import math

import numpy as np
import pinocchio as pin
from pnc.robot_system.dart_robot_system import DartRobotSystem
from pnc.robot_system.pinocchio_robot_system import PinocchioRobotSystem

urdf_file = cwd + '/robot_model/manipulator/two_link_manipulator.urdf'
package_dir = cwd + '/robot_model/manipulator'

pin_robot_sys = PinocchioRobotSystem(urdf_file, package_dir, True, True)
dart_robot_sys = DartRobotSystem(urdf_file, True, True)


def analytic_jacobian(q):
    J = np.zeros((6, 2))

    J[3, 0] = -np.sin(q[0]) - np.sin(q[0] + q[1])
    J[3, 1] = -np.sin(q[0] + q[1])
    J[4, 0] = np.cos(q[0]) + np.cos(q[0] + q[1])
    J[4, 1] = np.cos(q[0] + q[1])
    J[2, 0] = 1.
    J[2, 1] = 1.

    return J


def analytic_jacobian_dot(q, qdot):
    Jdot = np.zeros((6, 2))

    Jdot[3, 0] = -np.cos(q[0]) * qdot[0] - np.cos(q[0] + q[1]) * (qdot[0] +
                                                                  qdot[1])
    Jdot[3, 1] = -np.cos(q[0] + q[1]) * (qdot[0] + qdot[1])

    Jdot[4, 0] = -np.sin(q[0]) * qdot[0] - np.sin(q[0] + q[1]) * (qdot[0] +
                                                                  qdot[1])
    Jdot[4, 1] = -np.sin(q[0] + q[1]) * (qdot[0] + qdot[1])

    return Jdot


# Arbitrary q and qdot
q = np.array([0.2, 0.1])
qdot = np.array([0.012323, 0.01])

# Update Robot
dart_robot_sys.update_system(None, None, None, None, None, None, None, None, {
    'j0': q[0],
    'j1': q[1]
}, {
    'j0': qdot[0],
    'j1': qdot[1]
})
pin_robot_sys.update_system(None, None, None, None, None, None, None, None, {
    'j0': q[0],
    'j1': q[1]
}, {
    'j0': qdot[0],
    'j1': qdot[1]
})

# Compare EE SE(3)
print("=" * 80)
print("EE SE3")
print("=" * 80)
print(dart_robot_sys.get_link_iso('ee'))
print(pin_robot_sys.get_link_iso('ee'))

# Compare jacobian
jac_pin = pin_robot_sys.get_link_jacobian('ee')
jac_dart = dart_robot_sys.get_link_jacobian('ee')
an_jac = analytic_jacobian(q)
print("=" * 80)
print("Jacobian")
print("=" * 80)
print("jacobian from pinocchio")
print(jac_pin)
print("jacobian from dart")
print(jac_dart)
print("analytic jacobian")
print(an_jac)

jodt_qdot_pin_classic = pin_robot_sys.get_link_jacobian_dot_times_qdot('ee')
jodt_qdot_dart_classic = dart_robot_sys.get_link_jacobian_dot_times_qdot('ee')
an_jac_dot = analytic_jacobian_dot(q, qdot)
jdot_qdot_an = np.dot(an_jac_dot, qdot)
print("=" * 80)
print("Jacobian Dot * Q Dot")
print("=" * 80)
print("classical jdotqdot from pinocchio")
print(jodt_qdot_pin_classic)
print("classical jdotqdot from dart")
print(jodt_qdot_dart_classic)
print("analytic jdotqdot")
print(jdot_qdot_an)
