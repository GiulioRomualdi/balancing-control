# This software may be modified and distributed under the terms of the BSD-3-Clause license.

import idyntree.swig as idyn
import numpy as np


def evaluate_local_zmp(wrench):
    tau_x = wrench[3]
    tau_y = wrench[4]
    f_z = wrench[2]
    if f_z > 0.1:
        return [-tau_y / f_z, tau_x / f_z, 0.]
    return [0., 0., 0.]


def evaluate_global_zmp(left_wrench, right_wrench, kindyn: idyn.KinDynComputations, l_sole_frame, r_sole_frame):
    left_zmp = idyn.Position(np.array([0., 0., 0]))
    zmp_left_defined = 0
    if left_wrench[2] > 0.1:
        left_zmp = idyn.Position(evaluate_local_zmp(left_wrench))
        zmp_left_defined = 1

    right_zmp = idyn.Position(np.array([0., 0., 0]))
    zmp_right_defined = 0
    if right_wrench[2] > 0.1:
        right_zmp = idyn.Position(evaluate_local_zmp(right_wrench))
        zmp_right_defined = 1

    total_z = right_wrench[2] * zmp_right_defined + left_wrench[2] * zmp_left_defined

    inertial_zmp_left = kindyn.getWorldTransform(l_sole_frame) * left_zmp
    inertial_zmp_right = kindyn.getWorldTransform(r_sole_frame) * right_zmp

    inertial_global_zmp = left_wrench[2] * zmp_left_defined * inertial_zmp_left.toNumPy() / total_z + \
                          right_wrench[2] * zmp_right_defined * inertial_zmp_right.toNumPy() / total_z

    return inertial_global_zmp
