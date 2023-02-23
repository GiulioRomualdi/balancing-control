# This software may be modified and distributed under the terms of the BSD-3-Clause license.

import os
import time
import numpy as np

import bipedal_locomotion_framework.bindings as blf
import yarp
import idyntree.swig as idyn

import manifpy as manif

from robot_basic_motions.wbc import WBC
from robot_basic_motions.zmp import evaluate_local_zmp, evaluate_global_zmp


def build_remote_control_board_driver(param_handler: blf.parameters_handler.IParametersHandler, local_prefix: str):
    param_handler.set_parameter_string("local_prefix", local_prefix)
    return blf.robot_interface.construct_remote_control_board_remapper(param_handler)


def build_contact_wrench_driver(param_handler: blf.parameters_handler.IParametersHandler, local_prefix: str):
    param_handler.set_parameter_string("local_prefix", local_prefix)
    return blf.robot_interface.construct_generic_sensor_client(param_handler)


def build_contact_wrenches_driver(params_contact_wrenches: blf.parameters_handler.IParametersHandler,
                                  local_prefix: str):
    # build contact wrenches polydrivers
    contact_wrenches_drivers = dict()

    for wrench_name in params_contact_wrenches.get_parameter_vector_string("left_contact_wrenches_group"):
        contact_wrenches_drivers[wrench_name] = build_contact_wrench_driver(
            params_contact_wrenches.get_group(wrench_name), local_prefix)
        assert contact_wrenches_drivers[wrench_name].is_valid()

    for wrench_name in params_contact_wrenches.get_parameter_vector_string("right_contact_wrenches_group"):
        contact_wrenches_drivers[wrench_name] = build_contact_wrench_driver(
            params_contact_wrenches.get_group(wrench_name), local_prefix)
        assert contact_wrenches_drivers[wrench_name].is_valid()

    return contact_wrenches_drivers


def build_kin_dyn(param_handler):
    rf = yarp.ResourceFinder()
    robot_model_path = rf.findFile("model.urdf")
    joint_list = param_handler.get_group("ROBOT_CONTROL").get_parameter_vector_string("joints_list")
    ml = idyn.ModelLoader()
    ml.loadReducedModelFromFile(robot_model_path, joint_list)

    kindyn = idyn.KinDynComputations()
    kindyn.loadRobotModel(ml.model())
    return kindyn


def get_base_frame(base_frame: str, kindyn: idyn.KinDynComputations):
    frame_base_index = kindyn.model().getFrameIndex(base_frame)
    link_base_index = kindyn.model().getFrameLink(frame_base_index)
    base_frame_name = kindyn.model().getLinkName(link_base_index)
    return base_frame_name, kindyn.getRelativeTransform(frame_base_index, link_base_index)


def create_new_spline(knots_positions, motion_duration, dt):
    com_spline = blf.planners.QuinticSpline()
    com_spline.set_initial_conditions([0, 0, 0], [0, 0, 0])
    com_spline.set_final_conditions([0, 0, 0], [0, 0, 0])
    com_spline.set_advance_time_step(dt)
    com_spline.set_knots(knots_positions, [0, motion_duration], )
    return com_spline


def main():
    param_handler = blf.parameters_handler.YarpParametersHandler()

    robot_name = os.environ["YARP_ROBOT_NAME"]
    param_handler.set_from_file_path('./config/' + robot_name + '/balancing.ini')

    dt = param_handler.get_parameter_float("dt")

    kindyn = build_kin_dyn(param_handler=param_handler)
    kindyn_with_measured = build_kin_dyn(param_handler=param_handler)

    # Create the polydrivers
    poly_drivers = build_contact_wrenches_driver(params_contact_wrenches=param_handler.get_group("CONTACT_WRENCHES"),
                                                 local_prefix="balancing_controller")
    poly_drivers["REMOTE_CONTROL_BOARD"] = build_remote_control_board_driver(
        param_handler=param_handler.get_group("ROBOT_CONTROL"),
        local_prefix="balancing_controller")
    assert poly_drivers["REMOTE_CONTROL_BOARD"].is_valid()

    # just to wait that everything is in place
    time.sleep(2)

    robot_control = blf.robot_interface.YarpRobotControl()
    assert robot_control.initialize(param_handler.get_group("ROBOT_CONTROL"))
    assert robot_control.set_driver(poly_drivers["REMOTE_CONTROL_BOARD"].poly)

    # Create the sensor bridge
    sensor_bridge = blf.robot_interface.YarpSensorBridge()
    assert sensor_bridge.initialize(param_handler.get_group("SENSOR_BRIDGE"))
    assert sensor_bridge.set_drivers_list(list(poly_drivers.values()))

    # set the kindyn computations with the robot state
    assert sensor_bridge.advance()
    are_joint_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
    assert are_joint_ok

    base_frame = param_handler.get_parameter_string("base_frame")
    base_link, frame_T_link = get_base_frame(base_frame, kindyn=kindyn)

    right_contact_frame = param_handler.get_parameter_string("right_contact_frame")
    left_contact_frame = param_handler.get_parameter_string("left_contact_frame")

    assert kindyn.setFloatingBase(base_link)
    gravity = [0., 0., -blf.math.StandardAccelerationOfGravitation]
    base_velocity = idyn.Twist()
    base_velocity.zero()
    joint_velocity = joint_positions * 0
    assert kindyn.setRobotState(frame_T_link, joint_positions, base_velocity, joint_velocity, gravity)
    initial_com_position = kindyn.getCenterOfMassPosition().toNumPy()

    assert kindyn_with_measured.setFloatingBase(base_link)
    assert kindyn_with_measured.setRobotState(frame_T_link, joint_positions, base_velocity, joint_velocity, gravity)

    # create and initialize the IK
    ik = WBC(param_handler=param_handler.get_group("IK"), kindyn=kindyn)
    I_H_r_sole = kindyn.getWorldTransform(right_contact_frame)
    I_H_l_sole = kindyn.getWorldTransform(left_contact_frame)
    assert ik.tasks["right_foot_task"].set_set_point(
        blf.conversions.to_manif_pose(I_H_r_sole.getRotation().toNumPy(), I_H_r_sole.getPosition().toNumPy()))
    assert ik.tasks["left_foot_task"].set_set_point(
        blf.conversions.to_manif_pose(I_H_l_sole.getRotation().toNumPy(), I_H_l_sole.getPosition().toNumPy()))
    assert ik.tasks["joint_regularization_task"].set_set_point(joint_positions)
    assert ik.tasks["torso_task"].set_set_point(manif.SO3.Identity())

    desired_joint_positions = joint_positions.copy()

    com_knots_delta_x = param_handler.get_parameter_vector_float("com_knots_delta_x")
    com_knots_delta_y = param_handler.get_parameter_vector_float("com_knots_delta_y")
    com_knots_delta_z = param_handler.get_parameter_vector_float("com_knots_delta_z")
    motion_duration = param_handler.get_parameter_float("motion_duration")
    motion_timeout = param_handler.get_parameter_float("motion_timeout")

    spline = create_new_spline(
        [initial_com_position + np.array([com_knots_delta_x[0], com_knots_delta_y[0], com_knots_delta_z[0]]),
         initial_com_position + np.array([com_knots_delta_x[1], com_knots_delta_y[1], com_knots_delta_z[1]])],
        motion_duration, dt)

    index = 0
    knot_index = 1

    port = blf.yarp_utilities.BufferedPortVectorsCollection()
    port.open("/balancing_controller/logger/data:o")

    while True:
        tic = time.time()

        # get the feedback
        assert sensor_bridge.advance()
        are_joint_ok, joint_positions, _ = sensor_bridge.get_joint_positions()
        assert are_joint_ok
        assert kindyn.setRobotState(frame_T_link, desired_joint_positions, base_velocity, joint_velocity, gravity)
        assert kindyn_with_measured.setRobotState(frame_T_link, joint_positions, base_velocity, joint_velocity, gravity)

        # solve the IK
        com_spline_output = spline.get_output()
        assert ik.tasks["com_task"].set_set_point(com_spline_output.position, com_spline_output.velocity)
        assert ik.solver.advance()
        assert ik.solver.is_output_valid()

        # integrate the system
        desired_joint_positions += ik.solver.get_output().joint_velocity * dt

        # send the joint pose
        robot_control.set_references(desired_joint_positions, blf.robot_interface.YarpRobotControl.PositionDirect)

        # send the data
        is_ok, left_wrench_front, _ = sensor_bridge.get_cartesian_wrench("left_foot_front")
        assert is_ok
        is_ok, left_wrench_rear, _ = sensor_bridge.get_cartesian_wrench("left_foot_rear")
        assert is_ok
        left_wrench = left_wrench_front + left_wrench_rear

        is_ok, right_wrench_front, _ = sensor_bridge.get_cartesian_wrench("right_foot_front")
        assert is_ok
        is_ok, right_wrench_rear, _ = sensor_bridge.get_cartesian_wrench("right_foot_rear")
        assert is_ok
        right_wrench = right_wrench_front + right_wrench_rear

        global_zmp = evaluate_global_zmp(left_wrench=left_wrench, right_wrench=right_wrench,
                                         l_sole_frame=left_contact_frame,
                                         r_sole_frame=right_contact_frame,
                                         kindyn=kindyn)
        global_zmp_from_measured = evaluate_global_zmp(left_wrench=left_wrench, right_wrench=right_wrench,
                                                       l_sole_frame=left_contact_frame,
                                                       r_sole_frame=right_contact_frame,
                                                       kindyn=kindyn_with_measured)
        local_zmp_left = evaluate_local_zmp(left_wrench)
        local_zmp_right = evaluate_local_zmp(right_wrench)
        com_from_desired = kindyn.getCenterOfMassPosition().toNumPy()
        com_from_measured = kindyn_with_measured.getCenterOfMassPosition().toNumPy()

        data = port.prepare()
        data.vectors = {"global_zmp": global_zmp, "global_zmp_from_measured": global_zmp_from_measured,
                        "local_zmp_left": local_zmp_left, "local_zmp_right": local_zmp_right,
                        "com_from_desired": com_from_desired, "com_from_measured": com_from_measured}
        port.write()

        # Final steps
        spline.advance()

        if index * dt >= motion_duration + motion_timeout:
            if knot_index + 1 >= len(com_knots_delta_x):
                print("Application ended")
                break

            spline = create_new_spline(
                [initial_com_position + np.array(
                    [com_knots_delta_x[knot_index], com_knots_delta_y[knot_index], com_knots_delta_z[knot_index]]),
                 initial_com_position + np.array([com_knots_delta_x[knot_index + 1], com_knots_delta_y[knot_index + 1],
                                                  com_knots_delta_z[knot_index + 1]])],
                motion_duration)

            knot_index += 1
            index = 0
        else:
            index += 1

        toc = time.time()
        delta_time = toc - tic
        if delta_time < dt:
            time.sleep(dt - delta_time)

    port.close()


if __name__ == '__main__':
    network = yarp.Network()
    main()
