from typing import List, Tuple
import argparse
import numpy as np
from pybullet_tools.utils import INF, connect, add_data_path, HideOutput,\
    disconnect, get_joint_positions, joints_from_names,\
    load_model, dump_body, wait_if_gui, joints_from_names,\
    enable_gravity, joint_controller, get_time_step, control_joints,\
    all_close
from pybullet_tools.pr2_utils import DRAKE_PR2_URDF, SIDE_HOLDING_LEFT_ARM,\
    TOP_HOLDING_LEFT_ARM, PR2_GROUPS, REST_LEFT_ARM
from pybullet_tools.pr2_utils import set_joint_positions, rightarm_from_leftarm,\
    open_arm

# def control(robot, joints, path : List[Tuple]):
#     '''
#     VERY SHORT HORIZON control
#     '''
#     print("Anything in here")
#     tolerance = 1e-3
#     timeout = INF
#     for v in path:
#         assert(len(joints) == len(v))
#         dt = get_time_step()
#         time_elapsed = 0.
#         control_joints(robot, joints, v)
#         positions = get_joint_positions(robot, joints)
#         print("joint_controller")

#         while not all_close(positions, v, atol=tolerance) and (time_elapsed < timeout):
#             yield positions
#             time_elapsed += dt
#             positions = get_joint_positions(robot, joints)
#         # joint_controller(robot, joints, v)
#         # enable_gravity()
#         # for _ in :
    

def main(teleport : bool):
    # Initialize World
    connect(use_gui=True)
    add_data_path()

    # Put in PR2 Robot
    pr2_urdf = DRAKE_PR2_URDF
    with HideOutput():
        pr2 = load_model(pr2_urdf, fixed_base=True)
    dump_body(pr2)

    # Set Arm Positions
    arm_start = np.array(SIDE_HOLDING_LEFT_ARM)
    arm_goal = np.array(TOP_HOLDING_LEFT_ARM)

    # Get Joint variables
    left_joints = joints_from_names(pr2, PR2_GROUPS['left_arm'])
    right_joints = joints_from_names(pr2, PR2_GROUPS['right_arm'])
    torso_joints = joints_from_names(pr2, PR2_GROUPS['torso'])
    
    left_initial_state = get_joint_positions(pr2, left_joints)
    print(f"Left arm joint initial position: {left_initial_state}")
    wait_if_gui("Press Enter to set initial position")
    print(f"Setting Left-arm to position: {arm_start}")
    set_joint_positions(pr2, left_joints, arm_start)
    set_joint_positions(pr2, right_joints, rightarm_from_leftarm(REST_LEFT_ARM))
    set_joint_positions(pr2, torso_joints, [0.2])

    wait_if_gui(f"Short horizon move to {arm_goal}")
    # set_joint_positions(pr2, left_joints, arm_goal)
    # control(pr2, left_joints, [arm_goal])

    tolerance = 1e-3
    timeout = INF
    for v in [arm_goal]:
        assert(len(left_joints) == len(v))
        dt = get_time_step()
        time_elapsed = 0.
        positions_before = get_joint_positions(pr2, left_joints)
        control_joints(pr2, left_joints, v)
        positions = get_joint_positions(pr2, left_joints)
        print(f"before: {positions_before}, {positions}")

        while not all_close(positions, v, atol=tolerance) and (time_elapsed < timeout):
            time_elapsed += dt
            positions = get_joint_positions(pr2, left_joints)
        #     yield positions


    open_arm(pr2, 'left')

    left_joints_states = get_joint_positions(pr2, left_joints)
    print(f"Left arm set to {left_joints_states}")

    wait_if_gui("Finish?")
    disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Base example for PR2 PyBullet")
    parser.add_argument('-t', '--teleport', type=str, help='Teleport or not', default='true')
    args = parser.parse_args()
    args.teleport = args.teleport.lower() == 'true'
    main(args.teleport)
