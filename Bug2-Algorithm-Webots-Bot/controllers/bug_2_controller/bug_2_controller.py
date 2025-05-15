import math
from initialization import * 
from navigation_utils import *

TIMESTEP = 32

def main() -> None:
    robot = RobotDevices()

    start_position = robot.get_robot_position()[:2]  # Get the start position of the robot
    current_position = start_position
    goal_position = robot.robot.getFromDef('GOAL').getPosition()[:2]   
    print(f"Start position: {start_position} -> Goal position: {goal_position}")
    robot.initialize_devices(TIMESTEP)
    robot.init_robot_state()

    state = 'go_to_goal'    # Initial state
    substate = None         # Substate for wall following
    robot_speed = 10
    goal_threshold = 0.1   

    desired_right_heading = None    # Desired heading when turning right
    desired_left_heading = None     # Desired heading when following the wall
    hit_point = None                # The point where the robot first touches an obstacle 
    leave_point = list()            # The point where the robot leaves the wall
    clean_point = list()            # The point where the robot does not touch the wall

    while robot.robot.step(TIMESTEP) != -1:
        # Read sensor values
        sensors_dict = robot.read_sensors_values()
        gps_values = sensors_dict['gps']
        compass_values = sensors_dict['compass']
        sonar_front_value, sonar_left_value, sonar_back_left_value = sensors_dict['sonar'][0], sensors_dict['sonar'][1], sensors_dict['sonar'][3]
        sonar_right_value, sonar_back_right_value = sensors_dict['sonar'][2], sensors_dict['sonar'][4]
        lidar_front_value, lidar_left_value, lidar_right_value = sensors_dict['lidar'][0], sensors_dict['lidar'][1], sensors_dict['lidar'][2]
        lidar_center_index = len(lidar_left_value) // 2
        heading = robot.get_robot_heading(compass_values)  # Get the robot's heading

        # Update the robot state
        robot.update_robot_state()
        current_position = gps_values[:2]
        distance_to_goal = distance(current_position, goal_position)

        # Print sensor values and robot state
        print(f"Current position: {current_position} -> Goal position: {goal_position}")
        print("Left Sonar: ", sonar_left_value)
        print("Left Lidar: ", lidar_left_value[lidar_center_index])
        print("Left Back Sonar: ", sonar_back_left_value)
        print("Front Sonar: ", sonar_front_value)   
        print("Front Lidar: ", lidar_front_value[lidar_center_index])
        print(f"Heading: {heading}")

        # Conditon to check if the robot is free to move
        free_front = sonar_front_value > 4000 and lidar_front_value[lidar_center_index]  > 1.0
        free_left = sonar_left_value > 4000 and lidar_left_value[lidar_center_index] > 1.0 and sonar_back_left_value > 4000
        free_right = sonar_right_value > 4000 and lidar_right_value[lidar_center_index] > 1.0 and sonar_back_right_value > 4000

        # Condition to check if the robot is stuck
        obstacle_front = sonar_front_value < 1500 and lidar_front_value[lidar_center_index] < 0.3
        obstacle_left = sonar_left_value < 3000 and lidar_left_value[lidar_center_index] < 0.3

        # Check if the robot has reached the goal
        if distance_to_goal < goal_threshold:
            print("Goal reached!")
            robot.update_motor_speed((0, 0, 0))
            break
        
        if state == 'go_to_goal': 
            if obstacle_front: 
                print("Obstacle detected, switching to follow_wall state")
                state = 'follow_wall'
                robot.update_motor_speed((0, 0, 0))
                hit_point = current_position            # Record the point where the robot first touches an obstacle
                leave_point.append(current_position)    # Record the point where the robot leaves the wall
            else: 
                print(compass_values)
                motor_speeds = turn_to_goal(current_position, goal_position, heading, robot_speed, 0.1)
                robot.update_motor_speed(motor_speeds)
                print(f"Moving towards goal: {motor_speeds}")
        elif state == 'follow_wall': 
            if substate is None:
                if obstacle_front:
                    substate = 'turn_right'
                elif free_left:
                    print("Wall lost on the left. Starting left turn to search for wall.")
                    substate = 'turn_left'
                else:
                    substate = 'move_forward'
            
            if substate == 'turn_left':
                if desired_left_heading is None:
                    desired_left_heading = (heading + math.pi / 2) % (2 * math.pi)
                print("Wall disappeared on the left — turning left and moving forward to find wall")
                robot.update_motor_speed((-robot_speed, robot_speed, 0))  # Turn left
                if abs((heading - desired_left_heading + math.pi) % (2 * math.pi) - math.pi) < 0.35:
                    print("Finished turning left.")
                    substate = 'search_wall'
                    desired_left_heading = None
            elif substate == 'search_wall':
                robot.update_motor_speed((robot_speed, robot_speed, robot_speed))
                print("Searching for wall by moving forward")
                if obstacle_left:
                    substate = None  # Reset substate to None to start wall-following again
                    print("Wall detected on the left. Returning to wall-follow mode.")
            elif substate == 'turn_right':
                print("Obstacle ahead. Turning right.")
                robot.update_motor_speed((robot_speed, -robot_speed, 0))  # Turn right
                if desired_right_heading is None:
                    desired_right_heading = (heading - math.pi / 2) % (2 * math.pi)
                diff = abs((desired_right_heading - heading + math.pi) % (2*math.pi) - math.pi)
                print(diff)
                if diff < 0.035:
                    if obstacle_left and free_front:
                        substate = None
                        robot.update_motor_speed((0, 0, 0))
            elif substate == 'move_forward':
                print("Following wall, moving forward")
                robot.update_motor_speed((robot_speed, robot_speed, robot_speed))
                if free_left:
                    substate = 'turn_left'

            leave_point.append(current_position)  # Update the leave point

            if distance(current_position, hit_point) < 0.05 and is_on_M_line(start_position, current_position, goal_position) and distance_to_goal > goal_threshold and len(leave_point) > 1000: 
                print("Goal impossible to reach! You returned to hit point on M-line but goal still too far.")
                robot.update_motor_speed((0, 0, 0))
                break
            elif is_on_M_line(start_position, current_position, goal_position) and distance(current_position, hit_point) >= 0.1:
                print("On M line, switching to go_to_goal state")
                state = 'go_to_goal'    
                substate = None
                robot.update_motor_speed((0, 0, 0))

            # Check if the robot has lost the wall on the left, front, and right
            if free_left and free_front and free_right:
                clean_point.append(current_position)  # Update the clean point
                if len(clean_point) > 5000:
                    print("Wall lost on the left, front, and right. Switching to go_to_goal state")
                    state = 'go_to_goal'
                    substate = None
                    robot.update_motor_speed((0, 0, 0))
                    break

if __name__ == '__main__':
    print("Starting Bug2 controller")
    main()
