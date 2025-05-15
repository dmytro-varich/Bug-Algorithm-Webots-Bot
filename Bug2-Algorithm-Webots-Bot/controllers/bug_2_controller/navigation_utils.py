import math
import numpy as np


def distance(point1: list, point2: list) -> float:
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def is_on_M_line(start: list, current: list, goal: list, tolerance: float=0.1) -> bool:
    """Check if the robot is on the M line."""
    dx1 = goal[0] - start[0]
    dy1 = goal[1] - start[1]

    dx2 = goal[0] - current[0]
    dy2 = goal[1] - current[1]

    # Let's check how “parallel” they are - using the vector product 
    cross = abs(dx1 * dy2 - dx2 * dy1)

    return cross < tolerance


def turn_to_goal(position: list, goal: list, heading: float, speed: int=10, tolerance: float=0.1) -> tuple: 
    """Calculate the motor speeds to turn towards the goal."""
    goal_angle = math.atan2(goal[1] - position[1], goal[0] - position[0])
    print(f"Goal angle: {goal_angle}")
    angle_diff = goal_angle - heading

    # Normalize the angle difference to the range [-pi, pi]
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
    print(f"Angle difference: {angle_diff}")
    if abs(angle_diff) < tolerance:
        print("Moving forward")
        return [speed, speed, speed * 0.6]  # Move forward
    if angle_diff > 0: 
        print("Turning left")
        return [-speed * 0.5, speed * 0.5, 0]  # Turn left
    else: 
        print("Turning right")
        return [speed * 0.5, -speed * 0.5, 0]  # Turn right