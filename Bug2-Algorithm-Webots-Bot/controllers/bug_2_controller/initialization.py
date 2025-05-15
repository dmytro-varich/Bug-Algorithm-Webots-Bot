import math
import numpy as np
from controller import Supervisor


class RobotDevices:
    def __init__(self):
        self.robot = Supervisor()  # Creating a robot instance (Supervisor)
        self.gps = None
        self.compass = None
        self.motor_left = None
        self.motor_right = None
        self.motor_rear = None  
        self.sonar_front = None
        self.sonar_left = None
        self.sonar_right = None
        self.sonar_back_left = None
        self.sonar_back_right = None
        self.lidar_front = None
        self.lidar_left = None
        self.lidar_right = None
        self.motor_speeds = np.array([0.0, 0.0, 0.0]) 
        self.robot_position = self.robot.getFromDef('ROBOT').getPosition()[:3]
    
    def initialize_devices(self, timestep: int) -> None:
        """Initialize all robot devices."""
        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.motor_left = self.robot.getDevice("motor_left")
        self.motor_right = self.robot.getDevice("motor_right")
        self.motor_rear = self.robot.getDevice("motor_rear")  
        self.sonar_front = self.robot.getDevice("sonar_front")
        self.sonar_left = self.robot.getDevice("sonar_left")
        self.sonar_right = self.robot.getDevice("sonar_right")
        self.sonar_back_left = self.robot.getDevice("sonar_back_left")
        self.sonar_back_right = self.robot.getDevice("sonar_back_right")
        self.lidar_front = self.robot.getDevice("lidar_front")
        self.lidar_left = self.robot.getDevice("lidar_left")
        self.lidar_right = self.robot.getDevice("lidar_right")
        
        # Set motor positions
        self.motor_rear.setPosition(float('inf'))
        self.motor_left.setPosition(float('inf'))
        self.motor_right.setPosition(float('inf'))

        # Set initial motor speeds
        self.motor_rear.setVelocity(0.0)
        self.motor_left.setVelocity(0.0)
        self.motor_right.setVelocity(0.0)
        
        # Enable distance sensors
        self.sonar_front.enable(timestep)
        self.sonar_left.enable(timestep)
        self.sonar_right.enable(timestep)
        self.sonar_back_left.enable(timestep)
        self.sonar_back_right.enable(timestep)
        
        # Enable LiDAR sensors
        self.lidar_front.enable(timestep)
        self.lidar_left.enable(timestep)
        self.lidar_right.enable(timestep)
           
        # Enable GPS and compass
        self.gps.enable(timestep)
        self.compass.enable(timestep)

    def get_robot_position(self) -> list:
        """Get the robot's current position."""
        return self.robot_position

    def read_sensors_values(self) -> dict:
        """Read sensor values."""
        gps_values = self.gps.getValues()
        compass_values = self.compass.getValues()
        
        sonar_front_values = self.sonar_front.getValue()
        sonar_left_values = self.sonar_left.getValue()
        sonar_right_values = self.sonar_right.getValue()
        sonar_back_left_values = self.sonar_back_left.getValue()
        sonar_back_right_values = self.sonar_back_right.getValue()
        
        lidar_front_values = self.lidar_front.getRangeImage()
        lidar_left_values = self.lidar_left.getRangeImage()
        lidar_right_values = self.lidar_right.getRangeImage()
        
        sensors_dict = {
            'gps': gps_values,
            'compass': compass_values,
            'sonar': (sonar_front_values, sonar_left_values, sonar_right_values, sonar_back_left_values, sonar_back_right_values),
            'lidar': (lidar_front_values, lidar_left_values, lidar_right_values),
        }
        return sensors_dict
    
    def init_robot_state(self, in_pos: list=None, in_speeds: list=None) -> None:
        """Initialize the robot state."""
        if in_pos is None:
            in_pos = self.robot_position  # Use the current position by default
        if in_speeds is None:
            in_speeds = self.motor_speeds  # Use the current motor speeds by default

        # Set the initial robot state
        self.robot_position = in_pos
        self.motor_speeds = in_speeds

    def update_robot_state(self) -> None:
        """Update the robot's state, including position and orientation."""
        sensor_values = self.read_sensors_values()
        gps_values = sensor_values['gps']
        compass_values = sensor_values['compass']
    
        # Update the robot's orientation angle
        self.robot_position[2] = math.atan2(compass_values[0], compass_values[1])
          
        # Update the robot's current position
        self.robot_position[0] = gps_values[0]
        self.robot_position[1] = gps_values[1]
    
    def update_motor_speed(self, motor_speeds: list = None):
        """Update the motor speeds of the robot."""
        if motor_speeds is None:
            motor_speeds = [0.0, 0.0, 0.0]  # Use default values if no speeds are provided
                
        self.motor_left.setVelocity(motor_speeds[0])
        self.motor_right.setVelocity(motor_speeds[1])
        self.motor_rear.setVelocity(motor_speeds[2])

    def get_robot_heading(self, north: list) -> float:
        """Calculate the robot's heading based on the compass reading."""
        angle = math.atan2(north[0], north[1])
        return angle