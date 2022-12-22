from controller import Robot
import numpy as np
import math

def gaussian(x, mu, sigma):
    return (1.0 / (sigma * math.sqrt(2.0 * math.pi))) * math.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))
    
BASE_SPEED = 2.5
TIME_STEP = 64  # not used for now

class TurtleBot:
    def __init__(self):
        self.robot = Robot()
        self.ts = int(self.robot.getBasicTimeStep())

        self.sensors = {"lidar": self.robot.getDevice("LDS-01")}
        self.motors = {
            "left": self.robot.getDevice("left wheel motor"),
            "right": self.robot.getDevice("right wheel motor"),
            "lidar_main": self.robot.getDevice("LDS-01_main_motor"),
            "lidar_secondary": self.robot.getDevice("LDS-01_secondary_motor")
        }

        self.lidar_w = int(self.sensors["lidar"].getHorizontalResolution())
        self.lidar_max_range = self.sensors["lidar"].getMaxRange()
        self.bc = self.braitenberg()
    
    def enable(self):
        for sensor in self.sensors.values():
            sensor.enable(self.ts)
            sensor.enablePointCloud()
        
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        
        self.motors["lidar_main"].setVelocity(30.0)
        self.motors["lidar_secondary"].setVelocity(60.0)

    def braitenberg(self):
        braitenberg_coefficients = []
        for i in range(0, self.lidar_w):
            braitenberg_coefficients.append(6 * gaussian(i, self.lidar_w / 4, self.lidar_w / 12))
        
        return braitenberg_coefficients

    def parse_lidar(self, lidar_values):
        # parse the lidar values and return 
        # for each direction

        north = lidar_values[0]
        north_east = lidar_values[45]
        east = lidar_values[90]
        south_east = lidar_values[135]
        south = lidar_values[180]
        south_west = lidar_values[225]
        west = lidar_values[270]
        north_west = lidar_values[315]
        
        return {"n": north, "ne": north_east, "e": east, "se": south_east, "s": south, "sw": south_west, "w": west, "nw": north_west}

    def fix_bump(self, sw, se):
        print("fix bump")
        self.motors["left"].setVelocity(-BASE_SPEED)
        self.motors["right"].setVelocity(-BASE_SPEED)
        self.robot.step(2000)

        if sw>se:
            self.motors["right"].setVelocity(0)
            self.motors["left"].setVelocity(BASE_SPEED)
            self.robot.step(2000)
        else:
            self.motors["left"].setVelocity(0)
            self.motors["right"].setVelocity(BASE_SPEED)
            self.robot.step(2000)

    def main_loop(self):
        while self.robot.step(self.ts) != -1:
            # reset speed
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED

            # get lidar values and check for bump (bump = south is inf)
            lidar_values = self.sensors["lidar"].getRangeImage()
            directions = self.parse_lidar(lidar_values)
            
            if directions["s"] == float('inf'):
                self.fix_bump(directions["sw"], directions["se"])

            # apply the braitenberg coefficients 
            i = int(0.25 * self.lidar_w)
            while i < (0.5 * self.lidar_w):
                j = int(self.lidar_w - i-1)
                k = int(i - 0.1 * self.lidar_w)

                if (lidar_values[i] != float('inf')) and (not math.isnan(lidar_values[i])) and (lidar_values[j] != float('inf')) and (not math.isnan(lidar_values[j])):
                    left_speed += self.bc[k] * ((1.0 - lidar_values[i] / self.lidar_max_range) - (1.0 - lidar_values[j] / self.lidar_max_range))
                    right_speed += self.bc[k] * ((1.0 - lidar_values[j] / self.lidar_max_range) - (1.0 - lidar_values[i] / self.lidar_max_range))
            
                i += 1
            
            # apply computed velocities
            self.motors["left"].setVelocity(left_speed)
            self.motors["right"].setVelocity(right_speed)

if __name__ == "__main__":
    robot = TurtleBot()
    robot.enable()
    robot.main_loop()