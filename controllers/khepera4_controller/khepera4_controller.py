from controller import Robot, Motor, DistanceSensor
import numpy as np
from collections import deque

# import opencv
import cv2 as cv


MAX_SPEED = 47.6
WHEEL_RADIUS = 21
INF = float('inf')

class ChaseFoodState:
    def __init__(self, r):
        self.r=r
    
    def check_transition(self):
        if self.r.has_bumped:
            # if we bump to the food we are done
            print("donete")
            self.r.stop()

    def tick(self):
        # compute food angle
        food_angle = self.r.get_food_angle(2000)

        if food_angle == "none":
            print("we lost food")
            self.r.state = WallFollowState(self.r)

        # turn to food
        if food_angle == "left":
            print("turning left")
            self.r.turn_left(.2*MAX_SPEED, 20)
        
        elif food_angle == "right":
            print("turning right")
            self.r.turn_right(.2*MAX_SPEED, 20)
        else:
            print("moving forward")
            self.r.move_forward(.2*MAX_SPEED, 500)

        
        # force sensors update
        self.r.update_sensors(bump_th=250, color_th=4000)

        # check transitions
        self.check_transition()


    def __str__(self):
        return "ChaseFoodState"

class WallFollowState:
    def __init__(self, r):
        self.r=r
        self.current_wall = "straight"
    
    def check_transition(self):
        if self.r.has_food:
            print("going to chase food")
            self.r.state = ChaseFoodState(self.r)
        elif self.r.has_enemy:
            print("going to avoid enemy")
            self.r.state = AvoidEnemyState(self.r)
        elif self.r.has_danger:
            print("going to avoid danger")
            self.r.state = AvoidDangerState(self.r)

    def tick(self):
        # just follow wall
        self.r.follow_wall(self.current_wall)
        
        # check transitions (sensors are updated regullary)
        self.check_transition()

    
    def __str__(self):
        return "WallFollowState"

class AvoidDangerState:
    def __init__(self, r):
        self.r=r

    def check_transitions(self):
        if self.r.has_food:
            print("going to chase food")
            self.r.state = ChaseFoodState(self.r)
        elif self.r.has_enemy:
            print("going to avoid enemy")
            self.r.state = AvoidEnemyState(self.r)
        if not self.r.has_danger:
            print("going to wall follow")
            self.r.state = WallFollowState(self.r)

    def tick(self):
        # move fast backwards and turn back
        self.r.turn_back(0.5*MAX_SPEED, 20)
        
        # force sensors update
        self.r.update_sensors(bump_th=250, color_th=2500)
        
        # check transitions
        self.check_transitions()

    
    def __str__(self):
        return "AvoidDangerState"

class AvoidEnemyState:
    def __init__(self, r):
        self.r=r

    def check_transitions(self):
        if self.r.has_food:
            print("going to chase food")
            self.r.state = ChaseFoodState(self.r)
        elif self.r.has_danger:
            print("going to avoid danger")
            self.r.state = AvoidDangerState(self.r)
        if not self.r.has_enemy:
            print("going to wall follow")
            self.r.state = WallFollowState(self.r)
    
    def tick(self):
        # move slowly backwards and turn left
        print("avoiding enemy")
        self.r.move_backward_turn(0.25*MAX_SPEED, 200)
        print("avoiding enemy done")

        # force sensors update
        self.r.update_sensors(bump_th=250, color_th=4000)

        # check transitions
        self.check_transitions()

    
    def __str__(self):
        return "AvoidEnemyState"

class FixBumpState:
    def __init__(self, r):
        self.r=r
    
    def check_transition(self):
        if not self.r.has_bumped:
            print("going to wall follow")
            self.r.state = WallFollowState(self.r)

    def tick(self):
        # move backwards and turn back. If we still are bumping
        # repeat the process
        self.r.move_backward(MAX_SPEED, 100)
        self.r.turn_back(MAX_SPEED, 4)

        # force sensors update
        print("fixing bump")
        self.r.update_sensors(bump_th=250, color_th=4000)

        self.check_transition()
    
    def __str__(self):
        return "FixBumpState"
    
class KheperaBot:
    def __init__(self):
        self.robot = Robot()
        self.ts    = int(self.robot.getBasicTimeStep())
        self.pic_idx = 0

        self.sensors = {
            "left": self.robot.getDevice("left infrared sensor"),
            "right": self.robot.getDevice("right infrared sensor"),
            "front": self.robot.getDevice("front infrared sensor"),
            "front left": self.robot.getDevice("front left infrared sensor"),
            "front right": self.robot.getDevice("front right infrared sensor"),
            "camera": self.robot.getDevice("camera")
        }
        self.motors={           
            "left wheel": self.robot.getDevice("left wheel motor"),
            "right wheel": self.robot.getDevice("right wheel motor")
        }
        self.init_sensors()
        self.init_motors()

        self.has_bumped = False     # bump  = the robot has ran into a wall
        self.has_enemy  = False     # enemy = the robot found something blue
        self.has_food   = False     # food  = the robot found something green
        self.has_danger = False     # danger = the robot found something red

        self.state = WallFollowState(self)

    # initialization

    def init_sensors(self):
        # init sensors -> enable them by timestep
        for sensor in self.sensors.values():
            sensor.enable(self.ts)
    
    def init_motors(self):
        # init motors -> set position to inf and velocity to 0
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0)

    # movements

    def move_forward(self, velocity, ammount):
        # move forward -> set velocity both wheels the same value
        self.motors["left wheel"].setVelocity(velocity)
        self.motors["right wheel"].setVelocity(velocity)
        self.robot.step(ammount)
    
    def move_backward(self, velocity, ammount):
        # move backward -> set velocity both wheels the same value but negative
        self.motors["left wheel"].setVelocity(-velocity)
        self.motors["right wheel"].setVelocity(-velocity)
        self.robot.step(self.ts*ammount)
    
    def move_backward_turn(self, velocity, ammount):
        # move backward and turn -> set velocity left wheel to negative velocity and right wheel to 0
        self.motors["left wheel"].setVelocity(-velocity)
        self.motors["right wheel"].setVelocity(-velocity)
        self.robot.step(int(0.75*self.ts*ammount))
        self.motors["left wheel"].setVelocity(-velocity)
        self.motors["right wheel"].setVelocity(-0.25*velocity)
        self.robot.step(int(0.25*self.ts*ammount))

    def turn_left(self, velocity, ammount=2):
        # turn left -> set velocity left wheel to 0 and right wheel to velocity
        self.motors["left wheel"].setVelocity(0)
        self.motors["right wheel"].setVelocity(velocity)
        self.robot.step(self.ts*ammount)
    
    def turn_right(self, velocity, ammount=2):
        # turn right -> set velocity left wheel to velocity and right wheel to 0
        self.motors["left wheel"].setVelocity(velocity)
        self.motors["right wheel"].setVelocity(0)
        self.robot.step(self.ts*ammount)
    
    def turn_back(self, velocity, ammount):
        # turn_back -> set velocity both wheels to negative velocity
        self.motors["left wheel"].setVelocity(0)
        self.motors["right wheel"].setVelocity(velocity)
        self.robot.step(self.ts*ammount)
        
        self.has_danger=False
    
    def stop(self):
        # stop -> set velocity both wheels to 0
        self.motors["left wheel"].setVelocity(0)
        self.motors["right wheel"].setVelocity(0)
        self.robot.step(self.ts)

        self.ts = -1
        return
    
    def follow_wall(self, w=None, threshold=150):
        speed_offset = 0.3 * (MAX_SPEED - 0.03 * self.sensors["front"].getValue())
        fl, fr = self.sensors["front left"].getValue(), self.sensors["front right"].getValue()
        l,  r  = self.sensors["left"].getValue(), self.sensors["right"].getValue()
        
        delta_r, delta_l = 0.02, 0.02
        # if we loose our wall turn HARDER
        if w=="right" and r<threshold and fr<threshold and l<threshold and fl<threshold:
            delta_l=2*delta_l
        if w=="left" and l<threshold and fl<threshold and r<threshold and fr<threshold:
            delta_r=2*delta_r

        speed_delta  = delta_l * fl - delta_r * fr
        
        self.motors["left wheel"].setVelocity(speed_offset + speed_delta)
        self.motors["right wheel"].setVelocity(speed_offset - speed_delta)
        
        if max(fl,l)<threshold and max(fr,r)<threshold:
            return "straight"
        
        return "left" if max(fl, l)>max(fr, r) else "right"

    # sensors
    def process_camera(self):
        # process image camera and returns an array of the number
        # of red, green and blue pixels
        w,h = self.sensors["camera"].getWidth(), self.sensors["camera"].getHeight()
        img = self.sensors["camera"].getImage()
        
        image_array = np.array(self.sensors["camera"].getImageArray(), dtype=np.uint8)
        image_array = cv.resize(image_array, (h//2, w//2))

        # take only center of image
        image_w, image_h = image_array.shape[0], image_array.shape[1]
        delta_size = 100
        image_array = image_array[image_w//2-delta_size:image_w//2+delta_size, image_h//2-delta_size:image_h//2+delta_size]

        # rotate image -90 degrees
        image_array = cv.rotate(image_array, cv.ROTATE_90_CLOCKWISE)
        # flip image
        image_array = cv.flip(image_array, 1)

        # save image as rgb
        if self.pic_idx%3==0 and False:
            print("save image")
            image_rgb = cv.cvtColor(image_array, cv.COLOR_BGR2RGB)
            cv.imwrite("image"+str(self.pic_idx)+".png", image_rgb)

        # remove white pixels
        #image_array[image_array.all() > 100] = 0
        
        # save red channel
        red_channel = image_array[:,:,0]
        red_channel[red_channel < 175] = 0
        red_channel[red_channel > 0] = 255
        
        # save green channel
        green_channel = image_array[:,:,1]
        green_channel[green_channel < 150] = 0
        green_channel[green_channel > 0] = 255
        
        # save blue channel
        blue_channel = image_array[:,:,2]
        blue_channel[blue_channel < 150] = 0
        blue_channel[blue_channel > 0] = 255

        # save image channels
        if self.pic_idx%3==0 and False:
            cv.imwrite("red"+str(self.pic_idx)+".png", red_channel)
            cv.imwrite("green"+str(self.pic_idx)+".png", green_channel)
            cv.imwrite("blue"+str(self.pic_idx)+".png", blue_channel)
        
        self.pic_idx += 1
        

        blue_channel[green_channel > 0] = 0
        blue_channel[red_channel > 0] = 0

        green_channel[blue_channel > 0] = 0
        green_channel[red_channel > 0] = 0

        red_channel[blue_channel > 0] = 0
        red_channel[green_channel > 0] = 0

        red_px = np.count_nonzero(red_channel)

        # count food pixels by summing left third, center third and right third
        green_px_left = np.count_nonzero(green_channel[:, :green_channel.shape[1]//3])
        green_px_center = np.count_nonzero(green_channel[:, green_channel.shape[1]//3:green_channel.shape[1]//3*2])
        green_px_right = np.count_nonzero(green_channel[:, green_channel.shape[1]//3:])

        green_px = green_px_left+green_px_right

        blue_px = np.count_nonzero(blue_channel)

        return red_px, green_px, blue_px, (green_px_left, green_px_center, green_px_right)

    def get_food_angle(self, th):
        # get food position by counting pixels
        r, g, b, (gl, gc, gr) = self.process_camera()

        print("-> Food:",gl, gc, gr)
        if gl<th and gr<th and gc<th:
            return "none"

        if gl>gr and gl>gc:
            return "left"
        elif gl<gr and gr>gc:
            return "right"
        else:
            return "center"

    def update_sensors(self, bump_th=1000, color_th=15000):
        bump_left_val = self.sensors["left"].getValue()
        bump_right_val = self.sensors["right"].getValue()
        bump_front_val = self.sensors["front"].getValue()

        print("-> Bumpers values:",bump_left_val, bump_right_val, bump_front_val)

        bump_left  = self.sensors["left"].getValue() > bump_th 
        bump_right = self.sensors["right"].getValue() > bump_th 
        bump_front = self.sensors["front"].getValue() > bump_th

        self.has_bumped = bump_left or bump_right or bump_front
        print("-> Bumpers:",bump_left, bump_right, bump_front)

        r, g, b, _ = self.process_camera()
        
        negative_th = color_th
        self.has_enemy = r > color_th and g < negative_th and b < negative_th
        self.has_food   = g > color_th and r < negative_th and b < negative_th
        self.has_danger  = b > color_th and r < negative_th and g < negative_th


        print("-> colors (RGB):",r,g,b)
        print("-> Enemy, Food or Danger:",self.has_enemy, self.has_food, self.has_danger)
        
    def main_loop(self):
        while self.robot.step(self.ts) != -1:
            if self.robot.getTime() % 1 <= self.ts / 500:
                self.update_sensors(bump_th=250, color_th=2550)
    
            self.state.tick()

robot = KheperaBot()
robot.main_loop()