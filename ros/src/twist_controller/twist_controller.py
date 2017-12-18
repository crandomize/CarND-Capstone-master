
import pid

from yaw_controller import YawController
from lowpass import LowPassFilter
import time
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704

MAX_SPEED = 40.0
MIN_SPEED = 1.0

class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, 
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
                max_steer_angle):

        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        
        self.yaw_controller = YawController(wheel_base,
                                            steer_ratio,
                                            MIN_SPEED,
                                            max_lat_accel,
                                            max_steer_angle)
        
        self.throttle_controller = pid.PID(kp=0.5, ki=0.002, kd=0.25,
                                 mn=-self.max_steer_angle, mx=self.max_steer_angle)
        

        self.timestamp = rospy.get_time()


    def control(self, twist_linear, twist_angular, velocity_linear, current_velocity, target_velocity, dbw_enabled ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        #return 1., 0., 0.

        new_timestamp = rospy.get_time()
        dt = new_timestamp - self.timestamp  
        self.timestamp = new_timestamp
       

        if(dt==0):
            return 0.0, 0.0, 0.0

        if not dbw_enabled:
            return 0.0, 0.0, 0.0

    
        ## Speed  (throttle, brake)
        #vel_cte = twist_linear.x - velocity_linear.x
        vel_cte = target_velocity - current_velocity

        if (vel_cte < 0): 
            #brakes
            brake = abs(305.0*vel_cte)
            #brake = max(brake, 1.0)
            throttle=0.0
        else:
            throttle = self.throttle_controller.step(vel_cte, dt)
            throttle = min(throttle, 1.0)
            brake = 0.0
        

        if (target_velocity < 0.1):
            brake = brake + 355.0
            throttle = 0.0

        '''
        if(pedals<-10.0): 
            pedals=-10.0
        if(pedals>1.0):
            pedals=1.0
        throttle=0.0
        brake=0.0
        if(pedals>0.4):
            throttle=pedals
        if(pedals<0.0):
            brake=abs(pedals)

        if target_velocity < 0.05 and current_velocity < 0.5:
            brake = 1.0
            throttle = 0.0
        '''
        

        rospy.loginfo("twist_controller.py vel_cte:%f targetvel:%f   currentvel:%f  throttle:%f  brake:%f",  
            vel_cte, target_velocity, current_velocity, throttle, brake)


        ## Steering
        steer = self.yaw_controller.get_steering(twist_linear.x, twist_angular.z, velocity_linear.x)

        return throttle,brake, steer