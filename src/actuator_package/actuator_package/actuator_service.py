#!/usr/bin/env python3
import os
import time
import json
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from robot_interfaces.srv import CmdActuatorService
from robot_interfaces.srv import CmdForwardService
from robot_interfaces.srv import CmdRotateService
from robot_interfaces.srv import BoolBool
from robot_interfaces.msg import MotionCompleteResponse
from robot_interfaces.srv import CmdPositionService
from example_interfaces.msg import String
from std_msgs.msg import Int32

# Stepper (set first BCM)
from RpiMotorLib import RpiMotorLib

# Servo
from adafruit_servokit import ServoKit

GPIO.setmode(GPIO.BCM)

class ActuatorService(Node):
    # Stepper Config
    direction = 22  # Direction (DIR) GPIO Pin
    step = 23  # Step GPIO Pin
    EN_pin = 24  # enable pin (LOW to enable)
    endstop_pin = 17  # GPIO BCM for Endstop Switch V1.2
    
    # Pump Config
    PUMP_GPIO = 27

    # Node State
    actuator_config = None
    elevator_position = -1    

    def __init__(self):
        super().__init__("actuator_service")

        self.pid_publisher = self.create_publisher(Int32, 'killable_nodes_pid', 10)        

        self.actuator_config = self.loadActuatorConfig()
        self.kit = ServoKit(channels=16)
        self.initServo()
        self.initStepper()        
        self.initPump()

        ''' Subscribers '''       
        self.create_subscription(
            Bool,
            'shutdown_topic',
            self.shutdown_callback,
            10)

        
        ''' Publisher '''
        if not hasattr(self, 'voice_publisher'):
            self.voice_publisher = self.create_publisher(String, "voice_topic", 10)
        
        ''' Services '''
        self.create_service(
            CmdActuatorService,
            "is_homed_service",
            self.is_homed_callback
        )
        
        self.create_service(
            CmdActuatorService,
            "cmd_build_service",
            self.build_callback
        )

        self.publish_pid()
        self.get_logger().info("🚀 Actuator Service has been started.")

    def publish_pid(self):
        msg = Int32()
        msg.data = os.getpid()  # Get the current process ID
        #self.get_logger().error(f'\033[91m[publish_pid] {msg.data}\033[0m')
        self.pid_publisher.publish(msg)

    
    def build_callback(self, request, response):
        self.get_logger().info(f"build_callback Called : param={request.param}")
        if self.elevator_position == -1:
            self.get_logger().error("🚧 Elevator not homed, aborting grab ⚠️")
            response.success = False
            return response

        try:  
            # Move forward
            self.cmd_forward(200, 'slow', False)
            time.sleep(2)
            
            # Position the elevator to suction the first plank
            step = self.actuator_config['elevator']['suction']
            self.move_elevator(step*0.1)
            
            # Move backward
            self.cmd_forward(-200, 'slow', False)
            self.move_elevator(step)            
            self.pump(True);
            
            # Rotate 180
            self.cmd_rotate(180);
            
            # Move elevator down
            step = self.actuator_config['elevator']['down']
            self.move_elevator(step)
            self.openServo([1, 2]);
            
            # Move backward
            self.cmd_forward(-100, 'slow', False)
            time.sleep(1.5)
            
            # Rotate 180
            self.cmd_rotate(-180);            
            step = self.actuator_config['elevator']['drop_suction']
            self.move_elevator(step * 0.7)
            
            # Move forward
            self.cmd_forward(100, 'slow', False)
            self.move_elevator(step)            
            self.pump(False);
            time.sleep(1)
            
            # Move elevator down
            step = self.actuator_config['elevator']['down']
            self.move_elevator(step)
            
            # Move backward
            self.cmd_forward(-100, 'slow', False)
            time.sleep(1.5)
            self.cmd_rotate(180);
            time.sleep(2.5)
            self.closeServo([1, 2]);
            self.cmd_forward(100, 'slow', False)   
            time.sleep(1.5)         
            
            # Rotate 180            
            self.cmd_rotate(-180);
            step = self.actuator_config['elevator']['approach_etage_1']
            self.move_elevator(step*0.8)
            self.cmd_forward(200, 'slow', False)
            self.move_elevator(step)
            time.sleep(1)
            
            step = self.actuator_config['elevator']['depose_etage_1']
            self.move_elevator(step)
            self.openServo([1, 2]);
            self.cmd_forward(-100, 'slow', False)
            time.sleep(2)
            
            self.cmd_rotate(-90);
            time.sleep(2)
            
            
            response.success = True            
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to execute build_callback: {e}")
            response.success = False
        
    def pump(self, turn_on):
        if turn_on:
            self.get_logger().info("Pump ON")
            GPIO.output(self.PUMP_GPIO, GPIO.HIGH)
        else:
            self.get_logger().info("Pump OFF")
            GPIO.output(self.PUMP_GPIO, GPIO.LOW)
        
    def move_elevator(self, step):
        step = int(step)  # Convert to int if it's a float
        if self.elevator_position == -1:
            self.get_logger().error("🚧 Elevator not homed, aborting grab ⚠️")
            raise ValueError("Elevator not homed, cannot move elevator.")
        if step < 0 or step > 3700:
            self.get_logger().error(f"Invalid elevator step: {step}. Must be between 0 and 3700.")
            raise ValueError(f"Invalid elevator step: {step}. Must be between 0 and 3700.")
        
        GPIO.output(self.EN_pin, GPIO.LOW)
        delta = step - self.elevator_position
        self.get_logger().info(f"Move elevator to {abs(delta)}")

        self.stepper_motor.motor_go(delta > 0,  # True=Clockwise, False=Counter-Clockwise
                                    "Half",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                                    abs(delta),  # number of steps
                                    .0005,  # step delay [sec]
                                    False,  # True = print verbose output
                                    0.00002)  # initial delay [sec]
        self.elevator_position = step       

    ''' Motion Functions '''
    def cmd_forward(self, distance_mm, mode='normal', evitement=True):
        service_name = "cmd_forward_service"

        self.get_logger().info(f"[Exec Action] forward of: {distance_mm}mm")
        client = self.create_client(CmdForwardService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdForwardService.Request()
        request.service_requester = self.__class__.__name__
        request.distance_mm = int(distance_mm)
        request.mode = mode
        request.evitement = evitement
        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")
    
    def cmd_rotate(self, angle_deg):
        service_name = "cmd_rotate_service"

        self.get_logger().info(f"[Exec Action] rotate with angle: '{angle_deg}'")
        client = self.create_client(CmdRotateService, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")

        request = CmdRotateService.Request()
        request.angle_deg = float(angle_deg)
        request.service_requester = str(self.__class__.__name__)

        client.call_async(request)

        self.get_logger().info(f"[Publish] {request} to {service_name}")

    ''' EndMotion Functions '''          
    
    def loadActuatorConfig(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/actuatorConfig.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Actuator Config] actuatorConfig.json")

        return config
    
    def initStepper(self):
        self.get_logger().info("Initialization Stepper (blocking)")

        self.stepper_motor = RpiMotorLib.A4988Nema(self.direction, self.step, (21, 21, 21), "DRV8825")     
        GPIO.setup(self.EN_pin, GPIO.OUT)
        GPIO.output(self.EN_pin, GPIO.HIGH)
        time.sleep(0.1)
        
    def initPump(self):
        self.get_logger().info("Initialization Pump")
        GPIO.setup(self.PUMP_GPIO, GPIO.OUT)
        GPIO.output(self.PUMP_GPIO, GPIO.LOW)
        
    def is_homed_callback(self, request, response):
        self.get_logger().info("Initialization Elevator: searching for endstop...")

        GPIO.setup(self.endstop_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.output(self.EN_pin, GPIO.LOW)

        while GPIO.input(self.endstop_pin):
            self.stepper_motor.motor_go(
                False, # True=Clockwise, False=Counter-Clockwise
                self.actuator_config['stepper']['step_type'], # Step type (Full,Half,1/4,1/8,1/16,1/32)
                self.actuator_config['stepper']['step'], # number of steps
                self.actuator_config['stepper']['step_delay'], # step delay [sec]
                False, # True = print verbose output
                self.actuator_config['stepper']['initial_delay'] # initial delay [sec]
            )

        GPIO.output(self.EN_pin, GPIO.HIGH)
        self.elevator_position = 0
        self.get_logger().info("✅ Elevator homed.")

        response.success = True
        return response

    def initServo(self):
        #self.kit.servo[0].angle = self.actuator_config['grabber']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['grabber']['motor1']['close']
        self.kit.servo[2].angle = self.actuator_config['grabber']['motor2']['close']
        #self.kit.servo[3].angle = self.actuator_config['grabber']['motor3']['close']
    
    def openServo(self, listServo):
        for i in range(len(listServo)):
            self.kit.servo[listServo[i]].angle = self.actuator_config['grabber']['motor' + str(listServo[i])]['open']
    
    def closeServo(self, listServo):
        for i in range(len(listServo)):
            self.kit.servo[listServo[i]].angle = self.actuator_config['grabber']['motor' + str(listServo[i])]['close']
    
    def shutdown_callback(self, msg):       
        if msg.data:                  
            try:
                self.get_logger().info(f"Shutdown signal received {msg}. Shutting down node... ")
                #self.initServo()                
                self.initStepper()  
                time.sleep(0.5)
                GPIO.cleanup()
            except Exception as e:
                self.get_logger().error(f"Error during shutdown: {e}")
            finally:
                rclpy.shutdown()  # Shutdown the ROS client library for Python

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()