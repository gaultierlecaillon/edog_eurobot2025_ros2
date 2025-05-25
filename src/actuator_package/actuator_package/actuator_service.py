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
    step_pin = 23  # Step GPIO Pin
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
        self.is_homed_callback()

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
            "cmd_build_service",
            self.build_callback
        )
        
        self.create_service(
            CmdActuatorService,
            "cmd_grab_service",
            self.grab_callback
        )
        
        self.create_service(
            CmdActuatorService,
            "cmd_drop_service",
            self.drop_callback
        )
        
        self.create_service(
            CmdActuatorService,
            "cmd_demo_actuator_service",
            self.demo_actuator_callback
        )

        self.publish_pid()
        self.get_logger().info("🚀 Actuator Service has been started.")

    def publish_pid(self):
        msg = Int32()
        msg.data = os.getpid()  # Get the current process ID
        #self.get_logger().error(f'\033[91m[publish_pid] {msg.data}\033[0m')
        self.pid_publisher.publish(msg)

    def demo_actuator_callback(self, request, response):
        self.get_logger().info(f"demo_actuator_callback Called : param={request.param}")
        
        try:
            self.openServo([0, 1, 2, 3, 4, 5, 6], self.actuator_config['demo_actuator']['delay']);
            time.sleep(1);
            self.closeServo([0, 1, 2, 3, 4, 5, 6]);    
            time.sleep(0.5);  
            
            self.pump(True);  
            self.openServo([0, 1, 2, 3, 4, 5, 6]);
            time.sleep(1);
            self.closeServo([0, 1, 2, 3, 4, 5, 6]);
            self.pump(False); 
            
            
            self.move_elevator(self.actuator_config['elevator']['carry']);            
            time.sleep(0.5);
            self.move_elevator(self.actuator_config['elevator']['down']);
                                     
            response.success = True            
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to execute demo_actuator_callback: {e}")
            response.success = False
            return response

    def grab_callback(self, request, response):
        self.get_logger().info(f"grab_callback Called : param={request.param}")
             
        if (self.elevator_position != self.actuator_config['elevator']['down']):
            self.get_logger().error("🚧 Elevator not down, aborting grab callback ⚠️")
            response.success = False
            return response
                        
        try:
            self.closeServo([0, 1, 2, 3]);
            self.cmd_forward(20, 'slow', False);
            time.sleep(0.5);
            self.move_elevator(self.actuator_config['elevator']['carry']);
            
            response.success = True            
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to execute grab_callback: {e}")
            response.success = False
            return response
            
    def drop_callback(self, request, response):
        self.get_logger().info(f"drop_callback Called : param={request.param}")
                            
        try:        
            self.openServo([0, 1, 2, 3]);    
            self.move_elevator(self.actuator_config['elevator']['down']);
            self.cmd_forward(-200, 'slow', False);
            time.sleep(1.5);
            
            response.success = True            
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to execute drop_callback: {e}")
            response.success = False
            return response

    def build_callback(self, request, response):
        self.get_logger().info(f"build_callback Called : param={request.param}")
                 
        if (self.elevator_position != self.actuator_config['elevator']['down']):
            self.get_logger().error("🚧 Elevator not down, aborting build callback ⚠️")
            response.success = False
            return response   
            
        try:
            # Get first plank
            self.closeServo([1, 2]);
            self.pump(True);        
            self.openVentouse();
            time.sleep(1);
            self.closeVentouse();
            time.sleep(0.5);
            
            # Get middle colomn
            self.openServo([0, 3]);
            self.cmd_forward(-150, 'slow', False);
            self.push_board();

            time.sleep(1.5);
            self.openVentouse();
            self.pump(False);
            time.sleep(1);          
            
            
            # Pile up
            self.move_elevator(self.actuator_config['elevator']['approach_etage_1']);         
            self.cmd_forward(150, 'slow', False);
            time.sleep(1);
            self.move_elevator(self.actuator_config['elevator']['depose_etage_1']);
            self.closeVentouse();
            self.openServo([1, 2]);
            time.sleep(1);
            self.cmd_forward(-200, 'normal', False);            
            
            self.move_elevator(self.actuator_config['elevator']['down']);
            self.initServo();
            
            response.success = True            
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to execute build_callback: {e}")
            response.success = False
            return response
        
   
    def pump(self, turn_on):
        if turn_on:
            self.get_logger().info("Pump ON")
            GPIO.output(self.PUMP_GPIO, GPIO.HIGH)
        else:
            self.get_logger().info("Pump OFF")
            GPIO.output(self.PUMP_GPIO, GPIO.LOW)
        
    def move_elevator(self, step):                  
        step = int(step)  # Ensure it's an integer

        if step == self.elevator_position:
            self.get_logger().info(f"Elevator already at position {step}, no movement needed.")
            return

        if self.elevator_position == -1:
            self.get_logger().error("🚧 Elevator not homed, aborting Moving elevator ⚠️")
            raise ValueError("Elevator not homed, cannot move elevator.")

        if step < 0 or step > self.actuator_config['elevator']['approach_etage_1']:
            self.get_logger().error(f"Invalid elevator step: {step}. Must be between 0 and 3700.")
            raise ValueError(f"Invalid elevator step: {step}. Must be between 0 and 3700.")

        GPIO.output(self.EN_pin, GPIO.LOW)
        delta = step - self.elevator_position
        self.get_logger().info(f"Move elevator by {abs(delta)} steps")

        self.stepper_motor.motor_go(
            delta > 0,           # Direction
            "Full",              # Step type
            abs(delta),          # Number of steps
            0.0005,              # Step delay
            False,               # Verbose
            0.00002              # Initial delay
        )

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

        self.stepper_motor = RpiMotorLib.A4988Nema(self.direction, self.step_pin, (21, 21, 21), "DRV8825")     
        GPIO.setup(self.EN_pin, GPIO.OUT)
        GPIO.output(self.EN_pin, GPIO.HIGH)
        time.sleep(0.1)
        
    def initPump(self):
        self.get_logger().info("Initialization Pump")
        GPIO.setup(self.PUMP_GPIO, GPIO.OUT)
        GPIO.output(self.PUMP_GPIO, GPIO.LOW)
        
    def is_homed_callback(self):
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

        self.elevator_position = 0
        self.move_elevator(self.actuator_config['elevator']['down'])
        GPIO.output(self.EN_pin, GPIO.HIGH)        
        self.get_logger().info("✅ Elevator homed.")

    def initServo(self):
        self.kit.servo[0].angle = self.actuator_config['grabber']['motor0']['close']
        self.kit.servo[1].angle = self.actuator_config['grabber']['motor1']['close']
        self.kit.servo[2].angle = self.actuator_config['grabber']['motor2']['close']
        self.kit.servo[3].angle = self.actuator_config['grabber']['motor3']['close']
        self.kit.servo[4].angle = self.actuator_config['grabber']['motor4']['close']
        self.kit.servo[5].angle = self.actuator_config['grabber']['motor5']['close']
        self.kit.servo[6].angle = self.actuator_config['grabber']['motor6']['close']
    
    def push_board(self):
        target_angle = self.actuator_config['grabber']['motor4']['open']        
        current_angle = self.actuator_config['grabber']['motor4']['close']

        step_delay = 0.008  # Adjust for desired speed
        step_size = 1      # Adjust for smoothness of servo motion
        
        # Gradually move servo 4 and 5
        for angle in range(int(current_angle), int(target_angle) + 1, step_size):
            self.kit.servo[4].angle = angle
            self.kit.servo[5].angle = angle
            time.sleep(step_delay)
        
        self.kit.servo[4].angle = self.actuator_config['grabber']['motor4']['close']
        self.kit.servo[5].angle = self.actuator_config['grabber']['motor5']['close']
        
    def openServo(self, listServo, delay=0):
        for i in range(len(listServo)):
            self.kit.servo[listServo[i]].angle = self.actuator_config['grabber']['motor' + str(listServo[i])]['open']
            if delay > 0 and i != len(listServo) - 1:
                time.sleep(delay)
            
    def closeServo(self, listServo):
        for i in range(len(listServo)):
            self.kit.servo[listServo[i]].angle = self.actuator_config['grabber']['motor' + str(listServo[i])]['close']
   
    def openVentouse(self):
        self.kit.servo[6].angle = self.actuator_config['grabber']['motor6']['open']
        
    def closeVentouse(self):
        self.kit.servo[6].angle = self.actuator_config['grabber']['motor6']['close']
    
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
