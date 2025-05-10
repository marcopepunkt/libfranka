import sys
sys.path.append("build/python")
import numpy as np
import time

from  franka_py import Robot, RobotState, set_default_behavior, move_to_joint_position, PDController

def state_reader_example(robot):
        
    i = 0  # Initialize the counter
    
    def state_callback(state):
        nonlocal i
        print("Joint positions:", state.q)
        i += 1
        if i > 10:
            print("Stopping the callback")
            return False
        else:
            return True
    
    robot.read(state_callback)  # Start reading robot state with callback
    
    
    q = robot.read_once()  # Read the robot state once
    print(q.q)  # Print the joint positions
    print("This is the robot state from the read_once")
    robot.stop()  # Stop the robot
    print("stopped the robot")    

def robot_mover_example(robot):
    q = robot.read_once().q  # Read the robot state once
    q = np.array(q)  # Convert to numpy array
    q += np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5] ) # Modify the joint positions
    move_to_joint_position(robot, q.tolist() ,0.5)  # Move to the modified joint positions

def pd_controller_example(robot):
    q = robot.read_once().q  # Read the robot state once
    print("Red the state")
    q = np.array(q)
    pd_controller = PDController(robot,q)
    print("PD controller initialized")
    pd_controller.start()
    print("PD controller started")
    # wait for 5 sec
    next_pose = q + 5* np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]) # Modify the joint positions
    pd_controller.update_target(next_pose)  # Set the target joint positions
    print("PD controller set target")
    time.sleep(1)  # Wait for 5 seconds
    pd_controller.update_target(q)  # Set the target joint positions back to the original
    print("PD controller set target back to original")
    time.sleep(1)  # Wait for 5 seconds
    print("Stopping the PD controller")
    pd_controller.stop()
    print("PD controller stopped")
    
    
    

if __name__ == "__main__":
    print("trying to connect to robot   ")
    robot = Robot("192.168.1.200")  # or whatever IP 
    print("connected to robot")   
    set_default_behavior(robot)
    print("Have set default behavior")
    
    pd_controller_example(robot)
    
    