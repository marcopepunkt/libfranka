import sys
sys.path.append("build/python")
import numpy as np
import time

from franka_py import Robot, RobotState, Gripper, set_default_behavior, move_to_joint_position, PDController

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

def pd_controller_with_gripper_example(robot, gripper):
    # Use the specified starting position
    q = np.array([0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4])
    print("Using specified starting position")
    
    # Read the gripper state
    gripper_state = gripper.read_once()
    print(f"Current gripper width: {gripper_state.width:.4f} m")
    print(f"Max gripper width: {gripper_state.max_width:.4f} m")
    
    # Create a 7x1 array with robot joint positions (we now use 7x1 instead of 9x1)
    full_state = np.zeros(7)
    full_state[:7] = q
    
    # Initialize PD controller with robot, gripper, and full state
    pd_controller = PDController(robot, gripper, full_state)
    print("PD controller initialized")
    
    # Start the controller
    pd_controller.start()
    print("PD controller started")
    
    # Demonstrate asynchronous gripper control
    print("\n--- Demonstrating asynchronous gripper control ---")
    
    # Define amplitude for each joint (small values for safety)
    joint_amplitudes = 2*np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
    
    # Move the robot in a sine pattern while opening and closing the gripper
    print("\nStarting sine movement with asynchronous gripper control...")
    print("The gripper will open and close based on the sine wave position")
    
    start_time = time.time()
    duration = 10  # seconds
    frequency = 0.5  # Hz
    last_gripper_state = "none"  # Track last gripper command
    
    while time.time() - start_time < duration:
        # Calculate the current phase based on elapsed time
        elapsed = time.time() - start_time
        phase = 2 * np.pi * frequency * elapsed
        
        # Create sine wave for each joint
        sine_value = np.sin(phase)
        
        # Update robot joints with sine pattern
        next_pose = q + joint_amplitudes * sine_value
        
        # Update the target
        pd_controller.update_target(next_pose)
        
        # Open and close gripper based on sine wave position
        # When sine is positive, open the gripper
        # When sine is negative, close the gripper
        if sine_value > 0.3 and last_gripper_state != "open":
            pd_controller.open_gripper()
            last_gripper_state = "open"
            print(f"Time: {elapsed:.1f}s, Robot moving with sine value: {sine_value:.3f} - Opening gripper")
        elif sine_value < -0.3 and last_gripper_state != "close":
            pd_controller.close_gripper()
            last_gripper_state = "close"
            print(f"Time: {elapsed:.1f}s, Robot moving with sine value: {sine_value:.3f} - Closing gripper")
        # Print status periodically
        elif int(elapsed * 10) != int((elapsed - 0.1) * 10):  # Print every 0.1 seconds
            print(f"Time: {elapsed:.1f}s, Robot moving with sine value: {sine_value:.3f}")
       
        # Small sleep to avoid overwhelming the controller
        time.sleep(1/30)
    
   
    
    # Move back to original position
    print("\nMoving back to original position...")
    pd_controller.update_target(q)
    time.sleep(1)  # Give time to move back
    
    print("Stopping the PD controller")
    pd_controller.stop()
    print("PD controller stopped")
    print("\n--- Asynchronous gripper control demonstration completed ---")
    
    
    

if __name__ == "__main__":
    try:
        # Connect to robot
        print("Connecting to robot...")
        robot = Robot("192.168.1.200")  # Use your robot's IP
        print("Connected to robot")
        
        # Set default behavior
        set_default_behavior(robot)
        print("Set default behavior")
        
        # Move to the specified starting position
        starting_position = [0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4]
        print("Moving to starting position...")
        move_to_joint_position(robot, starting_position, 0.5)
        print("Reached starting position")
        
        # Connect to gripper
        print("Connecting to gripper...")
        gripper = Gripper("192.168.1.200")  # Use the same IP as your robot
        print("Connected to gripper")
        
        # Perform homing of the gripper
        print("Performing gripper homing...")
        success = gripper.homing()
        print(f"Gripper homing {'successful' if success else 'failed'}")
        
        # Test the PD controller with gripper
        pd_controller_with_gripper_example(robot, gripper)
        
    except Exception as e:
        print(f"Error: {e}")