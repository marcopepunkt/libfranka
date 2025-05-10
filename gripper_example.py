import sys
sys.path.append("build/python")
import time

from franka_py import Gripper

def gripper_example():
    """
    Example demonstrating how to use the Franka gripper.
    """
    print("Connecting to gripper...")
    gripper = Gripper("192.168.1.200")  # Use the same IP as your robot
    print("Connected to gripper")
    
    # Read the current state
    state = gripper.read_once()
    print(f"Current gripper state:")
    print(f"  Width: {state.width:.4f} m")
    print(f"  Max width: {state.max_width:.4f} m")
    print(f"  Is grasped: {state.is_grasped}")
    print(f"  Temperature: {state.temperature}°C")
    
    # Perform homing
    print("\nPerforming homing...")
    success = gripper.homing()
    print(f"Homing {'successful' if success else 'failed'}")
    
    # Move the gripper to a specific width
    width = 0.04  # 4 cm opening
    speed = 0.1   # 10 cm/s
    print(f"\nMoving gripper to {width*100:.1f} cm with speed {speed*100:.1f} cm/s...")
    success = gripper.move(width, speed)
    print(f"Move {'successful' if success else 'failed'}")
    
    # Wait a moment
    time.sleep(1)
    
    # Grasp an object with specific force
    width = 0.02   # Expected object width: 2 cm
    speed = 0.05   # 5 cm/s
    force = 10     # 10 N
    print(f"\nGrasping object with width {width*100:.1f} cm, speed {speed*100:.1f} cm/s, force {force} N...")
    success = gripper.grasp(width, speed, force)
    print(f"Grasp {'successful' if success else 'failed'}")
    
    # Read the state again to check if an object is grasped
    state = gripper.read_once()
    print(f"\nAfter grasp attempt:")
    print(f"  Width: {state.width:.4f} m")
    print(f"  Is grasped: {state.is_grasped}")
    
    # Release the object by opening the gripper
    print("\nReleasing object...")
    success = gripper.move(0.08, 0.1)  # Open to 8 cm
    print(f"Release {'successful' if success else 'failed'}")

if __name__ == "__main__":
    try:
        gripper_example()
    except Exception as e:
        print(f"Error: {e}")
