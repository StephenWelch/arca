"""Xbox controller implementation for robot control."""

import pygame
import numpy as np


class XboxController:
    """Xbox controller class using pygame to read the 2nd device."""
    
    def __init__(self, device_id=1):
        """Initialize the Xbox controller.
        
        Args:
            device_id (int): Device ID for the controller (0 for first, 1 for second, etc.)
        """
        pygame.init()
        pygame.joystick.init()
        
        # Get the number of joysticks
        joystick_count = pygame.joystick.get_count()
        print(f"[INFO] Found {joystick_count} joystick(s)")
        
        if joystick_count <= device_id:
            raise RuntimeError(f"Controller with device_id {device_id} not found. Only {joystick_count} controller(s) available.")
        
        # Initialize the specified controller
        self.joystick = pygame.joystick.Joystick(device_id)
        self.joystick.init()
        
        print(f"[INFO] Using controller: {self.joystick.get_name()} (device {device_id})")
        print(f"[INFO] Number of axes: {self.joystick.get_numaxes()}")
        print(f"[INFO] Number of buttons: {self.joystick.get_numbuttons()}")
        
        # Xbox controller mapping - typical pygame mapping
        # Axis 0: Left stick horizontal
        # Axis 1: Left stick vertical  
        # Axis 2: Right stick horizontal
        # Axis 3: Right stick vertical
        # Axis 4: Left trigger
        # Axis 5: Right trigger
        self.left_stick_x = 0  # Left stick horizontal
        self.left_stick_y = 1  # Left stick vertical
        self.right_stick_x = 3  # Right stick horizontal
        self.right_stick_y = 4  # Right stick vertical
        
        # Deadzone to prevent drift
        self.deadzone = 0.05
        
        # Debug: Print all axis values to help identify correct mapping
        print("[DEBUG] Testing all axes - move your controller sticks:")
        for i in range(self.joystick.get_numaxes()):
            print(f"  Axis {i}: {self.joystick.get_axis(i):.3f}")
        
    def advance(self):
        """Get the current controller state.
        
        Returns:
            numpy.ndarray: Array with [x_velocity, y_velocity, angular_velocity]
        """
        # Process pygame events to update joystick state
        pygame.event.pump()
        
        # Get left stick values (for linear velocity)
        x_vel = self.joystick.get_axis(self.left_stick_y)  # Forward/backward
        y_vel = self.joystick.get_axis(self.left_stick_x)   # Left/right
        
        # Get right stick values (for angular velocity)
        angular_vel = self.joystick.get_axis(self.right_stick_x)  # Turn left/right
        
        
        # Apply deadzone
        if abs(x_vel) < self.deadzone:
            x_vel = 0.0
        if abs(y_vel) < self.deadzone:
            y_vel = 0.0
        if abs(angular_vel) < self.deadzone:
            angular_vel = 0.0
        
        # Scale the values (adjust these multipliers as needed)
        x_vel *= 0.5  # Forward/backward velocity scale
        y_vel *= 0.5  # Left/right velocity scale
        angular_vel *= 0.5  # Angular velocity scale - re-enabled
        
        return np.array([-y_vel, x_vel, -angular_vel], dtype=np.float32)
    
    def close(self):
        """Clean up pygame resources."""
        pygame.quit() 