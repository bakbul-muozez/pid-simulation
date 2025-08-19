import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional


class PIDController:
    """
    PID (Proportional-Integral-Derivative) Controller implementation.
    
    The PID controller is a control loop feedback mechanism widely used in 
    industrial control systems. It continuously calculates an error value 
    as the difference between a desired setpoint and a measured process variable.
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 setpoint: float = 0.0, sample_time: float = 0.01):
        """
        Initialize PID controller.
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain  
            kd (float): Derivative gain
            setpoint (float): Desired target value
            sample_time (float): Time between samples in seconds
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time
        
        # Internal state variables
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None
        
        # Output limits
        self.output_min = None
        self.output_max = None
        
        # History for plotting
        self.time_history = []
        self.error_history = []
        self.output_history = []
        self.setpoint_history = []
        self.process_variable_history = []
    
    def set_tunings(self, kp: float, ki: float, kd: float) -> None:
        """Update PID tuning parameters."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def set_setpoint(self, setpoint: float) -> None:
        """Update the target setpoint."""
        self.setpoint = setpoint
    
    def set_output_limits(self, min_output: float, max_output: float) -> None:
        """Set output limits for the controller."""
        self.output_min = min_output
        self.output_max = max_output
    
    def reset(self) -> None:
        """Reset the controller's internal state."""
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None
        self.time_history.clear()
        self.error_history.clear()
        self.output_history.clear()
        self.setpoint_history.clear()
        self.process_variable_history.clear()
    
    def compute(self, process_variable: float, current_time: float) -> float:
        """
        Compute PID output value.
        
        Args:
            process_variable (float): Current measured value
            current_time (float): Current time in seconds
            
        Returns:
            float: Controller output
        """
        # Calculate error
        error = self.setpoint - process_variable
        
        # Initialize time tracking
        if self._last_time is None:
            self._last_time = current_time
            self._last_error = error
            return 0.0
        
        # Calculate time delta
        dt = current_time - self._last_time
        
        if dt <= 0.0:
            return 0.0
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self._integral += error * dt
        integral = self.ki * self._integral
        
        # Derivative term
        derivative = self.kd * (error - self._last_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        if self.output_min is not None and output < self.output_min:
            output = self.output_min
            # Anti-windup: prevent integral from growing when output is saturated
            self._integral -= error * dt
        elif self.output_max is not None and output > self.output_max:
            output = self.output_max
            # Anti-windup: prevent integral from growing when output is saturated
            self._integral -= error * dt
        
        # Update history
        self.time_history.append(current_time)
        self.error_history.append(error)
        self.output_history.append(output)
        self.setpoint_history.append(self.setpoint)
        self.process_variable_history.append(process_variable)
        
        # Update state
        self._last_error = error
        self._last_time = current_time
        
        return output
    
    def get_components(self, process_variable: float) -> Tuple[float, float, float]:
        """
        Get individual P, I, D components for analysis.
        
        Args:
            process_variable (float): Current measured value
            
        Returns:
            Tuple[float, float, float]: (P, I, D) components
        """
        error = self.setpoint - process_variable
        
        p_component = self.kp * error
        i_component = self.ki * self._integral
        d_component = self.kd * (error - self._last_error) / self.sample_time if self._last_time else 0.0
        
        return p_component, i_component, d_component
