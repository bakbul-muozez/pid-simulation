import math
from typing import List, Tuple, Callable, Optional


class PlantModel:
    """
    Base class for plant models (systems to be controlled).
    """
    
    def __init__(self, initial_state: float = 0.0):
        """
        Initialize plant model.
        
        Args:
            initial_state (float): Initial system state
        """
        self.state = initial_state
        self.initial_state = initial_state
        
    def reset(self) -> None:
        """Reset plant to initial state."""
        self.state = self.initial_state
    
    def update(self, control_input: float, dt: float) -> float:
        """
        Update plant state based on control input.
        
        Args:
            control_input (float): Control signal from controller
            dt (float): Time step
            
        Returns:
            float: New plant output
        """
        raise NotImplementedError("Subclasses must implement update method")


class FirstOrderSystem(PlantModel):
    """
    First-order system model: τ*dy/dt + y = K*u
    
    Where:
    - τ (tau): Time constant
    - K: Steady-state gain
    - u: Control input
    - y: System output
    """
    
    def __init__(self, time_constant: float, gain: float = 1.0, 
                 initial_state: float = 0.0, noise_level: float = 0.0):
        """
        Initialize first-order system.
        
        Args:
            time_constant (float): System time constant (τ)
            gain (float): Steady-state gain (K)
            initial_state (float): Initial system output
            noise_level (float): Measurement noise standard deviation
        """
        super().__init__(initial_state)
        self.time_constant = time_constant
        self.gain = gain
        self.noise_level = noise_level
        
    def update(self, control_input: float, dt: float) -> float:
        """
        Update first-order system state.
        
        Uses Euler integration: y(k+1) = y(k) + dt/τ * (K*u - y(k))
        """
        # First-order differential equation solution
        dydt = (self.gain * control_input - self.state) / self.time_constant
        self.state += dydt * dt
        
        # Add measurement noise if specified
        output = self.state
        if self.noise_level > 0:
            import random
            output += random.gauss(0, self.noise_level)
            
        return output


class SecondOrderSystem(PlantModel):
    """
    Second-order system model: d²y/dt² + 2*ζ*ωn*dy/dt + ωn²*y = ωn²*K*u
    
    Where:
    - ωn: Natural frequency
    - ζ (zeta): Damping ratio
    - K: Steady-state gain
    """
    
    def __init__(self, natural_frequency: float, damping_ratio: float, 
                 gain: float = 1.0, initial_position: float = 0.0, 
                 initial_velocity: float = 0.0, noise_level: float = 0.0):
        """
        Initialize second-order system.
        
        Args:
            natural_frequency (float): Natural frequency (ωn)
            damping_ratio (float): Damping ratio (ζ)
            gain (float): Steady-state gain
            initial_position (float): Initial position
            initial_velocity (float): Initial velocity
            noise_level (float): Measurement noise standard deviation
        """
        super().__init__(initial_position)
        self.natural_frequency = natural_frequency
        self.damping_ratio = damping_ratio
        self.gain = gain
        self.velocity = initial_velocity
        self.initial_velocity = initial_velocity
        self.noise_level = noise_level
        
    def reset(self) -> None:
        """Reset system to initial conditions."""
        super().reset()
        self.velocity = self.initial_velocity
        
    def update(self, control_input: float, dt: float) -> float:
        """
        Update second-order system state using Euler integration.
        """
        wn = self.natural_frequency
        zeta = self.damping_ratio
        
        # Second-order differential equation
        # d²y/dt² = ωn²*K*u - 2*ζ*ωn*dy/dt - ωn²*y
        acceleration = (wn * wn * self.gain * control_input - 
                       2 * zeta * wn * self.velocity - 
                       wn * wn * self.state)
        
        # Euler integration
        self.velocity += acceleration * dt
        self.state += self.velocity * dt
        
        # Add measurement noise if specified
        output = self.state
        if self.noise_level > 0:
            import random
            output += random.gauss(0, self.noise_level)
            
        return output


class MotorModel(PlantModel):
    """
    DC Motor model with inertia and friction.
    
    Equations:
    - V = L*di/dt + R*i + Ke*ω  (electrical)
    - T = Kt*i                   (torque)
    - J*dω/dt = T - B*ω - Tload  (mechanical)
    """
    
    def __init__(self, resistance: float = 1.0, inductance: float = 0.01,
                 motor_constant: float = 0.1, inertia: float = 0.01,
                 friction: float = 0.1, initial_speed: float = 0.0,
                 initial_current: float = 0.0):
        """
        Initialize DC motor model.
        
        Args:
            resistance (float): Armature resistance (R) [Ohm]
            inductance (float): Armature inductance (L) [H]
            motor_constant (float): Motor constant (Kt = Ke) [Nm/A]
            inertia (float): Rotor inertia (J) [kg⋅m²]
            friction (float): Viscous friction coefficient (B) [Nm⋅s]
            initial_speed (float): Initial angular velocity [rad/s]
            initial_current (float): Initial armature current [A]
        """
        super().__init__(initial_speed)
        self.R = resistance
        self.L = inductance
        self.Kt = motor_constant  # Torque constant
        self.Ke = motor_constant  # Back-EMF constant
        self.J = inertia
        self.B = friction
        
        self.current = initial_current
        self.initial_current = initial_current
        
    def reset(self) -> None:
        """Reset motor to initial conditions."""
        super().reset()
        self.current = self.initial_current
        
    def update(self, voltage: float, dt: float, load_torque: float = 0.0) -> float:
        """
        Update motor state given applied voltage and load torque.
        
        Args:
            voltage (float): Applied voltage [V]
            dt (float): Time step [s]
            load_torque (float): External load torque [Nm]
            
        Returns:
            float: Angular velocity [rad/s]
        """
        # Back-EMF
        back_emf = self.Ke * self.state  # state is angular velocity
        
        # Current dynamics: L*di/dt = V - R*i - Ke*ω
        di_dt = (voltage - self.R * self.current - back_emf) / self.L
        self.current += di_dt * dt
        
        # Motor torque
        motor_torque = self.Kt * self.current
        
        # Angular velocity dynamics: J*dω/dt = T - B*ω - Tload
        dw_dt = (motor_torque - self.B * self.state - load_torque) / self.J
        self.state += dw_dt * dt
        
        return self.state
    
    def get_position(self, dt: float) -> float:
        """Get angular position by integrating velocity."""
        if not hasattr(self, 'position'):
            self.position = 0.0
        self.position += self.state * dt
        return self.position


class TankSystem(PlantModel):
    """
    Water tank level control system.
    
    Model: A*dh/dt = Qin - Qout
    Where Qout = K*sqrt(h) (outflow depends on height)
    """
    
    def __init__(self, tank_area: float = 1.0, outlet_coefficient: float = 0.1,
                 initial_level: float = 0.0, max_level: float = 10.0):
        """
        Initialize tank system.
        
        Args:
            tank_area (float): Cross-sectional area of tank [m²]
            outlet_coefficient (float): Outlet flow coefficient
            initial_level (float): Initial water level [m]
            max_level (float): Maximum tank level [m]
        """
        super().__init__(initial_level)
        self.area = tank_area
        self.k_outlet = outlet_coefficient
        self.max_level = max_level
        
    def update(self, inflow_rate: float, dt: float) -> float:
        """
        Update tank level.
        
        Args:
            inflow_rate (float): Input flow rate [m³/s]
            dt (float): Time step [s]
            
        Returns:
            float: Water level [m]
        """
        # Prevent negative levels
        if self.state < 0:
            self.state = 0
            
        # Outflow rate (depends on height)
        outflow_rate = self.k_outlet * math.sqrt(max(self.state, 0))
        
        # Level dynamics: A*dh/dt = Qin - Qout
        dh_dt = (inflow_rate - outflow_rate) / self.area
        self.state += dh_dt * dt
        
        # Constrain to physical limits
        self.state = max(0, min(self.state, self.max_level))
        
        return self.state
