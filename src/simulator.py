import time
from typing import List, Tuple, Optional, Dict, Any
from .pid_controller import PIDController
from .plant_models import PlantModel


class SimulationResults:
    """Container for simulation results."""
    
    def __init__(self):
        self.time = []
        self.setpoint = []
        self.process_variable = []
        self.control_output = []
        self.error = []
        self.p_component = []
        self.i_component = []
        self.d_component = []
        
    def add_data_point(self, t: float, sp: float, pv: float, co: float, 
                      error: float, p: float, i: float, d: float):
        """Add a data point to the results."""
        self.time.append(t)
        self.setpoint.append(sp)
        self.process_variable.append(pv)
        self.control_output.append(co)
        self.error.append(error)
        self.p_component.append(p)
        self.i_component.append(i)
        self.d_component.append(d)
    
    def get_summary(self) -> Dict[str, float]:
        """Get simulation performance summary."""
        if not self.error:
            return {}
            
        # Calculate performance metrics
        abs_errors = [abs(e) for e in self.error]
        squared_errors = [e**2 for e in self.error]
        
        # Settling time (within 2% of final value)
        final_value = self.process_variable[-1] if self.process_variable else 0
        setpoint_value = self.setpoint[-1] if self.setpoint else 0
        tolerance = 0.02 * abs(setpoint_value) if setpoint_value != 0 else 0.02
        
        settling_time = None
        for i in range(len(self.process_variable) - 1, -1, -1):
            if abs(self.process_variable[i] - setpoint_value) > tolerance:
                settling_time = self.time[i] if i < len(self.time) - 1 else self.time[-1]
                break
        
        # Overshoot
        if setpoint_value != 0:
            max_overshoot = 0
            for pv in self.process_variable:
                overshoot = abs((pv - setpoint_value) / setpoint_value) * 100
                max_overshoot = max(max_overshoot, overshoot)
        else:
            max_overshoot = 0
            
        return {
            'iae': sum(abs_errors),  # Integral Absolute Error
            'ise': sum(squared_errors),  # Integral Squared Error
            'mae': sum(abs_errors) / len(abs_errors),  # Mean Absolute Error
            'rmse': (sum(squared_errors) / len(squared_errors)) ** 0.5,  # Root Mean Square Error
            'settling_time': settling_time,
            'max_overshoot_percent': max_overshoot,
            'final_error': self.error[-1] if self.error else 0,
            'steady_state_error': abs(self.error[-1]) if self.error else 0
        }


class PIDSimulator:
    """
    PID Controller Simulation Engine.
    
    This class handles the simulation loop, data collection, and coordination
    between the PID controller and plant model.
    """
    
    def __init__(self, controller: PIDController, plant: PlantModel, 
                 sample_time: float = 0.01):
        """
        Initialize simulator.
        
        Args:
            controller (PIDController): PID controller instance
            plant (PlantModel): Plant model to control
            sample_time (float): Simulation time step
        """
        self.controller = controller
        self.plant = plant
        self.sample_time = sample_time
        self.current_time = 0.0
        
        # Simulation settings
        self.disturbance_function = None
        self.setpoint_function = None
        self.noise_level = 0.0
        
    def set_disturbance(self, disturbance_func):
        """
        Set a disturbance function.
        
        Args:
            disturbance_func: Function that takes time and returns disturbance value
        """
        self.disturbance_function = disturbance_func
    
    def set_setpoint_profile(self, setpoint_func):
        """
        Set a time-varying setpoint.
        
        Args:
            setpoint_func: Function that takes time and returns setpoint value
        """
        self.setpoint_function = setpoint_func
    
    def set_measurement_noise(self, noise_level: float):
        """Set measurement noise level."""
        self.noise_level = noise_level
    
    def run_simulation(self, duration: float, setpoint: Optional[float] = None,
                      real_time: bool = False) -> SimulationResults:
        """
        Run the simulation.
        
        Args:
            duration (float): Simulation duration in seconds
            setpoint (float, optional): Fixed setpoint value
            real_time (bool): Whether to run in real-time
            
        Returns:
            SimulationResults: Simulation data and results
        """
        # Initialize
        results = SimulationResults()
        self.current_time = 0.0
        self.controller.reset()
        self.plant.reset()
        
        # Get initial conditions
        process_variable = self.plant.state
        
        # Add measurement noise if specified
        if self.noise_level > 0:
            import random
            process_variable += random.gauss(0, self.noise_level)
        
        num_steps = int(duration / self.sample_time)
        
        print(f"Running simulation for {duration} seconds ({num_steps} steps)")
        print(f"Sample time: {self.sample_time} seconds")
        
        for step in range(num_steps):
            start_time = time.time() if real_time else None
            
            # Update setpoint if function provided
            if self.setpoint_function:
                current_setpoint = self.setpoint_function(self.current_time)
                self.controller.set_setpoint(current_setpoint)
            elif setpoint is not None:
                self.controller.set_setpoint(setpoint)
            
            # Compute control output
            control_output = self.controller.compute(process_variable, self.current_time)
            
            # Apply disturbance if specified
            disturbance = 0.0
            if self.disturbance_function:
                disturbance = self.disturbance_function(self.current_time)
            
            # Update plant with control input and disturbance
            if hasattr(self.plant, 'update'):
                if 'load_torque' in self.plant.update.__code__.co_varnames:
                    # Motor model with load torque
                    process_variable = self.plant.update(control_output, self.sample_time, disturbance)
                else:
                    # Other plant models
                    process_variable = self.plant.update(control_output + disturbance, self.sample_time)
            
            # Add measurement noise
            if self.noise_level > 0:
                import random
                process_variable += random.gauss(0, self.noise_level)
            
            # Get PID components for analysis
            p_comp, i_comp, d_comp = self.controller.get_components(process_variable)
            
            # Store results
            error = self.controller.setpoint - process_variable
            results.add_data_point(
                self.current_time,
                self.controller.setpoint,
                process_variable,
                control_output,
                error,
                p_comp,
                i_comp,
                d_comp
            )
            
            # Progress indicator
            if step % (num_steps // 10) == 0:
                progress = (step / num_steps) * 100
                print(f"Progress: {progress:.1f}% - Time: {self.current_time:.2f}s")
            
            # Update time
            self.current_time += self.sample_time
            
            # Real-time delay if requested
            if real_time and start_time:
                elapsed = time.time() - start_time
                sleep_time = self.sample_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        print("Simulation completed!")
        
        # Print summary
        summary = results.get_summary()
        if summary:
            print("\n=== Simulation Summary ===")
            print(f"Final Error: {summary['final_error']:.4f}")
            print(f"Steady State Error: {summary['steady_state_error']:.4f}")
            print(f"Mean Absolute Error: {summary['mae']:.4f}")
            print(f"Root Mean Square Error: {summary['rmse']:.4f}")
            if summary['settling_time']:
                print(f"Settling Time: {summary['settling_time']:.2f}s")
            print(f"Max Overshoot: {summary['max_overshoot_percent']:.1f}%")
        
        return results
    
    def step_response(self, amplitude: float = 1.0, duration: float = 10.0) -> SimulationResults:
        """Run a step response simulation."""
        print(f"Running step response simulation (amplitude: {amplitude})")
        return self.run_simulation(duration, setpoint=amplitude)
    
    def ramp_response(self, slope: float = 1.0, duration: float = 10.0) -> SimulationResults:
        """Run a ramp response simulation."""
        print(f"Running ramp response simulation (slope: {slope})")
        
        def ramp_setpoint(t):
            return slope * t
        
        self.set_setpoint_profile(ramp_setpoint)
        return self.run_simulation(duration)
    
    def sine_response(self, amplitude: float = 1.0, frequency: float = 0.1, 
                     duration: float = 20.0) -> SimulationResults:
        """Run a sinusoidal setpoint tracking simulation."""
        import math
        
        print(f"Running sinusoidal response (amp: {amplitude}, freq: {frequency} Hz)")
        
        def sine_setpoint(t):
            return amplitude * math.sin(2 * math.pi * frequency * t)
        
        self.set_setpoint_profile(sine_setpoint)
        return self.run_simulation(duration)
