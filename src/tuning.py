"""
PID Tuning utilities and methods.
Provides various tuning approaches and optimization algorithms.
"""

import math
from typing import Dict, List, Tuple, Optional, Callable
from .pid_controller import PIDController
from .plant_models import PlantModel
from .simulator import PIDSimulator, SimulationResults


class PIDTuner:
    """Class for PID controller tuning using various methods."""
    
    def __init__(self, plant: PlantModel, sample_time: float = 0.01):
        """
        Initialize PID tuner.
        
        Args:
            plant: Plant model to tune controller for
            sample_time: Simulation sample time
        """
        self.plant = plant
        self.sample_time = sample_time
        
    def ziegler_nichols_open_loop(self, step_size: float = 1.0, 
                                 test_duration: float = 20.0) -> Dict[str, float]:
        """
        Ziegler-Nichols open-loop tuning method (process reaction curve).
        
        Args:
            step_size: Size of step input for identification
            test_duration: Duration of identification test
            
        Returns:
            Dict with suggested PID parameters
        """
        print("Running Ziegler-Nichols open-loop identification...")
        
        # Create a P-only controller for step test
        controller = PIDController(kp=0, ki=0, kd=0, setpoint=0)
        simulator = PIDSimulator(controller, self.plant, self.sample_time)
        
        # Reset plant and apply step input
        self.plant.reset()
        
        # Simulate step response
        num_steps = int(test_duration / self.sample_time)
        time_data = []
        output_data = []
        
        for step in range(num_steps):
            current_time = step * self.sample_time
            
            # Apply step input directly to plant
            if hasattr(self.plant, 'update'):
                output = self.plant.update(step_size, self.sample_time)
            else:
                output = self.plant.state
                
            time_data.append(current_time)
            output_data.append(output)
        
        # Find process characteristics
        steady_state_value = output_data[-1]
        
        # Find delay time (L) - time to reach 10% of final value
        threshold_10 = 0.1 * steady_state_value
        delay_time = 0
        for i, value in enumerate(output_data):
            if value >= threshold_10:
                delay_time = time_data[i]
                break
        
        # Find time constant (T) - time to reach 63.2% of final value
        threshold_63 = 0.632 * steady_state_value
        time_constant = test_duration  # default fallback
        for i, value in enumerate(output_data):
            if value >= threshold_63:
                time_constant = time_data[i] - delay_time
                break
        
        # Process gain (K)
        process_gain = steady_state_value / step_size if step_size != 0 else 1.0
        
        print(f"Process characteristics:")
        print(f"  Gain (K): {process_gain:.3f}")
        print(f"  Delay time (L): {delay_time:.3f} s")
        print(f"  Time constant (T): {time_constant:.3f} s")
        
        # Ziegler-Nichols tuning rules
        if delay_time > 0 and time_constant > 0:
            # P controller
            kp_p = 1.0 / (process_gain * delay_time / time_constant)
            
            # PI controller  
            kp_pi = 0.9 / (process_gain * delay_time / time_constant)
            ki_pi = kp_pi / (3.3 * delay_time)
            
            # PID controller
            kp_pid = 1.2 / (process_gain * delay_time / time_constant)
            ki_pid = kp_pid / (2.0 * delay_time)
            kd_pid = kp_pid * 0.5 * delay_time
        else:
            print("Warning: Could not determine process characteristics properly")
            kp_p = kp_pi = kp_pid = 1.0
            ki_pi = ki_pid = 0.1
            kd_pid = 0.01
        
        return {
            'process_gain': process_gain,
            'delay_time': delay_time,
            'time_constant': time_constant,
            'p_only': {'kp': kp_p, 'ki': 0, 'kd': 0},
            'pi': {'kp': kp_pi, 'ki': ki_pi, 'kd': 0},
            'pid': {'kp': kp_pid, 'ki': ki_pid, 'kd': kd_pid}
        }
    
    def cohen_coon_tuning(self, step_size: float = 1.0, 
                         test_duration: float = 20.0) -> Dict[str, float]:
        """
        Cohen-Coon tuning method (improved over Ziegler-Nichols for processes with delay).
        
        Args:
            step_size: Size of step input for identification
            test_duration: Duration of identification test
            
        Returns:
            Dict with suggested PID parameters
        """
        print("Running Cohen-Coon tuning...")
        
        # First get basic process characteristics using Z-N method
        zn_results = self.ziegler_nichols_open_loop(step_size, test_duration)
        
        K = zn_results['process_gain']
        L = zn_results['delay_time']
        T = zn_results['time_constant']
        
        if L <= 0 or T <= 0:
            print("Warning: Invalid process characteristics, falling back to Z-N")
            return zn_results
        
        # Cohen-Coon parameters
        alpha = L / T
        
        # P controller
        kp_p = (1.03 + 0.35 * alpha) / (K * alpha)
        
        # PI controller
        kp_pi = (0.9 + 0.083 * alpha) / (K * alpha)
        ki_pi = kp_pi * (1.27 + 0.6 * alpha) / (L * (1 + 0.3 * alpha))
        
        # PID controller
        kp_pid = (1.35 + 0.25 * alpha) / (K * alpha)
        ki_pid = kp_pid * (1.35 + 0.25 * alpha) / (L * (0.54 + 0.33 * alpha))
        kd_pid = kp_pid * L * (0.5 - 0.15 * alpha) / (1.35 + 0.25 * alpha)
        
        return {
            'process_gain': K,
            'delay_time': L,
            'time_constant': T,
            'alpha': alpha,
            'p_only': {'kp': kp_p, 'ki': 0, 'kd': 0},
            'pi': {'kp': kp_pi, 'ki': ki_pi, 'kd': 0},
            'pid': {'kp': kp_pid, 'ki': ki_pid, 'kd': kd_pid}
        }
    
    def imc_tuning(self, desired_closed_loop_time_constant: float = None,
                   step_size: float = 1.0, test_duration: float = 20.0) -> Dict[str, float]:
        """
        Internal Model Control (IMC) tuning method.
        
        Args:
            desired_closed_loop_time_constant: Desired closed-loop response speed
            step_size: Step size for identification
            test_duration: Test duration
            
        Returns:
            Dict with suggested PID parameters
        """
        print("Running IMC tuning...")
        
        # Get process characteristics
        zn_results = self.ziegler_nichols_open_loop(step_size, test_duration)
        
        K = zn_results['process_gain']
        L = zn_results['delay_time']
        T = zn_results['time_constant']
        
        if desired_closed_loop_time_constant is None:
            # Rule of thumb: λ = L (for good disturbance rejection)
            lambda_c = max(L, 0.1)  # Minimum value to avoid instability
        else:
            lambda_c = desired_closed_loop_time_constant
        
        print(f"Using closed-loop time constant λ = {lambda_c:.3f} s")
        
        if K != 0 and lambda_c > 0:
            # IMC-PID tuning rules for first-order plus dead time
            kp = T / (K * (lambda_c + L))
            ki = 1 / T
            kd = 0  # IMC typically doesn't use derivative action
            
            # For PID with derivative action (optional)
            kp_with_d = (T + 0.5 * L) / (K * (lambda_c + 0.5 * L))
            ki_with_d = 1 / (T + 0.5 * L)
            kd_with_d = (T * L) / (2 * T + L)
        else:
            print("Warning: Invalid parameters for IMC tuning")
            kp = ki = kd = 0.1
            kp_with_d = ki_with_d = kd_with_d = 0.1
        
        return {
            'process_gain': K,
            'delay_time': L,
            'time_constant': T,
            'lambda_c': lambda_c,
            'pi': {'kp': kp, 'ki': ki, 'kd': 0},
            'pid': {'kp': kp_with_d, 'ki': ki_with_d, 'kd': kd_with_d}
        }
    
    def manual_tuning_guide(self) -> Dict[str, str]:
        """Provide manual tuning guidelines."""
        return {
            'step_1': 'Start with Kp only (Ki=0, Kd=0)',
            'step_2': 'Increase Kp until system oscillates consistently',
            'step_3': 'Set Kp to 50% of oscillation value',
            'step_4': 'Add Ki to eliminate steady-state error (start small)',
            'step_5': 'Add Kd to reduce overshoot and improve stability',
            'kp_effects': 'Higher Kp: faster response, but can cause overshoot/instability',
            'ki_effects': 'Higher Ki: eliminates steady-state error, but can cause overshoot',
            'kd_effects': 'Higher Kd: reduces overshoot, but sensitive to noise',
            'tuning_order': 'Usually tune in order: Kp → Ki → Kd'
        }
    
    def optimize_parameters(self, initial_params: Dict[str, float],
                           optimization_target: str = 'ise',
                           test_duration: float = 10.0,
                           setpoint: float = 1.0,
                           max_iterations: int = 50) -> Dict[str, float]:
        """
        Optimize PID parameters using simple gradient descent.
        
        Args:
            initial_params: Starting PID parameters {'kp': x, 'ki': y, 'kd': z}
            optimization_target: 'ise', 'iae', 'settling_time', or 'overshoot'
            test_duration: Simulation duration for each test
            setpoint: Test setpoint value
            max_iterations: Maximum optimization iterations
            
        Returns:
            Optimized PID parameters
        """
        print(f"Optimizing PID parameters using {optimization_target} criterion...")
        
        current_params = initial_params.copy()
        best_params = current_params.copy()
        best_performance = float('inf')
        
        # Optimization parameters
        learning_rate = 0.1
        epsilon = 1e-6  # Small change for gradient calculation
        
        for iteration in range(max_iterations):
            # Evaluate current parameters
            performance = self._evaluate_performance(
                current_params, optimization_target, test_duration, setpoint
            )
            
            if performance < best_performance:
                best_performance = performance
                best_params = current_params.copy()
            
            print(f"Iteration {iteration + 1}: {optimization_target} = {performance:.6f}")
            
            # Calculate gradients
            gradients = {}
            for param in ['kp', 'ki', 'kd']:
                # Forward difference
                params_plus = current_params.copy()
                params_plus[param] += epsilon
                performance_plus = self._evaluate_performance(
                    params_plus, optimization_target, test_duration, setpoint
                )
                
                gradient = (performance_plus - performance) / epsilon
                gradients[param] = gradient
            
            # Update parameters using gradient descent
            for param in ['kp', 'ki', 'kd']:
                current_params[param] -= learning_rate * gradients[param]
                # Ensure parameters stay positive
                current_params[param] = max(0.001, current_params[param])
        
        print(f"Optimization completed. Best {optimization_target}: {best_performance:.6f}")
        print(f"Optimized parameters: Kp={best_params['kp']:.4f}, "
              f"Ki={best_params['ki']:.4f}, Kd={best_params['kd']:.4f}")
        
        return best_params
    
    def _evaluate_performance(self, params: Dict[str, float], criterion: str,
                            duration: float, setpoint: float) -> float:
        """Evaluate PID performance with given parameters."""
        # Create controller with test parameters
        controller = PIDController(
            kp=params['kp'], 
            ki=params['ki'], 
            kd=params['kd'],
            setpoint=setpoint
        )
        
        # Run simulation
        simulator = PIDSimulator(controller, self.plant, self.sample_time)
        results = simulator.run_simulation(duration, setpoint)
        
        # Calculate performance metric
        summary = results.get_summary()
        
        if criterion == 'ise':
            return summary.get('ise', float('inf'))
        elif criterion == 'iae':
            return summary.get('iae', float('inf'))
        elif criterion == 'settling_time':
            return summary.get('settling_time', float('inf')) or float('inf')
        elif criterion == 'overshoot':
            return summary.get('max_overshoot_percent', float('inf'))
        else:
            return summary.get('rmse', float('inf'))
    
    def compare_tuning_methods(self, test_setpoint: float = 1.0,
                              test_duration: float = 15.0) -> Dict[str, Dict]:
        """
        Compare different tuning methods on the same plant.
        
        Args:
            test_setpoint: Setpoint for comparison tests
            test_duration: Duration of each test
            
        Returns:
            Dictionary with results from each tuning method
        """
        print("Comparing tuning methods...")
        
        results = {}
        
        # Ziegler-Nichols
        try:
            zn_params = self.ziegler_nichols_open_loop()
            for method_name, params in [('ZN_P', zn_params['p_only']),
                                      ('ZN_PI', zn_params['pi']),
                                      ('ZN_PID', zn_params['pid'])]:
                controller = PIDController(**params, setpoint=test_setpoint)
                simulator = PIDSimulator(controller, self.plant, self.sample_time)
                sim_results = simulator.run_simulation(test_duration, test_setpoint)
                results[method_name] = {
                    'parameters': params,
                    'performance': sim_results.get_summary(),
                    'simulation_data': sim_results
                }
        except Exception as e:
            print(f"Ziegler-Nichols tuning failed: {e}")
        
        # Cohen-Coon
        try:
            cc_params = self.cohen_coon_tuning()
            for method_name, params in [('CC_PI', cc_params['pi']),
                                      ('CC_PID', cc_params['pid'])]:
                controller = PIDController(**params, setpoint=test_setpoint)
                simulator = PIDSimulator(controller, self.plant, self.sample_time)
                sim_results = simulator.run_simulation(test_duration, test_setpoint)
                results[method_name] = {
                    'parameters': params,
                    'performance': sim_results.get_summary(),
                    'simulation_data': sim_results
                }
        except Exception as e:
            print(f"Cohen-Coon tuning failed: {e}")
        
        # IMC
        try:
            imc_params = self.imc_tuning()
            for method_name, params in [('IMC_PI', imc_params['pi']),
                                      ('IMC_PID', imc_params['pid'])]:
                controller = PIDController(**params, setpoint=test_setpoint)
                simulator = PIDSimulator(controller, self.plant, self.sample_time)
                sim_results = simulator.run_simulation(test_duration, test_setpoint)
                results[method_name] = {
                    'parameters': params,
                    'performance': sim_results.get_summary(),
                    'simulation_data': sim_results
                }
        except Exception as e:
            print(f"IMC tuning failed: {e}")
        
        # Print comparison summary
        print("\\n=== Tuning Method Comparison ===")
        print(f"{'Method':<10} {'Kp':<8} {'Ki':<8} {'Kd':<8} {'IAE':<8} {'Settling':<10}")
        print("-" * 60)
        
        for method, data in results.items():
            params = data['parameters']
            perf = data['performance']
            settling = perf.get('settling_time', 'N/A')
            settling_str = f"{settling:.2f}" if isinstance(settling, (int, float)) else str(settling)
            
            print(f"{method:<10} {params['kp']:<8.3f} {params['ki']:<8.3f} "
                  f"{params['kd']:<8.3f} {perf.get('iae', 0):<8.3f} {settling_str:<10}")
        
        return results


def suggest_starting_parameters(plant_type: str = "general") -> Dict[str, float]:
    """
    Suggest starting PID parameters based on plant type.
    
    Args:
        plant_type: Type of plant ("motor", "tank", "thermal", "general")
        
    Returns:
        Suggested starting parameters
    """
    suggestions = {
        "motor": {"kp": 10.0, "ki": 5.0, "kd": 0.1},
        "tank": {"kp": 2.0, "ki": 0.5, "kd": 0.05},
        "thermal": {"kp": 1.0, "ki": 0.1, "kd": 0.01},
        "general": {"kp": 1.0, "ki": 0.1, "kd": 0.01},
        "fast_system": {"kp": 0.5, "ki": 0.8, "kd": 0.1},
        "slow_system": {"kp": 5.0, "ki": 0.1, "kd": 0.5}
    }
    
    return suggestions.get(plant_type, suggestions["general"])
