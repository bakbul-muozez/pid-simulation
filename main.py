#!/usr/bin/env python3
"""
PID Simulation Main Example
Demonstrates basic usage of the PID simulation package.
"""

import sys
import os

# Add src directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.pid_controller import PIDController
from src.plant_models import FirstOrderSystem, SecondOrderSystem, MotorModel
from src.simulator import PIDSimulator
from src.tuning import PIDTuner, suggest_starting_parameters

# Try to import visualization, but don't fail if matplotlib is not available
try:
    from src.visualization import PIDPlotter, plot_simple_response
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    print("Note: Plotting not available (matplotlib not installed)")


def basic_pid_example():
    """Basic PID control example with first-order system."""
    print("=== Basic PID Control Example ===")
    
    # Create a first-order plant (time constant = 2.0 seconds, gain = 1.0)
    plant = FirstOrderSystem(time_constant=2.0, gain=1.0)
    
    # Create PID controller with initial parameters
    controller = PIDController(kp=1.0, ki=0.5, kd=0.1, setpoint=1.0)
    
    # Create simulator
    simulator = PIDSimulator(controller, plant, sample_time=0.01)
    
    # Run step response simulation
    results = simulator.step_response(amplitude=1.0, duration=15.0)
    
    # Print summary
    print("\\nSimulation completed!")
    summary = results.get_summary()
    print(f"Final error: {summary['final_error']:.4f}")
    print(f"Settling time: {summary['settling_time']:.2f} s" if summary['settling_time'] else "Settling time: Not achieved")
    print(f"Max overshoot: {summary['max_overshoot_percent']:.1f}%")
    
    # Plot results if available
    if PLOTTING_AVAILABLE:
        plotter = PIDPlotter()
        plotter.plot_response(results, title="Basic PID Control - First Order System")
    else:
        print("\\nTime data points (first 10):")
        for i in range(min(10, len(results.time))):
            print(f"t={results.time[i]:.2f}s, SP={results.setpoint[i]:.3f}, PV={results.process_variable[i]:.3f}")
    
    return results


def motor_control_example():
    """DC Motor speed control example."""
    print("\\n=== DC Motor Control Example ===")
    
    # Create DC motor model
    motor = MotorModel(
        resistance=2.0,      # 2 ohm armature resistance
        inductance=0.1,      # 0.1 H inductance
        motor_constant=0.5,  # 0.5 Nm/A motor constant
        inertia=0.02,        # 0.02 kg⋅m² inertia
        friction=0.1         # 0.1 Nm⋅s friction
    )
    
    # Get suggested starting parameters for motor
    suggested_params = suggest_starting_parameters("motor")
    
    # Create PID controller for speed control (setpoint in rad/s)
    controller = PIDController(
        kp=suggested_params['kp'],
        ki=suggested_params['ki'],
        kd=suggested_params['kd'],
        setpoint=10.0  # 10 rad/s target speed
    )
    
    # Set reasonable voltage limits
    controller.set_output_limits(-24.0, 24.0)  # ±24V supply
    
    # Create simulator
    simulator = PIDSimulator(controller, motor, sample_time=0.001)  # 1ms sample time
    
    # Add a load disturbance after 5 seconds
    def load_disturbance(t):
        return 2.0 if t > 5.0 else 0.0  # 2 Nm load torque
    
    simulator.set_disturbance(load_disturbance)
    
    # Run simulation
    results = simulator.run_simulation(duration=10.0, setpoint=10.0)
    
    print(f"\\nMotor control completed!")
    summary = results.get_summary()
    print(f"Final speed error: {summary['final_error']:.4f} rad/s")
    print(f"Steady-state error: {summary['steady_state_error']:.4f} rad/s")
    
    # Plot results
    if PLOTTING_AVAILABLE:
        plotter = PIDPlotter()
        plotter.plot_response(results, title="DC Motor Speed Control with Load Disturbance", show_components=True)
    
    return results


def auto_tuning_example():
    """Automatic tuning example using Ziegler-Nichols method."""
    print("\\n=== Auto-Tuning Example ===")
    
    # Create a second-order plant for tuning
    plant = SecondOrderSystem(
        natural_frequency=2.0,
        damping_ratio=0.5,
        gain=1.5
    )
    
    # Create tuner
    tuner = PIDTuner(plant, sample_time=0.01)
    
    # Apply Ziegler-Nichols tuning
    tuning_results = tuner.ziegler_nichols_open_loop(step_size=1.0, test_duration=15.0)
    
    print("\\nTuning completed! Suggested parameters:")
    for method, params in tuning_results.items():
        if isinstance(params, dict) and 'kp' in params:
            print(f"{method.upper()}: Kp={params['kp']:.3f}, Ki={params['ki']:.3f}, Kd={params['kd']:.3f}")
    
    # Test the PID tuned parameters
    pid_params = tuning_results['pid']
    controller = PIDController(
        kp=pid_params['kp'],
        ki=pid_params['ki'], 
        kd=pid_params['kd'],
        setpoint=1.0
    )
    
    simulator = PIDSimulator(controller, plant, sample_time=0.01)
    results = simulator.step_response(amplitude=1.0, duration=20.0)
    
    print(f"\\nTuned controller performance:")
    summary = results.get_summary()
    print(f"Settling time: {summary['settling_time']:.2f} s" if summary['settling_time'] else "Settling time: Not achieved")
    print(f"Max overshoot: {summary['max_overshoot_percent']:.1f}%")
    print(f"Final error: {summary['final_error']:.4f}")
    
    if PLOTTING_AVAILABLE:
        plotter = PIDPlotter()
        plotter.plot_response(results, title="Auto-Tuned PID Controller (Ziegler-Nichols)")
    
    return results


def comparison_example():
    """Compare different tuning methods."""
    print("\\n=== Tuning Methods Comparison ===")
    
    # Create test plant
    plant = FirstOrderSystem(time_constant=3.0, gain=2.0, noise_level=0.01)
    
    # Create tuner and compare methods
    tuner = PIDTuner(plant, sample_time=0.01)
    comparison_results = tuner.compare_tuning_methods(test_setpoint=1.0, test_duration=20.0)
    
    # Find best method based on IAE
    best_method = None
    best_iae = float('inf')
    
    for method, data in comparison_results.items():
        iae = data['performance'].get('iae', float('inf'))
        if iae < best_iae:
            best_iae = iae
            best_method = method
    
    if best_method:
        print(f"\\nBest method: {best_method} (IAE = {best_iae:.4f})")
        best_params = comparison_results[best_method]['parameters']
        print(f"Best parameters: Kp={best_params['kp']:.3f}, Ki={best_params['ki']:.3f}, Kd={best_params['kd']:.3f}")
    
    # Plot comparison if available
    if PLOTTING_AVAILABLE and comparison_results:
        plotter = PIDPlotter()
        results_list = [data['simulation_data'] for data in comparison_results.values()]
        labels = list(comparison_results.keys())
        plotter.compare_responses(results_list, labels, title="Tuning Methods Comparison")
    
    return comparison_results


def tracking_example():
    """Setpoint tracking example with time-varying reference."""
    print("\\n=== Setpoint Tracking Example ===")
    
    # Create plant
    plant = FirstOrderSystem(time_constant=1.5, gain=1.0)
    
    # Create well-tuned controller
    controller = PIDController(kp=2.0, ki=1.0, kd=0.5, setpoint=0.0)
    simulator = PIDSimulator(controller, plant, sample_time=0.01)
    
    # Run sine wave tracking
    results = simulator.sine_response(amplitude=2.0, frequency=0.1, duration=30.0)
    
    print("\\nTracking simulation completed!")
    summary = results.get_summary()
    print(f"RMS tracking error: {summary['rmse']:.4f}")
    print(f"Mean absolute error: {summary['mae']:.4f}")
    
    if PLOTTING_AVAILABLE:
        plotter = PIDPlotter()
        plotter.plot_response(results, title="Sinusoidal Setpoint Tracking")
    
    return results


def interactive_tuning():
    """Interactive tuning session."""
    print("\\n=== Interactive Tuning Session ===")
    print("This example shows how to manually adjust PID parameters")
    
    # Create a challenging plant
    plant = SecondOrderSystem(natural_frequency=1.0, damping_ratio=0.3, gain=1.0)
    
    # Start with poor parameters
    kp, ki, kd = 0.5, 0.1, 0.01
    
    print("\\nManual tuning guidelines:")
    tuner = PIDTuner(plant)
    guide = tuner.manual_tuning_guide()
    for step, instruction in guide.items():
        print(f"{step}: {instruction}")
    
    # Simulate a few iterations of manual tuning
    tuning_iterations = [
        {"kp": 0.5, "ki": 0.1, "kd": 0.01, "note": "Initial guess"},
        {"kp": 1.0, "ki": 0.1, "kd": 0.01, "note": "Increase Kp"},
        {"kp": 1.5, "ki": 0.3, "kd": 0.01, "note": "Increase Kp and Ki"},
        {"kp": 1.2, "ki": 0.25, "kd": 0.05, "note": "Reduce Kp, add Kd"},
    ]
    
    print("\\nTuning progression:")
    for i, params in enumerate(tuning_iterations):
        controller = PIDController(kp=params["kp"], ki=params["ki"], kd=params["kd"], setpoint=1.0)
        simulator = PIDSimulator(controller, plant, sample_time=0.01)
        results = simulator.step_response(amplitude=1.0, duration=15.0)
        summary = results.get_summary()
        
        print(f"Iteration {i+1} ({params['note']}):")
        print(f"  Kp={params['kp']:.2f}, Ki={params['ki']:.2f}, Kd={params['kd']:.2f}")
        print(f"  Overshoot: {summary['max_overshoot_percent']:.1f}%, "
              f"Settling: {summary['settling_time']:.2f}s" if summary['settling_time'] else "Not settled")
        print()
    
    # Final tuned result
    final_params = tuning_iterations[-1]
    controller = PIDController(kp=final_params["kp"], ki=final_params["ki"], 
                              kd=final_params["kd"], setpoint=1.0)
    simulator = PIDSimulator(controller, plant, sample_time=0.01)
    results = simulator.step_response(amplitude=1.0, duration=15.0)
    
    if PLOTTING_AVAILABLE:
        plotter = PIDPlotter()
        plotter.plot_response(results, title="Manually Tuned PID Controller", show_components=True)
    
    return results


def main():
    """Main function to run all examples."""
    print("PID Simulation Examples")
    print("=" * 50)
    
    try:
        # Run examples
        basic_pid_example()
        motor_control_example()
        auto_tuning_example()
        comparison_example()
        tracking_example()
        interactive_tuning()
        
        print("\\n" + "=" * 50)
        print("All examples completed successfully!")
        print("\\nTo create your own simulation:")
        print("1. Import the required modules")
        print("2. Create a plant model (FirstOrderSystem, SecondOrderSystem, etc.)")
        print("3. Create a PID controller with initial parameters")
        print("4. Create a simulator and run simulations")
        print("5. Analyze results and tune parameters as needed")
        
    except KeyboardInterrupt:
        print("\\nSimulation interrupted by user")
    except Exception as e:
        print(f"\\nError occurred: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
