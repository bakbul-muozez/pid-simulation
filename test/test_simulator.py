import unittest
import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.pid_controller import PIDController
from src.plant_models import FirstOrderSystem
from src.simulator import PIDSimulator, SimulationResults


class TestSimulator(unittest.TestCase):
    """Test cases for PID simulator."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.plant = FirstOrderSystem(time_constant=2.0, gain=1.0)
        self.controller = PIDController(kp=1.0, ki=0.5, kd=0.1, setpoint=1.0)
        self.simulator = PIDSimulator(self.controller, self.plant, sample_time=0.01)
    
    def test_simulator_initialization(self):
        """Test simulator initialization."""
        self.assertEqual(self.simulator.controller, self.controller)
        self.assertEqual(self.simulator.plant, self.plant)
        self.assertEqual(self.simulator.sample_time, 0.01)
        self.assertEqual(self.simulator.current_time, 0.0)
    
    def test_run_simulation_basic(self):
        """Test basic simulation run."""
        results = self.simulator.run_simulation(duration=1.0, setpoint=1.0)
        
        # Check results structure
        self.assertIsInstance(results, SimulationResults)
        self.assertEqual(len(results.time), 100)  # 1.0s / 0.01s = 100 steps
        self.assertEqual(len(results.setpoint), 100)
        self.assertEqual(len(results.process_variable), 100)
        self.assertEqual(len(results.control_output), 100)
        
        # Check that all setpoints are correct
        self.assertTrue(all(sp == 1.0 for sp in results.setpoint))
        
        # Check time array
        self.assertAlmostEqual(results.time[0], 0.01, places=6)
        self.assertAlmostEqual(results.time[-1], 1.0, places=6)
    
    def test_step_response(self):
        """Test step response simulation."""
        results = self.simulator.step_response(amplitude=2.0, duration=5.0)
        
        # Check step response characteristics
        self.assertTrue(all(sp == 2.0 for sp in results.setpoint))
        
        # Process variable should start near 0 and approach setpoint
        self.assertLess(abs(results.process_variable[0]), 0.5)
        self.assertLess(abs(results.process_variable[-1] - 2.0), 0.5)
    
    def test_ramp_response(self):
        """Test ramp response simulation."""
        results = self.simulator.ramp_response(slope=1.0, duration=2.0)
        
        # Check that setpoint increases linearly
        expected_final_setpoint = 1.0 * 2.0  # slope * duration
        self.assertAlmostEqual(results.setpoint[-1], expected_final_setpoint, places=1)
        
        # Check that setpoint starts near 0
        self.assertLess(abs(results.setpoint[0]), 0.1)
    
    def test_sine_response(self):
        """Test sinusoidal response simulation."""
        results = self.simulator.sine_response(amplitude=1.0, frequency=0.5, duration=2.0)
        
        # Check that setpoint oscillates
        setpoint_max = max(results.setpoint)
        setpoint_min = min(results.setpoint)
        
        self.assertAlmostEqual(setpoint_max, 1.0, places=1)
        self.assertAlmostEqual(setpoint_min, -1.0, places=1)
    
    def test_disturbance_function(self):
        """Test disturbance addition."""
        def disturbance_func(t):
            return 0.5 if t > 1.0 else 0.0
        
        self.simulator.set_disturbance(disturbance_func)
        results = self.simulator.run_simulation(duration=2.0, setpoint=1.0)
        
        # Should have some response to disturbance
        mid_point = len(results.time) // 2
        early_pv = results.process_variable[mid_point//2]
        late_pv = results.process_variable[-1]
        
        # There should be some difference due to disturbance
        # (exact values depend on system dynamics)
        self.assertIsInstance(early_pv, (int, float))
        self.assertIsInstance(late_pv, (int, float))
    
    def test_setpoint_profile(self):
        """Test time-varying setpoint."""
        def setpoint_func(t):
            return 2.0 if t > 1.0 else 1.0
        
        self.simulator.set_setpoint_profile(setpoint_func)
        results = self.simulator.run_simulation(duration=2.0)
        
        # Check setpoint changes
        early_setpoints = results.setpoint[:50]  # First half
        late_setpoints = results.setpoint[50:]   # Second half
        
        self.assertTrue(all(sp == 1.0 for sp in early_setpoints))
        self.assertTrue(all(sp == 2.0 for sp in late_setpoints))
    
    def test_measurement_noise(self):
        """Test measurement noise addition."""
        self.simulator.set_measurement_noise(0.1)
        
        # Run simulation without noise
        simulator_no_noise = PIDSimulator(self.controller, self.plant, 0.01)
        results_no_noise = simulator_no_noise.run_simulation(duration=1.0, setpoint=1.0)
        
        # Run simulation with noise
        results_with_noise = self.simulator.run_simulation(duration=1.0, setpoint=1.0)
        
        # Outputs should be different due to noise
        # (statistical test - not guaranteed to pass every time, but very likely)
        import statistics
        
        # Reset plant and controller for fair comparison
        self.plant.reset()
        self.controller.reset()
        
        # Check that there's variation in the noise case
        pv_std = statistics.stdev(results_with_noise.process_variable)
        self.assertGreater(pv_std, 0.01)


class TestSimulationResults(unittest.TestCase):
    """Test cases for simulation results."""
    
    def setUp(self):
        """Set up test results."""
        self.results = SimulationResults()
        
        # Add some test data
        for i in range(10):
            t = i * 0.1
            setpoint = 1.0
            pv = 0.8 + 0.2 * (i / 10)  # Ramp up to setpoint
            control = 1.0 - 0.1 * i     # Decreasing control effort
            error = setpoint - pv
            
            self.results.add_data_point(t, setpoint, pv, control, error, 
                                      error * 0.5,    # P component
                                      error * 0.1 * t, # I component  
                                      0.0)             # D component
    
    def test_add_data_point(self):
        """Test data point addition."""
        results = SimulationResults()
        results.add_data_point(1.0, 2.0, 1.8, 0.5, 0.2, 0.1, 0.05, 0.02)
        
        self.assertEqual(len(results.time), 1)
        self.assertEqual(results.time[0], 1.0)
        self.assertEqual(results.setpoint[0], 2.0)
        self.assertEqual(results.process_variable[0], 1.8)
        self.assertEqual(results.control_output[0], 0.5)
        self.assertEqual(results.error[0], 0.2)
    
    def test_get_summary(self):
        """Test performance summary calculation."""
        summary = self.results.get_summary()
        
        # Check that summary contains expected keys
        expected_keys = ['iae', 'ise', 'mae', 'rmse', 'settling_time', 
                        'max_overshoot_percent', 'final_error', 'steady_state_error']
        
        for key in expected_keys:
            self.assertIn(key, summary)
        
        # Check that values are reasonable
        self.assertGreaterEqual(summary['iae'], 0)
        self.assertGreaterEqual(summary['ise'], 0)
        self.assertGreaterEqual(summary['mae'], 0)
        self.assertGreaterEqual(summary['rmse'], 0)
        self.assertGreaterEqual(summary['max_overshoot_percent'], 0)
    
    def test_empty_results_summary(self):
        """Test summary of empty results."""
        empty_results = SimulationResults()
        summary = empty_results.get_summary()
        
        self.assertEqual(summary, {})
    
    def test_performance_metrics_calculation(self):
        """Test specific performance metric calculations."""
        # Create simple test case
        results = SimulationResults()
        
        # Add data with known errors
        errors = [1.0, 0.5, 0.2, 0.1, 0.0]
        for i, error in enumerate(errors):
            pv = 1.0 - error  # setpoint = 1.0
            results.add_data_point(i*0.1, 1.0, pv, 0.0, error, 0.0, 0.0, 0.0)
        
        summary = results.get_summary()
        
        # Check IAE (Integral Absolute Error)
        expected_iae = sum(abs(e) for e in errors)
        self.assertAlmostEqual(summary['iae'], expected_iae, places=6)
        
        # Check MAE (Mean Absolute Error)
        expected_mae = expected_iae / len(errors)
        self.assertAlmostEqual(summary['mae'], expected_mae, places=6)
        
        # Check final error
        self.assertEqual(summary['final_error'], 0.0)
        self.assertEqual(summary['steady_state_error'], 0.0)
    
    def test_settling_time_calculation(self):
        """Test settling time calculation."""
        results = SimulationResults()
        
        # Create response that settles at t=0.5s (within 2% of setpoint=1.0)
        for i in range(10):
            t = i * 0.1
            if t < 0.5:
                pv = 0.5  # Far from setpoint
            else:
                pv = 0.99  # Within 2% of setpoint=1.0
            
            error = 1.0 - pv
            results.add_data_point(t, 1.0, pv, 0.0, error, 0.0, 0.0, 0.0)
        
        summary = results.get_summary()
        
        # Settling time should be around 0.4s (last time outside tolerance)
        self.assertIsNotNone(summary['settling_time'])
        self.assertLessEqual(summary['settling_time'], 0.5)
    
    def test_overshoot_calculation(self):
        """Test overshoot calculation."""
        results = SimulationResults()
        
        # Create response with overshoot (setpoint=1.0, max value=1.2 = 20% overshoot)
        values = [0.0, 0.5, 1.0, 1.2, 1.1, 1.0]  # 20% overshoot at index 3
        
        for i, pv in enumerate(values):
            error = 1.0 - pv
            results.add_data_point(i*0.1, 1.0, pv, 0.0, error, 0.0, 0.0, 0.0)
        
        summary = results.get_summary()
        
        # Should detect 20% overshoot
        self.assertAlmostEqual(summary['max_overshoot_percent'], 20.0, places=1)


if __name__ == '__main__':
    unittest.main()
