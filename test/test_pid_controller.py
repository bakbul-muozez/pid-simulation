import unittest
import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.pid_controller import PIDController


class TestPIDController(unittest.TestCase):
    """Test cases for PID Controller."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.controller = PIDController(kp=1.0, ki=0.5, kd=0.1, setpoint=10.0)
    
    def test_initialization(self):
        """Test PID controller initialization."""
        self.assertEqual(self.controller.kp, 1.0)
        self.assertEqual(self.controller.ki, 0.5)
        self.assertEqual(self.controller.kd, 0.1)
        self.assertEqual(self.controller.setpoint, 10.0)
        self.assertEqual(self.controller._integral, 0.0)
        self.assertEqual(self.controller._last_error, 0.0)
    
    def test_set_tunings(self):
        """Test parameter updates."""
        self.controller.set_tunings(2.0, 1.0, 0.2)
        self.assertEqual(self.controller.kp, 2.0)
        self.assertEqual(self.controller.ki, 1.0)
        self.assertEqual(self.controller.kd, 0.2)
    
    def test_set_setpoint(self):
        """Test setpoint updates."""
        self.controller.set_setpoint(20.0)
        self.assertEqual(self.controller.setpoint, 20.0)
    
    def test_output_limits(self):
        """Test output limiting functionality."""
        self.controller.set_output_limits(-100.0, 100.0)
        self.assertEqual(self.controller.output_min, -100.0)
        self.assertEqual(self.controller.output_max, 100.0)
    
    def test_reset(self):
        """Test controller reset."""
        # Set some state
        self.controller._integral = 5.0
        self.controller._last_error = 2.0
        self.controller.time_history = [1, 2, 3]
        
        # Reset
        self.controller.reset()
        
        # Check state is cleared
        self.assertEqual(self.controller._integral, 0.0)
        self.assertEqual(self.controller._last_error, 0.0)
        self.assertEqual(len(self.controller.time_history), 0)
    
    def test_compute_proportional_only(self):
        """Test P-only controller."""
        p_controller = PIDController(kp=2.0, ki=0.0, kd=0.0, setpoint=10.0)
        
        # First call returns 0 (no time reference)
        output1 = p_controller.compute(5.0, 0.0)
        self.assertEqual(output1, 0.0)
        
        # Second call should compute P term
        output2 = p_controller.compute(5.0, 0.1)
        expected = 2.0 * (10.0 - 5.0)  # kp * error
        self.assertEqual(output2, expected)
    
    def test_compute_pi_controller(self):
        """Test PI controller."""
        pi_controller = PIDController(kp=1.0, ki=0.5, kd=0.0, setpoint=10.0)
        
        # First call
        pi_controller.compute(8.0, 0.0)  # Initialize
        
        # Second call - should have P and I terms
        output = pi_controller.compute(8.0, 0.1)
        error = 10.0 - 8.0  # 2.0
        
        # P term: 1.0 * 2.0 = 2.0
        # I term: 0.5 * (2.0 * 0.1) = 0.1
        expected = 2.0 + 0.1
        self.assertAlmostEqual(output, expected, places=6)
    
    def test_compute_full_pid(self):
        """Test full PID controller."""
        pid = PIDController(kp=1.0, ki=0.5, kd=0.2, setpoint=10.0)
        
        # Initialize with first measurement
        pid.compute(5.0, 0.0)
        
        # Second measurement
        output = pid.compute(6.0, 0.1)
        
        error1 = 10.0 - 5.0  # 5.0
        error2 = 10.0 - 6.0  # 4.0
        dt = 0.1
        
        # P term: 1.0 * 4.0 = 4.0
        # I term: 0.5 * (4.0 * 0.1) = 0.2
        # D term: 0.2 * (4.0 - 5.0) / 0.1 = -2.0
        expected = 4.0 + 0.2 - 2.0
        self.assertAlmostEqual(output, expected, places=6)
    
    def test_output_limits_clamping(self):
        """Test output limiting and anti-windup."""
        controller = PIDController(kp=10.0, ki=1.0, kd=0.0, setpoint=100.0)
        controller.set_output_limits(-10.0, 10.0)
        
        # Initialize
        controller.compute(0.0, 0.0)
        
        # Large error should saturate output
        output = controller.compute(0.0, 1.0)
        self.assertEqual(output, 10.0)  # Should be clamped to max
    
    def test_get_components(self):
        """Test getting individual PID components."""
        controller = PIDController(kp=2.0, ki=1.0, kd=0.5, setpoint=10.0)
        
        # Set some internal state
        controller._integral = 2.0
        controller._last_error = 3.0
        
        p, i, d = controller.get_components(7.0)
        
        error = 10.0 - 7.0  # 3.0
        expected_p = 2.0 * 3.0  # 6.0
        expected_i = 1.0 * 2.0  # 2.0 (using current integral)
        expected_d = 0.5 * (3.0 - 3.0) / 0.01  # 0.0 (no change in error)
        
        self.assertEqual(p, expected_p)
        self.assertEqual(i, expected_i)
        self.assertEqual(d, expected_d)
    
    def test_zero_time_delta(self):
        """Test behavior with zero time delta."""
        controller = PIDController(kp=1.0, ki=0.5, kd=0.1, setpoint=10.0)
        
        # Initialize
        controller.compute(5.0, 1.0)
        
        # Same time should return 0
        output = controller.compute(6.0, 1.0)
        self.assertEqual(output, 0.0)
    
    def test_history_tracking(self):
        """Test that history is properly tracked."""
        controller = PIDController(kp=1.0, ki=0.5, kd=0.1, setpoint=10.0)
        
        # Run a few iterations
        controller.compute(5.0, 0.0)
        controller.compute(6.0, 0.1)
        controller.compute(7.0, 0.2)
        
        # Check history lengths
        self.assertEqual(len(controller.time_history), 2)  # Excludes first initialization
        self.assertEqual(len(controller.error_history), 2)
        self.assertEqual(len(controller.output_history), 2)
        self.assertEqual(len(controller.setpoint_history), 2)
        self.assertEqual(len(controller.process_variable_history), 2)
        
        # Check values
        self.assertEqual(controller.setpoint_history[-1], 10.0)
        self.assertEqual(controller.process_variable_history[-1], 7.0)


if __name__ == '__main__':
    unittest.main()
