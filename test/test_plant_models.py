import unittest
import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from src.plant_models import FirstOrderSystem, SecondOrderSystem, MotorModel, TankSystem


class TestPlantModels(unittest.TestCase):
    """Test cases for plant models."""
    
    def test_first_order_system_initialization(self):
        """Test FirstOrderSystem initialization."""
        system = FirstOrderSystem(time_constant=2.0, gain=1.5, initial_state=5.0)
        
        self.assertEqual(system.time_constant, 2.0)
        self.assertEqual(system.gain, 1.5)
        self.assertEqual(system.state, 5.0)
        self.assertEqual(system.initial_state, 5.0)
    
    def test_first_order_system_update(self):
        """Test FirstOrderSystem dynamics."""
        system = FirstOrderSystem(time_constant=1.0, gain=2.0, initial_state=0.0)
        
        # Apply step input
        output = system.update(control_input=1.0, dt=0.1)
        
        # For first-order system: dy/dt = (K*u - y) / τ
        # With τ=1, K=2, u=1, y0=0: dy/dt = (2*1 - 0) / 1 = 2
        # y(0.1) ≈ 0 + 2*0.1 = 0.2
        expected = 0.2
        self.assertAlmostEqual(output, expected, places=6)
    
    def test_first_order_system_steady_state(self):
        """Test FirstOrderSystem steady state."""
        system = FirstOrderSystem(time_constant=1.0, gain=3.0)
        
        # Apply constant input and simulate to steady state
        for _ in range(1000):  # Long enough to reach steady state
            output = system.update(control_input=2.0, dt=0.01)
        
        # Steady state should be K*u = 3.0*2.0 = 6.0
        self.assertAlmostEqual(output, 6.0, places=1)
    
    def test_second_order_system_initialization(self):
        """Test SecondOrderSystem initialization."""
        system = SecondOrderSystem(
            natural_frequency=2.0,
            damping_ratio=0.5,
            gain=1.0,
            initial_position=1.0,
            initial_velocity=0.5
        )
        
        self.assertEqual(system.natural_frequency, 2.0)
        self.assertEqual(system.damping_ratio, 0.5)
        self.assertEqual(system.gain, 1.0)
        self.assertEqual(system.state, 1.0)  # position
        self.assertEqual(system.velocity, 0.5)
    
    def test_second_order_system_reset(self):
        """Test SecondOrderSystem reset functionality."""
        system = SecondOrderSystem(
            natural_frequency=1.0,
            damping_ratio=0.7,
            initial_position=2.0,
            initial_velocity=1.0
        )
        
        # Change state
        system.state = 5.0
        system.velocity = 3.0
        
        # Reset
        system.reset()
        
        self.assertEqual(system.state, 2.0)
        self.assertEqual(system.velocity, 1.0)
    
    def test_motor_model_initialization(self):
        """Test MotorModel initialization."""
        motor = MotorModel(
            resistance=1.5,
            inductance=0.05,
            motor_constant=0.1,
            inertia=0.01,
            friction=0.05,
            initial_speed=10.0,
            initial_current=2.0
        )
        
        self.assertEqual(motor.R, 1.5)
        self.assertEqual(motor.L, 0.05)
        self.assertEqual(motor.Kt, 0.1)
        self.assertEqual(motor.Ke, 0.1)
        self.assertEqual(motor.J, 0.01)
        self.assertEqual(motor.B, 0.05)
        self.assertEqual(motor.state, 10.0)  # initial speed
        self.assertEqual(motor.current, 2.0)
    
    def test_motor_model_update(self):
        """Test MotorModel dynamics."""
        motor = MotorModel(
            resistance=1.0,
            inductance=0.1,
            motor_constant=0.5,
            inertia=0.01,
            friction=0.1,
            initial_speed=0.0,
            initial_current=0.0
        )
        
        # Apply voltage step
        speed = motor.update(voltage=12.0, dt=0.001)
        
        # Current should increase (L*di/dt = V - R*i - Ke*ω)
        # Speed should increase (J*dω/dt = Kt*i - B*ω)
        self.assertGreater(motor.current, 0.0)
        self.assertGreater(speed, 0.0)
    
    def test_motor_position_integration(self):
        """Test motor position calculation."""
        motor = MotorModel(initial_speed=5.0)  # 5 rad/s initial speed
        
        # Get position after some time
        position = motor.get_position(dt=0.1)
        
        # Position should be approximately speed * time = 5 * 0.1 = 0.5
        self.assertAlmostEqual(position, 0.5, places=6)
        
        # Another step
        position = motor.get_position(dt=0.1)
        self.assertAlmostEqual(position, 1.0, places=6)
    
    def test_tank_system_initialization(self):
        """Test TankSystem initialization."""
        tank = TankSystem(
            tank_area=2.0,
            outlet_coefficient=0.5,
            initial_level=3.0,
            max_level=10.0
        )
        
        self.assertEqual(tank.area, 2.0)
        self.assertEqual(tank.k_outlet, 0.5)
        self.assertEqual(tank.state, 3.0)  # initial level
        self.assertEqual(tank.max_level, 10.0)
    
    def test_tank_system_level_limits(self):
        """Test tank level physical constraints."""
        tank = TankSystem(tank_area=1.0, max_level=5.0, initial_level=0.0)
        
        # Test upper limit
        tank.state = 4.9
        level = tank.update(inflow_rate=10.0, dt=1.0)  # Large inflow
        self.assertLessEqual(level, 5.0)  # Should not exceed max
        
        # Test lower limit
        tank.state = 0.1
        level = tank.update(inflow_rate=0.0, dt=1.0)  # No inflow, only outflow
        self.assertGreaterEqual(level, 0.0)  # Should not go negative
    
    def test_tank_system_steady_state(self):
        """Test tank system equilibrium."""
        tank = TankSystem(tank_area=1.0, outlet_coefficient=1.0, initial_level=4.0)
        
        # At level=4, outflow = k*sqrt(h) = 1*sqrt(4) = 2
        # For equilibrium, inflow should equal outflow
        inflow_equilibrium = 2.0
        
        # Run simulation to verify equilibrium
        for _ in range(100):
            level = tank.update(inflow_rate=inflow_equilibrium, dt=0.1)
        
        # Level should remain approximately at 4.0
        self.assertAlmostEqual(level, 4.0, places=1)
    
    def test_plant_model_base_class(self):
        """Test base PlantModel functionality."""
        from src.plant_models import PlantModel
        
        # Test that base class raises NotImplementedError
        plant = PlantModel(initial_state=5.0)
        self.assertEqual(plant.state, 5.0)
        self.assertEqual(plant.initial_state, 5.0)
        
        # Reset should work
        plant.state = 10.0
        plant.reset()
        self.assertEqual(plant.state, 5.0)
        
        # Update should raise NotImplementedError
        with self.assertRaises(NotImplementedError):
            plant.update(1.0, 0.1)
    
    def test_noise_addition(self):
        """Test measurement noise in plant models."""
        # Test with noise
        system_with_noise = FirstOrderSystem(
            time_constant=1.0, 
            gain=1.0, 
            noise_level=0.1
        )
        
        # Test without noise
        system_no_noise = FirstOrderSystem(
            time_constant=1.0, 
            gain=1.0, 
            noise_level=0.0
        )
        
        # Run multiple times and check that noise system has variation
        outputs_noise = []
        outputs_no_noise = []
        
        for _ in range(10):
            # Reset both systems
            system_with_noise.reset()
            system_no_noise.reset()
            
            # Apply same input
            out_noise = system_with_noise.update(1.0, 0.1)
            out_no_noise = system_no_noise.update(1.0, 0.1)
            
            outputs_noise.append(out_noise)
            outputs_no_noise.append(out_no_noise)
        
        # No-noise outputs should be identical
        self.assertTrue(all(x == outputs_no_noise[0] for x in outputs_no_noise))
        
        # With-noise outputs should have some variation (statistical test)
        import statistics
        noise_std = statistics.stdev(outputs_noise)
        self.assertGreater(noise_std, 0.01)  # Should have some variation


if __name__ == '__main__':
    unittest.main()
