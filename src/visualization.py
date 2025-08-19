"""
Visualization module for PID simulation results.
Provides plotting functions for analysis and presentation.
"""

import math
from typing import List, Optional, Tuple, Dict, Any

try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available. Plotting functions will not work.")

from .simulator import SimulationResults


class PIDPlotter:
    """Class for creating various plots of PID simulation results."""
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8), style: str = 'seaborn-v0_8'):
        """
        Initialize plotter.
        
        Args:
            figsize: Figure size (width, height)
            style: Matplotlib style to use
        """
        if not MATPLOTLIB_AVAILABLE:
            raise ImportError("matplotlib is required for plotting functionality")
        
        self.figsize = figsize
        try:
            plt.style.use(style)
        except:
            print(f"Warning: Style '{style}' not available, using default")
    
    def plot_response(self, results: SimulationResults, title: str = "PID Control Response",
                     save_path: Optional[str] = None, show_components: bool = False) -> None:
        """
        Plot the main control response.
        
        Args:
            results: Simulation results to plot
            title: Plot title
            save_path: Path to save figure (if provided)
            show_components: Whether to show P, I, D components
        """
        if show_components:
            fig = plt.figure(figsize=(self.figsize[0], self.figsize[1] + 4))
            gs = gridspec.GridSpec(3, 1, height_ratios=[2, 1, 1], hspace=0.3)
            ax1 = fig.add_subplot(gs[0])
            ax2 = fig.add_subplot(gs[1])
            ax3 = fig.add_subplot(gs[2])
        else:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figsize, 
                                          gridspec_kw={'height_ratios': [2, 1]})
        
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # Main response plot
        ax1.plot(results.time, results.setpoint, 'r--', linewidth=2, 
                label='Setpoint', alpha=0.8)
        ax1.plot(results.time, results.process_variable, 'b-', linewidth=2, 
                label='Process Variable')
        ax1.set_ylabel('Output', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        ax1.set_title('System Response', fontsize=14)
        
        # Control output plot
        ax2.plot(results.time, results.control_output, 'g-', linewidth=2, 
                label='Control Output')
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Control Signal', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        ax2.set_title('Control Signal', fontsize=14)
        
        # PID components plot (if requested)
        if show_components:
            ax3.plot(results.time, results.p_component, 'r-', linewidth=1.5, 
                    label='P Component', alpha=0.7)
            ax3.plot(results.time, results.i_component, 'g-', linewidth=1.5, 
                    label='I Component', alpha=0.7)
            ax3.plot(results.time, results.d_component, 'b-', linewidth=1.5, 
                    label='D Component', alpha=0.7)
            ax3.set_xlabel('Time (s)', fontsize=12)
            ax3.set_ylabel('Component Value', fontsize=12)
            ax3.grid(True, alpha=0.3)
            ax3.legend(fontsize=10)
            ax3.set_title('PID Components', fontsize=14)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {save_path}")
        
        plt.show()
    
    def plot_error(self, results: SimulationResults, title: str = "Control Error",
                  save_path: Optional[str] = None) -> None:
        """Plot the control error over time."""
        fig, ax = plt.subplots(figsize=self.figsize)
        
        ax.plot(results.time, results.error, 'r-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Error', fontsize=12)
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Add statistics text box
        if results.error:
            max_error = max(abs(e) for e in results.error)
            final_error = abs(results.error[-1])
            avg_error = sum(abs(e) for e in results.error) / len(results.error)
            
            stats_text = f'Max Error: {max_error:.3f}\\nFinal Error: {final_error:.3f}\\nAvg Error: {avg_error:.3f}'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
                   verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Error plot saved to: {save_path}")
        
        plt.show()
    
    def plot_components(self, results: SimulationResults, title: str = "PID Components",
                       save_path: Optional[str] = None) -> None:
        """Plot individual P, I, D components."""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(self.figsize[0], self.figsize[1] + 2))
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # Proportional component
        ax1.plot(results.time, results.p_component, 'r-', linewidth=2)
        ax1.set_ylabel('P Component', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Proportional Component', fontsize=14)
        
        # Integral component
        ax2.plot(results.time, results.i_component, 'g-', linewidth=2)
        ax2.set_ylabel('I Component', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.set_title('Integral Component', fontsize=14)
        
        # Derivative component
        ax3.plot(results.time, results.d_component, 'b-', linewidth=2)
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('D Component', fontsize=12)
        ax3.grid(True, alpha=0.3)
        ax3.set_title('Derivative Component', fontsize=14)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Components plot saved to: {save_path}")
        
        plt.show()
    
    def plot_phase_portrait(self, results: SimulationResults, title: str = "Phase Portrait",
                           save_path: Optional[str] = None) -> None:
        """Plot error vs. error derivative (phase portrait)."""
        if len(results.error) < 2:
            print("Not enough data points for phase portrait")
            return
            
        # Calculate error derivative
        error_derivative = []
        for i in range(1, len(results.error)):
            dt = results.time[i] - results.time[i-1]
            de_dt = (results.error[i] - results.error[i-1]) / dt if dt > 0 else 0
            error_derivative.append(de_dt)
        
        fig, ax = plt.subplots(figsize=self.figsize)
        
        # Plot trajectory
        ax.plot(results.error[1:], error_derivative, 'b-', linewidth=1.5, alpha=0.7)
        
        # Mark start and end points
        ax.plot(results.error[1], error_derivative[0], 'go', markersize=8, label='Start')
        ax.plot(results.error[-1], error_derivative[-1], 'ro', markersize=8, label='End')
        
        ax.set_xlabel('Error', fontsize=12)
        ax.set_ylabel('Error Derivative', fontsize=12)
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)
        
        # Add origin lines
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax.axvline(x=0, color='k', linestyle='--', alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Phase portrait saved to: {save_path}")
        
        plt.show()
    
    def compare_responses(self, results_list: List[SimulationResults], 
                         labels: List[str], title: str = "Response Comparison",
                         save_path: Optional[str] = None) -> None:
        """Compare multiple simulation results."""
        if len(results_list) != len(labels):
            raise ValueError("Number of results must match number of labels")
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figsize,
                                      gridspec_kw={'height_ratios': [2, 1]})
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        colors = plt.cm.tab10(range(len(results_list)))
        
        # Plot setpoint (assuming all have same setpoint)
        if results_list:
            ax1.plot(results_list[0].time, results_list[0].setpoint, 'k--', 
                    linewidth=2, label='Setpoint', alpha=0.8)
        
        # Plot all responses
        for i, (results, label) in enumerate(zip(results_list, labels)):
            color = colors[i]
            ax1.plot(results.time, results.process_variable, 
                    color=color, linewidth=2, label=label)
            ax2.plot(results.time, results.control_output, 
                    color=color, linewidth=2, label=label)
        
        ax1.set_ylabel('Output', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        ax1.set_title('System Response Comparison', fontsize=14)
        
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Control Signal', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        ax2.set_title('Control Signal Comparison', fontsize=14)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Comparison plot saved to: {save_path}")
        
        plt.show()
    
    def plot_bode(self, frequencies: List[float], magnitude_db: List[float], 
                 phase_deg: List[float], title: str = "Bode Plot",
                 save_path: Optional[str] = None) -> None:
        """Plot Bode diagram (magnitude and phase)."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figsize)
        fig.suptitle(title, fontsize=16, fontweight='bold')
        
        # Magnitude plot
        ax1.semilogx(frequencies, magnitude_db, 'b-', linewidth=2)
        ax1.set_ylabel('Magnitude (dB)', fontsize=12)
        ax1.grid(True, alpha=0.3, which='both')
        ax1.set_title('Magnitude Response', fontsize=14)
        
        # Phase plot
        ax2.semilogx(frequencies, phase_deg, 'r-', linewidth=2)
        ax2.set_xlabel('Frequency (rad/s)', fontsize=12)
        ax2.set_ylabel('Phase (degrees)', fontsize=12)
        ax2.grid(True, alpha=0.3, which='both')
        ax2.set_title('Phase Response', fontsize=14)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Bode plot saved to: {save_path}")
        
        plt.show()


def create_summary_report(results: SimulationResults, filename: str = "simulation_report.txt") -> None:
    """Create a text summary report of simulation results."""
    summary = results.get_summary()
    
    with open(filename, 'w') as f:
        f.write("PID Simulation Report\\n")
        f.write("=" * 50 + "\\n\\n")
        
        f.write(f"Simulation Duration: {results.time[-1]:.2f} seconds\\n")
        f.write(f"Number of Data Points: {len(results.time)}\\n")
        f.write(f"Sample Rate: {len(results.time) / results.time[-1]:.1f} Hz\\n\\n")
        
        f.write("Performance Metrics:\\n")
        f.write("-" * 20 + "\\n")
        for key, value in summary.items():
            if value is not None:
                if isinstance(value, float):
                    f.write(f"{key.replace('_', ' ').title()}: {value:.6f}\\n")
                else:
                    f.write(f"{key.replace('_', ' ').title()}: {value}\\n")
        
        f.write("\\nFinal Values:\\n")
        f.write("-" * 15 + "\\n")
        f.write(f"Final Setpoint: {results.setpoint[-1]:.4f}\\n")
        f.write(f"Final Process Variable: {results.process_variable[-1]:.4f}\\n")
        f.write(f"Final Control Output: {results.control_output[-1]:.4f}\\n")
        f.write(f"Final Error: {results.error[-1]:.4f}\\n")
    
    print(f"Summary report saved to: {filename}")


def plot_simple_response(time_data: List[float], setpoint_data: List[float],
                        pv_data: List[float], title: str = "Control Response") -> None:
    """Simple plotting function that doesn't require class instantiation."""
    if not MATPLOTLIB_AVAILABLE:
        print("matplotlib not available for plotting")
        return
    
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, setpoint_data, 'r--', linewidth=2, label='Setpoint')
    plt.plot(time_data, pv_data, 'b-', linewidth=2, label='Process Variable')
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show()
