"""
Visualization module for PID simulation results.
Provides plotting functions for analysis and presentation.
"""

import math
import os
import re
from typing import List, Optional, Tuple, Dict, Any
"""
Visualization module for PID simulation results.
This simplified module sets a non-interactive backend (Agg) when Tcl/Tk is
not available and saves plots to the `plots/` directory when a display is
not present.
"""

import os
import re
from typing import List, Optional, Tuple

MATPLOTLIB_AVAILABLE = False

# First detect if tkinter (Tcl/Tk) is usable
TK_AVAILABLE = False
try:
    import tkinter as _tk  # type: ignore
    try:
        _root = _tk.Tk()
        _root.withdraw()
        _root.destroy()
        TK_AVAILABLE = True
    except Exception:
        TK_AVAILABLE = False
except Exception:
    TK_AVAILABLE = False

try:
    import matplotlib
    if not TK_AVAILABLE:
        try:
            matplotlib.use('Agg')
        except Exception:
            pass

    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    MATPLOTLIB_AVAILABLE = True
except Exception:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available or backend initialization failed; plotting disabled.")

from .simulator import SimulationResults


class PIDPlotter:
    def __init__(self, figsize: Tuple[int, int] = (12, 8), style: str = 'seaborn-v0_8'):
        if not MATPLOTLIB_AVAILABLE:
            raise ImportError("matplotlib is required for plotting functionality")
        self.figsize = figsize
        try:
            plt.style.use(style)
        except Exception:
            pass

    def _sanitize_filename(self, s: str) -> str:
        s = re.sub(r'[^A-Za-z0-9_\-\. ]+', '', s)
        s = s.strip().replace(' ', '_')
        return s[:150]

    def _finalize_plot(self, save_path: Optional[str], title: str = "plot") -> None:
        try:
            backend = plt.get_backend().lower()
        except Exception:
            backend = 'agg'

        non_interactive = any(b in backend for b in ('agg', 'pdf', 'svg', 'ps', 'cairo'))

        if save_path:
            try:
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                print(f"Plot saved to: {save_path}")
            except Exception as e:
                print(f"Failed to save plot to {save_path}: {e}")
            return

        if non_interactive:
            try:
                os.makedirs('plots', exist_ok=True)
                fname = os.path.join('plots', f"{self._sanitize_filename(title)}.png")
                plt.savefig(fname, dpi=300, bbox_inches='tight')
                print(f"Plot saved to: {fname} (backend={plt.get_backend()})")
            except Exception as e:
                print(f"Failed to save plot to file: {e}")
            return

        try:
            plt.show()
        except Exception as e:
            print(f"Could not display plot interactively: {e}")
            try:
                os.makedirs('plots', exist_ok=True)
                fname = os.path.join('plots', f"{self._sanitize_filename(title)}.png")
                plt.savefig(fname, dpi=300, bbox_inches='tight')
                print(f"Plot saved to: {fname} (fallback)")
            except Exception as e2:
                print(f"Failed fallback save: {e2}")

    def plot_response(self, results: SimulationResults, title: str = "PID Control Response",
                      save_path: Optional[str] = None, show_components: bool = False) -> None:
        if show_components:
            fig = plt.figure(figsize=(self.figsize[0], self.figsize[1] + 4))
            gs = gridspec.GridSpec(3, 1, height_ratios=[2, 1, 1], hspace=0.3)
            ax1 = fig.add_subplot(gs[0])
            ax2 = fig.add_subplot(gs[1])
            ax3 = fig.add_subplot(gs[2])
        else:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figsize, gridspec_kw={'height_ratios': [2, 1]})

        fig.suptitle(title, fontsize=16, fontweight='bold')

        ax1.plot(results.time, results.setpoint, 'r--', linewidth=2, label='Setpoint', alpha=0.8)
        ax1.plot(results.time, results.process_variable, 'b-', linewidth=2, label='Process Variable')
        ax1.set_ylabel('Output', fontsize=12)
        ax1.grid(True, alpha=0.3)
        ax1.legend(fontsize=10)
        ax1.set_title('System Response', fontsize=14)

        ax2.plot(results.time, results.control_output, 'g-', linewidth=2, label='Control Output')
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Control Signal', fontsize=12)
        ax2.grid(True, alpha=0.3)
        ax2.legend(fontsize=10)
        ax2.set_title('Control Signal', fontsize=14)

        if show_components:
            ax3.plot(results.time, results.p_component, 'r-', linewidth=1.5, label='P Component', alpha=0.7)
            ax3.plot(results.time, results.i_component, 'g-', linewidth=1.5, label='I Component', alpha=0.7)
            ax3.plot(results.time, results.d_component, 'b-', linewidth=1.5, label='D Component', alpha=0.7)
            ax3.set_xlabel('Time (s)', fontsize=12)
            ax3.set_ylabel('Component Value', fontsize=12)
            ax3.grid(True, alpha=0.3)
            ax3.legend(fontsize=10)
            ax3.set_title('PID Components', fontsize=14)

        plt.tight_layout()
        self._finalize_plot(save_path, title)

    def compare_responses(self, results_list: List[SimulationResults], labels: List[str],
                          title: str = "Response Comparison", save_path: Optional[str] = None) -> None:
        """Compare multiple simulation results on the same axes."""
        if len(results_list) != len(labels):
            raise ValueError("Number of results must match number of labels")

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figsize, gridspec_kw={'height_ratios': [2, 1]})
        fig.suptitle(title, fontsize=16, fontweight='bold')

        colors = plt.cm.tab10(range(len(results_list)))
        if results_list:
            ax1.plot(results_list[0].time, results_list[0].setpoint, 'k--', linewidth=2, label='Setpoint', alpha=0.8)

        for i, (results, label) in enumerate(zip(results_list, labels)):
            color = colors[i]
            ax1.plot(results.time, results.process_variable, color=color, linewidth=2, label=label)
            ax2.plot(results.time, results.control_output, color=color, linewidth=2, label=label)

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
        self._finalize_plot(save_path, title)


def create_summary_report(results: SimulationResults, filename: str = "simulation_report.txt") -> None:
    summary = results.get_summary()
    with open(filename, 'w') as f:
        f.write("PID Simulation Report\n")
        f.write("=" * 50 + "\n\n")
        f.write(f"Simulation Duration: {results.time[-1]:.2f} seconds\n")
        f.write(f"Number of Data Points: {len(results.time)}\n")
        f.write(f"Sample Rate: {len(results.time) / results.time[-1]:.1f} Hz\n\n")

        f.write("Performance Metrics:\n")
        f.write("-" * 20 + "\n")
        for key, value in summary.items():
            if value is not None:
                if isinstance(value, float):
                    f.write(f"{key.replace('_', ' ').title()}: {value:.6f}\n")
                else:
                    f.write(f"{key.replace('_', ' ').title()}: {value}\n")

        f.write("\nFinal Values:\n")
        f.write("-" * 15 + "\n")
        f.write(f"Final Setpoint: {results.setpoint[-1]:.4f}\n")
        f.write(f"Final Process Variable: {results.process_variable[-1]:.4f}\n")
        f.write(f"Final Control Output: {results.control_output[-1]:.4f}\n")
        f.write(f"Final Error: {results.error[-1]:.4f}\n")

    print(f"Summary report saved to: {filename}")


def plot_simple_response(time_data: List[float], setpoint_data: List[float], pv_data: List[float],
                         title: str = "Control Response") -> None:
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

    try:
        backend = plt.get_backend().lower()
    except Exception:
        backend = 'agg'

    non_interactive = any(b in backend for b in ('agg', 'pdf', 'svg', 'ps', 'cairo'))
    if non_interactive:
        os.makedirs('plots', exist_ok=True)
        fname = os.path.join('plots', f"{re.sub(r'[^A-Za-z0-9_\-\. ]+', '', title).strip().replace(' ', '_')}.png")
        try:
            plt.savefig(fname, dpi=300, bbox_inches='tight')
            print(f"Plot saved to: {fname} (backend={plt.get_backend()})")
        except Exception as e:
            print(f"Failed to save simple response plot: {e}")
    else:
        try:
            plt.show()
        except Exception as e:
            print(f"Could not display simple response plot: {e}")
            os.makedirs('plots', exist_ok=True)
            fname = os.path.join('plots', f"{re.sub(r'[^A-Za-z0-9_\-\. ]+', '', title).strip().replace(' ', '_')}.png")
            try:
                plt.savefig(fname, dpi=300, bbox_inches='tight')
                print(f"Plot saved to: {fname} (fallback)")
            except Exception as e2:
                print(f"Failed to save fallback simple response plot: {e2}")
