#!/bin/env python

import os
import pathlib
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import sys  # Import sys to handle command-line arguments

class CostAnalyzer:
    def __init__(self, base_dir, csv_file_name="eval.csv"):
        self.base_dir_path = self.str2path(base_dir)
        if not self.base_dir_path.exists():
            print(f"Error: The directory {self.base_dir_path} does not exist.")
            sys.exit(1)
        self.controller_dirs = self.find_subdirectories(self.base_dir_path)
        self.csv_file_name = csv_file_name
        # Catppuccin colors
        self.colors = [
            'rgba(239, 159, 118, 1.0)',  # Peach
            'rgba(166, 209, 137, 1.0)',  # Green
            'rgba(202, 158, 230, 1.0)',  # Mauve
            'rgba(133, 193, 220, 1.0)',  # Sapphire
            'rgba(231, 130, 132, 1.0)',  # Red
            'rgba(129, 200, 190, 1.0)',  # Teal
            'rgba(242, 213, 207, 1.0)',  # Rosewater
            'rgba(229, 200, 144, 1.0)',  # Yellow
            'rgba(108, 111, 133, 1.0)',  # subtext0
        ]
        self.num_controllers = len(self.controller_dirs)
        self.num_envs = 0
        self.num_steps = 0
        self.all_costs = None
        self.best_envs = None

    @staticmethod
    def str2path(s):
        """Convert a string path to a pathlib.Path object with expanded user and variables."""
        return pathlib.Path(os.path.expandvars(os.path.expanduser(s)))

    def find_subdirectories(self, base_path):
        """Find all subdirectories in the given base directory."""
        subdirs = [d.name for d in base_path.iterdir() if d.is_dir()]
        # Arrange subdirectories in alphabetical order
        subdirs.sort()
        print(f"Found subdirectories: {subdirs}")
        return subdirs

    def load_and_normalize_costs(self):
        """Load and normalize the costs from CSV files, store in a dictionary."""
        costs_dict = {}
        for controller_dir in self.controller_dirs:
            csv_file_path = self.base_dir_path / controller_dir / self.csv_file_name
            costs = np.loadtxt(csv_file_path, delimiter=",")
            costs = costs / costs[:, 0][:, None]  # Normalize each row by the first element
            costs_dict[controller_dir] = costs
        self.num_envs = costs_dict[self.controller_dirs[0]].shape[0]
        self.num_steps = costs_dict[self.controller_dirs[0]].shape[1]
        self.all_costs = np.zeros((self.num_controllers, self.num_envs, self.num_steps))
        for idx, controller_dir in enumerate(self.controller_dirs):
            self.all_costs[idx] = costs_dict[controller_dir]
        return costs_dict

    def compute_best_num_envs(self, costs_dict):
        """Compute the best number of environments for each controller over time."""
        all_costs_temp = self.all_costs.copy()
        # Find index of ClairvoyantCVT controller, if it exists, if not ignore it
        clair_str = "ClairvoyantCVT"
        clairvoyant_idx = self.controller_dirs.index(clair_str) if clair_str in self.controller_dirs else None
        if clairvoyant_idx is not None:
            all_costs_temp[clairvoyant_idx] = np.inf
        controller_indices = np.arange(self.num_controllers)[:, None, None]
        self.best_envs = (controller_indices == np.argmin(all_costs_temp, axis=0)[None, ...]).sum(axis=1)
        self.best_envs[:, 0] = self.num_envs // (self.num_controllers - 1)

    def plot_costs(self, costs_dict):
        """Plot the normalized costs over time for each controller."""
        fig = make_subplots(rows=3, cols=1, vertical_spacing=0.05, shared_xaxes=True, specs=[[{'rowspan': 2}], [{}], [{}]])
        for idx, controller_dir in enumerate(self.controller_dirs):
            costs = costs_dict[controller_dir]
            mean_cost = np.mean(costs, axis=0)
            std_cost = np.std(costs, axis=0)
            time_steps = np.arange(costs.shape[1])
            color = self.colors[idx % len(self.colors)]  # Cycle through colors
            
            # Shaded area for standard deviation
            fig.add_trace(go.Scatter(
                x=np.concatenate([time_steps, time_steps[::-1]]),
                y=np.concatenate([mean_cost + std_cost, (mean_cost - std_cost)[::-1]]),
                fill="toself",
                fillcolor=color.replace('1.0', '0.2'),
                line=dict(color='rgba(255,255,255,0)'),
                legendgroup=controller_dir,
                showlegend=False,
            ),
                          row=1, col=1)

        for idx, controller_dir in enumerate(self.controller_dirs):
            costs = costs_dict[controller_dir]
            mean_cost = np.mean(costs, axis=0)
            std_cost = np.std(costs, axis=0)
            time_steps = np.arange(costs.shape[1])
            color = self.colors[idx % len(self.colors)]  # Cycle through colors

            best_envs = self.best_envs[idx]
            
            # Mean cost line
            fig.add_trace(go.Scatter(
                x=time_steps,
                y=mean_cost,
                mode="lines",
                name="",
                line=dict(color=color),
                legendgroup=controller_dir,
                legendgrouptitle_text=controller_dir,
            ),
                          row=1, col=1)

            fig.add_trace(go.Scatter(
                x=time_steps,
                y=best_envs,
                mode="lines",
                showlegend=False,
                line=dict(color=color),
                legendgroup=controller_dir,
            ),
                          row=3, col=1)
            

        # Update plot layout
        fig.update_layout(
            yaxis_title="Normalized cost",
            legend=dict(
                # orientation="h",
                # xanchor="right",
                x=1,
                y=1,
                bgcolor="rgba(255, 255, 255, 0.8)"

            ),
            yaxis3_title="Number of Best Environments",
            xaxis3_title="Time Step",
        )

        return fig

    def run_analysis(self):
        """Load data, generate plots, and output results."""
        costs_dict = self.load_and_normalize_costs()
        self.compute_best_num_envs(costs_dict)
        fig = self.plot_costs(costs_dict)
        fig.write_html(self.base_dir_path / "costs_time.html")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script_name.py <base_dir>")
        sys.exit(1)
    base_dir = sys.argv[1]
    analyzer = CostAnalyzer(base_dir)
    analyzer.run_analysis()
