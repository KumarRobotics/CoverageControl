#!/bin/env python

import os
import pathlib
import numpy as np
import plotly.graph_objects as go
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
        return costs_dict

    def plot_costs(self, costs_dict):
        """Plot the normalized costs over time for each controller."""
        fig = go.Figure()
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
                name=controller_dir + " ± std",
                legendgroup=controller_dir,
                legendgrouptitle_text=controller_dir,
            ))

        for idx, controller_dir in enumerate(self.controller_dirs):
            costs = costs_dict[controller_dir]
            mean_cost = np.mean(costs, axis=0)
            std_cost = np.std(costs, axis=0)
            time_steps = np.arange(costs.shape[1])
            color = self.colors[idx % len(self.colors)]  # Cycle through colors
            
            # Mean cost line
            fig.add_trace(go.Scatter(
                x=time_steps,
                y=mean_cost,
                mode="lines",
                name=controller_dir,
                line=dict(color=color),
                # legendgroup="means",
                # legendgrouptitle_text="Mean costs",
                legendgroup=controller_dir,
                legendgrouptitle_text=controller_dir,
            ))
            
        # Update plot layout
        fig.update_layout(
            title="Normalized costs over time",
            xaxis_title="Time step",
            yaxis_title="Normalized cost",
            legend=dict(
                # orientation="h",
                # xanchor="right",
                x=1,
                y=1,
                bgcolor="rgba(255, 255, 255, 0.8)"
            )
        )
        return fig

    def run_analysis(self):
        """Load data, generate plots, and output results."""
        costs_dict = self.load_and_normalize_costs()
        fig = self.plot_costs(costs_dict)
        fig.write_html(self.base_dir_path / "costs_time.html")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python script_name.py <base_dir>")
        sys.exit(1)
    base_dir = sys.argv[1]
    analyzer = CostAnalyzer(base_dir)
    analyzer.run_analysis()
