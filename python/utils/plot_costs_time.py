#!/bin/env python

import os
import pathlib
import numpy as np
import plotly.graph_objects as go
import sys  # Import sys to handle command-line arguments

class CostAnalyzer:
    def __init__(self, base_dir, controller_dirs, csv_file_name="eval.csv"):
        self.base_dir_path = self.str2path(base_dir)
        self.controller_dirs = controller_dirs
        self.csv_file_name = csv_file_name
        self.colors = ['rgba(31, 119, 180, 0.8)', 'rgba(255, 127, 14, 0.8)', 
                       'rgba(44, 160, 44, 0.8)', 'rgba(214, 39, 40, 0.8)']

    @staticmethod
    def str2path(s):
        """Convert a string path to a pathlib.Path object with expanded user and variables."""
        return pathlib.Path(os.path.expandvars(os.path.expanduser(s)))

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
            
            # Mean cost line
            fig.add_trace(go.Scatter(
                x=time_steps,
                y=mean_cost,
                mode="lines",
                name=controller_dir,
                line=dict(color=color),
                legendgroup="means",
                legendgrouptitle_text="Mean costs",
            ))
            
            # Shaded area for standard deviation
            fig.add_trace(go.Scatter(
                x=np.concatenate([time_steps, time_steps[::-1]]),
                y=np.concatenate([mean_cost + std_cost, (mean_cost - std_cost)[::-1]]),
                fill="toself",
                fillcolor=color.replace('0.8', '0.2'),
                line=dict(color='rgba(255,255,255,0)'),
                name=controller_dir + " ± std",
                legendgroup="stds",
                legendgrouptitle_text="Standard deviations",
            ))

        # Update plot layout
        fig.update_layout(
            title="Normalized costs over time",
            xaxis_title="Time step",
            yaxis_title="Normalized cost",
            legend=dict(
                orientation="h",
                xanchor="center",
                x=0.5,
                y=1.15,
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
    controller_dirs = ["lpac_k1", "lpac_k2", "lpac_k3", "DecentralizedCVT", "CentralizedCVT", "ClairvoyantCVT"]
    analyzer = CostAnalyzer(base_dir, controller_dirs)
    analyzer.run_analysis()

