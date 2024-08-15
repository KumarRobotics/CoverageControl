import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Enable LaTeX font rendering
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

# Define the parameters
eta_duals = [0.1, 1.0, 10.0, 100.0]
T_0s = [25, 50, 75, 100]
envs = list(range(10))  # Assuming this corresponds to the environment IDs
eval_dir = sys.argv[1]  # Path to the directory containing the evaluation results

# Set fixed axis ranges
x_axis_range = (0, 3000)  # Assuming 3000 columns as per your description
y_axis_range = (0, .3)  # Adjust this based on the expected range of max values

# Initialize a figure for the grid of plots
fig, axes = plt.subplots(len(eta_duals), len(T_0s), figsize=(24, 24))
fig.suptitle('Max Objective Values for Different $\\eta$ and $T_0$', fontsize=24)

# Iterate over eta_duals and T_0s to read data and plot
for i, eta_dual in enumerate(eta_duals):
    for j, T_0 in enumerate(T_0s):
        # Initialize a list to store max values for each environment
        all_max_values = []
        
        for env_id in envs:
            # Construct the file path
            file_path = f"{eval_dir}/{eta_dual}_{T_0}/obj_values_{env_id}.csv"
            
            # Check if the file exists
            if os.path.exists(file_path):
                # Load the obj_values from the CSV file
                obj_values = np.loadtxt(file_path, delimiter=",")
                
                # Compute the max value for each column
                max_values = np.max(obj_values, axis=0)
                
                # Append to the list of max values
                all_max_values.append(max_values)
                
                # Plot individual environment max values with light, thin lines
                ax = axes[i, j]
                ax.plot(max_values, color='lightgray', linewidth=0.5)
        
        # Plot the average max values across all environments
        if all_max_values:
            avg_max_values = np.mean(all_max_values, axis=0)
            ax.plot(avg_max_values, color='blue', linewidth=2)  # Thicker line for the average
            
            # Set plot title and axis ranges
            # ax.set_xlim(x_axis_range)
            ax.set_ylim(y_axis_range)
            ax.set_title(rf'$\eta$: {eta_dual}, $T_0$: {T_0}', fontsize=18)
        ax.tick_params(axis='both', which='major', labelsize=14)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.subplots_adjust(top=0.95)
# plt.show()
plt.savefig(sys.argv[2])  # Save the figure to a file
