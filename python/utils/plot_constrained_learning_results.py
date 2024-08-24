import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Enable LaTeX font rendering
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

# Define the parameters
# eta_duals = [0.1, 1.0, 10.0, 100.0]
# T_0s = [25, 50, 75, 100]
eta_duals = [1]
T_0s = [25]
envs = list(range(100))  # Assuming this corresponds to the environment IDs
eval_dir = sys.argv[1]  # Path to the directory containing the evaluation results

# Set fixed axis ranges
x_axis_range = (0, 3000)  # Assuming 3000 columns as per your description
y_axis_range = (0, .3)  # Adjust this based on the expected range of max values

# Initialize a figure for the grid of plots
fig, axes = plt.subplots(len(eta_duals), len(T_0s), figsize=(24, 24))
# fig.suptitle('Max Objective Values for Different $\\eta$ and $T_0$', fontsize=24)

# Iterate over eta_duals and T_0s to read data and plot
for i, eta_dual in enumerate(eta_duals):
    for j, T_0 in enumerate(T_0s):
        # Initialize a list to store max values for each environment
        all_max_values = []
        all_obj_values = []
        
        for env_id in envs:
            # Construct the file path
            file_path = f"{eval_dir}/{eta_dual}_{T_0}/obj_values_{env_id}.csv"
            
            # Check if the file exists
            if os.path.exists(file_path):
                # Load the obj_values from the CSV file
                obj_values = np.loadtxt(file_path, delimiter=",")
                all_obj_values.append(obj_values)
                
                # Compute the max value for each column
                max_values = np.max(obj_values, axis=0)
                
                # Append to the list of max values
                all_max_values.append(max_values)
                
                # Plot individual environment max values with light, thin lines
                if len(eta_duals) > 1 and len(T_0s) > 1:
                    ax = axes[i, j]
                elif len(eta_duals) > 1:
                    ax = axes[j]
                elif len(T_0s) > 1:
                    ax = axes[i]
                else:
                    ax = axes
                ax.plot(max_values, color='lightgray', linewidth=0.5)
        
        # Plot the average max values across all environments
        if all_max_values:
            avg_max_values = np.mean(all_max_values, axis=0)
            median_max_values = np.median(all_max_values, axis=0)
            interquartile_range = np.percentile(all_max_values, 75, axis=0) - np.percentile(all_max_values, 25, axis=0)
            ax.plot(avg_max_values, linewidth=2, label='Average')
            ax.plot(median_max_values, color='red', linewidth=2, label='Median')
            ax.fill_between(range(len(avg_max_values)), median_max_values - interquartile_range, median_max_values + interquartile_range, color='red', alpha=0.2, label='Interquartile Range')
            ax.legend(fontsize=20)

            # Set plot title and axis ranges
            # ax.set_xlim(x_axis_range)
            ax.set_ylim(y_axis_range)
            ax.set_title(rf'$\eta$: {eta_dual}, $T_0$: {T_0}', fontsize=32)
        ax.tick_params(axis='both', which='major', labelsize=30)

# Adjust layout to prevent overlap
plt.tight_layout()
plt.subplots_adjust(top=0.95)
# plt.show()
plt.savefig(sys.argv[2])  # Save the figure to a file
