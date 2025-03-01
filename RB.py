import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Class definition for scheduling reactors based on available power
class ReactorScheduler:
    def __init__(self, num_reactors, interval, max_power):
        self.num_reactors = num_reactors
        self.max_power = max_power  # Maximum available power
        self.reactor_minutes = [0 for _ in range(num_reactors)]  # Track reactor time in minutes
        self.current_index = 0
        self.interval = interval  # Set the interval dynamically
        self.running_reactors = []  # Store the number of running reactors for each interval
        self.total_energy_consumed = 0  # Total energy consumed by reactors
    
    def get_operational_reactors(self, available_power):
        # Adjust the logic based on the percentage of available power
        if available_power < 10:
            return 0
        elif available_power < 20:
            return 1
        elif available_power < 30:
            return 2
        elif available_power < 40:
            return 3
        elif available_power < 50:
            return 4
        elif available_power < 60:
            return 5
        elif available_power < 70:
            return 6
        elif available_power < 80:
            return 7
        elif available_power < 90:
            return 8
        elif available_power < 100:  # Correctly set to < 100
            return 9
        else:
            return 10
    
    def update_reactor_minutes(self, num_active_reactors):
        # Power consumption per reactor is 10% of max_power
        reactor_power_consumption = 0.1 * self.max_power
        
        # Energy consumed by the reactors during this interval
        energy_consumed = num_active_reactors * reactor_power_consumption * (self.interval / 60)
        self.total_energy_consumed += energy_consumed  # Add to total energy consumed by all reactors

        for i in range(num_active_reactors):
            reactor_index = (self.current_index + i) % self.num_reactors
            self.reactor_minutes[reactor_index] += self.interval  # Increment by the adjustable interval
        self.current_index = (self.current_index + num_active_reactors) % self.num_reactors
        self.running_reactors.append(num_active_reactors)  # Track the number of running reactors for plotting
    
    def schedule_reactors(self, power_readings):
        # Simulate reactor operation based on the averaged power readings
        for available_power in power_readings:
            num_active_reactors = self.get_operational_reactors(available_power)
            self.update_reactor_minutes(num_active_reactors)
    
    def print_reactor_minutes(self):
        # Print the total minutes each reactor has operated
        for i, minutes in enumerate(self.reactor_minutes):
            print(f"Reactor {i+1}: {minutes:.0f} minutes")

    def calculate_efficiency(self, total_solar_power):
        # Calculate the energy utilization efficiency
        if total_solar_power == 0:
            return 0
        efficiency = self.total_energy_consumed / total_solar_power
        return efficiency

# Define the interval (e.g., check every X minutes)
interval_minutes = 20  # Adjust this value for the desired interval in minutes

# Load the photovoltaic power data
file_path = 'onemin-Ground-2017-06-04.csv'  # Adjust the file path as necessary
data = pd.read_csv(file_path)

# Convert the TIMESTAMP column to datetime
data['TIMESTAMP'] = pd.to_datetime(data['TIMESTAMP'])

# Set the TIMESTAMP column as the index to facilitate resampling
data.set_index('TIMESTAMP', inplace=True)

# Resample the data to the specified interval and calculate the average of InvPDC_kW_Avg
resampling_rule = f'{interval_minutes}min'  # Dynamic interval for resampling
resampled_data = data.resample(resampling_rule).mean()

# Reset the index so that TIMESTAMP is a column again
resampled_data.reset_index(inplace=True)

# Extract the resampled time and DC power columns
resampled_time = resampled_data['TIMESTAMP']
resampled_dc_power_kw = resampled_data['InvPDC_kW_Avg']
print(resampled_dc_power_kw.iloc[40])
print(resampled_dc_power_kw[0:5], len(resampled_dc_power_kw))
# Function to calculate energy utilization efficiency for a given x
def calculate_efficiency_for_x(x, interval_minutes, resampled_dc_power_kw):
    # Calculate max_power and power percentages
    max_power = resampled_dc_power_kw.max() / x
    power_percentages = (resampled_dc_power_kw / max_power) * 100
    
    # Initialize the reactor scheduler with the interval and max power
    scheduler = ReactorScheduler(10, interval_minutes, max_power)
    
    # Schedule reactors based on the resampled power readings
    scheduler.schedule_reactors(power_percentages)
    
    # Calculate total solar power generation (kWh)
    total_solar_power_generated = resampled_dc_power_kw.sum() * (interval_minutes / 60)  # Convert to kWh
    
    # Calculate energy utilization efficiency
    efficiency = scheduler.calculate_efficiency(total_solar_power_generated)
    return efficiency, scheduler

# Try a broader range of values of x and find the one that maximizes energy utilization efficiency
best_x = None
best_efficiency = 0
best_scheduler = None
x_values = np.linspace(1.0, 2.0, 50)  # 50 values of x between 1.0 and 2.0

efficiency_values = []

for x in x_values:
    efficiency, scheduler = calculate_efficiency_for_x(x, interval_minutes, resampled_dc_power_kw)
    efficiency_values.append(efficiency)
    print(f"x: {x}, Energy Utilization Efficiency: {efficiency:.2%}")
    
    if efficiency > best_efficiency:
        best_efficiency = efficiency
        best_x = x
        best_scheduler = scheduler

# Output the best x and corresponding efficiency
print(f"\nBest x: {best_x}, Highest Energy Utilization Efficiency: {best_efficiency:.2%}")

# Plot DC power and number of running reactors averaged over the specified interval vs time for the best x
fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot the averaged DC power data (line plot)
ax1.plot(resampled_time, resampled_dc_power_kw, label=f'Average DC Power (kW) every {interval_minutes} minutes', color='blue')
ax1.set_xlabel('Time')
ax1.set_ylabel('DC Power (kW)', color='blue')
ax1.tick_params(axis='y', labelcolor='blue')

# Create a secondary y-axis to plot the number of running reactors
ax2 = ax1.twinx()

# Plot the number of running reactors as a line plot
ax2.plot(resampled_time, best_scheduler.running_reactors, label='Number of Running Reactors', color='orange', linestyle='--')

# Align the scales of both axes
ax2.set_ylim(0, 1.2 * 10)  # Limit to 0-10 reactors
ax1.set_ylim(0, 1.2 * resampled_dc_power_kw.max() / best_x)  # Align with the best max power
ax2.set_ylabel('Number of Running Reactors', color='orange')
ax2.tick_params(axis='y', labelcolor='orange')

# Customize the plot
plt.title(f'DC Power and Number of Running Reactors Averaged Every {interval_minutes} Minutes (Best x = {best_x})')
fig.tight_layout()

# Show the plot for DC power and reactors
plt.show()

# Plot Energy Utilization Efficiency vs x
plt.figure(figsize=(8, 5))
plt.plot(x_values, efficiency_values, label='Energy Utilization Efficiency', color='green')
plt.xlabel('x')
plt.ylabel('Energy Utilization Efficiency')
plt.title('Energy Utilization Efficiency vs x')
plt.grid(True)
plt.tight_layout()

# Show the plot for Energy Utilization Efficiency vs x
plt.show()
