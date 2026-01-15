import tkinter as tk
from tkinter import ttk
import os
import subprocess
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.cm import get_cmap
from matplotlib.ticker import MultipleLocator

class PlotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("PlotJuggler GUI")
        self.root.geometry("1000x600")
        self.last_file_state = {}
        
        # Store current data for click events
        self.current_data = None
        self.current_filtered_data = None

        # Log directory
        self.log_dir = "./tools/param_tuner/logs/"
        self.profile_path = "./tools/param_tuner/profile.xml"

        # Main Layout (PanedWindow)
        self.paned_window = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True)

        # Left Frame (File List)
        self.left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(self.left_frame, weight=1)

        # Right Frame (Plot)
        self.right_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(self.right_frame, weight=2)

        # --- Left Frame Content ---
        self.list_label = ttk.Label(self.left_frame, text="Log Files")
        self.list_label.pack(side=tk.TOP, pady=5)

        self.scrollbar = ttk.Scrollbar(self.left_frame)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(self.left_frame, columns=("Filename", "Date"), show="headings", yscrollcommand=self.scrollbar.set)
        self.tree.heading("Filename", text="Filename")
        self.tree.heading("Date", text="Date")
        self.tree.column("Filename", width=200)
        self.tree.column("Date", width=120)
        self.tree.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        self.scrollbar.config(command=self.tree.yview)

        self.btn_frame = ttk.Frame(self.left_frame)
        self.btn_frame.pack(fill=tk.X, padx=5, pady=5)

        self.refresh_btn = ttk.Button(self.btn_frame, text="Refresh", command=self.load_files)
        self.refresh_btn.pack(side=tk.LEFT, padx=2)

        self.pj_btn = ttk.Button(self.btn_frame, text="Open PlotJuggler", command=self.run_plotjuggler)
        self.pj_btn.pack(side=tk.RIGHT, padx=2)

        self.kill_pj_btn = ttk.Button(self.btn_frame, text="Kill PJ", command=self.kill_plotjuggler)
        self.kill_pj_btn.pack(side=tk.RIGHT, padx=2)

        self.show_output_var = tk.BooleanVar(value=True)
        self.output_chk = ttk.Checkbutton(self.btn_frame, text="Show Output", variable=self.show_output_var)
        self.output_chk.pack(side=tk.RIGHT, padx=5)

        # Sensor visibility controls
        self.show_left45_var = tk.BooleanVar(value=True)
        self.left45_chk = ttk.Checkbutton(self.btn_frame, text="Left45", variable=self.show_left45_var, command=self.on_sensor_toggle)
        self.left45_chk.pack(side=tk.RIGHT, padx=5)

        self.show_right45_var = tk.BooleanVar(value=True)
        self.right45_chk = ttk.Checkbutton(self.btn_frame, text="Right45", variable=self.show_right45_var, command=self.on_sensor_toggle)
        self.right45_chk.pack(side=tk.RIGHT, padx=5)

        self.status_label = ttk.Label(self.left_frame, text="Ready", wraplength=300)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=2)

        # --- Right Frame Content ---
        self.figure = plt.figure(figsize=(5, 4), dpi=100, facecolor='black')
        # plt.style.use('dark_background') # This changes style globally, better set params locally or context
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.right_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Bindings
        self.tree.bind("<<TreeviewSelect>>", self.on_file_select)
        self.tree.bind("<Double-1>", self.run_plotjuggler)

        self.load_files()
        self.auto_refresh()

    def auto_refresh(self):
        self.load_files(check_updates=True)
        self.root.after(1000, self.auto_refresh)

    def load_files(self, check_updates=False):
        try:
            files = glob.glob(os.path.join(self.log_dir, "*.csv"))
            current_state = {f: os.path.getmtime(f) for f in files}

            if check_updates and current_state == self.last_file_state:
                return

            self.last_file_state = current_state

            for item in self.tree.get_children():
                self.tree.delete(item)

            files.sort(key=os.path.getmtime, reverse=True)
            for f in files:
                filename = os.path.basename(f)
                mod_time = os.path.getmtime(f)
                from datetime import datetime
                date_str = datetime.fromtimestamp(mod_time).strftime('%Y-%m-%d %H:%M')
                self.tree.insert("", tk.END, values=(filename, date_str))
            
            # Select first item if available
            if self.tree.get_children():
                first = self.tree.get_children()[0]
                self.tree.selection_set(first)
                self.plot_file(os.path.join(self.log_dir, self.tree.item(first)['values'][0]))

            self.status_label.config(text=f"Loaded {len(files)} files.")
        except Exception as e:
            self.status_label.config(text=f"Error loading files: {e}")

    def on_file_select(self, event):
        selected_item = self.tree.selection()
        if not selected_item:
            return
        filename = self.tree.item(selected_item)['values'][0]
        file_path = os.path.join(self.log_dir, filename)
        self.plot_file(file_path)

    def on_sensor_toggle(self):
        """Re-plot when sensor visibility is toggled"""
        selected_item = self.tree.selection()
        if not selected_item:
            return
        filename = self.tree.item(selected_item)['values'][0]
        file_path = os.path.join(self.log_dir, filename)
        self.plot_file(file_path)

    def plot_file(self, file_path):
        self.figure.clear()
        sen_min = 20
        sen_max = 80.5
        sensor_x_offset = 25.0  # mm forward from robot center
        sensor_angle_deg = 58  # sensor mounting angle in degrees
        sensor_angle_rad = np.radians(sensor_angle_deg)
        deviation_deg_th = 25 # deg
        try:
            # Logic ported from trajectory_plot.py
            with plt.style.context('dark_background'):
                ax = self.figure.add_subplot(111)
                
                data = pd.read_csv(file_path)
                
                if 'x' in data.columns and 'y' in data.columns:
                    # Sort by timestamp first, then by index (if available) to preserve chronological order
                    if 'index' in data.columns:
                        data = data.sort_values(by=['timestamp', 'index']).reset_index(drop=True)
                    else:
                        # If no index column, sort only by timestamp and preserve original order within each timestamp
                        data = data.sort_values(by='timestamp', kind='stable').reset_index(drop=True)
                    
                    # Use ang_kf_sum if available (no reset across motions), otherwise fallback to ang_kf
                    if 'ang_kf_sum' in data.columns:
                        # ang_kf_sum is already cumulative across motions, just convert to radians
                        data['angle_corrected'] = np.radians(data['ang_kf_sum'])
                    elif 'ang_kf' in data.columns:
                        # Fallback: Calculate cumulative angle from ang_kf with timestamp correction
                        cumulative_angle = np.zeros(len(data))
                        angle_offset = 0.0
                        prev_timestamp = None

                        for idx in range(len(data)):
                            current_timestamp = data['timestamp'].iloc[idx]
                            ang_kf_deg = data['ang_kf'].iloc[idx]
                            ang_kf_rad = np.radians(ang_kf_deg)  # Convert to radians

                            # When timestamp changes, check if we need to reset based on ideal_ang deviation
                            if prev_timestamp is not None and current_timestamp != prev_timestamp:
                                # Check if ideal_ang exists and if deviation is > 5 degrees
                                if 'ideal_ang' in data.columns:
                                    ideal_ang_deg = data['ideal_ang'].iloc[idx]
                                    ideal_ang_rad = np.radians(ideal_ang_deg)

                                    # Calculate deviation between current ang_kf and ideal_ang
                                    deviation_deg = abs(ang_kf_deg - ideal_ang_deg)

                                    # Only reset if deviation > deviation_deg_th degrees
                                    if deviation_deg > deviation_deg_th:
                                        # Reset: don't carry over the previous angle
                                        angle_offset = 0.0
                                    else:
                                        # Continue: carry over the previous cumulative angle
                                        angle_offset = cumulative_angle[idx - 1]
                                else:
                                    # If no ideal_ang column, use previous behavior (always carry over)
                                    angle_offset = cumulative_angle[idx - 1]

                            cumulative_angle[idx] = ang_kf_rad + angle_offset
                            prev_timestamp = current_timestamp

                        data['angle_corrected'] = cumulative_angle
                    
                    # Store data for click events
                    self.current_data = data
                    
                    if len(data) > 2:
                        last_motion_state = data['timestamp'].iloc[-2]
                        # filtered_data = data[data['timestamp'] != last_motion_state]
                        filtered_data = data[data['timestamp'].diff().fillna(0) >= 0]
                        self.current_filtered_data = filtered_data
                        
                        unique_states = filtered_data['timestamp'].unique()
                        num_states = len(unique_states)
                        cmap = get_cmap('viridis', num_states) if num_states > 0 else 'viridis'

                        for i, (state, group) in enumerate(filtered_data.groupby('timestamp')):
                            pos_x = group['x'] + 45 - 9
                            pos_y = group['y']
                            color = cmap(i / num_states) if num_states > 0 else 'cyan'
                            ax.plot(pos_x.to_numpy(), pos_y.to_numpy(), ".", markersize=4, color=color, label=f'State {state}')
                            
                            # Plot left45_d sensor positions if available and enabled
                            if self.show_left45_var.get() and 'left45_d' in group.columns and 'angle_corrected' in group.columns:
                                left45_d = group['left45_d'].to_numpy()
                                angle = group['angle_corrected'].to_numpy()
                                x = group['x'].to_numpy()
                                y = group['y'].to_numpy()

                                # Filter: only plot when left45_d is between sen_min and sen_max
                                valid_mask = (left45_d >= sen_min) & (left45_d < sen_max)

                                if np.any(valid_mask):
                                    # Calculate left45_d sensor coordinates
                                    # Sensor is mounted at 29.7mm forward, at 58deg angle
                                    # y / x = tan(58deg), so x = y / tan(58deg)
                                    sensor_y = left45_d[valid_mask]  # perpendicular distance to wall
                                    sensor_x = sensor_y / np.tan(sensor_angle_rad)  # forward distance from sensor mount

                                    # Sensor mount position (29.7mm forward from robot center)
                                    sensor_mount_x = x[valid_mask] + sensor_x_offset * np.cos(angle[valid_mask])
                                    sensor_mount_y = y[valid_mask] + sensor_x_offset * np.sin(angle[valid_mask])

                                    # Wall position in robot frame: forward by sensor_x, left by sensor_y
                                    # Transform to global frame
                                    left45_d_x = sensor_mount_x + sensor_x * np.cos(angle[valid_mask]) - sensor_y * np.sin(angle[valid_mask]) + 45 - 9
                                    left45_d_y = sensor_mount_y + sensor_x * np.sin(angle[valid_mask]) + sensor_y * np.cos(angle[valid_mask])

                                    # Plot sensor positions with different marker
                                    ax.plot(left45_d_x, left45_d_y, "o", markersize=6,
                                           color=color, alpha=0.6, markeredgecolor='white',
                                           markeredgewidth=0.5)
                            
                            # Plot right45_d sensor positions if available and enabled
                            if self.show_right45_var.get() and 'right45_d' in group.columns and 'angle_corrected' in group.columns:
                                right45_d = group['right45_d'].to_numpy()
                                angle = group['angle_corrected'].to_numpy()
                                x = group['x'].to_numpy()
                                y = group['y'].to_numpy()

                                # Filter: only plot when right45_d is between sen_min and sen_max
                                valid_mask = (right45_d >= sen_min) & (right45_d < sen_max)

                                if np.any(valid_mask):
                                    # Calculate right45_d sensor coordinates
                                    # Sensor is mounted at 29.7mm forward, at 58deg angle
                                    # y / x = tan(58deg), so x = y / tan(58deg)
                                    sensor_y = right45_d[valid_mask]  # perpendicular distance to wall
                                    sensor_x = sensor_y / np.tan(sensor_angle_rad)  # forward distance from sensor mount

                                    # Sensor mount position (29.7mm forward from robot center)
                                    sensor_mount_x = x[valid_mask] + sensor_x_offset * np.cos(angle[valid_mask])
                                    sensor_mount_y = y[valid_mask] + sensor_x_offset * np.sin(angle[valid_mask])

                                    # Wall position in robot frame: forward by sensor_x, right by sensor_y (negative y)
                                    # Transform to global frame
                                    right45_d_x = sensor_mount_x + sensor_x * np.cos(angle[valid_mask]) + sensor_y * np.sin(angle[valid_mask]) + 45 - 9
                                    right45_d_y = sensor_mount_y + sensor_x * np.sin(angle[valid_mask]) - sensor_y * np.cos(angle[valid_mask])

                                    # Plot sensor positions with different marker (square for right sensor)
                                    ax.plot(right45_d_x, right45_d_y, "s", markersize=6,
                                           color=color, alpha=0.6, markeredgecolor='white',
                                           markeredgewidth=0.5)
                    
                    ax.set_title('Position Plot (with Left45 & Right45 Sensors)', color='white')
                    ax.set_xlabel('x', color='white')
                    ax.set_ylabel('y', color='white')
                    ax.grid(True, color='gray', linestyle='--', linewidth=0.5)
                    ax.xaxis.set_major_locator(MultipleLocator(45))
                    ax.yaxis.set_major_locator(MultipleLocator(45))

                    # Wall drawing logic from trajectory_plot.py
                    x_min, x_max = min(data['x']), max(data['x'])
                    y_min, y_max = min(data['y']), max(data['y'])
                    y_max_abs = max(abs(data['y']))

                    alpha = 0.25
                    lw = 1.5 # Adjusted for smaller view

                    for i in range(0, int(x_max+90), 90):
                        if y_max_abs > 100 and y_min < 0:
                            ax.plot([i, i], [45, y_min-45], color=(1, 0, 0, alpha), linewidth=lw)
                        else:
                            ax.plot([i, i], [45, y_max+45], color=(1, 0, 0, alpha), linewidth=lw)

                    for i in range(0, int(max(-y_min, y_max)+90), 90):
                         if y_max_abs > 100 and y_min < 0:
                             ax.plot([0, x_max+90], [-i+45, -i+45], color=(1, 0, 0, alpha), linewidth=lw)
                         else:
                             ax.plot([0, x_max+90], [i+45, i+45], color=(1, 0, 0, alpha), linewidth=lw)

                    ax.axis('equal')
                else:
                    ax.text(0.5, 0.5, "No x/y data found", ha='center', va='center', color='red')

            self.figure.tight_layout()
            self.canvas.draw()
            
            # Connect click event
            self.canvas.mpl_connect('button_press_event', self.on_plot_click)

        except Exception as e:
            self.status_label.config(text=f"Plot Error: {e}")
            print(e)
            # Draw empty if error
            self.figure.clear()
            self.canvas.draw()

    def on_plot_click(self, event):
        """Handle click events on the plot to display point information"""
        if event.inaxes is None or self.current_data is None:
            return
        
        # Get click coordinates
        click_x = event.xdata
        click_y = event.ydata
        
        if click_x is None or click_y is None:
            return
        
        try:
            # Find nearest point in the data
            # Account for the offset applied in plotting (+ 45 - 9)
            data_x = self.current_data['x'] + 45 - 9
            data_y = self.current_data['y']
            
            # Calculate distances to all points
            distances = np.sqrt((data_x - click_x)**2 + (data_y - click_y)**2)
            nearest_idx = distances.idxmin()
            
            # Get the nearest point data
            nearest_point = self.current_data.iloc[nearest_idx]
            
            # Print information to console (concise)
            print(f"\n[{nearest_idx}] ts={nearest_point.get('timestamp', 'N/A')} | ", end="")
            print(f"pos=({nearest_point['x']:.1f}, {nearest_point['y']:.1f}) | ", end="")
            
            if 'ang_kf' in nearest_point:
                print(f"ang_kf={nearest_point['ang_kf']:.1f}° | ", end="")
            
            if 'angle_corrected' in nearest_point:
                print(f"ang_corrected={np.degrees(nearest_point['angle_corrected']):.1f}° | ", end="")
            
            if 'left45_d' in nearest_point:
                print(f"left45_d={nearest_point['left45_d']:.1f}", end="")
                
                # Calculate and show sensor position
                if 'angle_corrected' in nearest_point:
                    angle = nearest_point['angle_corrected']
                    left45_d = nearest_point['left45_d']
                    x = nearest_point['x']
                    y = nearest_point['y']
                    
                    left45_d_x = x + left45_d * np.cos(angle + np.pi/2) + 45 - 9
                    left45_d_y = y + left45_d * np.sin(angle + np.pi/2)
                    
                    print(f" → ({left45_d_x:.1f}, {left45_d_y:.1f})", end="")
            
            print()  # New line
            
        except Exception as e:
            print(f"Error in click handler: {e}")

    def run_plotjuggler(self, event=None):
        selected_item = self.tree.selection()
        if not selected_item:
            return
        filename = self.tree.item(selected_item)['values'][0]
        file_path = os.path.join(self.log_dir, filename)
        
        cmd = [
            "ros2", "run", "plotjuggler", "plotjuggler",
            "-d", file_path,
            "-l", self.profile_path
        ]
        self.status_label.config(text=f"Opening {filename} in PlotJuggler...")
        self.status_label.config(text=f"Opening {filename} in PlotJuggler...")
        try:
            if self.show_output_var.get():
                subprocess.Popen(cmd)
            else:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.status_label.config(text=f"Error: {e}")

    def kill_plotjuggler(self):
        try:
            subprocess.run(["pkill", "-f", "plotjuggler"])
            self.status_label.config(text="Killed all PlotJuggler instances.")
        except Exception as e:
            self.status_label.config(text=f"Error killing PlotJuggler: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = PlotGUI(root)
    root.mainloop()
