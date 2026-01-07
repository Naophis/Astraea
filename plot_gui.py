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

    def plot_file(self, file_path):
        self.figure.clear()
        try:
            # Logic ported from trajectory_plot.py
            with plt.style.context('dark_background'):
                ax = self.figure.add_subplot(111)
                
                data = pd.read_csv(file_path)
                
                if 'x' in data.columns and 'y' in data.columns:
                    data = data.sort_values(by=['timestamp', 'x', 'y']).reset_index(drop=True)
                    if len(data) > 2:
                        last_motion_state = data['timestamp'].iloc[-2]
                        # filtered_data = data[data['timestamp'] != last_motion_state]
                        filtered_data = data[data['timestamp'].diff().fillna(0) >= 0]
                        
                        unique_states = filtered_data['timestamp'].unique()
                        num_states = len(unique_states)
                        cmap = get_cmap('viridis', num_states) if num_states > 0 else 'viridis'

                        for i, (state, group) in enumerate(filtered_data.groupby('timestamp')):
                            pos_x = group['x'] + 45 - 9
                            pos_y = group['y']
                            color = cmap(i / num_states) if num_states > 0 else 'cyan'
                            ax.plot(pos_x.to_numpy(), pos_y.to_numpy(), ".", markersize=4, color=color, label=f'State {state}')
                    
                    ax.set_title('Position Plot', color='white')
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

        except Exception as e:
            self.status_label.config(text=f"Plot Error: {e}")
            print(e)
            # Draw empty if error
            self.figure.clear()
            self.canvas.draw()

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
