import numpy as np
import matplotlib.pyplot as plt
import math

from slalom import Slalom
from plot import Plot
from plotorval import PlotOrval

import sys
import os
import yaml

import tkinter
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

def read_yaml(filename):
    # 現在のスクリプトのディレクトリを取得
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # ファイルパスを結合
    filepath = os.path.join(current_dir, filename)

    with open(filepath, 'r') as file:
        data = yaml.safe_load(file)
    return data

class RedirectText(object):
    def __init__(self, text_ctrl):
        self.output = text_ctrl

    def write(self, string):
        self.output.insert(tkinter.END, string)
        self.output.see(tkinter.END)

    def flush(self):
        pass

class SlalomGUI:
    def __init__(self, master):
        self.master = master
        master.title("Slalom Simulator")
        
        self.window_width = 600
        master.geometry(f"{self.window_width}x600")

        self.data = read_yaml("../param_tuner/profile/hardware.yaml")
        self.plot = Plot()
        self.plot_orval = PlotOrval()

        # Parameters
        self.v = tkinter.IntVar(value=2100)
        self.turn_type = tkinter.StringVar(value="dia135")
        self.k = tkinter.DoubleVar(value=self.data["slip_param_k2"])
        self.k_y = tkinter.StringVar(value=str(self.data["slip_param_K"]))
        self.hf_cl = tkinter.IntVar(value=0)
        self.show_plot = tkinter.BooleanVar(value=False)

        self.create_widgets()

    def create_widgets(self):
        # Layout
        frame = ttk.Frame(self.master, padding="10")
        frame.grid(row=0, column=0, sticky=(tkinter.W, tkinter.E, tkinter.N, tkinter.S))

        # Velocity
        ttk.Label(frame, text="Velocity (v):").grid(row=0, column=0, sticky=tkinter.W)
        ttk.Entry(frame, textvariable=self.v).grid(row=0, column=1, sticky=(tkinter.W, tkinter.E))

        # Turn Type
        # Turn Type
        ttk.Label(frame, text="Turn Type:").grid(row=1, column=0, sticky=tkinter.W)
        turn_types = ["normal", "large", "orval", "dia45", "dia135", "dia45_2", "dia135_2", "dia90"]
        
        # Radio Buttons Frame
        radio_frame = ttk.Frame(frame)
        radio_frame.grid(row=1, column=1, sticky=(tkinter.W, tkinter.E))
        
        for i, t_type in enumerate(turn_types):
            rb = ttk.Radiobutton(radio_frame, text=t_type, variable=self.turn_type, value=t_type, command=self.update_radius)
            # Arrange in 2 columns
            rb.grid(row=i // 2, column=i % 2, sticky=tkinter.W, padx=5)

        # Default Radius Mapping (based on Plot class in plot.py, assuming hf_cl=0)
        self.DEFAULT_RADIUS = {
            "normal": 27,
            "large": 60.5,
            "orval": 52.25,
            "dia45": 54.0,
            "dia135": 45.0,
            "dia45_2": 52,
            "dia135_2": 45.0,
            "dia90": 42.0
        }

        # K
        ttk.Label(frame, text="Slip Param K:").grid(row=2, column=0, sticky=tkinter.W)
        ttk.Entry(frame, textvariable=self.k).grid(row=2, column=1, sticky=(tkinter.W, tkinter.E))

        # K_y (List)
        ttk.Label(frame, text="Slip Param K_y (comma sep):").grid(row=3, column=0, sticky=tkinter.W)
        ttk.Entry(frame, textvariable=self.k_y).grid(row=3, column=1, sticky=(tkinter.W, tkinter.E))
        
        # HF CL
        ttk.Label(frame, text="HF CL:").grid(row=4, column=0, sticky=tkinter.W)
        ttk.Entry(frame, textvariable=self.hf_cl).grid(row=4, column=1, sticky=(tkinter.W, tkinter.E))

        # Radius (Optional)
        ttk.Label(frame, text="Radius (Optional):").grid(row=5, column=0, sticky=tkinter.W)
        self.rad = tkinter.StringVar(value="")
        ttk.Entry(frame, textvariable=self.rad).grid(row=5, column=1, sticky=(tkinter.W, tkinter.E))

        # Show Plot Checkbox
        ttk.Checkbutton(frame, text="Show Plot", variable=self.show_plot).grid(row=6, column=0, columnspan=2, sticky=tkinter.W)

        # Plot Button
        ttk.Button(frame, text="Plot", command=self.run_plot).grid(row=7, column=0, columnspan=2, pady=10)

        # Output Log
        ttk.Label(frame, text="Output Log:").grid(row=8, column=0, sticky=tkinter.W)
        self.log_text = ScrolledText(frame, height=10, width=80)
        self.log_text.grid(row=9, column=0, columnspan=2, sticky=(tkinter.W, tkinter.E))

        # Redirect stdout
        sys.stdout = RedirectText(self.log_text)
        
        # Initialize radius
        self.update_radius()

    def update_radius(self, event=None):
        t_type = self.turn_type.get()
        if t_type in self.DEFAULT_RADIUS:
            self.rad.set(str(self.DEFAULT_RADIUS[t_type]))

    def run_plot(self):
        # Clear previous log
        self.log_text.delete(1.0, tkinter.END)
        
        v = self.v.get()
        t_type = self.turn_type.get()
        k = self.k.get()
        try:
            # Parse list from string
            k_y_str = self.k_y.get()
            list_k_y = [float(x.strip()) for x in k_y_str.split(',')]
        except ValueError:
            print("Invalid format for K_y")
            return

        hf_cl = self.hf_cl.get()
        
        # Parse Radius
        rad_val = None
        rad_str = self.rad.get().strip()
        if rad_str:
            try:
                rad_val = float(rad_str)
            except ValueError:
                print("Invalid format for Radius")
                return

        offset = {
            "prev": 7,
            "after": 7,
            "prev_dia": 7,
            "after_dia": 7,
        }

        show = self.show_plot.get()

        if t_type == "orval":
             # Assuming PlotOrval has a similar interface or handling it via Plot if it supports it.
             # The original code had `p.exe("orval", ...)` commented out and `po = PlotOrval()`.
             # Looking at plot.py, it handles "orval" type in `exe`.
             # Let's stick to using `self.plot.exe` as it seems to handle "orval" too based on line 45 of plot.py.
             self.plot.exe(t_type, v, show, 0, k, list_k_y, offset, hf_cl, rad=rad_val)
        else:
             # dia45_mode was 0 in original code, passing 0 for mode
             self.plot.exe(t_type, v, show, 0, k, list_k_y, offset, hf_cl, rad=rad_val)

if __name__ == "__main__":
    root = tkinter.Tk()
    app = SlalomGUI(root)
    root.mainloop()
