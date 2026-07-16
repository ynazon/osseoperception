#!/usr/bin/env python3
"""
Simple tkinter GUI to perform the initializations from test_scripts/control_test.py
up to the filter / trajectory setup (roughly lines 1..197 in the original file).

- Use "Simulate" for running on non-RPi/dev machines (skips hardware calls).
- Saves initialized objects in gui.state for later use.
"""

import threading
import traceback
import time
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

# numeric and signal processing
import math
import numpy as np
from scipy.signal import butter, lfilter_zi

# Try to import repo utilities (robot_fxns)
try:
    import robot_fxns as robot
except Exception:
    # If robot_fxns is not available, create a minimal stub to allow GUI to run in simulate mode.
    robot = None

# Hardware libraries - imports are attempted at runtime inside init to allow simulate mode.
# e.g. serial, busio, adafruit_ads1x15, gpiozero, rtplot
import importlib

class InitGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Osseoperception - Initialization GUI")
        self.geometry("820x620")
        self.resizable(False, False)

        self.state = {}  # will hold initialized objects
        self.create_widgets()

    def create_widgets(self):
        frm = ttk.Frame(self, padding=10)
        frm.grid(sticky="NSEW")

        # Left: configuration
        cfg = ttk.LabelFrame(frm, text="Configuration", padding=10)
        cfg.grid(row=0, column=0, sticky="NW", padx=(0,10))

        r = 0
        ttk.Label(cfg, text="Sampling freq (Hz):").grid(row=r, column=0, sticky="W")
        self.sampling_freq_var = tk.DoubleVar(value=100.0)
        ttk.Entry(cfg, textvariable=self.sampling_freq_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Desired force (N):").grid(row=r, column=0, sticky="W")
        self.desired_force_var = tk.DoubleVar(value=7.0)
        ttk.Entry(cfg, textvariable=self.desired_force_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Testing flag:").grid(row=r, column=0, sticky="W")
        self.testing_flag_var = tk.StringVar(value="off")
        
        testing_frame = ttk.Frame(cfg)
        testing_frame.grid(row=r, column=1, sticky="W")
        ttk.Radiobutton(testing_frame, text="on", variable=self.testing_flag_var, value="on").pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(testing_frame, text="off", variable=self.testing_flag_var, value="off").pack(side=tk.LEFT, padx=2)
        r += 1

        ttk.Label(cfg, text="Traj type:").grid(row=r, column=0, sticky="W")
        self.traj_type_var = tk.StringVar(value="ls4")
        ttk.Entry(cfg, textvariable=self.traj_type_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Op cond:").grid(row=r, column=0, sticky="W")
        self.op_cond_var = tk.StringVar(value="hold_const")
        ttk.Entry(cfg, textvariable=self.op_cond_var, width=12).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Name:").grid(row=r, column=0, sticky="W")
        self.name_var = tk.StringVar(value="p1_test_t5")
        ttk.Entry(cfg, textvariable=self.name_var, width=20).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Date:").grid(row=r, column=0, sticky="W")
        self.date_var = tk.StringVar(value="6_11_26")
        ttk.Entry(cfg, textvariable=self.date_var, width=20).grid(row=r, column=1, sticky="W")
        r += 1

        self.simulate_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(cfg, variable=self.simulate_var, text="Simulate (no hardware)").grid(row=r, column=0, columnspan=2, sticky="W")
        r += 1

        # Start Options section
        ttk.Separator(cfg, orient="horizontal").grid(row=r, column=0, columnspan=2, sticky="EW", pady=(8, 8))
        r += 1

        ttk.Label(cfg, text="Start Options:", font=("TkDefaultFont", 10, "bold")).grid(row=r, column=0, columnspan=2, sticky="W")
        r += 1

        self.start_option_var = tk.StringVar(value="Auto")
        
        options = [
            ("Serial", "Serial"),
            ("MRI/Key", "MRI/Key"),
            ("User", "User"),
            ("Auto", "Auto")
        ]
        
        for label, value in options:
            ttk.Radiobutton(cfg, text=label, variable=self.start_option_var, value=value).grid(row=r, column=0, columnspan=2, sticky="W", padx=(20, 0))
            r += 1

        # Buttons
        btn_frame = ttk.Frame(cfg)
        btn_frame.grid(row=r, column=0, columnspan=2, pady=(8,0))
        ttk.Button(btn_frame, text="Initialize", command=self.start_initialize).grid(row=0, column=0, padx=4)
        ttk.Button(btn_frame, text="Test ADC read", command=self.test_adc).grid(row=0, column=1, padx=4)
        ttk.Button(btn_frame, text="Cleanup", command=self.cleanup).grid(row=0, column=2, padx=4)
        r += 1

        # Right: status/log
        st = ttk.LabelFrame(frm, text="Status / Log", padding=10)
        st.grid(row=0, column=1, sticky="NSEW")
        self.log = scrolledtext.ScrolledText(st, width=70, height=36, state='normal')
        self.log.grid(row=0, column=0, sticky="NSEW")

        # initial text
        self.log_insert("Ready. Fill parameters and click Initialize.\nDefault values match control_test.py defaults where possible.\n")

    def log_insert(self, text):
        ts = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{ts}] {text}\n")
        self.log.see(tk.END)
        self.update_idletasks()

    def start_initialize(self):
        # Run initialization in a background thread to avoid blocking the UI
        t = threading.Thread(target=self.initialize_all, daemon=True)
        t.start()

    def initialize_all(self):
        self.log_insert("Initialization started...")
        simulate = self.simulate_var.get()
        start_option = self.start_option_var.get()
        self.log_insert(f"Start option selected: {start_option}")

        # 1) rtplot client initialize plots (if available)
        try:
            rtplot = importlib.import_module("rtplot.client")
            client = rtplot.client
        except Exception:
            try:
                # fallback direct import (rtplot may expose client at top-level)
                import rtplot
                client = rtplot.client
            except Exception:
                client = None

        plot_config1 = {'names' : ['Desired Force','Actual Force'],
                        'colors' : ['b','r'],
                        'line_style': ['',''],
                        'title' : "Robot Force",
                        'ylabel': "Output Force (N)",
                        'xlabel': "Time (s)",
                        'yrange': [-10,25]
                        }

        if client is not None:
            try:
                client.initialize_plots(plot_config1)
                self.state['rtplot_client'] = client
                self.log_insert("rtplot: initialized plot_config1")
            except Exception as e:
                self.log_insert(f"rtplot initialize failed: {e}")
        else:
            self.log_insert("rtplot: not available (skipped)")

        # 2) Initialize data saving variables (basic metadata)
        try:
            testing_flag = self.testing_flag_var.get() == "on"
            test_time = 5.0
            name = self.name_var.get()
            date = self.date_var.get()
            trial_name = f"{name}_{date}.xlsx"
            save_location = "/home/pi/osseoperception/test_scripts/test_data/" + date + "/"
            file_save_path = save_location + trial_name
            self.state['testing_flag'] = testing_flag
            self.state['test_time'] = test_time
            self.state['trial_name'] = trial_name
            self.state['file_save_path'] = file_save_path
            self.state['start_option'] = start_option
            self.log_insert(f"Data variables initialized (testing_flag={testing_flag}, file path, metadata)")
        except Exception as e:
            self.log_insert(f"Data variables init failed: {e}")

        # 3) Control variables & arrays
        try:
            sampling_freq = float(self.sampling_freq_var.get())
            loop_dt = 1.0/sampling_freq
            desired_force = float(self.desired_force_var.get())
            self.state['sampling_freq'] = sampling_freq
            self.state['loop_dt'] = loop_dt
            self.state['desired_force'] = desired_force

            # arrays
            self.state.update({
                'desired_force_vector': [],
                'raw_force_vector': [],
                'filtered_measured_force_vector': [],
                'voltage_vector': [],
                'control_effort_vector': [],
                'time_loop_start': [],
                'filtered_voltage_vector': [],
            })
            self.log_insert("Control variables and data vectors initialized")
        except Exception as e:
            self.log_insert(f"Control variables init failed: {e}")

        # 4) Create control trajectory using robot.make_ls (if robot_fxns available)
        try:
            traj_type = self.traj_type_var.get()
            op_cond = self.op_cond_var.get()
            if robot is not None:
                traj = robot.make_ls(sampling_freq, traj_type, op_cond)
                self.state['traj'] = traj
                self.log_insert(f"Trajectory: robot.make_ls -> type={traj_type}, op_cond={op_cond}, len={len(traj)}")
            else:
                # simulate a trajectory
                traj = np.ones(int(sampling_freq*2)) * self.state['desired_force']  # 2s constant
                self.state['traj'] = traj
                self.log_insert("Trajectory: robot_fxns not available, created simulated constant trajectory")
        except Exception as e:
            self.log_insert(f"Trajectory creation failed: {e}")

        # 5) Calculate and display trajectory execution time
        try:
            traj = self.state.get('traj')
            sampling_freq = self.state.get('sampling_freq')
            
            if traj is not None and sampling_freq is not None:
                traj_length = len(traj)
                loop_dt = 1.0 / sampling_freq
                trajectory_time = traj_length * loop_dt
                self.log_insert(f"Trajectory execution time: {trajectory_time:.2f} seconds ({traj_length} samples @ {sampling_freq}Hz)")
                self.state['trajectory_time'] = trajectory_time
            else:
                self.log_insert("Could not calculate trajectory time (traj or sampling_freq missing)")
        except Exception as e:
            self.log_insert(f"Trajectory time calculation failed: {e}")

        # 6) Create lowpass Butterworth filter and initial zi
        try:
            fc = 20.0
            order = 2
            wn = fc / (sampling_freq / 2.0)
            b, a = butter(order, wn, btype='low')
            zi = lfilter_zi(b, a)
            self.state['filter_b'] = b
            self.state['filter_a'] = a
            self.state['filter_zi'] = zi
            self.log_insert(f"Filter: Butterworth lowpass (order={order}, fc={fc}Hz) design complete")
        except Exception as e:
            self.log_insert(f"Filter design failed: {e}")

        # 7) Initialize Serial Port (if not simulating)
        if not simulate:
            try:
                import serial
                ser = serial.Serial(
                    port='/dev/ttyAMA0',
                    baudrate=115200,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1
                )
                self.state['serial_port'] = ser
                self.log_insert("Serial port initialized: /dev/ttyAMA0 @ 115200 baud")
            except Exception as e:
                self.log_insert(f"Serial port initialization failed: {e}")
                self.state['serial_port'] = None
        else:
            self.log_insert("Serial port: skipped (simulate mode)")
            self.state['serial_port'] = None

        # 8) Initialize GPIO pins (if not simulating)
        if not simulate:
            try:
                import gpiozero
                # Start pin (input)
                start_pin = 17  # Pin 11 is GPIO 17
                input_pin = gpiozero.Button(start_pin, pull_up=False)
                self.state['input_pin'] = input_pin
                self.log_insert(f"Start pin initialized: GPIO {start_pin}")
                
                # Stop pin (output)
                stop_pin = 27  # Pin 13 is GPIO 27
                output_pin = gpiozero.LED(stop_pin)
                output_pin.on()
                self.state['output_pin'] = output_pin
                self.log_insert(f"Stop pin initialized: GPIO {stop_pin}")
            except Exception as e:
                self.log_insert(f"GPIO pin initialization failed: {e}")
                self.state['input_pin'] = None
                self.state['output_pin'] = None
        else:
            self.log_insert("GPIO pins: skipped (simulate mode)")
            self.state['input_pin'] = None
            self.state['output_pin'] = None

        # 9) Initialize ADC (if not simulating)
        if not simulate:
            try:
                import busio
                import adafruit_ads1x15.ads1115 as ADS
                from adafruit_ads1x15.analog_in import AnalogIn
                
                # I2C pins
                scl_pin = 3
                sda_pin = 2
                i2c = busio.I2C(scl_pin, sda_pin)
                ads = ADS.ADS1115(i2c)
                ads.gain = 2/3  # Set adc max value to 6.144 V
                chan = AnalogIn(ads, ADS.P0)  # Create single-ended input on channel 0
                
                self.state['i2c'] = i2c
                self.state['ads'] = ads
                self.state['adc_channel'] = chan
                self.log_insert("ADC initialized: ADS1115 on I2C (SCL=GPIO3, SDA=GPIO2), gain=2/3 V")
            except Exception as e:
                self.log_insert(f"ADC initialization failed: {e}")
                self.state['i2c'] = None
                self.state['ads'] = None
                self.state['adc_channel'] = None
        else:
            self.log_insert("ADC: skipped (simulate mode)")
            self.state['i2c'] = None
            self.state['ads'] = None
            self.state['adc_channel'] = None

        # 10) Initialize PWM (if not simulating)
        if not simulate:
            try:
                import gpiozero
                
                # Valve PWM
                pwm_freq = 5000.0
                valve_pwm_pin = 18
                valve_pwm = gpiozero.PWMOutputDevice(valve_pwm_pin, active_high=True, initial_value=1.0, frequency=pwm_freq)
                self.state['valve_pwm'] = valve_pwm
                self.log_insert(f"Valve PWM initialized: GPIO {valve_pwm_pin} @ {pwm_freq}Hz")
                
                # Signal PWM
                signal_pwm_freq = 1000.0
                signal_pwm_pin = 13
                signal_pwm = gpiozero.PWMOutputDevice(signal_pwm_pin, active_high=True, initial_value=0.0, frequency=signal_pwm_freq)
                self.state['signal_pwm'] = signal_pwm
                self.log_insert(f"Signal PWM initialized: GPIO {signal_pwm_pin} @ {signal_pwm_freq}Hz")
            except Exception as e:
                self.log_insert(f"PWM initialization failed: {e}")
                self.state['valve_pwm'] = None
                self.state['signal_pwm'] = None
        else:
            self.log_insert("PWM: skipped (simulate mode)")
            self.state['valve_pwm'] = None
            self.state['signal_pwm'] = None

        self.log_insert("Initialization complete. Objects stored in gui.state")
        self.log_insert("Ready to begin control loop (not implemented in GUI).")

    def test_adc(self):
        if self.state.get('adc_channel'):
            try:
                voltage = self.state['adc_channel'].voltage
                self.log_insert(f"ADC test: read voltage = {voltage:.3f} V")
            except Exception as e:
                self.log_insert(f"ADC test failed: {e}")
        else:
            self.log_insert("ADC not initialized (simulate mode or initialization failed).")

    def cleanup(self):
        """Clean up hardware resources"""
        cleanup_msg = "Cleanup: "
        
        # Close serial port
        if self.state.get('serial_port'):
            try:
                self.state['serial_port'].close()
                cleanup_msg += "serial port closed; "
            except:
                pass
        
        # Close PWM devices
        if self.state.get('valve_pwm'):
            try:
                self.state['valve_pwm'].value = 0.5
                self.state['valve_pwm'].close()
                cleanup_msg += "valve PWM closed; "
            except:
                pass
        
        if self.state.get('signal_pwm'):
            try:
                self.state['signal_pwm'].value = 0.0
                self.state['signal_pwm'].close()
                cleanup_msg += "signal PWM closed; "
            except:
                pass
        
        # Close GPIO pins
        if self.state.get('output_pin'):
            try:
                self.state['output_pin'].close()
                cleanup_msg += "output pin closed; "
            except:
                pass
        
        if self.state.get('input_pin'):
            try:
                self.state['input_pin'].close()
                cleanup_msg += "input pin closed; "
            except:
                pass
        
        if cleanup_msg == "Cleanup: ":
            cleanup_msg = "Cleanup: nothing to close (simulate mode or already cleaned)"
        
        self.log_insert(cleanup_msg)

if __name__ == "__main__":
    app = InitGUI()
    app.mainloop()
