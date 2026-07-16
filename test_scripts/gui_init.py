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
        self.title("OSSE Perception - Initialization GUI")
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
        ttk.Label(cfg, text="Serial port:").grid(row=r, column=0, sticky="W")
        self.serial_port_var = tk.StringVar(value="/dev/ttyAMA0")
        ttk.Entry(cfg, textvariable=self.serial_port_var, width=20).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Baudrate:").grid(row=r, column=0, sticky="W")
        self.baud_var = tk.IntVar(value=115200)
        ttk.Entry(cfg, textvariable=self.baud_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Start pin (GPIO):").grid(row=r, column=0, sticky="W")
        self.start_pin_var = tk.IntVar(value=17)
        ttk.Entry(cfg, textvariable=self.start_pin_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Stop pin (GPIO):").grid(row=r, column=0, sticky="W")
        self.stop_pin_var = tk.IntVar(value=27)
        ttk.Entry(cfg, textvariable=self.stop_pin_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="SCL pin:").grid(row=r, column=0, sticky="W")
        self.scl_pin_var = tk.StringVar(value="3")
        ttk.Entry(cfg, textvariable=self.scl_pin_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="SDA pin:").grid(row=r, column=0, sticky="W")
        self.sda_pin_var = tk.StringVar(value="2")
        ttk.Entry(cfg, textvariable=self.sda_pin_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Valve PWM pin:").grid(row=r, column=0, sticky="W")
        self.valve_pwm_pin_var = tk.IntVar(value=18)
        ttk.Entry(cfg, textvariable=self.valve_pwm_pin_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Valve PWM freq (Hz):").grid(row=r, column=0, sticky="W")
        self.valve_pwm_freq_var = tk.DoubleVar(value=5000.0)
        ttk.Entry(cfg, textvariable=self.valve_pwm_freq_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Signal PWM pin:").grid(row=r, column=0, sticky="W")
        self.signal_pwm_pin_var = tk.IntVar(value=13)
        ttk.Entry(cfg, textvariable=self.signal_pwm_pin_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Signal PWM freq (Hz):").grid(row=r, column=0, sticky="W")
        self.signal_pwm_freq_var = tk.DoubleVar(value=1000.0)
        ttk.Entry(cfg, textvariable=self.signal_pwm_freq_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Sampling freq (Hz):").grid(row=r, column=0, sticky="W")
        self.sampling_freq_var = tk.DoubleVar(value=100.0)
        ttk.Entry(cfg, textvariable=self.sampling_freq_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Desired force (N):").grid(row=r, column=0, sticky="W")
        self.desired_force_var = tk.DoubleVar(value=7.0)
        ttk.Entry(cfg, textvariable=self.desired_force_var, width=10).grid(row=r, column=1, sticky="W")
        r += 1

        ttk.Label(cfg, text="Testing flag:").grid(row=r, column=0, sticky="W")
        self.testing_flag_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(cfg, variable=self.testing_flag_var, text="testing_flag").grid(row=r, column=1, sticky="W")
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

        # 2) Serial
        try:
            serial = importlib.import_module("serial")
            port = self.serial_port_var.get()
            baud = int(self.baud_var.get())
            if not simulate:
                ser = serial.Serial(port=port, baudrate=baud, parity=serial.PARITY_NONE,
                                    stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1)
                self.state['serial'] = ser
                self.log_insert(f"Serial: opened {port} @ {baud}")
            else:
                self.state['serial'] = None
                self.log_insert(f"Serial: simulate mode, would open {port} @ {baud}")
        except Exception as e:
            self.log_insert(f"Serial import/open failed: {e}")

        # 3) GPIO (start/stop buttons)
        try:
            gpiozero = importlib.import_module("gpiozero")
            start_pin = int(self.start_pin_var.get())
            stop_pin = int(self.stop_pin_var.get())
            if not simulate:
                btn = gpiozero.Button(start_pin, pull_up=False)
                led = gpiozero.LED(stop_pin)
                led.on()  # match original: configure stop pin to high
                self.state['start_button'] = btn
                self.state['stop_led'] = led
                self.log_insert(f"GPIO: start Button on GPIO{start_pin}, stop LED on GPIO{stop_pin} (LED on)")
            else:
                self.state['start_button'] = None
                self.state['stop_led'] = None
                self.log_insert(f"GPIO: simulate mode, would configure start GPIO{start_pin} and stop GPIO{stop_pin} (LED on)")
        except Exception as e:
            self.log_insert(f"GPIO init failed: {e}")

        # 4) I2C + ADS1115
        try:
            busio = importlib.import_module("busio")
            ads_mod = importlib.import_module("adafruit_ads1x15.ads1115")
            analog_in_mod = importlib.import_module("adafruit_ads1x15.analog_in")
            ADS = ads_mod
            AnalogIn = getattr(analog_in_mod, "AnalogIn")
            scl_pin = None
            sda_pin = None
            # try to use raw values the same way your script does (numbers)
            scl_val = self.scl_pin_var.get()
            sda_val = self.sda_pin_var.get()
            # prefer numeric if given
            try:
                scl_pin = int(scl_val)
                sda_pin = int(sda_val)
            except Exception:
                # fall back to string names (e.g. board.SCL)
                scl_pin = scl_val
                sda_pin = sda_val

            if not simulate:
                # busio.I2C usually expects board.SCL/board.SDA objects; your script passes integers.
                try:
                    i2c = busio.I2C(scl_pin, sda_pin)
                except Exception:
                    # try using board pins if available
                    try:
                        import board
                        i2c = busio.I2C(board.SCL, board.SDA)
                    except Exception as ee:
                        raise ee
                ads = ADS.ADS1115(i2c)
                ads.gain = 2/3
                chan = AnalogIn(ads, ADS.P0)
                self.state['i2c'] = i2c
                self.state['ads'] = ads
                self.state['ads_chan'] = chan
                self.log_insert("ADS1115: initialized AnalogIn channel (ADS1115, gain=2/3)")
            else:
                self.state['i2c'] = None
                self.state['ads'] = None
                self.state['ads_chan'] = None
                self.log_insert("ADS1115: simulate mode, skipped hardware initialization")
        except Exception as e:
            self.log_insert(f"ADS1115 / I2C init failed: {e}")

        # 5) PWM outputs
        try:
            gpiozero = importlib.import_module("gpiozero")
            valve_pin = int(self.valve_pwm_pin_var.get())
            valve_freq = float(self.valve_pwm_freq_var.get())
            signal_pin = int(self.signal_pwm_pin_var.get())
            signal_freq = float(self.signal_pwm_freq_var.get())
            if not simulate:
                valve_pwm = gpiozero.PWMOutputDevice(valve_pin, active_high=True, initial_value=1.0, frequency=valve_freq)
                signal_pwm = gpiozero.PWMOutputDevice(signal_pin, active_high=True, initial_value=0.0, frequency=signal_freq)
                self.state['valve_pwm'] = valve_pwm
                self.state['signal_pwm'] = signal_pwm
                self.log_insert(f"PWM: valve PWM on GPIO{valve_pin} @ {valve_freq}Hz, signal PWM on GPIO{signal_pin} @ {signal_freq}Hz")
            else:
                self.state['valve_pwm'] = None
                self.state['signal_pwm'] = None
                self.log_insert(f"PWM: simulate mode, would configure valve GPIO{valve_pin} ({valve_freq}Hz), signal GPIO{signal_pin} ({signal_freq}Hz)")
        except Exception as e:
            self.log_insert(f"PWM init failed: {e}")

        # 6) Initialize data saving variables (basic metadata)
        try:
            testing_flag = bool(self.testing_flag_var.get())
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
            self.log_insert("Data variables initialized (testing_flag, file path, metadata)")
        except Exception as e:
            self.log_insert(f"Data variables init failed: {e}")

        # 7) Control variables & arrays
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

        # 8) Create control trajectory using robot.make_ls (if robot_fxns available)
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

        # 9) Create lowpass Butterworth filter and initial zi
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

        # 10) Try to get load cell zero offset if robot_fxns has function
        try:
            if (not simulate) and (robot is not None) and hasattr(robot, "load_cell_zero"):
                force_offset = robot.load_cell_zero()
                self.state['force_offset'] = force_offset
                self.log_insert(f"Load cell zero offset: {force_offset}")
            else:
                self.state['force_offset'] = 0.0
                self.log_insert("Load cell zero: skipped (simulate or robot_fxns missing)")
        except Exception as e:
            self.log_insert(f"Load cell zero failed: {e}")

        self.log_insert("Initialization complete. Objects stored in gui.state")
        self.log_insert("Ready to begin control loop (not implemented in GUI).")

    def test_adc(self):
        # Read single ADC sample (if available)
        if 'ads_chan' in self.state and self.state['ads_chan'] is not None:
            try:
                chan = self.state['ads_chan']
                v = chan.voltage
                self.log_insert(f"ADC voltage read: {v:.6f} V")
            except Exception as e:
                self.log_insert(f"ADC read failed: {e}")
        else:
            self.log_insert("ADC not initialized (or running in simulate mode).")

    def cleanup(self):
        # Attempt to gracefully close hardware resources
        self.log_insert("Cleanup started...")
        # Serial
        try:
            ser = self.state.get('serial', None)
            if ser:
                ser.close()
                self.log_insert("Serial: closed")
        except Exception as e:
            self.log_insert(f"Serial cleanup error: {e}")

        # PWM
        try:
            valve_pwm = self.state.get('valve_pwm', None)
            if valve_pwm:
                valve_pwm.value = 0.5
                valve_pwm.close()
                self.log_insert("Valve PWM: stopped and closed")
            signal_pwm = self.state.get('signal_pwm', None)
            if signal_pwm:
                signal_pwm.value = 0.0
                signal_pwm.close()
                self.log_insert("Signal PWM: stopped and closed")
        except Exception as e:
            self.log_insert(f"PWM cleanup error: {e}")

        # GPIO
        try:
            btn = self.state.get('start_button', None)
            if btn:
                btn.close()
            led = self.state.get('stop_led', None)
            if led:
                led.close()
            self.log_insert("GPIO: closed")
        except Exception as e:
            self.log_insert(f"GPIO cleanup error: {e}")

        self.log_insert("Cleanup finished.")

if __name__ == "__main__":
    app = InitGUI()
    app.mainloop()