import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import messagebox

# Initialize Serial Port
ser = serial.Serial('COM3', 115200)  # Change 'COM3' to your Arduino COM port
time.sleep(2)  # Allow time for the serial connection to initialize

# Initialize variables
angles = []
speeds = []
Kp = 5.7
Ki = 5.0
Kd = 0.1
setpoint = 0.0

# Function to update graph
def update_graph(frame):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            parts = line.split('|')
            if len(parts) >= 2:
                angle = float(parts[0].split(':')[1].strip())
                speed = float(parts[1].split(':')[1].strip())
                
                angles.append(angle)
                speeds.append(speed)

                if len(angles) > 100:  # Limit the number of data points
                    angles.pop(0)
                    speeds.pop(0)

                ax1.clear()
                ax2.clear()
                
                ax1.plot(angles, label='Angle (°)', color='blue')
                ax2.plot(speeds, label='Motor Speed', color='red')

                ax1.set_title('Angle from MPU6050')
                ax2.set_title('Motor Speed')
                ax1.set_xlabel('Time (ms)')
                ax2.set_xlabel('Time (ms)')
                ax1.set_ylabel('Angle (°)')
                ax2.set_ylabel('Speed')

                ax1.legend()
                ax2.legend()
                ax1.grid()
                ax2.grid()
                plt.tight_layout()
        except Exception as e:
            print(f"Error parsing line: {line} - {e}")

# Function to send PID values
def update_pid():
    global Kp, Ki, Kd, setpoint
    try:
        Kp = float(entry_kp.get())
        Ki = float(entry_ki.get())
        Kd = float(entry_kd.get())
        setpoint = float(entry_setpoint.get())
        command = f'SET_PID,{Kp},{Ki},{Kd},{setpoint}\n'
        ser.write(command.encode())
        messagebox.showinfo("Success", "PID values updated successfully!")
    except ValueError:
        messagebox.showerror("Error", "Invalid input. Please enter numerical values.")

# GUI Setup
root = tk.Tk()
root.title("PID Tuning and Data Monitoring")

# Create input fields for PID tuning
tk.Label(root, text="Kp:").grid(row=0, column=0)
entry_kp = tk.Entry(root)
entry_kp.insert(0, str(Kp))
entry_kp.grid(row=0, column=1)

tk.Label(root, text="Ki:").grid(row=1, column=0)
entry_ki = tk.Entry(root)
entry_ki.insert(0, str(Ki))
entry_ki.grid(row=1, column=1)

tk.Label(root, text="Kd:").grid(row=2, column=0)
entry_kd = tk.Entry(root)
entry_kd.insert(0, str(Kd))
entry_kd.grid(row=2, column=1)

tk.Label(root, text="Setpoint:").grid(row=3, column=0)
entry_setpoint = tk.Entry(root)
entry_setpoint.insert(0, str(setpoint))
entry_setpoint.grid(row=3, column=1)

tk.Button(root, text="Update PID", command=update_pid).grid(row=4, column=0, columnspan=2)

# Setup Matplotlib
fig, (ax1, ax2) = plt.subplots(2, 1)
ani = FuncAnimation(fig, update_graph, interval=100)

plt.tight_layout()
plt.show(block=False)

# Start the GUI loop
root.mainloop()

# Close serial connection
ser.close()
