import time
import stretch4_body.subsystem.power_periph as pimu
import stretch4_body.subsystem.omnibase as base
import matplotlib.pyplot as plt
def get_cpu_temperature():
    try:
        with open("/sys/class/thermal/thermal_zone3/temp", "r") as f:
            temp_str = f.read().strip()
            # Temperature is in milli-degrees Celsius, so divide by 1000
            temperature_celsius = float(temp_str) / 1000.0
            return temperature_celsius
    except FileNotFoundError:
        print("Error: thermal_zone3/temp file not found. Your system might use a different path or not expose this information.")
        return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None

p = pimu.PowerPeriph()
b = base.OmniBase()
p.startup()
if not b.startup():
    exit()
cpu_temp = []
p_temp = []
w0_temp = []
w1_temp = []
w2_temp = []
start = time.time()
sample = start - 60
while time.time() - start <= 3*660:
    p.pull_status()
    b.pull_status()
    if time.time() - sample >= 30:
        cpu_temp.append(get_cpu_temperature())
        p_temp.append(p.status['temp'])
        w0_temp.append(b.wheels[0].status['temperature'])
        w1_temp.append(b.wheels[1].status['temperature'])
        w2_temp.append(b.wheels[2].status['temperature'])
        sample = time.time()
    
    b.rotate_by(6.28, b.params['motion']['max']['vel_w_r'], b.params['motion']['max']['accel_w_r'])
    b.push_command()
    p.trigger_motor_sync()
b.stop()
b.push_command()
p.trigger_motor_sync()

# print("Results: ")
# print("CPU: ", cpu_temp)
# print("Pimu: ", p_temp)
# print("W0: ", w0_temp)
# print("W1: ", w1_temp)
# print("W2: ", w2_temp)
plt.figure(1)
plt.plot(cpu_temp)
plt.title('CPU Temp')

plt.figure(2)
plt.plot(p_temp)
plt.title('PIMU Temp')

plt.figure(3)
plt.plot(w0_temp)
plt.title('W0 Temp')

plt.figure(4)
plt.plot(w1_temp)
plt.title('W1 Temp')

plt.figure(5)
plt.plot(w2_temp)
plt.title('W2 Temp')
plt.show()
