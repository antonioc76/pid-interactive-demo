import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import numpy as np

def compute_pid(reference_signal, dt, kp_slider:Slider, ki_slider:Slider, kd_slider:Slider):
    kp = kp_slider.val
    ki = ki_slider.val
    kd = kd_slider.val

    pid0 = 0
    pid_signal = np.zeros(len(reference_signal))
    pid_signal[0] = pid0
    error = np.zeros(len(reference_signal))
    U_p = np.zeros(len(reference_signal))
    U_i = np.zeros(len(reference_signal))
    U_d = np.zeros(len(reference_signal))

    for i in range(len(reference_signal)-1):
        error[i] = reference_signal[i] - pid_signal[i]
        U_p[i] = kp * error[i]
        U_i[i] = ki * (sum(error*dt))
        U_d[i] = (kd * (error[i] - error[i-1]) / dt) if i >= 1 else 0

        pid_signal[i+1] = U_p[i] + U_i[i] + U_d[i]
    
    return pid_signal, error
    

def update(val):
    axes[0].clear()
    axes[1].clear()
    reference_signal = reference_signal = np.ones(int(on_time/dt)) * voltage_slider.val
    pid_signal, error = compute_pid(reference_signal, dt, kp_slider, ki_slider, kd_slider)

    axes[0].set_xlabel('time (s)')
    axes[0].set_ylabel('Voltage (V)')
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel('error (V)')

    axes[0].set_ylim((-voltage_slider.valinit, voltage_slider.valinit*2.1))
    axes[1].set_ylim((-voltage_slider.valinit, voltage_slider.valinit*2.1))
    
    axes[0].plot(times, reference_signal, label='Reference signal')
    axes[0].plot(times, pid_signal, label='PID signal')
    axes[1].plot(times, error, label='Error')
    
    axes[0].legend(loc='lower right')
    axes[1].legend(loc='lower right')


def reset(event):
    voltage_slider.reset()
    kp_slider.reset()
    ki_slider.reset()
    kd_slider.reset()


if __name__ == "__main__":
    fig, axes = plt.subplots(2,1)
    plt.subplots_adjust(left=0.25, bottom=0.4)

    ax_voltage = plt.axes([0.25, 0.25, 0.65, 0.03])
    voltage_slider = Slider(ax_voltage, 'Voltage', 0, 20, valinit=10)

    on_time = 10
    dt = 0.1

    reference_signal = np.ones(int(on_time/dt)) * voltage_slider.val
    times = np.zeros(len(reference_signal))
    for i in range(len(reference_signal)-1):
        times[i+1] = times[i] + dt

    ax_kp = plt.axes([0.25, 0.2, 0.65, 0.03])
    ax_ki = plt.axes([0.25, 0.15, 0.65, 0.03])
    ax_kd = plt.axes([0.25, 0.1, 0.65, 0.03])

    kp_slider = Slider(ax_kp,'kp', 0, 2, valinit=0.3)
    ki_slider = Slider(ax_ki, 'ki', 0, 2 / dt, valinit=0.5)
    kd_slider = Slider(ax_kd, 'kd', -2 * dt, 2 * dt, valinit=0)

    pid_signal, error = compute_pid(reference_signal, dt, kp_slider, ki_slider, kd_slider)

    axes[0].set_ylim((-voltage_slider.valinit, voltage_slider.valinit*2.1))
    axes[1].set_ylim((-voltage_slider.valinit, voltage_slider.valinit*2.1))

    axes[0].set_xlabel('time (s)')
    axes[0].set_ylabel('Voltage (V)')
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel('error (V)')

    axes[0].plot(times, reference_signal, label='Reference signal')
    axes[0].plot(times, pid_signal, label='PID signal')
    
    axes[1].plot(times, error, label='Error')

    voltage_slider.on_changed(update)
    kp_slider.on_changed(update)
    ki_slider.on_changed(update)
    kd_slider.on_changed(update)

    resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
    button = Button(resetax, 'Reset', color='gold',
                hovercolor='skyblue')
    
    button.on_clicked(reset)

    axes[0].legend(loc='lower right')
    axes[1].legend(loc='lower right')
    plt.show()