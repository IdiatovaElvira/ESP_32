import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np

# определение порта
def find_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.device:
            return port.device
    return None


port = find_port()
if port is None:
    print("Порт не найден")
    exit()

ser = serial.Serial(port, 115200)  
plt.ion()  # интерактивный режим для matplotlib

fig, ax = plt.subplots(3, 1, figsize=(10, 8)) #инициализируем з графика в одном столбце

max_len = 100  # Максимальное количество точек на графике
accel_x = []
accel_y = []
accel_z = []
gyro_x = []
gyro_y = []
gyro_z = []
temperature = []

# Временные метки
time_data = []

# Инициализация графиков
graph_accel_x, = ax[0].plot([], [], label='AccelX')
graph_accel_y, = ax[0].plot([], [], label='AccelY')
graph_accel_z, = ax[0].plot([], [], label='AccelZ')
graph_gyro_x, = ax[1].plot([], [], label='GyroX')
graph_gyro_y, = ax[1].plot([], [], label='GyroY')
graph_gyro_z, = ax[1].plot([], [], label='GyroZ')
graph_temperature, = ax[2].plot([], [], label='Temperature')

# Отрисовка осей
ax[0].set_title('Acceleration')
# ax[0].set_xlabel('Time')
ax[0].set_ylabel('m/s^2')
ax[0].legend()

ax[1].set_title('Gyroscope')
# ax[1].set_xlabel('Time')
ax[1].set_ylabel('rad/s')
ax[1].legend()

ax[2].set_title('Temperature')
ax[2].set_xlabel('Time')
ax[2].set_ylabel('°C')
ax[2].legend()

while True:
    try:
        data = ser.readline().decode() # чтение строки данных с serial port, декодируем её из байтов в строку
        if data:
            values = data.split(',')
            if len(values) == 7: #проверяем правильно ли считались данные
                accel_x.append(float(values[0]))
                accel_y.append(float(values[1]))
                accel_z.append(float(values[2]))
                gyro_x.append(float(values[3]))
                gyro_y.append(float(values[4]))
                gyro_z.append(float(values[5]))
                temperature.append(float(values[6]))

            time_data.append(len(time_data))  # индекс для времени

            if len(accel_x) > max_len:
                accel_x.pop(0)
                accel_y.pop(0)
                accel_z.pop(0)
                gyro_x.pop(0)
                gyro_y.pop(0)
                gyro_z.pop(0)
                temperature.pop(0)

            graph_accel_x.set_data(time_data, accel_x) # обновление данных
            graph_accel_y.set_data(time_data, accel_y)
            graph_accel_z.set_data(time_data, accel_z)
            graph_gyro_x.set_data(time_data, gyro_x)
            graph_gyro_y.set_data(time_data, gyro_y)
            graph_gyro_z.set_data(time_data, gyro_z)
            graph_temperature.set_data(time_data, temperature)

            #Обновление границ осей и масштаба
            ax[0].relim()
            ax[0].autoscale_view()
            ax[1].relim()
            ax[1].autoscale_view()
            ax[2].relim()
            ax[2].autoscale_view()

            plt.pause(0.01)
    #прерывание при Ctrl+c
    except KeyboardInterrupt:
        break

ser.close()

