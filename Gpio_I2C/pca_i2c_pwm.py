import board 
import busio 
import time
from adafruit_pca9685 import PCA9685

from adafruit_servokit import ServoKit



def dutyPercentage(value): #Ingresa un porcentaje y lo convierte en valores válidos
    return int(round(float(value/100) * 65535)) # 0XFFFF

i2c = busio.I2C(board.SCL, board.SDA) #define el bus de I2C
pca_module = PCA9685(i2c) #Convierte lo conectado en SDA/SCL en un objeto pca9685

kit = ServoKit(channels = 16) #Define los canales del módulo que se usa
kit.servo[0].set_pulse_width_range(500,2400) #Extraído de la documentación

pca_module.frequency = 100 #Define la frecuencia de los 16 PWM

pwm_7 = pca_module.channels[7] #elige el primer canal 0-15

pwm_7.duty_cycle =  dutyPercentage(20) #Coloca el valor del duty

while True:
    print("120")
    kit.servo[0].angle = 120
    time.sleep(1)
    print("70")
    kit.servo[0].angle = 70
    time.sleep(1)
    print("45")
    kit.servo[0].angle = 45
    time.sleep(1)











