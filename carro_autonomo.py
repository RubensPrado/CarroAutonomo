import cv2
import numpy as np
from time import sleep
from gpiozero import Motor, PWMOutputDevice

# Configurações dos pinos do motor
motor_esquerda = Motor(forward=17, backward=18)
motor_direita = Motor(forward=27, backward=22)
stepper = PWMOutputDevice(pin=23, frequency=500)

# Constantes PID
Kp = 0.1
Ki = 0.01
Kd = 0.05

# Variáveis PID
setpoint = 320  # Centro ideal baseado na largura da imagem (640 pixels / 2)
integral = 0.0
last_error = 0.0

def process_image(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 1.5)
    edges = cv2.Canny(blur, 50, 150)
    
    # Aqui você poderia adicionar lógica mais complexa para detectar o centro da pista
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, None, 50, 10)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # Calcula o ponto médio das linhas detectadas como centro da pista
        return np.mean([line[0][0] + line[0][2] for line in lines]) / 2
    return setpoint  # Retorna o centro da imagem se nenhuma linha for detectada

def calculate_pid(measured_value):
    global integral, last_error
    error = setpoint - measured_value
    integral += error
    derivative = error - last_error
    output = Kp * error + Ki * integral + Kd * derivative
    last_error = error
    return output

def control_motors(steering):
    if steering > 0:
        motor_esquerda.forward(speed=0.5)
        motor_direita.forward(speed=0.5 + steering)
    else:
        motor_esquerda.forward(speed=0.5 - steering)
        motor_direita.forward(speed=0.5)
    stepper.value = min(max(steering, -1), 1)  # Garante que o valor esteja entre -1 e 1

# Configuração da câmera
cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        lane_center = process_image(frame)
        pid_output = calculate_pid(lane_center)
        control_motors(pid_output)
        
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    motor_esquerda.stop()
    motor_direita.stop()
    stepper.close()

