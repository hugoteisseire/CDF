import can
import time
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import WorkMode, CalibrationResult, Direction
from mks_servo_can.mks_enums import CalibrationResult, RunMotorResult
from lib_moteur import read_motor_speed,read_motor_position,move_relative,reset_zero,stop_pos_soft,move_velocity,stop_velocity_soft
#servo.set_work_mode(WorkMode.SrvFoc)

# Initialise l'interface CAN
bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
notifier = can.Notifier(bus, [])
bus.socket.setblocking(False)
# Crée les objets moteur
servo1 = MksServo(bus, notifier, 1)
servo2 = MksServo(bus, notifier, 2)
servo3 = MksServo(bus, notifier, 3)
print(servo1.set_subdivisions(16))
print(servo3.set_subdivisions(16))
print(servo2.set_subdivisions(16))
time.sleep(1)
print(servo3.b_calibrate_encoder())
print(servo2.b_calibrate_encoder())
print(servo1.b_calibrate_encoder())
time.sleep(10)

#reset_zero(bus, can_id=0x01)

#time.sleep(100.005)

#aller_vers_position_async(servo3, 504000, vitesse_rpm=300, acceleration=10)

reset_zero(bus, can_id=0x02)
reset_zero(bus, can_id=0x01)
reset_zero(bus, can_id=0x03)
frequency_hz = 0.1  # fréquence de l'onde sinusoïdale
amplitude = 1500    # amplitude max de la vitesse
offset = 1500       # vitesse de base (pour rester toujours positive)
acceleration = 10   # acceleration constante
speed=100
#move_relative(bus, can_id=0x03, direction=0, speed=1000, acceleration=80, pulses=1000000)
#move_relative(bus, can_id=0x02, direction=0, speed=1000, acceleration=80, pulses=1000000)
while True:
    move_relative(bus, can_id=0x01, direction=0, speed=1000, acceleration=200, pulses=200*16)
    time.sleep(0.5)

#stop_pos_soft(bus, can_id=1,acceleration=50)
#stop_pos_soft(bus, can_id=2,acceleration=100)
#stop_pos_soft(bus, can_id=3,acceleration=100)
t1 = time.perf_counter()
#move_velocity(bus, can_id=0x01, direction=1, speed=100, acceleration=100)
#move_velocity(bus, can_id=0x02, direction=1, speed=100, acceleration=100)
#move_velocity(bus, can_id=0x03, direction=1, speed=100, acceleration=100)
t2 = time.perf_counter()
print(f"Δt reset: {(t2 - t1)*1000:.2f} ms")
time.sleep(0.05)
flag=False
i = 0


speed = read_motor_speed(bus, can_id=2,timeout=0.2)   

while i < 1000:


    #pos1=read_motor_position(bus, can_id=1,timeout=0.2)   
    #pos2=read_motor_position(bus, can_id=2,timeout=0.2)   
    #pos3=read_motor_position(bus, can_id=3,timeout=0.2) 
    speed1 = read_motor_speed(bus, can_id=1,timeout=0.2) 
    speed2 = read_motor_speed(bus, can_id=2,timeout=0.2) 
    speed3 = read_motor_speed(bus, can_id=3,timeout=0.2)    
    if i%100==0 :
        flag=True
        t1 = time.perf_counter()
        speed+=100
        move_velocity(bus, can_id=0x01, direction=1, speed=speed, acceleration=200)
        move_velocity(bus, can_id=0x02, direction=1, speed=speed, acceleration=200)
        move_velocity(bus, can_id=0x03, direction=1, speed=speed, acceleration=200)
        t2 = time.perf_counter()
        print(f"Δt stop: {(t2 - t1)*1000:.2f} ms")
    time.sleep(0.005)
    print(f"speed1 {speed1} |speed2 {speed2} |speed3 {speed3} |  Δt: {(t2 - t1)*1000:.2f} ms")
    #print(f"pos1 {pos1} |pos1 {pos2} |pos1 {pos3} |1-2: {(pos2/pos1)*100}  |  1-3: {(pos3/pos1)*100}|  2-3: {(pos3/pos2)*100}  |  Δt: {(t2 - t1)*1000:.2f} ms")
    i += 1

stop_velocity_soft(bus, can_id=1,acceleration=250)
stop_velocity_soft(bus, can_id=2,acceleration=250)
stop_velocity_soft(bus, can_id=3,acceleration=250)
# Fermeture propre (ne s'exécute pas si le script reste en boucle)
notifier.stop()
bus.shutdown()


