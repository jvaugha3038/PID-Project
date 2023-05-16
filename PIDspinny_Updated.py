import board
from lcd.lcd import LCD
from lcd.i2c_pcf8574_interface import I2CPCF8574Interface
import time
import rotaryio
import digitalio
import adafruit_mpu6050
import pwmio
from simpleio import map_range

# get and i2c object
i2c = board.I2C()
fan = pwmio.PWMOut(board.D11, duty_cycle=0, frequency=440, variable_frequency=True)
# some LCDs are 0x3f... some are 0x27.

mpu = adafruit_mpu6050.MPU6050(i2c)
lcd = LCD(I2CPCF8574Interface(i2c, 0x27), num_rows=2, num_cols=16)
#screen test
lcd.print("hey")
print("hey")
time.sleep(1)
lcd.clear()
#setting up stuff
encoder = rotaryio.IncrementalEncoder(board.D1, board.D2, divisor=4)
button = digitalio.DigitalInOut(board.D3)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP
#subtract 12.9 degrees
#variable soup
KP = 1
KI = 1
KD = 1
encoder.position = 0
menu = 1
m_edit = False
update = True
last_position = -2
Set = 0
dt = .1
prev = 0
deg = -12.9
pdeg = deg
updTime = 0
ierr = 0
op = 0
P = 0
I = 0
D = 0
toggle=1
allow=1 # if 1, allows the button to be pressed again
now=0
#defining the pid function
def pid(Set,ierr,dt,KP,KI,KD):
        global prev
        global deg
        global op
        # Parameters in terms of PID coefficients
        op0 = 0
        # upper and lower bounds on heater level
        ophi = 100
        oplo = 10
        # calculate the error
        print("deg = "+str(deg))
        prev = deg
        
        deg=((round(float(mpu.gyro[0])+0.038, 1)*(dt)*(180/3.14159))*-1)+prev
        # calculate the measurement derivative
        dpv = (deg - prev) / dt
        error = Set-deg
        # calculate the integral error
        ierr = ierr + KI * error * dt


        # calculate the PID output
        P = KP * error
        I = ierr
        D = -KD * dpv
        op = op0 + P + I + D
        # implement anti-reset windup
        if op < oplo or op > ophi:
            I = I - KI * error * dt
            # clip output
            op = max(oplo,min(ophi,op))
        # return the controller output and PID terms
        return [op,P,I,D,error]

while True:
    if toggle == 1:
        print(str(pid(Set,ierr,dt,KP,KI,KD)))
        fan.duty_cycle = int(map_range(op, 0, 100, 6540, 49050)) # out of 65400
        print("-------------")
    else:
        fan.duty_cycle = 0

    position = encoder.position
    if position > last_position: # Changes the PID values if edit mode is on, changes the menu if edit mode is off
        if m_edit == True:
            if menu == 1:
                KP += 1
            elif menu == 2:
                KI += 1
            elif menu == 3:
                KD += 1
            elif menu == 4:
                Set += 1
        else:
            menu+=1
    elif position < last_position:
        if m_edit == True:
            if menu == 1:
                KP -= 1
            elif menu == 2:
                KI -= 1
            elif menu == 3:
                KD -= 1
            elif menu == 4:
                Set -= 1
        else:
            menu-=1

    if menu == 0: # Stops the menu from going too far
        menu = 5
    elif menu > 6:
        menu = 1

# checks which page is selected
    if (position != last_position) or (not button.value) or (update == True):
        lcd.clear()
        if menu == 1:
            lcd.print("kP = "+str(KP))
        if menu == 2:
            lcd.print("kI = "+str(KI))
        if menu == 3:
            lcd.print("kD = "+str(KD))
        if menu == 4:
            lcd.print("Setpoint = "+str(Set) + "  ")
        if menu == 5:
            lcd.print("PID is ")
            if toggle == 1:
                lcd.print("ON     ")
            else:
                lcd.print("OFF    ")
        if menu == 6:
            lcd.print("Recalibrate!! ")
        lcd.print("  A = " + str(round(deg)))
        if m_edit == True:
            lcd.print("   Editing")
        update = False

    if not button.value and allow == 1:   # Toggles the edit mode
        if menu == 5:
            toggle *= -1
        elif menu == 6:
            deg = -12.9
            prev = 0
        else:
            if m_edit == False:
                m_edit = True
            else:
                m_edit = False
        update = True # updates te LCD to reflect recent changes
        allow = 0 # maybe combine allow and now into one variable that does both?
        now = time.monotonic()
    if (now + 0.5) < time.monotonic():
        allow = 1
    if round(deg) != pdeg and time.monotonic() > (updTime + 1):
        update = True
        updTime = time.monotonic()
    pdeg = round(deg) # previous degrees is only used to determine if the lcd should be updated
    last_position = position # To check if the rotary encoder has turned
    time.sleep(dt) # Sleeps for a controlled amount of time to make the gyroscope and PID work.

