# PID-Project

## Table of Contents
* [Table of Contents](#Table-of-Contents)
* [Goal](#Goal)
* [CAD](#CAD)
* [Code](#Code)
* [Building](#Building)
* [Bill of Materials](#Bill-of-Materials)
* [Tuning](#Tuning)
* [End Result](#End-Result)
* [Problems](#Problems)
* [Reflection](#Reflection)
---
# `Goal`
The goal of this project was to create a PID system that would control a dveice. We decided to create an arm with a fan on one end, that would use the PID to power the fan and balance the arm at a certain angle. This meant that we needed a gyroscope to know the current angle, and a small drone fan. We also needed a way to change the PID variable while testing the design, so we would need to make a rotary encoder + LCD menu.

# `CAD`

![Planning_Doc](https://github.com/jvaugha3038/PID-Project/assets/113116247/3aa01ba1-9bcc-4b66-aa9c-25665719a949)


[Document](https://cvilleschools.onshape.com/documents/f00cf12c984b3d4ce9458e93/w/95ae618fe06d1a19a8a98ffb/e/a00b791e7332e8573e6656cd)

We had made a planning paper before starting to outline the design we wanted. This made the process a lot easier, as I didn't have to make it up as I went. The basic design was pretty simple at first, two supports and an arm. It's always the details that are problematic, though. One such detail is how we have the arm spin. The original plan was to have a ball bearing in the arm and a rod through the supports and arms. It turned out the bearing was too big, and also unnecessary for this small of a design. We went instead for a bushing design, where there is a static rod connected that the arm spins around, no bearing needed.

There was also the issue of weight, as the tiny little motor needed to be able to lift itself and the arm off of the ground. I had designed the arm to be very light and have a counterweight to balance out the weight, and thanks to Onshape's handy center of mass feature, I was able to determine that the center of mass was just 100mm towards the motor. Given that the arm was 250mm long, that seemed to be a pretty good spot, and it confirmed that the motor wouldn't be overburdened. 

A rude awakening came when I remembered that we like need an arduino, screen, and rotary encoder to, ya know, to do the pid'ing. So I had to make space for those, which made it look a little worse to be honest. I got it all to work though, except for the battery pack. The pack was just way bigger than I thought it would be, and I couldn't fit it between the supports like I did with the arduino. My solution was to just have it sit on one of the supports. No connections, just sitting there :). I know this wasn't the most professional decision, but my idea was that it could help weigh down the support to stop it from moving while giving easy access to the batteries.

![Screenshot 2023-04-26 10 38 11 AM](https://user-images.githubusercontent.com/113116247/234610805-64f86760-13fe-4a1f-bbc3-2b34732a7c41.png)


# `Code`
The code was challenging overall, but once the main hurdle was cleared (finding good PID code online), it started to get easier. The first problem was that both the gyroscope and the LCD screen required SDA and SCL pins, and the Metro M4 only had one of each. The solution we came up with was to make them both share the SDA and SCL pins, which didn't seem like it should have worked, but it did. The first PID system we tried was simple-pid (from river), but I ended up switching to one that was even simpler. Its a function instead of a library, that just takes a variable and spits out an output when called on. (found here: https://apmonitor.com/pdc/index.php/Main/TCLabPIDControl) 

<details>
<summary><b>Click to Show<b></summary>
        
<p>
        
```python

import board
from lcd.lcd import LCD
from lcd.i2c_pcf8574_interface import I2CPCF8574Interface
import time
import rotaryio
import digitalio
import adafruit_mpu6050
import pwmio

# get and i2c object
i2c = board.I2C()
fan = pwmio.PWMOut(board.D7, duty_cycle=0, frequency=440, variable_frequency=True)
# some LCDs are 0x3f... some are 0x27.

mpu = adafruit_mpu6050.MPU6050(i2c)
lcd = LCD(I2CPCF8574Interface(i2c, 0x27), num_rows=2, num_cols=16)
#screen test
lcd.print("hey")
print("hey")
time.sleep(1)
lcd.clear()
#setting up stuff
encoder = rotaryio.IncrementalEncoder(board.D1, board.D2, divisor=2)
button = digitalio.DigitalInOut(board.D3)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP
# on/off switch setup
switch = digitalio.DigitalInOut(board.D8)
switch.direction = digitalio.Direction.INPUT
switch.pull = digitalio.Pull.UP

#subtract 12.9 degrees
#variable soup
KP = 1
KI = 1
KD = 1
encoder.position = 0
menu = 1
m_edit = False
last_position = -2
Set = 45
dt = .1
prev = 0
deg = -12.9
ierr = 0
op = 0
P = 0
I = 0
D = 0
#defining the pid function
def pid(Set,ierr,dt,KP,KI,KD):
        global prev
        global deg
        # Parameters in terms of PID coefficients
        op0 = 0
        # upper and lower bounds on heater level
        ophi = 100
        oplo = 10
        # calculate the error
        print("deg = "+str(deg))
        prev = deg
        
        deg=(round(float(mpu.gyro[0])+0.038, 1)*(dt)*(180/3.14159))+prev
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
    print(str(pid(Set,ierr,dt,KP,KI,KD)))
    
    position = encoder.position
    if position > last_position: # Changes the PID values if edit mode is on, changes the menu if edit mode is off
        if m_edit == True:
            if menu == 1:
                KP += 1
            elif menu == 2:
                KI += 1
            elif menu == 3:
                KD += 1
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
        else:
            menu-=1

    if menu == 0: # Stops the menu from going too far
        menu = 3
    elif menu > 3:
        menu = 1

# checks which page is selected
    if position != last_position or not button.value:
        lcd.clear()
        if menu == 1:
            lcd.print("kP = "+str(KP))
        if menu == 2:
            lcd.print("kI = "+str(KI))
        if menu == 3:
            lcd.print("kD = "+str(KD))
        if m_edit == True:
            lcd.print("          Editing ^v")

    if not button.value:   # Toggles the edit mode
       if m_edit == False:
            m_edit = True
       else:
            m_edit = False

    last_position = position
    time.sleep(dt) # Sleeps for a controlled amount of time to make the gyroscope and PID work.
    print("-------------")
```
</p>  
    
</details>
        
# `Building`
The actual building of the frame was pretty easy, since its literally 4 parts including the Metro M4. The wiring was a nightmare though. We needed a lot of wires, because we had a ton of things we were connecting (6 things total, 3 of which needed 4+ wires) and all of these things were very close to eachother. We also had to keep the wires low and out of the way so they wouldn't run into the fan and mess with the PID. Lastly, the fan wires had to reach all the way to the end of the arm, and every other wire had to be as short as possible.
## Bill of Materials
       
| Item | Name                               | Quantity |
| ---- | ---------------------------------- | -------- |
| 1    | Arm                                | 1        |
| 2    | Bushing                            | 2        |
| 3    | Dowel (6.5cm+)                     | 1        |
| 4    | Base                               | 1        |
| 5    | Set Holder                         | 2        |
| 6    | Base                               | 1        |
| 7    | Gyroscope                          | 1        |
| 8    | Encoder Knob                       | 1        |
| 9    | Rotary Encoder                     | 1        |
| 10   | Panel Mount SPDT                   | 1        |
| 11   | Arduino Uno                        | 1        |
| 12   | LCD                                | 1        |
| 13   | LCD Backpack                       | 1        |
| 14   | LCD_PCB                            | 1        |
| 15   | metal_casing                       | 1        |
| 16   | Battery Pack                       | 1        |
| 17   | #4-40 x 12" Socket Head Cap Screws | 6        |
| 18   | #4-40 Machine Screw Nuts           | 6        |
        
        
## Switching Transistors
We were at the point that we had everything working **except** for the motor (which is kind of important). We got the motor working through some tinkering, and using a transistor to utilize the 6V battery pack for the motor. When connected directly to the 6v and ground it went super fast, but on max power from the PWM and transistor it could barely lift itself. Thanks to Mr. Dierolf we found that the bottleneck was the transistor, which could only give us about half power. So we switched from using a tiny little NPN transistor to using an actual 6V regulator. The wiring was different, so thanks Paul Weder for the wiring diagram. When switching it on for the first time it went hard. It went past 45, and into an uncontrollable oscillation. Problem solved :) 

# `Tuning`
Ah yes, tuning. If only we were able to make sufficent progress here before the fan wire broke.
After the whole fan incident, we could actually start tuning. And by that I mean it immediately started working at default values. Cool.

# `End Result`

https://github.com/jvaugha3038/PID-Project/assets/113116247/464c805f-a0af-45dd-803c-50ebdb393c28


# `Problems`
## Big problems
* The fan wire broke off at the base.
  * This is easily the worst thing that could have happened at the time it did. I hate the fan. We had to get a new fan (which was a different size), remake the arm, and re-tune it.
* The angle kept counting up even when the fan wasnt working.
  * We had to add an offset into the code to calibrate the gyroscope.
* simple_pid library didn't want to work.
  * Switched over to a PID function which works fine.
* A few variables didn't want to work with the PID function *even though they were global variables already*.
  * We made sure to tell the function that they were global variables.

## Less severe problems
* The arm can move side to side on the axle.
  * Don't worry about it its fine :)
* Wires interfered with the arm.
  * We tried (and failed) to crimp some wires ourselves, then gave up and used shorter wires and jumper wires.
* The supports are held together by the Metro M4.
  * *It's fine don't worry about it*
* When editing variables with the rotary encoder in the LCD menu, the edit page closed/opened too fast.
  * We added a cooldown to the button.
* The LCD wouldn't display the output variable.
  * Frankly I forgot why I wanted it to do this. Who cares. Thats what the fan does.
        
## Side Notes
* We used no acrylic
* String/thin wires are terrible. Avoid at all costs.    
* Planning before starting to build does indeed make the project take less time. Shocker.
* We finished this and the tic tac toe project on the same day.
* The onshape document had 3 pages, compared to the 21 pages in the tic tac toe project.
        
# `Reflection`
We used no acrylic, which is sure to make up for the absurd amount we used in the tic tac toe project. Speaking of which, this project felt much more manageable, since it was a much smaller and less ambitious project that used an LCD and a rotary encoder, both of which we used in the last class assignment we did. The fan breaking halted all progress for about a week, and my inability to understand simple-pid also didn't help, but we still managed to create a working project on time. Personally, I much preferred the PID function over simple-pid, because its much easier to understand. I just cut out some of the variables at the beginning of it, and started plugging numbers into it and it worked fine. Also the complete lack of tuning our project required made it very easy. 
