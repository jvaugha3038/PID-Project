# PID-Project

## Table of Contents
* [Table of Contents](#TableOfContents)
* [Goal](#Goal)
* [Process](#Process)
* [CAD](#CAD)
* [Code](#Code)
* [Building](#Building)
* [Tuning](#Tuning)
* [End Result](#EndResult)
* [Problems](#Problems)
* [Reflection](#Reflection)
---
# `Goal`


# `Process`


# `CAD`
We had made a planning papper before starting to outline the design we wanted. This made the process a lot easier, as I didn't have to make it up as I went. The basic design was pretty simple at first, two supports and an arm. It's always the details that are problematic, though. One such detail is how we have the arm spin. The original plan was to have a ball bearing in the arm and a rod through the supports and arms. It turned out the bearing was too big, and also unnecessary for this small of a design. We went instead for a bushing design, where there is a static rod connected that the arm spins around, no bearing needed.
There was also the issue of weight, as the tiny little motor needed to be able to lift itself and the arm off of the ground. I had designed the arm to be very light and have a counterweight to balance out the weight, and thanks to Onshape's handy center of mass feature, I was able to determine that the center of mass was just 100mm towards the motor. Given that the arm was 250mm long, that seemed to be a pretty good spot, and it confirmed that the motor wouldn't be overburdened. 

A rude awakening came when I remembered that we like need an arduino, screen, and rotary encoder to, ya know, to do the pid'ing. So I had to make space for those, which made it look a little worse to be honest. I got it all to work though, except for the battery pack. The pack was just way bigger than I thought it would be, and I couldn't fit it between the supports like I did with the arduino. My solution was to just have it sit on one of the supports. No connections, just sitting there :). I know this wasn't the most professional decision, but my idea was that it could help weigh down the support to stop it from moving while giving easy access to the batteries.
![Screenshot 2023-04-26 10 38 11 AM](https://user-images.githubusercontent.com/113116247/234610805-64f86760-13fe-4a1f-bbc3-2b34732a7c41.png)


# `Code`
The code was challenging overall, but once the main hurdle was cleared (finding good PID code online), it strted to get easier. The first problem was that both the gyroscope and the LCD screen required SDA and SCL pins, and the Metro M4 only had one of each. 

# `Building`


# `Building`


# `End Result`


# `Problems`


# `Reflection`
