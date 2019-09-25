Setup opencv3 according to this document:
https://jkjung-avt.github.io/opencv3-on-tx2/

OpenCV compilation issues:
https://github.com/opencv/opencv/issues/8704

Motor controller:
https://www.dfrobot.com/wiki/index.php/Quad_Motor_Driver_Shield_for_Arduino_SKU:DRI0039#More


Pin connections:

Servo | Interrupt (Position Feedback) | Control (PWM)
0     | 18                            | 5
1     | 2                             | 6
2     | 3                             | 7
3     | N/C                           | 8

Drive Motor:

Feedback Int: 18
Forward:
Backward:


Current Issues:
Singulation
Empty detection

