# Marlin-5X
Marlin-5X is 5-axis controller based on marlin framework.

It imeplements just movement parts of marlin(not temp controller or any other feauture).
For now it's just compatibale with Arduino Mega2560.
For moving six stepper motors (including extruder) in the loop function of main.ino implement plan_buffer_line function. Like
    plan_buffer_line(a, b, c, d, e, f, feed_rate, extruder);
