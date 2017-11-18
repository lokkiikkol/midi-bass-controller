# midi-bass-controller

ok, here is an overview of the parts you will need for this:

an arduino micro

4 fsr's: i used these: https://www.sparkfun.com/products/9376 connect one end to ground the other to 5v via a 4.7k resistor. the lead with the resistor goes to an analog in (directly at the lead before the resistor) (see in the arduino file which)

4 softpots, i used 500mm: https://www.sparkfun.com/products/8681 connect the leftmost lead to 5v via a 7.5k resistor and the middle one to ground. leave the right hand lead unconnected. from the leftmost lead to the corresponding analog in (see code again) (also before the resistor)

a joystick: https://www.sparkfun.com/products/9032 one outer lead to 5v the other outer lead to ground, middle to an analog in (see code which) i only use one axis, and the button is also not used
a pressure sensor: i used this https://www.conrad.ch/de/drucksensor-1-st-nxp-semiconductors-mpxv4006gc7u-0-kpa-bis-6-kpa-print-1182921.html see documentation for wiring, but it is one pin to 5v one to ground and one to an analog in, no extra hardware, all other pins are unconnected.

an accelerometer: i used https://www.aliexpress.com/item/GY-61-ADXL335-three-axis-accelerometer-tilt-angle-module-alternative-MMA7260/32803571540.html?spm=a2g0s.9042311.0.0.84d3Li direct connection to the arduino as well. (check code for which analog in) again i only used one axis, since i only need to get vibrato (fast movement of my hand on the neck) place the accelerometer somewhere where the movement is the biggest (i used the end of the neck)

3 momentary switches: one end to ground the other to an analog in (uses arduino internal pullups, check code for pin)
check code for the functions, or just try it. first switches fsr mode, second one switches to breath-controller mode, third one turns on fulllegato mode (triggering notes without fsr's) first and third together toggle bowmode (note is only on as long as fsr is pressed)

1 switch like this: https://www.conrad.ch/de/kippschalter-250-vac-2-a-1-x-einausein-335620-tastend0tastend-1-st-071086.html to switch octaves

check the code for the midi in cc's that switch the modes etc.
midi in circuit used: http://www.electronicsteacher.com/tutorial/midi-controller.php upper part of the diagram, pin 2 is the rx pin of the arduino. midi out pin is tx. (just a 220 ohm resistor to pin 5 of a midi controller pin 4 via 220 ohm to 5v, pin 2 is grounded)

usb midi: the arduino micro can be used as a usb-midi device, i used this to enable it: https://github.com/rkistner/arcore
it can be a little bit fiddly with newer arduino versions. i used 1.6.5


