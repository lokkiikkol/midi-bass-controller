# midi-bass-controller

ok, here is an overview of the parts you will need for this:

an arduino micro

4 fsr's: i used these: https://www.sparkfun.com/products/9376 connect one end to ground the other to 5v via a 4.7k resistor. the lead with the resistor goes to an analog in (directly at the lead before the resistor) (see in the arduino file which)

4 softpots, i used 500mm: https://www.sparkfun.com/products/8681 connect the leftmost lead to 5v via a 7.5k resistor and the middle one to ground. leave the right hand lead unconnected. from the leftmost lead to the corresponding analog in (see code again) (also before the resistor) i had to cut the softpots sligthly on both sides to fit 4 on a bass neck that was originally designed for 5 strings (check picture)

a joystick: https://www.sparkfun.com/products/9032 one outer lead to 5v the other outer lead to ground, middle to an analog in (see code which) i only use one axis, and the button is also not used

a pressure sensor: i used this https://www.conrad.ch/de/drucksensor-1-st-nxp-semiconductors-mpxv4006gc7u-0-kpa-bis-6-kpa-print-1182921.html see documentation for wiring, but it is one pin to 5v one to ground and one to an analog in, no extra hardware, all other pins are unconnected.

an accelerometer: i used https://www.aliexpress.com/item/GY-61-ADXL335-three-axis-accelerometer-tilt-angle-module-alternative-MMA7260/32803571540.html?spm=a2g0s.9042311.0.0.84d3Li direct connection to the arduino as well. (check code for which analog in) again i only used one axis, since i only need to get vibrato (fast movement of my hand on the neck) place the accelerometer somewhere where the movement is the biggest (i used the end of the neck)

3 momentary switches: one end to ground the other to a digital in (uses arduino internal pullups, check code for pin)
check code for the functions, or just try it. first switches fsr mode, second one switches to breath-controller mode, third one turns on fulllegato mode (triggering notes without fsr's) first and third together toggle bowmode (note is only on as long as fsr is pressed)

1 switch like this: https://www.conrad.ch/de/kippschalter-250-vac-2-a-1-x-einausein-335620-tastend0tastend-1-st-071086.html to switch octaves

an led on PIN_LED connected via a current limiting resistor (between 1 to 10k depending on brightness needed and color of the led)

check the code for the midi in cc's that switch the modes etc. also the bass reacts to program changes, it forwards them on all active channels and you can save 127 playstates on the bass (channelmode, octaves etc.)

midi in circuit used: http://www.electronicsteacher.com/tutorial/midi-controller.php upper part of the diagram, pin 2 is the rx pin of the arduino. midi out pin is tx. (just a 220 ohm resistor to pin 5 of a midi controller pin 4 via 220 ohm to 5v, pin 2 is grounded)

usb midi: the arduino micro can be used as a usb-midi device, i used this to enable it: https://github.com/rkistner/arcore
it can be a little bit fiddly with newer arduino versions. i used 1.6.5

before you can use the controller you will have to calibrate it:

press the channelmode switch (PIN_BUTTON_CHANNEL) during startup of the arduino to access calibration:
start on the lowest (in pitch) string and press down just past the 24th fret, hit the fsr. the led will blink to indicate that the value was saved, press down just pass the 23th fret etc... once you reach past the 1st fret, the led will blink twice and you can continue on the 24th fret of the second string. once all strings are finished, the controller should work as expected.

a note on synths to use:

set your synth to monoponic and legato mode (a little glide when notes are played legato is nice) only retrigger attacks when all notes are released. this gives the most "bass" like feeling.

in polyphonic mode you will need four instances of the same instrument on four channels 2-5. channel 1 is reserved for general modulation (joystick (cc1), accelerometer (pitchbend) and breath (cc2)) each string is only controlled via the corresponding fsr. this enables fourpart polyphony and four part modulation via aftertouch.

in channelmode (channel 6 to 9) all modulation will be present on the selected channel. channels are selected via the 4 fsrs first fsr enables channel 6 and all strings are working monophonically. second fsr will be channel 7 etc.




