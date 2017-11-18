  /*
* Arduino Ribbon Synth MIDI controller
* ------------------------------------
* ©2014 Dean Miller
* ©2016 Simon Iten many additions, 4 strings, 24 frets, fsr, channelmode, 
* midi message thinning, usb and din midi etc..
*/
#pragma GCC optimize ("-O3")
#include <EEPROM.h>




//------- Constants -------//

#define PIN_LED 13
// the 4 strings
#define PIN_SOFTPOT_1 A0
#define PIN_SOFTPOT_2 A1
#define PIN_SOFTPOT_3 A2
#define PIN_SOFTPOT_4 A3
//the 4 fsr's
#define PIN_FSR_1 A4
#define PIN_FSR_2 A5
#define PIN_FSR_3 A6
#define PIN_FSR_4 A7
#define PIN_BREATH A8
#define PIN_SUSTAIN A9
#define PIN_BUTTON_CHANNEL 3
#define PIN_BUTTON_BREATH 11
#define PIN_OCTAVE_UP 5
#define PIN_OCTAVE_DOWN 2
#define PIN_BUTTON_LEGATO 7
//joystick
#define PIN_JOYSTICK_Y A11
//accelerometer
#define PIN_ACCEL A10

#define UP 0
#define DOWN 1
#define SUSTAIN 2


#define NUM_STRINGS 4
#define PADDING 1
#define MINVELOCITY 60
//where does the channelmode start, default mode starts on channel 2  general modulation happens on channel one (kind of mpe))
#define CHANNEL_OFFSET 4
//how long before a note stays on when you release the fsr in non-bowmode
#define ATTACK 120
//which cc numbers are used to save settings and change the modes via midi.
#define SAVE_CC 115
#define CHANNELMODE_CC 119
#define FULLLEGATO_CC 118
#define OCTAVE_CC 117
#define BREATH_CC 116
#define ALLMUTE_CC 123
#define BOWMODE_CC 114
#define DUMMY_CC 113
//lowest open string to calculate midi note values on the neck
#define LOWEST_NOTE 4
//max breath-pressure
#define BREATHMAX 200
// corrects false note splutter when softpot is released
#define STABLETIME 15
//buttonpresstime (detect multiple presses, debounce)
#define BUTTONPRESS 100
//thin out cc messages
#define CONTROL_RATE 4
//thin out aftertouch messages
#define AFTERTOUCH_RATE 3

//------ Global Variables ---------//

/*
fret defs stored in EEPROM for calibration purposes.
Lower output voltages from USB ports result in different values read from
SoftPots and wonky fret definitions.
*/
unsigned int fretDefs[4][25];
const byte mapfsr[] = {
127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,
126,126,126,126,126,126,126,126,125,125,125,125,125,124,124,124,124,123,123,123,123,122,122,122,121,121,121,120,120,120,119,119,118,118,118,117,117,116,
116,116,115,115,114,114,113,113,112,112,111,111,110,110,109,109,108,107,107,106,106,105,105,104,103,103,102,102,101,100,100,99,98,98,97,96,96,95,94,94,93,
92,92,91,90,89,89,88,87,87,86,85,84,84,83,82,81,81,80,79,78,78,77,76,75,75,74,73,72,71,71,70,69,68,68,67,66,65,64,64,63,62,61,61,60,59,58,57,57,56,55,54,54,
53,52,51,51,50,49,48,47,47,46,45,44,44,43,42,42,41,40,39,39,38,37,37,36,35,34,34,33,32,32,31,30,30,29,28,28,27,26,26,25,25,24,23,23,22,22,21,20,20,19,19,18,18,
17,17,16,16,15,15,14,14,13,13,12,12,11,11,10,10,10,9,9,9,9,9,8,8,8,8,8,8,8,8,7,7,7,7,7,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,5,
5,5,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,
2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};
byte fsrTouched[] = {false, false, false, false};
byte activeNote[] = {0,0,0,0};
byte activeVelo[] = {0,0,0,0};
unsigned int fsrVals[] = {0, 0, 0, 0};
byte fsrReadOld[] = {0, 0, 0, 0};
byte breathOld = 0;
const byte fsrPins[] = {PIN_FSR_1, PIN_FSR_2, PIN_FSR_3, PIN_FSR_4};
unsigned int fsrInit[] = {0, 0, 0, 0};

unsigned int softPotVals[4];
const byte softPotPins[] = {PIN_SOFTPOT_1, PIN_SOFTPOT_2, PIN_SOFTPOT_3, PIN_SOFTPOT_4};

byte softPotValsOld[] = {0, 0, 0, 0};

byte fretTouched[4];
byte maybeNote[] {0,0,0,0};
byte stableCount[] = {0,0,0,0};
byte noteFretted[4];
byte stringActive[] = {false, false, false, false};



unsigned int calibrationMax[] = {0, 0, 0, 0};
unsigned int calibrationMin[4];
unsigned int breathMin;

//octave offset
byte octave = 2;

//default offsets for the 4 strings
byte offsets[] = {28, 33, 38, 43};

//states of control buttons
byte buttonStates[] = {false, false, false};
//unsigned int stickZeroX = 0;
unsigned int stickZeroY = 0;
// breath  mode, when off breath just controls cc2, when on, it can trigger notes as well
boolean breathMode = false;
boolean fullLegatoMode = false;
// allows for 4 sounds to be triggered by the 4 fsr's.
boolean channelMode = false;
//toggle bowmode (hold sound only as long as fsr is touched) and regular mode (hold sound as long as string is pressed)
boolean bowMode = false;
// store which fsr has been hit in channelmode
byte c = 0;
byte oldc = 0;
//unsigned int xPosold = 0;
unsigned int yPosold = 0;
unsigned int stickValold = 0;
byte afterBreath = 0;
byte breathNote = 0;
byte attackcounter[] = {0,0,0,0};
byte runningbyte = 0;
//midi in variables
byte state = 0;
byte saveSlot = 0;
byte statusByte;
byte dataByte1;
byte dataByte2;
byte breathcounter = 0;

 byte breathzero = 1;
 byte anyTouched = 0;
 byte controlchannel = 1;
 byte buttonread = 0;
 byte buttoncount = 0;
 byte oldbutton = 0;

 //Accelerometer
int readAccel;
int readAccelOld = 0;
int accelZero;
bool isMiddle = 0;
// thin control data a little
byte controlcounter = 0;
byte atcounter[] = {0,0,0,0};
//--------- Setup ----------------//
void setup() {


  
  digitalWrite(PIN_LED, HIGH);
  //read fret definitions from EEPROM
  for (byte i=NUM_STRINGS; i--;){
 
    fretDefs[i][0] = 255;
    for (byte j=1; j<24; j++){
      fretDefs[i][j] = EEPROM.read(j + (24*i));
    }
    fretDefs[i][24]=0;
    calibrationMin[i] = EEPROM.read(24 + (24*i));
  }

 
  //begin at MIDI spec baud rate
  Serial1.begin(31250);
  
  pinMode(PIN_BUTTON_BREATH, INPUT_PULLUP);  
  pinMode(PIN_BUTTON_LEGATO, INPUT_PULLUP);  
  pinMode(PIN_OCTAVE_UP, INPUT_PULLUP);  
  pinMode(PIN_OCTAVE_DOWN, INPUT_PULLUP);  
  pinMode(PIN_BUTTON_CHANNEL, INPUT_PULLUP); 
  
  pinMode(PIN_LED, OUTPUT);
  while(millis() < 800) {
    for (byte i=NUM_STRINGS; i--;){
      fsrInit[i] = analogRead(fsrPins[i]);
      unsigned int val = analogRead(softPotPins[i]);
      if (val > calibrationMax[i]) calibrationMax[i] = val;
    }
    //read pressure sensor minvalue
    breathMin = analogRead(PIN_BREATH);
    //calibrate joystick
    stickZeroY = analogRead(PIN_JOYSTICK_Y);
    //calibrate accelerometer
    accelZero = analogRead(PIN_ACCEL);
    
    } 
    digitalWrite(PIN_LED, LOW);
    //press leftmost (from above) button during startup to calibrate
    if ( digitalRead(PIN_BUTTON_CHANNEL) == LOW ) {
        calibrate();
  }
  //write defaults to all program loccations, overwriting everything
 /* if (digitalRead(PIN_BUTTON_BREATH) == LOW && !buttonStates[BREATH]) {
    buttonStates[BREATH] = true;
    writeDefaults();
  } */
}

//----------Main Loop---------------//
void loop() {
// -------------  SERIAL MIDI INPUT for Control of the Bass, see midiParser() function for details---------//
if (Serial1.available() > 0) {
    // read the incoming byte:
   byte incomingByte = Serial1.read();
  
   switch (state){
      case 0:
      if (incomingByte > 0x7F && incomingByte < 0xF0 ){    // this reads all statusbytes, realtime stuff is ignored
         statusByte = incomingByte;
         state=1;
        }
        if (incomingByte == 0xF0) state=3;
       case 1:
       // get first databyte
       if(incomingByte < 0x80) {
        dataByte1 = incomingByte;
        //check for two byte messages...
       if ((statusByte & 0xF0) == 0xC0  | (statusByte & 0xF0) == 0xD0) {
        midiParser();
        state = 0; //reset state machine 
       } else state = 2;
       } else{
      state = 0;  // reset state machine as this should be a note number
      }
       break;
       
       case 2:
       // get second databyte
       if(incomingByte < 0x80) {
       dataByte2 = incomingByte;
       }
       midiParser();
         state = 0;  // reset state machine to start
         break;
          //stay here for sysex, wait for it to end...
         case 3:
         if (incomingByte == 0xF7) state=0;
         if(incomingByte > 0x7F && incomingByte < 0xF0) {
              statusByte = incomingByte;
              state=1;   
         }
         break;
  }
}
//reset fsrVals to zero
memset(fsrVals,0,sizeof(fsrVals));

//------------------------- READ ALL SENSORS (Softpots and FSRs)---------------//
 //read values of all sensors 
 unsigned int fsrRead[4];
 
 unsigned int breathRead = analogRead(PIN_BREATH);
//debug without breathsensor attached
//breathRead = 5;
   if (breathMode) {
     
    if (!channelMode) {
if (breathRead > (breathMin + 25) && !fsrTouched[0]) {
  
  
 fsrVals[0] = map((breathRead), breathMin + 25, BREATHMAX, MINVELOCITY, 127);
    memset(attackcounter,0,sizeof(attackcounter));
     
      fsrVals[1] = fsrVals[0];
      fsrVals[2] = fsrVals[0];
      fsrVals[3] = fsrVals[0];
      fsrTouched[0] = 1;
      fsrTouched[1] = 1;
      fsrTouched[2] = 1;
      fsrTouched[3] = 1;
    }

   
    
   } else {
    if (breathRead > (breathMin + 25) && !fsrTouched[c]) {
   
      fsrVals[c] = map((breathRead), breathMin + 25, BREATHMAX, MINVELOCITY, 127);
      attackcounter[c] = 0;
      fsrTouched[c] = 1; 
    }
 
   

  }
    anyTouched = fsrTouched[0] + fsrTouched[1] + fsrTouched[2] + fsrTouched[3];
if (!channelMode) {
 if (breathRead < (breathMin + 15) && fsrTouched[0]) {
      memset(fsrTouched,0,sizeof(fsrTouched));
 }
 } else {
      if (breathRead < (breathMin + 15) && fsrTouched[c]) {
      fsrTouched[c] = 0;
    }
   
 }
      
     //fsr 1 to 4 in breathmode
     fsrRead[0] = analogRead(fsrPins[0]);
      byte frettouch; 
    if (channelMode) {
    frettouch = fretTouched[0] + fretTouched[1] + fretTouched[2] + fretTouched[3];
    }
    if (!channelMode) frettouch = fretTouched[0];
    if (frettouch) {
 if (fsrRead[0] < (fsrInit[0] - 20)) {
 
   fsrRead[0] = mapfsr[(fsrRead[0] - 10) >> 1];
      if ((fsrRead[0] - fsrReadOld[0])) {
        c=0;
        atcounter[0] = atcounter[0] + 1;
         if ( !anyTouched && atcounter[0] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD1 + CHANNEL_OFFSET, fsrRead[0]);
      else aftertouchChange(0xD1 ,fsrRead[0]);
      atcounter[0] = 0;
         }
      }
       fsrReadOld[0] = fsrRead[0];
    }
    }
    
     fsrRead[1] = analogRead(fsrPins[1]);
     if (!channelMode) frettouch = fretTouched[1];
    if (frettouch) {
 if (fsrRead[1] < (fsrInit[1] - 20)) {
   fsrRead[1] = mapfsr[(fsrRead[1] - 10) >> 1];
       if ((fsrRead[1] - fsrReadOld[1])) {
        c= 1;
         atcounter[1] = atcounter[1] + 1;
         if ( !anyTouched && atcounter[1] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, fsrRead[1]);
      else aftertouchChange(0xD2 ,fsrRead[1]);
      atcounter[1] = 0;
         }
      }
       fsrReadOld[1] = fsrRead[1]; 
    }
    }
    
     fsrRead[2] = analogRead(fsrPins[2]);
     if (!channelMode) frettouch = fretTouched[2];
    if (frettouch) {
 if (fsrRead[2] < (fsrInit[2] - 20)) {
  fsrRead[2] = mapfsr[(fsrRead[2] - 10) >> 1];
     if ((fsrRead[2] - fsrReadOld[2])) {
      c= 2;
      atcounter[2] = atcounter[2] + 1;
       if ( !anyTouched && atcounter[2] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD3 + CHANNEL_OFFSET, fsrRead[2]);
      else aftertouchChange(0xD3 ,fsrRead[2]);
      atcounter[2] = 0;
      }
     }
       fsrReadOld[2] = fsrRead[2]; 
    }
    }
    
     fsrRead[3] = analogRead(fsrPins[3]);
     if (!channelMode) frettouch = fretTouched[3];
    if (frettouch) {
 if (fsrRead[3] < (fsrInit[3] - 20)) {
   fsrRead[3] = mapfsr[(fsrRead[3] - 10) >> 1];
      if ((fsrRead[3] - fsrReadOld[3])) {
        c = 3;
        atcounter[3] = atcounter[3] + 1;
        if ( !anyTouched && atcounter[3] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD4 + CHANNEL_OFFSET, fsrRead[3]);
      else aftertouchChange(0xD4 ,fsrRead[3]);
      atcounter[3] = 0;
      }
      }
       fsrReadOld[3] = fsrRead[3]; 
    }
    }
     if (c != oldc && channelMode) {
      noteOn(0x91 + oldc, activeNote[0], 0);
       memset(fsrTouched,0,sizeof(fsrTouched));
     }

 softPotVals[0] = analogRead(softPotPins[0]);
 softPotVals[0] = map(softPotVals[0], calibrationMin[0], calibrationMax[0], 0, 255);
 softPotVals[0] = constrain(softPotVals[0], 0, 255);
  softPotVals[1] = analogRead(softPotPins[1]);
 softPotVals[1] = map(softPotVals[1], calibrationMin[1], calibrationMax[1], 0, 255);
 softPotVals[1] = constrain(softPotVals[1], 0, 255);
  softPotVals[2] = analogRead(softPotPins[2]);
 softPotVals[2] = map(softPotVals[2], calibrationMin[2], calibrationMax[2], 0, 255);
 softPotVals[2] = constrain(softPotVals[2], 0, 255);
  softPotVals[3] = analogRead(softPotPins[3]);
 softPotVals[3] = map(softPotVals[3], calibrationMin[3], calibrationMax[3], 0, 255);
 softPotVals[3] = constrain(softPotVals[3], 0, 255);
  
}
//end of BREATHMODE
  //BREATH-CONTROLLER//

if (breathcounter < 4) {
  breathcounter = breathcounter + 1;
 
} else if (!anyTouched) {
  unsigned int breathValue;
  breathcounter = 0;
 
  breathValue = breathRead;
  
  if (breathValue > breathMin + 20) {
    breathzero = 0;
    if (breathValue > BREATHMAX) breathValue = BREATHMAX;
   afterBreath = map(breathValue, breathMin + 15, BREATHMAX, 0, 127);
   if (((breathOld - breathValue))) controllerChange(2,afterBreath);
    breathOld = breathValue;
  } else if ((!breathzero)) {
    breathzero = 1;
    controllerChange(2,0);
     breathOld = breathValue;
   }
}
  

if (!breathMode) {
  if (channelMode) {
   if (breathRead > (breathMin + 25) && !breathNote) {
    noteOn(0x95,60,127);
    breathNote = 1;
    }
 
   
    if (breathRead < (breathMin + 15) && breathNote) {
    noteOn(0x95,60,0);
    breathNote = 0;
    }
  
} else {

  if (breathRead > (breathMin + 25) && !breathNote) {
    noteOn(0x99,60,127);
    breathNote = 1;
    }
 
   
    if (breathRead < (breathMin + 15) && breathNote) {
    noteOn(0x99,60,0); 
    breathNote = 0;
    }
}
//BEGINNING OF NONE BREATHMODE  
    //-STRING ONE-//
    //read fsr vals
    byte bowTemp = bowMode;
    byte frettouch; 
    if (channelMode) {
    frettouch = fretTouched[0] + fretTouched[1] + fretTouched[2] + fretTouched[3];
    }
    if (!channelMode) frettouch = fretTouched[0];
    fsrRead[0] = analogRead(fsrPins[0]);
     if (frettouch) {
    if (fsrRead[0] < (fsrInit[0] - 30) && !fsrTouched[0]) {
      attackcounter[0] = 0;
      fsrVals[0] = map((fsrRead[0]), fsrInit[0] - 30, 60, MINVELOCITY, 127);
      fsrTouched[0] = 1;
      c= 0;
    }
    if (fsrRead[0] < (fsrInit[0] - 20) && fsrTouched[0]) {
     

   fsrRead[0] = mapfsr[(fsrRead[0] - 10) >> 1];
     if ((fsrRead[0] - fsrReadOld[0]) && attackcounter[0]) {
      atcounter[0] = atcounter[0] + 1;
      if (atcounter[0] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD1 + CHANNEL_OFFSET, fsrRead[0]);
      else aftertouchChange(0xD1 ,fsrRead[0]);
      atcounter[0] = 0;
      }
     }
       if (bowTemp) attackcounter[0] = 1;
       if (attackcounter[0] < ATTACK) attackcounter[0] = attackcounter[0] + 1;
      
       fsrReadOld[0] = fsrRead[0]; 

    }
    if (fsrRead[0] > (fsrInit[0] - 15) && fsrTouched[0]) {
      if (channelMode)  aftertouchChange(0xD1 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD1 ,0);
      fsrTouched[0] = 0;   
    } 
   
} else fsrTouched[0] = 0;
    //read the value of all the softPots
    softPotVals[0] = analogRead(softPotPins[0]);
    softPotVals[0] = map(softPotVals[0], calibrationMin[0], calibrationMax[0], 0, 255);
   softPotVals[0] = constrain(softPotVals[0], 0, 255);

//--STRING TWO--//
 if (!channelMode) frettouch = fretTouched[1];
   fsrRead[1] = analogRead(fsrPins[1]);
     if (frettouch) {
    if (fsrRead[1] < (fsrInit[1] - 30) && !fsrTouched[1]) {
      attackcounter[1] = 0;
      fsrVals[1] = map((fsrRead[1]), fsrInit[1] - 30, 60, MINVELOCITY, 127);
      fsrTouched[1] = 1;
      c= 1;
    }
    if (fsrRead[1] < (fsrInit[1] - 20) && fsrTouched[1]) {
     
   fsrRead[1] = mapfsr[(fsrRead[1] - 10) >> 1];
       if ((fsrRead[1] - fsrReadOld[1]) && attackcounter[1]) {
         atcounter[1] = atcounter[1] + 1;
      if (atcounter[1] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, fsrRead[1]);
      else aftertouchChange(0xD2 ,fsrRead[1]);
      atcounter[1] = 0;
      }
       }
      if (bowTemp) attackcounter[1] = 1;
       if (attackcounter[1] < ATTACK) attackcounter[1] = attackcounter[1] + 1;
        
       fsrReadOld[1] = fsrRead[1]; 
    }
    if (fsrRead[1] > (fsrInit[1] - 15) && fsrTouched[1]) {
        if (channelMode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD2 ,0);
      fsrTouched[1] = 0;
    } 
   
} else fsrTouched[1] = 0;
   
    //read the value of all the softPots
    softPotVals[1] = analogRead(softPotPins[1]);
    softPotVals[1] = map(softPotVals[1], calibrationMin[1], calibrationMax[1], 0, 255);
   softPotVals[1] = constrain(softPotVals[1], 0, 255);

//-STRING THREE--//
 if (!channelMode) frettouch = fretTouched[2];
 fsrRead[2] = analogRead(fsrPins[2]);
     if (frettouch) {
    if (fsrRead[2] < (fsrInit[2] - 30) && !fsrTouched[2]) {
      attackcounter[2] = 0;
     
      fsrVals[2] = map((fsrRead[2]), fsrInit[2] - 30, 60, MINVELOCITY, 127);
      fsrTouched[2] = 1;
      c= 2;
    }
    if (fsrRead[2] < (fsrInit[2] - 20) && fsrTouched[2]) {
     
   fsrRead[2] = mapfsr[(fsrRead[2] - 10) >> 1];
        if ((fsrRead[2] - fsrReadOld[2]) && attackcounter[2]) {
           atcounter[2] = atcounter[2] + 1;
      if (atcounter[2] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD3 + CHANNEL_OFFSET, fsrRead[2]);
      else aftertouchChange(0xD3 ,fsrRead[2]);
      atcounter[2] = 0;
      }
        }
      if (bowTemp) attackcounter[2] = 1;
       if (attackcounter[2] < ATTACK) attackcounter[2] = attackcounter[2] + 1;
        
       fsrReadOld[2] = fsrRead[2]; 
    }
    if (fsrRead[2] > (fsrInit[2] - 15) && fsrTouched[2]) {
        if (channelMode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD2 ,0);
      fsrTouched[2] = 0;
    } 
   
   
} else fsrTouched[2] = 0;
    //read the value of all the softPots
    softPotVals[2] = analogRead(softPotPins[2]);
    softPotVals[2] = map(softPotVals[2], calibrationMin[2], calibrationMax[2], 0, 255);
   softPotVals[2] = constrain(softPotVals[2], 0, 255);

//-- STRING FOUR--//
 if (!channelMode) frettouch = fretTouched[3];
 fsrRead[3] = analogRead(fsrPins[3]);
    if (frettouch) {
    if (fsrRead[3] < (fsrInit[3] - 30) && !fsrTouched[3]) {
      attackcounter[3] = 0;
      
      fsrVals[3] = map((fsrRead[3]), fsrInit[3] - 30, 60, MINVELOCITY, 127);
      fsrTouched[3] = 1;
      c=3;
    }
    if (fsrRead[3] < (fsrInit[3] - 20) && fsrTouched[3]) {
     
   fsrRead[3] = mapfsr[(fsrRead[3] - 10) >> 1];
       if ((fsrRead[3] - fsrReadOld[3]) && attackcounter[3]) {
         atcounter[3] = atcounter[3] + 1;
      if (atcounter[3] == AFTERTOUCH_RATE) {
      if (channelMode)  aftertouchChange(0xD4 + CHANNEL_OFFSET, fsrRead[3]);
      else aftertouchChange(0xD4 ,fsrRead[3]);
      atcounter[3] = 0;
      }
       }
        if (bowTemp) attackcounter[3] = 1;
       if (attackcounter[3] < ATTACK) attackcounter[3] = attackcounter[3] + 1;
      
      fsrReadOld[3] = fsrRead[3];
    }
    if (fsrRead[3] > (fsrInit[3] - 15) && fsrTouched[3]) {
        if (channelMode)  aftertouchChange(0xD3 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD3 ,0);
      fsrTouched[3] = 0;
    } 
    } else fsrTouched[3] = 0;
     
   
    //read the value of all the softPots
    softPotVals[3] = analogRead(softPotPins[3]);
    softPotVals[3] = map(softPotVals[3], calibrationMin[3], calibrationMax[3], 0, 255);
   softPotVals[3] = constrain(softPotVals[3], 0, 255);

}
  
 //------------------DETERMINE FRETS----------------------//
 
    //---------Get Fret Numbers------//

    //--STRING ONE--//
 
   byte softPotVal = softPotVals[0];
   
    //check for open strings
    if (softPotVal == 255) {
      softPotValsOld[0] = softPotVal;
      fretTouched[0]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (softPotVal <= fretDefs[0][k] && 
          softPotVal > fretDefs[0][j] ) {
            
if (((uint8_t)(abs( (int8_t) (softPotVal-softPotValsOld[0]))) > PADDING)) {
  softPotValsOld[0] = softPotVal;
            fretTouched[0] = j;
    
            break;
  
  } 
    }
    }
    
    if (softPotVal <= fretDefs[0][23]) {
      softPotValsOld[0] = softPotVal;
      fretTouched[0]=24;
    
    }

//--STRING TWO--//

   softPotVal = softPotVals[1];
    
    //check for open strings
    if (softPotVal == 255) {
      softPotValsOld[1] = softPotVal;
      fretTouched[1]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (softPotVal <= fretDefs[1][k] && 
          softPotVal > fretDefs[1][j] ) {
if (((uint8_t)(abs( (int8_t) (softPotVal-softPotValsOld[1]))) > PADDING)) {
  
  softPotValsOld[1] = softPotVal;
            fretTouched[1] = j;

            break;
} 
  }
    }
    
    if (softPotVal <= fretDefs[1][23]) {
      softPotValsOld[1] = softPotVal;
      fretTouched[1]=24;
     
    }

    //--STRING THREE--//

       softPotVal = softPotVals[2];
   
    //check for open strings
    if (softPotVal == 255) {
      softPotValsOld[2] = softPotVal;
      fretTouched[2]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (softPotVal <= fretDefs[2][k] && 
          softPotVal > fretDefs[2][j]) {
            
         
if(((uint8_t)(abs( (int8_t) (softPotVal-softPotValsOld[2]))) > PADDING)) {
  
  softPotValsOld[2] = softPotVal;
            fretTouched[2] = j;
 
            break;
} 

  }
    }
    
    if (softPotVal <= fretDefs[2][23]) {
      softPotValsOld[2] = softPotVal;
      fretTouched[2]=24;
  
  
      
    }

    //---STRING FOUR--//

       softPotVal = softPotVals[3];
   
    //check for open strings
    if (softPotVal == 255) {
      softPotValsOld[3] = softPotVal;
      fretTouched[3]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (softPotVal <= fretDefs[3][k] && 
          softPotVal > fretDefs[3][j] ) {
if (((uint8_t)(abs( (int8_t) (softPotVal-softPotValsOld[3]))) > PADDING)) {
  
  softPotValsOld[3] = softPotVal;
            fretTouched[3] = j;
           
            break;
} 

  }
    }
    
    if (softPotVal <= fretDefs[3][23]) {
      softPotValsOld[3] = softPotVal;
      fretTouched[3]=24;
     
      
    }
  
 //----------- FULL LEGATO MODE-------------------//
 
 //if we are in full legato mode, run the functions
 if (fullLegatoMode) {
   byte vel = 80;
  
  //--STRING ONE--//
   if(channelMode) {
   if (c != oldc) {
      noteOn(0x91 + oldc, activeNote[0], 0);
    if (stringActive[0]) noteOn(0x91 + c, activeNote[0], vel);
    }
   } else c = 0;
    if (fretTouched[0]) {
   
          noteFretted[0] = fretTouched[0] + offsets[0];
   
    
      
      if (!stringActive[0]) {
        noteOn(0x91 + c, noteFretted[0], vel);
        
        //register new note as the active one
        activeNote[0] = noteFretted[0];
       
        stringActive[0] = true;
      } else if (noteFretted[0] > activeNote[0]) {
          noteOn(0x91 + c, noteFretted[0], vel);
          
          //turn off old note
     
        noteOn(0x91 +c, activeNote[0], 0);
    
         
        
          //register new note as the active one
          activeNote[0] = noteFretted[0];
         
        
      } else if (noteFretted[0] < activeNote[0]) {
    if (noteFretted[0] == maybeNote[0]) stableCount[0] += 1;
    if (stableCount[0] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[0], activeVelo[0]);
        noteOn(0x91 + c, activeNote[0], 0);
     
      
      //register new note as the active one
      activeNote[0] = noteFretted[0];
      stableCount[0] = 0;
      maybeNote[0] = 0;
    }
    maybeNote[0] = noteFretted[0];
   }
    }

   //--STRING TWO--//
   if(channelMode) {
   if (c != oldc) {
      noteOn(0x91 + oldc, activeNote[1], 0);
    if (stringActive[1]) noteOn(0x91 + c, activeNote[1], vel);
    }
   } else c = 1;
    if (fretTouched[1]) {
   
          noteFretted[1] = fretTouched[1] + offsets[1];
   
    
      
      if (!stringActive[1]) {
        noteOn(0x91 + c, noteFretted[1], vel);
        
        //register new note as the active one
        activeNote[1] = noteFretted[1];
      
        stringActive[1] = true;
      } else if (noteFretted[1] > activeNote[1]) {
          noteOn(0x91 + c, noteFretted[1], vel);
          
          //turn off old note
     
        noteOn(0x91 +c, activeNote[1], 0);
    
        
        
          //register new note as the active one
          activeNote[1] = noteFretted[1];
         
        
      } else if (noteFretted[1] < activeNote[1]) {
    if (noteFretted[1] == maybeNote[1]) stableCount[1] += 1;
    if (stableCount[1] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[1], activeVelo[1]);
        noteOn(0x91 + c, activeNote[1], 0);
     
      
      //register new note as the active one
      activeNote[1] = noteFretted[1];
      stableCount[1] = 0;
      maybeNote[1] = 0;
    }
    maybeNote[1] = noteFretted[1];
   }
    }

     //--STRING THREE--//
   if(channelMode) {
   if (c != oldc) {
      noteOn(0x91 + oldc, activeNote[2], 0);
    if (stringActive[2]) noteOn(0x91 + c, activeNote[2], vel);
    }
   } else c = 2;
    if (fretTouched[2]) {
   
          noteFretted[2] = fretTouched[2] + offsets[2];
   
    
      
      if (!stringActive[2]) {
        noteOn(0x91 + c, noteFretted[2], vel);
        
        //register new note as the active one
        activeNote[2] = noteFretted[2];
      
        stringActive[2] = true;
      } else if (noteFretted[2] > activeNote[2]) {
          noteOn(0x91 + c, noteFretted[2], vel);
          
          //turn off old note
     
        noteOn(0x91 +c, activeNote[2], 0);
    
          
        
          //register new note as the active one
          activeNote[2] = noteFretted[2];
         
        
      }else if (noteFretted[2] < activeNote[2]) {
    if (noteFretted[2] == maybeNote[2]) stableCount[2] += 1;
    if (stableCount[2] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[2], activeVelo[2]);
        noteOn(0x91 + c, activeNote[2], 0);
     
      
      //register new note as the active one
      activeNote[2] = noteFretted[2];
      stableCount[2] = 0;
      maybeNote[2] = 0;
    }
    maybeNote[2] = noteFretted[2];
   }
    }
    
     //--STRING FOUR--//
   if(channelMode) {
   if (c != oldc) {
      noteOn(0x91 + oldc, activeNote[3], 0);
    if (stringActive[3]) noteOn(0x91 + c, activeNote[3], vel);
    }
   } else c = 3;
    if (fretTouched[3]) {
   
          noteFretted[3] = fretTouched[3] + offsets[3];
   
    
      
      if (!stringActive[3]) {
        noteOn(0x91 + c, noteFretted[3], vel);
        
        //register new note as the active one
        activeNote[3] = noteFretted[3];
       
        stringActive[3] = true;
      } else if (noteFretted[3] > activeNote[3]) {
          noteOn(0x91 + c, noteFretted[3], vel);
          
          //turn off old note
     
        noteOn(0x91 +c, activeNote[3], 0);
    
         
        
          //register new note as the active one
          activeNote[3] = noteFretted[3];
        
        
      } else if (noteFretted[3] < activeNote[3]) {
    if (noteFretted[3] == maybeNote[3]) stableCount[3] += 1;
    if (stableCount[3] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[3], activeVelo[3]);
        noteOn(0x91 + c, activeNote[3], 0);
     
      
      //register new note as the active one
      activeNote[3] = noteFretted[3];
      stableCount[3] = 0;
      maybeNote[3] = 0;
    }
    maybeNote[3] = noteFretted[3];
   }
    }
  
 // ---------- CLEANUP FULL LEGATO-------------------//
 
   //send not off messages and reset necessary things
   
//--STRING ONE--//
       if (!channelMode) {
    c = 0;
    oldc = 0;
    }
   
    //no fret is touched and the string is marked active
    if ((!fretTouched[0] && stringActive[0]) ){
      
    
         noteOn(0x91 + oldc, activeNote[0], 0);
   
      //mark as inactive
      stringActive[0] = false;
     
    }

 //--STRING TWO--//
       if (!channelMode) {
    c = 1;
    oldc = 1;
    }
   
    //no fret is touched and the string is marked active
    if ((!fretTouched[1] && stringActive[1]) ){
    
        
         noteOn(0x91 + oldc, activeNote[1], 0);
     
      //mark as inactive
      stringActive[1] = false;
    
    }

    //--STRING THREE--//
       if (!channelMode) {
    c = 2;
    oldc = 2;
    }
   
    //no fret is touched and the string is marked active
    if ((!fretTouched[2] && stringActive[2]) ){
      
         noteOn(0x91 + oldc, activeNote[2], 0);
     
      //mark as inactive
      stringActive[2] = false;
     
    }

    //--STRING FOUR--//
       if (!channelMode) {
    c = 3;
    oldc = 3;
    }
   
    //no fret is touched and the string is marked active
    if ((!fretTouched[3] && stringActive[3]) ){
       
          noteOn(0x91 + oldc, activeNote[3], 0);
        
      //mark as inactive
      stringActive[3] = false;
     
    }
 }
 //END OF FULL LEGATO MODE//
 
 //-----------LEGATO TEST ----------------//
 
 //otherwise just do the regular thing
 else {
   //test for legato action
   
 //----STRING ONE--//
      if (!channelMode) {
    c = 0;
    oldc = 0;
    }
    if (stringActive[0]) {
       if (fretTouched[0]) noteFretted[0] = fretTouched[0] + offsets[0];
  
   if (noteFretted[0] > activeNote[0]) {
   
      //turn on new note

      noteOn(0x91 + c, noteFretted[0], activeVelo[0]);
        noteOn(0x91 + oldc, activeNote[0], 0);
     
      
      //register new note as the active one
      activeNote[0] = noteFretted[0];
     
     
   } else if (noteFretted[0] < activeNote[0]) {
    if (noteFretted[0] == maybeNote[0]) stableCount[0] += 1;
    if (stableCount[0] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[0], activeVelo[0]);
        noteOn(0x91 + oldc, activeNote[0], 0);
     
      
      //register new note as the active one
      activeNote[0] = noteFretted[0];
      stableCount[0] = 0;
      maybeNote[0] = 0;
    }
    maybeNote[0] = noteFretted[0];
   }
    }
    
else if (fretTouched[0] && fsrTouched[c] && !fsrVals[c]) {
   
          noteFretted[0] = fretTouched[0] + offsets[0];
   

      

    noteOn(0x91 +c, noteFretted[0], activeVelo[0]);
   
        //register new note as the active one
        activeNote[0] = noteFretted[0];
      
        stringActive[0] = true;

    }

  //----STRING TWO--//
      if (!channelMode) {
    c = 1;
    oldc = 1;
    }
    if (stringActive[1]) {
     if (fretTouched[1])   noteFretted[1] = fretTouched[1] + offsets[1];
 
   if (noteFretted[1] > activeNote[1]) {
    
      //turn on new note

      noteOn(0x91 + c, noteFretted[1], activeVelo[1]);
        noteOn(0x91 + oldc, activeNote[1], 0);
    
      
      //register new note as the active one
      activeNote[1] = noteFretted[1];
    
      
   } else if (noteFretted[1] < activeNote[1]) {
    if (noteFretted[1] == maybeNote[1]) stableCount[1] += 1;
    if (stableCount[1] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[1], activeVelo[1]);
        noteOn(0x91 + oldc, activeNote[1], 0);
     
      
      //register new note as the active one
      activeNote[1] = noteFretted[1];
      stableCount[1] = 0;
      maybeNote[1] = 0;
    }
    maybeNote[1] = noteFretted[1];
   }
    }
    
else if (fretTouched[1] && fsrTouched[c] && !fsrVals[c]) {
   
          noteFretted[1] = fretTouched[1] + offsets[1];
 
    noteOn(0x91 +c, noteFretted[1], activeVelo[1]);
     //    free(activeNotes[1]);
        //register new note as the active one
        activeNote[1] = noteFretted[1];
       
        stringActive[1] = true;

    }
    //----STRING THREE--//
      if (!channelMode) {
    c = 2;
    oldc = 2;
    }
    if (stringActive[2]) {
      if (fretTouched[2])    noteFretted[2] = fretTouched[2] + offsets[2];

   if (noteFretted[2] > activeNote[2]) {
    
      //turn on new note

      noteOn(0x91 + c, noteFretted[2], activeVelo[2]);
        noteOn(0x91 + oldc, activeNote[2], 0);
   
      
      //register new note as the active one
      activeNote[2] = noteFretted[2];
   
     
   } else if (noteFretted[2] < activeNote[2]) {
    if (noteFretted[2] == maybeNote[2]) stableCount[2] += 1;
    if (stableCount[2] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[2], activeVelo[2]);
        noteOn(0x91 + oldc, activeNote[2], 0);
     
      
      //register new note as the active one
      activeNote[2] = noteFretted[2];
      stableCount[2] = 0;
      maybeNote[2] = 0;
    }
    maybeNote[2] = noteFretted[2];
   }
    }
    
 else if (fretTouched[2] && fsrTouched[c] && !fsrVals[c] ) {
   
          noteFretted[2] = fretTouched[2] + offsets[2];
   

      noteOn(0x91 +c, noteFretted[2], activeVelo[2]);
      //   free(activeNotes[2]);
        //register new note as the active one
        activeNote[2] = noteFretted[2];
      
        stringActive[2] = true;

    }
    //----STRING FOUR--//
      if (!channelMode) {
    c = 3;
    oldc = 3;
    }
    if (stringActive[3]) {
        if (fretTouched[3])  noteFretted[3] = fretTouched[3] + offsets[3];

   if (noteFretted[3] > activeNote[3]) {
    
      //turn on new note

      noteOn(0x91 + c, noteFretted[3], activeVelo[3]);
        noteOn(0x91 + oldc, activeNote[3], 0);
    
      
      //register new note as the active one
      activeNote[3] = noteFretted[3];
    
     
   } else if (noteFretted[3] < activeNote[3]) {
    if (noteFretted[3] == maybeNote[3]) stableCount[3] += 1;
    if (stableCount[3] == STABLETIME) {
       noteOn(0x91 + c, noteFretted[3], activeVelo[3]);
        noteOn(0x91 + oldc, activeNote[3], 0);
     
      
      //register new note as the active one
      activeNote[3] = noteFretted[3];
      stableCount[3] = 0;
      maybeNote[3] = 0;
    }
    maybeNote[3] = noteFretted[3];
   }
    }
    
 else if (fretTouched[3] && fsrTouched[c] && !fsrVals[c]) {
   
          noteFretted[3] = fretTouched[3] + offsets[3];
   
  
   noteOn(0x91 +c, noteFretted[3], activeVelo[3]);
  
        //register new note as the active one
        activeNote[3] = noteFretted[3];
       
        stringActive[3] = true;
    }
  
  //------------PICK NOTES--------------------//
  
   //use this info to determine which notes to pluck

  
  //---STRING ONE----//
    if (!channelMode) {
    c = 0;
    oldc = 0;
    }
    //if the fsr was hit, play the fretted note
    if ((fsrVals[c] && fretTouched[0]) ){
          noteFretted[0] = fretTouched[0] + offsets[0];
   
      
      if (stringActive[0]){
       noteOn(0x91 + oldc , activeNote[0], 0);
      
        }
      
      if (!stringActive[0]) {
        
        //mark string as active
        stringActive[0] = true;
      }
        //register with active notes
        
        activeNote[0] = noteFretted[0];
        activeVelo[0] = fsrVals[c];
    
       
        
        //turn on fretted note
         noteOn(0x91 + c, activeNote[0], activeVelo[0]);
      
      }

      //---STRING TWO----//
    if (!channelMode) {
    c = 1;
    oldc = 1;
    }
    //if the fsr was hit, play the fretted note
    if ((fsrVals[c] && fretTouched[1]) ){
          noteFretted[1] = fretTouched[1] + offsets[1];
   
      
      if (stringActive[1]){
        noteOn(0x91 + oldc , activeNote[1], 0);
      
        }
      
      if (!stringActive[1]) {
        
        //mark string as active
        stringActive[1] = true;
      }
        //register with active notes
        
        activeNote[1] = noteFretted[1];
        activeVelo[1] = fsrVals[c];
    
     
        
        //turn on fretted note
         noteOn(0x91 + c, activeNote[1], activeVelo[1]);
      
      }

      //---STRING THREE----//
    if (!channelMode) {
    c = 2;
    oldc = 2;
    }
    //if the fsr was hit, play the fretted note
    if ((fsrVals[c] && fretTouched[2]) ){
          noteFretted[2] = fretTouched[2] + offsets[2];
   
      
      if (stringActive[2]){
        noteOn(0x91 + oldc , activeNote[2], 0);
       
        }
      
      if (!stringActive[2]) {
        
        //mark string as active
        stringActive[2] = true;
      }
        //register with active notes
        
        activeNote[2] = noteFretted[2];
        activeVelo[2] = fsrVals[c];
    
      
        
        //turn on fretted note
         noteOn(0x91 + c, activeNote[2], activeVelo[2]);
      
      }

      //---STRING FOUR----//
    if (!channelMode) {
    c = 3;
    oldc = 3;
    }
    //if the fsr was hit, play the fretted note
    if ((fsrVals[c] && fretTouched[3]) ){
          noteFretted[3] = fretTouched[3] + offsets[3];
   
      
      if (stringActive[3]){
        noteOn(0x91 + oldc , activeNote[3], 0);
      
        }
      
      if (!stringActive[3]) {
        
        //mark string as active
        stringActive[3] = true;
      }
        //register with active notes
        
        activeNote[3] = noteFretted[3];
        activeVelo[3] = fsrVals[c];
    
       
        
        //turn on fretted note
         noteOn(0x91 + c, activeNote[3], activeVelo[3]);
      
      }
    
    
    //------------------CLEAN UP------------------------//
    
  //send not off messages and reset necessary things

//---STRING ONE----//
       if (!channelMode) {
    c = 0;
    oldc = 0;
    }

    //no fret is touched and the string is marked active
    if (!fretTouched[0] && stringActive[0]) {
     
          noteOn(0x91 + oldc, activeNote[0], 0);
 
      stringActive[0] = false;
     
    }
    if (!fsrTouched[c] && stringActive[0] && attackcounter[c] < ATTACK) {
       noteOn(0x91 + oldc, activeNote[0], 0);
      stringActive[0] = false;
      
    }

//---STRING TWO----//
       if (!channelMode) {
    c = 1;
    oldc = 1;
    }

    //no fret is touched and the string is marked active
    if (!fretTouched[1] && stringActive[1]) { 
     
          noteOn(0x91 + oldc, activeNote[1], 0);
   
      stringActive[1] = false;
     
    }
if (!fsrTouched[c] && stringActive[1] && attackcounter[c] < ATTACK){
    noteOn(0x91 + oldc, activeNote[1], 0);
      stringActive[1] = false;
     
}
//---STRING THREE----//
       if (!channelMode) {
    c = 2;
    oldc = 2;
    }

    //no fret is touched and the string is marked active
    if (!fretTouched[2] && stringActive[2]) { 
       
   
          noteOn(0x91 + oldc, activeNote[2], 0);
  
      stringActive[2] = false;
     
    }
if (!fsrTouched[c] && stringActive[2] && attackcounter[c] < ATTACK) {
     noteOn(0x91 + oldc, activeNote[2], 0);
      stringActive[2] = false;
      
}
//---STRING FOUR----//
       if (!channelMode) {
    c = 3;
    oldc = 3;
    }

    //no fret is touched and the string is marked active
    if (!fretTouched[3] && stringActive[3]) { 
       
          noteOn(0x91 + oldc, activeNote[3], 0);
  
      stringActive[3] = false;
     
    }
if (!fsrTouched[c] && stringActive[3] && attackcounter[c] < ATTACK){
        noteOn(0x91 + oldc, activeNote[3], 0);
      stringActive[3] = false;
     
}
 }
 oldc = c;
 
 //------------------READ CONTROLS-----------------//
 byte isHit = fsrVals[0] + fsrVals[1] + fsrVals[2] + fsrVals[3]; 
 byte anyActive = stringActive[0] + stringActive[1] + stringActive[2] + stringActive[3];
 //check for control changes
byte channelbutton = !digitalRead(PIN_BUTTON_CHANNEL);
byte breathbutton = !digitalRead(PIN_BUTTON_BREATH);
byte legatobutton = !digitalRead(PIN_BUTTON_LEGATO);
 buttonread = (channelbutton << 2) | (breathbutton << 1) | legatobutton;
 
 if (buttonread){
  if(buttoncount < (BUTTONPRESS + 2)) {
 
  buttoncount = buttoncount + 1;
 }
 }else {
  buttoncount = 0;
 }

 if (buttoncount == BUTTONPRESS) {
  switch (buttonread) {
    case 4:
    channelMode = !channelMode;
    break;
    case 2:
    breathMode = !breathMode;
    break;
    case 1: 
    fullLegatoMode = !fullLegatoMode;
    break;
    case 6:
    bowMode = !bowMode;
    break;
  }
 }

  //sustain Pedal, holds last fretted notes

if (digitalRead(PIN_SUSTAIN) == HIGH) {
  if (!buttonStates[SUSTAIN]) {
  controllerAllChannels(64, 127);
  buttonStates[SUSTAIN] = true;
}

} 
if (digitalRead(PIN_SUSTAIN) == LOW && buttonStates[SUSTAIN]) {
  controllerAllChannels(64, 0);

buttonStates[SUSTAIN] = false;
}

//---- CHANGING THE OCTAVE -------//
  //UP and down buttons to change offset/octave.
  
  //---- UP BUTTON ----
  if (digitalRead(PIN_OCTAVE_UP) == LOW) {
     if (!buttonStates[UP]) {
      if (octave < 1) octave = 1;
        octave = octave - 1;
        byte stringbase = 12*octave + LOWEST_NOTE;
        offsets[0] = stringbase; 
        offsets[1] = stringbase + 5; 
        offsets[2] = stringbase + 10; 
        offsets[3] = stringbase + 15;           
    }
    
    buttonStates[UP] = true;
  }
  //reset state once button is no longer being pressed
  if (digitalRead(PIN_OCTAVE_UP) == HIGH && buttonStates[UP]) buttonStates[UP] = false;
  
  //----DOWN BUTTON----
  if (digitalRead(PIN_OCTAVE_DOWN) == LOW) {
    if (!buttonStates[DOWN]) {
      
      octave = octave + 1;
      if (octave > 7) octave = 7;
      byte stringbase = 12*octave + LOWEST_NOTE;
        offsets[0] = stringbase; 
        offsets[1] = stringbase + 5; 
        offsets[2] = stringbase + 10; 
        offsets[3] = stringbase + 15;           
   
    }
    
    buttonStates[DOWN] = true;
  }
  //reset state once button is no longer being pressed
  if (digitalRead(PIN_OCTAVE_DOWN) == HIGH && buttonStates[DOWN]) buttonStates[DOWN] = false;

 readAccel = analogRead(PIN_ACCEL);
 
  if (!isHit && anyActive) {
    if (controlcounter == CONTROL_RATE) {
  
    controlcounter = 0;
    //read positions from joystick center

    int yPos = map(analogRead(PIN_JOYSTICK_Y), stickZeroY, 1010, 0, 127);
  
    yPos = constrain(yPos, -127, 127);
  
       if (yPos - yPosold) {
  
          controllerChange(1, abs(yPos));  
    } 
    yPosold = yPos;

    //Accelerometer sends pitchbend, naive approach...

   
if (abs(accelZero - readAccel)) {
if (abs(readAccel - readAccelOld) > 3 ) {
  int bend = map(readAccel,accelZero,675,0,8191);
  bend = constrain(bend, -8192,8191);
bend = bend + 8192;
  byte highnibble = bend >> 7;
  byte lownibble = bend & 0x7f;
  if (channelMode) {
  if (runningbyte != 0xE1 + CHANNEL_OFFSET + c) Serial1.write(byte(0xE1 + CHANNEL_OFFSET +c));
  Serial1.write(byte(lownibble));
  Serial1.write(byte(highnibble));
  runningbyte = 0xE1 + CHANNEL_OFFSET + c;
 
} else {
  if (runningbyte != 0xE0) Serial1.write(byte(0xE0));
  Serial1.write(byte(lownibble));
  Serial1.write(byte(highnibble));
  runningbyte = 0xE0;
}
MIDIEvent event = {0x0E, 0xE0, lownibble, highnibble};
  MIDIUSB.write(event);
 readAccelOld = readAccel;
  isMiddle = 0;
}
} else if (!isMiddle) {
  if (channelMode) {
  if (runningbyte != 0xE1 + CHANNEL_OFFSET + c) Serial1.write(byte(0xE1 + CHANNEL_OFFSET + c));
  Serial1.write(byte(0));
  Serial1.write(byte(64));
    runningbyte = 0xE1 + CHANNEL_OFFSET + c;
  } else {
    if (runningbyte != 0xE0) Serial1.write(byte(0xE0));
  Serial1.write(byte(0));
  Serial1.write(byte(64));
    runningbyte = 0xE0;
  }
  MIDIEvent event = {0x0E, 0xE0, 0, 64};
  MIDIUSB.write(event);
  isMiddle = 1;
   }
  } else if (controlcounter < CONTROL_RATE) controlcounter = controlcounter + 1;
  } else accelZero = readAccel;
  }

//----CALIBRATION----//
/* Likely will only have to calibrate once. This is done by activating calibration mode and
* "plucking" each note on each string starting with the upper bound (after the 21st fret) and descending down
* to just after the 1st fret. Starts with the low E string, then the A string and then the D string.
* fret definitions are stored in EEPROM. Once it is calibrated for the voltage put out by the MIDI --> USB board
* you can just have the calibration button do something else because you probably won't need it again.
*/

void calibrate() {
  for (byte i=0; i<NUM_STRINGS; i++) {
    //Flash the LED to indicate calibration
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
  
    int sensorMax = 0;
    int sensorMin = 1023;
    int val;
    
      //loop through the array of fret definitions
      for (byte j=24; j>0; j--) {
      
        int response = false;
        int fsrHit = 0;
        //wait for response
        while (!response) {
        
          //read piezo val
          int fsrVal = analogRead(fsrPins[i]);
          //get the sensor min value (highest fret) on the first round
          if (j==24) {
            int fretVal = analogRead(softPotPins[i]);
            if (fretVal > sensorMax) (sensorMax = fretVal);
       
            //if the fsr is hit, register this as the definition for this fret
            if (fsrVal < (fsrInit[i] - 80) && fsrHit == 0) {
              
              int fretVal = analogRead(softPotPins[i]);
              sensorMin = fretVal;
              val = fretVal;
              response = true;
              fsrHit = 1;
            }
            if (fsrVal > (fsrInit[i] -50) && fsrHit == 1) fsrHit == 0;
          }
        
          else {
            //get the rest of the fret definitions
            //if the piezo is hit, register this as the definition for this fret
            if (fsrVal < (fsrInit[i] - 80) && fsrHit == 0) {
              int fretVal = analogRead(softPotPins[i]);
              fretVal = map(fretVal, sensorMin, sensorMax, 0, 255);
              if (fretVal > 255) fretVal = 255;
              val = fretVal;
              response = true;
              fsrHit = 1;
            } 
            if (fsrVal >  (fsrInit[i] -50) && fsrHit == 1) fsrHit == 0;
          }
        }
      
        //write to memory
        digitalWrite(PIN_LED, LOW);
        EEPROM.write(j + (24*i), val);
        
        delay(100);
        digitalWrite(PIN_LED, HIGH);
      }
    
    //update global definitions
    calibrationMin[i] = EEPROM.read(24 + (24*i));

    for (byte j=1; j<24; j++) {
      fretDefs[i][j] = EEPROM.read(j + (i*24));
    }
  }
  
  digitalWrite(PIN_LED, LOW);
}

//write default programs to eeprom, so unused slots don't output garbage values, restore defaults
//likely will only have to write defaults once
void writeDefaults() {
  for (byte i=100; i<228; i++) {
    EEPROM.write(i, 2);
  }

}


//-------------MIDI OUT FUNCTIONS-----------------//

//note-on (and off) message
inline void noteOn(byte cmd, byte pitch, byte velocity) {
  if (channelMode){ cmd = cmd + CHANNEL_OFFSET;
  }
  if (cmd != runningbyte) Serial1.write(byte(cmd));
  Serial1.write(byte(pitch));
  Serial1.write(byte(velocity));
    //kind of mpe, starting at channel 2, channel 1 is for general modulation only
  MIDIEvent noteOn = {0x09, cmd, pitch, velocity};
  MIDIUSB.write(noteOn);
  runningbyte = cmd;
}



//Sends controller change to the specified controller
inline void controllerChange(byte controller, byte value) {
    if (channelMode) {     
  if (runningbyte != (0xB1 + CHANNEL_OFFSET + c)) Serial1.write(byte(0xB1 + CHANNEL_OFFSET + c));
   Serial1.write(byte(controller));
   Serial1.write(byte(value));
   runningbyte = 0xB1 + CHANNEL_OFFSET + c;
   
  } else {
    
 if (runningbyte != 0xB0) Serial1.write(byte(0xB0));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
  runningbyte = 0xB0;
  }
 
   //kind of mpe, channel 1 is for general modulation only 
 MIDIEvent event = {0x0B, 0xB0, controller, value};
  MIDIUSB.write(event);

}

inline void controllerAllChannels(byte controller, byte value) {
    if (channelMode) {
   Serial1.write(byte(0xB1 + CHANNEL_OFFSET));
   Serial1.write(byte(controller));
   Serial1.write(byte(value));
   MIDIEvent event = {0x0B, 0xB1 + CHANNEL_OFFSET, controller, value};
   MIDIUSB.write(event);
   
   Serial1.write(byte(0xB2 + CHANNEL_OFFSET));
   Serial1.write(byte(controller));
   Serial1.write(byte(value));
   MIDIEvent event1 = {0x0B, 0xB2 + CHANNEL_OFFSET, controller, value};
   MIDIUSB.write(event1);
   
   Serial1.write(byte(0xB3 + CHANNEL_OFFSET));
   Serial1.write(byte(controller));
   Serial1.write(byte(value));
   MIDIEvent event2 = {0x0B, 0xB3 + CHANNEL_OFFSET, controller, value};
   MIDIUSB.write(event2);
   
   Serial1.write(byte(0xB4 + CHANNEL_OFFSET));
   Serial1.write(byte(controller));
   Serial1.write(byte(value));
   MIDIEvent event3 = {0x0B, 0xB4 + CHANNEL_OFFSET, controller, value};
   MIDIUSB.write(event3);

  } else {
  Serial1.write(byte(0xB1));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
  MIDIEvent event = {0x0B, 0xB1, controller, value};
  MIDIUSB.write(event);
  
  Serial1.write(byte(0xB2));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
  MIDIEvent event1 = {0x0B, 0xB2, controller, value};
  MIDIUSB.write(event1);
  
  Serial1.write(byte(0xB3));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
  MIDIEvent event2 = {0x0B, 0xB3, controller, value};
  MIDIUSB.write(event2);
  
  Serial1.write(byte(0xB4));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
  MIDIEvent event3 = {0x0B, 0xB4, controller, value};
  MIDIUSB.write(event3);
  }
  runningbyte = 0;
}

inline void aftertouchChange(byte cmd, byte value) {
 if (runningbyte != cmd) Serial1.write(byte(cmd));
 Serial1.write(byte(value));
   //kind of mpe, starting at channel 2, channel 1 is for general modulation only
 MIDIEvent aftertouch = { 0x0D, cmd, value};
 MIDIUSB.write(aftertouch);

runningbyte = cmd;
}

inline void programChange(byte progNr) { 
  
    Serial1.write(byte(0xC0));
    Serial1.write(byte(progNr));
    Serial1.write(byte(0xC1 + CHANNEL_OFFSET));
    Serial1.write(byte(progNr));
    Serial1.write(byte(0xC2 + CHANNEL_OFFSET));
    Serial1.write(byte(progNr));
    Serial1.write(byte(0xC3 + CHANNEL_OFFSET));
    Serial1.write(byte(progNr));
    Serial1.write(byte(0xC4 + CHANNEL_OFFSET));
    Serial1.write(byte(progNr));
    
    
  MIDIEvent programchange = { 0x0C, 0xC0, progNr};
  MIDIUSB.write(programchange);
    runningbyte = 0;
}
//-------------------OTHER FUNCTIONS---------------//
//serial midi in parser control the ribbonbass modes, mirror program change to midi out.
inline void midiParser(){
  switch (statusByte){
    
    case 0xC0: //program change
    saveSlot = dataByte1;
    loadSettings(dataByte1);
    programChange(dataByte1);
    break;
    case 0xB0: //cc messages 
    switch (dataByte1) {
       case ALLMUTE_CC:
    controllerAllChannels(123,0);
       break; 
      case SAVE_CC:
      saveSettings();
      break; 
      
      case CHANNELMODE_CC:
      if (stringActive[0]) {
              if (!channelMode) oldc = 0;
        noteOn(0x91 + oldc, activeNote[0], 0);    
        }

       if (stringActive[1]) {
              if (!channelMode) oldc = 1;
        noteOn(0x91 + oldc, activeNote[1], 0);    
        }

       if (stringActive[2]) {
              if (!channelMode) oldc = 2;
        noteOn(0x91 + oldc, activeNote[2], 0);    
        }

        if (stringActive[3]) {
              if (!channelMode) oldc = 3;
        noteOn(0x91 + oldc, activeNote[3], 0);    
        }     
      c = 0;
      channelMode = !channelMode;
      break; 
        case BOWMODE_CC:
      bowMode = !bowMode;
        break; 
        case DUMMY_CC:
        bowMode = !bowMode;
        break;
      case BREATH_CC:
      breathMode = !breathMode;
      break;
      case FULLLEGATO_CC:
      if (stringActive[0]) {
              if (!channelMode) oldc = 0;
        noteOn(0x91 + oldc, activeNote[0], 0);    
        }

       if (stringActive[1]) {
              if (!channelMode) oldc = 1;
        noteOn(0x91 + oldc, activeNote[1], 0);    
        }

       if (stringActive[2]) {
              if (!channelMode) oldc = 2;
        noteOn(0x91 + oldc, activeNote[2], 0);    
        }

        if (stringActive[3]) {
              if (!channelMode) oldc = 3;
        noteOn(0x91 + oldc, activeNote[3], 0);    
        }     
      fullLegatoMode = !fullLegatoMode;
      break;
      case OCTAVE_CC:
      octave = dataByte2 & 7;
      byte stringbase = 12*octave + LOWEST_NOTE;
        offsets[0] = stringbase; 
        offsets[1] = stringbase + 5; 
        offsets[2] = stringbase + 10; 
        offsets[3] = stringbase + 15; 
       break;
           
    }
    break;
   
  }
}



//save settings to eeprom on cc SAVE_CC (slot is last received program change)
 void saveSettings() {
 // if (octave < 0) octave = 0;
  byte settings = (channelMode << 7) | (fullLegatoMode << 6) | (breathMode << 5) | (bowMode << 4) | octave;
  EEPROM.write(saveSlot + 100, settings);
}

// load settings from eeprom on program change
void loadSettings(byte prgCh) {

   if (stringActive[0]) {
              if (!channelMode) oldc = 0;
        noteOn(0x91 + oldc, activeNote[0], 0);    
        }

       if (stringActive[1]) {
              if (!channelMode) oldc = 1;
        noteOn(0x91 + oldc, activeNote[1], 0);    
        }

       if (stringActive[2]) {
              if (!channelMode) oldc = 2;
        noteOn(0x91 + oldc, activeNote[2], 0);    
        }

        if (stringActive[3]) {
              if (!channelMode) oldc = 3;
        noteOn(0x91 + oldc, activeNote[3], 0);    
        }     
  c = 0;
  byte settings = EEPROM.read(prgCh + 100);
  channelMode = (settings >> 7) & 1;
  fullLegatoMode = (settings >> 6) & 1;
  breathMode = (settings >> 5) & 1;
  bowMode = (settings >> 4) & 1;
  octave = settings & 15;
  byte stringbase = 12*octave + LOWEST_NOTE;
        offsets[0] = stringbase; 
        offsets[1] = stringbase + 5; 
        offsets[2] = stringbase + 10; 
        offsets[3] = stringbase + 15;           
}

