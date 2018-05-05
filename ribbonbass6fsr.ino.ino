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
#define PIN_FSR_5 A8
#define PIN_FULLLEGATO A9
#define PIN_BUTTON_CHANNEL 3
#define PIN_BUTTON_FSR 11
#define PIN_OCTAVE_UP 5
#define PIN_OCTAVE_DOWN 2
#define PIN_BUTTON_LEGATO 7
//joystick
#define PIN_JOYSTICK_Y A11
//accelerometer
#define PIN_ACCEL A10

#define UP 0
#define DOWN 1
#define FULLLEGATO 2


#define NUM_STRINGS 4
//resistance to change note when finger position not precise...
#define PADDING 3
#define MAXREADING 630
#define MINVELOCITY 60
//where does the channelmode start, default mode starts on channel 2  general modulation happens on channel one (kind of mpe))
#define CHANNEL_OFFSET 4
//how long before a note stays on when you release the fsr in non-bowmode
#define ATTACK 120
#define ATTACK_LEGATO 120
//which cc numbers are used to save settings and change the modes via midi.
#define SAVE_CC 115
#define CHANNELMODE_CC 119
#define FULLLEGATO_CC 118
#define OCTAVE_CC 117
#define FSR_SLAVE_CC 116
#define ALLMUTE_CC 123
#define BOWMODE_CC 114
#define DUMMY_CC 113
//lowest open string to calculate midi note values on the neck
#define LOWEST_NOTE 4

// corrects false note splutter when softpot is released
#define STABLETIME 14
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
 byte fret_Defs[4][25];
 byte map_fsr[] = {
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
byte fsr_Touched[] = {false, false, false, false};
byte active_Note[] = {0,0,0,0};
byte active_Velo[] = {0,0,0,0};
unsigned int fsr_Vals[] = {0, 0, 0, 0};
byte fsr_Read_Old[] = {0, 0, 0, 0};

 byte fsr_Pins[] = {PIN_FSR_1, PIN_FSR_2, PIN_FSR_3, PIN_FSR_4};
unsigned int fsr_Init[] = {0, 0, 0, 0};

unsigned int soft_Pot_Vals[4];
 byte soft_Pot_Pins[] = {PIN_SOFTPOT_1, PIN_SOFTPOT_2, PIN_SOFTPOT_3, PIN_SOFTPOT_4};

byte soft_Pot_Vals_Old[] = {0, 0, 0, 0};

byte fret_Touched[4];
byte maybe_Note[] {0,0,0,0};
byte stable_Count[] = {0,0,0,0};
byte note_Fretted[4];
byte string_Active[] = {false, false, false, false};



//unsigned int calibration_Max[] = {630, 630, 630, 630};
unsigned int calibration_Min[4];


//octave offset
byte octave = 2;

//default offsets for the 4 strings
byte offsets[] = {28, 33, 38, 43};

//states of control buttons
byte button_States[] = {false, false, false};
unsigned int stick_Zero_Y = 0;
// fifth fsr  mode, when off fifth fsr is fixed to fourth fsr sound otherwise dynamic (last hit fsr)
boolean fsr_Add_Slave_Mode = false;
//no need to press the fsr to trigger a note, simply press a finger on a softpot
boolean full_Legato_Mode = false;
// allows for 4 sounds to be triggered by the 4 fsr's.
boolean channel_Mode = false;
//toggle bowmode (hold sound only as long as fsr is touched) and regular mode (hold sound as long as string is pressed)
boolean bow_Mode = false;
// store which fsr has been hit in channelmode
byte c = 0;
byte old_c = 0;

byte y_Pos_old = 0;
byte y_Pos2_old = 0;

byte attack_counter[] = {0,0,0,0};
byte running_byte = 0;
//midi in variables
byte state = 0;
byte save_Slot = 0;
byte status_Byte;
byte data_Byte1;
byte data_Byte2;

 byte button_read = 0;
 byte button_count = 0;
 

 //Accelerometer
int read_Accel;
int read_Accel_Old = 0;
int accel_Zero;
bool is_Middle = 0;
// thin control data a little
byte control_counter = 0;
byte at_counter[] = {0,0,0,0};
unsigned int fsr_Add_Init;
unsigned int fsr_Foot_Init;
byte fsr_Add_Touched = 0;
byte fsr_Foot_Touched = 0;
byte fsr_Add_Old = 0;
byte fsr_Foot_Old = 0;
byte at_counter_add = 0;
byte at_counter_foot = 0;

//--------- Setup ----------------//
void setup() {


  
  digitalWrite(PIN_LED, HIGH);
  //read fret definitions from EEPROM
  for (byte i=NUM_STRINGS; i--;){
 
    fret_Defs[i][0] = 255;
    for (byte j=1; j<24; j++){
      fret_Defs[i][j] = EEPROM.read(j + (24*i));
    }
    fret_Defs[i][24]=0;
    calibration_Min[i] = EEPROM.read(24 + (24*i));
  }

 
  //begin at MIDI spec baud rate
  Serial1.begin(31250);
  
  pinMode(PIN_BUTTON_FSR, INPUT_PULLUP);  
  pinMode(PIN_BUTTON_LEGATO, INPUT_PULLUP);  
  pinMode(PIN_OCTAVE_UP, INPUT_PULLUP);  
  pinMode(PIN_OCTAVE_DOWN, INPUT_PULLUP);  
  pinMode(PIN_BUTTON_CHANNEL, INPUT_PULLUP); 
  
  pinMode(PIN_LED, OUTPUT);
  while(millis() < 800) {
    for (byte i=NUM_STRINGS; i--;){
      fsr_Init[i] = analogRead(fsr_Pins[i]);
  //    unsigned int val = analogRead(soft_Pot_Pins[i]);
    //  if (val > calibration_Max[i]) calibration_Max[i] = val;
    }
    fsr_Add_Init = analogRead(PIN_FSR_5);
    fsr_Foot_Init = analogRead(PIN_FULLLEGATO);
 
    //calibrate joystick
    stick_Zero_Y = analogRead(PIN_JOYSTICK_Y);
    //calibrate accelerometer
    accel_Zero = analogRead(PIN_ACCEL);
    
    } 
    digitalWrite(PIN_LED, LOW);
    //press leftmost (from above) button during startup to calibrate
    if ( digitalRead(PIN_BUTTON_CHANNEL) == LOW ) {
        calibrate();
  }
  //write defaults to all program loccations, overwriting everything
 /* if (digitalRead(PIN_BUTTON_FSR) == LOW && !button_States[BREATH]) {
    button_States[BREATH] = true;
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
         status_Byte = incomingByte;
         state=1;
        }
        if (incomingByte == 0xF0) state=3;
       case 1:
       // get first databyte
       if(incomingByte < 0x80) {
        data_Byte1 = incomingByte;
        //check for two byte messages...
       if ((status_Byte & 0xF0) == 0xC0  | (status_Byte & 0xF0) == 0xD0) {
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
       data_Byte2 = incomingByte;
       }
       midiParser();
         state = 0;  // reset state machine to start
         break;
          //stay here for sysex, wait for it to end...
         case 3:
         if (incomingByte == 0xF7) state=0;
         if(incomingByte > 0x7F && incomingByte < 0xF0) {
              status_Byte = incomingByte;
              state=1;   
         }
         break;
  }
}
//reset fsr_Vals to zero
memset(fsr_Vals,0,sizeof(fsr_Vals));

//------------------------- READ ALL SENSORS (Softpots and FSRs)---------------//
 //read values of all sensors 
 
 unsigned int fsr_Read[4];

  byte fret_touch = fret_Touched[0] + fret_Touched[1] + fret_Touched[2] + fret_Touched[3];
  
unsigned int fsr_Add = analogRead(PIN_FSR_5);

if (!fsr_Foot_Touched) {
if (channel_Mode) {
   
   //  if (fret_touch) {
    if (fsr_Add < (fsr_Add_Init - 30) && !fsr_Add_Touched) {
      if (!fsr_Add_Slave_Mode) c = 3;
      attack_counter[c] = 0;
      fsr_Vals[c] = map(fsr_Add, fsr_Add_Init - 30, 60, MINVELOCITY, 127);
      fsr_Add_Touched = 1;
      fsr_Touched[c] = 1;
    }
    if (fsr_Add < (fsr_Add_Init - 20) && fsr_Add_Touched) {
     

   fsr_Add = map_fsr[(fsr_Add - 10) >> 1];
     if ((fsr_Add - fsr_Add_Old) && attack_counter[c]) {
      at_counter_add = at_counter_add + 1;
      if (at_counter_add == AFTERTOUCH_RATE) {
 if (fret_touch)  aftertouchChange(0xD1 + c + CHANNEL_OFFSET, fsr_Add);
   
      at_counter_add = 0;
   
      }
     }
      
       if (attack_counter[c] < ATTACK) attack_counter[c] = attack_counter[c] + 1;
      
       fsr_Add_Old = fsr_Add; 

    }
    if (fsr_Add > (fsr_Add_Init - 15) && fsr_Add_Touched) {
  if (fret_touch)   aftertouchChange(0xD1 + c + CHANNEL_OFFSET, 0);
      fsr_Add_Touched = 0;   
      fsr_Touched[c] = 0;
    } 
   
//} else fsr_Add_Touched = 0;
} else {
    if (fsr_Add < (fsr_Add_Init - 30) && !fsr_Add_Touched) {
      memset(attack_counter,0,sizeof(attack_counter));
      fsr_Vals[0] = map(fsr_Add, fsr_Add_Init - 30, 60, MINVELOCITY, 127);
      fsr_Vals[1] = fsr_Vals[0];
      fsr_Vals[2] = fsr_Vals[0];
      fsr_Vals[3] = fsr_Vals[0];
      fsr_Add_Touched = 1;
      memset(fsr_Touched,1,sizeof(fsr_Touched));
    }
    if (fsr_Add < (fsr_Add_Init - 20) && fsr_Add_Touched) {
     

   fsr_Add = map_fsr[(fsr_Add - 10) >> 1];
     if ((fsr_Add - fsr_Add_Old) && attack_counter[0]) {
      at_counter_add = at_counter_add + 1;
      if (at_counter_add == AFTERTOUCH_RATE) {
  if (fret_Touched[0]) aftertouchChange(0xD1 , fsr_Add);
   if (fret_Touched[1]) aftertouchChange(0xD2 , fsr_Add);
   if (fret_Touched[2]) aftertouchChange(0xD3 , fsr_Add);
  if (fret_Touched[3]) aftertouchChange(0xD4 , fsr_Add);
    at_counter_add = 0;
      }
     }
      
       if (attack_counter[0] < ATTACK) {
        attack_counter[0] = attack_counter[0] + 1;
         attack_counter[1] = attack_counter[0];
          attack_counter[2] = attack_counter[0];
           attack_counter[3] = attack_counter[0];
        
       }
      
       fsr_Add_Old = fsr_Add; 

    }
    if (fsr_Add > (fsr_Add_Init - 15) && fsr_Add_Touched) {
  if (fret_Touched[0])  aftertouchChange(0xD1 , 0);
 if (fret_Touched[1])   aftertouchChange(0xD2 , 0);
  if (fret_Touched[2]) aftertouchChange(0xD3 , 0);
  if (fret_Touched[3])  aftertouchChange(0xD4 , 0);
      fsr_Add_Touched = 0;   
      memset(fsr_Touched,0,sizeof(fsr_Touched));
    }    
}
}
unsigned int fsr_Foot = analogRead(PIN_FULLLEGATO);

if (!fsr_Add_Touched) {
if (channel_Mode) {
   
   //  if (fret_touch) {
    if (fsr_Foot < (fsr_Foot_Init - 450) && !fsr_Foot_Touched) {
      if (!fsr_Add_Slave_Mode) c = 3;
      attack_counter[c] = 120;
      fsr_Vals[c] = map(fsr_Foot, fsr_Foot_Init - 450, 60, MINVELOCITY, 127);
      fsr_Foot_Touched = 1;
      fsr_Touched[c] = 1;
    }
    if (fsr_Foot < (fsr_Foot_Init - 440) && fsr_Foot_Touched) {
     
    if (fsr_Foot > 501) fsr_Foot = 501;
   fsr_Foot = map_fsr[(fsr_Foot)];
     if ((fsr_Foot - fsr_Foot_Old) && attack_counter[c]) {
      at_counter_add = at_counter_add + 1;
      if (at_counter_add == AFTERTOUCH_RATE) {
  if (fret_touch)  aftertouchChange(0xD1 + c + CHANNEL_OFFSET, fsr_Foot);
   
      at_counter_add = 0;
     
      }
     }
      
      
       fsr_Foot_Old = fsr_Foot; 

    }
    if (fsr_Foot > (fsr_Foot_Init - 430) && fsr_Foot_Touched) {
 if (fret_touch) aftertouchChange(0xD1 + c + CHANNEL_OFFSET, 0);
      fsr_Foot_Touched = 0;   
      fsr_Touched[c] = 0;
    } 
} else {
  
    if (fsr_Foot < (fsr_Foot_Init - 450) && !fsr_Foot_Touched) {
      memset(attack_counter,120,sizeof(attack_counter));
      fsr_Vals[0] = map(fsr_Foot, fsr_Foot_Init - 450, 60, MINVELOCITY, 127);
      fsr_Vals[1] = fsr_Vals[0];
      fsr_Vals[2] = fsr_Vals[0];
      fsr_Vals[3] = fsr_Vals[0];
      fsr_Foot_Touched = 1;
      memset(fsr_Touched,1,sizeof(fsr_Touched));
    }
    if (fsr_Foot < (fsr_Foot_Init - 440) && fsr_Foot_Touched) {
     
    if (fsr_Foot > 501) fsr_Foot = 501;
   fsr_Foot = map_fsr[fsr_Foot];
     if ((fsr_Foot - fsr_Foot_Old) && attack_counter[0]) {
      at_counter_add = at_counter_add + 1;
      if (at_counter_add == AFTERTOUCH_RATE) {
       
  if (fret_Touched[0]) aftertouchChange(0xD1 , fsr_Foot);
 if (fret_Touched[1])   aftertouchChange(0xD2 , fsr_Foot);
  if (fret_Touched[2]) aftertouchChange(0xD3 , fsr_Foot);
  if (fret_Touched[3]) aftertouchChange(0xD4 , fsr_Foot);
    at_counter_add = 0;
        
      }
     }
      
      
       fsr_Foot_Old = fsr_Foot; 

    }
    if (fsr_Foot > (fsr_Foot_Init - 430) && fsr_Foot_Touched) {
    
   if (fret_Touched[0]) aftertouchChange(0xD1 , 0);
  if (fret_Touched[1])  aftertouchChange(0xD2 , 0);
   if (fret_Touched[2]) aftertouchChange(0xD3 , 0);
   if (fret_Touched[3]) aftertouchChange(0xD4 , 0);
   
      fsr_Foot_Touched = 0;   
      memset(fsr_Touched,0,sizeof(fsr_Touched));
    } 
 }
}
    //-STRING ONE-//
    //read fsr vals
 if(!fsr_Add_Touched && !fsr_Foot_Touched) {
if (!channel_Mode) fret_touch = fret_Touched[0];
    fsr_Read[0] = analogRead(fsr_Pins[0]);
     if (fret_touch) {
    if (fsr_Read[0] < (fsr_Init[0] - 30) && !fsr_Touched[0]) {
      attack_counter[0] = 0;
      fsr_Vals[0] = map((fsr_Read[0]), fsr_Init[0] - 30, 60, MINVELOCITY, 127);
      fsr_Touched[0] = 1;
      c= 0;
    }
    if (fsr_Read[0] < (fsr_Init[0] - 20) && fsr_Touched[0]) {
     

   fsr_Read[0] = map_fsr[(fsr_Read[0] - 10) >> 1];
     if ((fsr_Read[0] - fsr_Read_Old[0]) && attack_counter[0]) {
      at_counter[0] = at_counter[0] + 1;
      if (at_counter[0] == AFTERTOUCH_RATE) {
        
      if (channel_Mode)  aftertouchChange(0xD1 + CHANNEL_OFFSET, fsr_Read[0]);
      else aftertouchChange(0xD1 ,fsr_Read[0]);
      at_counter[0] = 0;
        
      }
     }
      
       if (attack_counter[0] < ATTACK) attack_counter[0] = attack_counter[0] + 1;
      
       fsr_Read_Old[0] = fsr_Read[0]; 

    }
    if (fsr_Read[0] > (fsr_Init[0] - 15) && fsr_Touched[0]) {
     
      if (channel_Mode)  aftertouchChange(0xD1 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD1 ,0);
    
      fsr_Touched[0] = 0;   
    }    
} else {
  fsr_Touched[0] = 0;
  if (fsr_Read[0] < (fsr_Init[0] - 30)) c = 0;
}
 }
    //read the value of all the softPots
    soft_Pot_Vals[0] = analogRead(soft_Pot_Pins[0]);
    soft_Pot_Vals[0] = map(soft_Pot_Vals[0], calibration_Min[0], MAXREADING, 0, 255);
   soft_Pot_Vals[0] = constrain(soft_Pot_Vals[0], 0, 255);

//--STRING TWO--//
 if(!fsr_Add_Touched && !fsr_Foot_Touched) {
  if (!channel_Mode) fret_touch = fret_Touched[1];
   fsr_Read[1] = analogRead(fsr_Pins[1]);
     if (fret_touch) {
    if (fsr_Read[1] < (fsr_Init[1] - 30) && !fsr_Touched[1]) {
      attack_counter[1] = 0;
      fsr_Vals[1] = map((fsr_Read[1]), fsr_Init[1] - 30, 60, MINVELOCITY, 127);
      fsr_Touched[1] = 1;
      c= 1;
    }
    if (fsr_Read[1] < (fsr_Init[1] - 20) && fsr_Touched[1]) {
     
   fsr_Read[1] = map_fsr[(fsr_Read[1] - 10) >> 1];
       if ((fsr_Read[1] - fsr_Read_Old[1]) && attack_counter[1]) {
         at_counter[1] = at_counter[1] + 1;
      if (at_counter[1] == AFTERTOUCH_RATE) {
      
      if (channel_Mode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, fsr_Read[1]);
      else aftertouchChange(0xD2 ,fsr_Read[1]);
      at_counter[1] = 0;
        
      }
       }
      
       if (attack_counter[1] < ATTACK) attack_counter[1] = attack_counter[1] + 1;
        
       fsr_Read_Old[1] = fsr_Read[1]; 
    }
    if (fsr_Read[1] > (fsr_Init[1] - 15) && fsr_Touched[1]) {
    
        if (channel_Mode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD2 ,0);
     
      fsr_Touched[1] = 0;
    }    
} else {
  fsr_Touched[1] = 0;
  if (fsr_Read[1] < (fsr_Init[1] - 30)) c = 1;
}
 }
    //read the value of all the softPots
    soft_Pot_Vals[1] = analogRead(soft_Pot_Pins[1]);
    soft_Pot_Vals[1] = map(soft_Pot_Vals[1], calibration_Min[1], MAXREADING, 0, 255);
   soft_Pot_Vals[1] = constrain(soft_Pot_Vals[1], 0, 255);

//-STRING THREE--//
 if(!fsr_Add_Touched && !fsr_Foot_Touched) {
 if (!channel_Mode) fret_touch = fret_Touched[2];
 fsr_Read[2] = analogRead(fsr_Pins[2]);
     if (fret_touch) {
    if (fsr_Read[2] < (fsr_Init[2] - 30) && !fsr_Touched[2]) {
      attack_counter[2] = 0;
     
      fsr_Vals[2] = map((fsr_Read[2]), fsr_Init[2] - 30, 60, MINVELOCITY, 127);
      fsr_Touched[2] = 1;
      c= 2;
    }
    if (fsr_Read[2] < (fsr_Init[2] - 20) && fsr_Touched[2]) {
     
   fsr_Read[2] = map_fsr[(fsr_Read[2] - 10) >> 1];
        if ((fsr_Read[2] - fsr_Read_Old[2]) && attack_counter[2]) {
           at_counter[2] = at_counter[2] + 1;
      if (at_counter[2] == AFTERTOUCH_RATE) {
     
      if (channel_Mode)  aftertouchChange(0xD3 + CHANNEL_OFFSET, fsr_Read[2]);
      else aftertouchChange(0xD3 ,fsr_Read[2]);
      at_counter[2] = 0;
      
      }
        }
      
       if (attack_counter[2] < ATTACK) attack_counter[2] = attack_counter[2] + 1;
        
       fsr_Read_Old[2] = fsr_Read[2]; 
    }
    if (fsr_Read[2] > (fsr_Init[2] - 15) && fsr_Touched[2]) {
   
        if (channel_Mode)  aftertouchChange(0xD2 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD2 ,0);
   
      fsr_Touched[2] = 0;
    }  
} else {
  fsr_Touched[2] = 0;
  if (fsr_Read[2] < (fsr_Init[2] - 30)) c = 2;
}
 }
    //read the value of all the softPots
    soft_Pot_Vals[2] = analogRead(soft_Pot_Pins[2]);
    soft_Pot_Vals[2] = map(soft_Pot_Vals[2], calibration_Min[2], MAXREADING, 0, 255);
   soft_Pot_Vals[2] = constrain(soft_Pot_Vals[2], 0, 255);

//-- STRING FOUR--//
if(!fsr_Add_Touched && !fsr_Foot_Touched) {
  if (!channel_Mode) fret_touch = fret_Touched[3];
 fsr_Read[3] = analogRead(fsr_Pins[3]);
    if (fret_touch) {
    if (fsr_Read[3] < (fsr_Init[3] - 30) && !fsr_Touched[3]) {
      attack_counter[3] = 0;
      
      fsr_Vals[3] = map((fsr_Read[3]), fsr_Init[3] - 30, 60, MINVELOCITY, 127);
      fsr_Touched[3] = 1;
      c=3;
    }
    if (fsr_Read[3] < (fsr_Init[3] - 20) && fsr_Touched[3]) {
     
   fsr_Read[3] = map_fsr[(fsr_Read[3] - 10) >> 1];
       if ((fsr_Read[3] - fsr_Read_Old[3]) && attack_counter[3]) {
         at_counter[3] = at_counter[3] + 1;
      if (at_counter[3] == AFTERTOUCH_RATE) {
       
      if (channel_Mode)  aftertouchChange(0xD4 + CHANNEL_OFFSET, fsr_Read[3]);
      else aftertouchChange(0xD4 ,fsr_Read[3]);
      at_counter[3] = 0;
        
      }
       }
       
       if (attack_counter[3] < ATTACK) attack_counter[3] = attack_counter[3] + 1;
      
      fsr_Read_Old[3] = fsr_Read[3];
    }
    if (fsr_Read[3] > (fsr_Init[3] - 15) && fsr_Touched[3]) {
   
        if (channel_Mode)  aftertouchChange(0xD3 + CHANNEL_OFFSET, 0);
      else aftertouchChange(0xD3 ,0);
     
      fsr_Touched[3] = 0;
    } 
    }  else {
  fsr_Touched[3] = 0;
  if (fsr_Read[3] < (fsr_Init[3] - 30)) c = 3;
}
}
    //read the value of all the softPots
    soft_Pot_Vals[3] = analogRead(soft_Pot_Pins[3]);
    soft_Pot_Vals[3] = map(soft_Pot_Vals[3], calibration_Min[3], MAXREADING, 0, 255);
   soft_Pot_Vals[3] = constrain(soft_Pot_Vals[3], 0, 255);

//}
  
 //------------------DETERMINE FRETS----------------------//
 
    //---------Get Fret Numbers------//

    //--STRING ONE--//
 
   byte soft_Pot_Val = soft_Pot_Vals[0];
   
    //check for open strings
    if (soft_Pot_Val == 255) {
      soft_Pot_Vals_Old[0] = soft_Pot_Val;
      fret_Touched[0]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (soft_Pot_Val <= fret_Defs[0][k] && 
          soft_Pot_Val > fret_Defs[0][j] ) {
            
if (((uint8_t)(abs( (int8_t) (soft_Pot_Val-soft_Pot_Vals_Old[0]))) > PADDING)) {
  soft_Pot_Vals_Old[0] = soft_Pot_Val;
            fret_Touched[0] = j;
    
            break;
  
  } 
    }
    }
    
    if (soft_Pot_Val <= fret_Defs[0][23]) {
      soft_Pot_Vals_Old[0] = soft_Pot_Val;
      fret_Touched[0]=24;
    
    }

//--STRING TWO--//

   soft_Pot_Val = soft_Pot_Vals[1];
    
    //check for open strings
    if (soft_Pot_Val == 255) {
      soft_Pot_Vals_Old[1] = soft_Pot_Val;
      fret_Touched[1]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (soft_Pot_Val <= fret_Defs[1][k] && 
          soft_Pot_Val > fret_Defs[1][j] ) {
if (((uint8_t)(abs( (int8_t) (soft_Pot_Val-soft_Pot_Vals_Old[1]))) > PADDING)) {
  
  soft_Pot_Vals_Old[1] = soft_Pot_Val;
            fret_Touched[1] = j;

            break;
} 
  }
    }
    
    if (soft_Pot_Val <= fret_Defs[1][23]) {
      soft_Pot_Vals_Old[1] = soft_Pot_Val;
      fret_Touched[1]=24;
     
    }

    //--STRING THREE--//

       soft_Pot_Val = soft_Pot_Vals[2];
   
    //check for open strings
    if (soft_Pot_Val == 255) {
      soft_Pot_Vals_Old[2] = soft_Pot_Val;
      fret_Touched[2]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (soft_Pot_Val <= fret_Defs[2][k] && 
          soft_Pot_Val > fret_Defs[2][j]) {
            
         
if(((uint8_t)(abs( (int8_t) (soft_Pot_Val-soft_Pot_Vals_Old[2]))) > PADDING)) {
  
  soft_Pot_Vals_Old[2] = soft_Pot_Val;
            fret_Touched[2] = j;
 
            break;
} 

  }
    }
    
    if (soft_Pot_Val <= fret_Defs[2][23]) {
      soft_Pot_Vals_Old[2] = soft_Pot_Val;
      fret_Touched[2]=24;
  
  
      
    }

    //---STRING FOUR--//

       soft_Pot_Val = soft_Pot_Vals[3];
   
    //check for open strings
    if (soft_Pot_Val == 255) {
      soft_Pot_Vals_Old[3] = soft_Pot_Val;
      fret_Touched[3]=0;
    }
    
    //loop through the array of fret definitions
    for (byte j=1; j<24; j++) {
      
      byte k = j-1;
     if (soft_Pot_Val <= fret_Defs[3][k] && 
          soft_Pot_Val > fret_Defs[3][j] ) {
if (((uint8_t)(abs( (int8_t) (soft_Pot_Val-soft_Pot_Vals_Old[3]))) > PADDING)) {
  
  soft_Pot_Vals_Old[3] = soft_Pot_Val;
            fret_Touched[3] = j;
           
            break;
} 

  }
    }
    
    if (soft_Pot_Val <= fret_Defs[3][23]) {
      soft_Pot_Vals_Old[3] = soft_Pot_Val;
      fret_Touched[3]=24;
     
      
    }
  
 
   //test for legato action
   byte enable_legato;
   if (channel_Mode) {
    if(attack_counter[c] < ATTACK_LEGATO && fsr_Add_Touched) enable_legato = 0;
    else enable_legato = 1;
   }
 //----STRING ONE--//
 
      if (!channel_Mode) {
       if(attack_counter[0] < ATTACK_LEGATO && fsr_Add_Touched) enable_legato = 0;
    else enable_legato = 1;
    c = 0;
    old_c = 0;
    }
  
    if (string_Active[0] ) {
      if (enable_legato ) {
       if (fret_Touched[0]) note_Fretted[0] = fret_Touched[0] + offsets[0];
  
   if (note_Fretted[0] > active_Note[0]) {
   
      //turn on new note
   noteOn(0x91 + c, note_Fretted[0], active_Velo[0]);
        noteOn(0x91 + old_c, active_Note[0], 0);
     
      
      //register new note as the active one
      active_Note[0] = note_Fretted[0];
     
     
   } 
  else if (note_Fretted[0] < active_Note[0]) {
    if (note_Fretted[0] == maybe_Note[0]) stable_Count[0] += 1;
    if (stable_Count[0] == STABLETIME) {
   noteOn(0x91 + c, note_Fretted[0], active_Velo[0]);
        noteOn(0x91 + old_c, active_Note[0], 0);
     
      
      //register new note as the active one
      active_Note[0] = note_Fretted[0];
      stable_Count[0] = 0;
      maybe_Note[0] = 0;
    }
    maybe_Note[0] = note_Fretted[0];
   } 
    }
    }
    
else if (fret_Touched[0] && fsr_Touched[c] && !fsr_Vals[c]) {
   
          note_Fretted[0] = fret_Touched[0] + offsets[0];
   

      

    noteOn(0x91 +c, note_Fretted[0], active_Velo[0]);
   
        //register new note as the active one
        active_Note[0] = note_Fretted[0];
      
        string_Active[0] = true;

    }
 
  //----STRING TWO--//
 
      if (!channel_Mode) {
         if(attack_counter[1] < ATTACK_LEGATO && fsr_Add_Touched) enable_legato = 0;
    else enable_legato = 1;
    c = 1;
    old_c = 1;
    }
   
    if (string_Active[1] ) {
       if (enable_legato ) {
     if (fret_Touched[1])   note_Fretted[1] = fret_Touched[1] + offsets[1];
 
   if (note_Fretted[1] > active_Note[1]) {
    
      //turn on new note

     noteOn(0x91 + c, note_Fretted[1], active_Velo[1]);
        noteOn(0x91 + old_c, active_Note[1], 0);
    
      
      //register new note as the active one
      active_Note[1] = note_Fretted[1];
    
      
   } 
   else if (note_Fretted[1] < active_Note[1]) {
    if (note_Fretted[1] == maybe_Note[1]) stable_Count[1] += 1;
    if (stable_Count[1] == STABLETIME) {
   noteOn(0x91 + c, note_Fretted[1], active_Velo[1]);
        noteOn(0x91 + old_c, active_Note[1], 0);
     
      
      //register new note as the active one
      active_Note[1] = note_Fretted[1];
      stable_Count[1] = 0;
      maybe_Note[1] = 0;
    }
    maybe_Note[1] = note_Fretted[1];
   }
    }
    }
else if (fret_Touched[1] && fsr_Touched[c] && !fsr_Vals[c]) {
   
          note_Fretted[1] = fret_Touched[1] + offsets[1];
 
    noteOn(0x91 +c, note_Fretted[1], active_Velo[1]);
   
        //register new note as the active one
        active_Note[1] = note_Fretted[1];
       
        string_Active[1] = true;

    }
  
    //----STRING THREE--//
 
      if (!channel_Mode) {
         if(attack_counter[2] < ATTACK_LEGATO && fsr_Add_Touched) enable_legato = 0;
    else enable_legato = 1;
    c = 2;
    old_c = 2;
    }
  
    if (string_Active[2] ) {
       if (enable_legato) {
      if (fret_Touched[2])    note_Fretted[2] = fret_Touched[2] + offsets[2];

   if (note_Fretted[2] > active_Note[2]) {
    
      //turn on new note

    noteOn(0x91 + c, note_Fretted[2], active_Velo[2]);
        noteOn(0x91 + old_c, active_Note[2], 0);
   
      
      //register new note as the active one
      active_Note[2] = note_Fretted[2];
   
     
   } 
   
  else if (note_Fretted[2] < active_Note[2]) {
    if (note_Fretted[2] == maybe_Note[2]) stable_Count[2] += 1;
    if (stable_Count[2] == STABLETIME) {
      noteOn(0x91 + c, note_Fretted[2], active_Velo[2]);
        noteOn(0x91 + old_c, active_Note[2], 0);
     
      
      //register new note as the active one
      active_Note[2] = note_Fretted[2];
      stable_Count[2] = 0;
      maybe_Note[2] = 0;
    }
    maybe_Note[2] = note_Fretted[2];
   }
    }
    }
 else if (fret_Touched[2] && fsr_Touched[c] && !fsr_Vals[c] ) {
   
          note_Fretted[2] = fret_Touched[2] + offsets[2];
   

      noteOn(0x91 +c, note_Fretted[2], active_Velo[2]);
      
        //register new note as the active one
        active_Note[2] = note_Fretted[2];
      
        string_Active[2] = true;

    }
      
    //----STRING FOUR--//
 
      if (!channel_Mode) {
         if(attack_counter[3] < ATTACK_LEGATO && fsr_Add_Touched) enable_legato = 0;
    else enable_legato = 1;
    c = 3;
    old_c = 3;
      }
   
    if (string_Active[3] ) {
       if (enable_legato) {
        if (fret_Touched[3])  note_Fretted[3] = fret_Touched[3] + offsets[3];

   if (note_Fretted[3] > active_Note[3]) {
    
      //turn on new note

    noteOn(0x91 + c, note_Fretted[3], active_Velo[3]);
        noteOn(0x91 + old_c, active_Note[3], 0);
    
      
      //register new note as the active one
      active_Note[3] = note_Fretted[3];
    
     
   } 
   else if (note_Fretted[3] < active_Note[3]) {
    if (note_Fretted[3] == maybe_Note[3]) stable_Count[3] += 1;
    if (stable_Count[3] == STABLETIME) {
       noteOn(0x91 + c, note_Fretted[3], active_Velo[3]);
        noteOn(0x91 + old_c, active_Note[3], 0);
     
      
      //register new note as the active one
      active_Note[3] = note_Fretted[3];
      stable_Count[3] = 0;
      maybe_Note[3] = 0;
    }
    maybe_Note[3] = note_Fretted[3];
   }
    }
    }
 else if (fret_Touched[3] && fsr_Touched[c] && !fsr_Vals[c]) {
   
          note_Fretted[3] = fret_Touched[3] + offsets[3];
   
  
   noteOn(0x91 +c, note_Fretted[3], active_Velo[3]);
  
        //register new note as the active one
        active_Note[3] = note_Fretted[3];
       
        string_Active[3] = true;
    }
     
  //------------PICK NOTES--------------------//
  
   //use this info to determine which notes to pluck

  
  //---STRING ONE----//

    if (!channel_Mode) {
    c = 0;
    old_c = 0;
    }
    //if the fsr was hit, play the fretted note
    if ((fsr_Vals[c] && fret_Touched[0]) ){
          note_Fretted[0] = fret_Touched[0] + offsets[0];
   
      
      if (string_Active[0]){
 if (!fsr_Foot_Touched) noteOn(0x91 + old_c , active_Note[0], 0);
      
        }
      
      if (!string_Active[0]) {
        
        //mark string as active
        string_Active[0] = true;
      }
        //register with active notes
        
        active_Note[0] = note_Fretted[0];
        active_Velo[0] = fsr_Vals[c];
    
       
      
        //turn on fretted note
  if (!fsr_Foot_Touched)  noteOn(0x91 + c, active_Note[0], active_Velo[0]);
      
      }

      //---STRING TWO----//
    if (!channel_Mode) {
    c = 1;
    old_c = 1;
    }
    //if the fsr was hit, play the fretted note
    if ((fsr_Vals[c] && fret_Touched[1]) ){
          note_Fretted[1] = fret_Touched[1] + offsets[1];
   
      
      if (string_Active[1]){
      if (!fsr_Foot_Touched)  noteOn(0x91 + old_c , active_Note[1], 0);
      
        }
      
      if (!string_Active[1]) {
        
        //mark string as active
        string_Active[1] = true;
      }
        //register with active notes
        
        active_Note[1] = note_Fretted[1];
        active_Velo[1] = fsr_Vals[c];
    
     
        
        //turn on fretted note
     if (!fsr_Foot_Touched)   noteOn(0x91 + c, active_Note[1], active_Velo[1]);
      
      }

      //---STRING THREE----//
    if (!channel_Mode) {
    c = 2;
    old_c = 2;
    }
    //if the fsr was hit, play the fretted note
    if ((fsr_Vals[c] && fret_Touched[2]) ){
          note_Fretted[2] = fret_Touched[2] + offsets[2];
   
      
      if (string_Active[2]){
       if (!fsr_Foot_Touched) noteOn(0x91 + old_c , active_Note[2], 0);
       
        }
      
      if (!string_Active[2]) {
        
        //mark string as active
        string_Active[2] = true;
      }
        //register with active notes
        
        active_Note[2] = note_Fretted[2];
        active_Velo[2] = fsr_Vals[c];
    
      
        
        //turn on fretted note
       if (!fsr_Foot_Touched)  noteOn(0x91 + c, active_Note[2], active_Velo[2]);
      
      }

      //---STRING FOUR----//
    if (!channel_Mode) {
    c = 3;
    old_c = 3;
    }
    //if the fsr was hit, play the fretted note
    if ((fsr_Vals[c] && fret_Touched[3]) ){
          note_Fretted[3] = fret_Touched[3] + offsets[3];
   
      
      if (string_Active[3]){
      if (!fsr_Foot_Touched)  noteOn(0x91 + old_c , active_Note[3], 0);
      
        }
      
      if (!string_Active[3]) {
        
        //mark string as active
        string_Active[3] = true;
      }
        //register with active notes
        
        active_Note[3] = note_Fretted[3];
        active_Velo[3] = fsr_Vals[c];
    
       
        
        //turn on fretted note
     if (!fsr_Foot_Touched)    noteOn(0x91 + c, active_Note[3], active_Velo[3]);
      
      }
    
  
    //------------------CLEAN UP------------------------//
    
  //send not off messages and reset necessary things

//---STRING ONE----//
       if (!channel_Mode) {
    c = 0;
    old_c = 0;
    }

    //no fret is touched and the string is marked active
    if (!fret_Touched[0] && string_Active[0]) {
     
          noteOn(0x91 + old_c, active_Note[0], 0);
 
      string_Active[0] = false;
     
    }
    if (!fsr_Touched[c] && string_Active[0] && (attack_counter[c] < ATTACK || bow_Mode)) {
       noteOn(0x91 + old_c, active_Note[0], 0);
      string_Active[0] = false;
      
    }

//---STRING TWO----//
       if (!channel_Mode) {
    c = 1;
    old_c = 1;
    }

    //no fret is touched and the string is marked active
    if (!fret_Touched[1] && string_Active[1]) { 
     
          noteOn(0x91 + old_c, active_Note[1], 0);
   
      string_Active[1] = false;
     
    }
if (!fsr_Touched[c] && string_Active[1] && (attack_counter[c] < ATTACK || bow_Mode)){
    noteOn(0x91 + old_c, active_Note[1], 0);
      string_Active[1] = false;
     
}
//---STRING THREE----//
       if (!channel_Mode) {
    c = 2;
    old_c = 2;
    }

    //no fret is touched and the string is marked active
    if (!fret_Touched[2] && string_Active[2]) { 
       
   
          noteOn(0x91 + old_c, active_Note[2], 0);
  
      string_Active[2] = false;
     
    }
if (!fsr_Touched[c] && string_Active[2] && (attack_counter[c] < ATTACK || bow_Mode)) {
     noteOn(0x91 + old_c, active_Note[2], 0);
      string_Active[2] = false;
      
}
//---STRING FOUR----//
       if (!channel_Mode) {
    c = 3;
    old_c = 3;
    }

    //no fret is touched and the string is marked active
    if (!fret_Touched[3] && string_Active[3]) { 
       
          noteOn(0x91 + old_c, active_Note[3], 0);
  
      string_Active[3] = false;
     
    }
if (!fsr_Touched[c] && string_Active[3] && (attack_counter[c] < ATTACK || bow_Mode)) {
        noteOn(0x91 + old_c, active_Note[3], 0);
      string_Active[3] = false;
     
}
// }
 old_c = c;
 
 //------------------READ CONTROLS-----------------//
 byte is_Hit = fsr_Vals[0] + fsr_Vals[1] + fsr_Vals[2] + fsr_Vals[3]; 
 byte any_Active = string_Active[0] + string_Active[1] + string_Active[2] + string_Active[3];
 //check for control changes
byte channelbutton = !digitalRead(PIN_BUTTON_CHANNEL);
byte fsrbutton = !digitalRead(PIN_BUTTON_FSR);
byte legatobutton = !digitalRead(PIN_BUTTON_LEGATO);
 button_read = (channelbutton << 2) | (fsrbutton << 1) | legatobutton;
 
 if (button_read){
  if(button_count < (BUTTONPRESS + 2)) {
 
  button_count = button_count + 1;
 }
 }else {
  button_count = 0;
 }

 if (button_count == BUTTONPRESS) {
  switch (button_read) {
    case 1: 
    bow_Mode = !bow_Mode;
    break;
    case 2:
    fsr_Add_Slave_Mode = !fsr_Add_Slave_Mode;
    break;
 //   case 3:
 //   running_byte = 0;
  //  break;
    case 4:
    channel_Mode = !channel_Mode;
    break;
  //  case 6:
   // bow_Mode = !bow_Mode;
   // break;
 
  }
 }

  
//---- CHANGING THE OCTAVE -------//
  //UP and down buttons to change offset/octave.
  
  //---- UP BUTTON ----
  if (digitalRead(PIN_OCTAVE_UP) == LOW) {
     if (!button_States[UP]) {
      if (octave < 1) octave = 1;
        octave = octave - 1;
        byte string_base = 12*octave + LOWEST_NOTE;
        offsets[0] = string_base; 
        offsets[1] = string_base + 5; 
        offsets[2] = string_base + 10; 
        offsets[3] = string_base + 15;           
    }
    
    button_States[UP] = true;
  }
  //reset state once button is no longer being pressed
  if (digitalRead(PIN_OCTAVE_UP) == HIGH && button_States[UP]) button_States[UP] = false;
  
  //----DOWN BUTTON----
  if (digitalRead(PIN_OCTAVE_DOWN) == LOW) {
    if (!button_States[DOWN]) {
      
      octave = octave + 1;
      if (octave > 7) octave = 7;
      byte string_base = 12*octave + LOWEST_NOTE;
        offsets[0] = string_base; 
        offsets[1] = string_base + 5; 
        offsets[2] = string_base + 10; 
        offsets[3] = string_base + 15;           
   
    }
    
    button_States[DOWN] = true;
  }
  //reset state once button is no longer being pressed
  if (digitalRead(PIN_OCTAVE_DOWN) == HIGH && button_States[DOWN]) button_States[DOWN] = false;

 read_Accel = analogRead(PIN_ACCEL);
 
  if (!is_Hit && any_Active) {
    if (control_counter == CONTROL_RATE) {
  
    control_counter = 0;
    //read positions from joystick center
    int stick = analogRead(PIN_JOYSTICK_Y);
    byte yPos;
    byte yPos2;
    if (stick > (stick_Zero_Y + 5)) {
      yPos2 = 0;
      yPos = map(stick, stick_Zero_Y + 5, 1010, 0, 127);
    if (yPos >= 127) yPos = 127;
    }
    else if (stick < (stick_Zero_Y - 5)) { 
      yPos = 0;
      yPos2 = map(stick,stick_Zero_Y - 5,5, 0, 127);
    if (yPos2 >= 127) yPos2 = 127;
    } else {
      yPos = 0;
      yPos2 = 0; 
    }
  //  int yPos = map(analogRead(PIN_JOYSTICK_Y), stick_Zero_Y, 1010, 0, 127);
  //  int yPos2 = map(analogRead(Pin

       if (yPos - y_Pos_old)    controllerChange(1, yPos);  
    y_Pos_old = yPos;
        if (yPos2 - y_Pos2_old) controllerChange(2, yPos2);
    y_Pos2_old = yPos2;

    //Accelerometer sends pitchbend, naive approach...

   
if (abs(accel_Zero - read_Accel)) {
if (abs(read_Accel - read_Accel_Old) > 3 ) {
  int bend = map(read_Accel,accel_Zero,675,0,8191);
  bend = constrain(bend, -8192,8191);
bend = bend + 8192;
  byte highnibble = bend >> 7;
  byte lownibble = bend & 0x7f;
  if (channel_Mode) {
  if (running_byte != 0xE1 + CHANNEL_OFFSET + c) Serial1.write(byte(0xE1 + CHANNEL_OFFSET +c));
  Serial1.write(byte(lownibble));
  Serial1.write(byte(highnibble));
  running_byte = 0xE1 + CHANNEL_OFFSET + c;
 
} else {
  if (running_byte != 0xE0) Serial1.write(byte(0xE0));
  Serial1.write(byte(lownibble));
  Serial1.write(byte(highnibble));
  running_byte = 0xE0;
}
MIDIEvent event = {0x0E, 0xE0, lownibble, highnibble};
  MIDIUSB.write(event);
 read_Accel_Old = read_Accel;
  is_Middle = 0;
}
} else if (!is_Middle) {
  if (channel_Mode) {
  if (running_byte != 0xE1 + CHANNEL_OFFSET + c) Serial1.write(byte(0xE1 + CHANNEL_OFFSET + c));
  Serial1.write(byte(0));
  Serial1.write(byte(64));
    running_byte = 0xE1 + CHANNEL_OFFSET + c;
  } else {
    if (running_byte != 0xE0) Serial1.write(byte(0xE0));
  Serial1.write(byte(0));
  Serial1.write(byte(64));
    running_byte = 0xE0;
  }
  MIDIEvent event = {0x0E, 0xE0, 0, 64};
  MIDIUSB.write(event);
  is_Middle = 1;
   }
  } else if (control_counter < CONTROL_RATE) control_counter = control_counter + 1;
  } else accel_Zero = read_Accel;
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
  
   
    int sensorMin = 1023;
    int val;
    
      //loop through the array of fret definitions
      for (byte j=24; j>0; j--) {
      
        int response = false;
        int fsrHit = 0;
        //wait for response
        while (!response) {
        
          //read piezo val
          int fsrVal = analogRead(fsr_Pins[i]);
          //get the sensor min value (highest fret) on the first round
          if (j==24) {
            int fretVal = analogRead(soft_Pot_Pins[i]);
      //      if (fretVal > sensorMax) (sensorMax = fretVal);
       
            //if the fsr is hit, register this as the definition for this fret
            if (fsrVal < (fsr_Init[i] - 80) && fsrHit == 0) {
              
              int fretVal = analogRead(soft_Pot_Pins[i]);
              sensorMin = fretVal;
              val = fretVal;
              response = true;
              fsrHit = 1;
            }
            if (fsrVal > (fsr_Init[i] -50) && fsrHit == 1) fsrHit == 0;
          }
        
          else {
            //get the rest of the fret definitions
            //if the piezo is hit, register this as the definition for this fret
            if (fsrVal < (fsr_Init[i] - 80) && fsrHit == 0) {
              int fretVal = analogRead(soft_Pot_Pins[i]);
              fretVal = map(fretVal, sensorMin, MAXREADING, 0, 255);
              if (fretVal > 255) fretVal = 255;
              val = fretVal;
              response = true;
              fsrHit = 1;
            } 
            if (fsrVal >  (fsr_Init[i] -50) && fsrHit == 1) fsrHit == 0;
          }
        }
      
        //write to memory
        digitalWrite(PIN_LED, LOW);
        EEPROM.write(j + (24*i), val);
        
        delay(100);
        digitalWrite(PIN_LED, HIGH);
      }
    
    //update global definitions
    calibration_Min[i] = EEPROM.read(24 + (24*i));

    for (byte j=1; j<24; j++) {
      fret_Defs[i][j] = EEPROM.read(j + (i*24));
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
  if (channel_Mode){ cmd = cmd + CHANNEL_OFFSET;
  }
  if (cmd != running_byte) Serial1.write(byte(cmd));
  Serial1.write(byte(pitch));
  Serial1.write(byte(velocity));
    //kind of mpe, starting at channel 2, channel 1 is for general modulation only
  MIDIEvent noteOn = {0x09, cmd, pitch, velocity};
  MIDIUSB.write(noteOn);
  running_byte = cmd;
}



//Sends controller change to the specified controller
inline void controllerChange(byte controller, byte value) {
    if (channel_Mode) {     
  if (running_byte != (0xB1 + CHANNEL_OFFSET + c)) Serial1.write(byte(0xB1 + CHANNEL_OFFSET + c));
   Serial1.write(byte(controller));
   Serial1.write(byte(value));
   running_byte = 0xB1 + CHANNEL_OFFSET + c;
   
  } else {
    
 if (running_byte != 0xB0) Serial1.write(byte(0xB0));
  Serial1.write(byte(controller));
  Serial1.write(byte(value));
  running_byte = 0xB0;
  }
 
   //kind of mpe, channel 1 is for general modulation only 
 MIDIEvent event = {0x0B, 0xB0, controller, value};
  MIDIUSB.write(event);

}

inline void controllerAllChannels(byte controller, byte value) {
    if (channel_Mode) {
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
  running_byte = 0;
}

inline void aftertouchChange(byte cmd, byte value) {
 if (running_byte != cmd) Serial1.write(byte(cmd));
 Serial1.write(byte(value));
   //kind of mpe, starting at channel 2, channel 1 is for general modulation only
 MIDIEvent aftertouch = { 0x0D, cmd, value};
 MIDIUSB.write(aftertouch);

running_byte = cmd;
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
  MIDIEvent programchange1 = { 0x0C, 0xC1 + CHANNEL_OFFSET, progNr};
  MIDIUSB.write(programchange1);
  MIDIEvent programchange2 = { 0x0C, 0xC2 + CHANNEL_OFFSET, progNr};
  MIDIUSB.write(programchange2);
  MIDIEvent programchange3 = { 0x0C, 0xC3 + CHANNEL_OFFSET, progNr};
  MIDIUSB.write(programchange3);
  MIDIEvent programchange4 = { 0x0C, 0xC4 + CHANNEL_OFFSET, progNr};
  MIDIUSB.write(programchange4);
    running_byte = 0;
}
//-------------------OTHER FUNCTIONS---------------//
//serial midi in parser control the ribbonbass modes, mirror program change to midi out.
inline void midiParser(){
  switch (status_Byte){
    
    case 0xC0: //program change
    save_Slot = data_Byte1;
    loadSettings(data_Byte1);
    programChange(data_Byte1);
    break;
    case 0xB0: //cc messages 
    switch (data_Byte1) {
       case ALLMUTE_CC:
    controllerAllChannels(123,0);
       break; 
      case SAVE_CC:
      saveSettings();
      break; 
      
      case CHANNELMODE_CC:
      if (string_Active[0]) {
              if (!channel_Mode) old_c = 0;
        noteOn(0x91 + old_c, active_Note[0], 0);    
        }

       if (string_Active[1]) {
              if (!channel_Mode) old_c = 1;
        noteOn(0x91 + old_c, active_Note[1], 0);    
        }

       if (string_Active[2]) {
              if (!channel_Mode) old_c = 2;
        noteOn(0x91 + old_c, active_Note[2], 0);    
        }

        if (string_Active[3]) {
              if (!channel_Mode) old_c = 3;
        noteOn(0x91 + old_c, active_Note[3], 0);    
        }     
      c = 0;
      channel_Mode = !channel_Mode;
      break; 
        case BOWMODE_CC:
      bow_Mode = !bow_Mode;
        break; 
        case DUMMY_CC:
        bow_Mode = !bow_Mode;
        break;
      case FSR_SLAVE_CC:
      fsr_Add_Slave_Mode = !fsr_Add_Slave_Mode;
      break;
   /*   case FULLLEGATO_CC:
      if (string_Active[0]) {
              if (!channel_Mode) old_c = 0;
        noteOn(0x91 + old_c, active_Note[0], 0);    
        }

       if (string_Active[1]) {
              if (!channel_Mode) old_c = 1;
        noteOn(0x91 + old_c, active_Note[1], 0);    
        }

       if (string_Active[2]) {
              if (!channel_Mode) old_c = 2;
        noteOn(0x91 + old_c, active_Note[2], 0);    
        }

        if (string_Active[3]) {
              if (!channel_Mode) old_c = 3;
        noteOn(0x91 + old_c, active_Note[3], 0);    
        }     
      full_Legato_Mode = !full_Legato_Mode;
      break;
      */
      case OCTAVE_CC:
      octave = data_Byte2 & 7;
      byte string_base = 12*octave + LOWEST_NOTE;
        offsets[0] = string_base; 
        offsets[1] = string_base + 5; 
        offsets[2] = string_base + 10; 
        offsets[3] = string_base + 15; 
       break;
           
    }
    break;
   
  }
}



//save settings to eeprom on cc SAVE_CC (slot is last received program change)
 void saveSettings() {
 // if (octave < 0) octave = 0;
  byte settings = (channel_Mode << 7) | (full_Legato_Mode << 6) | (fsr_Add_Slave_Mode << 5) | (bow_Mode << 4) | octave;
  EEPROM.write(save_Slot + 100, settings);
}

// load settings from eeprom on program change
void loadSettings(byte prgCh) {

  
  byte settings = EEPROM.read(prgCh + 100);
  if (((settings >> 7) & 1) != channel_Mode) {
     if (string_Active[0]) {
              if (!channel_Mode) old_c = 0;
        noteOn(0x91 + old_c, active_Note[0], 0);    
        }

       if (string_Active[1]) {
              if (!channel_Mode) old_c = 1;
        noteOn(0x91 + old_c, active_Note[1], 0);    
        }

       if (string_Active[2]) {
              if (!channel_Mode) old_c = 2;
        noteOn(0x91 + old_c, active_Note[2], 0);    
        }

        if (string_Active[3]) {
              if (!channel_Mode) old_c = 3;
        noteOn(0x91 + old_c, active_Note[3], 0);    
        }     
  c = 0;
  }
  channel_Mode = (settings >> 7) & 1;
  full_Legato_Mode = (settings >> 6) & 1;
  fsr_Add_Slave_Mode = (settings >> 5) & 1;
  bow_Mode = (settings >> 4) & 1;
  octave = settings & 15;
  byte string_base = 12*octave + LOWEST_NOTE;
        offsets[0] = string_base; 
        offsets[1] = string_base + 5; 
        offsets[2] = string_base + 10; 
        offsets[3] = string_base + 15;           
}

