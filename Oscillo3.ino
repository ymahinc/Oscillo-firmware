#include <Wire.h>

char inChar = '0'; 
String currentCmd = "";
int state = 0; // state stores the command level:
int trigState = 0; 
// 0 is the state in wich we wait for a root command:
//HELLO 
//GETYRES1 
//GETYRES2 
//GETXRES 
//GETTRMOD 
//GETITR 
//YRES
//XRES
//INVTRIG
//TRIGMOD
//
//When YRES command is received => switch to state 1 
//                              and wait for channel number .Next command should be "1" or "2" to be valid.
//                              If a valid command is received => switch to state 2
//                                                                Next command should be new y res for deired channel
//                                                                Should be beetween 1 and 9 included to be valid
//                                                                 => switch back to state 0
//When XRES  command is received => switch to state 3
//                                  and wait for new x resolution. Next command should be beetween "1" and "18" included to be valid.
//                                  => switch back to state 0
//When INVTRIG command is received  => switch to state 4 
//                                  and wait for tigger detection mode. Next command should be "1" or "2" to be valid.
//                                  => switch back to state 0
//When TRIGMOD command is received => switch to state 5
//                                    and wait for new trigger mode. Next command should be beetween "1" and "5" included to be valid.
//                                    => switch back to state 0
//
int currentCanal = 1;
int currentYRes1 = 1;
int currentYRes2 = 1;
int currentXRes = 1;
bool invTrig = false;
int trigMode = 1;
byte BufferVoie1[1024];
byte BufferVoie2[1024];
int ptr = 0;
int currentCouplingMode1 = 0;
int currentCouplingMode2 = 0;
int currentTrigVal = 127;
/*
#define DSRPin 2
#define RTSPin 3*/
#define TimeDivMux1S2Pin 50
#define TimeDivMux1S1Pin 51
#define TimeDivMux1S0Pin 52
#define TimeDivMux2S0Pin 39
#define TimeDivMux2S1Pin 29

//#define TrigSelMuxAPin 38 // PJ5
//#define TrigSelMuxBPin 39 // PJ6

#define RSTPin 27
#define EOSPin 25
#define StartPin 24
#define PolPin 22
//#define INCAPin 8//pj7
#define HalfFullPin 26
#define TriggeredPin 23

#define RL3Pin 37
#define RL4Pin 41
#define RL5Pin 40

#define RL7Pin 30
#define RL8Pin 33
#define RL9Pin 35
#define RL10Pin 36
#define RL11Pin 32
#define RL12Pin 34

#define EnableTDA1 40
#define EnableTDA2 31

void setup() {
  DDRF = B00000000; // Port C as INPUT, Channel 2
  DDRL = B00000000; // Port L as INPUT, Channel 1
  
  DDRD = B11111011;  // 0 = input, 1 = output
  DDRJ = B11111110;  // 0 = input, 1 = output
  
  pinMode(TimeDivMux1S2Pin, OUTPUT);
  pinMode(TimeDivMux1S1Pin, OUTPUT);
  pinMode(TimeDivMux1S0Pin, OUTPUT);
  pinMode(TimeDivMux2S0Pin, OUTPUT);
  pinMode(TimeDivMux2S1Pin, OUTPUT);
 
  digitalWrite(TimeDivMux1S2Pin, LOW);
  digitalWrite(TimeDivMux1S1Pin, LOW);
  digitalWrite(TimeDivMux1S0Pin, LOW);
  digitalWrite(TimeDivMux2S0Pin, LOW);
  digitalWrite(TimeDivMux2S1Pin, LOW);

  applyTrigMode();

  bitClear (PORTD, 4); //enable TDA 1 to low
  pinMode(EnableTDA2, OUTPUT);
  digitalWrite(EnableTDA2, LOW);
  
  pinMode(RSTPin, OUTPUT);
  pinMode(EOSPin, INPUT);
  pinMode(StartPin, OUTPUT);
  pinMode(PolPin, OUTPUT);
  //pinMode(INCAPin, OUTPUT);
  pinMode(HalfFullPin, INPUT);
  pinMode(TriggeredPin, INPUT);

  digitalWrite(RSTPin, HIGH);
  digitalWrite(StartPin, HIGH);
  digitalWrite(PolPin, LOW);
  bitClear(PORTJ,7); // clear INCA
  
  pinMode(RL3Pin, OUTPUT);
  pinMode(RL4Pin, OUTPUT);
  pinMode(RL5Pin, OUTPUT);
  pinMode(RL7Pin, OUTPUT);
  pinMode(RL8Pin, OUTPUT);
  pinMode(RL9Pin, OUTPUT);
  pinMode(RL10Pin, OUTPUT);
  pinMode(RL11Pin, OUTPUT);
  pinMode(RL12Pin, OUTPUT);

  Wire.begin();
  Serial.begin(9600);

  updateTrigVal();

  applyChan1CouplingMode();
  applyChan2CouplingMode();
  
  Serial.begin(9600);
  Serial3.begin(2000000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  while (!Serial3) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  // manage serial communication with the computer
  processEvents();
 
  // reset digital section to start acquisition process
  PORTA &= B11011111;    // fast equivalent of digitalWrite(RSTPin, LOW);
  //delayMicroseconds(1);
  PORTA |= B100000;      // fast equivalent of digitalWrite(RSTPin, HIGH);
  // wait for RAM Half Full
  while (digitalRead(HalfFullPin)) {
    processEvents();
  }    

  //memset(BufferVoie1, 127, sizeof(BufferVoie1));
  //memset(BufferVoie2, 127, sizeof(BufferVoie2));
  
  if ( trigMode == 4 ){ // If trigmod is 4, then launch auto triggering by setting startpin low   
      PORTA &= B11111011;  // fast equivalent of digitalWrite (StartPin, LOW);
  }
      
  // wait for until RAM is full 
  while (!digitalRead(EOSPin)) { 
    processEvents();
  }
  // restore start pin HIGH if needed
  PORTH |= B10;  // fast equivalent of digitalWrite (StartPin, HIGH); 
  // then retreive samples from RAM
  for ( int i = 0; i < 1024; i++){     
    PORTJ |= B10000000;      // fast equivalent of digitalWrite digitalWrite (INCAPin, HIGH);       
    BufferVoie1[i] = PINL; // Stored in reverse bit order, helps routing 
    BufferVoie2[i] = PINF; // Stored in reverse bit order, helps routing 
    PORTJ &= B01111111; // fast equivalent of digitalWrite digitalWrite (INCAPin, LOW); 
  }

  PORTA |= B00000100; 
}

void processEvents(){
  trigState = !digitalRead(TriggeredPin);
  // if we get data in buffer from computer
  if (Serial3.available() > 0) {
    // get incoming byte:
    inChar = Serial3.read();
    
    if ( inChar == 13 || inChar == '\n' ){
      // InChar is the end line character so we should have a complete command string
      // set commandRecognized to false, will be toggled to true if currentCmd is valid
      bool commandRecognized = false;

    if ( state == 8 ){ // waiting for chan number to change coupling mode
        int value = currentCmd.toInt();
        if ( value >= 0 && value <= 255 ){
          currentTrigVal = value;
          //Serial.print("Canal " + String(currentCanal));
          commandRecognized = true;
          Serial3.println("OK");
          updateTrigVal();
        }
        state = 0;
      }
      
      if ( state == 7 ){ // waiting to new trig mod
        if ( currentCmd == "AC" ){
          if ( currentCanal == 1 ){
            currentCouplingMode1 = 0;
            applyChan1CouplingMode();
          }else{
            currentCouplingMode2 = 0;
            applyChan2CouplingMode();
          }
          commandRecognized = true;
          Serial3.println("OK");
        }
        if ( currentCmd == "DC" ){
          if ( currentCanal == 1 ){
            currentCouplingMode1 = 1;
            applyChan1CouplingMode();
          }else{
            currentCouplingMode2 = 1;
            applyChan2CouplingMode();
          }
          commandRecognized = true;
          Serial3.println("OK");
        }
        if ( currentCmd == "GND" ){
          if ( currentCanal == 1 ){
            currentCouplingMode1 = 2;
            applyChan1CouplingMode();
          }else{
            currentCouplingMode2 = 2;
            applyChan2CouplingMode();
          }
          commandRecognized = true;
          Serial3.println("OK");
        }
        state = 0;
      }
      
      if ( state == 6 ){ // waiting for chan number to change coupling mode
        if ( currentCmd == "1" ){
          currentCanal = 1;
          //Serial.print("Canal " + String(currentCanal));
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 7;
        }
        if ( currentCmd == "2" ){
          currentCanal = 2;
          //Serial.print("Canal " + String(currentCanal));
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 7;
        }
      }
      
      if ( state == 5 ){ // waiting to new trig mod
        int value = currentCmd.toInt();
        if ( value > 0 && value < 6 ){
          trigMode = value;
          applyTrigMode();
          commandRecognized = true;
          //Serial.println(" nouvelle valeur: " + String(trigMode));
          Serial3.println("OK");
        }
        state = 0;
      }
      
      if ( state == 4 ){ // waiting to invert trig value
        int value = currentCmd.toInt();
        if ( value > 0 && value < 3 ){
          invTrig = bool(value - 1);
          digitalWrite(PolPin,invTrig);
          commandRecognized = true;
          //Serial.println(" nouvelle valeur: " + String(invTrig));
          Serial3.println("OK");
        }
        state = 0;
      }

      if ( state == 3 ){ // waiting to new x res
        int value = currentCmd.toInt();
        if ( value > 0 && value < 19 ){
          currentXRes = value;
          applyTimeRes();
          commandRecognized = true;
          //Serial.println(" nouvelle res Time/Div: " + String(currentXRes));
          Serial3.println("OK");
        }
        state = 0;
      }
      
      if ( state == 2 ){ // waiting to new y res
        int value = currentCmd.toInt();
        if ( value > 0 && value < 10 ){
          int currentYRes = value;
          commandRecognized = true;
          //Serial.println(" nouvelle res: " + String(currentYRes));
          if ( currentCanal == 1 ){
            currentYRes1 = currentYRes;
            applyCanal1Res();
          }else{
            currentYRes2 = currentYRes;
            applyCanal2Res();
          }
          Serial3.println("OK");
        }
        state = 0;
      }

      if ( state == 1 ){ // waiting for channel number to change y res
        if ( currentCmd == "1" ){
          currentCanal = 1;
          //Serial.print("Canal " + String(currentCanal));
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 2;
        }
        if ( currentCmd == "2" ){
          currentCanal = 2;
          //Serial.print("Canal " + String(currentCanal));
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 2;
        }
      }
      
      if ( state == 0 ){ // Waiting for new command      
        if ( currentCmd == "HELLO" ){
          //Serial.println("Il faut repondre HELLO!!");
          commandRecognized = true;
          Serial3.println("HELLO");
        }
        if ( currentCmd == "GETYRES1" ){
          //Serial.println("Demande de YRES1 ");
          commandRecognized = true;
          Serial3.println(String(currentYRes1));
        }
        if ( currentCmd == "GETYRES2" ){
          //Serial.println("Demande de YRES2 ");
          commandRecognized = true;
          Serial3.println(String(currentYRes2));
        }
        if ( currentCmd == "GETXRES" ){
          //Serial.println("Demande de XRES ");
          commandRecognized = true;
          Serial3.println(String(currentXRes));
        }
        if ( currentCmd == "GETTRMOD" ){
          //Serial.println("Demande de TRIGMOD ");
          commandRecognized = true;
          Serial3.println(String(trigMode));
        }
        if ( currentCmd == "GETITR" ){
          //Serial.println("Demande de INVTRIG ");
          commandRecognized = true;
          Serial3.println(String(invTrig));
        }   
        if ( currentCmd == "YRES" ){
          //Serial.println("Changer resolution ");
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 1;
        }
        if ( currentCmd == "XRES" ){
          //Serial.print("Changer Time/Div ");
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 3;
        }
        if ( currentCmd == "INVTRIG" ){
          //Serial.print("Inverser trigger ");
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 4;
        }
        if ( currentCmd == "TRIGMOD" ){
          //Serial.print("Change trigger mode ");
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 5;
        }
        
        if ( currentCmd == "GETDATA1" ){
          commandRecognized = true;
          Serial3.write(BufferVoie1,1024);
        }
      
        if ( currentCmd == "GETDATA2" ){
          commandRecognized = true;
          Serial3.write(BufferVoie2,1024); 
        }

        if ( currentCmd == "GETTRIG" ){
          commandRecognized = true;
          Serial3.write(trigState); 
        }

        if ( currentCmd == "COUP" ){
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 6;
        }

        if ( currentCmd == "TRIGV" ){
          commandRecognized = true;
          Serial3.println("WAIT");
          state = 8;
        }
      }
      
      if ( ! commandRecognized ) {
        //Serial.println(currentCmd + " Commande non reconnue");
        Serial3.println("Unknow command: " + currentCmd);
        state = 0;
      }
      currentCmd = "";
    }else{
      // inChar is not a end line character, concatenate to get complete command string
      currentCmd = currentCmd + inChar;
    }
  }
}

void updateTrigVal(){
  Wire.beginTransmission(0x2C);
  Wire.write(0);
  Wire.write(currentTrigVal);
  Wire.endTransmission();
}

void applyCanal1Res(){
  //digitalWrite(RL7Pin, HIGH);
  //bitSet(PORTD,5); //RL1
  //bitSet(PORTD,6); //RL2
  //bitSet(PORTD,7); //RL6
 
  switch(currentYRes1){
    case 1:
    digitalWrite(RL4Pin, HIGH);
    bitSet(PORTD,6); //RL2
    bitSet(PORTD,5); //RL1
    break;
    case 2:
    digitalWrite(RL4Pin, HIGH);
    bitSet(PORTD,6); //RL2
    bitClear(PORTD,5); //RL1
    break;
    case 3:
    digitalWrite(RL4Pin, HIGH);
    bitClear(PORTD,6); //RL2
    break;
    
    case 4:
    digitalWrite(RL4Pin, LOW);
    bitClear(PORTD,7); //RL6
    bitSet(PORTD,6); //RL2
    bitSet(PORTD,5); //RL1
    break;
    case 5:
    digitalWrite(RL4Pin, LOW);
    bitClear(PORTD,7); //RL6
    bitSet(PORTD,6); //RL2
    bitClear(PORTD,5); //RL1
    break;
    case 6:
    digitalWrite(RL4Pin, LOW);
    bitClear(PORTD,7); //RL6
    bitClear(PORTD,6); //RL2  
    break;
    
    case 7:
    digitalWrite(RL4Pin, LOW);
    bitSet(PORTD,7); //RL6  
    bitSet(PORTD,6); //RL2
    bitSet(PORTD,5); //RL1
    break;
    case 8:
    digitalWrite(RL4Pin, LOW);
    bitSet(PORTD,7); //RL6
    bitSet(PORTD,6); //RL2
    bitClear(PORTD,5); //RL1
    break;
    case 9:
    digitalWrite(RL4Pin, LOW);
    bitSet(PORTD,7); //RL6
    bitClear(PORTD,6); //RL2
    break;
  }
}

void applyChan1CouplingMode(){
  switch(currentCouplingMode1){
    case 0:   // DC Coupling
    digitalWrite(RL5Pin, HIGH);
    digitalWrite(RL3Pin, HIGH);
    break;
    case 1: // AC Coupling
    digitalWrite(RL5Pin, HIGH);
    digitalWrite(RL3Pin, LOW);
    break;
    case 2: // GND Coupling
    digitalWrite(RL5Pin, LOW);
    break;
  }
}

void applyChan2CouplingMode(){
  switch(currentCouplingMode2){
    case 0:   // DC Coupling
    digitalWrite(RL12Pin, LOW);
    break;
    case 1: // AC Coupling
    digitalWrite(RL12Pin, HIGH);
    digitalWrite(RL10Pin, HIGH);
    break;
    case 2: // GND Coupling
    digitalWrite(RL12Pin, HIGH);
    digitalWrite(RL10Pin, LOW);
    break;
  }
}

void applyCanal2Res(){
/*  switch(currentYRes2){
    case 1:
    digitalWrite(RL2CANAL2Pin, HIGH);
    digitalWrite(RL3CANAL2Pin, HIGH);
    digitalWrite(RL1CANAL2Pin, HIGH);
    digitalWrite(RL4CANAL2Pin, HIGH);
    break;
    case 2:
    digitalWrite(RL2CANAL2Pin, HIGH);
    digitalWrite(RL3CANAL2Pin, HIGH);
    digitalWrite(RL1CANAL2Pin, LOW);
    digitalWrite(RL4CANAL2Pin, HIGH);
    break;
    case 3:
    digitalWrite(RL2CANAL2Pin, HIGH);
    digitalWrite(RL3CANAL2Pin, HIGH);
    digitalWrite(RL1CANAL2Pin, LOW);
    digitalWrite(RL4CANAL2Pin, LOW);
    break;
    case 4:
    digitalWrite(RL2CANAL2Pin, HIGH);
    digitalWrite(RL3CANAL2Pin, LOW);
    digitalWrite(RL1CANAL2Pin, HIGH);
    digitalWrite(RL4CANAL2Pin, HIGH);
    break;
    case 5:
    digitalWrite(RL2CANAL2Pin, HIGH);
    digitalWrite(RL3CANAL2Pin, LOW);
    digitalWrite(RL1CANAL2Pin, LOW);
    digitalWrite(RL4CANAL2Pin, HIGH);
    break;
    case 6:
    digitalWrite(RL2CANAL2Pin, HIGH);
    digitalWrite(RL3CANAL2Pin, LOW);
    digitalWrite(RL1CANAL2Pin, LOW);
    digitalWrite(RL4CANAL2Pin, LOW);
    break;
    case 7:
    digitalWrite(RL2CANAL2Pin, LOW);
    digitalWrite(RL3CANAL2Pin, LOW);
    digitalWrite(RL1CANAL2Pin, HIGH);
    digitalWrite(RL4CANAL2Pin, HIGH);
    break;
    case 8:
    digitalWrite(RL2CANAL2Pin, LOW);
    digitalWrite(RL3CANAL2Pin, LOW);
    digitalWrite(RL1CANAL2Pin, LOW);
    digitalWrite(RL4CANAL2Pin, HIGH);
    break;
    case 9:
    digitalWrite(RL2CANAL2Pin, LOW);
    digitalWrite(RL3CANAL2Pin, LOW);
    digitalWrite(RL1CANAL2Pin, LOW);
    digitalWrite(RL4CANAL2Pin, LOW);
    break;
  }*/
}

void applyTrigMode(){
  //Serial.println(currentXRes);
  
  switch(trigMode){
    case 1:   
    bitSet(PORTJ,5); //TrigSelMuxAPin
    bitClear(PORTJ,6); //TrigSelMuxBPin
    break;
    case 2:   
    bitClear(PORTJ,5); //TrigSelMuxAPin
    bitSet(PORTJ,6); //TrigSelMuxBPin
    break;
    case 3:
    bitClear(PORTJ,5); //TrigSelMuxAPin
    bitClear(PORTJ,6); //TrigSelMuxBPin
    break;
    default:
    bitSet(PORTJ,5); //TrigSelMuxAPin
    bitSet(PORTJ,6); //TrigSelMuxBPin
    break;
  }
}

void applyTimeRes(){
  //Serial.println(currentXRes);
  
  switch(currentXRes){
    case 1:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, LOW);    
    break;  
    case 2:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, HIGH);  
    break;    
    case 3:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, HIGH);
    digitalWrite(TimeDivMux2S0Pin, LOW);   
    break;

    case 4:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, LOW);    
    break;
    case 5:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, HIGH);  
    break;
    case 6:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, HIGH);
    digitalWrite(TimeDivMux2S0Pin, LOW);   
    break;

    case 7:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, HIGH);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, LOW);    
    break;
    case 8:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, HIGH);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, HIGH);  
    break;
    case 9:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, HIGH);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, HIGH);
    digitalWrite(TimeDivMux2S0Pin, LOW);   
    break;

    case 10:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, HIGH);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, LOW);    
    break;
    case 11:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, HIGH);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, HIGH);  
    break;
    case 12:
    digitalWrite(TimeDivMux1S2Pin, LOW);
    digitalWrite(TimeDivMux1S1Pin, HIGH);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, HIGH);
    digitalWrite(TimeDivMux2S0Pin, LOW);   
    break;

    case 13:
    digitalWrite(TimeDivMux1S2Pin, HIGH);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, LOW);    
    break;
    case 14:
    digitalWrite(TimeDivMux1S2Pin, HIGH);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, HIGH);  
    break;
    case 15:
    digitalWrite(TimeDivMux1S2Pin, HIGH);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, LOW);
    digitalWrite(TimeDivMux2S1Pin, HIGH);
    digitalWrite(TimeDivMux2S0Pin, LOW);   
    break;

    case 16:
    digitalWrite(TimeDivMux1S2Pin, HIGH);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, LOW);    
    break;
    case 17:
    digitalWrite(TimeDivMux1S2Pin, HIGH);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, LOW);
    digitalWrite(TimeDivMux2S0Pin, HIGH);  
    break;
    case 18:
    digitalWrite(TimeDivMux1S2Pin, HIGH);
    digitalWrite(TimeDivMux1S1Pin, LOW);
    digitalWrite(TimeDivMux1S0Pin, HIGH);
    digitalWrite(TimeDivMux2S1Pin, HIGH);
    digitalWrite(TimeDivMux2S0Pin, LOW);   
    break;
  }
}

