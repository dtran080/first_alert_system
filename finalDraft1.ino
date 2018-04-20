//lcd library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//pin for IR sensor: analog
const int pinL = A2;
const int pinC = A0;
const int pinR = A1;
int delayTimeL, counterL, delayTimeC, counterC, delayTimeR, counterR;
int timeUnit;
//digital for led lights
const int ledL = 8;
const int ledC = 10;
const int ledR = 12;
//led light state is on:
//bool ledLeft = false;
//bool ledCenter = false;
//bool ledRight = false;
const int minVoltThreshold = 130;
/// done with IR reader declaration
/// lcd button display system ()
LiquidCrystal_I2C lcd(0x3f,16,2);  // run ic2_scanner sketch and get the IC2 address, which is 0x3f in my case,it could be 0x3f in many cases

//special character
byte customBackslash[8] = {
  0b00000,
  0b10000,
  0b01000,
  0b00100,
  0b00010,
  0b00001,
  0b00000,
  0b00000
};
byte block[8] = {
	0b11110,
	0b11101,
	0b01011,
	0b10111,
	0b01011,
	0b11101,
	0b11110,
	0b11111};
//button pins
int btn1Pin = 5;  //left
int btn2Pin = 6; //flash
int btn3Pin = 7; //right
bool URGENT = false;  //immidate flash light
//start counter
int currentState = 0;
int btn1State = LOW;
int btn2State = LOW;
int btn3State = LOW;
//previous time
long btn1Prev = 0;
long btn2Prev = 0;
long btn3Prev = 0;
long urgentPrev = 0;
long currentMilis = 0;
const int SWITCH_TIME = 7000;
///done with 
void setup() {
    // IR Scanner Declaration
    //Serial.begin(9600);
    pinMode(ledL, OUTPUT);
    pinMode(ledC, OUTPUT);
    pinMode(ledR, OUTPUT);
    
    digitalWrite(ledL, HIGH);
    digitalWrite(ledC, HIGH);
    digitalWrite(ledR, HIGH);

    delayTimeL = 0;
    counterL = 0;
    delayTimeC = 0;
    counterC = 0;
    delayTimeR = 0;
    counterR = 0;

    timeUnit = 0;
    ///done with IR Scanner 
    // Declaration for LCD and button
    lcd.init();
    lcd.backlight();
    Serial.begin(9600); // Starts the serial communication
    lcd.createChar(7, customBackslash);
    lcd.createChar(0, block);
}

void loop() {
    int sumL = 0;
    int sumC = 0;
    int sumR = 0;
    for(int i=0;i<25;i++){
      sumL += analogRead(pinL);
      sumC += analogRead(pinC);
      sumR += analogRead(pinR);
      delay(2);
    }
    //0.05 seconds
    //NO DELAY AFTER THIS POINT.
    double voltageL = sumL/25.0;
    double voltageC = sumC/25.0;
    double voltageR = sumR/25.0;
    // ten samples in half second

    //LEFT
    if(minVoltThreshold < voltageL){ // cut off at around 200 in?
  //    delayTimeL = calcBlink(voltageL);
      blinkAt(voltageL, ledL, counterL, delayTimeL);
  //    counterL++;
    }else{
      digitalWrite(ledL, LOW);
      delayTimeL = 0;
    }
    
    //CENTER
    if(minVoltThreshold < voltageC){ // cut off at around 200 in?
  //    delayTimeC = calcBlink(voltageC);
      blinkAt(voltageC, ledC, counterC, delayTimeC);
  //    counterC++;
    }else{
      digitalWrite(ledC, LOW);
      delayTimeC = 0;
    }

    //RIGHT
    if(minVoltThreshold < voltageR){ // cut off at around 200 in?
  //    delayTimeR = calcBlink(voltageR);
      blinkAt(voltageR, ledR, counterR, delayTimeR);
  //    counterR++;
    }else{
      digitalWrite(ledR, LOW);
      delayTimeR = 0;
    }
    
  //   Serial.print("Reading: "); 
  //   Serial.print(voltageL);
  //   Serial.print(", ");
  //   Serial.print(voltageC);
  //   Serial.print(", ");
  //   Serial.print(voltageR);
  //   Serial.print(". ");;
  //   Serial.print("\tDelay: ");
  //   Serial.print((double)delayTimeL * 0.05);
  //   Serial.print("sec, ");
  //   Serial.print((double)delayTimeC * 0.05);
  //   Serial.print("sec, ");
  //   Serial.print((double)delayTimeR * 0.05);
  //   Serial.print("sec, ");
  //   Serial.println();
    timeUnit++;
    if(timeUnit < 0){// must reset if timeUnit overflows to negative
      timeUnit = 0;
    }

    //LED LIGHT AND BUTTONS SWITCHES
    readPin();
    currentMilis = millis();
    if (btn1State==HIGH)
    {
      btn1Prev = currentMilis; 
    //	Serial.print("left");
      currentState = 1;
      leftSetup();
    } 
    else{
      
    }
    
    if (btn2State==HIGH)
    {
      btn2Prev = currentMilis; 
      currentState = 2;
    //	Serial.print("middle");
    } 
    else{
    }

    if (btn3State==HIGH)
    {
      btn3Prev = currentMilis; 
      currentState = 3;
    // Serial.print("right");
      rightSetUp();
    
    } 
    else{
      //btn3Prev = millis();
    }
    
    // if (currentMilis-urgentPrev<SWITCH_TIME)
    //   flashingURGENT();
    
    if (currentState==1){
      left();
    } else if (currentState==2){
      flashing();
    } else if (currentState==3){
      right();
    }


}
// IR SCANNER methods
void blinkAt(double voltage, int LED, int& counter, int& delayTime){
  delayTime = calcBlink(voltage);
  if(delayTime == 2){
    digitalWrite(LED, HIGH); //if very close
  }else if(counter <= delayTime/2){
    digitalWrite(LED, LOW);
  }else if(delayTime/2 <= counter && counter < delayTime){
    digitalWrite(LED, HIGH);
  }else{
    counter = 0;
  }
  counter++;
}

int calcBlink(int result){
//  int maxVoltageCap = 400;
//  int minVoltageCap = 200;
//  if(result > maxVoltageCap){
//    result = maxVoltageCap; // in case it overloads, stay max besides 
//  }
////  int scaled = map(result, 200, 600, 160000, 16); //distance and voltage is reciprocal, linear conversion
//  int scaled = map(result, minVoltageCap, maxVoltageCap, 400, 4); //distance and voltage is reciprocal, linear conversion
////  int scaled = map(result, 200, 600, 0, 20); //distance and voltage is reciprocal, linear conversion
//  int delayTime = pow(scaled, 0.5); //voltage and blinkTime is proportional, 
////  int delayTime = sqrt(scaled);
////delayTime MUST be >2 to compensate for halftime for LOW.
////  int delayTime = 22 - ( scaled); //upside down squared. initial should be max possible + min possible

  int delayTime = 2;
  //below threshold will not trigger light
  URGENT = false;
  if(minVoltThreshold < result && result <=200){  //farthest ~ 600 cm
    delayTime = 20; //1 sec blink cycle
    //URGENT = false;
  }else if(200 < result && result <=250){
   // URGENT = false;
    delayTime = 8;  //0.4
  }else if(250 < result && result <=370){
    // URGENT = true;
    // urgentPrev = millis();
    delayTime = 4; // 0.2 sec blink cycle
  }else if(370 < result){                         //closest ~50 cm
    // URGENT = true;
    // urgentPrev = millis();
    delayTime = 2; //light will stay on non-blink
  }
  
  
  return delayTime;
}
/// DONE:: IR SCANNER METHODS

/// LED LIGHT METHODS

void rightSetUp(){	//btn3
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("---------");
	//lcd.setCursor(6,0);
	for (int i=0;i<20;i++){
		lcd.write(byte(7));
	}
	lcd.print("     ");
	lcd.setCursor(0,1);
	lcd.print("---------////////////////////");
	lcd.print("     ");

}

void flashBlock(){	//btn2
	lcd.clear();
	lcd.setCursor(0,0);
	for (int i=0;i<16;i++){
		lcd.write(byte(0));
	}
	lcd.setCursor(0,1);
	
	for (int i=0;i<16;i++){
		lcd.write(byte(0));
	}
}
void left(){ // left setup
	while (millis()-btn1Prev<SWITCH_TIME){
		//act1Count++;
		//Serial.print(currentMilis-btn1Prev);
		readPin();
		if ((millis()-btn1Prev)%75==0){
			lcd.scrollDisplayLeft();
		}
		if (btn1State==LOW && (btn2State==HIGH || btn3State==HIGH)){

			return;
		}
	}
}
void flashing(){
	while (millis()-btn2Prev<SWITCH_TIME){
		readPin();
		
		if ((millis()-btn2Prev)%500==5){ //stop
			lcd.clear();
			//btn2Prev = 0;
			
		}
		if ((millis()-btn2Prev)%500==200){ //flash
			flashBlock();
		}

		if (btn2State==LOW && (btn1State==HIGH || btn3State==HIGH)){
			return;
		}
	}
}
void flashingURGENT(){
    while (millis()-urgentPrev<SWITCH_TIME){
      readPin();
      
      if ((millis()-urgentPrev)%400==5){ //stop
        lcd.clear();
      }
      if ((millis()-btn2Prev)%400==180){ //flash
        flashBlock();
      }

      if ((btn1State==HIGH || btn3State==HIGH)&&(millis()-urgentPrev>SWITCH_TIME/2)){ //3 seconds of URGENT before switches
        return;
      }
    }

}
void right(){ //right setup
	while (millis()-btn3Prev<SWITCH_TIME){
		//act1Count++;
		//Serial.print(currentMilis-btn1Prev);
		readPin();
		if ((millis()-btn3Prev)%75==0){
			lcd.scrollDisplayRight();
		}
		if (btn3State==LOW && (btn2State==HIGH || btn1State==HIGH)){
			//Serial.print("Switch from 1");
			//act1Count=0;
			return;
		}
	}
}
void leftSetup(){	//btn1
	
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("     ////////////////////---------");
	lcd.setCursor(0,1);
	lcd.print("     ");
	for (int i=0;i<20;i++){
		lcd.write(byte(7));
	}
	lcd.print("---------");
}
void readPin(){
	btn1State = digitalRead(btn1Pin);
	btn2State = digitalRead(btn2Pin);
	btn3State = digitalRead(btn3Pin);
}
