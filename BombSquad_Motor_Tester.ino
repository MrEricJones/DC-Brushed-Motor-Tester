/* BombSquad Motor Tester
 *
 *  This program reads a potentiometer and runs a Victor or Talon motor controller from full 
 *  forward to full reverse using PWM.  This program uses the servo library.  Since these 
 *  motor controllers don't usepwm from 0 to 255 like servo, the range is 
 *  modified to go from 1000 to 2000 with 1500 being neutral.  Adjustments must be made to 
 *  the "map" function to calibrate the pot to the full range of the motor controller. There
 *  is also voltage and current monitoring for power display.
*/ 

//-------------------------------------------------------------------------------------------------------
#include <Servo.h>
#include <LiquidCrystal.h>
#include <LcdBarGraph.h>
//-------------------------------------------------------------------------------------------------------
const int talon_pin = 13;
const int trigger_pin = 7;
const int leftbutton_pin = 8;
const int centerbutton_pin = 9;
const int rightbutton_pin = 10;
const int pot_pin = A0;
const int current_pin = A2;
const int batvoltage_pin = A4;


Servo talon;                                   // declare servo object to control the victor

int potvalue = 0;                               // initiate "potvalue" as a variable
int mappedpotvalue = 0;
int displaypotvalue = 0;
int bargraphvalue = 0;
int bargraphvalue2 = 0;
int voltageval = 0;
float tmpv_value = 0;
float voltage = 0.0;
float dispvoltage = 0.0;
int tmpcval = 512;
float current = 0.0;
float dispcurrent = 0.0;
float power = 0;
int throttle = 1500;
int trigger = 1;
int dispbutton1 = 0;
int dispbutton2 = 0;
int dispbutton3 = 0;
int rate_accel = 5;
int accel_inc = 5;
int rate_decel = 5;
int decel_inc = 5;

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
byte lcdNumCols = 20;                            // number of columns in the LCD

LcdBarGraph lbg1(&lcd, 20, 0, 2);
LcdBarGraph lbg0(&lcd, 20, 0, 3);                 // creating bar graph instance, format is (&lcd, lcdNumCols, start X, start Y). So (&lcd, 16, 0, 1) would set the bargraph length to 16 columns and start the bargraph at column 0 on row 1.

byte PWM_PIN = 6;
 
int pwm_value;

const int numReadings = 20;               // the number of points to average
int readings[numReadings];                 // the readings from the analog input
int readIndex = 0;                         // the index of the current reading
int total = 0;                             // the running total
int avg_cur = 0;                           // the average

byte adjPWM = 5;                           // increment of adjustment
unsigned long changeIntervalMillis = 10;
unsigned long prevChangeMillis = 0;

int motorPWM=1500;

//-------------------------------------------------------------------------------------------------------
void setup() {                                    // put your setup code here, to run once:
  talon.attach(talon_pin);                              // attaches the servo on pin 9 to the servo object 
  Serial.begin(115200);                             // initialize serial communications at 9600 bps:
  lcd.begin(20, 4);                               // set up the LCD's number of columns and rows:
  lcd.clear();
  pinMode(trigger_pin, INPUT_PULLUP);
  pinMode(leftbutton_pin, INPUT_PULLUP);
  pinMode(centerbutton_pin, INPUT_PULLUP);
  pinMode(rightbutton_pin, INPUT_PULLUP);
  pinMode(PWM_PIN, INPUT);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) // this starts the array out
  {readings[thisReading] = 0;                                         // with a fresh set of zeros
  }
}

//-------------------------------------------------------------------------------------------------------

void loop() {
    lcdprint();                                   // This loop prints info to the 20 x 4 LCD
	serprint();                                   // This loop transmits debugging data to the serial port
	read_pot();                                   // This loop reads the potentiometer
    read_buttons();                               // This loops read all the buttons
	map_pot();                                    // This loop reads the input from the pot
	drive_pwm_smooth();                           // This loop sends PWM to the speed controller with acc/dec funtions
    cvpmont();                                    // This loop reads the voltage and current circuits and calculates power
    //PWMdetect();                                // This loop reads the PWM from D6 for testing/debugging(slows the program down badly)
}

void lcdprint() {  
  lcd.setCursor(3,0);
  lcd.print("Voltage = ");
  lcd.setCursor(13,0);
  lcd.print(dispvoltage);
  //lcd.setCursor(14,1);
  //lcd.print(" ");
  lcd.setCursor(1,1);
  lcd.print("Current = ");
  lcd.setCursor(11,1);
  lcd.print(dispcurrent);
  //lcd.setCursor(5,2);
  //lcd.print("PWM = ");
  //lcd.setCursor(11,2);
  //lcd.print(motorPWM);
  lbg1.drawValue(bargraphvalue2, 1000);  // output
  lbg0.drawValue(bargraphvalue, 1000);   // setpoint
}

void serprint() {  
  Serial.print("T=");                     
  Serial.print(trigger);
	Serial.print(" L=");                     
	Serial.print(dispbutton1);
  Serial.print(" C=");                
  Serial.print(dispbutton2);
	Serial.print(" R=");                
	Serial.print(dispbutton3);
	Serial.print(" POT=");                         // print out the potentiometer value you read:
  Serial.print(potvalue);
	Serial.print("  Throt=");
	Serial.print(motorPWM);
  Serial.print("  MappedPotValue="); 
  Serial.print(mappedpotvalue);    
  Serial.print("  Batt="); 
  Serial.print(voltage); Serial.print("V"); 
  Serial.print("  Current="); 
  Serial.print(current); Serial.print("A");
	Serial.print("  rawCurrent=");
	Serial.print(tmpcval); Serial.print("A");
  Serial.print("  Power="); 
  Serial.println(power); Serial.println("W");
}

void read_pot() {
  potvalue = analogRead(pot_pin);                        // read the input on analog pin 0 and write to var potvalue:
  displaypotvalue = map(potvalue, 5, 1017, 1000, 2000);
  bargraphvalue = displaypotvalue - 1000;                    //Display speed on lcd in graph form
  //float voltage = potvalue * (5.0 / 1023.0);           // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
}

void read_buttons() {
trigger = digitalRead(trigger_pin);
dispbutton1 = digitalRead(leftbutton_pin);
dispbutton2 = digitalRead(centerbutton_pin);
dispbutton3 = digitalRead(rightbutton_pin);
};

void map_pot() {
  if(trigger == 0) {
     mappedpotvalue = map(potvalue, 5, 1017, 1000, 2000);       //1000:full rev  1500:neutral  2000:full forward
  }
  else {
     mappedpotvalue = 1500;
  }
}


void drive_pwm_smooth() {
if (millis() - prevChangeMillis >= changeIntervalMillis) {
	prevChangeMillis += changeIntervalMillis;
	if (motorPWM > mappedpotvalue) {
		motorPWM -= adjPWM;
	}
	else if (motorPWM < mappedpotvalue) {
		motorPWM += adjPWM;
	}
	talon.writeMicroseconds(motorPWM);
	bargraphvalue2 = motorPWM - 1000;
}
}

void cvpmont() {
//-------CURRENT----------------------
  tmpcval = analogRead(current_pin);
  total = total - readings[readIndex];                   // subtract the last reading:
  readings[readIndex] = tmpcval;                         // read from the sensor:
  total = total + readings[readIndex];                   // add the reading to the total:
  readIndex = readIndex + 1;                             // advance to the next position in the array:
   if (readIndex >= numReadings) {                       // if we're at the end of the array...
     readIndex = 0;                                      // ...wrap around to the beginning:
   }
  avg_cur = total / numReadings;
  current = ((((avg_cur * (5000 / 1023.0))-2500)/66));
  dispcurrent = round(current*100)/100;
//-------VOLTAGE----------------------  
  voltageval = analogRead(batvoltage_pin);
  tmpv_value = (voltageval * 5.0) / 1024.0; 
  voltage = tmpv_value / (30000.0/(100000.0+30000.0));
  dispvoltage = round(voltage*100)/100;
//-------POWER------------------------
  power = current*voltage;
  delay(1);
}

void PWMdetect() {
  pwm_value = pulseIn(PWM_PIN, HIGH);
}








