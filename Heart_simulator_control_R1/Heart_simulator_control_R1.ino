// Version: 1.4
// By : Henry Chan
// A motor control with flow sensor programme for Heart Simulator
// Two outputs: one is PWM at 490Hz and one is a 50% duty-cycle variable frequency output
// PWM & Pulse output will be activated by a switch
// Flow sensor counter is detected by ext. interrupt input; overall number of counts is added & reset in FLOW_TIMER
// Display is refreshed regularly inside the timer interrupt to ensure no display corruption every timer period (500ms)
// A 4 lines x 20 characters LCD module is attached using I2C interface
// LED is to show the output status
// A temp sensor detection is available

// include the library code:
#include <neotimer.h>
#include <Wire.h>  // Arduino IDE built-in
#include <LiquidCrystal_I2C.h>

#define PWM_PIN A0    // PWM control VR
#define PULSE_PIN A1  // PULSE output control
#define TEMP_IN0 A3   // Temp sensor1
#define PWM_OUT 9     // PWM output port (490Hz)
#define PULSE_OUT 10  // PULSE output port
#define OUTPUT_LED 12 // Output enable LED
#define OUTPUT_DET 5  // Output enable detection
#define FLOW_IN0 3    // FLOW sensor input1 as EXT INT
#define FLOW_TIMER 500  // FLOW_TIMER in milli-seconds
#define FLOW_TIMER_END 4 // FLOW_TIMER_END to calculate the flow counts once

// Set the LCD I2C address, lines and rows
LiquidCrystal_I2C lcd(0x27,20,4); //Address: 0x27 for PCF8754; 0x3F for PCF8754A
Neotimer mytimer = Neotimer(FLOW_TIMER); // Set timer interval

byte index = 0;
int PWM_val = 0;    // PWM variable
int PULSE_val = 0;  // Pulse variable
int temp = 25;      // temp1 value
int pulse_half_period = 300;  //Pulse output half period
boolean pulse_output = HIGH;  //PULSE_OUTPUT port value
int FLOW_count = 0; // Flow rate count
float rate = 0;       //Flow rate
int timer_flag = 0;
byte customChar[] = {0x06,0x09,0x09,0x06,0x00,0x00,0x00,0x00}; //Character for degree symbol
byte heart[] = {0x0A,0x1F,0x1F,0x1F,0x1F,0x0E,0x04,0x00};// Heart symbol
byte flash[] = {0x00,0x00,0x08,0x15,0x02,0x00,0x00,0x00}; // Flash symbol
byte danger[]= {0x06,0x0F,0x0F,0x0F,0x06,0x00,0x06,0x06}; // Danger symbol

// Counting the flow rate
void counting() {
  FLOW_count ++;
}

// Calculate the flow rate
void flow_rate() {
  //Calculate the flow rate per timer interrupt
  timer_flag++;
  
  // Display all the values at timer interal
  lcd.setCursor(0, 2);
  lcd.print("FLOW:     PPM:   "); //clear the values and refresh the titles
  lcd.setCursor(5, 2);
  lcd.print(rate);
  lcd.setCursor(14, 2);
  lcd.print(30000/pulse_half_period);
  lcd.setCursor(0, 3);
  lcd.print("TEMP:    C");
  lcd.setCursor(5, 3);
  lcd.print(temp);
  lcd.setCursor(8, 3);
  lcd.write(0);
  if (temp > 60){
    lcd.setCursor(11,3);
    lcd.write(3);
  }
  else{
    lcd.setCursor(11,3);
    lcd.print(' ');
  }
  
  // Calculate the flow count every 4 timer periods
  if (timer_flag == FLOW_TIMER_END){
  Serial.println(FLOW_count);
  rate = 62*FLOW_count/1000;
  timer_flag =0;  //reset timer_flag 
  FLOW_count = 0; //reset the counter
  }
}

void setup() {
  pinMode(PWM_OUT, OUTPUT);
  pinMode(PULSE_OUT, OUTPUT);
  pinMode(FLOW_IN0, INPUT_PULLUP);
  pinMode(OUTPUT_DET, INPUT_PULLUP);
  pinMode(OUTPUT_LED, OUTPUT);

  //initate IO ports
  digitalWrite(PWM_OUT, HIGH);
  digitalWrite(PULSE_OUT, HIGH);
  digitalWrite(OUTPUT_LED, LOW);
    
  // init LCD
  lcd.begin();
  
  lcd.createChar(0, customChar);
  lcd.createChar(1, heart);
  lcd.createChar(2, flash);
  lcd.createChar(3, danger);
  
    lcd.setCursor(0,0);
  // Print a message to the LCD.
  lcd.print("  HEART SIMULATOR");
  lcd.setCursor(0,1);
  lcd.print("OUTPUT:");
  lcd.setCursor(0,2);
  lcd.print("FLOW:     PPM:");

  Serial.begin(115200); //  setup serial 
}

void loop() {
  if(mytimer.repeat()){
    flow_rate();
    if (digitalRead(OUTPUT_DET) == 0){
      if (index == 0){
        lcd.setCursor(11,1);
        lcd.write(1);
        index++;
      }
      else{
        lcd.setCursor(11,1);
        lcd.write(2);
        index++;
      }
      if (index ==2) index =0; //reset the index
    }
    else {
      lcd.setCursor(11,1);
      lcd.print(' ');   
    }
  }
 
  attachInterrupt(digitalPinToInterrupt(FLOW_IN0), counting, FALLING); //Enable the interrupt for flow detection
  temp = analogRead(TEMP_IN0)*50/1024*10;   // read temp sensor1
  PWM_val = analogRead(PWM_PIN);  // read PWM VR input
  PULSE_val = analogRead(PULSE_PIN); // read Pulse VR input

  //Debug values
  Serial.print("PWM = "); 
  Serial.print(PWM_val);
  Serial.print(" PULSE = ");
  Serial.print(PULSE_val);
  Serial.print(" TEMP = "); 
  Serial.println(temp);

  // Give the pulse output at 50% duty cycle
  // Map to min 0.667Hz (=40ppm,Period=1.5s) to max 3Hz (=180ppm,Period=0.333s)
  // Calculate the half period of the pulse
  pulse_half_period = map(PULSE_val,0,1023,166,750);
    
  //OUTPUT control
  if (digitalRead(OUTPUT_DET) == 0) {
    digitalWrite(OUTPUT_LED,HIGH); //Output LED turns high
    lcd.setCursor(7,1);
    lcd.print("ON ");
    //Output is enabled
    //PWM_OUT port is only for reference
    analogWrite(PWM_OUT, map(PWM_val,0,1023,0,255));  // write PWM value to PWM port, highest speed when output = 0
    analogWrite(PULSE_OUT,map(PWM_val,0,1023,0,255)); // write PWM value to PULSE_OUT port, highest speed when output = 0
    delay(pulse_half_period); 
    //PWM_val = analogRead(PWM_PIN);  // read PWM VR input
    //PULSE_val = analogRead(PULSE_PIN); // read Pulse VR input
    digitalWrite(PULSE_OUT, HIGH); // Write high to the PULSE PORT to stop in the half cycle
    delay(pulse_half_period);
  }
  else{
    digitalWrite(OUTPUT_LED,LOW);
    lcd.setCursor(7,1);
    lcd.print("OFF");
    //Turn off all the outputs otherwise
    digitalWrite(PWM_OUT, HIGH);
    digitalWrite(PULSE_OUT, HIGH);
  }
}
