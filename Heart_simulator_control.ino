// Version: 3
// By : Henry Chan
// A motor control with flow sensor programme for Heart Simulator
// Two outputs: one is PWM at 490Hz and one is a 50% duty-cycle variable frequency output
// PWM & Pulse output will be activated by a switch
// Flow sensor counter is detected by ext. interrupt input; overall number of counts is added & reset in Timer2
// Display is refreshed regularly inside the timer interrupt to ensure no display corruption every timer period (500ms) using NeoTimer
// A 4 lines x 20 characters LCD module is attached using I2C interface
// LED is to show the output status
// A temp sensor detection is available
// **v2.1
// Replace the delay method by millis method for the output pulse
// This can avoid conflict of neotimer with delay
// **v3
// Use rotatory encoder as inputs

// include the library code:
#include <neotimer.h>
#include <Wire.h>  // Arduino IDE built-in
#include <LiquidCrystal_I2C.h>

#define PWM_A A0      // PWM control A Pin
#define PWM_B 8       // PWM control B Pin
#define PULSE_A A1    // PULSE output control A Pin
#define PULSE_B 7     // PULSE output control B Pin
#define TEMP_IN0 A3   // Temp sensor1
#define PWM_OUT 9     // PWM output port (490Hz)
#define PULSE_OUT 10  // PULSE output port
#define OUTPUT_LED 12 // Output enable LED
#define OUTPUT_DET 5  // Output enable detection
#define FLOW_IN0 3    // FLOW sensor input1 as EXT INT
#define DISPLAY_TIMER 500  // DISPLAY_TIMER in milli-seconds
#define debug 2

// Set the LCD I2C address, lines and rows
LiquidCrystal_I2C lcd(0x27,20,4); //Address: 0x27 for PCF8754; 0x3F for PCF8754A
Neotimer mytimer = Neotimer(DISPLAY_TIMER); // Set timer interval

unsigned long time_1 = 0;
int PWM_val = 125;    // PWM variable
int PULSE_val = 60;  // Pulse variable
int temp = 25;      // temp1 value
int pulse_half_period = 300;  //Pulse output half period
boolean pulse_output = HIGH;  //PULSE_OUTPUT port value
boolean pulse_flag = HIGH;    //A flag to control the pulse output
int FLOW_count = 0; // Flow rate count
float rate = 0;       //Flow rate
byte timer_flag = 0;
byte display_timer_flag = 0;

byte aState;
byte aLastState;
byte aState1;
byte aLastState1;
boolean repeated = false;
boolean repeated1 = false;    

byte degree[] = {0x06,0x09,0x09,0x06,0x00,0x00,0x00,0x00}; //Character for degree symbol
byte heart[] = {0x0A,0x1F,0x1F,0x1F,0x1F,0x0E,0x04,0x00};// Heart symbol
byte flash[] = {0x00,0x00,0x08,0x15,0x02,0x00,0x00,0x00}; // Flash symbol
byte danger[]= {0x06,0x0F,0x0F,0x0F,0x06,0x00,0x06,0x06}; // Danger symbol

// Counting the flow rate
void counting() {
  FLOW_count ++;
}

// Routine to display
void display_rate() {  
  //Display the icon here
  //digitalWrite(debug, digitalRead(debug) ^ 1);  //Turns debug IO ON and OFF
  if (digitalRead(OUTPUT_DET) == 0){
      if (display_timer_flag == 0){
        lcd.setCursor(11,1);
        lcd.write(1);
        display_timer_flag++;
      }
      else{
        lcd.setCursor(11,1);
        lcd.write(2);
        display_timer_flag++;
      }
      if (display_timer_flag ==2) display_timer_flag =0; //reset the flag
    }
    else {
      lcd.setCursor(11,1);
      lcd.print(' ');   
    }
  
  // Display all the values at timer interal
  lcd.setCursor(0, 2);
  lcd.print("FLOW:    L PPM:   "); //clear the values and refresh the titles
  lcd.setCursor(5, 2);
  lcd.print(rate);
  lcd.setCursor(15, 2);
  lcd.print(PULSE_val);
  lcd.setCursor(0, 3);
  lcd.print("TEMP:   C");
  lcd.setCursor(5, 3);
  lcd.print(temp);
  lcd.setCursor(7, 3);
  lcd.write(0);
  if (temp > 50){
    lcd.setCursor(11,3);
    lcd.write(3);
  }
  else{
    lcd.setCursor(11,3);
    lcd.print(' ');
  }
  Serial.println(rate);
  //digitalWrite(debug,LOW);
  //Serial.println(rate);
}

ISR(TIMER2_COMPA_vect)      //timer2 interrupt 33Hz
{
  //digitalWrite(debug,HIGH);
  timer_flag++;
  
  //Collect the flow count
  //Count = 120 @ 2sec
  if (timer_flag == 121){
  digitalWrite(debug, digitalRead(debug) ^ 1);  //Turns debug IO ON and OFF
  rate = float(FLOW_count)*0.062;
  FLOW_count = 0; //reset the counter
  }
  if (timer_flag > 121) timer_flag = 0; //Reset timer_flag
  //digitalWrite(debug,LOW);
}

void setup() {
  pinMode(PWM_OUT, OUTPUT);
  pinMode(PULSE_OUT, OUTPUT);
  pinMode(FLOW_IN0, INPUT_PULLUP);
  pinMode(OUTPUT_DET, INPUT_PULLUP);
  pinMode(OUTPUT_LED, OUTPUT);
  pinMode(debug, OUTPUT);
  pinMode(PULSE_A, INPUT_PULLUP);
  pinMode(PULSE_B, INPUT_PULLUP);
  pinMode(PWM_A, INPUT_PULLUP);
  pinMode(PWM_B, INPUT_PULLUP);

  //initate IO ports
  digitalWrite(PWM_OUT, HIGH);
  digitalWrite(PULSE_OUT, HIGH);
  digitalWrite(OUTPUT_LED, LOW);
  digitalWrite(debug, LOW);

  aLastState = digitalRead(PULSE_A);
  aLastState1 = digitalRead(PWM_A);
     
// initialize timer2 
  noInterrupts();           // disable all interrupts
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  // Value = 16M/desired frequency/Prescaler
  OCR2A = 255;// Timer period = ((reg+1)*prescaler)/16M (reg must be <255)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS22 bit for 1024 prescaler
  TCCR2B |= (1 << CS22)|(1 << CS21)|(1<<CS20);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  interrupts();             // enable all interrupts

  // init LCD
  lcd.begin();
  
  lcd.createChar(0, degree);
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
    display_rate();
  }
  attachInterrupt(digitalPinToInterrupt(FLOW_IN0), counting, FALLING); //Enable the interrupt for flow detection
  // read temp sensor1, temp = ADC *5/1024 * 100 (as 1mV = 0.1degC)
  // Calculation in float and then change to integer
  temp=int(analogRead(TEMP_IN0)*5/10.24);   
  
  aState = digitalRead(PULSE_A); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){     
   // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(PULSE_B) != aState) { 
      if ((PULSE_val<140)&&(repeated==true)) {
        PULSE_val++;
        //Serial.print("Position: ");
        //Serial.println(PULSE_val);        
      }
    } 
    else {
      if ((PULSE_val>40)&&(repeated==true)) {
        PULSE_val--;
        //Serial.print("Position: ");
        //Serial.println(PULSE_val);
      }
    }
    repeated = !repeated;
  } 
  aLastState = aState; // Updates the previous state of the outputA with the current state

  aState1 = digitalRead(PWM_A); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState1 != aLastState1){     
   // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(PWM_B) != aState1) { 
      if ((PWM_val>0)&&(repeated1==true)) {
        PWM_val -=5;
        //Serial.print("PWM: ");
        //Serial.println(PWM_val);
      }
    } 
    else {
      if ((PWM_val<255)&&(repeated1==true)) {
        PWM_val +=5;
        //Serial.print("PWM: ");
        //Serial.println(PWM_val);
      }
    }
    repeated1 = !repeated1;
  } 
  aLastState1 = aState1; // Updates the previous state of the outputA with the current state
 
  
  //PWM_val = analogRead(PWM_A);  // read PWM VR input
  //PULSE_val = analogRead(PULSE_PIN); // read Pulse VR input
  

  // Give the pulse output at 50% duty cycle
  // Map to min 0.667Hz (=40ppm,Period=1.5s) to max 3Hz (=180ppm,Period=0.333s)
  // Calculate the half period of the pulse
  //pulse_half_period = map(PULSE_val,0,1023,166,750);
  pulse_half_period = 30000/PULSE_val;
    
  //OUTPUT control
  if (digitalRead(OUTPUT_DET) == 0) {
    digitalWrite(OUTPUT_LED,HIGH); //Output LED turns high
    lcd.setCursor(7,1);
    lcd.print("ON ");
    //Output is enabled
    //PWM_OUT port is only for reference
    analogWrite(PWM_OUT, PWM_val);  // write PWM value to PWM port, highest speed when output = 0
    
    if(millis() >= time_1 + pulse_half_period){
      time_1 += pulse_half_period;
      pulse_flag = !pulse_flag;
      if (pulse_flag == HIGH){
        analogWrite(PULSE_OUT,PWM_val); // write PWM value to PULSE_OUT port, highest speed when output = 0    
      }
      else {
        digitalWrite(PULSE_OUT, HIGH); // Write high to the PULSE PORT to stop in the half cycle
      }
    }
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
