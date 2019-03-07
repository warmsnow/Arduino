//==============================================================================
// LCD section
//==============================================================================
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
LiquidCrystal_I2C lcd(0x27,16,2);

// LCD Initialization
void lcd_init(void)
{
  lcd.init();  //invoking initialized function of LCD in LiquidCrystal_I2C.h  
  delay(10);  //delaying for 10 millisecond
  lcd.backlight(); //open backlight of LCD1602
  lcd.clear();    //clear screen
}

int car_state = 0;

void display_car_state(void)
{
  lcd.setCursor(0, 1);
   switch(car_state)
   {
     case 1:lcd.print("Go   "); break;
     case 2:lcd.print("Back "); break;
     case 3:lcd.print("Left "); break;
     case 4:lcd.print("Right"); break;
     case 5:lcd.print("Stop "); break;
     default:                   break;
   }
}  

//==============================================================================
// IR Remote Control section
//==============================================================================
#include "IRremote.h"

// IR Codes
#define IR_Go      0x00ff629d
#define IR_Back    0x00ffa857
#define IR_Left    0x00ff22dd
#define IR_Right   0x00ffc23d
#define IR_Stop    0x00ff02fd
#define IR_ESC     0x00ff52ad
#define IR_Speed_UP     0x00ffb04f  // increasing speed
#define IR_Speed_DOWN   0x00ff30cf  // decreasing speed

int IR_RCVR_PIN = 12;           // Set pin 12 for IR Receiver

IRrecv ir_rcvr(IR_RCVR_PIN);
decode_results ir_code;

// IR Remote Control Mode
void IR_Control(void) {
    unsigned long Key;
    lcd.setCursor(0,0);
    lcd.print("IR Remote Ctrl");
    while(Key!=IR_ESC) {
      
        if (ir_rcvr.decode(&ir_code)) {
            Key = ir_code.value;
            switch (Key) {
                case IR_Go         : move_forward(); break;
                case IR_Back       : move_back();    break;
                case IR_Left       : move_left();    break;
                case IR_Right      : move_right();   break;
                case IR_Stop       : stop();         break;
                case IR_Speed_UP   : SpeedUp();      break;
                case IR_Speed_DOWN : SpeedDown();    break;
                default            :                 break;   
            }
        ir_rcvr.resume();
        }
    }
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("IR Remote Ctrl");
    stop();
}

//==============================================================================
// Motor Control section
//==============================================================================
// Motors
int pinLB = 2;                  // Left-Back motor, pin 2
int pinLF = 4;                  // Left-Forward motor, pin 4
int pinRB = 7;                  // Right-Back motor, pin 7 
int pinRF = 8;                  // Right-Forward motor, pin 8 
#define Lpwm_pin  5             // adjusting left speed 
#define Rpwm_pin  10            // adjusting right speed
unsigned char Lpwm_val = 200;   // Left motors speed
unsigned char Rpwm_val = 200;   // Right motors speed

// Config motors
void motor_config(void) {
    pinMode(pinLB,OUTPUT);
    pinMode(pinLF,OUTPUT);
    pinMode(pinRB,OUTPUT);
    pinMode(pinRF,OUTPUT);
    pinMode(Lpwm_pin,OUTPUT);
    pinMode(Rpwm_pin,OUTPUT);  
}

// Move and Stop MBot
void move_forward() {
    digitalWrite(pinRB,LOW);
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,LOW);
    digitalWrite(pinLF,HIGH); 
    car_state = 1;
    display_car_state();
}

void move_back() {
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,LOW);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,LOW);
    car_state = 2;
    display_car_state();
}

void move_left() {
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinRF,LOW );
    digitalWrite(pinLB,LOW);
    digitalWrite(pinLF,HIGH);
    car_state = 3;
    display_car_state();
}

void move_right() {
    digitalWrite(pinRB,LOW);
    digitalWrite(pinRF,HIGH);
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinLF,LOW);
    car_state = 4;
    display_car_state();
}

void stop() {
    digitalWrite(pinLB,HIGH);
    digitalWrite(pinRB,HIGH);
    digitalWrite(pinLF,HIGH);
    digitalWrite(pinRF,HIGH);
    car_state = 5;
    display_car_state();
}

// Set and adjust speed
void Set_Speed(unsigned char LM_speed, unsigned char RM_speed) {
    analogWrite(Lpwm_pin, LM_speed);
    analogWrite(Rpwm_pin, RM_speed);
    //Display_Speed(LM_speed);
}

void SpeedUp(void) {
    if (Rpwm_val+10 <= 300 && Lpwm_val+10 <= 300) {
        Rpwm_val+=10;
        Lpwm_val+=10;
        Set_Speed(Lpwm_val, Rpwm_val);
        Display_Speed(Lpwm_val);
    }
}

void SpeedDown(void) {
    if (Rpwm_val-10 >= 0 && Lpwm_val-10 >= 0) {
        Rpwm_val-=10;
        Lpwm_val-=10;
        Set_Speed(Lpwm_val, Rpwm_val);
        Display_Speed(Lpwm_val);
    }
}

void Display_Speed(unsigned char V)
{
     lcd.setCursor(11, 1);
     lcd.print("V=    ");
     lcd.setCursor(13, 1);
     lcd.print(V,DEC);
}

////////////////////////////////////////////////////////////////////////////////
// MAIN SECTION
////////////////////////////////////////////////////////////////////////////////
void setup() {
    motor_config();
    Set_Speed(Lpwm_val, Rpwm_val);  // Set initial speed
    ir_rcvr.enableIRIn();           // Enable IR remote controller
    lcd_init();
    Display_Speed(Lpwm_val);
    stop();                         // Stop motors
}

void loop() {
    IR_Control();
    delay(10);
}
