//This is my experimental code for driving the rover with RC signal
//encoder is used to initializing swashplate location before engine starting up


//***Hardware Configuration***//
//Stepper is 200 FULL step/rev
//Stepper is set to run on 12V source
//Stepper DRIVER is set to half-step stepping and max 2.5A
//!!!hence, to reach 1 REV, it takes 400 steps
//INPUT use pwm pulse of (900,2100) to map swashplate angle of (0deg, 10deg)
//RX input is divided by 5 to eliminate jitter..

//***Nomenclature***//
//ang_ : indicates variables related to swashplate ANGLE
//enc_ : indicates variable related to encoder location
//pos_ : indicates variables related to stepper motor position


//include libraries
#include "AccelStepper.h"//this library is modified to use microsecond scale for higher speed vs original ms scale speed
#include "PinChangeInt.h"


//***Input pin groups***//
//RC receiver group
#define RX0 18   //set D18 as pwm input pin ... left track
#define RX1 19   //set D19 as pwm input pin ... right track
#define RX2 20   //set D20 as pwm input pin ... emergency switch
int RX0_new_flag = 0; // input update flag
int RX1_new_flag = 0; //input update flag
char RX_inputs[4] = {RX0, RX1, RX2}; //wrap into array for initialization

//***Output pin groups***//
//Stepper motor group
#define STEPPER0_CLK   22  //set D22 as stepper 0 CLK output... PinName PA0
#define STEPPER1_CLK   23  //set D23 as stepper 1 CLK output... PinName PA1
#define STEPPER0_DIR   24  //set D24 as stepper 0 DIR output... PinName PA2
#define STEPPER1_DIR   25  //set D25 as stepper 1 DIR output... PinName PA3
#define STEPPER0ENABLE 26  //set D26 as Enable PIN for controller 0... PinName PA4
#define STEPPER1ENABLE 27  //set D27 as ENABLE PIN for controller 1... PinName PA5
#define step_delay 600 //set the stepping delay (microseconds)
#define enc_hyst 7
//Emergency switch
#define ES_pin 3 //set pin D3 as emergency switch output pinf

//***Variables group***//
//RX related variables
volatile int prev_time0 = 0;
volatile int prev_time1 = 0;
volatile int prev_time2 = 0;
int r0 = 0; //left track input reading
int r1 = 0; //right track input reading
int es_value = 0; //emergency switch input reading

//Stepper parameters
#define stepper_SPD 2000  //2000 pulse per second
#define stepper_ACC 3000  //3000 pps^2
int left_target = 0; //initialize left track target
int right_target = 0;


//general parameters
volatile int i;
volatile int j;
volatile int k;
int ini_flag = 1; //flag to check initialization on swashplate position

//encoder parameters
char EncReg[10] ={0,0,0,0,0,0,0,0,0,0};
#define CSpin0    42 //PL7
#define CLKpin0   44 //PL5
#define DATA_pin0 46 //PL3
#define CSpin1    43 //PL6
#define CLKpin1   45 //PL4
#define DATA_pin1 47 //PL2
#define enc_left_center  533                     //left swashplate center location
#define enc_left_min     (enc_left_center + 30)  //left swashplate allowed min angle
#define enc_left_max     (enc_left_center - 30)  //left swashplate allowed max angle
#define enc_right_center 837                     //right swashplate center location (edited 1-15-16 by Brandon Jameson)
#define enc_right_min    (enc_right_center - 30) //right swashplate allowed min angle
#define enc_right_max    (enc_right_center + 30) //right swashplate allowed max angle
int enc_left_cur = 0;     //current left encoder reading
int enc_right_cur = 0;    //current right encoder  reading
int mask1 = _BV(3);       //encoder interpretation mask
int mask2 = _BV(2);       //encoder interpretation mask
char buffer_usart [50];   //used for serial printf/debug
int deg_to_enc = 1024/360;// ratio from degree to encoder location.. linear.. use equation format to not round decimal point. yet

//stepper - encoder loop parameters.. this loop includes a feedforward and a PI controller
int enc_left_target = 0;  //desired left encoder location .. translated from desired swashplate angle
int enc_right_target = 0; //desired right encoder location .. translated from desired swashplate angle
int enc_to_stepper = 149; //number of step per encoder location change
int pos_left_ff = 0;  //calculated left FF for desired stepper position
int pos_right_ff = 0; //calculated right FF for desired stepper position

//define one step cw/ccw output on DIRs and CLKs
//move  signal outputs to port manipulation method for less delay   9.6us vs 2.12us ..measured on o-scope
void FWD0()
{
    PORTA |= (1 << PA2); // set direction HIGH
    PORTA ^= (1 << PA0); // set clock LOW
    delayMicroseconds(step_delay);
    PORTA ^= (1 << PA0);  // set clock HIGH
}
void REV0()
{
    PORTA &= ~(1 << PA2); // set direction LOW
    PORTA ^= (1 << PA0); // set clock LOW
    delayMicroseconds(step_delay);
    PORTA ^= (1 << PA0);  // set clock HIGH
}

void FWD1()
{
    PORTA |= (1 << PA3); // set direction LOW
    PORTA ^= (1 << PA1); // set clock LOW
    delayMicroseconds(step_delay);
    PORTA ^= (1 << PA1);  // set clock HIGH
}
void REV1()
{
    PORTA &= ~(1 << PA3); // set direction LOW
    PORTA ^= (1 << PA1); // set clock LOW
    delayMicroseconds(step_delay);
    PORTA ^= (1 << PA1);  // set clock HIGH
}

void center_steppers()
{
    signed long diff_0, diff_1 = 0;
    do {
        get_encoders();
        diff_0 = enc_left_cur - enc_left_center;
        diff_1 = enc_right_cur - enc_right_center;

        if(diff_0 < -enc_hyst) {
            FWD0();
        }
        else if(diff_0 > enc_hyst) {
            REV0();
        }
        if(diff_1 < -enc_hyst) {
            FWD1();
        }
        else if(diff_1 > enc_hyst) {
            REV1();
        }
    } while((diff_0 < -enc_hyst || diff_0 > enc_hyst) || (diff_1 < -enc_hyst || diff_1 > enc_hyst));
    enc_right_target = enc_right_center;
    enc_left_target = enc_left_center;
}


//GET encoder reading. SSI interface is defined on datasheet. direct port manipulation is used to speed up interfacing time
void get_encoders() {

    //Read encoder
    PORTL ^= (1<<PL7)|(1<<PL6);
    for (i=0; i<10; i++)
    {
        PORTL ^= (1<<PL5)|(1<<PL4);
        //delayMicroseconds(1);
        PORTL |= (1<<PL5)|(1<<PL4);
        delayMicroseconds(1);
        EncReg[i] = PINL;
    }

    for (i=0;i<=5;i++)
    {
        PORTL ^= (1<<PL5)|(1<<PL4);
        PORTL |= (1<<PL5)|(1<<PL4);
    }
    PORTL ^= (1<<PL5)|(1<<PL4);
    PORTL |= (1<<PL5)|(1<<PL7)|(1<<PL4)|(1<<PL6);

    //parse data
    enc_left_cur = 0;//rest string to prevent overflow
    enc_right_cur = 0;//reset string to prevent overflow
    for (i=0;i<=9;i++)
    {
        enc_left_cur |= ((EncReg[i]&mask1)!=0)<<(9-i);
        enc_right_cur |= ((EncReg[i]&mask2)!=0)<<(9-i);
    }
}


void rising0()
{
    attachInterrupt(digitalPinToInterrupt(RX0), &falling0, FALLING);
    prev_time0 = micros();
}


void falling0() {
    attachInterrupt(digitalPinToInterrupt(RX0), &rising0, RISING);
    r0 = micros() - prev_time0;
    RX0_new_flag = 1;

}


void rising1()
{
    attachInterrupt(digitalPinToInterrupt(RX1), &falling1, FALLING);
    prev_time1 = micros();
}


void falling1() {
    attachInterrupt(digitalPinToInterrupt(RX1), &rising1, RISING);
    r1 = micros() - prev_time1;
    RX1_new_flag = 1;
}


void rising2()
{
    attachInterrupt(digitalPinToInterrupt(RX2), &falling2, FALLING);
    prev_time2 = micros();
}


void falling2() {
    attachInterrupt(digitalPinToInterrupt(RX2), &rising2, RISING);
    es_value = micros() - prev_time2;
    //put emergency check into interrupt to maximuze effectiveness
    checkES();
}


void checkES()
{
    // if we get a valid E stop signal (900 - 1400), or an invalid signal ( less than 900 or more than 1900)
    // E stop!
    static int valid_estop = 0;
    //Serial.println(es_value);
    if ((es_value <= 1400 && es_value >= 1000))
    {
        valid_estop += 1;
        if(valid_estop >= 5) {
            digitalWrite(ES_pin, LOW);  // emergency stop triggered. engage ES relay
            //disable both stepper
            digitalWrite(STEPPER0ENABLE, HIGH);
            digitalWrite(STEPPER1ENABLE, HIGH);

            //enter infinity loop and wait for rescue
            while (1)
            {
                //add any signaling here
                //signaling
                Serial.println("Emergency Stop Engaged... waiting for rescue...");
                delay(10000);
            }
        }
    }
    // if we get a valid OK signal, enable steppers
    else
    {
        valid_estop = 0;
    }

}

/**************************************************************
*                       SETUP FUNCTION                        *
**************************************************************/


void setup()
{
    Serial.begin(115200);           // set up Serial library at 57600 bps
    Serial.println("System Initializing... All output DISABLED \n");

    pinMode(ES_pin,OUTPUT);
    digitalWrite(ES_pin, LOW);//engage, cannot start engine before init completed

    //enable stepper pins and all set HIGH- DISABLED
    for (i=22;i<=27;i++)
    {
        pinMode(i,OUTPUT);
        digitalWrite(i,HIGH);
    }

    //set up Encoder IOs
    pinMode(42, OUTPUT);
    pinMode(44, OUTPUT);
    pinMode(43, OUTPUT);
    pinMode(45, OUTPUT);
    pinMode(46, INPUT);
    pinMode(47, INPUT);
    digitalWrite(42, HIGH);
    digitalWrite(44, HIGH);
    digitalWrite(43, HIGH);
    digitalWrite(45, HIGH);

    //initialize RX input pins
    for (i=RX0;i<=RX2;i++)
    {
        pinMode(i,INPUT);
        digitalWrite(i,HIGH);//pull high
    }

    //enable rx input interrupts
    attachInterrupt(digitalPinToInterrupt(RX0), &rising0, RISING);//left track  
    attachInterrupt(digitalPinToInterrupt(RX1), &rising1, RISING);//right track
    attachInterrupt(digitalPinToInterrupt(RX2), &rising2, RISING);//emergency stop

    //***reset/center swashplate angle***//
    Serial.println("Centering Steppers");
    digitalWrite(STEPPER0ENABLE,LOW);
    digitalWrite(STEPPER1ENABLE,LOW);
    // Centering steppers
    center_steppers();
    digitalWrite(ES_pin, HIGH);   // disengage ES relay
    Serial.println("Done");
}


/**************************************************************
*                        LOOP FUNCTION                        *
**************************************************************/

void loop()
{
    // stepper difference to desired
    static signed long diff_0, diff_1 = 0;
    //when both input are updated.. compute new destiation

    if (RX0_new_flag && RX1_new_flag)
    {
        // Move the steppers
        get_encoders();
        //convert rc input of desired angle in pwm to encoder location
        enc_left_target  = map(r0 - 1100, 0, 800, enc_left_center, enc_left_max);
        enc_right_target = map(r1 - 1100, 0, 800, enc_right_center, enc_right_max);
        // Update the difference
        diff_0 = enc_left_cur - enc_left_target;
        diff_1 = enc_right_cur - enc_right_target;
        /*
        Serial.print("Left  Current: ");
        Serial.print(enc_left_cur);
        Serial.print("   Left  Target: ");
        Serial.println(enc_left_target);
        Serial.print("Right Current: ");
        Serial.print(enc_left_cur);
        Serial.print("   Right Target: ");
        Serial.println(enc_left_target);
        */
    }
    if(diff_0 < -enc_hyst) {
        FWD0();
    }
    else if(diff_0 > enc_hyst) {
        REV0();
    }
    if(diff_1 < -enc_hyst) {
        FWD1();
    }
    else if(diff_1 > enc_hyst) {
        REV1();
    }
}
