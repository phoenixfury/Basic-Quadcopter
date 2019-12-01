//Declairing global variables
#include <Wire.h>
//pid gains
float pitch_p_gain = 1.0 ;
float pitch_d_gain = 16 ;
float pitch_i_gain = 0.05 ;
int pitch_max_output = 400 ;

float roll_p_gain = pitch_p_gain;
float roll_d_gain = pitch_d_gain;
float roll_i_gain = pitch_i_gain;
int roll_max_output = 400 ;

float yaw_p_gain = 2.5 ;
float yaw_d_gain = 0.0 ;
float yaw_i_gain = 0.03;
int yaw_max_output = 400 ;

//Reciever variables
byte last_channel_1,last_channel_2,last_channel_3,last_channel_4;
unsigned long current_time,timer1,timer2,timer3,timer4,x;
volatile int reciever_channel_1,reciever_channel_2,reciever_channel_3,reciever_channel_4;

//MPU 6050 Variables
long acc_x,acc_y,acc_z,acc_total_vec;
float gyro_x,gyro_y,gyro_z,gyro_x_cal,gyro_y_cal,gyro_z_cal,gyro_pitch,gyro_roll,gyro_yaw;   int cal_int;
float roll_angle,pitch_angle,acc_roll,acc_pitch;
int pitch_angle_output,roll_angle_output; boolean set_gyro_angles;
boolean auto_level = true;                 //Auto level on (true) or off (false)
float roll_level_adjust, pitch_level_adjust; //auto level variables

//safety variable
int start;

//battery variable
int battery_voltage;
int  counter; //for testing
//PID Variables
float pitch_error,pitch_setpoint,pitch_i_total,pitch_last_error,pid_pitch_output;
float roll_error,roll_setpoint,roll_i_total,roll_last_error,pid_roll_output;
float yaw_error,yaw_setpoint,yaw_i_total,yaw_last_error,pid_yaw_output;
//Escs Variables
unsigned long loop_timer,esc_loop_timer,timer_channel_1,timer_channel_2,timer_channel_3,timer_channel_4;
int throttle;
int esc_1,esc_2,esc_3,esc_4;


void setup() {
 //Pin inturruption initialization
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT1 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT2 (digital input 11)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT4);                           // set PCINT3 (digital input 12)to trigger an interrupt on state change
 //Serial.begin(57600); 
 //MPU Set up and initialization & Gyro calibration
  //TWBR=12; //Setting the I2C speed to 400 kHz
 
 Wire.begin();
  Wire.setClock(400000);
  //TWBR=12;
 MPU_Set();                                                         //Initializing the MPU
 DDRB |= B00000010;                                                  //declare pin 9 as output
 DDRD |= B11110000;                                                  //declare pins 4,5,6,7 as outputs
 digitalWrite(9,HIGH);                                                    //Turn on the warning led.
 for (cal_int = 0; cal_int <250 ; cal_int ++){                           //Wait 5 seconds to give the mpu time to start
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
  //starting the calibration of the gyro
  for(cal_int=0;cal_int <750;cal_int++) 
    {
      if(cal_int % 15 == 0)digitalWrite(9, !digitalRead(9));
     Gyro_data();
     gyro_x_cal += gyro_x;                                              
     gyro_y_cal += gyro_y;                                              
     gyro_z_cal += gyro_z;
     PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
     delayMicroseconds(1000);                                                //Wait 1000us.
     PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
     delayMicroseconds(3000);                                                //Wait 3000us.
    } 
    //getting the calibration average
    gyro_x_cal = gyro_x_cal/2000;
    gyro_y_cal = gyro_y_cal/2000;
    gyro_z_cal = gyro_z_cal/2000;

    start = 0;
  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 72) * 1.2317;

  loop_timer = micros();                                                    //Set the timer for the next loop.
 Serial.begin(115200);
  //When everything is done, turn off the led.
  digitalWrite(9,LOW);                                                     //Turn off the warning led.
}

void loop() {
  //complementary filter to reduce the gyro's noise
  gyro_pitch = (gyro_pitch *0.75) + ((gyro_x/65.5) *0.25);
  gyro_roll = (gyro_roll *0.75) + ((gyro_y/65.5) *0.25);
  gyro_yaw = (gyro_yaw *0.75) + ((gyro_z/65.5) *0.25);

  //calculating the gyro angles
  pitch_angle += gyro_x * 0.0000611;          // angle per 250 loops is angle * 1/250/65.5
  roll_angle += gyro_y * 0.0000611;

  pitch_angle -= roll_angle * sin(gyro_z * 0.000001066); //If the MPU has yawed transfer the roll angle to the pitch angel.
  roll_angle += pitch_angle * sin(gyro_z * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angel.
                                                        //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  //calculating the acc angles
  acc_total_vec = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  
  if(abs(acc_y) < acc_total_vec){ //to make sure the asin doesn't produce a nan function
  acc_pitch = asin((float)acc_y/acc_total_vec)* 57.296;  //asin produces an angle in radians
  }
  
  if(abs(acc_y) < acc_total_vec){ 
  acc_roll = asin((float)acc_x/acc_total_vec)* -57.296;  //57.296 = 1 / (3.142 / 180)
  }
  //acc_roll -=  -1.0;                                           //roll angle has an offset of -1 degree
  if(set_gyro_angles){                                                 //To make sure the gyro angle is the same as the acc at first start
    pitch_angle = pitch_angle * 0.9996 + acc_pitch * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    roll_angle = roll_angle * 0.9996 + acc_roll * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    pitch_angle = acc_pitch;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    roll_angle = acc_roll;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the MPU started flag
  }
  //Getting the final angles using a complementary filter to reduce the noise
  pitch_angle_output = pitch_angle_output *0.5 + pitch_angle *0.5;
  roll_angle_output = roll_angle_output *0.5 + roll_angle *0.5;
  pitch_level_adjust = pitch_angle_output * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = roll_angle_output * 15;                                      //Calculate the roll angle correction
  
  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  } 
 //Print_data();
 //For starting the motors: throttle low and yaw right (step 1).
  if(reciever_channel_3 < 1050 && reciever_channel_4 > 1950)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if(start == 1 && reciever_channel_3 < 1050 && 1450 < reciever_channel_4 && reciever_channel_4 < 1570
  ){
    start = 2;
    //reseting the pid variables for a clean start
    roll_i_total=0;
    roll_last_error=0;
    pitch_i_total=0;
    pitch_last_error=0;
    yaw_i_total=0;
    yaw_last_error=0;
  }

if(start == 2 && reciever_channel_3 < 1050 && reciever_channel_4 < 1050)start = 0;

//The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second  (500-32)/3 = 156d/s ).
  roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(reciever_channel_1 > 1516)roll_setpoint = reciever_channel_1 - 1516;
  else if(reciever_channel_1 < 1484)roll_setpoint = reciever_channel_1 - 1484;

  roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-32)/3 = 156d/s ).
  pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(reciever_channel_2> 1516)pitch_setpoint = reciever_channel_2 - 1516;
  else if(reciever_channel_2 < 1484)pitch_setpoint = reciever_channel_2 - 1484;

  pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-32)/3 = 156d/s ).
  yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(reciever_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(reciever_channel_4 > 1560)yaw_setpoint = (reciever_channel_4 - 1516)/3.0;
    else if(reciever_channel_4 < 1484)yaw_setpoint = (reciever_channel_4 - 1484)/3.0;
  }
 calculate_pid();
 
  
  //loop_timer = micros();
Gyro_data(); Acc_data();
 //x=micros()-loop_timer;//We wait until 4000us are passed.
 //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  //battery_voltage = battery_voltage * 0.92 + (analogRead(0)+ 72 ) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1080 && battery_voltage > 1020)digitalWrite(9, HIGH);

  throttle = reciever_channel_3; //base throttle

  if(start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_pitch_output + pid_roll_output - pid_yaw_output; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle+ pid_pitch_output + pid_roll_output + pid_yaw_output; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle+ pid_pitch_output - pid_roll_output - pid_yaw_output; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle- pid_pitch_output - pid_roll_output + pid_yaw_output; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 1040){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3400);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3400);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3400);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3400);           //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1150) esc_1 = 1150;                                         //Keep the motors running.
    if (esc_2 < 1150) esc_2 = 1150;                                         //Keep the motors running.
    if (esc_3 < 1150) esc_3 = 1150;                                         //Keep the motors running.
    if (esc_4 < 1150) esc_4 = 1150;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for esc4.
  }
  if(micros() - loop_timer > 4050)digitalWrite(9, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  while(micros() - loop_timer < 4000);  
  x=micros()-loop_timer;//We wait until 4000us are passed.
  
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
 Print_data();
}
 ISR(PCINT0_vect){
  current_time=micros();
  //channel 1 input "roll" pin 8
  if( PINB & B00000001){
    if(last_channel_1 == 0){
      last_channel_1 = 1;
      timer1 = current_time;
    }
  }
  else if ( last_channel_1 == 1){
    last_channel_1 = 0;
    reciever_channel_1 = current_time - timer1;
  }
  //channel 2 input "pitch" pin 10
  if( PINB & B00000100){
    if(last_channel_2 == 0){
      last_channel_2 = 1;
      timer2 = current_time;
    }
  }
  else if ( last_channel_2 == 1){
    last_channel_2 = 0;
    reciever_channel_2 = current_time - timer2;
  }
  //channel 3 input "throttle" pin 11
  if( PINB & B00001000){
    if(last_channel_3 == 0){
      last_channel_3 = 1;
      timer3 = current_time;
    }
  }
  else if ( last_channel_3 == 1){
    last_channel_3 = 0;
    reciever_channel_3 = current_time - timer3;
  }
  //channel 4 input "yaw" pin 12
  if( PINB & B00010000){
    if(last_channel_4 == 0){
      last_channel_4 = 1;
      timer4 = current_time;
    }
  }
  else if ( last_channel_4 == 1){
    last_channel_4 = 0;
    reciever_channel_4 = current_time - timer4;
  } 
}

//Mpu initialization
void MPU_Set()
{
  Wire.beginTransmission(0b1101000);                          //Start communication with the MPU
  Wire.write(0x6B);                                           //configuring the power management register
  Wire.write(0b00000000);                                     //set the power management to 0 to start the mpu
  Wire.endTransmission();                                     //end transmittion
  Wire.beginTransmission(0b1101000);                          
  Wire.write(0x1B);                                           //Configuring the gyro
  Wire.write(0b00001000);                                     //setting the full scale to 500 degrees/sec
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);                                           //configuring the accelerometer
  Wire.write(0b00010000);                                     //setting the full scale to 8g
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);                        
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();
}
void Gyro_data()
{
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);         //writing to the gyro register
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  while (Wire.available()<6);
  gyro_x=Wire.read()<<8|Wire.read();
  gyro_y=Wire.read()<<8|Wire.read();
  gyro_z=Wire.read()<<8|Wire.read();
  gyro_z=gyro_z*-1;
  if(cal_int == 2000){
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
  }
}
void Acc_data(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);         //writing to the accelerometer register
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  while (Wire.available()<6);
  acc_x=Wire.read()<<8|Wire.read();
  acc_y=Wire.read()<<8|Wire.read();
  acc_z=Wire.read()<<8|Wire.read();
}

void calculate_pid()
{
  //pitch calculations
 pitch_error = gyro_pitch - pitch_setpoint;
 pitch_i_total = pitch_i_total + pitch_i_gain * pitch_error;
 if(pitch_i_total>pitch_max_output) pitch_i_total = pitch_max_output;
 else if(pitch_i_total<pitch_max_output*-1) pitch_i_total = pitch_max_output *-1;

 pid_pitch_output = (pitch_error * pitch_p_gain) + pitch_i_total + (pitch_d_gain * (pitch_error - pitch_last_error));
 if(pid_pitch_output>pitch_max_output) pid_pitch_output = pitch_max_output;
 else if(pid_pitch_output<pitch_max_output*-1) pid_pitch_output = pitch_max_output *-1;
 pitch_last_error = pitch_error ;
 
//roll calculations
 roll_error = gyro_roll - roll_setpoint;
 roll_i_total = roll_i_total + roll_i_gain * roll_error;
 if(roll_i_total>roll_max_output) roll_i_total = roll_max_output;
 else if(roll_i_total<roll_max_output*-1) roll_i_total = roll_max_output *-1;

 pid_roll_output = (roll_error * roll_p_gain) + roll_i_total + (roll_d_gain * (roll_error - roll_last_error));
 if(pid_roll_output>roll_max_output) pid_roll_output = roll_max_output;
 else if(pid_roll_output<roll_max_output *-1) pid_roll_output = roll_max_output *-1;
 roll_last_error = roll_error ;

 //yaw calculations
 yaw_error = gyro_yaw - yaw_setpoint;
 yaw_i_total = yaw_i_total + yaw_i_gain * yaw_error;
 if(yaw_i_total>yaw_max_output) yaw_i_total = yaw_max_output;
 else if(yaw_i_total<yaw_max_output*-1) yaw_i_total = yaw_max_output *-1;

 pid_yaw_output = (yaw_error * yaw_p_gain) + yaw_i_total + (yaw_d_gain * (yaw_error - yaw_last_error));
 if(pid_yaw_output>yaw_max_output) pid_yaw_output = yaw_max_output;
 else if(pid_yaw_output<yaw_max_output *-1) pid_yaw_output = yaw_max_output *-1;
 yaw_last_error = yaw_error ;
  
}
void Print_data()
{
  if (counter==0)Serial.print("pitch angle=");
  if (counter==1)Serial.print(x);
    if (counter==2)Serial.print("acc=");
  if (counter==3)Serial.print(esc_2);
  if (counter==4)Serial.print(" roll angle=");
  if (counter==5)Serial.print(pid_roll_output);
  //if (counter==6)Serial.println(start);
  
  if (counter==7)Serial.print("roll");
  if (counter==10) Serial.print(start);
 if (counter==11)Serial.print("yaw");
 if (counter==12)Serial.print(reciever_channel_4);
 if (counter==13)Serial.print("throttle=");
 if (counter==14)Serial.println(reciever_channel_3);
 counter++;
  if (counter==60) counter=0;}
 //if (counter==6)Serial.println(pid_pitch_output);
//if (counter==7)Serial.println(pid_yaw_output);
  /*if (counter==8)Serial.println(gyro_pitch);
  if (counter==9)Serial.println(pitch_angle_output);
  if (counter==10)Serial.println(roll_angle_output);
  if (counter==11)Serial.println(gyro_roll);
  if (counter==12)Serial.println(gyro_yaw); if (counter==13)Serial.print("battery voltage=");
 if (counter==14)Serial.println(battery_voltage);
  counter++;
  if (counter==60) counter=0;
  }*/
  
