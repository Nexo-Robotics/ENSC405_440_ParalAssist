#include <Wire.h>
#include <PWM.h>
#include <Servo.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
//#define Wire.write(x) Wire.send(x)
//#define Wire.read() Wire.receive()
#endif


static uint8_t nunchuck_buf[6];   // array to store nunchuck data,

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
  DDRC |= _BV(pwrpin) | _BV(gndpin);
  PORTC &= ~ _BV(gndpin);
  PORTC |=  _BV(pwrpin);
  delay(100);  // wait for things to stabilize
}

// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
static void nunchuck_init(){
  Wire.begin();                // join i2c bus as master
  Wire.beginTransmission(0x52);// transmit to device 0x52
  #if (ARDUINO >= 100)
    Wire.write((uint8_t)0x40);// sends memory address
    Wire.write((uint8_t)0x00);// sends sent a zero.
  #else
    Wire.send((uint8_t)0x40);// sends memory address
    Wire.send((uint8_t)0x00);// sends sent a zero.
  #endif
  Wire.endTransmission();// stop transmitting
}

// Send a request for data to the nunchuck
// was "send_zero()"
static void nunchuck_send_request(){
  Wire.beginTransmission(0x52);// transmit to device 0x52
  #if (ARDUINO >= 100)
    Wire.write((uint8_t)0x00);// sends one byte
  #else
    Wire.send((uint8_t)0x00);// sends one byte
  #endif
  Wire.endTransmission();// stop transmitting
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
static char nunchuk_decode_byte (char x){
  x = (x ^ 0x17) + 0x17;
  return x;
}

// Receive data back from the nunchuck,
// returns 1 on successful read. returns 0 on failure
static int nunchuck_get_data(){
  int cnt = 0;
  Wire.requestFrom (0x52, 6);// request data from nunchuck
  while (Wire.available ()) {
    // receive byte as an integer
    #if (ARDUINO >= 100)
      nunchuck_buf[cnt] = nunchuk_decode_byte( Wire.read() );
    #else
      nunchuck_buf[cnt] = nunchuk_decode_byte( Wire.receive() );
    #endif
    cnt++;
  }
    nunchuck_send_request();  // send request for next data payload
    // If we recieved the 6 bytes, then go print them
    if (cnt >= 5) {
      return 1;   // success
    }
  return 0; //failure
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.  That is why I
// multiply them by 2 * 2
static void nunchuck_print_data(){
  static int i = 0;
  int joy_x_axis = nunchuck_buf[0];
  int joy_y_axis = nunchuck_buf[1];
  int accel_x_axis = nunchuck_buf[2]; // * 2 * 2;
  int accel_y_axis = nunchuck_buf[3]; // * 2 * 2;
  int accel_z_axis = nunchuck_buf[4]; // * 2 * 2;
  int z_button = 0; 
  int c_button = 0;

  // byte nunchuck_buf[5] contains bits for z and c buttons
  // it also contains the least significant bits for the accelerometer data
  // so we have to check each bit of byte outbuf[5]
  if ((nunchuck_buf[5] >> 0) & 1)
    z_button = 1;
  if ((nunchuck_buf[5] >> 1) & 1)
    c_button = 1;

  if ((nunchuck_buf[5] >> 2) & 1)
    accel_x_axis += 1;
  if ((nunchuck_buf[5] >> 3) & 1)
    accel_x_axis += 2;

  if ((nunchuck_buf[5] >> 4) & 1)
    accel_y_axis += 1;
  if ((nunchuck_buf[5] >> 5) & 1)
    accel_y_axis += 2;

  if ((nunchuck_buf[5] >> 6) & 1)
    accel_z_axis += 1;
  if ((nunchuck_buf[5] >> 7) & 1)
    accel_z_axis += 2;

  Serial.print(i, DEC);
  Serial.print("\t");

  Serial.print("joy:");
  Serial.print(joy_x_axis, DEC);
  Serial.print(",");
  Serial.print(joy_y_axis, DEC);
  Serial.print("  \t");

  Serial.print("acc:");
  Serial.print(accel_x_axis, DEC);
  Serial.print(",");
  Serial.print(accel_y_axis, DEC);
  Serial.print(",");
  Serial.print(accel_z_axis, DEC);
  Serial.print("\t");

  Serial.print("but:");
  Serial.print(z_button, DEC);
  Serial.print(",");
  Serial.print(c_button, DEC);

  Serial.print("\r\n");  // newline
  i++;
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_zbutton()
{
  return ((nunchuck_buf[5] >> 0) & 1) ? 0 : 1;  // voodoo
}

// returns cbutton state: 1=pressed, 0=notpressed
static int nunchuck_cbutton()
{
  return ((nunchuck_buf[5] >> 1) & 1) ? 0 : 1;  // voodoo
}

// returns value of x-axis joystick
static int nunchuck_joyx()
{
  return nunchuck_buf[0];
}

// returns value of y-axis joystick
static int nunchuck_joyy()
{
  return nunchuck_buf[1];
}


/* acceleration is not used in the this application
// returns value of x-axis accelerometer
static int nunchuck_accelx(){
  return nunchuck_buf[2];   // FIXME: this leaves out 2-bits of the data
}

// returns value of y-axis accelerometer
static int nunchuck_accely(){
  return nunchuck_buf[3];   // FIXME: this leaves out 2-bits of the data
}

// returns value of z-axis accelerometer
static int nunchuck_accelz(){
  return nunchuck_buf[4];   // FIXME: this leaves out 2-bits of the data
}
*/


///////////// Global Variables Declaration /////////////////
const int state_change_threshold = 5;
const int homing_threshold = 30;
const int joystick_disp_threshold_l = 50;
const int joystick_disp_threshold_h = 200;

// these motor_angle_* variables give software joint limit in each joint. In order to have consistent joint limits 
// the limit values used in comparison with motor_pwm_* should be function of m*_speed and these motor*_angle_* variables
const int motor1_angle_low = 42;
const int motor2_angle_low = 32;
const int motor3_angle_low = 32;
const int motor4_angle_low = 8; 
const int motor1_angle_high = 165;
const int motor2_angle_high = 165;
const int motor3_angle_high = 165;
const int motor4_angle_high = 31; 

const int m1_speed = 2;
const int m2_speed = 2;
const int m3_speed = 2;
const int m4_speed = 4;

int loop_cnt = 0;
byte joyx, joyy, zbut, cbut;
byte accx, accy, accz; // we dont use accelerations but just keeping it
bool state_z = false; // z=0 ==> M1;    z=1 ==> M4;
bool state_c = false; // c=0 ==> M2;    c=1 ==> M3;
bool z_changed = false; // indicate whether it recorded state change before homing
bool c_changed = false; // indicate whether it recorded state change before homing
int z_counter = 0; // counter for the z button. accounts for debouncing
int c_counter = 0; // counter for the c button. accounts for debouncing

int motor_pwm_1 = 100; // set as 0 degree for now
int motor_pwm_2 = 200; // 
int motor_pwm_3 = 120;
int motor_pwm_4 = 7; // 81 for mid point

bool plotter = true;

Servo gripper;
/////////////////////////////////////////////////////////////

void _print(int mappedx, int mappedy, bool plotter) { // used to print the status during debugging process
  if (plotter == false){
    Serial.print("\nZ Button:  ");
    Serial.print(zbut);
    Serial.print(" | C Button:  ");
    Serial.print(cbut);
    Serial.print(" | mappedx:  ");
    Serial.print(mappedx);
    Serial.print(" | mappedy:  ");
    Serial.print(mappedy);
    Serial.print(" | state_z:  ");
    Serial.print(state_z);
    Serial.print(" | state_c:  ");
    Serial.print(state_c);
    Serial.print(" | z_counter: ");
    Serial.print(z_counter);
    Serial.print(" | c_counter: ");
    Serial.print(c_counter);
    Serial.print("\n");
    
      // print x value
    if (mappedx < 50){
      Serial.print("left | ");
    }
    else if (mappedx > 200){
      Serial.print("right | ");
    }
    else{
      Serial.print("dead | ");
    }
    if (state_z == false){
      Serial.print("motor_pwm_1:");
      Serial.print(" ");
      Serial.println(motor_pwm_1);
    }
    else{
      Serial.print("motor_pwm_4:");
      Serial.print(" ");
      Serial.println(motor_pwm_4);
    }

    // print y value
    if (mappedy < 50){
      Serial.print("pull | ");
    }
    else if (mappedy > 200){
      Serial.print("push | ");
    }
    else{
      Serial.print("dead | ");
    }
    if (state_c == 0){
      Serial.print("motor_pwm_2:");
      Serial.print(" ");
      Serial.println(motor_pwm_2);
      Serial.print(" ");
    }
    else{
      Serial.print("motor_pwm_3:");
      Serial.print(" ");
      Serial.println(motor_pwm_3);
      Serial.print(" ");
    }

    Serial.print("\n");
  }
  else{
    // print x value
    Serial.print("z_counter:"); Serial.print(z_counter); Serial.print(", ");
    Serial.print("c_counter:"); Serial.print(c_counter); Serial.print(", ");
    // Serial.print("mappedx:"); Serial.print(mappedx); Serial.print(", ");
    // Serial.print("mappedy:"); Serial.print(mappedy); Serial.print(", ");
    Serial.print("motor_pwm_1:"); Serial.print(motor_pwm_1); Serial.print(", ");
    Serial.print("motor_pwm_2:"); Serial.print(motor_pwm_2); Serial.print(", ");
    Serial.print("motor_pwm_3:"); Serial.print(motor_pwm_3); Serial.print(", ");
    Serial.print("motor_pwm_4:"); Serial.print(motor_pwm_4); Serial.print(", ");
    Serial.println();
  }
}


#define motorPIN1 3 // elbow joint
#define motorPIN2 9 
#define motorPIN3 10
#define motorPIN4 5 // gripper


bool set_pin_frequency(){ // set PWM frequencies in the analog output pins
  bool pin1set = SetPinFrequencySafe(motorPIN1,300);  
  bool pin2set = SetPinFrequencySafe(motorPIN2,300);
  bool pin3set = SetPinFrequencySafe(motorPIN3,300);
  bool pin4set = SetPinFrequency(motorPIN4, 50);

  if(pin1set){
    pinMode(motorPIN1,OUTPUT); // pin 3
    pwmWrite(motorPIN1, motor_pwm_1);
  }
  else {
    Serial.print("pin1set: false\n");
  }

  if(pin2set){
    pinMode(motorPIN2,OUTPUT); // pin 9
    pwmWrite(motorPIN2, motor_pwm_2);
  }
  else {
    Serial.print("pin2set: false\n");
  }

  if(pin3set){
    pinMode(motorPIN3,OUTPUT); // pin 10
    pwmWrite(motorPIN3, motor_pwm_3);
  }
  else {
    Serial.print("pin3set: false\n");
  }

  if(pin4set){
    pinMode(motorPIN4,OUTPUT); // pin 5
    pwmWrite(motorPIN4, motor_pwm_4);
  }
  else {
    Serial.print("pin4set: false\n");
  }

  if (pin1set && pin2set && pin3set && pin4set) {
    return true;
  }
  else{
    return false;
  }
}

void setup() {
  Serial.begin(9600);
  nunchuck_setpowerpins();
  nunchuck_init(); // send the initilization handshake
  if(plotter == false){
    // Serial.print("Wii Nunchuck Ready\n");
  }
  else{
    Serial.println("mappedx mappedy motor_pwm_1 motor_pwm_2 motor_pwm_3 motor_pwm_4");
  }

  InitTimers();                    // won't touch timer0

  bool pins_are_set;
  do{
    pins_are_set = set_pin_frequency();
  } while (pins_are_set != true);
   
}

void loop() {
  if ( loop_cnt > 10 ) { // every 1000 msecs get new data
    loop_cnt = 0;

    nunchuck_get_data(); // read values from the nunchuck

    // check C and Z status
    zbut = nunchuck_zbutton();  //  0 - 1
    cbut = nunchuck_cbutton();  //  0 - 1

    // check Z button
    if (zbut){
      z_counter++; // counter resets when either state changes or HOMING is called
      if (z_counter > homing_threshold){
        // turn on led to indicate possible z state change
        if (plotter== false){
          Serial.print("------------- z homing ready ---------------\n");
        }
      }
      else if (z_counter > state_change_threshold){
        // turn on led to indicate possible homing
        if (plotter == false){
          Serial.print("------------- Z state toggle ready ---------------\n");
        }
      }
    }
    else{
      if((z_counter > homing_threshold) && (c_counter > homing_threshold)){ // homing
        if (plotter == false){
          Serial.print("HOMING\n");
        }
        z_counter = 0;
        c_counter = 0;
      }
      else if (z_counter > state_change_threshold){ // state change
        if (plotter == false){
          Serial.print("------------- toggle z_state ---------------\n");
        }
        state_z = !state_z;
        z_counter = 0;
      }
      else{ // bouncing button
        z_counter = 0; // reset counter 
      }
    }

    // check C button
    if (cbut){
      c_counter++; // counter resets when either state changes or  is called
      if (c_counter > homing_threshold){
        // turn on led to indicate possible c state change
        if (plotter == false){
          Serial.print("------------- c homing ready ---------------\n");
        }
      }
      else if (c_counter > state_change_threshold){
        // turn on led to indicate possible homing
        if (plotter == false){
          Serial.print("------------- c_state toggle ready---------------\n");
        }
      }
    }
    else{
      if((c_counter > homing_threshold) && (z_counter > homing_threshold)){ // homing
        if (plotter ==false) {
          Serial.print("HOMING\n");      
        }
        c_counter = 0;
        z_counter = 0;
      }
      else if (c_counter > state_change_threshold){ // state change
        if (plotter == false){
          Serial.print("------------- toggle c_state ---------------\n");
        }
        state_c = !state_c;
        c_counter = 0;
      }
      else{ // bouncing button
        c_counter = 0; // reset counter 
      }
    }

    // check joystick value and control motor
    joyx = nunchuck_joyx();     //  15 - 221
    joyy = nunchuck_joyy();     //  29 - 229
    int mappedx = map(joyx, 15, 221, 0, 255);
    int mappedy = map(joyy, 15, 221, 0, 255);
    // int motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
    
    _print(mappedx, mappedy, plotter); // third parameter controls the monitor(false)/plotter(true) 
    
    // deadzone checking and motor actuation
    {
      if (mappedx < joystick_disp_threshold_l){ // we consider joystic's x value first
        if (state_z == false){ // control motor1
          if (motor_pwm_1 > motor1_angle_low){
            motor_pwm_1 -= m1_speed;
            pwmWrite(motorPIN1, motor_pwm_1);
          }
        }
        else{ // control motor4
          if (motor_pwm_4 > motor4_angle_low){
            motor_pwm_4 -= m4_speed;
            // motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
            pwmWrite(motorPIN4, motor_pwm_4);
          }
        }
      }
      else if (mappedx > joystick_disp_threshold_h){
        if (state_z == false){ // control motor2
          if (motor_pwm_1 < motor1_angle_high){
            motor_pwm_1 += m1_speed;
            pwmWrite(motorPIN1, motor_pwm_1);
          }
        }
        else{
          if (motor_pwm_4 < motor4_angle_high){
            motor_pwm_4 += m4_speed;
            // motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
            pwmWrite(motorPIN4, motor_pwm_4);
          }
        }
      }
      else{ // now we consider y values
        if (mappedy < joystick_disp_threshold_l){ // we consider joystic's x value first
          if (state_c == false){ // control motor2   
            if (motor_pwm_2 > motor2_angle_low){
              motor_pwm_2 -= m2_speed;
              pwmWrite(motorPIN2, motor_pwm_2);
            }
          }
          else{ // control motor3
            if (motor_pwm_3 > motor3_angle_low){
              motor_pwm_3 -= m3_speed;
              pwmWrite(motorPIN3, motor_pwm_3);
            }
          }
        }
        else if (mappedy > joystick_disp_threshold_l){
          if (state_c == false){ // control motor2
            if (motor_pwm_2 < motor2_angle_high){
              motor_pwm_2 += m2_speed; 
              pwmWrite(motorPIN2, motor_pwm_2);
            }
          }
          else{ // control motor3
            if (motor_pwm_3 < motor3_angle_high){
              motor_pwm_3 += m3_speed;
              pwmWrite(motorPIN3, motor_pwm_3);
            }
          }
        }
      }
    }
  }
  loop_cnt++;
  // delay(1);

}
