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

//static double L1 = ???  //define the length of L1 here
//static double L2 = ???  //define the length of L2 here

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

// Inverse kinematic function 
// Input: Coordinate: x,y,z; Length of links: L1, L2; Reference parameter of M1, M2 and M3
// This function will update the angle of three joints to the passed in M1, M2 and M3 parameters.

static void IK(int x, int y, int z, double L1, double L2, double & M1, double & M2, double & M3 )
{
  M1 = atan((x/y)*180/3.14); //M1 angle updated

  //Temp value during calculation, detail please referece https://drive.google.com/file/d/1QWOK2UDI9y4r2QihcjxmzVdvYBgfkfEr/view?usp=sharing
  double c1 = sqrt( pow(x,2) + pow(y,2) );
  double b = sqrt( pow(z,2) + pow(c1,2) );
  double P = L1 + L2 + b;
  double x = 2 * sqrt(P*(P-L1)*(P-L2)*(P-b))/b;
  //End of temp calculation
  
  M2 = atan((z/c1)*180/3.14) + asin((x/L1)*180/3.14); //M2 angle updated
  M3 = 180 - acos((x/L1)*180/3.14) - acos((x/L2)*180/3.14); //M3 angle updated

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


  bool pin1set = SetPinFrequencySafe(motorPIN1,300);  
  bool pin2set = SetPinFrequencySafe(motorPIN2,300);
  bool pin3set = SetPinFrequencySafe(motorPIN3,300);
  bool pin4set = SetPinFrequency(motorPIN4, 50);

  if(pin1set){
    pinMode(motorPIN1,OUTPUT); // pin 3
    // Serial.print("    pin1: ");
    // Serial.print(pin1set);
    // Serial.print("\n");
    pwmWrite(motorPIN1, motor_pwm_1);
  }
  if(pin2set){
    pinMode(motorPIN2,OUTPUT); // pin 9
    // Serial.print("    pin2: ");
    // Serial.print(pin2set);
    // Serial.print("\n");
    pwmWrite(motorPIN2, motor_pwm_2);
  }
  if(pin3set){
    pinMode(motorPIN3,OUTPUT);
    // Serial.print("    pin3: "); // pin 10
    // Serial.print(pin3set);
    // Serial.print("\n");
    pwmWrite(motorPIN3, motor_pwm_3);
  }
  if(pin4set){
    pinMode(motorPIN4,OUTPUT);
    // Serial.print("    pin4: "); // pin 5
    // Serial.print(pin4set);
    // Serial.print("\n");
    pwmWrite(motorPIN4, motor_pwm_4);
  }
   
}

void loop() {
  if ( loop_cnt > 10 ) { // every 1000 msecs get new data
    loop_cnt = 0;

    nunchuck_get_data(); // read values from the nunchuck

    // check C and Z status
    zbut = nunchuck_zbutton();  //  0 - 1
    cbut = nunchuck_cbutton();  //  0 - 1
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
    if (mappedx < 50){ // we consider joystic's x value first
      //Serial.print("\tleft\n");
      if (state_z == false){ // control motor1
        if (motor_pwm_1>42){
          // --motor_pwm_1;
          motor_pwm_1 -= 2;
          pwmWrite(motorPIN1, motor_pwm_1);
        }
      }
      else{ // control motor4
        if (motor_pwm_4>8){
          motor_pwm_4 -= 4;
          // motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
          pwmWrite(motorPIN4, motor_pwm_4);
          // pwmWrite(motorPIN4, motor_pwm_4);
        }
      }
    }
    else if (mappedx > 200){
      //Serial.print("\tright\n");
      if (state_z == false){ // control motor2
        if (motor_pwm_1<165){
          // ++motor_pwm_1; 
          motor_pwm_1 += 2;
          // Serial.print(motor_pwm_1);
          pwmWrite(motorPIN1, motor_pwm_1);
        }
      }
      else{
        if (motor_pwm_4<31){
          motor_pwm_4 += 4;
          // Serial.print(motor_pwm_4);
          // motor4angle = map(motor_pwm_4, 32, 195, 0, 180);
          pwmWrite(motorPIN4, motor_pwm_4);
          // pwmWrite(motorPIN4, motor_pwm_4);
        }
      }
    }
    else{ // now we consider y values
      //Serial.print("--------------- X DEAD, CONTROL Y ------------\n");
      if (mappedy < 50){ // we consider joystic's x value first
        //Serial.print("\nleft on motor 2 or 3\n");
        if (state_c == false){ // control motor2   
          //Serial.print("control motor 2\n");       
          if (motor_pwm_2>32){
            // --motor_pwm_2;
            motor_pwm_2 -= 2;
            //Serial.print("--------------- decrease motor_pwm_2 ------------\n");
            pwmWrite(motorPIN2, motor_pwm_2);
          }
        }
        else{ // control motor3
          if (motor_pwm_3>32){
            // --motor_pwm_3;
            motor_pwm_3 -= 2;
            //Serial.print("--------------- decrease motor_pwm_3 ------------\n");
            pwmWrite(motorPIN3, motor_pwm_3);
          }
        }
      }
      else if (mappedy > 200){
        //Serial.print("\tright\n");
        if (state_c == false){ // control motor2
          if (motor_pwm_2<165){
            // ++motor_pwm_2;
            motor_pwm_2 += 2; 
            //Serial.print("--------------- increase motor_pwm_2 ------------\n");
            // Serial.print(motor_pwm_2);
            pwmWrite(motorPIN2, motor_pwm_2);
          }
        }
        else{ // control motor3
          if (motor_pwm_3<165){
            // ++motor_pwm_3; 
            motor_pwm_3 += 2;
            //Serial.print("\n\n--------------- increase motor_pwm_3 ------------\n\n");
            // Serial.print(motor_pwm_3);
            pwmWrite(motorPIN3, motor_pwm_3);
          }
        }
      }
    }
  }
  loop_cnt++;
  // delay(1);

}
