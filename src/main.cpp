#include <Arduino.h>
#include <stdint.h>

#include <ADC.h>
#include <ADC_util.h>
#include "curveFitting.h"

#define ENCODER_Y 2
#define ENCODER_G 3
#define PPERREV 134.4
#define AR_LIM 15
#define VR_LIM 15
#define XR_LIM 15

#define DISPlACEMENT 19

#define MOTORL 14 /* Goes to IN1 */
#define MOTORR 15 /* Goes to IN2 */

#define BUFFER_SIZE 10
#define PRINT_TIMER 100000
#define MEASURE_TIMER 5000

IntervalTimer sys_clock;
IntervalTimer print_clock;

// Encoder variables
// Clockwise = Negative
volatile int rotation = 0;

const float ball_radius = 13.5;
const float beam_length = 170;
const float travel_dist = beam_length - 2*ball_radius; 
const float travel_range = travel_dist/2;

const double ball_start = 24;
const double ball_end = 988;
const double ball_center = 512; // 496 // 528
const double ball_range = (ball_end - ball_start)/2;
const double pos_x_ratio = travel_range / (ball_end - ball_center);
const double neg_x_ratio = travel_range / (ball_center - ball_start);
const double TICK_TIME = (MEASURE_TIMER * 0.000001);

volatile double x;
volatile float v;
volatile float prev_v = 0;
volatile float a;
volatile float ar;

volatile float x_res;
volatile float v_res;
volatile float a_res;
volatile float ar_res;

volatile int idx = 0;

// Ticks for printing and calculating
volatile char new_val = 0;
volatile char print_now = 0;

// 12/494 

volatile double Kp = 0.0062;  // 0.0062
volatile double Kd = 0.00068; // 0.0007
volatile double Kdd = 0.00;
volatile double Kddr = -0.085; // - 0.080
volatile double response = 0;

double dist(int x){
  return x-ball_center;
}

double x_buf[BUFFER_SIZE];
float v_buf[BUFFER_SIZE];
float a_buf[BUFFER_SIZE];

volatile int move_idx = 1;
volatile float sum = 0;
volatile float prev_sum = 0;
volatile float tmp = 0;

int prev_time = 0;
int curr_time = 0;

void yellow_ISR();
void green_ISR();
void tenth_tick();
void sys_tick();
void print_x_buf();
void calculate_motion();
void calculate_averages();
void print_state();
void drive_cw(double);
void drive_acw(double);

void setup() {

  Serial.begin(115200);


  print_clock.begin(tenth_tick, PRINT_TIMER);
  sys_clock.begin(sys_tick, MEASURE_TIMER);

  // Encoder Setup:
  pinMode(ENCODER_Y, INPUT);
  pinMode(ENCODER_G, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_Y), yellow_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_G), green_ISR, CHANGE);

  // Motor Setup:
  pinMode(MOTORL, OUTPUT);
  pinMode(MOTORR, OUTPUT);

  // Displacement Setup:
  pinMode(DISPlACEMENT, INPUT);
}

void tenth_tick(){
  print_now = 1;
}

void sys_tick(){
  new_val = 1;
}

void print_state(){
  
  if (print_now){
//    Serial.print(" r: " + String(rotation, DEC));
//    Serial.print(" x: " + String(x, DEC));
    Serial.print(" p: " + String(x_res, DEC));
//    Serial.print(" v: " + String(v, DEC));
    Serial.print(" d: " + String(v_res, DEC));
//    Serial.print(" a: " + String(ar, DEC));
    Serial.print(" dd: " + String(ar_res, DEC));
    Serial.print(" res: " + String(response, DEC) + "\n");
    print_now = 0;
  }
}


//                        _______         _______       
//               G ______|       |_______|       |______ 
// negative <---      _______         _______         __  --> positive
//               Y __|       |_______|       |_______| 

void green_ISR(){
  if (digitalReadFast(ENCODER_Y) != digitalReadFast(ENCODER_G)){
    rotation++;
  } else {
    rotation--;
  }
};

void yellow_ISR(){
  if (digitalReadFast(ENCODER_G) == digitalReadFast(ENCODER_Y)){
    rotation++;
  } else {
    rotation--;
  }
};

void loop() {
  calculate_motion();
  print_state();

  if (response < -0.005){
    drive_cw(response);
  } 
  
  if ( response > 0.005){
    drive_acw(-response);
  }

}


void calculate_motion(){
  if (new_val){
    x = dist(analogRead(DISPlACEMENT));
    if (idx < BUFFER_SIZE){
      x_buf[idx] = x;
      idx++;
    } else {
      move_idx= 1;
      prev_sum = sum;
      sum = 0;
      sum += x_buf[0]; 
      for (; move_idx < BUFFER_SIZE; move_idx++){
        tmp = x_buf[move_idx];
        sum += tmp;
        x_buf[move_idx-1] = tmp;
      }
      x_buf[BUFFER_SIZE-1] = x;
      sum = (float)sum/BUFFER_SIZE;
      prev_v = v;
      v = (sum - prev_sum)/TICK_TIME;
      a = (v - prev_v)/TICK_TIME;
      x_res = Kp*x;
      v_res = Kd*v;
      a_res = Kdd*a;
      ar = rotation;
      ar_res = Kddr*ar;

      if(v_res > VR_LIM) { v_res = VR_LIM; }
      if(v_res < -VR_LIM) { v_res = -VR_LIM; }
      if(ar_res > AR_LIM) { ar_res = AR_LIM; }
      if(ar_res < -AR_LIM) { ar_res = -AR_LIM;}
      if(x_res > XR_LIM){ x_res = XR_LIM; }
      if(x_res < -XR_LIM){ x_res = -XR_LIM; }

      response = -(x_res + v_res + a_res + ar_res); 
    }
    new_val = 0;
  }
}

void drive_cw(double v){
  int in = v*(255.0/12.0) + 5;
  analogWrite(MOTORR, in);
  analogWrite(MOTORL, 0);
}

void drive_acw(double v){
  int in = v*(255.0/12.0) + 5;
  analogWrite(MOTORR, 0);
  analogWrite(MOTORL, in);
}

void print_x_buf(){
  Serial.print("x_buf: [");
  for (int i=0; i<BUFFER_SIZE; i++){
    Serial.print(String(x_buf[i], DEC) +", ");
  }
  Serial.print("]\n");
}
