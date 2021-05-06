#include <Arduino.h>
#include <stdint.h>

#include <ADC.h>
#include <ADC_util.h>
#include "curveFitting.h"

#define ENCODER_Y 2
#define ENCODER_G 3
#define PPERREV 134.4

#define DISPlACEMENT 19

#define MOTORL 14 /* Goes to IN1 */
#define MOTORR 15 /* Goes to IN2 */

#define BUFFER_SIZE 100
#define PRINT_TIMER 100000
#define MEASURE_TIMER 10000

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
const double ball_center = 496;
const double ball_range = (ball_end - ball_start)/2;
const double pos_x_ratio = travel_range / (ball_end - ball_center);
const double neg_x_ratio = travel_range / (ball_center - ball_start);
const double TICK_TIME = (MEASURE_TIMER * 0.000001);

volatile double x;
volatile float v;
volatile float prev_v = 0;
volatile float a;
volatile int idx = 0;

// Ticks for printing and calculating
volatile char new_val = 0;
volatile char print_now = 0;

volatile float Kp = 8;
volatile float Ki = 0;
volatile float Kd = 3;
volatile float Kdd = 8;
volatile float response = 0;

double dist(int x){
  return ((x-ball_center) >= 0) ? (x-ball_center)*pos_x_ratio : (x-ball_center)*neg_x_ratio;
}

double x_buf[BUFFER_SIZE];
double t_buf[BUFFER_SIZE];
const int order = 2;
double coeffs[order+1];

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

void setup() {

  Serial.begin(115200);


  print_clock.begin(tenth_tick, PRINT_TIMER);
  sys_clock.begin(sys_tick, MEASURE_TIMER);

  // Encoder Setup:
  pinMode(ENCODER_Y, INPUT);
  pinMode(ENCODER_G, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_Y), yellow_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_G), green_ISR, RISING);

  // Motor Setup:
  pinMode(MOTORL, OUTPUT);
  pinMode(MOTORR, OUTPUT);

  // Displacement Setup:
  pinMode(DISPlACEMENT, INPUT);

  for (int i = 0; i < BUFFER_SIZE; i++) t_buf[i] = i*TICK_TIME;
}

void tenth_tick(){
  print_now = 1;
}

void sys_tick(){
  new_val = 1;
}

double x_t;
double v_t;
double a_t;

void print_state(){
  
  if (print_now){
    x_t = coeffs[0]*t_buf[BUFFER_SIZE-1]*t_buf[BUFFER_SIZE-1] + coeffs[1]*t_buf[BUFFER_SIZE-1] + coeffs[2];
    v_t = 2*coeffs[0]*t_buf[BUFFER_SIZE-1] + coeffs[1]*t_buf[BUFFER_SIZE-1];
    a_t = 2*coeffs[0];

    //Serial.print(" t: " + String(rotation, DEC));
    Serial.print(" x: " + String(x, DEC));
    Serial.print(" x_t: " + String(x_t, DEC));
    Serial.print(" v: " + String(v, DEC));
    Serial.print(" v_t: " + String(v_t, DEC));
    Serial.print(" a_t: " + String(a_t, DEC) + "\n");
    //Serial.print(" res: " + String(response/255, DEC) + "\n");
    print_now = 0;
  }
}


//                        _______         _______       
//               G ______|       |_______|       |______ 
// negative <---      _______         _______         __  --> positive
//               Y __|       |_______|       |_______| 

void green_ISR(){
  if (digitalReadFast(ENCODER_Y)){
    rotation--;
  } else {
    rotation++;
  }
};

void yellow_ISR(){
  if (digitalReadFast(ENCODER_G)){
    rotation++;
  } else {
    rotation --;
  }
};

void loop() {
  calculate_motion();
  print_state();
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
      int ret = fitCurve(order, BUFFER_SIZE/sizeof(double), t_buf, x_buf, sizeof(coeffs)/sizeof(double), coeffs);
      if (ret != 0) Serial.println("ERROR");
      sum = (float)sum/BUFFER_SIZE;
      prev_v = v;
      v = (sum - prev_sum);
      a = (v - prev_v)/TICK_TIME;

      response = -(Kp*x + Kd*v + Kdd*a); 
    }
    new_val = 0;
  }
}

void drive_motor(){
  analogWrite(MOTORR, 0);
  analogWrite(MOTORL, 0);
}

void print_x_buf(){
  Serial.print("x_buf: [");
  for (int i=0; i<BUFFER_SIZE; i++){
    Serial.print(String(x_buf[i], DEC) +", ");
  }
  Serial.print("]\n");
}
