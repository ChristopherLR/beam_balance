#define ENCODER_Y 2
#define ENCODER_G 3
#define PPERREV 134.4


#define MOTORL 14 /* Goes to IN1 */
#define MOTORR 15 /* Goes to IN2 */

IntervalTimer sys_clock;

float y = 0.0;
float y_prev = 0.0;
float g = 0.0;
float g_prev = 0.0;
int prev_time = 0;
int curr_time = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENCODER_Y, INPUT);
  pinMode(ENCODER_G, INPUT);
  Serial.begin(115200);

  sys_clock.begin(tenth_tick, 1000000);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y), yellow_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_G), green_ISR, RISING);

  pinMode(MOTORL, OUTPUT);
  pinMode(MOTORR, OUTPUT);
}

void tenth_tick(){

  float y_diff = y - y_prev;
  float g_diff = g - g_prev;
  y_prev = y;
  g_prev = g;
  float y_rpm = ((y_diff/PPERREV))*60;
  float g_rpm = ((g_diff/PPERREV))*60;
  Serial.println("y: " +  String(y_rpm, DEC) + " g: " + String(g_rpm, DEC));
}


void green_ISR(){
  g++;    
};

void yellow_ISR(){
  y++;
};

void loop() {
  for (int v = 0; v < 250; v+=25){
    Serial.println(String(v, DEC) + " : ");
    for (int i = 0; i < 10; i++){
      analogWrite(MOTORR, v);
      analogWrite(MOTORL, 0);
      delay(1000);
    }
  }
}
