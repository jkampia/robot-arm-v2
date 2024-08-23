const int CLK = 52;
const int DT = 53;
const int pul[6] = {2, 5, 8,  11, 30, 33};
const int dir[6] = {3, 6, 9,  12, 31, 34};
const int ena[6] = {4, 7, 10, 13, 32, 35};

const double STEPS_PER_RAD = 2037.18327158;

int lastCLK; 
int curCLK; 

int counter; 
bool writemsg = false; 
String msg = "";
float msg_float;
int motor_num;  

unsigned long time1;

void setup() {
  
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); 

  for (int i = 0; i < 6; i++) {
    pinMode(pul[i], OUTPUT); 
    pinMode(dir[i], OUTPUT);
    pinMode(ena[i], OUTPUT);
  }

  for (int i = 0; i < 6; i++) {
    digitalWrite(ena[i], LOW);
  }
  digitalWrite(ena[4], HIGH);

}

void loop() {
  for (int i = 0; i < 500; i++) {
    step_motor(pul[4], 0.2);
  }
  for (int i = 0; i < 500; i++) {
    step_motor(pul[4], -0.2);
  }

}

void cancerous_serial_read() {
  while (Serial.available() > 0) { 
    char indata = Serial.read();
    if (indata == '>') {
      writemsg = false;
      //Serial.println("Read took " + String(micros() - time1) + " microseconds");
      //Serial.println("Motor chosen: " + String(motor_num));
      msg_float = msg.toFloat();
      //Serial.println(msg_float);
    }
    if (writemsg && counter > 0) {
      msg += indata; 
    }
    else {
      motor_num = String(indata).toInt();
    }
    counter++; 
    if (indata == '<') {
      writemsg = true; 
      counter = 0; 
      msg = "";
    }
  }
}

void step_motor(const int motor_idx, const float speed_rads) {
  if (speed_rads > 0) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);
  int step_delay_micros = 1 / (speed_rads * STEPS_PER_RAD) * 1000000;
  //Serial.println(step_delay_micros);
  //digitalWrite(pul[motor_idx], HIGH);
  if (step_delay_micros != 0) {
    digitalWrite(pul[motor_idx], HIGH);
    delayMicroseconds(step_delay_micros*0.5);
    //while(micros() - time1 < step_delay_micros); 
    digitalWrite(pul[motor_idx], LOW);
    delayMicroseconds(step_delay_micros*0.5);
  }

}
