
const int pul[6] = {2, 5, 8,  11, 30, 33};
const int dir[6] = {3, 6, 9,  12, 31, 34};
const int ena[6] = {4, 7, 10, 13, 32, 35};
int ena_flags[6] = {1, 1, 1, 1, 1, 1};
const float STEPS_PER_RAD[6] = {2037.183271, 2037.183271, 5092.958179, 0, 0, 0};

// J1: POSITIVE ANGULAR VELOCITY IS CW
// J2: POSITIVE ANGULAR VELOCITY IS

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

  process_ena_flags();
}

void loop() {

  time1 = micros(); 
  cancerous_serial_read(); 
  process_ena_flags(); 
  step_individual_motor(motor_num, msg_float);
    
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
    if (writemsg && counter > 1) {
      msg += indata; 
    }
    else if (counter == 0) {
      motor_num = String(indata).toInt();
    }
    else if (counter == 1) {
      ena_flags[motor_num] = String(indata).toInt();  
    }
    counter++; 
    if (indata == '<') {
      writemsg = true; 
      counter = 0; 
      msg = "";
    }
  }
}

void process_ena_flags() {
  for (int i = 0; i < 6; i++) {
    if (ena_flags[i] == 1) digitalWrite(ena[i], HIGH);
    else digitalWrite(ena[i], LOW);
  }
}

void step_individual_motor(const int motor_idx, const float speed_rads) {
  if (speed_rads == 0) return; 
  else if (speed_rads > 0) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);

  const int step_delay_micros = 1 / (abs(speed_rads) * STEPS_PER_RAD[motor_idx]) * 1000000;
  //Serial.println(step_delay_micros);
  //digitalWrite(pul[motor_idx], HIGH);
  digitalWrite(pul[motor_idx], HIGH);
  //delayMicroseconds(step_delay_micros);
  while(micros() - time1 < step_delay_micros); 
  digitalWrite(pul[motor_idx], LOW);
}

void linear_concurrent_step(const int *motor_flags, const float 
