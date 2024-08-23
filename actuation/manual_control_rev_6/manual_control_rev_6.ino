// Written by Jonathan Kampia
// jonathankampia@gmail.com

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923

const int pul[6] = {2, 5, 8,  24, 27, 30};
const int dir[6] = {3, 6, 9,  25, 28, 31};
const int ena[6] = {4, 7, 10, 26, 29, 32};
int ena_flags[6] = {1, 1, 1, 1, 1, 1};

const float angle = 1;
const float accel = 0.1;
const float delay0 = 2000 * sqrt(2 * angle / accel) * 0.67703;
const float d0[6] = {delay0, delay0, delay0, delay0, delay0, delay0};
const int min_step_delay[6] = {100, 100, 100, 100, 100, 100};

const float STEPS_PER_RAD[6] = {3055.774907, 5092.958179, 5092.958179, 5092.958179, 2037.183271, 1018.591634};
const float joint_maxvel_radps[6] = {0.1, 0.15, 0.2, 1, 1, 1};
float STEPS_PER_DEG[6];
float DEGS_PER_STEP[6];

float cur_joint_angs[6] = {0, 0, 0, 0, 0, 0}; 
float cur_pose[6] = {0, 0, 0, 0, 0, 0};

float dh_theta[6] = {0, -PI_2, 0, 0, 0, 0};
const float dh_alpha[6] = {-PI_2, 0, -PI_2, PI_2, -PI_2, 0};
const float dh_d[6] = {167.01, 0, 0, 174.39, 0, 0};
const float dh_a[6] = {0, 181.04, 0, 0, 0, 0};

//const float RAD_TO_DEG = 57.295777754771045;
//const float DEG_TO_RAD = 0.01745329251;

// J1: POSITIVE ANGULAR VELOCITY IS CW, DIR = HIGH IS CW
// J2: POSITIVE ANGULAR VELOCITY IS CCW, DIR = HIGH IS CCW
// J3: POSITIVE ANG VEL IS CW, DIR = HIGH IS CW

struct pose_6D {
  float a[6];
  int p; 
};
int queue_len = 0; 
pose_6D joint_ang_queue[100]; // max queue size is 100

int counter = 0; 
bool writemsg = false; 
String msg = "";
float msg_float;
int motor_num; 
int command_id; 
int ena_straight_line_path;
String path_speed_msg = ""; 
float path_speed; 
String IK_pose_msg[6];
float IK_pose[6]; 
String new_joint_pose_msg[6];
float new_joint_pose[6];  
int pose_idx; 
bool new_command = false;
String custom_msg; 
int num_segs = 1; 

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
  
  for (int i = 0; i < 6; i++) {
    STEPS_PER_DEG[i] = STEPS_PER_RAD[i] * PI/180;
    DEGS_PER_STEP[i] = 1/STEPS_PER_DEG[i];
  }

  solve_FK(cur_joint_angs, cur_pose);
  Serial.print("Home position: ");
  print_matrix(cur_pose, 1, 6);
}

int count = 0;
void loop() {
  
  time1 = micros(); 
  custom_serial_read2(); 
  print_data2(); 
  parse_command2();
  
  //print_data();
  
  //float *test_step_profile = new float[800];
  //generate_accel_curve(0, test_step_profile, 800);
  //free(test_step_profile);
  //delay(10000000);
  //digitalWrite(4, HIGH);
  
}

void process_ena_flags() {
  for (int i = 0; i < 6; i++) {
    if (ena_flags[i] == 1) digitalWrite(ena[i], HIGH);
    else digitalWrite(ena[i], LOW);
  }
}

void step_individual_motor_vel(const int motor_idx, const float speed_rads) {
  if (speed_rads == 0) return; 
  else if (speed_rads > 0) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);

  const int step_delay_micros = 1 / (abs(speed_rads) * STEPS_PER_RAD[motor_idx]) * 1000000;
  //Serial.println(step_delay_micros);
  digitalWrite(pul[motor_idx], HIGH);
  while(micros() - time1 < step_delay_micros*0.5); 
  digitalWrite(pul[motor_idx], LOW);
  while(micros() - time1 < step_delay_micros); 
}

void step_individual_motor(const int motor_idx, const int angle_deg, const int step_delay) {
  float angle_rads = angle_deg * DEG_TO_RAD; 
  int num_steps = angle_rads * STEPS_PER_RAD[motor_idx]; 
  if (num_steps > 0) digitalWrite(dir[motor_idx], HIGH);
  else digitalWrite(dir[motor_idx], LOW);
  for (int i = 0; i < abs(num_steps); i++) {
    digitalWrite(pul[motor_idx], HIGH);
    delayMicroseconds(step_delay);
    digitalWrite(pul[motor_idx], LOW);
    delayMicroseconds(step_delay);
  }
  int num_full_rots = abs(floor(angle_deg / 360));
  if (angle_deg > 360) {
    cur_joint_angs[motor_idx] += (angle_rads - 360*num_full_rots);
  }
  else if (angle_deg < -360) {
    cur_joint_angs[motor_idx] += (angle_rads + 360*num_full_rots);
  } 
  else {
    cur_joint_angs[motor_idx] = angle_deg;
  }
}

void joint_space_move(const float *start_angs, const float *end_angs) {
  Serial.println("Moving with no accel");
  float steps_to_move[6];
  int calc_step_delay[6]; 
  float max_steps = 0;
  int max_step_index = 0; 
  for (int i = 0; i < 6; i++) {
    steps_to_move[i] = (end_angs[i] - start_angs[i]) * DEG_TO_RAD * STEPS_PER_RAD[i];
    if (abs(steps_to_move[i]) > max_steps) {
      max_steps = abs(steps_to_move[i]);
      max_step_index = i; 
    }
  }
  Serial.print("Steps to move: ");
  print_matrix(steps_to_move, 1, 6);
  Serial.println("Max steps: " + String(max_steps));

  calc_step_delay[max_step_index] = 1000000/(joint_maxvel_radps[max_step_index] * STEPS_PER_RAD[max_step_index]);
  //Serial.println(max_step_index);
  //Serial.println(calc_step_delay[max_step_index]);
  float total_move_time = max_steps * calc_step_delay[max_step_index];
  //Serial.println(total_move_time);
  for (int i = 0; i < 6; i++) {
    if (steps_to_move[i] == 0) continue; 
    calc_step_delay[i] = total_move_time / abs(steps_to_move[i]);
    Serial.println(String(calc_step_delay[i]) + ":" + String(steps_to_move[i]));
  }
  if (steps_to_move[0] < 0) digitalWrite(dir[0], HIGH);
  else digitalWrite(dir[0], LOW);
  if (steps_to_move[1] < 0) digitalWrite(dir[1], HIGH);
  else digitalWrite(dir[1], LOW);
  if (steps_to_move[2] > 0) digitalWrite(dir[2], HIGH);
  else digitalWrite(dir[2], LOW);
  
  unsigned int stepcount[6] = {0, 0, 0, 0, 0, 0};
  double motor_step_time[6];
  double half_motor_step_time[6];
  for (int i = 0; i < 6; i++) {
    motor_step_time[i] = calc_step_delay[i];
    half_motor_step_time[i] = calc_step_delay[i] * 0.5; 
  }

  unsigned long time1 = micros();
  while (stepcount[max_step_index] < max_steps) {
    for (int i = 0; i < 6; i++) {
      if (steps_to_move[i] == 0) continue; 
      if (micros() - time1 > half_motor_step_time[i] && steps_to_move[i] != 0) {
        digitalWrite(pul[i], HIGH);
        half_motor_step_time[i] += calc_step_delay[i];
      }
      if (micros() - time1 > motor_step_time[i] && steps_to_move[i] != 0) {
        digitalWrite(pul[i], LOW);
        stepcount[i]++; 
        motor_step_time[i] += calc_step_delay[i];
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    cur_joint_angs[i] = end_angs[i];
  }
  solve_FK(cur_joint_angs, cur_pose);
  //print_matrix(cur_joint_angs, 1, 6);
}

void joint_space_move_accel(const float *start_angs, const float *end_angs, bool accel) {
  Serial.println("Moving with no accel");
  int steps_to_move[6];
  float max_steps = 0;
  int max_step_index = 0; 
  for (int i = 0; i < 6; i++) {
    steps_to_move[i] = (end_angs[i] - start_angs[i]) * DEG_TO_RAD * STEPS_PER_RAD[i];
    if (abs(steps_to_move[i]) > max_steps) {
      max_steps = abs(steps_to_move[i]);
      max_step_index = i; 
    }
  }

  if (steps_to_move[0] < 0) digitalWrite(dir[0], HIGH);
  else digitalWrite(dir[0], LOW);
  if (steps_to_move[1] < 0) digitalWrite(dir[1], HIGH);
  else digitalWrite(dir[1], LOW);
  if (steps_to_move[2] > 0) digitalWrite(dir[2], HIGH);
  else digitalWrite(dir[2], LOW);
  if (steps_to_move[3] > 0) digitalWrite(dir[3], LOW);
  else digitalWrite(dir[3], HIGH);
  if (steps_to_move[4] > 0) digitalWrite(dir[4], HIGH);
  else digitalWrite(dir[4], LOW);
  if (steps_to_move[5] > 0) digitalWrite(dir[5], HIGH);
  else digitalWrite(dir[5], LOW);

  if (accel) go_steps_accel(steps_to_move);
  else go_steps_accel(steps_to_move);

  for (int i = 0; i < 6; i++) {
    cur_joint_angs[i] += steps_to_move[i] * DEGS_PER_STEP[i];
  }
  solve_FK(cur_joint_angs, cur_pose);
}

void scale_array(float *array, int len, float scalar) {
  for (int i = 0; i < len; i++) {
    array[i] *= scalar; 
  }
  long scaled_time = 0;
  for (int i = 0; i < len; i++) {
    scaled_time += array[i];
  }
  //Serial.println("Scaled time: " + String(scaled_time));
}

void calc_step_delays(const int motor, const int num_steps, float *delay_array) { 

  float d = d0[motor];
  int n = 0; 
  int rampUpStepCount = 0;
  int totalSteps = 0;

  for (int i = 0; i < num_steps; i++) {
    if (rampUpStepCount == 0) {
      n++; 
      d = d - (2 * d) / (4 * n + 1);
      if (d <= min_step_delay[motor]) {
        d = min_step_delay[motor]; 
        rampUpStepCount = i;
        //Serial.println(rampUpStepCount); 
      }
      if (i >= num_steps / 2) {
        rampUpStepCount = i;
        //Serial.println(rampUpStepCount); 
      }
    }
    else if (i >= num_steps - rampUpStepCount) {
      n--;
      d = (d * (4 * n + 1)) / (4 * n + 1 - 2);
    }
    delay_array[i] = d;
    //Serial.println(delay_array[i]);  
  }
}

void go_steps_accel(const int *num_steps) {

  Serial.println("Starting move.");

  float sum_time[6] = {0, 0, 0, 0, 0, 0};
  float scalar[6] = {0, 0, 0, 0, 0, 0};
  
  float *j1_delay_array = new float[abs(num_steps[0])];
  calc_step_delays(0, abs(num_steps[0]), j1_delay_array);
  for (int i = 0; i < abs(num_steps[0]); i++) sum_time[0] += j1_delay_array[i];
  //Serial.println("Sum time 1: " + String(sum_time[0]));

  float *j2_delay_array = new float[abs(num_steps[1])];
  calc_step_delays(1, abs(num_steps[1]), j2_delay_array);
  for (int i = 0; i < abs(num_steps[1]); i++) sum_time[1] += j2_delay_array[i];
  //Serial.println("Sum time 1: " + String(sum_time[0]));

  float *j3_delay_array = new float[abs(num_steps[2])];
  calc_step_delays(2, abs(num_steps[2]), j3_delay_array);
  for (int i = 0; i < abs(num_steps[2]); i++) sum_time[2] += j3_delay_array[i]; 
  //Serial.println("Sum time 3: " + String(sum_time[2]));

  float *j4_delay_array = new float[abs(num_steps[3])];
  calc_step_delays(3, abs(num_steps[3]), j4_delay_array);
  for (int i = 0; i < abs(num_steps[3]); i++) sum_time[3] += j4_delay_array[i]; 

  float *j5_delay_array = new float[abs(num_steps[4])];
  calc_step_delays(4, abs(num_steps[4]), j5_delay_array);
  for (int i = 0; i < abs(num_steps[4]); i++) sum_time[4] += j5_delay_array[i];

  float *j6_delay_array = new float[abs(num_steps[5])];
  calc_step_delays(5, abs(num_steps[5]), j6_delay_array);
  for (int i = 0; i < abs(num_steps[5]); i++) sum_time[5] += j6_delay_array[i];

  float max_time = 0; 
  int max_idx = 0; 
  for (int i = 0; i < 6; i++) {
    if (sum_time[i] > max_time) {
      max_time = sum_time[i];
      max_idx = i;
    }
    //Serial.println(sum_time[i]);
  }

  //Serial.println("Max time: " + String(max_time));

  for (int i = 0; i < 6; i++) {
    if (sum_time[i] != 0) {
      scalar[i] = max_time / sum_time[i];
      //Serial.println(scalar[i]);
    }
  }
  scale_array(j1_delay_array, abs(num_steps[0]), scalar[0]);
  scale_array(j2_delay_array, abs(num_steps[1]), scalar[1]);
  scale_array(j3_delay_array, abs(num_steps[2]), scalar[2]);
  scale_array(j4_delay_array, abs(num_steps[3]), scalar[3]);
  scale_array(j5_delay_array, abs(num_steps[4]), scalar[4]);
  scale_array(j6_delay_array, abs(num_steps[5]), scalar[5]);

  int stepCount[6];
  unsigned long ref_time[6];
  for (int i = 0; i < 6; i++) { 
    ref_time[i] = micros();
    stepCount[i] = 0; 
  }
  while (!check_finished(stepCount, num_steps)) {
    if (micros() - ref_time[0] > j1_delay_array[stepCount[0]] * 0.5 && micros() - ref_time[0] < j1_delay_array[stepCount[0]] && num_steps[0] != 0) {
      digitalWrite(pul[0], HIGH);
    }
    else if (micros() - ref_time[0] > j1_delay_array[stepCount[0]] && num_steps[0] != 0) {
      digitalWrite(pul[0], LOW);
      stepCount[0]++; 
      ref_time[0] = micros();   
    }
    if (micros() - ref_time[1] > j2_delay_array[stepCount[1]] * 0.5 && micros() - ref_time[1] < j2_delay_array[stepCount[1]] && num_steps[1] != 0) {
      digitalWrite(pul[1], HIGH);
    }
    else if (micros() - ref_time[1] > j2_delay_array[stepCount[1]] && num_steps[1] != 0) {
      digitalWrite(pul[1], LOW);
      stepCount[1]++; 
      ref_time[1] = micros();   
    }
    if (micros() - ref_time[2] > j3_delay_array[stepCount[2]] * 0.5 && micros() - ref_time[2] < j3_delay_array[stepCount[2]] && num_steps[2] != 0) {
      digitalWrite(pul[2], HIGH);
    }
    else if (micros() - ref_time[2] > j3_delay_array[stepCount[2]] && num_steps[2] != 0) {
      digitalWrite(pul[2], LOW);
      stepCount[2]++; 
      ref_time[2] = micros();   
    }
    if (micros() - ref_time[3] > j4_delay_array[stepCount[3]] * 0.5 && micros() - ref_time[3] < j4_delay_array[stepCount[3]] && num_steps[3] != 0) {
      digitalWrite(pul[3], HIGH);
    }
    else if (micros() - ref_time[3] > j4_delay_array[stepCount[3]] && num_steps[3] != 0) {
      digitalWrite(pul[3], LOW);
      stepCount[3]++; 
      //Serial.println(stepCount[3]);
      ref_time[3] = micros();   
    }
    if (micros() - ref_time[4] > j5_delay_array[stepCount[4]] * 0.5 && micros() - ref_time[4] < j5_delay_array[stepCount[4]] && num_steps[4] != 0) {
      digitalWrite(pul[4], HIGH);
    }
    else if (micros() - ref_time[4] > j5_delay_array[stepCount[4]] && num_steps[4] != 0) {
      digitalWrite(pul[4], LOW);
      stepCount[4]++; 
      ref_time[4] = micros();   
    }
    if (micros() - ref_time[5] > j6_delay_array[stepCount[5]] * 0.5 && micros() - ref_time[5] < j6_delay_array[stepCount[5]] && num_steps[5] != 0) {
      digitalWrite(pul[5], HIGH);
    }
    else if (micros() - ref_time[5] > j6_delay_array[stepCount[5]] && num_steps[5] != 0) {
      digitalWrite(pul[5], LOW);
      stepCount[5]++; 
      ref_time[5] = micros();   
    }
  }

  delete [] j1_delay_array;
  delete [] j2_delay_array;
  delete [] j3_delay_array;
  delete [] j4_delay_array;
  delete [] j5_delay_array;
  delete [] j6_delay_array;

  Serial.println("Move done.");
}

void go_segment(const int *num_steps, const float seg_len, const float vel_mms) {

  float seg_time = seg_len / vel_mms; 
  int calc_step_delay[6]; 
  for (int i = 0; i < 6; i++) {
    if (num_steps[i] != 0) { 
      calc_step_delay[i] = seg_time / num_steps[i];
    }
    else {
      calc_step_delay[i] = 0; 
    }
    Serial.println(String(calc_step_delay[i]) + ":" + String(num_steps[i]));
  }
  
  unsigned int stepcount[6] = {0, 0, 0, 0, 0, 0};
  double motor_step_time[6];
  double half_motor_step_time[6];
  for (int i = 0; i < 6; i++) {
    motor_step_time[i] = calc_step_delay[i];
    half_motor_step_time[i] = calc_step_delay[i] * 0.5; 
  }
  unsigned long time1 = micros();
  while (!check_finished(stepcount, num_steps)) {
    for (int i = 0; i < 6; i++) {
      if (num_steps[i] == 0) continue; 
      if (micros() - time1 > half_motor_step_time[i] && num_steps[i] != 0) {
        digitalWrite(pul[i], HIGH);
        half_motor_step_time[i] += calc_step_delay[i];
      }
      if (micros() - time1 > motor_step_time[i] && num_steps[i] != 0) {
        digitalWrite(pul[i], LOW);
        stepcount[i]++; 
        motor_step_time[i] += calc_step_delay[i];
      }
    }
  }
}

bool check_finished(int *stepCount, int *len) {
  //Serial.println(String(stepCount[3]) + ":" + String(len[3]));
  for (int i = 0; i < 6; i++) {
    if (stepCount[i] < abs(len[i])) {
      //Serial.println(i);
      return false; 
    }
  }
  return true; 
}

void home_on_startup() {
  const float adjustment_angs[6] = {0, 43, -75, 0, 0, 0};
  joint_space_move_accel(cur_joint_angs, adjustment_angs, true);
  // reset pose to 0
  for (int i = 0; i < 6; i++) cur_joint_angs[i] = 0; 
  solve_FK(cur_joint_angs, cur_pose);
}

void home_on_shutdown() {
  const float adjustment_angs[6] = {0, -43, 75, 0, 0, 0};
  for (int i = 0; i < 3; i++) {
    if (fabsf(cur_joint_angs[i]) > 0.1) {
      Serial.println("Not at home! Cannot return to shutdown position... Current joint angs are: ");
      print_matrix(cur_joint_angs, 1, 6);
      return;
    }
  }
  joint_space_move_accel(cur_joint_angs, adjustment_angs, true);
  // reset pose to 0
  for (int i = 0; i < 6; i++) cur_joint_angs[i] = 0; 
  solve_FK(cur_joint_angs, cur_pose);
}

void go_home() {
  const float home[6] = {0, 0, 0, 0, 0, 0};
  joint_space_move_accel(cur_joint_angs, home, true);
  for (int i = 0; i < 6; i++) cur_joint_angs[i] = 0;
  solve_FK(cur_joint_angs, cur_pose); 
}

void go_line_path(const float *start_pose, const float *end_pose, const float stepsize_mm, const float vel_mms) {
  float vector_mag = sqrt(pow(end_pose[0]-start_pose[0],2)+pow(end_pose[1]-start_pose[1],2)+pow(end_pose[2]-start_pose[2],2));
  Serial.println("Vector mag: " + String(vector_mag));
  int num_steps = vector_mag / stepsize_mm; 

  float unitvector[3]; 
  unitvector[0] = (end_pose[0] - start_pose[0]) / vector_mag;
  unitvector[1] = (end_pose[1] - start_pose[1]) / vector_mag;
  unitvector[2] = (end_pose[2] - start_pose[2]) / vector_mag;

  for (int i = 0; i < num_steps; i++) {
    float next[3]; 
    next[0] = start_pose[0] + unitvector[0] * stepsize_mm * i;
    next[1] = start_pose[1] + unitvector[1] * stepsize_mm * i;
    next[2] = start_pose[2] + unitvector[2] * stepsize_mm * i;
    //Serial.println("Point " + String(i) + ": (" + String(path_array[i].x) + ", " + String(path_array[i].y) + ", " + String(path_array[i].z) + ")" );

    float IK_in[6] = {next[0], next[1], next[2], start_pose[3], start_pose[4], start_pose[5]};
    float IK_out[6] = {0, 0, 0, 0, 0, 0};
    solve_IK(IK_in, IK_out);

    float step_difference[6];
    int steps_this_seg = 0;
    for (int i = 0; i < 6; i++) {
      step_difference[i] = floor((IK_out[i] - cur_joint_angs[i]) * STEPS_PER_DEG[i]);
      steps_this_seg += abs(step_difference[i]);
      cur_joint_angs[i] += step_difference[i] * DEGS_PER_STEP[i];
    }
    Serial.println("Steps in seg: " + String(steps_this_seg));

    if (step_difference[0] < 0) digitalWrite(dir[0], HIGH);
    else digitalWrite(dir[0], LOW);
    if (step_difference[1] < 0) digitalWrite(dir[1], LOW);
    else digitalWrite(dir[1], HIGH);
    if (step_difference[2] > 0) digitalWrite(dir[2], LOW);
    else digitalWrite(dir[2], HIGH);  

    unsigned int local_step_delay = stepsize_mm / vel_mms / steps_this_seg * 500000;
    if (local_step_delay < 100) {
      //Serial.println("Calculated step delay is " + String(local_step_delay) + ". This is too fast!");
      local_step_delay = 100; 
    }

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < abs(step_difference[i]); j++) {
        digitalWrite(pul[i], HIGH);
        delayMicroseconds(local_step_delay);
        digitalWrite(pul[i], LOW);
        delayMicroseconds(local_step_delay);
      }
    }
  }

  Serial.print("Final joint angs: ");
  print_matrix(cur_joint_angs, 1, 6);

  solve_FK(cur_joint_angs, cur_pose);
  Serial.print("Final pose: ");
  print_matrix(cur_pose, 1, 6);
}

bool check_pose_possible(const float *pose) {
  //Serial.print("Checking pose: ");
  //print_matrix(pose, 1, 6);
  float z_diff = pose[2] - dh_d[0];
  float xy_diff = sqrt(pow(pose[0], 2) + pow(pose[1], 2));
  float required_limb_length = sqrt(pow(z_diff, 2) + pow(xy_diff, 2));
  //Serial.println(required_limb_length);
  if (required_limb_length > dh_a[1] + dh_d[3]) {
    Serial.println("Pose potentially invalid! Distance to end effector from joint 2: " + String(required_limb_length));
    return false;
  }
  else {
    return true; 
  }
}

void jog_position(const float *start_angs, const float *end_pose) {
  if (!check_pose_possible(end_pose)) return;
  if (ena_straight_line_path == 0) {
    float IK_out[6] = {0, 0, 0, 0, 0, 0};
    solve_IK(end_pose, IK_out);
    Serial.print("IK solution: ");
    print_matrix(IK_out, 1, 6);
    //check_IK_solution(IK_out);
    joint_space_move_accel(start_angs, IK_out, true);
  }
}

void check_IK_solution(float *IK_solution) {
  // joints 4 and 6 are problem angles -- check if they are reverse of each other and set to 0 if so!
  if (abs(abs(IK_solution[3]) - abs(IK_solution[5])) < 0.01 && IK_solution[0] < 0.01) {
    Serial.println("Joints 4 and 6 are inverse of each other -- setting to 0.");
    IK_solution[3] = 0;
    IK_solution[5] = 0; 
  }
  Serial.print("New IK solution: ");
  print_matrix(IK_solution, 1, 6);
}

void add_to_queue(const float *joint_angs) {
  for (int i = 0; i < 6; i++) {
    joint_ang_queue[queue_len].a[i] = joint_angs[i];
  }
  queue_len++; 
}

void mat_multiply(const float* mat1, const float* mat2, float* matout, const int rows1, const int cols2, const int rows2) {
    float tmp;
    int i, j, k;
    for (i = 0; i < rows1; i++) {
        for (j = 0; j < cols2; j++) {
            tmp = 0.0f;
            for (k = 0; k < rows2; k++) {
                tmp += mat1[cols2 * i + k] * mat2[rows2 * k + j];
            }
            matout[rows2 * i + j] = tmp;
        }
    }
}

void print_matrix(const float *matrix, const int rows, const int cols) { // debugging func
    printf("\n [ ");
    for (int i = 0; i < rows * cols; i++) {
        if (i % cols == 0 && i != 0) printf(" ] \n [ ");
        printf("%f", matrix[i]);
        if ((i+1) % cols != 0) printf("\t");
    }
    printf(" ] \n");
}

void scale_matrix(float *matrix, float scalar, const int rows, const int cols) {
    for (int i = 0; i < rows * cols; i++) {
        matrix[i] *= scalar; 
    }
}

void set_dh_params(float *params, float *joints) {
    for (int i = 0; i < 6; i++) {
        params[i*4] = joints[i] * DEG_TO_RAD;
    }
    params[8] -= M_PI_2; // custom offsets
    params[20] += M_PI; 
}

void set_dh_theta(float *new_params, float *old_params, const float *joints) {
    for (int i = 0; i < 6; i++) {
        new_params[i] = old_params[i] + joints[i] * DEG_TO_RAD;
    }
    //params[1] -= PI_2;
    //params[2] += PI_2; 
}

void gen_identity_matrix(float * matrix, const int rows, const int cols) {
    for (int i = 0; i < rows * cols; i++) {
        if (i % (cols + 1) == 0) {
            matrix[i] = 1;
        }
        else {
            matrix[i] = 0; 
        }
    }
}

void pose_to_transformation_matrix(const float *pose, float *transform) {
    transform[0] = cos(pose[3])*cos(pose[4])*cos(pose[5])-sin(pose[3])*sin(pose[5]);
    transform[1] = -cos(pose[3])*cos(pose[4])*sin(pose[5])-sin(pose[3])*cos(pose[5]);
    transform[2] = cos(pose[3])*sin(pose[4]);
    transform[3] = pose[0];
 
    transform[4] = sin(pose[3])*cos(pose[4])*cos(pose[5])+cos(pose[3])*sin(pose[5]);
    transform[5] = -sin(pose[3])*cos(pose[4])*sin(pose[5])+cos(pose[3])*cos(pose[5]);
    transform[6] = sin(pose[3])*sin(pose[4]);
    transform[7] = pose[1];
  
    transform[8] = -sin(pose[4])*cos(pose[5]);
    transform[9] = sin(pose[4])*sin(pose[5]);
    transform[10] = cos(pose[4]);
    transform[11] = pose[2];

    transform[12] = 0.0;
    transform[13] = 0.0;
    transform[14] = 0.0;
    transform[15] = 1.0;
}

int sign(float arg) {
    if (arg >= 0) return 1; 
    else return -1; 
}

void invtran(float* in, float* out) {
  out[0] = in[0];
  out[1] = in[4];
  out[2] = in[8];
  out[3] = -in[0]*in[3]-in[4]*in[7]-in[8]*in[11];
  out[4] = in[1];
  out[5] = in[5];
  out[6] = in[9];
  out[7] = -in[1]*in[3]-in[5]*in[7]-in[9]*in[11];
  out[8] = in[2];
  out[9] = in[6];
  out[10] = in[10];
  out[11] = -in[2]*in[3]-in[6]*in[7]-in[10]*in[11];
  out[12] = 0.0;
  out[13] = 0.0;
  out[14] = 0.0;
  out[15] = 1.0;
}

void solve_FK(const float *joints, float *pose) {
    float J[6][16];
    float R[6][16];
    float TF[16];
    float ROT_F[16]; 
    float new_dh_theta[6]; 
    set_dh_theta(new_dh_theta, dh_theta, joints);
    //print_matrix(dh_theta, 6, 1);
    gen_identity_matrix(TF, 4, 4);
    for (int i = 0; i < 6; i++) { 
        float theta = new_dh_theta[i];
        float alpha = dh_alpha[i];
        float d = dh_d[i];
        float a = dh_a[i];
        J[i][0] = cos(theta);
        J[i][1] = -sin(theta) * cos(alpha);
        J[i][2] = sin(theta) * sin(alpha);
        J[i][3] = a * cos(theta);
        J[i][4] = sin(theta);
        J[i][5] = cos(theta) * cos(alpha);
        J[i][6] = -cos(theta) * sin(alpha);
        J[i][7] = a * sin(theta);
        J[i][8] = 0;
        J[i][9] = sin(alpha);
        J[i][10] = cos(alpha);
        J[i][11] = d;
        J[i][12] = 0;
        J[i][13] = 0;
        J[i][14] = 0;
        J[i][15] = 1;
    }
    mat_multiply(J[0], J[1], R[1], 4, 4, 4);
    for (int i = 2; i < 6; i++) {
        mat_multiply(R[i-1], J[i], R[i], 4, 4, 4);
    }
    mat_multiply(R[5], TF, ROT_F, 4, 4, 4);

    pose[0] = ROT_F[3];
    pose[1] = ROT_F[7];
    pose[2] = ROT_F[11];
    //pose.a[3] = atan2(-ROT_F[8], sqrt(pow(ROT_F[0], 2) + pow(ROT_F[4], 2)));
    //pose.a[4] = atan2(ROT_F[9]/cos(pose.a[3]), ROT_F[10]/cos(pose.a[3]));
    //pose.a[5] = atan2(ROT_F[4]/cos(pose.a[3]), ROT_F[0]/cos(pose.a[3]));
    pose[4] = atan2(sqrt(ROT_F[8] * ROT_F[8] + ROT_F[9] * ROT_F[9]), ROT_F[10]);
    pose[3] = atan2(ROT_F[6] / sin(pose[4]), ROT_F[2] / sin(pose[4]));
    pose[5] = atan2(ROT_F[9] / sin(pose[4]), -ROT_F[8] / sin(pose[4]));

    for (int i = 3; i < 6; i++) {
        pose[i] *= RAD_TO_DEG;
    }
}

void solve_IK(float *pose, float *joints) {
    pose[3] *= DEG_TO_RAD;
    pose[4] *= DEG_TO_RAD; 
    pose[5] *= DEG_TO_RAD; 
    float ROT_F[16];
    pose_to_transformation_matrix(pose, ROT_F);
    //print_matrix(ROT_F, 4, 4);
    float P_org_04[3] = {ROT_F[3] - ROT_F[2] * dh_d[5], ROT_F[7] - ROT_F[6] * dh_d[5], ROT_F[11] - ROT_F[10] * dh_d[5]};
    //print_matrix(P04, 3, 1);
    joints[0] = atan2(P_org_04[1], P_org_04[0]);
    float vector_O1O4[3] = {P_org_04[0], P_org_04[1], P_org_04[2] - dh_d[0]};
    float abs_O1O4 = sqrt(pow(vector_O1O4[2], 2) + pow(vector_O1O4[0], 2) + pow(vector_O1O4[1],2));
    //printf("abs_0104: %f \n", abs_O1O4);
    float j3_sols[2];
    j3_sols[0] = acos(-(pow(dh_a[1],2)+pow(dh_d[3],2)-pow(abs_O1O4,2))/(2*dh_a[1]*dh_d[3])) - PI_2; 
    //printf("sol1: %f\n", j3_sols[0]);
    j3_sols[1] = -acos(-(pow(dh_a[1],2)+pow(dh_d[3],2)-pow(abs_O1O4,2))/(2*dh_a[1]*dh_d[3])) + PI_2;
    joints[2] = j3_sols[0];
    float beta = atan2(P_org_04[2]-dh_d[0], P_org_04[0]*cos(joints[0])+P_org_04[1]*sin(joints[0]));
    float upsilon  = acos((pow(dh_a[1],2)-pow(dh_d[3],2)+pow(abs_O1O4,2))/(2*dh_a[1]*abs_O1O4));
    joints[1] = PI_2 - beta - upsilon; 
    float J[3][16];
    float R[3][16];
    for (int i = 0; i < 3; i++) {
        float theta = dh_theta[i] + joints[i];
        float alpha = dh_alpha[i];
        float d = dh_d[i];
        float a = dh_a[i];
        J[i][0] = cos(theta);
        J[i][1] = -sin(theta) * cos(alpha);
        J[i][2] = sin(theta) * sin(alpha);
        J[i][3] = a * cos(theta);
        J[i][4] = sin(theta);
        J[i][5] = cos(theta) * cos(alpha);
        J[i][6] = -cos(theta) * sin(alpha);
        J[i][7] = a * sin(theta);
        J[i][8] = 0;
        J[i][9] = sin(alpha);
        J[i][10] = cos(alpha);
        J[i][11] = d;
        J[i][12] = 0;
        J[i][13] = 0;
        J[i][14] = 0;
        J[i][15] = 1;
    }
    mat_multiply(J[0], J[1], R[1], 4, 4, 4); // R2 = J1 * J2
    mat_multiply(R[1], J[2], R[2], 4, 4, 4); // R3 = J2 * J3
    float inv_R3[16]; 
    invtran(R[2], inv_R3);
    float T36[16];
    mat_multiply(inv_R3, ROT_F, T36, 4, 4, 4);
    //print_matrix(T36, 4, 4);
    joints[3]=atan2(-T36[6], -T36[2]); // Jik(4)=atan2(-T36(2,3),-T36(1,3));
    joints[4]=atan2(sqrt(T36[2]*T36[2]+T36[6]*T36[6]), T36[10]); // Jik(5)=atan2(sqrt(T36(1,3)^2+T36(2,3)^2),T36(3,3));
    joints[5]=atan2(-T36[9], T36[8]); // Jik(6)=atan2(-T36(3,2),T36(3,1));

    for (int i = 0; i < 6; i++) joints[i] *= RAD_TO_DEG;
}

/// serial stuff

void custom_serial_read2() {
  while (Serial.available() > 0) {
    char indata = Serial.read(); 
    if (indata == '<') {
      msg = "";
      num_segs = 1; 
    }
    else if (indata == '>') {
      new_command = true;
      break;
    }
    else if (indata == ':') {
      num_segs++; 
      msg += indata; 
    }
    else {
      msg += indata; 
    }
  }
}

void print_data2() {
  if (!new_command) return;
  
  Serial.println(msg);
  
}

void parse_command2() {
  if (!new_command) return;
  
  String *segmented_msg = new String[num_segs];
  for (int i = 0; i < num_segs; i++) segmented_msg[i] = "";
  split_string(msg, segmented_msg);
  //for (int i = 0; i < num_segs; i++) {
    //Serial.println(segmented_msg[i]);
  //}

  command_id = segmented_msg[0].toInt();
  if (command_id == 0) {
    motor_num = segmented_msg[1].toInt();
    ena_flags[motor_num] = segmented_msg[2].toInt();
    process_ena_flags(); 
  }
  else if (command_id == 1) {
    ena_straight_line_path = 0; //segmented_msg[1].toInt(); 
    path_speed_msg = segmented_msg[2].toFloat();
    for (int i = 0; i < 6; i++) {
      IK_pose[i] = segmented_msg[i+3].toFloat(); 
      //Serial.println(IK_pose[i]);
    }
    if (ena_straight_line_path == 0) {
      jog_position(cur_joint_angs, IK_pose);
    }
    else {
      go_line_path(cur_pose, IK_pose, 0.2, 50);
    }
  }
  else if (command_id == 2) {
    for (int i = 0; i < 6; i++) {
      new_joint_pose[i] = segmented_msg[i+3].toFloat();
    }
    joint_space_move_accel(cur_joint_angs, new_joint_pose, true);
  } 
  else if (command_id == 3) {
    if (segmented_msg[1] == "start") home_on_startup(); 
    else if (segmented_msg[1] == "stop") home_on_shutdown(); 
    else if (segmented_msg[1] == "clearqueue") queue_len = 0; 
    else if (segmented_msg[1] == "runqueue") run_queue(500);
    else if (segmented_msg[1] == "calibrateseeker") calibrate_seeker(); 
  }
  else if (command_id == 4) {
    for (int i = 0; i < 6; i++) {
      new_joint_pose[i] = segmented_msg[i+1].toFloat(); 
      Serial.println(new_joint_pose[i]);
    }
    int POSE_OR_ANGS = segmented_msg[7].toInt();
    Serial.println("pose_or_angs: " + String(POSE_OR_ANGS));
    append_queue(new_joint_pose, POSE_OR_ANGS);
  }
  
  new_command = false;  
}

void split_string(String str, String *strs) {
  int total_len = str.length();
  int seg_idx = 0;  
  //Serial.println(total_len);
  for (int i = 0; i < total_len; i++) {
    if (str[i] == ':') {
      seg_idx++; 
      continue; 
    }
    strs[seg_idx] += str[i];
  }
}

void append_queue(float *pose_to_append, int pose_or_angs) {
  for (int i = 0; i < 6; i++) {
    joint_ang_queue[queue_len].a[i] = pose_to_append[i];
  }
  joint_ang_queue[queue_len].p = pose_or_angs;
  queue_len++; 
  Serial.println("Queue length: " + String(queue_len));
}

void run_queue(const int delay_btwn_points) {
  for (int i = 0; i < queue_len; i++) {
    float temp_data[6];
    for (int j = 0; j < 6; j++) {
      temp_data[j] = joint_ang_queue[i].a[j];
    }
    if (joint_ang_queue[i].p == 0) {
      joint_space_move_accel(cur_joint_angs, temp_data, true);
    }
    else {
      jog_position(cur_joint_angs, temp_data);
    }
    delay(delay_btwn_points);
  }
}

void calibrate_seeker() {
  
  //const float move1[6] = {0, 0, -90, 0, 0, 0};
  //joint_space_move_accel(cur_joint_angs, move1);
  //delay(3000);
  step_individual_motor(3, -180, 400);
  delay(500);
  //step_individual_motor(3, 180, 400);
 

  
  //step_individual_motor(3, 359, 100);
  //delay(500);
  //const float move4[6] = {0, 0, 0, 0, -90, 0};
  //joint_space_move_accel(cur_joint_angs, move4);
  //step_individual_motor(3, -359, 100);
  //delay(500);


  go_home(); 

}

void FreeMem(){ // for Teensy 3.0
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) &stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is (approximately) the free, available ram.
    Serial.println("Free RAM: " + String(stackTop - heapTop));
}