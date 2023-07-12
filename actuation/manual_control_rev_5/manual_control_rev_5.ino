#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923

const int pul[6] = {2, 5, 8,  11, 30, 33};
const int dir[6] = {3, 6, 9,  12, 31, 34};
const int ena[6] = {4, 7, 10, 13, 32, 35};
int ena_flags[6] = {1, 1, 1, 1, 1, 1};

const float STEPS_PER_RAD[6] = {3055.774907, 5092.958179, 5092.958179, 5092.958179, 2037.183271, 509.295818};
const float joint_maxvel_radps[6] = {0.1, 0.1, 0.2, 1, 1, 1};
float STEPS_PER_DEG[6];
float DEGS_PER_STEP[6];

float cur_joint_angs[6] = {0, 0, 0, 0, 0, 0}; 
float cur_pose[6] = {174.39, 0, 467.01, 0, 90, 180};

float dh_theta[6] = {0, -PI_2, 0, 0, 0, 0};
const float dh_alpha[6] = {-PI_2, 0, -PI_2, PI_2, -PI_2, 0};
const float dh_d[6] = {167.01, 0, 0, 174.39, 0, 0};
const float dh_a[6] = {0, 300, 0, 0, 0, 0};

//const float RAD_TO_DEG = 57.295777754771045;
//const float DEG_TO_RAD = 0.01745329251;

// J1: POSITIVE ANGULAR VELOCITY IS CW, DIR = HIGH IS CW
// J2: POSITIVE ANGULAR VELOCITY IS CCW, DIR = HIGH IS CCW
// J3: POSITIVE ANG VEL IS CW, DIR = HIGH IS CW

struct point_3D {
  float x, y, z; 
};

int counter; 
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
int pose_idx; 
bool new_command = false;

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
}

int count = 0;
void loop() {
  
  time1 = micros(); 
  custom_serial_read(); 
  parse_command();
  if (command_id == 0) {
    step_individual_motor(motor_num, msg_float);
  }
  
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

void step_individual_motor(const int motor_idx, const float speed_rads) {
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

void joint_space_move_no_accel(const float *start_pose, const float *end_pose) {
  Serial.println("moving with no accel");
  float steps_to_move[6];
  int calc_step_delay[6]; 
  float max_steps = 0;
  int max_step_index = 0; 
  for (int i = 0; i < 6; i++) {
    steps_to_move[i] = (end_pose[i] - start_pose[i]) * DEG_TO_RAD * STEPS_PER_RAD[i];
    Serial.println(steps_to_move[i]); 
    if (abs(steps_to_move[i]) > max_steps) {
      max_steps = abs(steps_to_move[i]);
      max_step_index = i; 
    }
  }
  Serial.println("Max steps: " + String(max_steps));
  calc_step_delay[max_step_index] = 1000000/(joint_maxvel_radps[max_step_index] * STEPS_PER_RAD[max_step_index]);
  //Serial.println(max_step_index);
  //Serial.println(calc_step_delay[max_step_index]);
  float total_move_time = max_steps * calc_step_delay[max_step_index];
  //Serial.println(total_move_time);
  for (int i = 0; i < 6; i++) {
    if (steps_to_move[i] == 0) continue; 
    calc_step_delay[i] = total_move_time / abs(steps_to_move[i]);
    //Serial.println(String(calc_step_delay[i]) + ":" + String(steps_to_move[i]));
  }
  if (steps_to_move[0] < 0) digitalWrite(dir[0], HIGH);
  else digitalWrite(dir[0], LOW);
  if (steps_to_move[1] < 0) digitalWrite(dir[1], LOW);
  else digitalWrite(dir[1], HIGH);
  if (steps_to_move[2] > 0) digitalWrite(dir[2], LOW);
  else digitalWrite(dir[2], HIGH);
  
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
    cur_joint_angs[i] = end_pose[i];
  }
}

void jog_position(const float *start_pose, const float *end_pose) {
  if (ena_straight_line_path == 0) {
    float IK_out[6] = {0, 0, 0, 0, 0, 0};
    solve_IK(end_pose, IK_out);
    print_matrix(IK_out, 1, 6);
    joint_space_move_no_accel(start_pose, IK_out);
  }
}

/*
void joint_space_move_with_accel(const float *start_pose, const float *end_pose) {
  float steps_to_move[6];
  float max_steps = 0;
  int max_step_index = 0; 
  for (int i = 0; i < 6; i++) {
    steps_to_move[i] = (end_pose[i] - start_pose[i]) * DEG_TO_RAD * STEPS_PER_RAD[i];
    Serial.println(steps_to_move[i]); 
    if (abs(steps_to_move[i]) > max_steps) {
      max_steps = abs(steps_to_move[i]);
      max_step_index = i; 
    }
  }

  //Serial.println(steps_to_move[0]);
  if (steps_to_move[0] < 0) digitalWrite(dir[0], HIGH);
  else digitalWrite(dir[0], LOW);
  //Serial.println(steps_to_move[1]);
  if (steps_to_move[1] < 0) digitalWrite(dir[1], HIGH);
  else digitalWrite(dir[1], LOW);
  if (steps_to_move[2] < 0) digitalWrite(dir[2], HIGH);
  else digitalWrite(dir[2], LOW);

  int j0_len = abs(steps_to_move[0]);
  float *joint0_delay_array = new float[j0_len];
  generate_accel_curve(0, joint0_delay_array, j0_len);

  int j1_len = abs(steps_to_move[1]);
  float *joint1_delay_array = new float[j1_len];
  generate_accel_curve(1, joint1_delay_array, j1_len);

  int j2_len = abs(steps_to_move[2]);
  float *joint2_delay_array = new float[j2_len];
  generate_accel_curve(2, joint2_delay_array, j2_len);

  int array_lengths[3] = {j0_len, j1_len, j2_len};
  scale_acceleration_arrays(joint0_delay_array, joint1_delay_array, joint2_delay_array, array_lengths);

  float time_sum = 0.0; 
  for (int i = 0; i < j0_len; i++) { 
    time_sum += joint0_delay_array[i];
    joint0_delay_array[i] = time_sum; 
  }
  Serial.println("J1 will take: " + String(time_sum) + " microseconds");
  delay(10);
  time_sum = 0.0; 
  for (int i = 0; i < j1_len; i++) { 
    time_sum += joint1_delay_array[i];
    joint1_delay_array[i] = time_sum; 
  }
  Serial.println("J2 will take: " + String(time_sum) + " microseconds");
  delay(10);
  time_sum = 0.0; 
  for (int i = 0; i < j2_len; i++) { 
    time_sum += joint2_delay_array[i];
    joint2_delay_array[i] = time_sum; 
  }
  Serial.println("J3 will take: " + String(time_sum) + " microseconds");
  delay(10);

  int steps_moved[6] = {0, 0, 0, 0, 0, 0};
  int half_steps_moved[6] = {0, 0, 0, 0, 0, 0};
  float full_checkpoint0 = joint0_delay_array[0];
  float half_checkpoint0 = joint0_delay_array[0]*0.5;  
  float full_checkpoint1 = joint1_delay_array[0];
  float half_checkpoint1 = joint1_delay_array[0]*0.5; 
  float full_checkpoint2 = joint2_delay_array[0];
  float half_checkpoint2 = joint2_delay_array[0]*0.5; 

  unsigned long time1 = micros();
  while (steps_moved[max_step_index] < abs(steps_to_move[max_step_index])) {
      
    if (steps_to_move[0] != 0) {
      //if (micros() - time1 > joint0_half_delay_array[half_steps_moved[max_step_index]]) {
      if (micros() - time1 > half_checkpoint0) {
        digitalWrite(pul[0], HIGH);
        half_steps_moved[0]++;
        half_checkpoint0 = full_checkpoint0 + (joint0_delay_array[half_steps_moved[0]] - full_checkpoint0)*0.5; 
        //Serial.println(half_steps_moved[0]);
      }
      //else if (micros() - time1 > joint0_delay_array[steps_moved[max_step_index]]) {
      else if (micros() - time1 > full_checkpoint0) {
        digitalWrite(pul[0], LOW);
        steps_moved[0]++;
        full_checkpoint0 = joint0_delay_array[steps_moved[0]];
      }
    }

    if (steps_to_move[1] != 0) {
      if (micros() - time1 > half_checkpoint1) {
        digitalWrite(pul[1], HIGH);
        half_steps_moved[1]++;
        half_checkpoint1 = full_checkpoint1 + (joint1_delay_array[half_steps_moved[1]] - full_checkpoint1)*0.5; 
        //Serial.println(half_steps_moved[0]);
      }
      //else if (micros() - time1 > joint0_delay_array[steps_moved[max_step_index]]) {
      else if (micros() - time1 > full_checkpoint1) {
        digitalWrite(pul[1], LOW);
        steps_moved[1]++;
        full_checkpoint1 = joint1_delay_array[steps_moved[1]];
      }
    }

    if (steps_to_move[2] != 0) {
      if (micros() - time1 > half_checkpoint2) {
        digitalWrite(pul[2], HIGH);
        half_steps_moved[2]++;
        half_checkpoint2 = full_checkpoint2 + (joint2_delay_array[half_steps_moved[2]] - full_checkpoint2)*0.5; 
        //Serial.println(half_steps_moved[0]);
      }
      //else if (micros() - time1 > joint0_delay_array[steps_moved[max_step_index]]) {
      else if (micros() - time1 > full_checkpoint2) {
        digitalWrite(pul[2], LOW);
        steps_moved[2]++;
        full_checkpoint2 = joint2_delay_array[steps_moved[2]];
      }
    }
  }

  FreeMem();

  free(joint0_delay_array);
  free(joint1_delay_array);
  free(joint2_delay_array);

  FreeMem();

  //Serial.println("Steps moved: " + String(steps_moved[0]));

  cur_joint_angs[0] = end_pose[0];
  cur_joint_angs[1] = end_pose[1];
  cur_joint_angs[2] = end_pose[2];
}

const float step_jerk[6] = {0.08, 0.05, 0.05, 0.1, 0.1, 0.1};
const int max_step_delay[6] = {4000, 6000, 4000, 4000, 4000, 4000};
const int min_step_delay[6] = {400, 800, 400, 100, 100, 100};
void generate_accel_curve(int motor_num, float *delay_array, const int num_steps) {
  if (num_steps == 0) return; 
   float step_accel = 0;
   float cur_time = 0;
   float step_delay = max_step_delay[motor_num];
   float prev_step_delay = max_step_delay[motor_num]; 
   bool accelerating = true;
   bool acceleration_finished = false;
   int half_steps = num_steps * 0.5;
   int deceleration_target = (max_step_delay[motor_num] - min_step_delay[motor_num])*0.5 + min_step_delay[motor_num];
  for (int i = 0; i < half_steps+1; i++) { 
    if (step_delay > deceleration_target && accelerating) step_accel += step_jerk[motor_num];
    else if (step_delay <= deceleration_target && accelerating) step_accel -= step_jerk[motor_num];
    step_delay -= step_accel; 
    if (step_delay < min_step_delay[motor_num] || step_delay - prev_step_delay > 0) {
      accelerating = false;
      step_delay = min_step_delay[motor_num]; 
    }
    delay_array[i] = step_delay;
    delay_array[num_steps - 1 - i] = step_delay;
    prev_step_delay = step_delay; 
  }
  /*
  float running_delay = 0; 
  for (int i = 0; i < num_steps; i++) { 
    delay_array[i] = floor(delay_array[i]);
    running_delay += delay_array[i];
    delay_array[i] = running_delay; 
    
    //Serial.println(String(delay_array[i]) + ":" + String(half_delay_array[i]));
  }
  
}

void scale_acceleration_arrays(float *array0, float *array1, float *array2, const int *array_lengths) {
  
  int maximum_time_index = 0;
  float array_total_time[6] = {0, 0, 0, 0, 0, 0};

  Serial.println("Steps to take: " + String(array_lengths[0]) + "\t" + String(array_lengths[1]) + "\t" + String(array_lengths[2]));
  
  if (array_lengths[0] != 0) for (int i = 0; i < array_lengths[0]; i++) array_total_time[0] += array0[i];;
  Serial.println("0: "+String(array_total_time[0]));
  
  if (array_lengths[1] != 0) for (int i = 0; i < array_lengths[1]; i++) array_total_time[1] += array1[i];;
  Serial.println("1: "+String(array_total_time[1]));
  
  if (array_lengths[2] != 0) for (int i = 0; i < array_lengths[2]; i++) array_total_time[2] += array1[i];;
  Serial.println("2: "+String(array_total_time[2]));
  
  for (int i = 0; i < 3; i++) {
    if (array_total_time[i] > array_total_time[maximum_time_index]) {
      //Serial.println(maximum_time);
      maximum_time_index = i; 
    }
  }

  FreeMem(); 

  float factor0 = 0;
  float factor1 = 0;
  float factor2 = 0;

  Serial.println("max time: " + String(array_total_time[maximum_time_index]) + " max time idx: " + String(maximum_time_index));

  if (array_total_time[0] != 0) factor0 = array_total_time[maximum_time_index] / array_total_time[0];
  if (array_total_time[1] != 0) factor1 = array_total_time[maximum_time_index] / array_total_time[1];
  if (array_total_time[2] != 0) factor2 = array_total_time[maximum_time_index] / array_total_time[2];

  Serial.println("Factors: " + String(factor0) + "\t" + String(factor1) + "\t" + String(factor2));

  for (int i = 0; i < array_lengths[0]; i++) array0[i] = array0[i] * factor0;
  for (int i = 0; i < array_lengths[1]; i++) array1[i] = array1[i] * factor1;
  for (int i = 0; i < array_lengths[2]; i++) array2[i] = array2[i] * factor2;

  for (int i = 0; i < 6; i++) array_total_time[i] = 0;
  for (int i = 0; i < array_lengths[0]; i++) array_total_time[0] += array0[i];
  Serial.println("0: "+String(array_total_time[0]));
  for (int i = 0; i < array_lengths[1]; i++) array_total_time[1] += array1[i];
  Serial.println("1: "+String(array_total_time[1]));
  for (int i = 0; i < array_lengths[2]; i++) array_total_time[2] += array1[i];
  Serial.println("2: "+String(array_total_time[2]));
}

void finalize_acceleration_array(float *in_array, int array_length) {
  float time_sum = 0.0; 
  for (int i = 0; i < array_length; i++) { 
    //array[i] = floor(array[i]);
    time_sum += in_array[i];
    in_array[i] = time_sum; 
    //Serial.println(String(delay_array[i]) + ":" + String(half_delay_array[i]));
  }
  Serial.println("Movement will take: " + String(time_sum) + " microseconds");
}

float sum_array_time(const float *array, const int length) {
  float time = 0;
  for (int i = 0; i < length; i++) {
    time += array[i];
  }
  return time; 
}
*/

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

void go_line_path(const float *start_pose, const float *end_pose, const float stepsize_mm) {
  float vector_mag = sqrt(pow(end_pose[0]-start_pose[0],2)+pow(end_pose[1]-start_pose[1],2)+pow(end_pose[2]-start_pose[2],2));
  Serial.println("Vector mag: " + String(vector_mag));
  int num_steps = vector_mag / stepsize_mm; 

  float unitvector[3]; 
  unitvector[0] = (end_pose[0] - start_pose[0]) / vector_mag;
  unitvector[1] = (end_pose[1] - start_pose[1]) / vector_mag;
  unitvector[2] = (end_pose[2] - start_pose[2]) / vector_mag;

  point_3D *path_array = new point_3D[num_steps];
  for (int i = 0; i < num_steps; i++) {
    path_array[i].x = start_pose[0] + unitvector[0] * stepsize_mm * i;
    path_array[i].y = start_pose[1] + unitvector[1] * stepsize_mm * i;
    path_array[i].z = start_pose[2] + unitvector[2] * stepsize_mm * i;
    //Serial.println("Point " + String(i) + ": (" + String(path_array[i].x) + ", " + String(path_array[i].y) + ", " + String(path_array[i].z) + ")" );
  }

  Serial.print("Cur joint angs: ");
  print_matrix(cur_joint_angs, 1, 6);

  for (int i = 0; i < num_steps; i++) {
    float IK_in[6] = {path_array[i].x, path_array[i].y, path_array[i].z, start_pose[3], start_pose[4], start_pose[5]};
    float IK_out[6] = {0, 0, 0, 0, 0, 0};
    solve_IK(IK_in, IK_out);
    //Serial.print("IK out");
    //print_matrix(IK_out, 1, 6);
    float step_difference[6];
    for (int i = 0; i < 6; i++) {
      step_difference[i] = floor((IK_out[i] - cur_joint_angs[i]) * STEPS_PER_DEG[i]);
      cur_joint_angs[i] += step_difference[i] * DEGS_PER_STEP[i];
    }
    //Serial.print("Step difference: ");
    //print_matrix(step_difference, 1, 6);

    
    if (step_difference[0] < 0) digitalWrite(dir[0], HIGH);
    else digitalWrite(dir[0], LOW);
    if (step_difference[1] < 0) digitalWrite(dir[1], LOW);
    else digitalWrite(dir[1], HIGH);
    if (step_difference[2] > 0) digitalWrite(dir[2], LOW);
    else digitalWrite(dir[2], HIGH);

    
    int steps_this_point = 0; 
    for (int i = 0; i < 3; i++) steps_this_point += abs(step_difference[i]);
    //for (int i = 0; i < 3; i++) Serial.println(abs(step_difference[i]));
    unsigned int this_step_delay = 1000000 / (100 / stepsize_mm * steps_this_point) * 0.5;


    
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < abs(step_difference[i]); j++) {
        delayMicroseconds(this_step_delay);
        digitalWrite(pul[i], HIGH);
        delayMicroseconds(this_step_delay);
        digitalWrite(pul[i], LOW);
      }
    }
    
    
  }

  // set all important globals
  Serial.print("Final joint angs: ");
  print_matrix(cur_joint_angs, 1, 6);

  solve_FK(cur_joint_angs, cur_pose);
  Serial.print("Final pose: ");
  print_matrix(cur_pose, 1, 6);

  free(path_array);

}

/// serial stuff

void custom_serial_read() {
  while (Serial.available() > 0) {
    const char indata[1] = {Serial.read()};
    if (indata[0] == '<') {
      counter = 0;
      msg = "";
    }
    else if (indata[0] == '>') {
      if (command_id == 0) msg_float = msg.toFloat();
      else if (command_id == 1)  {
        path_speed = path_speed_msg.toFloat();
        for (int i = 0; i < 6; i++) {
          IK_pose[i] = IK_pose_msg[i].toFloat(); 
          IK_pose_msg[i] = "";
        }  
      }
      new_command = true;
      //Serial.println("Read took " + String(micros() - time1) + " microseconds");
      //print_data(command_id);
      break;
    }
    else if (indata[0] == ':') counter++;
    else if (counter == 0) command_id = atoi(indata);
    else if (command_id == 0 && counter == 1) motor_num = atoi(indata);
    else if (command_id == 0 && counter == 2) ena_flags[motor_num] = atoi(indata);
    else if (command_id == 0 && counter == 3) msg += indata[0];
    else if (command_id == 1 && counter == 1) ena_straight_line_path = atoi(indata);
    else if (command_id == 1 && counter == 2) path_speed_msg += indata[0];
    else if (command_id == 1 && counter == 3) IK_pose_msg[0] += indata[0]; 
    else if (command_id == 1 && counter == 4) IK_pose_msg[1] += indata[0]; 
    else if (command_id == 1 && counter == 5) IK_pose_msg[2] += indata[0]; 
    else if (command_id == 1 && counter == 6) IK_pose_msg[3] += indata[0]; 
    else if (command_id == 1 && counter == 7) IK_pose_msg[4] += indata[0]; 
    else if (command_id == 1 && counter == 8) IK_pose_msg[5] += indata[0]; 
  }
}

void print_data(const int command_id) {
  Serial.println("Command id: " + String(command_id));
  if (command_id == 0) {
    Serial.println("Motor num: " + String(motor_num));
    Serial.println("Ena flag: " + String(ena_flags[motor_num]));
    Serial.println("Angular velocity: " + String(msg_float));
  }
  else if (command_id == 1) {
    Serial.println("Ena straight line path: " + String(ena_straight_line_path));
    Serial.println("Path speed: " + String(path_speed));
    Serial.print("Pose: [");
    for (int i = 0; i < 5; i++) {
      Serial.print(String(IK_pose[i], 6));
      Serial.print(", ");
    }
    Serial.print(String(IK_pose[5], 6));
    Serial.println("]");
  }
}

void parse_command() {
  if (!new_command) return; 
  if (command_id == 0) {
    process_ena_flags(); 
  }
  else if (command_id == 1) {
    print_data(command_id); 
    jog_position(cur_joint_angs, IK_pose);
    //go_line_path(cur_pose, IK_pose, 0.1);
  }
  new_command = false;
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