// Written by Jonathan Kampia, jonathankampia@gmail.com
// Explanatory video should be up on my youtube channel soon: https://www.youtube.com/channel/UCfLUm0v01AEeCJcA7EN2Glw

// Credit to (and for additional explanations):
// Skyentific's tiny 6dof robot arm code found here https://github.com/SkyentificGit/SmallRobotArm 
// Skyentific's ZYZ Euler explanation: https://www.youtube.com/watch?v=Sgsn2CM3bjY&t=305s&ab_channel=Skyentific
// Chris Annin's AR2 repository found here https://github.com/Chris-Annin/AR2
// Chris Annin's IK series: https://www.youtube.com/watch?v=FIx6olybAeQ&ab_channel=ChrisAnnin
// This paper https://iopscience.iop.org/article/10.1088/1742-6596/2338/1/012089/pdf fully geometric approach to 6dof IK

// My solution & kinematic model very closely resembles these resources, so if you have any confusion be sure to reference them

//////////////////////////////////// dh params & constants /////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <iostream>
#include <vector>

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923
const float RAD_TO_DEG = 57.295777754771045;
const float DEG_TO_RAD = 0.01745329251;
const double STEPS_PER_DEG[6] = {88.88888888888889, 88.88888888888889, 88.88888888888889, 88.88888888888889, 88.88888888888889, 88.88888888888889};
const double DEGS_PER_STEP[6] = {0.01125, 0.01125, 0.01125, 0.01125, 0.01125, 0.01125};

float dh_theta[6] = {0, -PI_2, 0, 0, 0, 0};
const float dh_alpha[6] = {-PI_2, 0, -PI_2, PI_2, -PI_2, 0};
const float dh_d[6] = {169.77, 0, 0, 222.63, 0, 36.25};
const float dh_a[6] = {0, 305, 0, 0, 0, 0}; 

struct point_3D {
    float x, y, z;
};

//////////////////////////////////// array utility functions ////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////// IK and FK solvers //////////////////////////////////////////////////////////////////////

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

void generate_line_path(std::vector<point_3D> &path_vec, point_3D start_point, point_3D end_point, const float stepsize_mm) {
  float vector_mag = sqrt(pow(end_point.x-start_point.x,2)+pow(end_point.y-start_point.y,2)+pow(end_point.z-start_point.z,2)); // mm
  int num_steps = vector_mag / stepsize_mm; 
  point_3D unitvector = {(end_point.x - start_point.x) / vector_mag, (end_point.y - start_point.y) / vector_mag, (end_point.z - start_point.z) / vector_mag};
  //printf("[ %f, %f, %f ] \n", unitvector.x, unitvector.y, unitvector.z);
  for (int i = 1; i <= num_steps; i++) {
    point_3D pushback_point = {start_point.x+unitvector.x*stepsize_mm*i, start_point.y+unitvector.y*stepsize_mm*i, start_point.z+unitvector.z*stepsize_mm*i};
    path_vec.push_back(pushback_point);
  }
  float tgt_xyz_error = sqrt(pow(end_point.x-path_vec.back().x,2)+pow(end_point.y-path_vec.back().y,2)+pow(end_point.z-path_vec.back().z,2));
  printf("num steps: %i \t vector size: %i \t xyz error: %f \n", num_steps, int(path_vec.size()), tgt_xyz_error);
  int num_kb = sizeof(point_3D) * int(path_vec.size()) * 0.001;  
  printf("xyz vector size: %iK \n", num_kb);
}

void execute_path(std::vector<point_3D> &path_vec, point_3D start_point, const float *effector_pose) {
  float prev_IK_out[6];
  float IK_out[6];
  float start_IK_in[6] = {start_point.x, start_point.y, start_point.z, effector_pose[0], effector_pose[1], effector_pose[2]};
  float final_IK_in[6] = {path_vec.back().x, path_vec.back().y, path_vec.back().z, effector_pose[0], effector_pose[1], effector_pose[2]};
  solve_IK(start_IK_in, prev_IK_out);

  while (path_vec.size() > 0) {
    // pseudocode go-to-point path_vec.front();
    // should use straight line path movement function that accepts mm/s velocity argument  
   
    float IK_in[6] = {path_vec.front().x, path_vec.front().y, path_vec.front().z, effector_pose[0], effector_pose[1], effector_pose[2]};
    solve_IK(IK_in, IK_out);

    float angle_difference[6]; 
    for (int i = 0; i < 6; i++) {
      angle_difference[i] = floor((IK_out[i] - prev_IK_out[i]) * STEPS_PER_DEG[i]);
      prev_IK_out[i] = prev_IK_out[i] + angle_difference[i] * DEGS_PER_STEP[i];
    }
    //print_matrix(angle_difference, 6, 1);
    //printArray(prev_IK_out, 6);

    path_vec.erase(path_vec.begin());
    // pseudocode for micros() delay to satisfy mm/s velocity cmd
  }

  printf("Path end pose: ");
  print_matrix(prev_IK_out, 6, 1);
  
  float final_IK_out[6]; 
  solve_IK(final_IK_in, final_IK_out);
  printf("Initial calculated end pose: ");
  print_matrix(final_IK_out, 6, 1);

  float correction_angles[6]; 
  for (int i = 0; i < 6; i++) correction_angles[i] = final_IK_out[i] - prev_IK_out[i]; 
  printf("Correction angles: ");
  print_matrix(correction_angles, 6, 1);

  float test[6];
  solve_FK(prev_IK_out, test);
  print_matrix(test, 6, 1);
}

int main(int argc, char **argv) {
    
    // Example of how to request an IK solve for a position in space, and request an FK solve for some joint angles
    // solve_IK takes an input (IK_in, in this case) and writes joint angle solutions to the second argument (IK_out)
    // args must be 6-element float arrays
    // input array is ordered [X_0, Y_0, Z_0, Z_rot, Y_rot, Z_rot] where _0 and _rot denote reference to GLOBAL FRAME 
    // output array is ordered [J1, J2, J3, J4, J5, J6] where all joint angles J reference JOINT FRAME
    // the opposite is true for solve_FK
    
    /*
    float IK_in[6] = {150, 0, 550, 0, 90, 180};
    float IK_out[6] = {0, 0, 0, 0, 0, 0};
    solve_IK(IK_in, IK_out);

    float FK_out[6] = {0, 0, 0, 0, 0, 0};
    solve_FK(IK_out, FK_out);
    
    print_matrix(IK_out, 6, 1);
    print_matrix(FK_out, 6, 1);
    */

    
    point_3D home_point_xyz = {323.08, 0, 474.77};
    const float effector_fwd[3] = {0, 90, 180}; 
    point_3D desired_point = {323.08, 0, 200};  
    std::vector<point_3D> point_list;

    generate_line_path(point_list, home_point_xyz, desired_point, 0.1);
    execute_path(point_list, home_point_xyz, effector_fwd);
    
}

// SERIAL MESSAGE STRUCTURE:
// < {command id} : {command} >
// ":" characters tell parser when to expect next topic
// "< and >" chars tell parser when message begins / ends

// if command id = 0 -> angular velocity command
// {command} = {motor number} : {enable or disable} : {angular velocity}

// if command id = 1 -> IK setpoint command
// {command} = {ena straight line path} : {path speed} : {X0} : {Y0} : {Z0} : {Z_rot} : {Y_rot} : {Z_rot}
// {ena straight line path} 1 = enable, 0 = disable | if disabled, path speed (mm/s) can be set to 0 

// if command id = 2 -> direct joint angle command
// {command} = {J1} : {J2} : {J3} : {J4} : {J5} : {J6}
