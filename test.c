#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "myrio/c/src/kalman_filter.c"

int main() {
    double X = 0;
    double Y = 0;
    double Psi = 0;
    double roll = 0;
    double rollRate = 0;
    double delta = 0;
    double v = 0;

    double dot_delta = 1;
    double latitude = 0;
    double longitude = 0;
    double a_y= 0;
    double w_x= 0;
    double w_z= 0;
    double delta_enc= 0;
    double speed= 0;
    // double *Kalman_Gain = malloc(7*7 * sizeof(*Kalman_Gain));
    // double *A_d = malloc(7* 7 * sizeof(*A_d));
    // double *B_d = malloc(7 * sizeof(*B_d));
    // double *C = malloc(7* 7 * sizeof(*C));
    // double *D = malloc(7 * sizeof(*D));
    double reset = 0;

    // for (int i = 0; i < 7; i++){

    //     B_d[i] = 0;
    //     D[i] = 0;
    //     for (int h = 0; h < 7; h++){
    //         A_d[7*i+h] = 0;
    //         C[7*i+h] = 0; 
    //         Kalman_Gain[7*i+h] = 0;
    //     }

    // }

    double Est_States[7] = {1,1,1,1,1,1,1};
    double Est_states_l[7] = {1.381773,-0.301169,0,1,1,1,1};
    double Est_states[7];


    Est_states[0] = Est_states_l[0] * cos(Est_States[2]) - Est_states_l[1] * sin(Est_States[2]);
    Est_states[1] = Est_states_l[0] * sin(Est_States[2]) + Est_states_l[1] * cos(Est_States[2]);
    Est_states[2] = Est_states_l[2] + Est_States[2];
    Est_states[3] = Est_states_l[3];
    Est_states[4] = Est_states_l[4];
    Est_states[5] = Est_states_l[5];
    Est_states[6] = Est_states_l[6];     
    
    printf("X: %f",Est_states[0]);
    printf("Y: %f",Est_states[1]);
    printf("Psi: %f",Est_states[2]);
    printf("Roll: %f",Est_states[3]);
    printf("RollRate: %f",Est_states[4]);
    printf("Delta: %f",Est_states[5]);
    printf("vel: %f\n",Est_states[6]);
}
