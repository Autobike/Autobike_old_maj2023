#include <stdio.h>
#include <stdlib.h>
#include "myrio/c/src/kalman_filter.c"

int main() {
    double X = 0;
    double Y = 0;
    double Psi = 0;
    double roll = 0;
    double rollRate = 0;
    double delta = 0;
    double v = 0;

    double dot_delta = 0;
    double latitude = 0;
    double longitude = 0;
    double a_y= 0;
    double w_x= 0;
    double w_z= 0;
    double delta_enc= 0;
    double speed= 0;
    double *Kalman_Gain = malloc(7*7 * sizeof(*Kalman_Gain));
    double *A_d = malloc(7* 7 * sizeof(*A_d));
    double *B_d = malloc(7 * sizeof(*B_d));
    double *C = malloc(7* 7 * sizeof(*C));
    double *D = malloc(7 * sizeof(*D));
    double Ts = 0.01;

    for (int i = 0; i < 7; i++){

        B_d[i] = 0;
        D[i] = 0;
        for (int h = 0; h < 7; h++){
            A_d[7*i+h] = 0;
            C[7*i+h] = 0; 
            Kalman_Gain[7*i+h] = 0;
        }

    }

    Kalman_filter(&X, &Y, &Psi, &roll, &rollRate, &delta, &v, dot_delta, latitude, longitude, a_y, w_x,
                          w_z, delta_enc, speed, Kalman_Gain, A_d,
                          B_d, C, D, Ts);

    printf("X: %f",X);
}