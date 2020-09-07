

/* Include Files */
#include "main.h"
#include "InformationFilterUpdate.h"
#include "stdio.h"
#include <time.h>

static void main_InformationFilterUpdate(void);



static void main_InformationFilterUpdate(void)
{
    double y_meas[13] = { 0.2018,-0.3185,-0.02881,7.27756,7.6464,7.4544,7.2981,7.214626736,0,0,7.550,7.2564,0 };
    double B_usedMeas_vec[13] = { 1,1,1,1,0,0,0,0,0,0,1,1,0 };
    double initialization_vec[4] = { 75.1,83.4,7.25,0.413 };
    // double Pk_init[64] = { 0.0100,0, 0, 0, 0, 0, 0, 0, 0, 0.0300, 0, 0,  0, 0, 0, 0, 0, 0, 0.0001, 0, 0, 0, 0, 0, 0, 0, 0, 0.0020, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0.0100 ,0 ,0 ,0 ,0, 0, 0, 0,  0 ,0.0300 ,0 ,0, 0 ,0 ,0 ,0 ,0 ,0 ,0.0001 ,0, 0 ,0, 0, 0, 0 ,0 ,0 ,0.0020 };
    double delta = 0.000734;
    double Rw[8] = { 10,1,10,1, 10,1,10,1 };
    double Re[13] = { 1,1,0.001,1,1,1,1,1,1,1,1,1,1 };
    double L_imuToRear = 0.1;
    double L_geometricWheelbase = 3.7;
    double L_trackWidth[5] = { 2.05,0,1.85,0,0 };
    double L_axlePos[5] = { 0,0,-3.7,0,0 };
    double T = 0.01;
    double xk_m_out[16];
    double op[16];


    clock_t start = clock();
    InformationFilterUpdate(y_meas, B_usedMeas_vec, initialization_vec, delta,
        Rw, Re, L_imuToRear, L_geometricWheelbase, L_trackWidth,
        L_axlePos, T, xk_m_out, op);
    clock_t end = clock();
    double cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

    printf("for loop took %f seconds to execute \n", cpu_time_used);
    for (int i = 0; i < 8; i++)
    {
        printf("%lf ", xk_m_out[i]);
    }
    printf("\n");
    for (int i = 0; i < 16; i++)
    {
        printf("%lf ", op[i]);
    }
    printf("\n");

}


int main(int argc, const char* const argv[])
{
    (void)argc;
    (void)argv;


    main_InformationFilterUpdate();


    InformationFilterUpdate_terminate();


}