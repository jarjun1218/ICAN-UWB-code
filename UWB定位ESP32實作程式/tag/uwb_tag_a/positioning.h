#ifndef _POSITIONING_h
#define _POSITIONING_h

#include <stdint.h>

#define MAX_NUM_ANCHOR 8

extern float anchor[MAX_NUM_ANCHOR][3];

float my_sqrt(float);
float my_abs(float);
float determinant_2(float mat[2][2]);
float determinant_3(float mat[3][3]);
float determinant_4(float mat[4][4]);
void inv_3_matrix(float mat[3][3], float inv_mat[3][3]);
void inv_4_matrix(float mat[4][4], float inv_mat[4][4]);
void get_position(float *distance_list, float *position_reslult);

#endif
