#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_1455056450359289667);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8825321344757757707);
void pose_H_mod_fun(double *state, double *out_1371500253593254558);
void pose_f_fun(double *state, double dt, double *out_6239891866939865755);
void pose_F_fun(double *state, double dt, double *out_4017855667471812861);
void pose_h_4(double *state, double *unused, double *out_4813675346921411758);
void pose_H_4(double *state, double *unused, double *out_5986127290522570922);
void pose_h_10(double *state, double *unused, double *out_8589871310771143927);
void pose_H_10(double *state, double *unused, double *out_1877719131843322335);
void pose_h_13(double *state, double *unused, double *out_3162448842721061253);
void pose_H_13(double *state, double *unused, double *out_4849985574870279765);
void pose_h_14(double *state, double *unused, double *out_3168593434760140681);
void pose_H_14(double *state, double *unused, double *out_8497375926847496165);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}