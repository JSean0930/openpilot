#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3714579942524060962);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6333227287057358051);
void car_H_mod_fun(double *state, double *out_938330572553981807);
void car_f_fun(double *state, double dt, double *out_1017829496363843236);
void car_F_fun(double *state, double dt, double *out_7405850719611581950);
void car_h_25(double *state, double *unused, double *out_2887710260934258609);
void car_H_25(double *state, double *unused, double *out_3856500270373887302);
void car_h_24(double *state, double *unused, double *out_1684875075448049734);
void car_H_24(double *state, double *unused, double *out_6029149869379386868);
void car_h_30(double *state, double *unused, double *out_2149410545838620321);
void car_H_30(double *state, double *unused, double *out_8384196600501495500);
void car_h_26(double *state, double *unused, double *out_4853352157954445458);
void car_H_26(double *state, double *unused, double *out_7598003589247943526);
void car_h_27(double *state, double *unused, double *out_3511331256479590537);
void car_H_27(double *state, double *unused, double *out_7887784161407631205);
void car_h_29(double *state, double *unused, double *out_6438752135966221087);
void car_H_29(double *state, double *unused, double *out_7873965256187103316);
void car_h_28(double *state, double *unused, double *out_1633558806636057000);
void car_H_28(double *state, double *unused, double *out_5910334984621777065);
void car_h_31(double *state, double *unused, double *out_973973215494243043);
void car_H_31(double *state, double *unused, double *out_8224211691481295002);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}