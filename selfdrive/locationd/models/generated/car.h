#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7600641781900611908);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6648479288120472990);
void car_H_mod_fun(double *state, double *out_5006018585713841786);
void car_f_fun(double *state, double dt, double *out_6595561906820683850);
void car_F_fun(double *state, double dt, double *out_6144493995809233927);
void car_h_25(double *state, double *unused, double *out_878719363787415205);
void car_H_25(double *state, double *unused, double *out_1814126200744486266);
void car_h_24(double *state, double *unused, double *out_1562404562889038322);
void car_H_24(double *state, double *unused, double *out_2168196950653281632);
void car_h_30(double *state, double *unused, double *out_733199539566760234);
void car_H_30(double *state, double *unused, double *out_8730816542236103021);
void car_h_26(double *state, double *unused, double *out_210310313334902201);
void car_H_26(double *state, double *unused, double *out_1927377118129569958);
void car_h_27(double *state, double *unused, double *out_4122953903873430440);
void car_H_27(double *state, double *unused, double *out_6556053230435678110);
void car_h_29(double *state, double *unused, double *out_3769985591011092919);
void car_H_29(double *state, double *unused, double *out_9205696187159056411);
void car_h_28(double *state, double *unused, double *out_6214677187579591889);
void car_H_28(double *state, double *unused, double *out_3700160864101144779);
void car_h_31(double *state, double *unused, double *out_6884545893033257236);
void car_H_31(double *state, double *unused, double *out_1844772162621446694);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}