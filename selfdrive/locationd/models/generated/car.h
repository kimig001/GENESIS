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
void car_err_fun(double *nom_x, double *delta_x, double *out_8823241645910388853);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_754757898947881340);
void car_H_mod_fun(double *state, double *out_4866200828999768844);
void car_f_fun(double *state, double dt, double *out_4892055470167629225);
void car_F_fun(double *state, double dt, double *out_5172459942356721641);
void car_h_25(double *state, double *unused, double *out_1469626072565606783);
void car_H_25(double *state, double *unused, double *out_4221956491092774322);
void car_h_24(double *state, double *unused, double *out_7723156681878050412);
void car_H_24(double *state, double *unused, double *out_8413401951594000329);
void car_h_30(double *state, double *unused, double *out_1626812740916735928);
void car_H_30(double *state, double *unused, double *out_4092617543949534252);
void car_h_26(double *state, double *unused, double *out_5808330693511964885);
void car_H_26(double *state, double *unused, double *out_480453172218718098);
void car_h_27(double *state, double *unused, double *out_4776824984894566685);
void car_H_27(double *state, double *unused, double *out_1917854232149109341);
void car_h_29(double *state, double *unused, double *out_4231218061725433579);
void car_H_29(double *state, double *unused, double *out_204491505279558308);
void car_h_28(double *state, double *unused, double *out_6876572308629669246);
void car_H_28(double *state, double *unused, double *out_6943401519823942046);
void car_h_31(double *state, double *unused, double *out_8057453999041901946);
void car_H_31(double *state, double *unused, double *out_4252602452969734750);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}