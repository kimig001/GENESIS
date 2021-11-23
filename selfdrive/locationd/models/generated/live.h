#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_3(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_19(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3414345200409230008);
void live_err_fun(double *nom_x, double *delta_x, double *out_4421444962371201403);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8914642413216044904);
void live_H_mod_fun(double *state, double *out_7074835256895391509);
void live_f_fun(double *state, double dt, double *out_3723698663117322244);
void live_F_fun(double *state, double dt, double *out_6195031421806224046);
void live_h_3(double *state, double *unused, double *out_8865756621770041179);
void live_H_3(double *state, double *unused, double *out_6323515166632599648);
void live_h_4(double *state, double *unused, double *out_4481981868727003479);
void live_H_4(double *state, double *unused, double *out_1368464342862762171);
void live_h_9(double *state, double *unused, double *out_9093590119180626247);
void live_H_9(double *state, double *unused, double *out_8089327269717986269);
void live_h_10(double *state, double *unused, double *out_2892453840131806359);
void live_H_10(double *state, double *unused, double *out_6863797242035625708);
void live_h_12(double *state, double *unused, double *out_6639665976220077696);
void live_H_12(double *state, double *unused, double *out_7873377237586339868);
void live_h_31(double *state, double *unused, double *out_6891914647250270402);
void live_H_31(double *state, double *unused, double *out_5198287462442920047);
void live_h_32(double *state, double *unused, double *out_1184031023859318897);
void live_H_32(double *state, double *unused, double *out_1102078281817481139);
void live_h_13(double *state, double *unused, double *out_7872145067070214499);
void live_H_13(double *state, double *unused, double *out_4850654448270771071);
void live_h_14(double *state, double *unused, double *out_9093590119180626247);
void live_H_14(double *state, double *unused, double *out_8089327269717986269);
void live_h_19(double *state, double *unused, double *out_4545558987924438147);
void live_H_19(double *state, double *unused, double *out_5924736952889134166);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}