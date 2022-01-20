#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4025156105647969873);
void live_err_fun(double *nom_x, double *delta_x, double *out_4971862151210719925);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_1175449850909647023);
void live_H_mod_fun(double *state, double *out_2742273176947733560);
void live_f_fun(double *state, double dt, double *out_334090933826292600);
void live_F_fun(double *state, double dt, double *out_3110784210182139823);
void live_h_4(double *state, double *unused, double *out_5455036878310112758);
void live_H_4(double *state, double *unused, double *out_6466449812818180840);
void live_h_9(double *state, double *unused, double *out_6269562564067507020);
void live_H_9(double *state, double *unused, double *out_6707639459447771485);
void live_h_10(double *state, double *unused, double *out_397430240853300482);
void live_H_10(double *state, double *unused, double *out_482438985351938964);
void live_h_12(double *state, double *unused, double *out_1899457965329291003);
void live_H_12(double *state, double *unused, double *out_6960837852859408981);
void live_h_31(double *state, double *unused, double *out_5156545870200553513);
void live_H_31(double *state, double *unused, double *out_8613632203518763400);
void live_h_32(double *state, double *unused, double *out_6608677393623002943);
void live_H_32(double *state, double *unused, double *out_1431616656115977679);
void live_h_13(double *state, double *unused, double *out_7424891219692984509);
void live_H_13(double *state, double *unused, double *out_5477107673330009735);
void live_h_14(double *state, double *unused, double *out_6269562564067507020);
void live_H_14(double *state, double *unused, double *out_6707639459447771485);
void live_h_33(double *state, double *unused, double *out_2587611344856094368);
void live_H_33(double *state, double *unused, double *out_5463075198879905796);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}