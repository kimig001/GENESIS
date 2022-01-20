#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 5.991464547107981;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.8                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7600641781900611908) {
   out_7600641781900611908[0] = delta_x[0] + nom_x[0];
   out_7600641781900611908[1] = delta_x[1] + nom_x[1];
   out_7600641781900611908[2] = delta_x[2] + nom_x[2];
   out_7600641781900611908[3] = delta_x[3] + nom_x[3];
   out_7600641781900611908[4] = delta_x[4] + nom_x[4];
   out_7600641781900611908[5] = delta_x[5] + nom_x[5];
   out_7600641781900611908[6] = delta_x[6] + nom_x[6];
   out_7600641781900611908[7] = delta_x[7] + nom_x[7];
   out_7600641781900611908[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6648479288120472990) {
   out_6648479288120472990[0] = -nom_x[0] + true_x[0];
   out_6648479288120472990[1] = -nom_x[1] + true_x[1];
   out_6648479288120472990[2] = -nom_x[2] + true_x[2];
   out_6648479288120472990[3] = -nom_x[3] + true_x[3];
   out_6648479288120472990[4] = -nom_x[4] + true_x[4];
   out_6648479288120472990[5] = -nom_x[5] + true_x[5];
   out_6648479288120472990[6] = -nom_x[6] + true_x[6];
   out_6648479288120472990[7] = -nom_x[7] + true_x[7];
   out_6648479288120472990[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5006018585713841786) {
   out_5006018585713841786[0] = 1.0;
   out_5006018585713841786[1] = 0.0;
   out_5006018585713841786[2] = 0.0;
   out_5006018585713841786[3] = 0.0;
   out_5006018585713841786[4] = 0.0;
   out_5006018585713841786[5] = 0.0;
   out_5006018585713841786[6] = 0.0;
   out_5006018585713841786[7] = 0.0;
   out_5006018585713841786[8] = 0.0;
   out_5006018585713841786[9] = 0.0;
   out_5006018585713841786[10] = 1.0;
   out_5006018585713841786[11] = 0.0;
   out_5006018585713841786[12] = 0.0;
   out_5006018585713841786[13] = 0.0;
   out_5006018585713841786[14] = 0.0;
   out_5006018585713841786[15] = 0.0;
   out_5006018585713841786[16] = 0.0;
   out_5006018585713841786[17] = 0.0;
   out_5006018585713841786[18] = 0.0;
   out_5006018585713841786[19] = 0.0;
   out_5006018585713841786[20] = 1.0;
   out_5006018585713841786[21] = 0.0;
   out_5006018585713841786[22] = 0.0;
   out_5006018585713841786[23] = 0.0;
   out_5006018585713841786[24] = 0.0;
   out_5006018585713841786[25] = 0.0;
   out_5006018585713841786[26] = 0.0;
   out_5006018585713841786[27] = 0.0;
   out_5006018585713841786[28] = 0.0;
   out_5006018585713841786[29] = 0.0;
   out_5006018585713841786[30] = 1.0;
   out_5006018585713841786[31] = 0.0;
   out_5006018585713841786[32] = 0.0;
   out_5006018585713841786[33] = 0.0;
   out_5006018585713841786[34] = 0.0;
   out_5006018585713841786[35] = 0.0;
   out_5006018585713841786[36] = 0.0;
   out_5006018585713841786[37] = 0.0;
   out_5006018585713841786[38] = 0.0;
   out_5006018585713841786[39] = 0.0;
   out_5006018585713841786[40] = 1.0;
   out_5006018585713841786[41] = 0.0;
   out_5006018585713841786[42] = 0.0;
   out_5006018585713841786[43] = 0.0;
   out_5006018585713841786[44] = 0.0;
   out_5006018585713841786[45] = 0.0;
   out_5006018585713841786[46] = 0.0;
   out_5006018585713841786[47] = 0.0;
   out_5006018585713841786[48] = 0.0;
   out_5006018585713841786[49] = 0.0;
   out_5006018585713841786[50] = 1.0;
   out_5006018585713841786[51] = 0.0;
   out_5006018585713841786[52] = 0.0;
   out_5006018585713841786[53] = 0.0;
   out_5006018585713841786[54] = 0.0;
   out_5006018585713841786[55] = 0.0;
   out_5006018585713841786[56] = 0.0;
   out_5006018585713841786[57] = 0.0;
   out_5006018585713841786[58] = 0.0;
   out_5006018585713841786[59] = 0.0;
   out_5006018585713841786[60] = 1.0;
   out_5006018585713841786[61] = 0.0;
   out_5006018585713841786[62] = 0.0;
   out_5006018585713841786[63] = 0.0;
   out_5006018585713841786[64] = 0.0;
   out_5006018585713841786[65] = 0.0;
   out_5006018585713841786[66] = 0.0;
   out_5006018585713841786[67] = 0.0;
   out_5006018585713841786[68] = 0.0;
   out_5006018585713841786[69] = 0.0;
   out_5006018585713841786[70] = 1.0;
   out_5006018585713841786[71] = 0.0;
   out_5006018585713841786[72] = 0.0;
   out_5006018585713841786[73] = 0.0;
   out_5006018585713841786[74] = 0.0;
   out_5006018585713841786[75] = 0.0;
   out_5006018585713841786[76] = 0.0;
   out_5006018585713841786[77] = 0.0;
   out_5006018585713841786[78] = 0.0;
   out_5006018585713841786[79] = 0.0;
   out_5006018585713841786[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6595561906820683850) {
   out_6595561906820683850[0] = state[0];
   out_6595561906820683850[1] = state[1];
   out_6595561906820683850[2] = state[2];
   out_6595561906820683850[3] = state[3];
   out_6595561906820683850[4] = state[4];
   out_6595561906820683850[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6595561906820683850[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6595561906820683850[7] = state[7];
   out_6595561906820683850[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6144493995809233927) {
   out_6144493995809233927[0] = 1;
   out_6144493995809233927[1] = 0;
   out_6144493995809233927[2] = 0;
   out_6144493995809233927[3] = 0;
   out_6144493995809233927[4] = 0;
   out_6144493995809233927[5] = 0;
   out_6144493995809233927[6] = 0;
   out_6144493995809233927[7] = 0;
   out_6144493995809233927[8] = 0;
   out_6144493995809233927[9] = 0;
   out_6144493995809233927[10] = 1;
   out_6144493995809233927[11] = 0;
   out_6144493995809233927[12] = 0;
   out_6144493995809233927[13] = 0;
   out_6144493995809233927[14] = 0;
   out_6144493995809233927[15] = 0;
   out_6144493995809233927[16] = 0;
   out_6144493995809233927[17] = 0;
   out_6144493995809233927[18] = 0;
   out_6144493995809233927[19] = 0;
   out_6144493995809233927[20] = 1;
   out_6144493995809233927[21] = 0;
   out_6144493995809233927[22] = 0;
   out_6144493995809233927[23] = 0;
   out_6144493995809233927[24] = 0;
   out_6144493995809233927[25] = 0;
   out_6144493995809233927[26] = 0;
   out_6144493995809233927[27] = 0;
   out_6144493995809233927[28] = 0;
   out_6144493995809233927[29] = 0;
   out_6144493995809233927[30] = 1;
   out_6144493995809233927[31] = 0;
   out_6144493995809233927[32] = 0;
   out_6144493995809233927[33] = 0;
   out_6144493995809233927[34] = 0;
   out_6144493995809233927[35] = 0;
   out_6144493995809233927[36] = 0;
   out_6144493995809233927[37] = 0;
   out_6144493995809233927[38] = 0;
   out_6144493995809233927[39] = 0;
   out_6144493995809233927[40] = 1;
   out_6144493995809233927[41] = 0;
   out_6144493995809233927[42] = 0;
   out_6144493995809233927[43] = 0;
   out_6144493995809233927[44] = 0;
   out_6144493995809233927[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6144493995809233927[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6144493995809233927[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6144493995809233927[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6144493995809233927[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6144493995809233927[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6144493995809233927[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6144493995809233927[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6144493995809233927[53] = -9.8000000000000007*dt;
   out_6144493995809233927[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6144493995809233927[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6144493995809233927[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6144493995809233927[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6144493995809233927[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6144493995809233927[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6144493995809233927[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6144493995809233927[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6144493995809233927[62] = 0;
   out_6144493995809233927[63] = 0;
   out_6144493995809233927[64] = 0;
   out_6144493995809233927[65] = 0;
   out_6144493995809233927[66] = 0;
   out_6144493995809233927[67] = 0;
   out_6144493995809233927[68] = 0;
   out_6144493995809233927[69] = 0;
   out_6144493995809233927[70] = 1;
   out_6144493995809233927[71] = 0;
   out_6144493995809233927[72] = 0;
   out_6144493995809233927[73] = 0;
   out_6144493995809233927[74] = 0;
   out_6144493995809233927[75] = 0;
   out_6144493995809233927[76] = 0;
   out_6144493995809233927[77] = 0;
   out_6144493995809233927[78] = 0;
   out_6144493995809233927[79] = 0;
   out_6144493995809233927[80] = 1;
}
void h_25(double *state, double *unused, double *out_878719363787415205) {
   out_878719363787415205[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1814126200744486266) {
   out_1814126200744486266[0] = 0;
   out_1814126200744486266[1] = 0;
   out_1814126200744486266[2] = 0;
   out_1814126200744486266[3] = 0;
   out_1814126200744486266[4] = 0;
   out_1814126200744486266[5] = 0;
   out_1814126200744486266[6] = 1;
   out_1814126200744486266[7] = 0;
   out_1814126200744486266[8] = 0;
}
void h_24(double *state, double *unused, double *out_1562404562889038322) {
   out_1562404562889038322[0] = state[4];
   out_1562404562889038322[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2168196950653281632) {
   out_2168196950653281632[0] = 0;
   out_2168196950653281632[1] = 0;
   out_2168196950653281632[2] = 0;
   out_2168196950653281632[3] = 0;
   out_2168196950653281632[4] = 1;
   out_2168196950653281632[5] = 0;
   out_2168196950653281632[6] = 0;
   out_2168196950653281632[7] = 0;
   out_2168196950653281632[8] = 0;
   out_2168196950653281632[9] = 0;
   out_2168196950653281632[10] = 0;
   out_2168196950653281632[11] = 0;
   out_2168196950653281632[12] = 0;
   out_2168196950653281632[13] = 0;
   out_2168196950653281632[14] = 1;
   out_2168196950653281632[15] = 0;
   out_2168196950653281632[16] = 0;
   out_2168196950653281632[17] = 0;
}
void h_30(double *state, double *unused, double *out_733199539566760234) {
   out_733199539566760234[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8730816542236103021) {
   out_8730816542236103021[0] = 0;
   out_8730816542236103021[1] = 0;
   out_8730816542236103021[2] = 0;
   out_8730816542236103021[3] = 0;
   out_8730816542236103021[4] = 1;
   out_8730816542236103021[5] = 0;
   out_8730816542236103021[6] = 0;
   out_8730816542236103021[7] = 0;
   out_8730816542236103021[8] = 0;
}
void h_26(double *state, double *unused, double *out_210310313334902201) {
   out_210310313334902201[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1927377118129569958) {
   out_1927377118129569958[0] = 0;
   out_1927377118129569958[1] = 0;
   out_1927377118129569958[2] = 0;
   out_1927377118129569958[3] = 0;
   out_1927377118129569958[4] = 0;
   out_1927377118129569958[5] = 0;
   out_1927377118129569958[6] = 0;
   out_1927377118129569958[7] = 1;
   out_1927377118129569958[8] = 0;
}
void h_27(double *state, double *unused, double *out_4122953903873430440) {
   out_4122953903873430440[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6556053230435678110) {
   out_6556053230435678110[0] = 0;
   out_6556053230435678110[1] = 0;
   out_6556053230435678110[2] = 0;
   out_6556053230435678110[3] = 1;
   out_6556053230435678110[4] = 0;
   out_6556053230435678110[5] = 0;
   out_6556053230435678110[6] = 0;
   out_6556053230435678110[7] = 0;
   out_6556053230435678110[8] = 0;
}
void h_29(double *state, double *unused, double *out_3769985591011092919) {
   out_3769985591011092919[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9205696187159056411) {
   out_9205696187159056411[0] = 0;
   out_9205696187159056411[1] = 1;
   out_9205696187159056411[2] = 0;
   out_9205696187159056411[3] = 0;
   out_9205696187159056411[4] = 0;
   out_9205696187159056411[5] = 0;
   out_9205696187159056411[6] = 0;
   out_9205696187159056411[7] = 0;
   out_9205696187159056411[8] = 0;
}
void h_28(double *state, double *unused, double *out_6214677187579591889) {
   out_6214677187579591889[0] = state[5];
   out_6214677187579591889[1] = state[6];
}
void H_28(double *state, double *unused, double *out_3700160864101144779) {
   out_3700160864101144779[0] = 0;
   out_3700160864101144779[1] = 0;
   out_3700160864101144779[2] = 0;
   out_3700160864101144779[3] = 0;
   out_3700160864101144779[4] = 0;
   out_3700160864101144779[5] = 1;
   out_3700160864101144779[6] = 0;
   out_3700160864101144779[7] = 0;
   out_3700160864101144779[8] = 0;
   out_3700160864101144779[9] = 0;
   out_3700160864101144779[10] = 0;
   out_3700160864101144779[11] = 0;
   out_3700160864101144779[12] = 0;
   out_3700160864101144779[13] = 0;
   out_3700160864101144779[14] = 0;
   out_3700160864101144779[15] = 1;
   out_3700160864101144779[16] = 0;
   out_3700160864101144779[17] = 0;
}
void h_31(double *state, double *unused, double *out_6884545893033257236) {
   out_6884545893033257236[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1844772162621446694) {
   out_1844772162621446694[0] = 0;
   out_1844772162621446694[1] = 0;
   out_1844772162621446694[2] = 0;
   out_1844772162621446694[3] = 0;
   out_1844772162621446694[4] = 0;
   out_1844772162621446694[5] = 0;
   out_1844772162621446694[6] = 0;
   out_1844772162621446694[7] = 0;
   out_1844772162621446694[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7600641781900611908) {
  err_fun(nom_x, delta_x, out_7600641781900611908);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6648479288120472990) {
  inv_err_fun(nom_x, true_x, out_6648479288120472990);
}
void car_H_mod_fun(double *state, double *out_5006018585713841786) {
  H_mod_fun(state, out_5006018585713841786);
}
void car_f_fun(double *state, double dt, double *out_6595561906820683850) {
  f_fun(state,  dt, out_6595561906820683850);
}
void car_F_fun(double *state, double dt, double *out_6144493995809233927) {
  F_fun(state,  dt, out_6144493995809233927);
}
void car_h_25(double *state, double *unused, double *out_878719363787415205) {
  h_25(state, unused, out_878719363787415205);
}
void car_H_25(double *state, double *unused, double *out_1814126200744486266) {
  H_25(state, unused, out_1814126200744486266);
}
void car_h_24(double *state, double *unused, double *out_1562404562889038322) {
  h_24(state, unused, out_1562404562889038322);
}
void car_H_24(double *state, double *unused, double *out_2168196950653281632) {
  H_24(state, unused, out_2168196950653281632);
}
void car_h_30(double *state, double *unused, double *out_733199539566760234) {
  h_30(state, unused, out_733199539566760234);
}
void car_H_30(double *state, double *unused, double *out_8730816542236103021) {
  H_30(state, unused, out_8730816542236103021);
}
void car_h_26(double *state, double *unused, double *out_210310313334902201) {
  h_26(state, unused, out_210310313334902201);
}
void car_H_26(double *state, double *unused, double *out_1927377118129569958) {
  H_26(state, unused, out_1927377118129569958);
}
void car_h_27(double *state, double *unused, double *out_4122953903873430440) {
  h_27(state, unused, out_4122953903873430440);
}
void car_H_27(double *state, double *unused, double *out_6556053230435678110) {
  H_27(state, unused, out_6556053230435678110);
}
void car_h_29(double *state, double *unused, double *out_3769985591011092919) {
  h_29(state, unused, out_3769985591011092919);
}
void car_H_29(double *state, double *unused, double *out_9205696187159056411) {
  H_29(state, unused, out_9205696187159056411);
}
void car_h_28(double *state, double *unused, double *out_6214677187579591889) {
  h_28(state, unused, out_6214677187579591889);
}
void car_H_28(double *state, double *unused, double *out_3700160864101144779) {
  H_28(state, unused, out_3700160864101144779);
}
void car_h_31(double *state, double *unused, double *out_6884545893033257236) {
  h_31(state, unused, out_6884545893033257236);
}
void car_H_31(double *state, double *unused, double *out_1844772162621446694) {
  H_31(state, unused, out_1844772162621446694);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
