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
void err_fun(double *nom_x, double *delta_x, double *out_8823241645910388853) {
   out_8823241645910388853[0] = delta_x[0] + nom_x[0];
   out_8823241645910388853[1] = delta_x[1] + nom_x[1];
   out_8823241645910388853[2] = delta_x[2] + nom_x[2];
   out_8823241645910388853[3] = delta_x[3] + nom_x[3];
   out_8823241645910388853[4] = delta_x[4] + nom_x[4];
   out_8823241645910388853[5] = delta_x[5] + nom_x[5];
   out_8823241645910388853[6] = delta_x[6] + nom_x[6];
   out_8823241645910388853[7] = delta_x[7] + nom_x[7];
   out_8823241645910388853[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_754757898947881340) {
   out_754757898947881340[0] = -nom_x[0] + true_x[0];
   out_754757898947881340[1] = -nom_x[1] + true_x[1];
   out_754757898947881340[2] = -nom_x[2] + true_x[2];
   out_754757898947881340[3] = -nom_x[3] + true_x[3];
   out_754757898947881340[4] = -nom_x[4] + true_x[4];
   out_754757898947881340[5] = -nom_x[5] + true_x[5];
   out_754757898947881340[6] = -nom_x[6] + true_x[6];
   out_754757898947881340[7] = -nom_x[7] + true_x[7];
   out_754757898947881340[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4866200828999768844) {
   out_4866200828999768844[0] = 1.0;
   out_4866200828999768844[1] = 0.0;
   out_4866200828999768844[2] = 0.0;
   out_4866200828999768844[3] = 0.0;
   out_4866200828999768844[4] = 0.0;
   out_4866200828999768844[5] = 0.0;
   out_4866200828999768844[6] = 0.0;
   out_4866200828999768844[7] = 0.0;
   out_4866200828999768844[8] = 0.0;
   out_4866200828999768844[9] = 0.0;
   out_4866200828999768844[10] = 1.0;
   out_4866200828999768844[11] = 0.0;
   out_4866200828999768844[12] = 0.0;
   out_4866200828999768844[13] = 0.0;
   out_4866200828999768844[14] = 0.0;
   out_4866200828999768844[15] = 0.0;
   out_4866200828999768844[16] = 0.0;
   out_4866200828999768844[17] = 0.0;
   out_4866200828999768844[18] = 0.0;
   out_4866200828999768844[19] = 0.0;
   out_4866200828999768844[20] = 1.0;
   out_4866200828999768844[21] = 0.0;
   out_4866200828999768844[22] = 0.0;
   out_4866200828999768844[23] = 0.0;
   out_4866200828999768844[24] = 0.0;
   out_4866200828999768844[25] = 0.0;
   out_4866200828999768844[26] = 0.0;
   out_4866200828999768844[27] = 0.0;
   out_4866200828999768844[28] = 0.0;
   out_4866200828999768844[29] = 0.0;
   out_4866200828999768844[30] = 1.0;
   out_4866200828999768844[31] = 0.0;
   out_4866200828999768844[32] = 0.0;
   out_4866200828999768844[33] = 0.0;
   out_4866200828999768844[34] = 0.0;
   out_4866200828999768844[35] = 0.0;
   out_4866200828999768844[36] = 0.0;
   out_4866200828999768844[37] = 0.0;
   out_4866200828999768844[38] = 0.0;
   out_4866200828999768844[39] = 0.0;
   out_4866200828999768844[40] = 1.0;
   out_4866200828999768844[41] = 0.0;
   out_4866200828999768844[42] = 0.0;
   out_4866200828999768844[43] = 0.0;
   out_4866200828999768844[44] = 0.0;
   out_4866200828999768844[45] = 0.0;
   out_4866200828999768844[46] = 0.0;
   out_4866200828999768844[47] = 0.0;
   out_4866200828999768844[48] = 0.0;
   out_4866200828999768844[49] = 0.0;
   out_4866200828999768844[50] = 1.0;
   out_4866200828999768844[51] = 0.0;
   out_4866200828999768844[52] = 0.0;
   out_4866200828999768844[53] = 0.0;
   out_4866200828999768844[54] = 0.0;
   out_4866200828999768844[55] = 0.0;
   out_4866200828999768844[56] = 0.0;
   out_4866200828999768844[57] = 0.0;
   out_4866200828999768844[58] = 0.0;
   out_4866200828999768844[59] = 0.0;
   out_4866200828999768844[60] = 1.0;
   out_4866200828999768844[61] = 0.0;
   out_4866200828999768844[62] = 0.0;
   out_4866200828999768844[63] = 0.0;
   out_4866200828999768844[64] = 0.0;
   out_4866200828999768844[65] = 0.0;
   out_4866200828999768844[66] = 0.0;
   out_4866200828999768844[67] = 0.0;
   out_4866200828999768844[68] = 0.0;
   out_4866200828999768844[69] = 0.0;
   out_4866200828999768844[70] = 1.0;
   out_4866200828999768844[71] = 0.0;
   out_4866200828999768844[72] = 0.0;
   out_4866200828999768844[73] = 0.0;
   out_4866200828999768844[74] = 0.0;
   out_4866200828999768844[75] = 0.0;
   out_4866200828999768844[76] = 0.0;
   out_4866200828999768844[77] = 0.0;
   out_4866200828999768844[78] = 0.0;
   out_4866200828999768844[79] = 0.0;
   out_4866200828999768844[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4892055470167629225) {
   out_4892055470167629225[0] = state[0];
   out_4892055470167629225[1] = state[1];
   out_4892055470167629225[2] = state[2];
   out_4892055470167629225[3] = state[3];
   out_4892055470167629225[4] = state[4];
   out_4892055470167629225[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4892055470167629225[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4892055470167629225[7] = state[7];
   out_4892055470167629225[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5172459942356721641) {
   out_5172459942356721641[0] = 1;
   out_5172459942356721641[1] = 0;
   out_5172459942356721641[2] = 0;
   out_5172459942356721641[3] = 0;
   out_5172459942356721641[4] = 0;
   out_5172459942356721641[5] = 0;
   out_5172459942356721641[6] = 0;
   out_5172459942356721641[7] = 0;
   out_5172459942356721641[8] = 0;
   out_5172459942356721641[9] = 0;
   out_5172459942356721641[10] = 1;
   out_5172459942356721641[11] = 0;
   out_5172459942356721641[12] = 0;
   out_5172459942356721641[13] = 0;
   out_5172459942356721641[14] = 0;
   out_5172459942356721641[15] = 0;
   out_5172459942356721641[16] = 0;
   out_5172459942356721641[17] = 0;
   out_5172459942356721641[18] = 0;
   out_5172459942356721641[19] = 0;
   out_5172459942356721641[20] = 1;
   out_5172459942356721641[21] = 0;
   out_5172459942356721641[22] = 0;
   out_5172459942356721641[23] = 0;
   out_5172459942356721641[24] = 0;
   out_5172459942356721641[25] = 0;
   out_5172459942356721641[26] = 0;
   out_5172459942356721641[27] = 0;
   out_5172459942356721641[28] = 0;
   out_5172459942356721641[29] = 0;
   out_5172459942356721641[30] = 1;
   out_5172459942356721641[31] = 0;
   out_5172459942356721641[32] = 0;
   out_5172459942356721641[33] = 0;
   out_5172459942356721641[34] = 0;
   out_5172459942356721641[35] = 0;
   out_5172459942356721641[36] = 0;
   out_5172459942356721641[37] = 0;
   out_5172459942356721641[38] = 0;
   out_5172459942356721641[39] = 0;
   out_5172459942356721641[40] = 1;
   out_5172459942356721641[41] = 0;
   out_5172459942356721641[42] = 0;
   out_5172459942356721641[43] = 0;
   out_5172459942356721641[44] = 0;
   out_5172459942356721641[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5172459942356721641[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5172459942356721641[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5172459942356721641[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5172459942356721641[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5172459942356721641[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5172459942356721641[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5172459942356721641[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5172459942356721641[53] = -9.8000000000000007*dt;
   out_5172459942356721641[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5172459942356721641[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5172459942356721641[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5172459942356721641[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5172459942356721641[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5172459942356721641[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5172459942356721641[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5172459942356721641[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5172459942356721641[62] = 0;
   out_5172459942356721641[63] = 0;
   out_5172459942356721641[64] = 0;
   out_5172459942356721641[65] = 0;
   out_5172459942356721641[66] = 0;
   out_5172459942356721641[67] = 0;
   out_5172459942356721641[68] = 0;
   out_5172459942356721641[69] = 0;
   out_5172459942356721641[70] = 1;
   out_5172459942356721641[71] = 0;
   out_5172459942356721641[72] = 0;
   out_5172459942356721641[73] = 0;
   out_5172459942356721641[74] = 0;
   out_5172459942356721641[75] = 0;
   out_5172459942356721641[76] = 0;
   out_5172459942356721641[77] = 0;
   out_5172459942356721641[78] = 0;
   out_5172459942356721641[79] = 0;
   out_5172459942356721641[80] = 1;
}
void h_25(double *state, double *unused, double *out_1469626072565606783) {
   out_1469626072565606783[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4221956491092774322) {
   out_4221956491092774322[0] = 0;
   out_4221956491092774322[1] = 0;
   out_4221956491092774322[2] = 0;
   out_4221956491092774322[3] = 0;
   out_4221956491092774322[4] = 0;
   out_4221956491092774322[5] = 0;
   out_4221956491092774322[6] = 1;
   out_4221956491092774322[7] = 0;
   out_4221956491092774322[8] = 0;
}
void h_24(double *state, double *unused, double *out_7723156681878050412) {
   out_7723156681878050412[0] = state[4];
   out_7723156681878050412[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8413401951594000329) {
   out_8413401951594000329[0] = 0;
   out_8413401951594000329[1] = 0;
   out_8413401951594000329[2] = 0;
   out_8413401951594000329[3] = 0;
   out_8413401951594000329[4] = 1;
   out_8413401951594000329[5] = 0;
   out_8413401951594000329[6] = 0;
   out_8413401951594000329[7] = 0;
   out_8413401951594000329[8] = 0;
   out_8413401951594000329[9] = 0;
   out_8413401951594000329[10] = 0;
   out_8413401951594000329[11] = 0;
   out_8413401951594000329[12] = 0;
   out_8413401951594000329[13] = 0;
   out_8413401951594000329[14] = 1;
   out_8413401951594000329[15] = 0;
   out_8413401951594000329[16] = 0;
   out_8413401951594000329[17] = 0;
}
void h_30(double *state, double *unused, double *out_1626812740916735928) {
   out_1626812740916735928[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4092617543949534252) {
   out_4092617543949534252[0] = 0;
   out_4092617543949534252[1] = 0;
   out_4092617543949534252[2] = 0;
   out_4092617543949534252[3] = 0;
   out_4092617543949534252[4] = 1;
   out_4092617543949534252[5] = 0;
   out_4092617543949534252[6] = 0;
   out_4092617543949534252[7] = 0;
   out_4092617543949534252[8] = 0;
}
void h_26(double *state, double *unused, double *out_5808330693511964885) {
   out_5808330693511964885[0] = state[7];
}
void H_26(double *state, double *unused, double *out_480453172218718098) {
   out_480453172218718098[0] = 0;
   out_480453172218718098[1] = 0;
   out_480453172218718098[2] = 0;
   out_480453172218718098[3] = 0;
   out_480453172218718098[4] = 0;
   out_480453172218718098[5] = 0;
   out_480453172218718098[6] = 0;
   out_480453172218718098[7] = 1;
   out_480453172218718098[8] = 0;
}
void h_27(double *state, double *unused, double *out_4776824984894566685) {
   out_4776824984894566685[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1917854232149109341) {
   out_1917854232149109341[0] = 0;
   out_1917854232149109341[1] = 0;
   out_1917854232149109341[2] = 0;
   out_1917854232149109341[3] = 1;
   out_1917854232149109341[4] = 0;
   out_1917854232149109341[5] = 0;
   out_1917854232149109341[6] = 0;
   out_1917854232149109341[7] = 0;
   out_1917854232149109341[8] = 0;
}
void h_29(double *state, double *unused, double *out_4231218061725433579) {
   out_4231218061725433579[0] = state[1];
}
void H_29(double *state, double *unused, double *out_204491505279558308) {
   out_204491505279558308[0] = 0;
   out_204491505279558308[1] = 1;
   out_204491505279558308[2] = 0;
   out_204491505279558308[3] = 0;
   out_204491505279558308[4] = 0;
   out_204491505279558308[5] = 0;
   out_204491505279558308[6] = 0;
   out_204491505279558308[7] = 0;
   out_204491505279558308[8] = 0;
}
void h_28(double *state, double *unused, double *out_6876572308629669246) {
   out_6876572308629669246[0] = state[5];
   out_6876572308629669246[1] = state[6];
}
void H_28(double *state, double *unused, double *out_6943401519823942046) {
   out_6943401519823942046[0] = 0;
   out_6943401519823942046[1] = 0;
   out_6943401519823942046[2] = 0;
   out_6943401519823942046[3] = 0;
   out_6943401519823942046[4] = 0;
   out_6943401519823942046[5] = 1;
   out_6943401519823942046[6] = 0;
   out_6943401519823942046[7] = 0;
   out_6943401519823942046[8] = 0;
   out_6943401519823942046[9] = 0;
   out_6943401519823942046[10] = 0;
   out_6943401519823942046[11] = 0;
   out_6943401519823942046[12] = 0;
   out_6943401519823942046[13] = 0;
   out_6943401519823942046[14] = 0;
   out_6943401519823942046[15] = 1;
   out_6943401519823942046[16] = 0;
   out_6943401519823942046[17] = 0;
}
void h_31(double *state, double *unused, double *out_8057453999041901946) {
   out_8057453999041901946[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4252602452969734750) {
   out_4252602452969734750[0] = 0;
   out_4252602452969734750[1] = 0;
   out_4252602452969734750[2] = 0;
   out_4252602452969734750[3] = 0;
   out_4252602452969734750[4] = 0;
   out_4252602452969734750[5] = 0;
   out_4252602452969734750[6] = 0;
   out_4252602452969734750[7] = 0;
   out_4252602452969734750[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_8823241645910388853) {
  err_fun(nom_x, delta_x, out_8823241645910388853);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_754757898947881340) {
  inv_err_fun(nom_x, true_x, out_754757898947881340);
}
void car_H_mod_fun(double *state, double *out_4866200828999768844) {
  H_mod_fun(state, out_4866200828999768844);
}
void car_f_fun(double *state, double dt, double *out_4892055470167629225) {
  f_fun(state,  dt, out_4892055470167629225);
}
void car_F_fun(double *state, double dt, double *out_5172459942356721641) {
  F_fun(state,  dt, out_5172459942356721641);
}
void car_h_25(double *state, double *unused, double *out_1469626072565606783) {
  h_25(state, unused, out_1469626072565606783);
}
void car_H_25(double *state, double *unused, double *out_4221956491092774322) {
  H_25(state, unused, out_4221956491092774322);
}
void car_h_24(double *state, double *unused, double *out_7723156681878050412) {
  h_24(state, unused, out_7723156681878050412);
}
void car_H_24(double *state, double *unused, double *out_8413401951594000329) {
  H_24(state, unused, out_8413401951594000329);
}
void car_h_30(double *state, double *unused, double *out_1626812740916735928) {
  h_30(state, unused, out_1626812740916735928);
}
void car_H_30(double *state, double *unused, double *out_4092617543949534252) {
  H_30(state, unused, out_4092617543949534252);
}
void car_h_26(double *state, double *unused, double *out_5808330693511964885) {
  h_26(state, unused, out_5808330693511964885);
}
void car_H_26(double *state, double *unused, double *out_480453172218718098) {
  H_26(state, unused, out_480453172218718098);
}
void car_h_27(double *state, double *unused, double *out_4776824984894566685) {
  h_27(state, unused, out_4776824984894566685);
}
void car_H_27(double *state, double *unused, double *out_1917854232149109341) {
  H_27(state, unused, out_1917854232149109341);
}
void car_h_29(double *state, double *unused, double *out_4231218061725433579) {
  h_29(state, unused, out_4231218061725433579);
}
void car_H_29(double *state, double *unused, double *out_204491505279558308) {
  H_29(state, unused, out_204491505279558308);
}
void car_h_28(double *state, double *unused, double *out_6876572308629669246) {
  h_28(state, unused, out_6876572308629669246);
}
void car_H_28(double *state, double *unused, double *out_6943401519823942046) {
  H_28(state, unused, out_6943401519823942046);
}
void car_h_31(double *state, double *unused, double *out_8057453999041901946) {
  h_31(state, unused, out_8057453999041901946);
}
void car_H_31(double *state, double *unused, double *out_4252602452969734750) {
  H_31(state, unused, out_4252602452969734750);
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
