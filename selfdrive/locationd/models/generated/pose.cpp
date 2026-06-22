#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1455056450359289667) {
   out_1455056450359289667[0] = delta_x[0] + nom_x[0];
   out_1455056450359289667[1] = delta_x[1] + nom_x[1];
   out_1455056450359289667[2] = delta_x[2] + nom_x[2];
   out_1455056450359289667[3] = delta_x[3] + nom_x[3];
   out_1455056450359289667[4] = delta_x[4] + nom_x[4];
   out_1455056450359289667[5] = delta_x[5] + nom_x[5];
   out_1455056450359289667[6] = delta_x[6] + nom_x[6];
   out_1455056450359289667[7] = delta_x[7] + nom_x[7];
   out_1455056450359289667[8] = delta_x[8] + nom_x[8];
   out_1455056450359289667[9] = delta_x[9] + nom_x[9];
   out_1455056450359289667[10] = delta_x[10] + nom_x[10];
   out_1455056450359289667[11] = delta_x[11] + nom_x[11];
   out_1455056450359289667[12] = delta_x[12] + nom_x[12];
   out_1455056450359289667[13] = delta_x[13] + nom_x[13];
   out_1455056450359289667[14] = delta_x[14] + nom_x[14];
   out_1455056450359289667[15] = delta_x[15] + nom_x[15];
   out_1455056450359289667[16] = delta_x[16] + nom_x[16];
   out_1455056450359289667[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8825321344757757707) {
   out_8825321344757757707[0] = -nom_x[0] + true_x[0];
   out_8825321344757757707[1] = -nom_x[1] + true_x[1];
   out_8825321344757757707[2] = -nom_x[2] + true_x[2];
   out_8825321344757757707[3] = -nom_x[3] + true_x[3];
   out_8825321344757757707[4] = -nom_x[4] + true_x[4];
   out_8825321344757757707[5] = -nom_x[5] + true_x[5];
   out_8825321344757757707[6] = -nom_x[6] + true_x[6];
   out_8825321344757757707[7] = -nom_x[7] + true_x[7];
   out_8825321344757757707[8] = -nom_x[8] + true_x[8];
   out_8825321344757757707[9] = -nom_x[9] + true_x[9];
   out_8825321344757757707[10] = -nom_x[10] + true_x[10];
   out_8825321344757757707[11] = -nom_x[11] + true_x[11];
   out_8825321344757757707[12] = -nom_x[12] + true_x[12];
   out_8825321344757757707[13] = -nom_x[13] + true_x[13];
   out_8825321344757757707[14] = -nom_x[14] + true_x[14];
   out_8825321344757757707[15] = -nom_x[15] + true_x[15];
   out_8825321344757757707[16] = -nom_x[16] + true_x[16];
   out_8825321344757757707[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1371500253593254558) {
   out_1371500253593254558[0] = 1.0;
   out_1371500253593254558[1] = 0.0;
   out_1371500253593254558[2] = 0.0;
   out_1371500253593254558[3] = 0.0;
   out_1371500253593254558[4] = 0.0;
   out_1371500253593254558[5] = 0.0;
   out_1371500253593254558[6] = 0.0;
   out_1371500253593254558[7] = 0.0;
   out_1371500253593254558[8] = 0.0;
   out_1371500253593254558[9] = 0.0;
   out_1371500253593254558[10] = 0.0;
   out_1371500253593254558[11] = 0.0;
   out_1371500253593254558[12] = 0.0;
   out_1371500253593254558[13] = 0.0;
   out_1371500253593254558[14] = 0.0;
   out_1371500253593254558[15] = 0.0;
   out_1371500253593254558[16] = 0.0;
   out_1371500253593254558[17] = 0.0;
   out_1371500253593254558[18] = 0.0;
   out_1371500253593254558[19] = 1.0;
   out_1371500253593254558[20] = 0.0;
   out_1371500253593254558[21] = 0.0;
   out_1371500253593254558[22] = 0.0;
   out_1371500253593254558[23] = 0.0;
   out_1371500253593254558[24] = 0.0;
   out_1371500253593254558[25] = 0.0;
   out_1371500253593254558[26] = 0.0;
   out_1371500253593254558[27] = 0.0;
   out_1371500253593254558[28] = 0.0;
   out_1371500253593254558[29] = 0.0;
   out_1371500253593254558[30] = 0.0;
   out_1371500253593254558[31] = 0.0;
   out_1371500253593254558[32] = 0.0;
   out_1371500253593254558[33] = 0.0;
   out_1371500253593254558[34] = 0.0;
   out_1371500253593254558[35] = 0.0;
   out_1371500253593254558[36] = 0.0;
   out_1371500253593254558[37] = 0.0;
   out_1371500253593254558[38] = 1.0;
   out_1371500253593254558[39] = 0.0;
   out_1371500253593254558[40] = 0.0;
   out_1371500253593254558[41] = 0.0;
   out_1371500253593254558[42] = 0.0;
   out_1371500253593254558[43] = 0.0;
   out_1371500253593254558[44] = 0.0;
   out_1371500253593254558[45] = 0.0;
   out_1371500253593254558[46] = 0.0;
   out_1371500253593254558[47] = 0.0;
   out_1371500253593254558[48] = 0.0;
   out_1371500253593254558[49] = 0.0;
   out_1371500253593254558[50] = 0.0;
   out_1371500253593254558[51] = 0.0;
   out_1371500253593254558[52] = 0.0;
   out_1371500253593254558[53] = 0.0;
   out_1371500253593254558[54] = 0.0;
   out_1371500253593254558[55] = 0.0;
   out_1371500253593254558[56] = 0.0;
   out_1371500253593254558[57] = 1.0;
   out_1371500253593254558[58] = 0.0;
   out_1371500253593254558[59] = 0.0;
   out_1371500253593254558[60] = 0.0;
   out_1371500253593254558[61] = 0.0;
   out_1371500253593254558[62] = 0.0;
   out_1371500253593254558[63] = 0.0;
   out_1371500253593254558[64] = 0.0;
   out_1371500253593254558[65] = 0.0;
   out_1371500253593254558[66] = 0.0;
   out_1371500253593254558[67] = 0.0;
   out_1371500253593254558[68] = 0.0;
   out_1371500253593254558[69] = 0.0;
   out_1371500253593254558[70] = 0.0;
   out_1371500253593254558[71] = 0.0;
   out_1371500253593254558[72] = 0.0;
   out_1371500253593254558[73] = 0.0;
   out_1371500253593254558[74] = 0.0;
   out_1371500253593254558[75] = 0.0;
   out_1371500253593254558[76] = 1.0;
   out_1371500253593254558[77] = 0.0;
   out_1371500253593254558[78] = 0.0;
   out_1371500253593254558[79] = 0.0;
   out_1371500253593254558[80] = 0.0;
   out_1371500253593254558[81] = 0.0;
   out_1371500253593254558[82] = 0.0;
   out_1371500253593254558[83] = 0.0;
   out_1371500253593254558[84] = 0.0;
   out_1371500253593254558[85] = 0.0;
   out_1371500253593254558[86] = 0.0;
   out_1371500253593254558[87] = 0.0;
   out_1371500253593254558[88] = 0.0;
   out_1371500253593254558[89] = 0.0;
   out_1371500253593254558[90] = 0.0;
   out_1371500253593254558[91] = 0.0;
   out_1371500253593254558[92] = 0.0;
   out_1371500253593254558[93] = 0.0;
   out_1371500253593254558[94] = 0.0;
   out_1371500253593254558[95] = 1.0;
   out_1371500253593254558[96] = 0.0;
   out_1371500253593254558[97] = 0.0;
   out_1371500253593254558[98] = 0.0;
   out_1371500253593254558[99] = 0.0;
   out_1371500253593254558[100] = 0.0;
   out_1371500253593254558[101] = 0.0;
   out_1371500253593254558[102] = 0.0;
   out_1371500253593254558[103] = 0.0;
   out_1371500253593254558[104] = 0.0;
   out_1371500253593254558[105] = 0.0;
   out_1371500253593254558[106] = 0.0;
   out_1371500253593254558[107] = 0.0;
   out_1371500253593254558[108] = 0.0;
   out_1371500253593254558[109] = 0.0;
   out_1371500253593254558[110] = 0.0;
   out_1371500253593254558[111] = 0.0;
   out_1371500253593254558[112] = 0.0;
   out_1371500253593254558[113] = 0.0;
   out_1371500253593254558[114] = 1.0;
   out_1371500253593254558[115] = 0.0;
   out_1371500253593254558[116] = 0.0;
   out_1371500253593254558[117] = 0.0;
   out_1371500253593254558[118] = 0.0;
   out_1371500253593254558[119] = 0.0;
   out_1371500253593254558[120] = 0.0;
   out_1371500253593254558[121] = 0.0;
   out_1371500253593254558[122] = 0.0;
   out_1371500253593254558[123] = 0.0;
   out_1371500253593254558[124] = 0.0;
   out_1371500253593254558[125] = 0.0;
   out_1371500253593254558[126] = 0.0;
   out_1371500253593254558[127] = 0.0;
   out_1371500253593254558[128] = 0.0;
   out_1371500253593254558[129] = 0.0;
   out_1371500253593254558[130] = 0.0;
   out_1371500253593254558[131] = 0.0;
   out_1371500253593254558[132] = 0.0;
   out_1371500253593254558[133] = 1.0;
   out_1371500253593254558[134] = 0.0;
   out_1371500253593254558[135] = 0.0;
   out_1371500253593254558[136] = 0.0;
   out_1371500253593254558[137] = 0.0;
   out_1371500253593254558[138] = 0.0;
   out_1371500253593254558[139] = 0.0;
   out_1371500253593254558[140] = 0.0;
   out_1371500253593254558[141] = 0.0;
   out_1371500253593254558[142] = 0.0;
   out_1371500253593254558[143] = 0.0;
   out_1371500253593254558[144] = 0.0;
   out_1371500253593254558[145] = 0.0;
   out_1371500253593254558[146] = 0.0;
   out_1371500253593254558[147] = 0.0;
   out_1371500253593254558[148] = 0.0;
   out_1371500253593254558[149] = 0.0;
   out_1371500253593254558[150] = 0.0;
   out_1371500253593254558[151] = 0.0;
   out_1371500253593254558[152] = 1.0;
   out_1371500253593254558[153] = 0.0;
   out_1371500253593254558[154] = 0.0;
   out_1371500253593254558[155] = 0.0;
   out_1371500253593254558[156] = 0.0;
   out_1371500253593254558[157] = 0.0;
   out_1371500253593254558[158] = 0.0;
   out_1371500253593254558[159] = 0.0;
   out_1371500253593254558[160] = 0.0;
   out_1371500253593254558[161] = 0.0;
   out_1371500253593254558[162] = 0.0;
   out_1371500253593254558[163] = 0.0;
   out_1371500253593254558[164] = 0.0;
   out_1371500253593254558[165] = 0.0;
   out_1371500253593254558[166] = 0.0;
   out_1371500253593254558[167] = 0.0;
   out_1371500253593254558[168] = 0.0;
   out_1371500253593254558[169] = 0.0;
   out_1371500253593254558[170] = 0.0;
   out_1371500253593254558[171] = 1.0;
   out_1371500253593254558[172] = 0.0;
   out_1371500253593254558[173] = 0.0;
   out_1371500253593254558[174] = 0.0;
   out_1371500253593254558[175] = 0.0;
   out_1371500253593254558[176] = 0.0;
   out_1371500253593254558[177] = 0.0;
   out_1371500253593254558[178] = 0.0;
   out_1371500253593254558[179] = 0.0;
   out_1371500253593254558[180] = 0.0;
   out_1371500253593254558[181] = 0.0;
   out_1371500253593254558[182] = 0.0;
   out_1371500253593254558[183] = 0.0;
   out_1371500253593254558[184] = 0.0;
   out_1371500253593254558[185] = 0.0;
   out_1371500253593254558[186] = 0.0;
   out_1371500253593254558[187] = 0.0;
   out_1371500253593254558[188] = 0.0;
   out_1371500253593254558[189] = 0.0;
   out_1371500253593254558[190] = 1.0;
   out_1371500253593254558[191] = 0.0;
   out_1371500253593254558[192] = 0.0;
   out_1371500253593254558[193] = 0.0;
   out_1371500253593254558[194] = 0.0;
   out_1371500253593254558[195] = 0.0;
   out_1371500253593254558[196] = 0.0;
   out_1371500253593254558[197] = 0.0;
   out_1371500253593254558[198] = 0.0;
   out_1371500253593254558[199] = 0.0;
   out_1371500253593254558[200] = 0.0;
   out_1371500253593254558[201] = 0.0;
   out_1371500253593254558[202] = 0.0;
   out_1371500253593254558[203] = 0.0;
   out_1371500253593254558[204] = 0.0;
   out_1371500253593254558[205] = 0.0;
   out_1371500253593254558[206] = 0.0;
   out_1371500253593254558[207] = 0.0;
   out_1371500253593254558[208] = 0.0;
   out_1371500253593254558[209] = 1.0;
   out_1371500253593254558[210] = 0.0;
   out_1371500253593254558[211] = 0.0;
   out_1371500253593254558[212] = 0.0;
   out_1371500253593254558[213] = 0.0;
   out_1371500253593254558[214] = 0.0;
   out_1371500253593254558[215] = 0.0;
   out_1371500253593254558[216] = 0.0;
   out_1371500253593254558[217] = 0.0;
   out_1371500253593254558[218] = 0.0;
   out_1371500253593254558[219] = 0.0;
   out_1371500253593254558[220] = 0.0;
   out_1371500253593254558[221] = 0.0;
   out_1371500253593254558[222] = 0.0;
   out_1371500253593254558[223] = 0.0;
   out_1371500253593254558[224] = 0.0;
   out_1371500253593254558[225] = 0.0;
   out_1371500253593254558[226] = 0.0;
   out_1371500253593254558[227] = 0.0;
   out_1371500253593254558[228] = 1.0;
   out_1371500253593254558[229] = 0.0;
   out_1371500253593254558[230] = 0.0;
   out_1371500253593254558[231] = 0.0;
   out_1371500253593254558[232] = 0.0;
   out_1371500253593254558[233] = 0.0;
   out_1371500253593254558[234] = 0.0;
   out_1371500253593254558[235] = 0.0;
   out_1371500253593254558[236] = 0.0;
   out_1371500253593254558[237] = 0.0;
   out_1371500253593254558[238] = 0.0;
   out_1371500253593254558[239] = 0.0;
   out_1371500253593254558[240] = 0.0;
   out_1371500253593254558[241] = 0.0;
   out_1371500253593254558[242] = 0.0;
   out_1371500253593254558[243] = 0.0;
   out_1371500253593254558[244] = 0.0;
   out_1371500253593254558[245] = 0.0;
   out_1371500253593254558[246] = 0.0;
   out_1371500253593254558[247] = 1.0;
   out_1371500253593254558[248] = 0.0;
   out_1371500253593254558[249] = 0.0;
   out_1371500253593254558[250] = 0.0;
   out_1371500253593254558[251] = 0.0;
   out_1371500253593254558[252] = 0.0;
   out_1371500253593254558[253] = 0.0;
   out_1371500253593254558[254] = 0.0;
   out_1371500253593254558[255] = 0.0;
   out_1371500253593254558[256] = 0.0;
   out_1371500253593254558[257] = 0.0;
   out_1371500253593254558[258] = 0.0;
   out_1371500253593254558[259] = 0.0;
   out_1371500253593254558[260] = 0.0;
   out_1371500253593254558[261] = 0.0;
   out_1371500253593254558[262] = 0.0;
   out_1371500253593254558[263] = 0.0;
   out_1371500253593254558[264] = 0.0;
   out_1371500253593254558[265] = 0.0;
   out_1371500253593254558[266] = 1.0;
   out_1371500253593254558[267] = 0.0;
   out_1371500253593254558[268] = 0.0;
   out_1371500253593254558[269] = 0.0;
   out_1371500253593254558[270] = 0.0;
   out_1371500253593254558[271] = 0.0;
   out_1371500253593254558[272] = 0.0;
   out_1371500253593254558[273] = 0.0;
   out_1371500253593254558[274] = 0.0;
   out_1371500253593254558[275] = 0.0;
   out_1371500253593254558[276] = 0.0;
   out_1371500253593254558[277] = 0.0;
   out_1371500253593254558[278] = 0.0;
   out_1371500253593254558[279] = 0.0;
   out_1371500253593254558[280] = 0.0;
   out_1371500253593254558[281] = 0.0;
   out_1371500253593254558[282] = 0.0;
   out_1371500253593254558[283] = 0.0;
   out_1371500253593254558[284] = 0.0;
   out_1371500253593254558[285] = 1.0;
   out_1371500253593254558[286] = 0.0;
   out_1371500253593254558[287] = 0.0;
   out_1371500253593254558[288] = 0.0;
   out_1371500253593254558[289] = 0.0;
   out_1371500253593254558[290] = 0.0;
   out_1371500253593254558[291] = 0.0;
   out_1371500253593254558[292] = 0.0;
   out_1371500253593254558[293] = 0.0;
   out_1371500253593254558[294] = 0.0;
   out_1371500253593254558[295] = 0.0;
   out_1371500253593254558[296] = 0.0;
   out_1371500253593254558[297] = 0.0;
   out_1371500253593254558[298] = 0.0;
   out_1371500253593254558[299] = 0.0;
   out_1371500253593254558[300] = 0.0;
   out_1371500253593254558[301] = 0.0;
   out_1371500253593254558[302] = 0.0;
   out_1371500253593254558[303] = 0.0;
   out_1371500253593254558[304] = 1.0;
   out_1371500253593254558[305] = 0.0;
   out_1371500253593254558[306] = 0.0;
   out_1371500253593254558[307] = 0.0;
   out_1371500253593254558[308] = 0.0;
   out_1371500253593254558[309] = 0.0;
   out_1371500253593254558[310] = 0.0;
   out_1371500253593254558[311] = 0.0;
   out_1371500253593254558[312] = 0.0;
   out_1371500253593254558[313] = 0.0;
   out_1371500253593254558[314] = 0.0;
   out_1371500253593254558[315] = 0.0;
   out_1371500253593254558[316] = 0.0;
   out_1371500253593254558[317] = 0.0;
   out_1371500253593254558[318] = 0.0;
   out_1371500253593254558[319] = 0.0;
   out_1371500253593254558[320] = 0.0;
   out_1371500253593254558[321] = 0.0;
   out_1371500253593254558[322] = 0.0;
   out_1371500253593254558[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6239891866939865755) {
   out_6239891866939865755[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6239891866939865755[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6239891866939865755[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6239891866939865755[3] = dt*state[12] + state[3];
   out_6239891866939865755[4] = dt*state[13] + state[4];
   out_6239891866939865755[5] = dt*state[14] + state[5];
   out_6239891866939865755[6] = state[6];
   out_6239891866939865755[7] = state[7];
   out_6239891866939865755[8] = state[8];
   out_6239891866939865755[9] = state[9];
   out_6239891866939865755[10] = state[10];
   out_6239891866939865755[11] = state[11];
   out_6239891866939865755[12] = state[12];
   out_6239891866939865755[13] = state[13];
   out_6239891866939865755[14] = state[14];
   out_6239891866939865755[15] = state[15];
   out_6239891866939865755[16] = state[16];
   out_6239891866939865755[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4017855667471812861) {
   out_4017855667471812861[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4017855667471812861[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4017855667471812861[2] = 0;
   out_4017855667471812861[3] = 0;
   out_4017855667471812861[4] = 0;
   out_4017855667471812861[5] = 0;
   out_4017855667471812861[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4017855667471812861[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4017855667471812861[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4017855667471812861[9] = 0;
   out_4017855667471812861[10] = 0;
   out_4017855667471812861[11] = 0;
   out_4017855667471812861[12] = 0;
   out_4017855667471812861[13] = 0;
   out_4017855667471812861[14] = 0;
   out_4017855667471812861[15] = 0;
   out_4017855667471812861[16] = 0;
   out_4017855667471812861[17] = 0;
   out_4017855667471812861[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4017855667471812861[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4017855667471812861[20] = 0;
   out_4017855667471812861[21] = 0;
   out_4017855667471812861[22] = 0;
   out_4017855667471812861[23] = 0;
   out_4017855667471812861[24] = 0;
   out_4017855667471812861[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4017855667471812861[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4017855667471812861[27] = 0;
   out_4017855667471812861[28] = 0;
   out_4017855667471812861[29] = 0;
   out_4017855667471812861[30] = 0;
   out_4017855667471812861[31] = 0;
   out_4017855667471812861[32] = 0;
   out_4017855667471812861[33] = 0;
   out_4017855667471812861[34] = 0;
   out_4017855667471812861[35] = 0;
   out_4017855667471812861[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4017855667471812861[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4017855667471812861[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4017855667471812861[39] = 0;
   out_4017855667471812861[40] = 0;
   out_4017855667471812861[41] = 0;
   out_4017855667471812861[42] = 0;
   out_4017855667471812861[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4017855667471812861[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4017855667471812861[45] = 0;
   out_4017855667471812861[46] = 0;
   out_4017855667471812861[47] = 0;
   out_4017855667471812861[48] = 0;
   out_4017855667471812861[49] = 0;
   out_4017855667471812861[50] = 0;
   out_4017855667471812861[51] = 0;
   out_4017855667471812861[52] = 0;
   out_4017855667471812861[53] = 0;
   out_4017855667471812861[54] = 0;
   out_4017855667471812861[55] = 0;
   out_4017855667471812861[56] = 0;
   out_4017855667471812861[57] = 1;
   out_4017855667471812861[58] = 0;
   out_4017855667471812861[59] = 0;
   out_4017855667471812861[60] = 0;
   out_4017855667471812861[61] = 0;
   out_4017855667471812861[62] = 0;
   out_4017855667471812861[63] = 0;
   out_4017855667471812861[64] = 0;
   out_4017855667471812861[65] = 0;
   out_4017855667471812861[66] = dt;
   out_4017855667471812861[67] = 0;
   out_4017855667471812861[68] = 0;
   out_4017855667471812861[69] = 0;
   out_4017855667471812861[70] = 0;
   out_4017855667471812861[71] = 0;
   out_4017855667471812861[72] = 0;
   out_4017855667471812861[73] = 0;
   out_4017855667471812861[74] = 0;
   out_4017855667471812861[75] = 0;
   out_4017855667471812861[76] = 1;
   out_4017855667471812861[77] = 0;
   out_4017855667471812861[78] = 0;
   out_4017855667471812861[79] = 0;
   out_4017855667471812861[80] = 0;
   out_4017855667471812861[81] = 0;
   out_4017855667471812861[82] = 0;
   out_4017855667471812861[83] = 0;
   out_4017855667471812861[84] = 0;
   out_4017855667471812861[85] = dt;
   out_4017855667471812861[86] = 0;
   out_4017855667471812861[87] = 0;
   out_4017855667471812861[88] = 0;
   out_4017855667471812861[89] = 0;
   out_4017855667471812861[90] = 0;
   out_4017855667471812861[91] = 0;
   out_4017855667471812861[92] = 0;
   out_4017855667471812861[93] = 0;
   out_4017855667471812861[94] = 0;
   out_4017855667471812861[95] = 1;
   out_4017855667471812861[96] = 0;
   out_4017855667471812861[97] = 0;
   out_4017855667471812861[98] = 0;
   out_4017855667471812861[99] = 0;
   out_4017855667471812861[100] = 0;
   out_4017855667471812861[101] = 0;
   out_4017855667471812861[102] = 0;
   out_4017855667471812861[103] = 0;
   out_4017855667471812861[104] = dt;
   out_4017855667471812861[105] = 0;
   out_4017855667471812861[106] = 0;
   out_4017855667471812861[107] = 0;
   out_4017855667471812861[108] = 0;
   out_4017855667471812861[109] = 0;
   out_4017855667471812861[110] = 0;
   out_4017855667471812861[111] = 0;
   out_4017855667471812861[112] = 0;
   out_4017855667471812861[113] = 0;
   out_4017855667471812861[114] = 1;
   out_4017855667471812861[115] = 0;
   out_4017855667471812861[116] = 0;
   out_4017855667471812861[117] = 0;
   out_4017855667471812861[118] = 0;
   out_4017855667471812861[119] = 0;
   out_4017855667471812861[120] = 0;
   out_4017855667471812861[121] = 0;
   out_4017855667471812861[122] = 0;
   out_4017855667471812861[123] = 0;
   out_4017855667471812861[124] = 0;
   out_4017855667471812861[125] = 0;
   out_4017855667471812861[126] = 0;
   out_4017855667471812861[127] = 0;
   out_4017855667471812861[128] = 0;
   out_4017855667471812861[129] = 0;
   out_4017855667471812861[130] = 0;
   out_4017855667471812861[131] = 0;
   out_4017855667471812861[132] = 0;
   out_4017855667471812861[133] = 1;
   out_4017855667471812861[134] = 0;
   out_4017855667471812861[135] = 0;
   out_4017855667471812861[136] = 0;
   out_4017855667471812861[137] = 0;
   out_4017855667471812861[138] = 0;
   out_4017855667471812861[139] = 0;
   out_4017855667471812861[140] = 0;
   out_4017855667471812861[141] = 0;
   out_4017855667471812861[142] = 0;
   out_4017855667471812861[143] = 0;
   out_4017855667471812861[144] = 0;
   out_4017855667471812861[145] = 0;
   out_4017855667471812861[146] = 0;
   out_4017855667471812861[147] = 0;
   out_4017855667471812861[148] = 0;
   out_4017855667471812861[149] = 0;
   out_4017855667471812861[150] = 0;
   out_4017855667471812861[151] = 0;
   out_4017855667471812861[152] = 1;
   out_4017855667471812861[153] = 0;
   out_4017855667471812861[154] = 0;
   out_4017855667471812861[155] = 0;
   out_4017855667471812861[156] = 0;
   out_4017855667471812861[157] = 0;
   out_4017855667471812861[158] = 0;
   out_4017855667471812861[159] = 0;
   out_4017855667471812861[160] = 0;
   out_4017855667471812861[161] = 0;
   out_4017855667471812861[162] = 0;
   out_4017855667471812861[163] = 0;
   out_4017855667471812861[164] = 0;
   out_4017855667471812861[165] = 0;
   out_4017855667471812861[166] = 0;
   out_4017855667471812861[167] = 0;
   out_4017855667471812861[168] = 0;
   out_4017855667471812861[169] = 0;
   out_4017855667471812861[170] = 0;
   out_4017855667471812861[171] = 1;
   out_4017855667471812861[172] = 0;
   out_4017855667471812861[173] = 0;
   out_4017855667471812861[174] = 0;
   out_4017855667471812861[175] = 0;
   out_4017855667471812861[176] = 0;
   out_4017855667471812861[177] = 0;
   out_4017855667471812861[178] = 0;
   out_4017855667471812861[179] = 0;
   out_4017855667471812861[180] = 0;
   out_4017855667471812861[181] = 0;
   out_4017855667471812861[182] = 0;
   out_4017855667471812861[183] = 0;
   out_4017855667471812861[184] = 0;
   out_4017855667471812861[185] = 0;
   out_4017855667471812861[186] = 0;
   out_4017855667471812861[187] = 0;
   out_4017855667471812861[188] = 0;
   out_4017855667471812861[189] = 0;
   out_4017855667471812861[190] = 1;
   out_4017855667471812861[191] = 0;
   out_4017855667471812861[192] = 0;
   out_4017855667471812861[193] = 0;
   out_4017855667471812861[194] = 0;
   out_4017855667471812861[195] = 0;
   out_4017855667471812861[196] = 0;
   out_4017855667471812861[197] = 0;
   out_4017855667471812861[198] = 0;
   out_4017855667471812861[199] = 0;
   out_4017855667471812861[200] = 0;
   out_4017855667471812861[201] = 0;
   out_4017855667471812861[202] = 0;
   out_4017855667471812861[203] = 0;
   out_4017855667471812861[204] = 0;
   out_4017855667471812861[205] = 0;
   out_4017855667471812861[206] = 0;
   out_4017855667471812861[207] = 0;
   out_4017855667471812861[208] = 0;
   out_4017855667471812861[209] = 1;
   out_4017855667471812861[210] = 0;
   out_4017855667471812861[211] = 0;
   out_4017855667471812861[212] = 0;
   out_4017855667471812861[213] = 0;
   out_4017855667471812861[214] = 0;
   out_4017855667471812861[215] = 0;
   out_4017855667471812861[216] = 0;
   out_4017855667471812861[217] = 0;
   out_4017855667471812861[218] = 0;
   out_4017855667471812861[219] = 0;
   out_4017855667471812861[220] = 0;
   out_4017855667471812861[221] = 0;
   out_4017855667471812861[222] = 0;
   out_4017855667471812861[223] = 0;
   out_4017855667471812861[224] = 0;
   out_4017855667471812861[225] = 0;
   out_4017855667471812861[226] = 0;
   out_4017855667471812861[227] = 0;
   out_4017855667471812861[228] = 1;
   out_4017855667471812861[229] = 0;
   out_4017855667471812861[230] = 0;
   out_4017855667471812861[231] = 0;
   out_4017855667471812861[232] = 0;
   out_4017855667471812861[233] = 0;
   out_4017855667471812861[234] = 0;
   out_4017855667471812861[235] = 0;
   out_4017855667471812861[236] = 0;
   out_4017855667471812861[237] = 0;
   out_4017855667471812861[238] = 0;
   out_4017855667471812861[239] = 0;
   out_4017855667471812861[240] = 0;
   out_4017855667471812861[241] = 0;
   out_4017855667471812861[242] = 0;
   out_4017855667471812861[243] = 0;
   out_4017855667471812861[244] = 0;
   out_4017855667471812861[245] = 0;
   out_4017855667471812861[246] = 0;
   out_4017855667471812861[247] = 1;
   out_4017855667471812861[248] = 0;
   out_4017855667471812861[249] = 0;
   out_4017855667471812861[250] = 0;
   out_4017855667471812861[251] = 0;
   out_4017855667471812861[252] = 0;
   out_4017855667471812861[253] = 0;
   out_4017855667471812861[254] = 0;
   out_4017855667471812861[255] = 0;
   out_4017855667471812861[256] = 0;
   out_4017855667471812861[257] = 0;
   out_4017855667471812861[258] = 0;
   out_4017855667471812861[259] = 0;
   out_4017855667471812861[260] = 0;
   out_4017855667471812861[261] = 0;
   out_4017855667471812861[262] = 0;
   out_4017855667471812861[263] = 0;
   out_4017855667471812861[264] = 0;
   out_4017855667471812861[265] = 0;
   out_4017855667471812861[266] = 1;
   out_4017855667471812861[267] = 0;
   out_4017855667471812861[268] = 0;
   out_4017855667471812861[269] = 0;
   out_4017855667471812861[270] = 0;
   out_4017855667471812861[271] = 0;
   out_4017855667471812861[272] = 0;
   out_4017855667471812861[273] = 0;
   out_4017855667471812861[274] = 0;
   out_4017855667471812861[275] = 0;
   out_4017855667471812861[276] = 0;
   out_4017855667471812861[277] = 0;
   out_4017855667471812861[278] = 0;
   out_4017855667471812861[279] = 0;
   out_4017855667471812861[280] = 0;
   out_4017855667471812861[281] = 0;
   out_4017855667471812861[282] = 0;
   out_4017855667471812861[283] = 0;
   out_4017855667471812861[284] = 0;
   out_4017855667471812861[285] = 1;
   out_4017855667471812861[286] = 0;
   out_4017855667471812861[287] = 0;
   out_4017855667471812861[288] = 0;
   out_4017855667471812861[289] = 0;
   out_4017855667471812861[290] = 0;
   out_4017855667471812861[291] = 0;
   out_4017855667471812861[292] = 0;
   out_4017855667471812861[293] = 0;
   out_4017855667471812861[294] = 0;
   out_4017855667471812861[295] = 0;
   out_4017855667471812861[296] = 0;
   out_4017855667471812861[297] = 0;
   out_4017855667471812861[298] = 0;
   out_4017855667471812861[299] = 0;
   out_4017855667471812861[300] = 0;
   out_4017855667471812861[301] = 0;
   out_4017855667471812861[302] = 0;
   out_4017855667471812861[303] = 0;
   out_4017855667471812861[304] = 1;
   out_4017855667471812861[305] = 0;
   out_4017855667471812861[306] = 0;
   out_4017855667471812861[307] = 0;
   out_4017855667471812861[308] = 0;
   out_4017855667471812861[309] = 0;
   out_4017855667471812861[310] = 0;
   out_4017855667471812861[311] = 0;
   out_4017855667471812861[312] = 0;
   out_4017855667471812861[313] = 0;
   out_4017855667471812861[314] = 0;
   out_4017855667471812861[315] = 0;
   out_4017855667471812861[316] = 0;
   out_4017855667471812861[317] = 0;
   out_4017855667471812861[318] = 0;
   out_4017855667471812861[319] = 0;
   out_4017855667471812861[320] = 0;
   out_4017855667471812861[321] = 0;
   out_4017855667471812861[322] = 0;
   out_4017855667471812861[323] = 1;
}
void h_4(double *state, double *unused, double *out_4813675346921411758) {
   out_4813675346921411758[0] = state[6] + state[9];
   out_4813675346921411758[1] = state[7] + state[10];
   out_4813675346921411758[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5986127290522570922) {
   out_5986127290522570922[0] = 0;
   out_5986127290522570922[1] = 0;
   out_5986127290522570922[2] = 0;
   out_5986127290522570922[3] = 0;
   out_5986127290522570922[4] = 0;
   out_5986127290522570922[5] = 0;
   out_5986127290522570922[6] = 1;
   out_5986127290522570922[7] = 0;
   out_5986127290522570922[8] = 0;
   out_5986127290522570922[9] = 1;
   out_5986127290522570922[10] = 0;
   out_5986127290522570922[11] = 0;
   out_5986127290522570922[12] = 0;
   out_5986127290522570922[13] = 0;
   out_5986127290522570922[14] = 0;
   out_5986127290522570922[15] = 0;
   out_5986127290522570922[16] = 0;
   out_5986127290522570922[17] = 0;
   out_5986127290522570922[18] = 0;
   out_5986127290522570922[19] = 0;
   out_5986127290522570922[20] = 0;
   out_5986127290522570922[21] = 0;
   out_5986127290522570922[22] = 0;
   out_5986127290522570922[23] = 0;
   out_5986127290522570922[24] = 0;
   out_5986127290522570922[25] = 1;
   out_5986127290522570922[26] = 0;
   out_5986127290522570922[27] = 0;
   out_5986127290522570922[28] = 1;
   out_5986127290522570922[29] = 0;
   out_5986127290522570922[30] = 0;
   out_5986127290522570922[31] = 0;
   out_5986127290522570922[32] = 0;
   out_5986127290522570922[33] = 0;
   out_5986127290522570922[34] = 0;
   out_5986127290522570922[35] = 0;
   out_5986127290522570922[36] = 0;
   out_5986127290522570922[37] = 0;
   out_5986127290522570922[38] = 0;
   out_5986127290522570922[39] = 0;
   out_5986127290522570922[40] = 0;
   out_5986127290522570922[41] = 0;
   out_5986127290522570922[42] = 0;
   out_5986127290522570922[43] = 0;
   out_5986127290522570922[44] = 1;
   out_5986127290522570922[45] = 0;
   out_5986127290522570922[46] = 0;
   out_5986127290522570922[47] = 1;
   out_5986127290522570922[48] = 0;
   out_5986127290522570922[49] = 0;
   out_5986127290522570922[50] = 0;
   out_5986127290522570922[51] = 0;
   out_5986127290522570922[52] = 0;
   out_5986127290522570922[53] = 0;
}
void h_10(double *state, double *unused, double *out_8589871310771143927) {
   out_8589871310771143927[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8589871310771143927[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8589871310771143927[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1877719131843322335) {
   out_1877719131843322335[0] = 0;
   out_1877719131843322335[1] = 9.8100000000000005*cos(state[1]);
   out_1877719131843322335[2] = 0;
   out_1877719131843322335[3] = 0;
   out_1877719131843322335[4] = -state[8];
   out_1877719131843322335[5] = state[7];
   out_1877719131843322335[6] = 0;
   out_1877719131843322335[7] = state[5];
   out_1877719131843322335[8] = -state[4];
   out_1877719131843322335[9] = 0;
   out_1877719131843322335[10] = 0;
   out_1877719131843322335[11] = 0;
   out_1877719131843322335[12] = 1;
   out_1877719131843322335[13] = 0;
   out_1877719131843322335[14] = 0;
   out_1877719131843322335[15] = 1;
   out_1877719131843322335[16] = 0;
   out_1877719131843322335[17] = 0;
   out_1877719131843322335[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1877719131843322335[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1877719131843322335[20] = 0;
   out_1877719131843322335[21] = state[8];
   out_1877719131843322335[22] = 0;
   out_1877719131843322335[23] = -state[6];
   out_1877719131843322335[24] = -state[5];
   out_1877719131843322335[25] = 0;
   out_1877719131843322335[26] = state[3];
   out_1877719131843322335[27] = 0;
   out_1877719131843322335[28] = 0;
   out_1877719131843322335[29] = 0;
   out_1877719131843322335[30] = 0;
   out_1877719131843322335[31] = 1;
   out_1877719131843322335[32] = 0;
   out_1877719131843322335[33] = 0;
   out_1877719131843322335[34] = 1;
   out_1877719131843322335[35] = 0;
   out_1877719131843322335[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1877719131843322335[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1877719131843322335[38] = 0;
   out_1877719131843322335[39] = -state[7];
   out_1877719131843322335[40] = state[6];
   out_1877719131843322335[41] = 0;
   out_1877719131843322335[42] = state[4];
   out_1877719131843322335[43] = -state[3];
   out_1877719131843322335[44] = 0;
   out_1877719131843322335[45] = 0;
   out_1877719131843322335[46] = 0;
   out_1877719131843322335[47] = 0;
   out_1877719131843322335[48] = 0;
   out_1877719131843322335[49] = 0;
   out_1877719131843322335[50] = 1;
   out_1877719131843322335[51] = 0;
   out_1877719131843322335[52] = 0;
   out_1877719131843322335[53] = 1;
}
void h_13(double *state, double *unused, double *out_3162448842721061253) {
   out_3162448842721061253[0] = state[3];
   out_3162448842721061253[1] = state[4];
   out_3162448842721061253[2] = state[5];
}
void H_13(double *state, double *unused, double *out_4849985574870279765) {
   out_4849985574870279765[0] = 0;
   out_4849985574870279765[1] = 0;
   out_4849985574870279765[2] = 0;
   out_4849985574870279765[3] = 1;
   out_4849985574870279765[4] = 0;
   out_4849985574870279765[5] = 0;
   out_4849985574870279765[6] = 0;
   out_4849985574870279765[7] = 0;
   out_4849985574870279765[8] = 0;
   out_4849985574870279765[9] = 0;
   out_4849985574870279765[10] = 0;
   out_4849985574870279765[11] = 0;
   out_4849985574870279765[12] = 0;
   out_4849985574870279765[13] = 0;
   out_4849985574870279765[14] = 0;
   out_4849985574870279765[15] = 0;
   out_4849985574870279765[16] = 0;
   out_4849985574870279765[17] = 0;
   out_4849985574870279765[18] = 0;
   out_4849985574870279765[19] = 0;
   out_4849985574870279765[20] = 0;
   out_4849985574870279765[21] = 0;
   out_4849985574870279765[22] = 1;
   out_4849985574870279765[23] = 0;
   out_4849985574870279765[24] = 0;
   out_4849985574870279765[25] = 0;
   out_4849985574870279765[26] = 0;
   out_4849985574870279765[27] = 0;
   out_4849985574870279765[28] = 0;
   out_4849985574870279765[29] = 0;
   out_4849985574870279765[30] = 0;
   out_4849985574870279765[31] = 0;
   out_4849985574870279765[32] = 0;
   out_4849985574870279765[33] = 0;
   out_4849985574870279765[34] = 0;
   out_4849985574870279765[35] = 0;
   out_4849985574870279765[36] = 0;
   out_4849985574870279765[37] = 0;
   out_4849985574870279765[38] = 0;
   out_4849985574870279765[39] = 0;
   out_4849985574870279765[40] = 0;
   out_4849985574870279765[41] = 1;
   out_4849985574870279765[42] = 0;
   out_4849985574870279765[43] = 0;
   out_4849985574870279765[44] = 0;
   out_4849985574870279765[45] = 0;
   out_4849985574870279765[46] = 0;
   out_4849985574870279765[47] = 0;
   out_4849985574870279765[48] = 0;
   out_4849985574870279765[49] = 0;
   out_4849985574870279765[50] = 0;
   out_4849985574870279765[51] = 0;
   out_4849985574870279765[52] = 0;
   out_4849985574870279765[53] = 0;
}
void h_14(double *state, double *unused, double *out_3168593434760140681) {
   out_3168593434760140681[0] = state[6];
   out_3168593434760140681[1] = state[7];
   out_3168593434760140681[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8497375926847496165) {
   out_8497375926847496165[0] = 0;
   out_8497375926847496165[1] = 0;
   out_8497375926847496165[2] = 0;
   out_8497375926847496165[3] = 0;
   out_8497375926847496165[4] = 0;
   out_8497375926847496165[5] = 0;
   out_8497375926847496165[6] = 1;
   out_8497375926847496165[7] = 0;
   out_8497375926847496165[8] = 0;
   out_8497375926847496165[9] = 0;
   out_8497375926847496165[10] = 0;
   out_8497375926847496165[11] = 0;
   out_8497375926847496165[12] = 0;
   out_8497375926847496165[13] = 0;
   out_8497375926847496165[14] = 0;
   out_8497375926847496165[15] = 0;
   out_8497375926847496165[16] = 0;
   out_8497375926847496165[17] = 0;
   out_8497375926847496165[18] = 0;
   out_8497375926847496165[19] = 0;
   out_8497375926847496165[20] = 0;
   out_8497375926847496165[21] = 0;
   out_8497375926847496165[22] = 0;
   out_8497375926847496165[23] = 0;
   out_8497375926847496165[24] = 0;
   out_8497375926847496165[25] = 1;
   out_8497375926847496165[26] = 0;
   out_8497375926847496165[27] = 0;
   out_8497375926847496165[28] = 0;
   out_8497375926847496165[29] = 0;
   out_8497375926847496165[30] = 0;
   out_8497375926847496165[31] = 0;
   out_8497375926847496165[32] = 0;
   out_8497375926847496165[33] = 0;
   out_8497375926847496165[34] = 0;
   out_8497375926847496165[35] = 0;
   out_8497375926847496165[36] = 0;
   out_8497375926847496165[37] = 0;
   out_8497375926847496165[38] = 0;
   out_8497375926847496165[39] = 0;
   out_8497375926847496165[40] = 0;
   out_8497375926847496165[41] = 0;
   out_8497375926847496165[42] = 0;
   out_8497375926847496165[43] = 0;
   out_8497375926847496165[44] = 1;
   out_8497375926847496165[45] = 0;
   out_8497375926847496165[46] = 0;
   out_8497375926847496165[47] = 0;
   out_8497375926847496165[48] = 0;
   out_8497375926847496165[49] = 0;
   out_8497375926847496165[50] = 0;
   out_8497375926847496165[51] = 0;
   out_8497375926847496165[52] = 0;
   out_8497375926847496165[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_1455056450359289667) {
  err_fun(nom_x, delta_x, out_1455056450359289667);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8825321344757757707) {
  inv_err_fun(nom_x, true_x, out_8825321344757757707);
}
void pose_H_mod_fun(double *state, double *out_1371500253593254558) {
  H_mod_fun(state, out_1371500253593254558);
}
void pose_f_fun(double *state, double dt, double *out_6239891866939865755) {
  f_fun(state,  dt, out_6239891866939865755);
}
void pose_F_fun(double *state, double dt, double *out_4017855667471812861) {
  F_fun(state,  dt, out_4017855667471812861);
}
void pose_h_4(double *state, double *unused, double *out_4813675346921411758) {
  h_4(state, unused, out_4813675346921411758);
}
void pose_H_4(double *state, double *unused, double *out_5986127290522570922) {
  H_4(state, unused, out_5986127290522570922);
}
void pose_h_10(double *state, double *unused, double *out_8589871310771143927) {
  h_10(state, unused, out_8589871310771143927);
}
void pose_H_10(double *state, double *unused, double *out_1877719131843322335) {
  H_10(state, unused, out_1877719131843322335);
}
void pose_h_13(double *state, double *unused, double *out_3162448842721061253) {
  h_13(state, unused, out_3162448842721061253);
}
void pose_H_13(double *state, double *unused, double *out_4849985574870279765) {
  H_13(state, unused, out_4849985574870279765);
}
void pose_h_14(double *state, double *unused, double *out_3168593434760140681) {
  h_14(state, unused, out_3168593434760140681);
}
void pose_H_14(double *state, double *unused, double *out_8497375926847496165) {
  H_14(state, unused, out_8497375926847496165);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
