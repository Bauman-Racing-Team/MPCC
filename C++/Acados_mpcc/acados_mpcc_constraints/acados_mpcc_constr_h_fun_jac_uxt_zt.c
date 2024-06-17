/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) acados_mpcc_constr_h_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fmax CASADI_PREFIX(fmax)
#define casadi_fmin CASADI_PREFIX(fmin)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[15] = {11, 1, 0, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s4[30] = {15, 6, 0, 4, 7, 9, 14, 19, 21, 7, 8, 9, 12, 7, 8, 9, 4, 5, 7, 8, 9, 12, 13, 7, 8, 9, 11, 13, 11, 13};
static const casadi_int casadi_s5[9] = {0, 6, 0, 0, 0, 0, 0, 0, 0};

casadi_real casadi_fmax(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>y ? x : y;
#else
  return fmax(x, y);
#endif
}

casadi_real casadi_fmin(casadi_real x, casadi_real y) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x<y ? x : y;
#else
  return fmin(x, y);
#endif
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

/* acados_mpcc_constr_h_fun_jac_uxt_zt:(i0[11],i1[4],i2[],i3[11])->(o0[6],o1[15x6,21nz],o2[0x6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][4] : 0;
  a1=7.6500000000000001e-01;
  a2=arg[0]? arg[0][5] : 0;
  a3=(a1*a2);
  a3=(a0+a3);
  a4=arg[0]? arg[0][3] : 0;
  a5=atan2(a3,a4);
  a6=arg[0]? arg[0][8] : 0;
  a5=(a5-a6);
  a7=3.;
  a7=(a4-a7);
  a8=2.;
  a7=(a7/a8);
  a8=0.;
  a9=casadi_fmax(a7,a8);
  a10=1.;
  a11=casadi_fmin(a9,a10);
  a12=(a5*a11);
  if (res[0]!=0) res[0][0]=a12;
  a12=(a1*a2);
  a12=(a0-a12);
  a13=atan2(a12,a4);
  a14=(a13*a11);
  if (res[0]!=0) res[0][1]=a14;
  a14=arg[0]? arg[0][0] : 0;
  a15=arg[3]? arg[3][0] : 0;
  a14=(a14-a15);
  a15=casadi_sq(a14);
  a16=arg[0]? arg[0][1] : 0;
  a17=arg[3]? arg[3][1] : 0;
  a16=(a16-a17);
  a17=casadi_sq(a16);
  a15=(a15+a17);
  if (res[0]!=0) res[0][2]=a15;
  a15=-1250.;
  a17=arg[0]? arg[0][9] : 0;
  a15=(a15*a17);
  a18=2.3100000000000001e-01;
  a15=(a15/a18);
  a19=tanh(a4);
  a20=(a15*a19);
  a21=-3.4830404999999999e+01;
  a22=tanh(a4);
  a23=(a21*a22);
  a20=(a20+a23);
  a23=2.1180400000000000e+03;
  a20=(a20/a23);
  a24=casadi_sq(a20);
  a25=4.0609311069375003e+03;
  a26=1.4339999999999999e+00;
  a27=-1.0886514426173051e+01;
  a28=(a1*a2);
  a28=(a0+a28);
  a29=atan2(a28,a4);
  a29=(a29-a6);
  a6=(a27*a29);
  a30=-3.6747531249999998e-01;
  a31=(a27*a29);
  a29=(a27*a29);
  a32=atan(a29);
  a31=(a31-a32);
  a31=(a30*a31);
  a6=(a6-a31);
  a31=atan(a6);
  a31=(a26*a31);
  a32=sin(a31);
  a32=(a25*a32);
  a33=2.0305000000000000e+03;
  a32=(a32/a33);
  a34=casadi_sq(a32);
  a24=(a24+a34);
  if (res[0]!=0) res[0][3]=a24;
  a24=-612.;
  a24=(a24*a17);
  a24=(a24/a18);
  a34=tanh(a4);
  a35=(a24*a34);
  a36=840.;
  a37=arg[0]? arg[0][7] : 0;
  a36=(a36*a37);
  a36=(a36/a18);
  a35=(a35+a36);
  a36=tanh(a4);
  a18=(a21*a36);
  a35=(a35+a18);
  a35=(a35/a23);
  a23=casadi_sq(a35);
  a2=(a1*a2);
  a0=(a0-a2);
  a2=atan2(a0,a4);
  a18=(a27*a2);
  a38=(a27*a2);
  a2=(a27*a2);
  a39=atan(a2);
  a38=(a38-a39);
  a38=(a30*a38);
  a18=(a18-a38);
  a38=atan(a18);
  a38=(a26*a38);
  a39=sin(a38);
  a39=(a25*a39);
  a39=(a39/a33);
  a33=casadi_sq(a39);
  a23=(a23+a33);
  if (res[0]!=0) res[0][4]=a23;
  a23=(a37*a17);
  if (res[0]!=0) res[0][5]=a23;
  a9=(a9<=a10);
  a23=5.0000000000000000e-01;
  a8=(a8<=a7);
  a23=(a23*a8);
  a9=(a9*a23);
  a5=(a5*a9);
  a23=casadi_sq(a3);
  a8=casadi_sq(a4);
  a23=(a23+a8);
  a3=(a3/a23);
  a3=(a11*a3);
  a5=(a5-a3);
  if (res[1]!=0) res[1][0]=a5;
  a23=(a4/a23);
  a5=(a11*a23);
  if (res[1]!=0) res[1][1]=a5;
  a23=(a1*a23);
  a23=(a11*a23);
  if (res[1]!=0) res[1][2]=a23;
  a23=(-a11);
  if (res[1]!=0) res[1][3]=a23;
  a13=(a13*a9);
  a9=casadi_sq(a12);
  a23=casadi_sq(a4);
  a9=(a9+a23);
  a12=(a12/a9);
  a12=(a11*a12);
  a13=(a13-a12);
  if (res[1]!=0) res[1][4]=a13;
  a9=(a4/a9);
  a13=(a11*a9);
  if (res[1]!=0) res[1][5]=a13;
  a13=-7.6500000000000001e-01;
  a9=(a13*a9);
  a11=(a11*a9);
  if (res[1]!=0) res[1][6]=a11;
  a14=(a14+a14);
  if (res[1]!=0) res[1][7]=a14;
  a16=(a16+a16);
  if (res[1]!=0) res[1][8]=a16;
  a20=(a20+a20);
  a16=4.7213461502143491e-04;
  a14=casadi_sq(a19);
  a14=(a10-a14);
  a15=(a15*a14);
  a22=casadi_sq(a22);
  a22=(a10-a22);
  a22=(a21*a22);
  a15=(a15+a22);
  a15=(a16*a15);
  a15=(a20*a15);
  a32=(a32+a32);
  a22=4.9248953459738983e-04;
  a31=cos(a31);
  a14=casadi_sq(a28);
  a11=casadi_sq(a4);
  a14=(a14+a11);
  a28=(a28/a14);
  a11=(a27*a28);
  a9=(a27*a28);
  a29=casadi_sq(a29);
  a29=(a10+a29);
  a9=(a9/a29);
  a28=(a27*a28);
  a9=(a9-a28);
  a9=(a30*a9);
  a11=(a11+a9);
  a6=casadi_sq(a6);
  a6=(a10+a6);
  a11=(a11/a6);
  a11=(a26*a11);
  a11=(a31*a11);
  a11=(a25*a11);
  a11=(a22*a11);
  a11=(a32*a11);
  a15=(a15-a11);
  if (res[1]!=0) res[1][9]=a15;
  a14=(a4/a14);
  a15=(a27*a14);
  a11=(a27*a14);
  a9=(a27*a14);
  a9=(a9/a29);
  a11=(a11-a9);
  a11=(a30*a11);
  a15=(a15-a11);
  a15=(a15/a6);
  a15=(a26*a15);
  a15=(a31*a15);
  a15=(a25*a15);
  a15=(a22*a15);
  a15=(a32*a15);
  if (res[1]!=0) res[1][10]=a15;
  a1=(a1*a14);
  a14=(a27*a1);
  a15=(a27*a1);
  a1=(a27*a1);
  a1=(a1/a29);
  a15=(a15-a1);
  a15=(a30*a15);
  a14=(a14-a15);
  a14=(a14/a6);
  a14=(a26*a14);
  a14=(a31*a14);
  a14=(a25*a14);
  a14=(a22*a14);
  a14=(a32*a14);
  if (res[1]!=0) res[1][11]=a14;
  a14=1.0886514426173051e+01;
  a29=(a14/a29);
  a29=(a14-a29);
  a29=(a30*a29);
  a14=(a14-a29);
  a14=(a14/a6);
  a14=(a26*a14);
  a31=(a31*a14);
  a31=(a25*a31);
  a31=(a22*a31);
  a32=(a32*a31);
  if (res[1]!=0) res[1][12]=a32;
  a32=-5.4112554112554108e+03;
  a32=(a32*a19);
  a32=(a16*a32);
  a20=(a20*a32);
  if (res[1]!=0) res[1][13]=a20;
  a35=(a35+a35);
  a20=casadi_sq(a34);
  a20=(a10-a20);
  a24=(a24*a20);
  a36=casadi_sq(a36);
  a36=(a10-a36);
  a21=(a21*a36);
  a24=(a24+a21);
  a24=(a16*a24);
  a24=(a35*a24);
  a39=(a39+a39);
  a38=cos(a38);
  a21=casadi_sq(a0);
  a36=casadi_sq(a4);
  a21=(a21+a36);
  a0=(a0/a21);
  a36=(a27*a0);
  a20=(a27*a0);
  a2=casadi_sq(a2);
  a2=(a10+a2);
  a20=(a20/a2);
  a0=(a27*a0);
  a20=(a20-a0);
  a20=(a30*a20);
  a36=(a36+a20);
  a18=casadi_sq(a18);
  a10=(a10+a18);
  a36=(a36/a10);
  a36=(a26*a36);
  a36=(a38*a36);
  a36=(a25*a36);
  a36=(a22*a36);
  a36=(a39*a36);
  a24=(a24-a36);
  if (res[1]!=0) res[1][14]=a24;
  a4=(a4/a21);
  a21=(a27*a4);
  a24=(a27*a4);
  a36=(a27*a4);
  a36=(a36/a2);
  a24=(a24-a36);
  a24=(a30*a24);
  a21=(a21-a24);
  a21=(a21/a10);
  a21=(a26*a21);
  a21=(a38*a21);
  a21=(a25*a21);
  a21=(a22*a21);
  a21=(a39*a21);
  if (res[1]!=0) res[1][15]=a21;
  a13=(a13*a4);
  a4=(a27*a13);
  a21=(a27*a13);
  a27=(a27*a13);
  a27=(a27/a2);
  a21=(a21-a27);
  a30=(a30*a21);
  a4=(a4-a30);
  a4=(a4/a10);
  a26=(a26*a4);
  a38=(a38*a26);
  a25=(a25*a38);
  a22=(a22*a25);
  a39=(a39*a22);
  if (res[1]!=0) res[1][16]=a39;
  a39=1.7168531455324907e+00;
  a39=(a39*a35);
  if (res[1]!=0) res[1][17]=a39;
  a39=-2.6493506493506493e+03;
  a39=(a39*a34);
  a16=(a16*a39);
  a35=(a35*a16);
  if (res[1]!=0) res[1][18]=a35;
  if (res[1]!=0) res[1][19]=a17;
  if (res[1]!=0) res[1][20]=a37;
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real acados_mpcc_constr_h_fun_jac_uxt_zt_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_constr_h_fun_jac_uxt_zt_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_constr_h_fun_jac_uxt_zt_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
