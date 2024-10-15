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
  #define CASADI_PREFIX(ID) acados_mpcc_cost_ext_cost_fun_jac_hess_ ## ID
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
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s4[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const casadi_int casadi_s5[32] = {15, 15, 0, 1, 2, 3, 4, 7, 10, 10, 10, 10, 10, 13, 13, 13, 13, 14, 0, 1, 2, 3, 4, 5, 10, 4, 5, 10, 4, 5, 10, 14};

casadi_real casadi_sq(casadi_real x) { return x*x;}

/* acados_mpcc_cost_ext_cost_fun_jac_hess:(i0[11],i1[4],i2[],i3[4])->(o0,o1[15],o2[15x15,14nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0000000000000001e-01;
  a1=arg[3]? arg[3][2] : 0;
  a2=sin(a1);
  a3=arg[3]? arg[3][0] : 0;
  a4=arg[0]? arg[0][6] : 0;
  a5=arg[3]? arg[3][3] : 0;
  a6=(a4-a5);
  a7=cos(a1);
  a6=(a6*a7);
  a3=(a3+a6);
  a6=arg[0]? arg[0][0] : 0;
  a8=(a3-a6);
  a8=(a2*a8);
  a9=cos(a1);
  a10=arg[3]? arg[3][1] : 0;
  a4=(a4-a5);
  a5=sin(a1);
  a4=(a4*a5);
  a10=(a10+a4);
  a4=arg[0]? arg[0][1] : 0;
  a11=(a10-a4);
  a11=(a9*a11);
  a8=(a8-a11);
  a11=(a0*a8);
  a12=(a11*a8);
  a13=100.;
  a14=cos(a1);
  a3=(a3-a6);
  a3=(a14*a3);
  a1=sin(a1);
  a10=(a10-a4);
  a10=(a1*a10);
  a3=(a3+a10);
  a10=(a13*a3);
  a4=(a10*a3);
  a12=(a12+a4);
  a4=1.7361111111111110e-03;
  a6=arg[1]? arg[1][0] : 0;
  a15=(a4*a6);
  a16=(a15*a6);
  a17=500.;
  a18=arg[1]? arg[1][1] : 0;
  a19=(a17*a18);
  a20=(a19*a18);
  a16=(a16+a20);
  a20=2.8843014394971626e-05;
  a21=arg[1]? arg[1][2] : 0;
  a22=(a20*a21);
  a23=(a22*a21);
  a16=(a16+a23);
  a23=1.0000000000000000e-04;
  a24=arg[1]? arg[1][3] : 0;
  a25=(a23*a24);
  a26=(a25*a24);
  a16=(a16+a26);
  a12=(a12+a16);
  a16=2.0000000000000001e-01;
  a26=30.;
  a27=arg[0]? arg[0][10] : 0;
  a26=(a26-a27);
  a27=casadi_sq(a26);
  a27=(a16*a27);
  a12=(a12+a27);
  if (res[0]!=0) res[0][0]=a12;
  a4=(a4*a6);
  a15=(a15+a4);
  if (res[1]!=0) res[1][0]=a15;
  a17=(a17*a18);
  a19=(a19+a17);
  if (res[1]!=0) res[1][1]=a19;
  a20=(a20*a21);
  a22=(a22+a20);
  if (res[1]!=0) res[1][2]=a22;
  a23=(a23*a24);
  a25=(a25+a23);
  if (res[1]!=0) res[1][3]=a25;
  a3=(a13*a3);
  a10=(a10+a3);
  a3=(a14*a10);
  a8=(a0*a8);
  a11=(a11+a8);
  a8=(a2*a11);
  a25=(a3+a8);
  a25=(-a25);
  if (res[1]!=0) res[1][4]=a25;
  a11=(a9*a11);
  a10=(a1*a10);
  a25=(a11-a10);
  if (res[1]!=0) res[1][5]=a25;
  a25=0.;
  if (res[1]!=0) res[1][6]=a25;
  if (res[1]!=0) res[1][7]=a25;
  if (res[1]!=0) res[1][8]=a25;
  if (res[1]!=0) res[1][9]=a25;
  a10=(a10-a11);
  a10=(a5*a10);
  a3=(a3+a8);
  a3=(a7*a3);
  a10=(a10+a3);
  if (res[1]!=0) res[1][10]=a10;
  if (res[1]!=0) res[1][11]=a25;
  if (res[1]!=0) res[1][12]=a25;
  if (res[1]!=0) res[1][13]=a25;
  a26=(a26+a26);
  a16=(a16*a26);
  a16=(-a16);
  if (res[1]!=0) res[1][14]=a16;
  a16=3.4722222222222220e-03;
  if (res[2]!=0) res[2][0]=a16;
  a16=1000.;
  if (res[2]!=0) res[2][1]=a16;
  a16=5.7686028789943251e-05;
  if (res[2]!=0) res[2][2]=a16;
  a16=2.0000000000000001e-04;
  if (res[2]!=0) res[2][3]=a16;
  a16=(a13*a14);
  a26=(a13*a14);
  a16=(a16+a26);
  a16=(a14*a16);
  a26=(a0*a2);
  a25=(a0*a2);
  a26=(a26+a25);
  a26=(a2*a26);
  a16=(a16+a26);
  if (res[2]!=0) res[2][4]=a16;
  a16=(a0*a9);
  a26=(a0*a9);
  a16=(a16+a26);
  a26=(a2*a16);
  a25=(a13*a1);
  a13=(a13*a1);
  a25=(a25+a13);
  a13=(a14*a25);
  a26=(a26-a13);
  a26=(-a26);
  if (res[2]!=0) res[2][5]=a26;
  a13=200.;
  a14=(a13*a14);
  a10=(a2*a7);
  a3=(a9*a5);
  a10=(a10-a3);
  a3=(a0*a10);
  a0=(a0*a10);
  a3=(a3+a0);
  a2=(a2*a3);
  a0=(a14+a2);
  a0=(-a0);
  if (res[2]!=0) res[2][6]=a0;
  if (res[2]!=0) res[2][7]=a26;
  a16=(a9*a16);
  a25=(a1*a25);
  a16=(a16+a25);
  if (res[2]!=0) res[2][8]=a16;
  a9=(a9*a3);
  a13=(a13*a1);
  a1=(a9-a13);
  if (res[2]!=0) res[2][9]=a1;
  if (res[2]!=0) res[2][10]=a0;
  if (res[2]!=0) res[2][11]=a1;
  a13=(a13-a9);
  a5=(a5*a13);
  a14=(a14+a2);
  a7=(a7*a14);
  a5=(a5+a7);
  if (res[2]!=0) res[2][12]=a5;
  a5=4.0000000000000002e-01;
  if (res[2]!=0) res[2][13]=a5;
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_cost_ext_cost_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int acados_mpcc_cost_ext_cost_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_cost_ext_cost_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_cost_ext_cost_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int acados_mpcc_cost_ext_cost_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_cost_ext_cost_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_cost_ext_cost_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_cost_ext_cost_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_cost_ext_cost_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_cost_ext_cost_fun_jac_hess_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real acados_mpcc_cost_ext_cost_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_cost_ext_cost_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_cost_ext_cost_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_cost_ext_cost_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_cost_ext_cost_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int acados_mpcc_cost_ext_cost_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
