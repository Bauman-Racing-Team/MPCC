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
  #define CASADI_PREFIX(ID) acados_mpcc_expl_ode_fun_ ## ID
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

/* acados_mpcc_expl_ode_fun:(i0[11],i1[4],i2[11])->(o0[11]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][3] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a2=(a0*a2);
  a3=arg[0]? arg[0][4] : 0;
  a4=sin(a1);
  a4=(a3*a4);
  a2=(a2-a4);
  if (res[0]!=0) res[0][0]=a2;
  a2=sin(a1);
  a2=(a0*a2);
  a1=cos(a1);
  a1=(a3*a1);
  a2=(a2+a1);
  if (res[0]!=0) res[0][1]=a2;
  a2=arg[0]? arg[0][5] : 0;
  if (res[0]!=0) res[0][2]=a2;
  a1=3.;
  a1=(a0-a1);
  a4=2.;
  a1=(a1/a4);
  a4=0.;
  a1=casadi_fmax(a1,a4);
  a4=1.;
  a1=casadi_fmin(a1,a4);
  a5=3.3333333333333335e-03;
  a6=3.5000000000000000e+00;
  a7=arg[0]? arg[0][7] : 0;
  a8=(a6*a7);
  a9=2.3100000000000001e-01;
  a8=(a8/a9);
  a10=arg[0]? arg[0][9] : 0;
  a11=3.0419999999999998e+00;
  a12=(a10/a11);
  a12=(a12/a9);
  a13=tanh(a0);
  a12=(a12*a13);
  a8=(a8-a12);
  a12=-3.4830404999999999e+01;
  a13=tanh(a0);
  a13=(a12*a13);
  a8=(a8+a13);
  a13=arg[0]? arg[0][8] : 0;
  a14=cos(a13);
  a15=tanh(a0);
  a15=(a12*a15);
  a16=2.0419999999999998e+00;
  a17=(a16*a10);
  a17=(a17/a11);
  a17=(a17/a9);
  a18=tanh(a0);
  a17=(a17*a18);
  a15=(a15-a17);
  a14=(a14*a15);
  a8=(a8+a14);
  a14=casadi_sq(a0);
  a8=(a8-a14);
  a14=sin(a13);
  a17=29700.;
  a18=7.6500000000000001e-01;
  a19=(a18*a2);
  a19=(a3+a19);
  a19=atan2(a19,a0);
  a19=(a19-a13);
  a19=(a17*a19);
  a14=(a14*a19);
  a8=(a8+a14);
  a14=300.;
  a20=(a14*a3);
  a20=(a20*a2);
  a8=(a8+a20);
  a8=(a5*a8);
  a8=(a1*a8);
  a20=(a4-a1);
  a6=(a6*a7);
  a6=(a6/a9);
  a7=(a10/a11);
  a7=(a7/a9);
  a21=tanh(a0);
  a7=(a7*a21);
  a6=(a6-a7);
  a7=tanh(a0);
  a7=(a12*a7);
  a6=(a6+a7);
  a7=cos(a13);
  a21=tanh(a0);
  a12=(a12*a21);
  a16=(a16*a10);
  a16=(a16/a11);
  a16=(a16/a9);
  a9=tanh(a0);
  a16=(a16*a9);
  a12=(a12-a16);
  a7=(a7*a12);
  a6=(a6+a7);
  a7=casadi_sq(a0);
  a6=(a6-a7);
  a6=(a5*a6);
  a20=(a20*a6);
  a8=(a8+a20);
  if (res[0]!=0) res[0][3]=a8;
  a8=sin(a13);
  a8=(a8*a15);
  a20=(a18*a2);
  a3=(a3-a20);
  a3=atan2(a3,a0);
  a17=(a17*a3);
  a3=cos(a13);
  a3=(a3*a19);
  a3=(a17+a3);
  a8=(a8-a3);
  a14=(a14*a0);
  a14=(a14*a2);
  a8=(a8-a14);
  a5=(a5*a8);
  a5=(a1*a5);
  a8=(a4-a1);
  a14=5.0000000000000000e-01;
  a2=arg[1]? arg[1][1] : 0;
  a3=(a2*a0);
  a20=(a13*a6);
  a3=(a3+a20);
  a14=(a14*a3);
  a8=(a8*a14);
  a5=(a5+a8);
  if (res[0]!=0) res[0][4]=a5;
  a5=9.0909090909090905e-03;
  a17=(a18*a17);
  a8=sin(a13);
  a8=(a8*a15);
  a15=cos(a13);
  a15=(a15*a19);
  a8=(a8-a15);
  a18=(a18*a8);
  a17=(a17+a18);
  a5=(a5*a17);
  a5=(a1*a5);
  a4=(a4-a1);
  a1=6.5359477124183007e-01;
  a0=(a2*a0);
  a13=(a13*a6);
  a0=(a0+a13);
  a1=(a1*a0);
  a4=(a4*a1);
  a5=(a5+a4);
  if (res[0]!=0) res[0][5]=a5;
  a5=arg[0]? arg[0][10] : 0;
  if (res[0]!=0) res[0][6]=a5;
  a5=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][7]=a5;
  if (res[0]!=0) res[0][8]=a2;
  a2=arg[1]? arg[1][2] : 0;
  if (res[0]!=0) res[0][9]=a2;
  a2=arg[1]? arg[1][3] : 0;
  if (res[0]!=0) res[0][10]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int acados_mpcc_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int acados_mpcc_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real acados_mpcc_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int acados_mpcc_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
