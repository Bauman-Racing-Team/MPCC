/* This file was automatically generated by CasADi 3.6.6.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) acados_mpcc_constr_h_fun_ ## ID
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

static const casadi_int casadi_s0[15] = {11, 1, 0, 11, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};

/* acados_mpcc_constr_h_fun:(i0[11],i1[4],i2[],i3[11])->(o0[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][4] : 0;
  a1=7.6500000000000001e-01;
  a2=arg[0]? arg[0][5] : 0;
  a1=(a1*a2);
  a2=(a0+a1);
  a3=arg[0]? arg[0][3] : 0;
  a2=atan2(a2,a3);
  a4=arg[0]? arg[0][8] : 0;
  a2=(a2-a4);
  a4=3.;
  a4=(a3-a4);
  a5=2.;
  a4=(a4/a5);
  a5=0.;
  a4=casadi_fmax(a4,a5);
  a5=1.;
  a4=casadi_fmin(a4,a5);
  a5=(a2*a4);
  if (res[0]!=0) res[0][0]=a5;
  a0=(a0-a1);
  a0=atan2(a0,a3);
  a4=(a0*a4);
  if (res[0]!=0) res[0][1]=a4;
  a4=arg[0]? arg[0][0] : 0;
  a1=arg[3]? arg[3][0] : 0;
  a4=(a4-a1);
  a4=casadi_sq(a4);
  a1=arg[0]? arg[0][1] : 0;
  a5=arg[3]? arg[3][1] : 0;
  a1=(a1-a5);
  a1=casadi_sq(a1);
  a4=(a4+a1);
  if (res[0]!=0) res[0][2]=a4;
  a4=-3.4830404999999999e+01;
  a3=tanh(a3);
  a4=(a4*a3);
  a1=2.0419999999999998e+00;
  a5=arg[0]? arg[0][9] : 0;
  a1=(a1*a5);
  a6=3.0419999999999998e+00;
  a1=(a1/a6);
  a7=2.3100000000000001e-01;
  a1=(a1/a7);
  a1=(a1*a3);
  a1=(a4-a1);
  a8=2.1180400000000000e+03;
  a1=(a1/a8);
  a1=casadi_sq(a1);
  a9=4.0609311069375003e+03;
  a10=1.4339999999999999e+00;
  a11=-1.0886514426173051e+01;
  a2=(a11*a2);
  a12=-3.6747531249999998e-01;
  a13=atan(a2);
  a13=(a2-a13);
  a13=(a12*a13);
  a2=(a2-a13);
  a2=atan(a2);
  a2=(a10*a2);
  a2=sin(a2);
  a2=(a9*a2);
  a13=2.0305000000000000e+03;
  a2=(a2/a13);
  a2=casadi_sq(a2);
  a1=(a1+a2);
  if (res[0]!=0) res[0][3]=a1;
  a1=3.5000000000000000e+00;
  a2=arg[0]? arg[0][7] : 0;
  a1=(a1*a2);
  a1=(a1/a7);
  a6=(a5/a6);
  a6=(a6/a7);
  a6=(a6*a3);
  a1=(a1-a6);
  a1=(a1+a4);
  a1=(a1/a8);
  a1=casadi_sq(a1);
  a11=(a11*a0);
  a0=atan(a11);
  a0=(a11-a0);
  a12=(a12*a0);
  a11=(a11-a12);
  a11=atan(a11);
  a10=(a10*a11);
  a10=sin(a10);
  a9=(a9*a10);
  a9=(a9/a13);
  a9=casadi_sq(a9);
  a1=(a1+a9);
  if (res[0]!=0) res[0][4]=a1;
  a2=(a2*a5);
  if (res[0]!=0) res[0][5]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void acados_mpcc_constr_h_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_constr_h_fun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int acados_mpcc_constr_h_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real acados_mpcc_constr_h_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_constr_h_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* acados_mpcc_constr_h_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_constr_h_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* acados_mpcc_constr_h_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int acados_mpcc_constr_h_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
