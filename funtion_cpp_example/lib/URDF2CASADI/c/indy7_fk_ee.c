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
  #define CASADI_PREFIX(ID) indy7_fk_ee_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)

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

static const casadi_int casadi_s0[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s1[23] = {4, 4, 0, 4, 8, 12, 16, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};

/* fk_T:(q[6])->(T_indy7_tcp[4x4]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=cos(a0);
  a2=2.2204460492503131e-16;
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=(a2*a4);
  a3=sin(a3);
  a5=(a5+a3);
  a6=(a1*a5);
  a0=sin(a0);
  a7=(a2*a3);
  a8=(a0*a7);
  a6=(a6-a8);
  a8=arg[0]? arg[0][2] : 0;
  a9=cos(a8);
  a10=(a6*a9);
  a11=(a2*a3);
  a11=(a4-a11);
  a12=(a1*a11);
  a13=(a2*a4);
  a14=(a0*a13);
  a12=(a12-a14);
  a8=sin(a8);
  a14=(a12*a8);
  a10=(a10+a14);
  a14=arg[0]? arg[0][3] : 0;
  a15=cos(a14);
  a16=(a2*a15);
  a17=-2.2204460492503131e-16;
  a14=sin(a14);
  a18=(a17*a14);
  a16=(a16+a18);
  a18=(a10*a16);
  a12=(a12*a9);
  a19=(a6*a8);
  a12=(a12-a19);
  a19=(a12*a15);
  a20=(a2*a1);
  a20=(a20+a0);
  a21=(a20*a14);
  a19=(a19-a21);
  a18=(a18+a19);
  a19=arg[0]? arg[0][4] : 0;
  a21=cos(a19);
  a22=(a2*a21);
  a19=sin(a19);
  a22=(a22+a19);
  a23=(a18*a22);
  a24=(a17*a15);
  a25=(a2*a14);
  a24=(a24-a25);
  a25=(a10*a24);
  a26=(a12*a14);
  a27=(a20*a15);
  a26=(a26+a27);
  a25=(a25-a26);
  a26=(a2*a19);
  a27=(a25*a26);
  a12=(a2*a12);
  a28=(a2*a20);
  a12=(a12+a28);
  a12=(a12-a10);
  a28=(a2*a19);
  a28=(a28-a21);
  a29=(a12*a28);
  a27=(a27+a29);
  a23=(a23+a27);
  a27=arg[0]? arg[0][5] : 0;
  a29=cos(a27);
  a30=(a2*a29);
  a27=sin(a27);
  a31=(a17*a27);
  a30=(a30+a31);
  a31=(a23*a30);
  a32=(a2*a19);
  a32=(a21-a32);
  a33=(a18*a32);
  a34=(a2*a21);
  a35=(a25*a34);
  a21=(a2*a21);
  a19=(a19+a21);
  a21=(a12*a19);
  a35=(a35+a21);
  a33=(a33+a35);
  a35=(a33*a29);
  a18=(a2*a18);
  a18=(a18-a25);
  a21=(a18*a27);
  a35=(a35-a21);
  a31=(a31+a35);
  if (res[0]!=0) res[0][0]=a31;
  a5=(a0*a5);
  a7=(a1*a7);
  a5=(a5+a7);
  a7=(a5*a9);
  a11=(a0*a11);
  a13=(a1*a13);
  a11=(a11+a13);
  a13=(a11*a8);
  a7=(a7+a13);
  a13=(a7*a16);
  a11=(a11*a9);
  a31=(a5*a8);
  a11=(a11-a31);
  a31=(a11*a15);
  a35=(a2*a0);
  a35=(a35-a1);
  a21=(a35*a14);
  a31=(a31-a21);
  a13=(a13+a31);
  a31=(a13*a22);
  a21=(a7*a24);
  a36=(a11*a14);
  a37=(a35*a15);
  a36=(a36+a37);
  a21=(a21-a36);
  a36=(a21*a26);
  a11=(a2*a11);
  a37=(a2*a35);
  a11=(a11+a37);
  a11=(a11-a7);
  a37=(a11*a28);
  a36=(a36+a37);
  a31=(a31+a36);
  a36=(a31*a30);
  a37=(a13*a32);
  a38=(a21*a34);
  a39=(a11*a19);
  a38=(a38+a39);
  a37=(a37+a38);
  a38=(a37*a29);
  a13=(a2*a13);
  a13=(a13-a21);
  a39=(a13*a27);
  a38=(a38-a39);
  a36=(a36+a38);
  if (res[0]!=0) res[0][1]=a36;
  a36=(a2*a3);
  a36=(a36-a4);
  a38=(a36*a9);
  a4=(a2*a4);
  a3=(a3+a4);
  a4=(a3*a8);
  a38=(a38+a4);
  a16=(a38*a16);
  a3=(a3*a9);
  a8=(a36*a8);
  a3=(a3-a8);
  a15=(a3*a15);
  a16=(a16+a15);
  a22=(a16*a22);
  a24=(a38*a24);
  a14=(a3*a14);
  a24=(a24-a14);
  a26=(a24*a26);
  a3=(a2*a3);
  a3=(a3-a38);
  a28=(a3*a28);
  a26=(a26+a28);
  a22=(a22+a26);
  a30=(a22*a30);
  a32=(a16*a32);
  a34=(a24*a34);
  a19=(a3*a19);
  a34=(a34+a19);
  a32=(a32+a34);
  a34=(a32*a29);
  a16=(a2*a16);
  a16=(a16-a24);
  a19=(a16*a27);
  a34=(a34-a19);
  a30=(a30+a34);
  if (res[0]!=0) res[0][2]=a30;
  a30=0.;
  if (res[0]!=0) res[0][3]=a30;
  a17=(a17*a29);
  a34=(a2*a27);
  a17=(a17-a34);
  a34=(a23*a17);
  a19=(a33*a27);
  a26=(a18*a29);
  a19=(a19+a26);
  a34=(a34-a19);
  if (res[0]!=0) res[0][4]=a34;
  a34=(a31*a17);
  a19=(a37*a27);
  a26=(a13*a29);
  a19=(a19+a26);
  a34=(a34-a19);
  if (res[0]!=0) res[0][5]=a34;
  a17=(a22*a17);
  a27=(a32*a27);
  a29=(a16*a29);
  a27=(a27+a29);
  a17=(a17-a27);
  if (res[0]!=0) res[0][6]=a17;
  if (res[0]!=0) res[0][7]=a30;
  a33=(a2*a33);
  a17=(a2*a18);
  a33=(a33+a17);
  a33=(a33-a23);
  if (res[0]!=0) res[0][8]=a33;
  a37=(a2*a37);
  a17=(a2*a13);
  a37=(a37+a17);
  a37=(a37-a31);
  if (res[0]!=0) res[0][9]=a37;
  a32=(a2*a32);
  a2=(a2*a16);
  a32=(a32+a2);
  a32=(a32-a22);
  if (res[0]!=0) res[0][10]=a32;
  if (res[0]!=0) res[0][11]=a30;
  a30=-4.5000000000000001e-01;
  a6=(a30*a6);
  a2=-3.0499999999999999e-02;
  a17=(a2*a20);
  a6=(a6+a17);
  a17=-1.0900000000000000e-01;
  a0=(a17*a0);
  a6=(a6-a0);
  a0=-2.6700000000000002e-01;
  a10=(a0*a10);
  a27=-7.4999999999999997e-02;
  a20=(a27*a20);
  a10=(a10+a20);
  a6=(a6+a10);
  a10=-1.1400000000000000e-01;
  a25=(a10*a25);
  a20=8.3000000000000004e-02;
  a12=(a20*a12);
  a25=(a25+a12);
  a6=(a6+a25);
  a25=-1.6800000000000001e-01;
  a23=(a25*a23);
  a12=6.9000000000000006e-02;
  a18=(a12*a18);
  a23=(a23+a18);
  a6=(a6+a23);
  a23=5.9999999999999998e-02;
  a33=(a23*a33);
  a6=(a6+a33);
  if (res[0]!=0) res[0][12]=a6;
  a17=(a17*a1);
  a5=(a30*a5);
  a2=(a2*a35);
  a5=(a5+a2);
  a17=(a17+a5);
  a7=(a0*a7);
  a27=(a27*a35);
  a7=(a7+a27);
  a17=(a17+a7);
  a21=(a10*a21);
  a11=(a20*a11);
  a21=(a21+a11);
  a17=(a17+a21);
  a31=(a25*a31);
  a13=(a12*a13);
  a31=(a31+a13);
  a17=(a17+a31);
  a37=(a23*a37);
  a17=(a17+a37);
  if (res[0]!=0) res[0][13]=a17;
  a17=2.9949999999999999e-01;
  a30=(a30*a36);
  a17=(a17+a30);
  a0=(a0*a38);
  a17=(a17+a0);
  a10=(a10*a24);
  a20=(a20*a3);
  a10=(a10+a20);
  a17=(a17+a10);
  a25=(a25*a22);
  a12=(a12*a16);
  a25=(a25+a12);
  a17=(a17+a25);
  a23=(a23*a32);
  a17=(a17+a23);
  if (res[0]!=0) res[0][14]=a17;
  a17=1.;
  if (res[0]!=0) res[0][15]=a17;
  return 0;
}

CASADI_SYMBOL_EXPORT int fk_T(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int fk_T_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int fk_T_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fk_T_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int fk_T_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void fk_T_release(int mem) {
}

CASADI_SYMBOL_EXPORT void fk_T_incref(void) {
}

CASADI_SYMBOL_EXPORT void fk_T_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int fk_T_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int fk_T_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real fk_T_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fk_T_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* fk_T_name_out(casadi_int i){
  switch (i) {
    case 0: return "T_indy7_tcp";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fk_T_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* fk_T_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int fk_T_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
