//
// File: InformationFilterUpdate.cu
//
// GPU Coder version                    : 1.5
// CUDA/C/C++ source code generated on  : 07-Sep-2020 12:29:29
//

// Include Files
#include "InformationFilterUpdate.h"
#include "MWCudaDimUtility.hpp"
#include <cmath>

// Function Declarations
static __global__ void InformationFilterUpdate_kernel1(double Rw[16]);
static __global__ void InformationFilterUpdate_kernel2(const double Rw[4],
  double b_Rw[16]);
static __global__ void InformationFilterUpdate_kernel3(double Rw_inv[16]);
static __global__ void InformationFilterUpdate_kernel4(signed char ipiv[4]);
static __global__ void InformationFilterUpdate_kernel5(signed char p[4]);
static __global__ void InformationFilterUpdate_kernel6(double Fk_inv[64]);
static __global__ void InformationFilterUpdate_kernel7(const double T, double x
  [64]);
static __global__ void InformationFilterUpdate_kernel8(const signed char iv[8],
  const signed char iv1[8], const signed char iv2[8], signed char ipiv[8],
  double x[64]);
static __global__ void InformationFilterUpdate_kernel9(signed char p[8]);
static __global__ void ab_InformationFilterUpdate_kern(const signed char iv3[8],
  const signed char iv4[8], const signed char iv5[8], double H_linear[104]);
static __global__ void b_InformationFilterUpdate_kerne(const double T, double
  Gk[32]);
static __global__ void bb_InformationFilterUpdate_kern(const double ih[8], const
  double H_linear[104], double hk[13]);
static __global__ void c_InformationFilterUpdate_kerne(const double a[64], const
  double Fk_inv[64], const int i, double x[64]);
static __global__ void cb_InformationFilterUpdate_kern(const double smax, const
  double delta, const double L_imuToRear, const double ih[8], double H_linear
  [104], double hk[13]);
static __global__ void d_InformationFilterUpdate_kerne(const double Fk_inv[64],
  const double x[64], const int i, double Ih[64]);
static __global__ void db_InformationFilterUpdate_kern(const double H_linear[104],
  double A[104]);
static __global__ void e_InformationFilterUpdate_kerne(double Rw[16]);
static __global__ void eb_InformationFilterUpdate_kern(const double ih[8], const
  double H_linear[104], const double hk[13], const double y_meas[13], double
  b_y_meas[13]);
static __global__ void f_InformationFilterUpdate_kerne(const double Ih[64],
  const double Gk[32], double b_Gk[32]);
static __global__ void fb_InformationFilterUpdate_kern(const double y_meas[13],
  const double C[104], const double ik[8], double op[8]);
static __global__ void g_InformationFilterUpdate_kerne(const double Gk[32],
  const double b_Gk[32], const double Rw_inv[16], double x[16]);
static __global__ void h_InformationFilterUpdate_kerne(signed char ipiv[4]);
static __global__ void i_InformationFilterUpdate_kerne(signed char p[4]);
static __global__ void j_InformationFilterUpdate_kerne(const double Gk[32],
  const double Ih[64], double b_Ih[32]);
static __global__ void k_InformationFilterUpdate_kerne(const double Rw[16],
  const double Ih[32], double b_Ih[32]);
static __global__ void l_InformationFilterUpdate_kerne(const double Gk[32],
  const double Ih[32], double calcEq[64]);
static __global__ void m_InformationFilterUpdate_kerne(double ih[8]);
static __global__ void n_InformationFilterUpdate_kerne(const double ih[8], const
  double a[64], double ik[8]);
static __global__ void o_InformationFilterUpdate_kerne(const double ik[8], const
  double Fk_inv[64], double ih[8]);
static __global__ void p_InformationFilterUpdate_kerne(const double calcEq[64],
  const double ih[8], double ik[8]);
static __global__ void q_InformationFilterUpdate_kerne(double Fk_inv[64]);
static __global__ void r_InformationFilterUpdate_kerne(const double calcEq[64],
  const double Ih[64], double x[64]);
static __global__ void s_InformationFilterUpdate_kerne(signed char ipiv[8]);
static __global__ void t_InformationFilterUpdate_kerne(signed char p[8]);
static __global__ void u_InformationFilterUpdate_kerne(const double ik[8], const
  double Fk_inv[64], double ih[8]);
static __global__ void v_InformationFilterUpdate_kerne(const double
  B_usedMeas_vec[13], const double Re[13], double Re_inv[13]);
static __global__ void w_InformationFilterUpdate_kerne(double Re_inv[169]);
static __global__ void x_InformationFilterUpdate_kerne(const double Re_inv[13],
  double b_Re_inv[169]);
static __global__ void y_InformationFilterUpdate_kerne(double H_linear[104]);

// Function Definitions

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Rw[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel1
  (double Rw[16])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 16) {
    // 4*1
    // 4*4
    // ---- Prediction step -------------------------
    Rw[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Rw[4]
//                double b_Rw[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel2(
  const double Rw[4], double b_Rw[16])
{
  unsigned int threadId;
  int j;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  j = static_cast<int>(threadId);
  if (j < 4) {
    b_Rw[j + (j << 2)] = Rw[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Rw_inv[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel3
  (double Rw_inv[16])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 16) {
    Rw_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel4
  (signed char ipiv[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel5
  (signed char p[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    p[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Fk_inv[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void InformationFilterUpdate_kernel6
  (double Fk_inv[64])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 64) {
    //  System matrix
    // 4*4
    Fk_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double T
//                double x[64]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel7(
  const double T, double x[64])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    x[52] = 0.0;
    x[60] = 0.0;
    x[6] = 0.0;
    x[14] = 0.0;
    x[22] = 0.0;
    x[30] = T;
    x[38] = 0.0;
    x[46] = 0.0;
    x[54] = 1.0;
    x[62] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv[8]
//                const signed char iv1[8]
//                const signed char iv2[8]
//                signed char ipiv[8]
//                double x[64]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel8(
  const signed char iv[8], const signed char iv1[8], const signed char iv2[8],
  signed char ipiv[8], double x[64])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    x[(i << 3) + 2] = static_cast<double>(iv2[i]);
    x[(i << 3) + 5] = static_cast<double>(iv1[i]);
    x[(i << 3) + 7] = static_cast<double>(iv[i]);
    ipiv[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel9
  (signed char p[8])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    p[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv3[8]
//                const signed char iv4[8]
//                const signed char iv5[8]
//                double H_linear[104]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void ab_InformationFilterUpdate_kern(
  const signed char iv3[8], const signed char iv4[8], const signed char iv5[8],
  double H_linear[104])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    H_linear[13 * i] = static_cast<double>(iv5[i]);
    H_linear[13 * i + 2] = static_cast<double>(iv4[i]);
    H_linear[13 * i + 3] = static_cast<double>(iv3[i]);
    H_linear[13 * i + 4] = 0.0;
    H_linear[13 * i + 5] = 0.0;
    H_linear[13 * i + 10] = 0.0;
    H_linear[13 * i + 11] = static_cast<double>(iv3[i]);
    H_linear[13 * i + 12] = static_cast<double>(iv3[i]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double T
//                double Gk[32]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void b_InformationFilterUpdate_kerne(
  const double T, double Gk[32])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Gk[21] = T;
    Gk[29] = 0.0;
    Gk[6] = 0.0;
    Gk[14] = T * T / 2.0;
    Gk[22] = 0.0;
    Gk[30] = T * T / 2.0;
    Gk[7] = 0.0;
    Gk[15] = T;
    Gk[23] = 0.0;
    Gk[31] = T;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ih[8]
//                const double H_linear[104]
//                double hk[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void bb_InformationFilterUpdate_kern(
  const double ih[8], const double H_linear[104], double hk[13])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 13) {
    //  Nonlinear parts
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += H_linear[i + 13 * i4] * ih[i4];
    }

    hk[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double a[64]
//                const double Fk_inv[64]
//                const int i
//                double x[64]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void c_InformationFilterUpdate_kerne(
  const double a[64], const double Fk_inv[64], const int i, double x[64])
{
  unsigned int threadId;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId);
  if (i5 < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += Fk_inv[i4 + (i << 3)] * a[i4 + (i5 << 3)];
    }

    x[i + (i5 << 3)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double smax
//                const double delta
//                const double L_imuToRear
//                const double ih[8]
//                double H_linear[104]
//                double hk[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void cb_InformationFilterUpdate_kern(
  const double smax, const double delta, const double L_imuToRear, const double
  ih[8], double H_linear[104], double hk[13])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    hk[0] -= ih[2] * ih[2] * L_imuToRear;
    hk[1] += ih[0] * ih[2];
    hk[10] = ih[0] * cos(delta) + smax * ih[2] * sin(delta);

    // updates the Hk 13*4matrix HK(5,1)=Hk matrix's 5th row 1st element
    // being updated with cos(delta)value . Likewise for all.
    H_linear[26] = -2.0 * ih[2] * L_imuToRear;
    H_linear[1] = ih[2];
    H_linear[27] = ih[0];
    H_linear[4] = cos(delta);
    H_linear[10] = cos(delta);
    H_linear[36] = smax * sin(delta);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Fk_inv[64]
//                const double x[64]
//                const int i
//                double Ih[64]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void d_InformationFilterUpdate_kerne(
  const double Fk_inv[64], const double x[64], const int i, double Ih[64])
{
  unsigned int threadId;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId);
  if (i5 < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += x[i + (i4 << 3)] * Fk_inv[i4 + (i5 << 3)];
    }

    Ih[i + (i5 << 3)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double H_linear[104]
//                double A[104]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void db_InformationFilterUpdate_kern
  (const double H_linear[104], double A[104])
{
  unsigned int threadId;
  int i;
  int i4;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i4 = static_cast<int>(threadId % 8U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i4)) / 8U);
  if (i < 13) {
    A[i4 + (i << 3)] = H_linear[i + 13 * i4];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Rw[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void e_InformationFilterUpdate_kerne
  (double Rw[16])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 16) {
    // 4*4
    Rw[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ih[8]
//                const double H_linear[104]
//                const double hk[13]
//                const double y_meas[13]
//                double b_y_meas[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void eb_InformationFilterUpdate_kern(
  const double ih[8], const double H_linear[104], const double hk[13], const
  double y_meas[13], double b_y_meas[13])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 13) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += H_linear[i + 13 * i4] * ih[i4];
    }

    b_y_meas[i] = (y_meas[i] - hk[i]) + d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Ih[64]
//                const double Gk[32]
//                double b_Gk[32]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void f_InformationFilterUpdate_kerne(
  const double Ih[64], const double Gk[32], double b_Gk[32])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 8U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 8U);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += Gk[i4 + (i << 3)] * Ih[i4 + (i5 << 3)];
    }

    b_Gk[i + (i5 << 2)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y_meas[13]
//                const double C[104]
//                const double ik[8]
//                double op[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void fb_InformationFilterUpdate_kern(
  const double y_meas[13], const double C[104], const double ik[8], double op[8])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 13; i4++) {
      d += C[i + (i4 << 3)] * y_meas[i4];
    }

    op[i] = ik[i] + d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[32]
//                const double b_Gk[32]
//                const double Rw_inv[16]
//                double x[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void g_InformationFilterUpdate_kerne(
  const double Gk[32], const double b_Gk[32], const double Rw_inv[16], double x
  [16])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 4U);
  if (i < 4) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += b_Gk[i + (i4 << 2)] * Gk[i4 + (i5 << 3)];
    }

    x[i + (i5 << 2)] = d + Rw_inv[i + (i5 << 2)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void h_InformationFilterUpdate_kerne
  (signed char ipiv[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[4]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void i_InformationFilterUpdate_kerne
  (signed char p[4])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 4) {
    p[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[32]
//                const double Ih[64]
//                double b_Ih[32]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void j_InformationFilterUpdate_kerne(
  const double Gk[32], const double Ih[64], double b_Ih[32])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 4U);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += Ih[i + (i4 << 3)] * Gk[i4 + (i5 << 3)];
    }

    b_Ih[i + (i5 << 3)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Rw[16]
//                const double Ih[32]
//                double b_Ih[32]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void k_InformationFilterUpdate_kerne(
  const double Rw[16], const double Ih[32], double b_Ih[32])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 4U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 4U);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Ih[i + (i4 << 3)] * Rw[i4 + (i5 << 2)];
    }

    b_Ih[i + (i5 << 3)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[32]
//                const double Ih[32]
//                double calcEq[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void l_InformationFilterUpdate_kerne(
  const double Gk[32], const double Ih[32], double calcEq[64])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 8U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 8U);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 4; i4++) {
      d += Ih[i + (i4 << 3)] * Gk[i5 + (i4 << 3)];
    }

    calcEq[i + (i5 << 3)] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double ih[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void m_InformationFilterUpdate_kerne
  (double ih[8])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    ih[5] = 0.0;
    ih[6] = 0.0;
    ih[7] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ih[8]
//                const double a[64]
//                double ik[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void n_InformationFilterUpdate_kerne(
  const double ih[8], const double a[64], double ik[8])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += a[i + (i4 << 3)] * ih[i4];
    }

    ik[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ik[8]
//                const double Fk_inv[64]
//                double ih[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void o_InformationFilterUpdate_kerne(
  const double ik[8], const double Fk_inv[64], double ih[8])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += Fk_inv[i4 + (i << 3)] * ik[i4];
    }

    ih[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double calcEq[64]
//                const double ih[8]
//                double ik[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void p_InformationFilterUpdate_kerne(
  const double calcEq[64], const double ih[8], double ik[8])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    // 4*1
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += calcEq[i + (i4 << 3)] * ih[i4];
    }

    ik[i] = ih[i] - d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Fk_inv[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void q_InformationFilterUpdate_kerne
  (double Fk_inv[64])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 64) {
    // 4*1
    Fk_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double calcEq[64]
//                const double Ih[64]
//                double x[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void r_InformationFilterUpdate_kerne(
  const double calcEq[64], const double Ih[64], double x[64])
{
  unsigned int threadId;
  int i;
  double d;
  int i5;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i5 = static_cast<int>(threadId % 8U);
  i = static_cast<int>((threadId - static_cast<unsigned int>(i5)) / 8U);
  if (i < 8) {
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += calcEq[i + (i4 << 3)] * Ih[i4 + (i5 << 3)];
    }

    x[i + (i5 << 3)] = Ih[i + (i5 << 3)] - d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void s_InformationFilterUpdate_kerne
  (signed char ipiv[8])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void t_InformationFilterUpdate_kerne
  (signed char p[8])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    p[i] = static_cast<signed char>(i + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ik[8]
//                const double Fk_inv[64]
//                double ih[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void u_InformationFilterUpdate_kerne(
  const double ik[8], const double Fk_inv[64], double ih[8])
{
  unsigned int threadId;
  double d;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 8) {
    // 4*4
    d = 0.0;
    for (int i4 = 0; i4 < 8; i4++) {
      d += Fk_inv[i + (i4 << 3)] * ik[i4];
    }

    ih[i] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double B_usedMeas_vec[13]
//                const double Re[13]
//                double Re_inv[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void v_InformationFilterUpdate_kerne(
  const double B_usedMeas_vec[13], const double Re[13], double Re_inv[13])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 13) {
    // 1*1
    Re_inv[i] = 1.0 / Re[i] * B_usedMeas_vec[i];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Re_inv[169]
// Return Type  : void
//
static __global__ __launch_bounds__(192, 1) void w_InformationFilterUpdate_kerne
  (double Re_inv[169])
{
  unsigned int threadId;
  int i;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i = static_cast<int>(threadId);
  if (i < 169) {
    // 1*13
    Re_inv[i] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Re_inv[13]
//                double b_Re_inv[169]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void x_InformationFilterUpdate_kerne(
  const double Re_inv[13], double b_Re_inv[169])
{
  unsigned int threadId;
  int j;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  j = static_cast<int>(threadId);
  if (j < 13) {
    b_Re_inv[j + 13 * j] = Re_inv[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double H_linear[104]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void y_InformationFilterUpdate_kerne
  (double H_linear[104])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    H_linear[100] = 0.0;
  }
}

//
// Arguments    : const double y_meas[13]
//                const double B_usedMeas_vec[13]
//                const double initialization_vec[4]
//                double delta
//                const double Rw[4]
//                const double Re[13]
//                double L_imuToRear
//                double L_geometricWheelbase
//                const double L_trackWidth[5]
//                const double L_axlePos[5]
//                double T
//                double op[8]
// Return Type  : void
//
void InformationFilterUpdate(const double y_meas[13], const double
  B_usedMeas_vec[13], const double initialization_vec[4], double delta, const
  double Rw[4], const double Re[13], double L_imuToRear, double
  L_geometricWheelbase, const double L_trackWidth[5], const double L_axlePos[5],
  double T, double op[8])
{
  int j;
  int c;
  int ar;
  int iy;
  int ia;
  int ix;
  double smax;
  signed char i1;
  double s;
  int i2;
  int b_j;
  int jy;
  int i;
  int kAcol;
  int i3;
  static const signed char iv[8] = { 0, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char iv1[8] = { 0, 0, 0, 0, 0, 1, 0, 0 };

  static const signed char iv2[8] = { 0, 0, 1, 0, 0, 0, 0, 0 };

  static const double a[64] = { 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    33.333333333333336, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10000.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 500.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 33.333333333333336, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 500.0 };

  static const signed char iv3[8] = { 1, 0, 0, 0, 1, 0, 0, 0 };

  static const signed char iv4[8] = { 0, 0, 1, 0, 0, 0, 1, 0 };

  static const signed char iv5[8] = { 0, 1, 0, 0, 0, 1, 0, 0 };

  double (*gpu_Rw)[16];
  double (*b_gpu_Rw)[4];
  double (*gpu_Rw_inv)[16];
  signed char (*gpu_ipiv)[4];
  signed char (*gpu_p)[4];
  double (*gpu_Fk_inv)[64];
  double (*gpu_x)[64];
  signed char (*gpu_iv)[8];
  signed char (*gpu_iv1)[8];
  signed char (*gpu_iv2)[8];
  signed char (*b_gpu_ipiv)[8];
  signed char (*b_gpu_p)[8];
  double (*gpu_Gk)[32];
  double (*gpu_a)[64];
  double (*gpu_Ih)[64];
  double (*b_gpu_Gk)[32];
  double (*b_gpu_x)[16];
  double (*b_gpu_Ih)[32];
  double (*c_gpu_Ih)[32];
  double (*gpu_calcEq)[64];
  double (*gpu_ih)[8];
  double (*gpu_ik)[8];
  double (*gpu_B_usedMeas_vec)[13];
  double (*gpu_Re)[13];
  double (*gpu_Re_inv)[13];
  double (*b_gpu_Re_inv)[169];
  double (*gpu_H_linear)[104];
  signed char (*gpu_iv3)[8];
  signed char (*gpu_iv4)[8];
  signed char (*gpu_iv5)[8];
  double (*gpu_hk)[13];
  double (*gpu_A)[104];
  double (*gpu_y_meas)[13];
  double (*b_gpu_y_meas)[13];
  double (*gpu_C)[104];
  double (*gpu_op)[8];
  bool syncIsDirty;
  cudaMallocManaged(&gpu_C, 832UL);
  cudaMallocManaged(&b_gpu_y_meas, 104UL);
  cudaMallocManaged(&gpu_A, 832UL);
  cudaMallocManaged(&gpu_hk, 104UL);
  cudaMallocManaged(&gpu_H_linear, 832UL);
  cudaMallocManaged(&b_gpu_Re_inv, 1352UL);
  cudaMallocManaged(&gpu_Re_inv, 104UL);
  cudaMallocManaged(&gpu_ik, 64UL);
  cudaMallocManaged(&gpu_ih, 64UL);
  cudaMallocManaged(&gpu_calcEq, 512UL);
  cudaMallocManaged(&c_gpu_Ih, 256UL);
  cudaMallocManaged(&b_gpu_Ih, 256UL);
  cudaMallocManaged(&b_gpu_x, 128UL);
  cudaMallocManaged(&b_gpu_Gk, 256UL);
  cudaMallocManaged(&gpu_Ih, 512UL);
  cudaMallocManaged(&gpu_Gk, 256UL);
  cudaMallocManaged(&b_gpu_p, 8UL);
  cudaMallocManaged(&b_gpu_ipiv, 8UL);
  cudaMallocManaged(&gpu_x, 512UL);
  cudaMallocManaged(&gpu_Fk_inv, 512UL);
  cudaMallocManaged(&gpu_p, 4UL);
  cudaMallocManaged(&gpu_ipiv, 4UL);
  cudaMallocManaged(&gpu_Rw_inv, 128UL);
  cudaMallocManaged(&gpu_Rw, 128UL);
  cudaMallocManaged(&gpu_op, 64UL);
  cudaMallocManaged(&b_gpu_Rw, 32UL);
  cudaMallocManaged(&gpu_iv, 8UL);
  cudaMallocManaged(&gpu_iv1, 8UL);
  cudaMallocManaged(&gpu_iv2, 8UL);
  cudaMallocManaged(&gpu_a, 512UL);
  cudaMallocManaged(&gpu_B_usedMeas_vec, 104UL);
  cudaMallocManaged(&gpu_Re, 104UL);
  cudaMallocManaged(&gpu_iv3, 8UL);
  cudaMallocManaged(&gpu_iv4, 8UL);
  cudaMallocManaged(&gpu_iv5, 8UL);
  cudaMallocManaged(&gpu_y_meas, 104UL);
  cudaMemcpy(gpu_y_meas, (void *)&y_meas[0], 104UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv5, (void *)&iv5[0], 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv4, (void *)&iv4[0], 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv3, (void *)&iv3[0], 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_Re, (void *)&Re[0], 104UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_B_usedMeas_vec, (void *)&B_usedMeas_vec[0], 104UL,
             cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_a, (void *)&a[0], 512UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv2, (void *)&iv2[0], 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv1, (void *)&iv1[0], 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv, (void *)&iv[0], 8UL, cudaMemcpyHostToDevice);
  cudaMemcpy(b_gpu_Rw, (void *)&Rw[0], 32UL, cudaMemcpyHostToDevice);

  // 4*1
  // 4*4
  // ---- Prediction step -------------------------
  InformationFilterUpdate_kernel1<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Rw);
  InformationFilterUpdate_kernel2<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_Rw, *gpu_Rw);
  InformationFilterUpdate_kernel3<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Rw_inv);
  InformationFilterUpdate_kernel4<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 3; j++) {
    c = j * 5;
    ar = 2 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_Rw)[c]);
    for (ia = 0; ia <= ar; ia++) {
      ix++;
      s = std::abs((*gpu_Rw)[ix]);
      if (s > smax) {
        iy = ia + 1;
        smax = s;
      }
    }

    if ((*gpu_Rw)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        ar = iy;
        for (ia = 0; ia < 4; ia++) {
          ix = j + ia * 4;
          iy = ar + ia * 4;
          smax = (*gpu_Rw)[ix];
          (*gpu_Rw)[ix] = (*gpu_Rw)[iy];
          (*gpu_Rw)[iy] = smax;
        }
      }

      i2 = (c - j) + 2;
      for (i = 0; i <= i2 - c; i++) {
        iy = (c + i) + 1;
        (*gpu_Rw)[iy] /= (*gpu_Rw)[c];
      }
    }

    ar = 2 - j;
    iy = c + 6;
    jy = c + 4;
    for (b_j = 0; b_j <= ar; b_j++) {
      smax = (*gpu_Rw)[jy];
      if ((*gpu_Rw)[jy] != 0.0) {
        ix = c;
        i2 = iy - 2;
        i3 = iy - j;
        for (kAcol = 0; kAcol <= i3 - i2; kAcol++) {
          i = (iy + kAcol) - 1;
          (*gpu_Rw)[i] += (*gpu_Rw)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  InformationFilterUpdate_kernel5<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_p);
  syncIsDirty = true;
  for (ia = 0; ia < 3; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*gpu_ipiv)[ia] > ia + 1) {
      iy = (*gpu_p)[(*gpu_ipiv)[ia] - 1];
      (*gpu_p)[(*gpu_ipiv)[ia] - 1] = (*gpu_p)[ia];
      (*gpu_p)[ia] = static_cast<signed char>(iy);
    }
  }

  for (ia = 0; ia < 4; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    i1 = (*gpu_p)[ia];
    (*gpu_Rw_inv)[ia + (((*gpu_p)[ia] - 1) << 2)] = 1.0;
    for (j = 0; j <= 3 - ia; j++) {
      b_j = ia + j;
      if ((*gpu_Rw_inv)[b_j + ((i1 - 1) << 2)] != 0.0) {
        for (i = 0; i <= 2 - b_j; i++) {
          iy = (b_j + i) + 1;
          (*gpu_Rw_inv)[iy + ((i1 - 1) << 2)] -= (*gpu_Rw_inv)[b_j + ((i1 - 1) <<
            2)] * (*gpu_Rw)[iy + (b_j << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    iy = (j << 2) - 1;
    for (ia = 0; ia < 4; ia++) {
      jy = 4 - ia;
      kAcol = (3 - ia) << 2;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Rw_inv)[(iy - ia) + 4] != 0.0) {
        (*gpu_Rw_inv)[(iy - ia) + 4] /= (*gpu_Rw)[(kAcol - ia) + 3];
        for (i = 0; i <= jy - 2; i++) {
          (*gpu_Rw_inv)[(i + iy) + 1] -= (*gpu_Rw_inv)[(iy - ia) + 4] * (*gpu_Rw)
            [i + kAcol];
        }
      }
    }
  }

  //  System matrix
  // 4*4
  InformationFilterUpdate_kernel6<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_Fk_inv);
  cudaDeviceSynchronize();
  (*gpu_x)[0] = 1.0;
  (*gpu_x)[8] = T;
  (*gpu_x)[16] = 0.0;
  (*gpu_x)[24] = 0.0;
  (*gpu_x)[32] = 0.0;
  (*gpu_x)[40] = 0.0;
  (*gpu_x)[48] = 0.0;
  (*gpu_x)[56] = 0.0;
  (*gpu_x)[1] = 0.0;
  (*gpu_x)[9] = 1.0;
  (*gpu_x)[17] = 0.0;
  (*gpu_x)[25] = 0.0;
  (*gpu_x)[33] = 0.0;
  (*gpu_x)[41] = 1.0;
  (*gpu_x)[49] = 0.0;
  (*gpu_x)[57] = T;
  (*gpu_x)[3] = 0.0;
  (*gpu_x)[11] = 0.0;
  (*gpu_x)[19] = 0.0;
  (*gpu_x)[27] = 1.0;
  (*gpu_x)[35] = 0.0;
  (*gpu_x)[43] = 0.0;
  (*gpu_x)[51] = 0.0;
  (*gpu_x)[59] = T;
  (*gpu_x)[4] = 0.0;
  (*gpu_x)[12] = T;
  (*gpu_x)[20] = 0.0;
  (*gpu_x)[28] = 0.0;
  (*gpu_x)[36] = 1.0;
  (*gpu_x)[44] = 0.0;
  InformationFilterUpdate_kernel7<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(T,
    *gpu_x);
  InformationFilterUpdate_kernel8<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_iv, *gpu_iv1, *gpu_iv2, *b_gpu_ipiv, *gpu_x);
  syncIsDirty = true;
  for (j = 0; j < 7; j++) {
    c = j * 9;
    kAcol = c - 4;
    ar = 6 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_x)[c]);
    for (ia = 0; ia <= ar; ia++) {
      ix++;
      s = std::abs((*gpu_x)[ix]);
      if (s > smax) {
        iy = ia + 1;
        smax = s;
      }
    }

    if ((*gpu_x)[c + iy] != 0.0) {
      if (iy != 0) {
        (*b_gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        ar = iy;
        for (ia = 0; ia < 8; ia++) {
          ix = j + ia * 8;
          iy = ar + ia * 8;
          smax = (*gpu_x)[ix];
          (*gpu_x)[ix] = (*gpu_x)[iy];
          (*gpu_x)[iy] = smax;
        }
      }

      i2 = (c - j) + 2;
      for (i = 0; i <= i2 - kAcol; i++) {
        iy = (c + i) + 1;
        (*gpu_x)[iy] /= (*gpu_x)[c];
      }
    }

    ar = 6 - j;
    iy = c + 10;
    jy = c + 8;
    for (b_j = 0; b_j <= ar; b_j++) {
      smax = (*gpu_x)[jy];
      if ((*gpu_x)[jy] != 0.0) {
        ix = c;
        i2 = iy - 6;
        i3 = iy - j;
        for (kAcol = 0; kAcol <= i3 - i2; kAcol++) {
          i = (iy + kAcol) - 1;
          (*gpu_x)[i] += (*gpu_x)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 8;
      iy += 8;
    }
  }

  InformationFilterUpdate_kernel9<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_p);
  syncIsDirty = true;
  for (ia = 0; ia < 7; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*b_gpu_ipiv)[ia] > ia + 1) {
      iy = (*b_gpu_p)[(*b_gpu_ipiv)[ia] - 1];
      (*b_gpu_p)[(*b_gpu_ipiv)[ia] - 1] = (*b_gpu_p)[ia];
      (*b_gpu_p)[ia] = static_cast<signed char>(iy);
    }
  }

  for (ia = 0; ia < 8; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    i1 = (*b_gpu_p)[ia];
    (*gpu_Fk_inv)[ia + (((*b_gpu_p)[ia] - 1) << 3)] = 1.0;
    for (j = 0; j <= 7 - ia; j++) {
      b_j = ia + j;
      if ((*gpu_Fk_inv)[b_j + ((i1 - 1) << 3)] != 0.0) {
        for (i = 0; i <= 6 - b_j; i++) {
          iy = (b_j + i) + 1;
          (*gpu_Fk_inv)[iy + ((i1 - 1) << 3)] -= (*gpu_Fk_inv)[b_j + ((i1 - 1) <<
            3)] * (*gpu_x)[iy + (b_j << 3)];
        }
      }
    }
  }

  for (j = 0; j < 8; j++) {
    iy = (j << 3) - 1;
    for (ia = 0; ia < 8; ia++) {
      jy = 8 - ia;
      kAcol = (7 - ia) << 3;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Fk_inv)[(iy - ia) + 8] != 0.0) {
        (*gpu_Fk_inv)[(iy - ia) + 8] /= (*gpu_x)[(kAcol - ia) + 7];
        for (i = 0; i <= jy - 2; i++) {
          (*gpu_Fk_inv)[(i + iy) + 1] -= (*gpu_Fk_inv)[(iy - ia) + 8] * (*gpu_x)
            [i + kAcol];
        }
      }
    }
  }

  //  Noise matrix
  if (syncIsDirty) {
    cudaDeviceSynchronize();
  }

  (*gpu_Gk)[0] = T * T / 2.0;
  (*gpu_Gk)[8] = 0.0;
  (*gpu_Gk)[16] = T * T / 2.0;
  (*gpu_Gk)[24] = 0.0;
  (*gpu_Gk)[1] = T;
  (*gpu_Gk)[9] = 0.0;
  (*gpu_Gk)[17] = T;
  (*gpu_Gk)[25] = 0.0;
  (*gpu_Gk)[2] = 0.0;
  (*gpu_Gk)[10] = T * T / 2.0;
  (*gpu_Gk)[18] = 0.0;
  (*gpu_Gk)[26] = T * T / 2.0;
  (*gpu_Gk)[3] = 0.0;
  (*gpu_Gk)[11] = T;
  (*gpu_Gk)[19] = 0.0;
  (*gpu_Gk)[27] = T;
  (*gpu_Gk)[4] = T * T / 2.0;
  (*gpu_Gk)[12] = 0.0;
  (*gpu_Gk)[20] = T * T / 2.0;
  (*gpu_Gk)[28] = 0.0;
  (*gpu_Gk)[5] = T;
  (*gpu_Gk)[13] = 0.0;
  b_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(T,
    *gpu_Gk);

  //  Prediction step alternative 2. Gives easier matrix to invert
  for (i = 0; i < 8; i++) {
    c_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*gpu_a, *gpu_Fk_inv, i, *gpu_x);
    d_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*gpu_Fk_inv, *gpu_x, i, *gpu_Ih);
  }

  // 4*4
  e_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Rw);
  f_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Ih, *gpu_Gk, *b_gpu_Gk);
  g_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Gk, *b_gpu_Gk, *gpu_Rw_inv, *b_gpu_x);
  h_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 3; j++) {
    c = j * 5;
    ar = 2 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*b_gpu_x)[c]);
    for (ia = 0; ia <= ar; ia++) {
      ix++;
      s = std::abs((*b_gpu_x)[ix]);
      if (s > smax) {
        iy = ia + 1;
        smax = s;
      }
    }

    if ((*b_gpu_x)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        ar = iy;
        for (ia = 0; ia < 4; ia++) {
          ix = j + ia * 4;
          iy = ar + ia * 4;
          smax = (*b_gpu_x)[ix];
          (*b_gpu_x)[ix] = (*b_gpu_x)[iy];
          (*b_gpu_x)[iy] = smax;
        }
      }

      i2 = (c - j) + 2;
      for (i = 0; i <= i2 - c; i++) {
        iy = (c + i) + 1;
        (*b_gpu_x)[iy] /= (*b_gpu_x)[c];
      }
    }

    ar = 2 - j;
    iy = c + 6;
    jy = c + 4;
    for (b_j = 0; b_j <= ar; b_j++) {
      smax = (*b_gpu_x)[jy];
      if ((*b_gpu_x)[jy] != 0.0) {
        ix = c;
        i2 = iy - 2;
        i3 = iy - j;
        for (kAcol = 0; kAcol <= i3 - i2; kAcol++) {
          i = (iy + kAcol) - 1;
          (*b_gpu_x)[i] += (*b_gpu_x)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  i_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_p);
  syncIsDirty = true;
  for (ia = 0; ia < 3; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*gpu_ipiv)[ia] > ia + 1) {
      iy = (*gpu_p)[(*gpu_ipiv)[ia] - 1];
      (*gpu_p)[(*gpu_ipiv)[ia] - 1] = (*gpu_p)[ia];
      (*gpu_p)[ia] = static_cast<signed char>(iy);
    }
  }

  for (ia = 0; ia < 4; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    i1 = (*gpu_p)[ia];
    (*gpu_Rw)[ia + (((*gpu_p)[ia] - 1) << 2)] = 1.0;
    for (j = 0; j <= 3 - ia; j++) {
      b_j = ia + j;
      if ((*gpu_Rw)[b_j + ((i1 - 1) << 2)] != 0.0) {
        for (i = 0; i <= 2 - b_j; i++) {
          iy = (b_j + i) + 1;
          (*gpu_Rw)[iy + ((i1 - 1) << 2)] -= (*gpu_Rw)[b_j + ((i1 - 1) << 2)] *
            (*b_gpu_x)[iy + (b_j << 2)];
        }
      }
    }
  }

  for (j = 0; j < 4; j++) {
    iy = (j << 2) - 1;
    for (ia = 0; ia < 4; ia++) {
      jy = 4 - ia;
      kAcol = (3 - ia) << 2;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Rw)[(iy - ia) + 4] != 0.0) {
        (*gpu_Rw)[(iy - ia) + 4] /= (*b_gpu_x)[(kAcol - ia) + 3];
        for (i = 0; i <= jy - 2; i++) {
          (*gpu_Rw)[(i + iy) + 1] -= (*gpu_Rw)[(iy - ia) + 4] * (*b_gpu_x)[i +
            kAcol];
        }
      }
    }
  }

  j_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Gk, *gpu_Ih, *b_gpu_Ih);
  k_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Rw, *b_gpu_Ih, *c_gpu_Ih);
  l_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_Gk, *c_gpu_Ih, *gpu_calcEq);

  // 4*4
  // 4*4
  cudaDeviceSynchronize();
  (*gpu_ih)[0] = initialization_vec[2];
  (*gpu_ih)[1] = 0.0;
  (*gpu_ih)[2] = 0.0;
  (*gpu_ih)[3] = 0.0;
  (*gpu_ih)[4] = initialization_vec[2];
  m_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih);
  n_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih, *gpu_a, *gpu_ik);
  o_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ik, *gpu_Fk_inv, *gpu_ih);

  // 4*1
  p_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_calcEq, *gpu_ih, *gpu_ik);

  // 4*1
  q_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_Fk_inv);
  r_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_calcEq, *gpu_Ih, *gpu_x);
  s_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 7; j++) {
    c = j * 9;
    kAcol = c - 4;
    ar = 6 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_x)[c]);
    for (ia = 0; ia <= ar; ia++) {
      ix++;
      s = std::abs((*gpu_x)[ix]);
      if (s > smax) {
        iy = ia + 1;
        smax = s;
      }
    }

    if ((*gpu_x)[c + iy] != 0.0) {
      if (iy != 0) {
        (*b_gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        ar = iy;
        for (ia = 0; ia < 8; ia++) {
          ix = j + ia * 8;
          iy = ar + ia * 8;
          smax = (*gpu_x)[ix];
          (*gpu_x)[ix] = (*gpu_x)[iy];
          (*gpu_x)[iy] = smax;
        }
      }

      i2 = (c - j) + 2;
      for (i = 0; i <= i2 - kAcol; i++) {
        iy = (c + i) + 1;
        (*gpu_x)[iy] /= (*gpu_x)[c];
      }
    }

    ar = 6 - j;
    iy = c + 10;
    jy = c + 8;
    for (b_j = 0; b_j <= ar; b_j++) {
      smax = (*gpu_x)[jy];
      if ((*gpu_x)[jy] != 0.0) {
        ix = c;
        i2 = iy - 6;
        i3 = iy - j;
        for (kAcol = 0; kAcol <= i3 - i2; kAcol++) {
          i = (iy + kAcol) - 1;
          (*gpu_x)[i] += (*gpu_x)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 8;
      iy += 8;
    }
  }

  t_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_p);
  syncIsDirty = true;
  for (ia = 0; ia < 7; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*b_gpu_ipiv)[ia] > ia + 1) {
      iy = (*b_gpu_p)[(*b_gpu_ipiv)[ia] - 1];
      (*b_gpu_p)[(*b_gpu_ipiv)[ia] - 1] = (*b_gpu_p)[ia];
      (*b_gpu_p)[ia] = static_cast<signed char>(iy);
    }
  }

  for (ia = 0; ia < 8; ia++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    i1 = (*b_gpu_p)[ia];
    (*gpu_Fk_inv)[ia + (((*b_gpu_p)[ia] - 1) << 3)] = 1.0;
    for (j = 0; j <= 7 - ia; j++) {
      b_j = ia + j;
      if ((*gpu_Fk_inv)[b_j + ((i1 - 1) << 3)] != 0.0) {
        for (i = 0; i <= 6 - b_j; i++) {
          iy = (b_j + i) + 1;
          (*gpu_Fk_inv)[iy + ((i1 - 1) << 3)] -= (*gpu_Fk_inv)[b_j + ((i1 - 1) <<
            3)] * (*gpu_x)[iy + (b_j << 3)];
        }
      }
    }
  }

  for (j = 0; j < 8; j++) {
    iy = (j << 3) - 1;
    for (ia = 0; ia < 8; ia++) {
      jy = 8 - ia;
      kAcol = (7 - ia) << 3;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Fk_inv)[(iy - ia) + 8] != 0.0) {
        (*gpu_Fk_inv)[(iy - ia) + 8] /= (*gpu_x)[(kAcol - ia) + 7];
        for (i = 0; i <= jy - 2; i++) {
          (*gpu_Fk_inv)[(i + iy) + 1] -= (*gpu_Fk_inv)[(iy - ia) + 8] * (*gpu_x)
            [i + kAcol];
        }
      }
    }
  }

  // 4*4
  u_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ik, *gpu_Fk_inv, *gpu_ih);

  // 4*1
  // ---- Measurement step --------------------------
  //  Extract front and rear track widths
  //  drive axle 1
  //  drive axle 2
  smax = L_axlePos[0] + L_geometricWheelbase;

  // 1*1
  v_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_B_usedMeas_vec, *gpu_Re, *gpu_Re_inv);

  // 1*13
  w_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(192U, 1U, 1U)>>>
    (*b_gpu_Re_inv);
  x_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Re_inv, *b_gpu_Re_inv);

  // 13*13
  //  Linear parts
  cudaDeviceSynchronize();
  (*gpu_H_linear)[1] = 0.0;
  (*gpu_H_linear)[14] = 0.0;
  (*gpu_H_linear)[27] = 0.0;
  (*gpu_H_linear)[40] = L_imuToRear;
  (*gpu_H_linear)[53] = 0.0;
  (*gpu_H_linear)[66] = 0.0;
  (*gpu_H_linear)[79] = 0.0;
  (*gpu_H_linear)[92] = L_imuToRear;
  (*gpu_H_linear)[6] = 1.0;
  (*gpu_H_linear)[19] = 0.0;
  (*gpu_H_linear)[32] = -L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[45] = 0.0;
  (*gpu_H_linear)[58] = 1.0;
  (*gpu_H_linear)[71] = 0.0;
  (*gpu_H_linear)[84] = -L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[97] = 0.0;
  (*gpu_H_linear)[7] = 1.0;
  (*gpu_H_linear)[20] = 0.0;
  (*gpu_H_linear)[33] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[46] = 0.0;
  (*gpu_H_linear)[59] = 1.0;
  (*gpu_H_linear)[72] = 0.0;
  (*gpu_H_linear)[85] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[98] = 0.0;
  (*gpu_H_linear)[8] = 1.0;
  (*gpu_H_linear)[21] = 0.0;
  (*gpu_H_linear)[34] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[47] = 0.0;
  (*gpu_H_linear)[60] = 1.0;
  (*gpu_H_linear)[73] = 0.0;
  (*gpu_H_linear)[86] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[99] = 0.0;
  (*gpu_H_linear)[9] = 1.0;
  (*gpu_H_linear)[22] = 0.0;
  (*gpu_H_linear)[35] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[48] = 0.0;
  (*gpu_H_linear)[61] = 1.0;
  (*gpu_H_linear)[74] = 0.0;
  (*gpu_H_linear)[87] = L_trackWidth[3] / 2.0;
  y_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_H_linear);
  ab_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_iv3, *gpu_iv4, *gpu_iv5, *gpu_H_linear);

  //  Nonlinear parts
  bb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih, *gpu_H_linear, *gpu_hk);

  // 13*1
  // the new hk 13*1 matrix is formed from this calculation
  cudaDeviceSynchronize();
  (*gpu_hk)[4] = ((*gpu_ih)[0] - L_trackWidth[0] * (*gpu_ih)[2] / 2.0) * std::
    cos(delta) + smax * (*gpu_ih)[2] * std::sin(delta);
  (*gpu_hk)[5] = ((*gpu_ih)[0] + L_trackWidth[0] * (*gpu_ih)[2] / 2.0) * std::
    cos(delta) + smax * (*gpu_ih)[2] * std::sin(delta);
  (*gpu_H_linear)[30] = -L_trackWidth[0] * std::cos(delta) / 2.0 + smax * std::
    sin(delta);
  (*gpu_H_linear)[5] = std::cos(delta);
  (*gpu_H_linear)[31] = L_trackWidth[0] * std::cos(delta) / 2.0 + smax * std::
    sin(delta);
  cb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(smax,
    delta, L_imuToRear, *gpu_ih, *gpu_H_linear, *gpu_hk);
  db_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*gpu_H_linear, *gpu_A);
  syncIsDirty = true;
  for (iy = 0; iy < 13; iy++) {
    kAcol = iy << 3;
    i2 = kAcol - 7;
    for (i = 0; i <= kAcol - i2; i++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_C)[kAcol + i] = 0.0;
    }
  }

  for (iy = 0; iy < 13; iy++) {
    jy = iy * 13 + 1;
    kAcol = iy << 3;
    ar = -1;
    i2 = jy - 12;
    for (b_j = 0; b_j <= jy - i2; b_j++) {
      ix = jy + b_j;
      ia = ar;
      i3 = kAcol - 6;
      c = kAcol + 1;
      for (i = 0; i <= c - i3; i++) {
        j = kAcol + i;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_C)[j] += (*b_gpu_Re_inv)[ix - 1] * (*gpu_A)[ia];
      }

      ar += 8;
    }
  }

  eb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih, *gpu_H_linear, *gpu_hk, *gpu_y_meas, *b_gpu_y_meas);
  fb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_y_meas, *gpu_C, *gpu_ik, *gpu_op);

  // Hk is always a 13*4 matrix and hk is always a 13*1 matrix
  cudaDeviceSynchronize();
  cudaMemcpy(&op[0], gpu_op, 64UL, cudaMemcpyDeviceToHost);
  cudaFree(*gpu_y_meas);
  cudaFree(*gpu_iv5);
  cudaFree(*gpu_iv4);
  cudaFree(*gpu_iv3);
  cudaFree(*gpu_Re);
  cudaFree(*gpu_B_usedMeas_vec);
  cudaFree(*gpu_a);
  cudaFree(*gpu_iv2);
  cudaFree(*gpu_iv1);
  cudaFree(*gpu_iv);
  cudaFree(*b_gpu_Rw);
  cudaFree(*gpu_op);
  cudaFree(*gpu_Rw);
  cudaFree(*gpu_Rw_inv);
  cudaFree(*gpu_ipiv);
  cudaFree(*gpu_p);
  cudaFree(*gpu_Fk_inv);
  cudaFree(*gpu_x);
  cudaFree(*b_gpu_ipiv);
  cudaFree(*b_gpu_p);
  cudaFree(*gpu_Gk);
  cudaFree(*gpu_Ih);
  cudaFree(*b_gpu_Gk);
  cudaFree(*b_gpu_x);
  cudaFree(*b_gpu_Ih);
  cudaFree(*c_gpu_Ih);
  cudaFree(*gpu_calcEq);
  cudaFree(*gpu_ih);
  cudaFree(*gpu_ik);
  cudaFree(*gpu_Re_inv);
  cudaFree(*b_gpu_Re_inv);
  cudaFree(*gpu_H_linear);
  cudaFree(*gpu_hk);
  cudaFree(*gpu_A);
  cudaFree(*b_gpu_y_meas);
  cudaFree(*gpu_C);
}

//
// Arguments    : void
// Return Type  : void
//
void InformationFilterUpdate_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void InformationFilterUpdate_terminate()
{
  // (no terminate code required)
}

//
// File trailer for InformationFilterUpdate.cu
//
// [EOF]
//
