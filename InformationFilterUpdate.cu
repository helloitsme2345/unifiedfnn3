//
// File: InformationFilterUpdate.cu
//
// GPU Coder version                    : 1.5
// CUDA/C/C++ source code generated on  : 07-Sep-2020 10:58:04
//

// Include Files
#include "InformationFilterUpdate.h"
#include "MWCudaDimUtility.hpp"
#include "cusolverDn.h"
#include <cmath>

// Variable Definitions
static unsigned int method;
static unsigned int state;
static unsigned int b_state[2];
static unsigned int c_state[625];
static boolean_T state_not_empty;
static boolean_T isInitialized_InformationFilterUpdate = false;

// Function Declarations
static __global__ void InformationFilterUpdate_kernel1(double Pk_init[256]);
static __global__ void InformationFilterUpdate_kernel2(signed char ipiv[16]);
static __global__ void InformationFilterUpdate_kernel3(const signed char Ik[256],
  double b_Ik[256]);
static __global__ void InformationFilterUpdate_kernel4(double Rw[64]);
static __global__ void InformationFilterUpdate_kernel5(const double Rw[8],
  double b_Rw[64]);
static __global__ void InformationFilterUpdate_kernel6(double Rw_inv[64]);
static __global__ void InformationFilterUpdate_kernel7(signed char ipiv[8]);
static __global__ void InformationFilterUpdate_kernel8(signed char p[8]);
static __global__ void InformationFilterUpdate_kernel9(double Fk_inv[256]);
static __global__ void ab_InformationFilterUpdate_kern(const double H_linear[208],
  double A[208]);
static __global__ void b_InformationFilterUpdate_kerne(const double T, double
  Pk_init[256]);
static void b_rand(double r[256]);
static __global__ void bb_InformationFilterUpdate_kern(const double xk_m_out[16],
  const double H_linear[208], const double hk[13], const double y_meas[13],
  double b_y_meas[13]);
static __global__ void c_InformationFilterUpdate_kerne(const signed char iv[16],
  const signed char iv1[16], const signed char iv2[16], const signed char iv3[16],
  const signed char iv4[16], const signed char iv5[16], const signed char iv6[16],
  const signed char iv7[16], signed char ipiv[16], double Pk_init[256]);
static __global__ void cb_InformationFilterUpdate_kern(const double y_meas[13],
  const double C[208], double op[16]);
static __global__ void d_InformationFilterUpdate_kerne(signed char p[16]);
static __global__ void e_InformationFilterUpdate_kerne(const double Fk_inv[256],
  double Pk_init[256]);
static void eml_rand_init();
static void eml_rand_mcg16807_stateful_init();
static void eml_rand_shr3cong_stateful_init();
static __global__ void f_InformationFilterUpdate_kerne(const double Gk[128],
  double A[128]);
static __global__ void g_InformationFilterUpdate_kerne(const double Rw_inv[64],
  double Rw[64], double y[64]);
static __global__ void h_InformationFilterUpdate_kerne(signed char ipiv[8]);
static __global__ void i_InformationFilterUpdate_kerne(signed char p[8]);
static __global__ void j_InformationFilterUpdate_kerne(const double Gk[128],
  double A[128]);
static __global__ void k_InformationFilterUpdate_kerne(double ih[16]);
static __global__ void l_InformationFilterUpdate_kerne(const double ih[16],
  const double Ik[256], double b_Ik[16]);
static __global__ void m_InformationFilterUpdate_kerne(const double Ik[16],
  const double Fk_inv[256], double ih[16]);
static __global__ void n_InformationFilterUpdate_kerne(const double Pk_init[256],
  const double ih[16], double op[16]);
static __global__ void o_InformationFilterUpdate_kerne(const double C[256],
  double Ih[256], double Pk_init[256]);
static __global__ void p_InformationFilterUpdate_kerne(signed char ipiv[16]);
static __global__ void q_InformationFilterUpdate_kerne(signed char p[16]);
static __global__ void r_InformationFilterUpdate_kerne(const double op[16],
  const double Pk_init[256], double xk_m_out[16]);
static __global__ void s_InformationFilterUpdate_kerne(const double
  B_usedMeas_vec[13], const double Re[13], double Re_inv[13]);
static __global__ void t_InformationFilterUpdate_kerne(double Re_inv[169]);
static __global__ void u_InformationFilterUpdate_kerne(const double Re_inv[13],
  double b_Re_inv[169]);
static __global__ void v_InformationFilterUpdate_kerne(double H_linear[208]);
static __global__ void w_InformationFilterUpdate_kerne(const signed char iv8[16],
  const signed char iv9[16], const signed char iv10[16], const signed char iv11
  [16], double H_linear[208]);
static __global__ void x_InformationFilterUpdate_kerne(const double xk_m_out[16],
  const double H_linear[208], double hk[13]);
static __global__ void y_InformationFilterUpdate_kerne(const double smax, const
  double delta, const double L_imuToRear, const double xk_m_out[16], double
  H_linear[208], double hk[13]);

// Function Definitions

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Pk_init[256]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void InformationFilterUpdate_kernel1
  (double Pk_init[256])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 256) {
    Pk_init[k] = floor(Pk_init[k] * 2.0);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel2
  (signed char ipiv[16])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    // cinclude comment
    // cinclude comment
    ipiv[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char Ik[256]
//                double b_Ik[256]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void InformationFilterUpdate_kernel3
  (const signed char Ik[256], double b_Ik[256])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 256) {
    // cinclude comment
    b_Ik[k] = static_cast<double>(Ik[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Rw[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void InformationFilterUpdate_kernel4
  (double Rw[64])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 64) {
    // 4*4
    // ---- Prediction step -------------------------
    Rw[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Rw[8]
//                double b_Rw[64]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel5(
  const double Rw[8], double b_Rw[64])
{
  unsigned int threadId;
  int j;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  j = static_cast<int>(threadId);
  if (j < 8) {
    b_Rw[j + (j << 3)] = Rw[j];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Rw_inv[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void InformationFilterUpdate_kernel6
  (double Rw_inv[64])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 64) {
    Rw_inv[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel7
  (signed char ipiv[8])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 8) {
    ipiv[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void InformationFilterUpdate_kernel8
  (signed char p[8])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 8) {
    p[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Fk_inv[256]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void InformationFilterUpdate_kernel9
  (double Fk_inv[256])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 256) {
    //  System matrix
    Fk_inv[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double H_linear[208]
//                double A[208]
// Return Type  : void
//
static __global__ __launch_bounds__(224, 1) void ab_InformationFilterUpdate_kern
  (const double H_linear[208], double A[208])
{
  unsigned int threadId;
  int k;
  int i3;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i3 = static_cast<int>(threadId % 16U);
  k = static_cast<int>((threadId - static_cast<unsigned int>(i3)) / 16U);
  if (k < 13) {
    A[i3 + (k << 4)] = H_linear[k + 13 * i3];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double T
//                double Pk_init[256]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void b_InformationFilterUpdate_kerne(
  const double T, double Pk_init[256])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    Pk_init[110] = 0.0;
    Pk_init[126] = 0.0;
    Pk_init[142] = 0.0;
    Pk_init[158] = 0.0;
    Pk_init[174] = 0.0;
    Pk_init[190] = 0.0;
    Pk_init[206] = 0.0;
    Pk_init[222] = 0.0;
    Pk_init[238] = 1.0;
    Pk_init[254] = T;
  }
}

//
// Arguments    : double r[256]
// Return Type  : void
//
static void b_rand(double r[256])
{
  unsigned int y;
  unsigned int u[2];
  if (method == 4U) {
    for (int k = 0; k < 256; k++) {
      int hi;
      unsigned int b_r;
      hi = static_cast<int>(state / 127773U);
      b_r = 16807U * (state - hi * 127773U);
      y = 2836U * hi;
      if (b_r < y) {
        state = ~(y - b_r) & 2147483647U;
      } else {
        state = b_r - y;
      }

      r[k] = static_cast<double>(state) * 4.6566128752457969E-10;
    }
  } else if (method == 5U) {
    for (int k = 0; k < 256; k++) {
      unsigned int b_r;
      b_r = 69069U * b_state[0] + 1234567U;
      y = b_state[1] ^ b_state[1] << 13;
      y ^= y >> 17;
      y ^= y << 5;
      b_state[0] = b_r;
      b_state[1] = y;
      r[k] = static_cast<double>(b_r + y) * 2.328306436538696E-10;
    }
  } else {
    int hi;
    unsigned int b_r;
    if (!state_not_empty) {
      for (hi = 0; hi < 625; hi++) {
        c_state[hi] = 0U;
      }

      b_r = 5489U;
      c_state[0] = 5489U;
      for (hi = 0; hi < 623; hi++) {
        b_r = ((b_r ^ b_r >> 30U) * 1812433253U + hi) + 1U;
        c_state[hi + 1] = b_r;
      }

      c_state[624] = 624U;
      state_not_empty = true;
    }

    for (int k = 0; k < 256; k++) {
      double c_r;

      // ========================= COPYRIGHT NOTICE ============================ 
      //  This is a uniform (0,1) pseudorandom number generator based on:        
      //                                                                         
      //  A C-program for MT19937, with initialization improved 2002/1/26.       
      //  Coded by Takuji Nishimura and Makoto Matsumoto.                        
      //                                                                         
      //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,      
      //  All rights reserved.                                                   
      //                                                                         
      //  Redistribution and use in source and binary forms, with or without     
      //  modification, are permitted provided that the following conditions     
      //  are met:                                                               
      //                                                                         
      //    1. Redistributions of source code must retain the above copyright    
      //       notice, this list of conditions and the following disclaimer.     
      //                                                                         
      //    2. Redistributions in binary form must reproduce the above copyright 
      //       notice, this list of conditions and the following disclaimer      
      //       in the documentation and/or other materials provided with the     
      //       distribution.                                                     
      //                                                                         
      //    3. The names of its contributors may not be used to endorse or       
      //       promote products derived from this software without specific      
      //       prior written permission.                                         
      //                                                                         
      //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    
      //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      
      //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  
      //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT  
      //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  
      //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       
      //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  
      //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  
      //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    
      //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
      //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
      //                                                                         
      // =============================   END   ================================= 
      do {
        for (hi = 0; hi < 2; hi++) {
          b_r = c_state[624] + 1U;
          if (b_r >= 625U) {
            int kk;
            for (kk = 0; kk < 227; kk++) {
              y = (c_state[kk] & 2147483648U) | (c_state[kk + 1] & 2147483647U);
              if ((y & 1U) == 0U) {
                y >>= 1U;
              } else {
                y = y >> 1U ^ 2567483615U;
              }

              c_state[kk] = c_state[kk + 397] ^ y;
            }

            for (kk = 0; kk < 396; kk++) {
              y = (c_state[kk + 227] & 2147483648U) | (c_state[kk + 228] &
                2147483647U);
              if ((y & 1U) == 0U) {
                y >>= 1U;
              } else {
                y = y >> 1U ^ 2567483615U;
              }

              c_state[kk + 227] = c_state[kk] ^ y;
            }

            y = (c_state[623] & 2147483648U) | (c_state[0] & 2147483647U);
            if ((y & 1U) == 0U) {
              y >>= 1U;
            } else {
              y = y >> 1U ^ 2567483615U;
            }

            c_state[623] = c_state[396] ^ y;
            b_r = 1U;
          }

          y = c_state[static_cast<int>(b_r) - 1];
          c_state[624] = b_r;
          y ^= y >> 11U;
          y ^= y << 7U & 2636928640U;
          y ^= y << 15U & 4022730752U;
          y ^= y >> 18U;
          u[hi] = y;
        }

        u[0] >>= 5U;
        u[1] >>= 6U;
        c_r = 1.1102230246251565E-16 * (static_cast<double>(u[0]) * 6.7108864E+7
          + static_cast<double>(u[1]));
      } while (c_r == 0.0);

      r[k] = c_r;
    }
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double xk_m_out[16]
//                const double H_linear[208]
//                const double hk[13]
//                const double y_meas[13]
//                double b_y_meas[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void bb_InformationFilterUpdate_kern(
  const double xk_m_out[16], const double H_linear[208], const double hk[13],
  const double y_meas[13], double b_y_meas[13])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 13) {
    d = 0.0;
    for (int i3 = 0; i3 < 16; i3++) {
      d += H_linear[k + 13 * i3] * xk_m_out[i3];
    }

    b_y_meas[k] = (y_meas[k] - hk[k]) + d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv[16]
//                const signed char iv1[16]
//                const signed char iv2[16]
//                const signed char iv3[16]
//                const signed char iv4[16]
//                const signed char iv5[16]
//                const signed char iv6[16]
//                const signed char iv7[16]
//                signed char ipiv[16]
//                double Pk_init[256]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void c_InformationFilterUpdate_kerne(
  const signed char iv[16], const signed char iv1[16], const signed char iv2[16],
  const signed char iv3[16], const signed char iv4[16], const signed char iv5[16],
  const signed char iv6[16], const signed char iv7[16], signed char ipiv[16],
  double Pk_init[256])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    // cinclude comment
    Pk_init[(k << 4) + 1] = static_cast<double>(iv7[k]);
    Pk_init[(k << 4) + 3] = static_cast<double>(iv6[k]);
    Pk_init[(k << 4) + 5] = static_cast<double>(iv5[k]);
    Pk_init[(k << 4) + 7] = static_cast<double>(iv4[k]);
    Pk_init[(k << 4) + 9] = static_cast<double>(iv3[k]);
    Pk_init[(k << 4) + 11] = static_cast<double>(iv2[k]);
    Pk_init[(k << 4) + 13] = static_cast<double>(iv1[k]);
    Pk_init[(k << 4) + 15] = static_cast<double>(iv[k]);
    ipiv[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double y_meas[13]
//                const double C[208]
//                double op[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void cb_InformationFilterUpdate_kern(
  const double y_meas[13], const double C[208], double op[16])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    d = 0.0;
    for (int i3 = 0; i3 < 13; i3++) {
      d += C[k + (i3 << 4)] * y_meas[i3];
    }

    op[k] += d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void d_InformationFilterUpdate_kerne
  (signed char p[16])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    p[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Fk_inv[256]
//                double Pk_init[256]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void e_InformationFilterUpdate_kerne
  (const double Fk_inv[256], double Pk_init[256])
{
  unsigned int threadId;
  int k;
  int i3;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i3 = static_cast<int>(threadId % 16U);
  k = static_cast<int>((threadId - static_cast<unsigned int>(i3)) / 16U);
  if (k < 16) {
    Pk_init[i3 + (k << 4)] = Fk_inv[k + (i3 << 4)];
  }
}

//
// Arguments    : void
// Return Type  : void
//
static void eml_rand_init()
{
  method = 7U;
}

//
// Arguments    : void
// Return Type  : void
//
static void eml_rand_mcg16807_stateful_init()
{
  state = 1144108930U;
}

//
// Arguments    : void
// Return Type  : void
//
static void eml_rand_shr3cong_stateful_init()
{
  for (int i = 0; i < 2; i++) {
    b_state[i] = 158852560U * i + 362436069U;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[128]
//                double A[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void f_InformationFilterUpdate_kerne
  (const double Gk[128], double A[128])
{
  unsigned int threadId;
  int k;
  int i3;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i3 = static_cast<int>(threadId % 8U);
  k = static_cast<int>((threadId - static_cast<unsigned int>(i3)) / 8U);
  if (k < 16) {
    A[i3 + (k << 3)] = Gk[k + (i3 << 4)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Rw_inv[64]
//                double Rw[64]
//                double y[64]
// Return Type  : void
//
static __global__ __launch_bounds__(64, 1) void g_InformationFilterUpdate_kerne(
  const double Rw_inv[64], double Rw[64], double y[64])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 64) {
    y[k] = 0.0;
    Rw[k] += Rw_inv[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void h_InformationFilterUpdate_kerne
  (signed char ipiv[8])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 8) {
    ipiv[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[8]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void i_InformationFilterUpdate_kerne
  (signed char p[8])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 8) {
    p[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Gk[128]
//                double A[128]
// Return Type  : void
//
static __global__ __launch_bounds__(128, 1) void j_InformationFilterUpdate_kerne
  (const double Gk[128], double A[128])
{
  unsigned int threadId;
  int k;
  int i3;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  i3 = static_cast<int>(threadId % 8U);
  k = static_cast<int>((threadId - static_cast<unsigned int>(i3)) / 8U);
  if (k < 16) {
    A[i3 + (k << 3)] = Gk[k + (i3 << 4)];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double ih[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void k_InformationFilterUpdate_kerne
  (double ih[16])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    ih[13] = 0.0;
    ih[14] = 0.0;
    ih[15] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double ih[16]
//                const double Ik[256]
//                double b_Ik[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void l_InformationFilterUpdate_kerne(
  const double ih[16], const double Ik[256], double b_Ik[16])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    d = 0.0;
    for (int i3 = 0; i3 < 16; i3++) {
      d += Ik[k + (i3 << 4)] * ih[i3];
    }

    b_Ik[k] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Ik[16]
//                const double Fk_inv[256]
//                double ih[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void m_InformationFilterUpdate_kerne(
  const double Ik[16], const double Fk_inv[256], double ih[16])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    d = 0.0;
    for (int i3 = 0; i3 < 16; i3++) {
      d += Fk_inv[i3 + (k << 4)] * Ik[i3];
    }

    ih[k] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Pk_init[256]
//                const double ih[16]
//                double op[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void n_InformationFilterUpdate_kerne(
  const double Pk_init[256], const double ih[16], double op[16])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    // 4*1
    d = 0.0;
    for (int i3 = 0; i3 < 16; i3++) {
      d += Pk_init[k + (i3 << 4)] * ih[i3];
    }

    op[k] = ih[k] - d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double C[256]
//                double Ih[256]
//                double Pk_init[256]
// Return Type  : void
//
static __global__ __launch_bounds__(256, 1) void o_InformationFilterUpdate_kerne
  (const double C[256], double Ih[256], double Pk_init[256])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 256) {
    // 4*1
    Pk_init[k] = 0.0;
    Ih[k] -= C[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char ipiv[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void p_InformationFilterUpdate_kerne
  (signed char ipiv[16])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    // cinclude comment
    ipiv[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                signed char p[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void q_InformationFilterUpdate_kerne
  (signed char p[16])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    p[k] = static_cast<signed char>(k + 1);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double op[16]
//                const double Pk_init[256]
//                double xk_m_out[16]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void r_InformationFilterUpdate_kerne(
  const double op[16], const double Pk_init[256], double xk_m_out[16])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    // 4*4
    d = 0.0;
    for (int i3 = 0; i3 < 16; i3++) {
      d += Pk_init[k + (i3 << 4)] * op[i3];
    }

    xk_m_out[k] = d;
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
static __global__ __launch_bounds__(32, 1) void s_InformationFilterUpdate_kerne(
  const double B_usedMeas_vec[13], const double Re[13], double Re_inv[13])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 13) {
    // 1*1
    Re_inv[k] = 1.0 / Re[k] * B_usedMeas_vec[k];
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                double Re_inv[169]
// Return Type  : void
//
static __global__ __launch_bounds__(192, 1) void t_InformationFilterUpdate_kerne
  (double Re_inv[169])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 169) {
    // 1*13
    Re_inv[k] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double Re_inv[13]
//                double b_Re_inv[169]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void u_InformationFilterUpdate_kerne(
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
//                double H_linear[208]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void v_InformationFilterUpdate_kerne
  (double H_linear[208])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    H_linear[205] = 0.0;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const signed char iv8[16]
//                const signed char iv9[16]
//                const signed char iv10[16]
//                const signed char iv11[16]
//                double H_linear[208]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void w_InformationFilterUpdate_kerne(
  const signed char iv8[16], const signed char iv9[16], const signed char iv10
  [16], const signed char iv11[16], double H_linear[208])
{
  unsigned int threadId;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 16) {
    H_linear[13 * k] = static_cast<double>(iv11[k]);
    H_linear[13 * k + 2] = static_cast<double>(iv10[k]);
    H_linear[13 * k + 3] = static_cast<double>(iv9[k]);
    H_linear[13 * k + 4] = static_cast<double>(iv8[k]);
    H_linear[13 * k + 5] = static_cast<double>(iv8[k]);
    H_linear[13 * k + 11] = static_cast<double>(iv9[k]);
    H_linear[13 * k + 12] = static_cast<double>(iv9[k]);
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double xk_m_out[16]
//                const double H_linear[208]
//                double hk[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void x_InformationFilterUpdate_kerne(
  const double xk_m_out[16], const double H_linear[208], double hk[13])
{
  unsigned int threadId;
  double d;
  int k;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  k = static_cast<int>(threadId);
  if (k < 13) {
    //  Nonlinear parts
    d = 0.0;
    for (int i3 = 0; i3 < 16; i3++) {
      d += H_linear[k + 13 * i3] * xk_m_out[i3];
    }

    hk[k] = d;
  }
}

//
// Arguments    : dim3 blockArg
//                dim3 gridArg
//                const double smax
//                const double delta
//                const double L_imuToRear
//                const double xk_m_out[16]
//                double H_linear[208]
//                double hk[13]
// Return Type  : void
//
static __global__ __launch_bounds__(32, 1) void y_InformationFilterUpdate_kerne(
  const double smax, const double delta, const double L_imuToRear, const double
  xk_m_out[16], double H_linear[208], double hk[13])
{
  unsigned int threadId;
  int tmpIdx;
  threadId = static_cast<unsigned int>(mwGetGlobalThreadIndex());
  tmpIdx = static_cast<int>(threadId);
  if (tmpIdx < 1) {
    hk[0] -= xk_m_out[2] * xk_m_out[2] * L_imuToRear;
    hk[1] += xk_m_out[0] * xk_m_out[2];
    hk[10] = xk_m_out[0] * cos(delta) + smax * xk_m_out[2] * sin(delta);

    // updates the Hk 13*4matrix HK(5,1)=Hk matrix's 5th row 1st element
    // being updated with cos(delta)value . Likewise for all.
    H_linear[26] = -2.0 * xk_m_out[2] * L_imuToRear;
    H_linear[1] = xk_m_out[2];
    H_linear[27] = xk_m_out[0];
    H_linear[4] = cos(delta);
    H_linear[10] = cos(delta);
    H_linear[36] = smax * sin(delta);
  }
}

//
// persistent Ik ik
// Arguments    : const double y_meas[13]
//                const double B_usedMeas_vec[13]
//                const double initialization_vec[4]
//                double delta
//                const double Rw[8]
//                const double Re[13]
//                double L_imuToRear
//                double L_geometricWheelbase
//                const double L_trackWidth[5]
//                const double L_axlePos[5]
//                double T
//                double xk_m_out[16]
//                double op[16]
// Return Type  : void
//
void InformationFilterUpdate(const double y_meas[13], const double
  B_usedMeas_vec[13], const double initialization_vec[4], double delta, const
  double Rw[8], const double Re[13], double L_imuToRear, double
  L_geometricWheelbase, const double L_trackWidth[5], const double L_axlePos[5],
  double T, double xk_m_out[16], double op[16])
{
  int j;
  int c;
  int jp1j;
  int b_c;
  int iy;
  int i;
  static const signed char Ik[256] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1 };

  int ix;
  double smax;
  signed char b_i;
  int k;
  double s;
  int jy;
  int i1;
  int ia;
  int i2;
  static const signed char iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1 };

  static const signed char iv1[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0 };

  static const signed char iv2[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0 };

  static const signed char iv3[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0 };

  static const signed char iv4[16] = { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0 };

  static const signed char iv5[16] = { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  static const signed char iv6[16] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  static const signed char iv7[16] = { 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  int ic;
  double C[128];
  double b_C[128];
  double c_C[128];
  static const signed char iv8[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0 };

  static const signed char iv9[16] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0,
    0, 0 };

  static const signed char iv10[16] = { 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0,
    1, 0 };

  static const signed char iv11[16] = { 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1,
    0, 0 };

  double (*gpu_Pk_init)[256];
  signed char (*gpu_ipiv)[16];
  signed char (*gpu_Ik)[256];
  double (*b_gpu_Ik)[256];
  double (*gpu_Rw)[64];
  double (*b_gpu_Rw)[8];
  double (*gpu_Rw_inv)[64];
  signed char (*b_gpu_ipiv)[8];
  signed char (*gpu_p)[8];
  double (*gpu_Fk_inv)[256];
  signed char (*gpu_iv)[16];
  signed char (*gpu_iv1)[16];
  signed char (*gpu_iv2)[16];
  signed char (*gpu_iv3)[16];
  signed char (*gpu_iv4)[16];
  signed char (*gpu_iv5)[16];
  signed char (*gpu_iv6)[16];
  signed char (*gpu_iv7)[16];
  signed char (*b_gpu_p)[16];
  double (*gpu_Gk)[128];
  double (*gpu_A)[128];
  double (*gpu_y)[64];
  double (*gpu_ih)[16];
  double (*c_gpu_Ik)[16];
  double (*gpu_op)[16];
  double (*gpu_C)[256];
  double (*gpu_Ih)[256];
  double (*gpu_xk_m_out)[16];
  double (*gpu_B_usedMeas_vec)[13];
  double (*gpu_Re)[13];
  double (*gpu_Re_inv)[13];
  double (*b_gpu_Re_inv)[169];
  double (*gpu_H_linear)[208];
  signed char (*gpu_iv8)[16];
  signed char (*gpu_iv9)[16];
  signed char (*gpu_iv10)[16];
  signed char (*gpu_iv11)[16];
  double (*gpu_hk)[13];
  double (*b_gpu_A)[208];
  double (*gpu_y_meas)[13];
  double (*b_gpu_y_meas)[13];
  double (*b_gpu_C)[208];
  boolean_T syncIsDirty;
  if (!isInitialized_InformationFilterUpdate) {
    InformationFilterUpdate_initialize();
  }

  cudaMallocManaged(&b_gpu_C, 1664ULL);
  cudaMallocManaged(&b_gpu_y_meas, 104ULL);
  cudaMallocManaged(&b_gpu_A, 1664ULL);
  cudaMallocManaged(&gpu_hk, 104ULL);
  cudaMallocManaged(&gpu_H_linear, 1664ULL);
  cudaMallocManaged(&b_gpu_Re_inv, 1352ULL);
  cudaMallocManaged(&gpu_Re_inv, 104ULL);
  cudaMallocManaged(&gpu_Ih, 2048ULL);
  cudaMallocManaged(&gpu_C, 2048ULL);
  cudaMallocManaged(&c_gpu_Ik, 128ULL);
  cudaMallocManaged(&gpu_ih, 128ULL);
  cudaMallocManaged(&gpu_y, 512ULL);
  cudaMallocManaged(&gpu_A, 1024ULL);
  cudaMallocManaged(&gpu_Gk, 1024ULL);
  cudaMallocManaged(&b_gpu_p, 16ULL);
  cudaMallocManaged(&gpu_Fk_inv, 2048ULL);
  cudaMallocManaged(&gpu_p, 8ULL);
  cudaMallocManaged(&b_gpu_ipiv, 8ULL);
  cudaMallocManaged(&gpu_Rw_inv, 512ULL);
  cudaMallocManaged(&gpu_Rw, 512ULL);
  cudaMallocManaged(&b_gpu_Ik, 2048ULL);
  cudaMallocManaged(&gpu_ipiv, 16ULL);
  cudaMallocManaged(&gpu_Pk_init, 2048ULL);
  cudaMallocManaged(&gpu_xk_m_out, 128ULL);
  cudaMallocManaged(&gpu_op, 128ULL);
  cudaMallocManaged(&gpu_Ik, 256ULL);
  cudaMallocManaged(&b_gpu_Rw, 64ULL);
  cudaMallocManaged(&gpu_iv, 16ULL);
  cudaMallocManaged(&gpu_iv1, 16ULL);
  cudaMallocManaged(&gpu_iv2, 16ULL);
  cudaMallocManaged(&gpu_iv3, 16ULL);
  cudaMallocManaged(&gpu_iv4, 16ULL);
  cudaMallocManaged(&gpu_iv5, 16ULL);
  cudaMallocManaged(&gpu_iv6, 16ULL);
  cudaMallocManaged(&gpu_iv7, 16ULL);
  cudaMallocManaged(&gpu_B_usedMeas_vec, 104ULL);
  cudaMallocManaged(&gpu_Re, 104ULL);
  cudaMallocManaged(&gpu_iv8, 16ULL);
  cudaMallocManaged(&gpu_iv9, 16ULL);
  cudaMallocManaged(&gpu_iv10, 16ULL);
  cudaMallocManaged(&gpu_iv11, 16ULL);
  cudaMallocManaged(&gpu_y_meas, 104ULL);
  cudaMemcpy(gpu_y_meas, (void *)&y_meas[0], 104ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv11, (void *)&iv11[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv10, (void *)&iv10[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv9, (void *)&iv9[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv8, (void *)&iv8[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_Re, (void *)&Re[0], 104ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_B_usedMeas_vec, (void *)&B_usedMeas_vec[0], 104ULL,
             cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv7, (void *)&iv7[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv6, (void *)&iv6[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv5, (void *)&iv5[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv4, (void *)&iv4[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv3, (void *)&iv3[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv2, (void *)&iv2[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv1, (void *)&iv1[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_iv, (void *)&iv[0], 16ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(b_gpu_Rw, (void *)&Rw[0], 64ULL, cudaMemcpyHostToDevice);
  cudaMemcpy(gpu_Ik, (void *)&Ik[0], 256ULL, cudaMemcpyHostToDevice);
  b_rand(*gpu_Pk_init);
  InformationFilterUpdate_kernel1<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
    (*gpu_Pk_init);
  InformationFilterUpdate_kernel2<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 15; j++) {
    c = j * 17;
    jp1j = c - 12;
    b_c = 14 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_Pk_init)[c]);
    for (k = 0; k <= b_c; k++) {
      ix++;
      s = std::abs((*gpu_Pk_init)[ix]);
      if (s > smax) {
        iy = k + 1;
        smax = s;
      }
    }

    if ((*gpu_Pk_init)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        b_c = iy;
        for (k = 0; k < 16; k++) {
          ix = j + k * 16;
          iy = b_c + k * 16;
          smax = (*gpu_Pk_init)[ix];
          (*gpu_Pk_init)[ix] = (*gpu_Pk_init)[iy];
          (*gpu_Pk_init)[iy] = smax;
        }
      }

      i1 = (c - j) + 2;
      for (i = 0; i <= i1 - jp1j; i++) {
        iy = (c + i) + 1;
        (*gpu_Pk_init)[iy] /= (*gpu_Pk_init)[c];
      }
    }

    b_c = 14 - j;
    iy = c + 18;
    jy = c + 16;
    for (ia = 0; ia <= b_c; ia++) {
      smax = (*gpu_Pk_init)[jy];
      if ((*gpu_Pk_init)[jy] != 0.0) {
        ix = c;
        i1 = iy - 14;
        i2 = iy - j;
        for (jp1j = 0; jp1j <= i2 - i1; jp1j++) {
          i = (iy + jp1j) - 1;
          (*gpu_Pk_init)[i] += (*gpu_Pk_init)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 16;
      iy += 16;
    }
  }

  InformationFilterUpdate_kernel3<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
    (*gpu_Ik, *b_gpu_Ik);
  syncIsDirty = true;
  for (i = 0; i < 15; i++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    b_i = (*gpu_ipiv)[i];
    if ((*gpu_ipiv)[i] != i + 1) {
      for (j = 0; j < 16; j++) {
        iy = static_cast<int>((*b_gpu_Ik)[i + (j << 4)]);
        (*b_gpu_Ik)[i + (j << 4)] = (*b_gpu_Ik)[(b_i + (j << 4)) - 1];
        (*b_gpu_Ik)[(b_i + (j << 4)) - 1] = iy;
      }
    }
  }

  for (j = 0; j < 16; j++) {
    jp1j = j << 4;
    for (k = 0; k < 16; k++) {
      jy = k << 4;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*b_gpu_Ik)[k + jp1j] != 0.0) {
        for (i = 0; i <= 14 - k; i++) {
          iy = (k + i) + 1;
          (*b_gpu_Ik)[iy + jp1j] -= (*b_gpu_Ik)[k + jp1j] * (*gpu_Pk_init)[iy +
            jy];
        }
      }
    }
  }

  for (j = 0; j < 16; j++) {
    jp1j = (j << 4) - 1;
    for (k = 0; k < 16; k++) {
      iy = 16 - k;
      jy = (15 - k) << 4;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*b_gpu_Ik)[(jp1j - k) + 16] != 0.0) {
        (*b_gpu_Ik)[(jp1j - k) + 16] /= (*gpu_Pk_init)[(jy - k) + 15];
        for (i = 0; i <= iy - 2; i++) {
          (*b_gpu_Ik)[(i + jp1j) + 1] -= (*b_gpu_Ik)[(jp1j - k) + 16] *
            (*gpu_Pk_init)[i + jy];
        }
      }
    }
  }

  // 4*4
  // ---- Prediction step -------------------------
  InformationFilterUpdate_kernel4<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_Rw);
  InformationFilterUpdate_kernel5<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_Rw, *gpu_Rw);
  InformationFilterUpdate_kernel6<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_Rw_inv);
  InformationFilterUpdate_kernel7<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 7; j++) {
    c = j * 9;
    jp1j = c - 4;
    b_c = 6 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_Rw)[c]);
    for (k = 0; k <= b_c; k++) {
      ix++;
      s = std::abs((*gpu_Rw)[ix]);
      if (s > smax) {
        iy = k + 1;
        smax = s;
      }
    }

    if ((*gpu_Rw)[c + iy] != 0.0) {
      if (iy != 0) {
        (*b_gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        b_c = iy;
        for (k = 0; k < 8; k++) {
          ix = j + k * 8;
          iy = b_c + k * 8;
          smax = (*gpu_Rw)[ix];
          (*gpu_Rw)[ix] = (*gpu_Rw)[iy];
          (*gpu_Rw)[iy] = smax;
        }
      }

      i1 = (c - j) + 2;
      for (i = 0; i <= i1 - jp1j; i++) {
        iy = (c + i) + 1;
        (*gpu_Rw)[iy] /= (*gpu_Rw)[c];
      }
    }

    b_c = 6 - j;
    iy = c + 10;
    jy = c + 8;
    for (ia = 0; ia <= b_c; ia++) {
      smax = (*gpu_Rw)[jy];
      if ((*gpu_Rw)[jy] != 0.0) {
        ix = c;
        i1 = iy - 6;
        i2 = iy - j;
        for (jp1j = 0; jp1j <= i2 - i1; jp1j++) {
          i = (iy + jp1j) - 1;
          (*gpu_Rw)[i] += (*gpu_Rw)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 8;
      iy += 8;
    }
  }

  InformationFilterUpdate_kernel8<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_p);
  syncIsDirty = true;
  for (k = 0; k < 7; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*b_gpu_ipiv)[k] > k + 1) {
      iy = (*gpu_p)[(*b_gpu_ipiv)[k] - 1];
      (*gpu_p)[(*b_gpu_ipiv)[k] - 1] = (*gpu_p)[k];
      (*gpu_p)[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 8; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    b_i = (*gpu_p)[k];
    (*gpu_Rw_inv)[k + (((*gpu_p)[k] - 1) << 3)] = 1.0;
    for (j = 0; j <= 7 - k; j++) {
      ia = k + j;
      if ((*gpu_Rw_inv)[ia + ((b_i - 1) << 3)] != 0.0) {
        for (i = 0; i <= 6 - ia; i++) {
          iy = (ia + i) + 1;
          (*gpu_Rw_inv)[iy + ((b_i - 1) << 3)] -= (*gpu_Rw_inv)[ia + ((b_i - 1) <<
            3)] * (*gpu_Rw)[iy + (ia << 3)];
        }
      }
    }
  }

  for (j = 0; j < 8; j++) {
    jp1j = (j << 3) - 1;
    for (k = 0; k < 8; k++) {
      iy = 8 - k;
      jy = (7 - k) << 3;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Rw_inv)[(jp1j - k) + 8] != 0.0) {
        (*gpu_Rw_inv)[(jp1j - k) + 8] /= (*gpu_Rw)[(jy - k) + 7];
        for (i = 0; i <= iy - 2; i++) {
          (*gpu_Rw_inv)[(i + jp1j) + 1] -= (*gpu_Rw_inv)[(jp1j - k) + 8] *
            (*gpu_Rw)[i + jy];
        }
      }
    }
  }

  //  System matrix
  InformationFilterUpdate_kernel9<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
    (*gpu_Fk_inv);
  cudaDeviceSynchronize();
  (*gpu_Pk_init)[0] = 1.0;
  (*gpu_Pk_init)[16] = T;
  (*gpu_Pk_init)[32] = 0.0;
  (*gpu_Pk_init)[48] = 0.0;
  (*gpu_Pk_init)[64] = 0.0;
  (*gpu_Pk_init)[80] = 0.0;
  (*gpu_Pk_init)[96] = 0.0;
  (*gpu_Pk_init)[112] = 0.0;
  (*gpu_Pk_init)[128] = 0.0;
  (*gpu_Pk_init)[144] = 0.0;
  (*gpu_Pk_init)[160] = 0.0;
  (*gpu_Pk_init)[176] = 0.0;
  (*gpu_Pk_init)[192] = 0.0;
  (*gpu_Pk_init)[208] = 0.0;
  (*gpu_Pk_init)[224] = 0.0;
  (*gpu_Pk_init)[240] = 0.0;
  (*gpu_Pk_init)[2] = 0.0;
  (*gpu_Pk_init)[18] = 0.0;
  (*gpu_Pk_init)[34] = 1.0;
  (*gpu_Pk_init)[50] = 0.0;
  (*gpu_Pk_init)[66] = 0.0;
  (*gpu_Pk_init)[82] = 0.0;
  (*gpu_Pk_init)[98] = 0.0;
  (*gpu_Pk_init)[114] = 0.0;
  (*gpu_Pk_init)[130] = T;
  (*gpu_Pk_init)[146] = 0.0;
  (*gpu_Pk_init)[162] = 0.0;
  (*gpu_Pk_init)[178] = 0.0;
  (*gpu_Pk_init)[194] = 0.0;
  (*gpu_Pk_init)[210] = 0.0;
  (*gpu_Pk_init)[226] = 0.0;
  (*gpu_Pk_init)[242] = 0.0;
  (*gpu_Pk_init)[4] = 0.0;
  (*gpu_Pk_init)[20] = 0.0;
  (*gpu_Pk_init)[36] = 0.0;
  (*gpu_Pk_init)[52] = 0.0;
  (*gpu_Pk_init)[68] = 1.0;
  (*gpu_Pk_init)[84] = 0.0;
  (*gpu_Pk_init)[100] = 0.0;
  (*gpu_Pk_init)[116] = 0.0;
  (*gpu_Pk_init)[132] = 0.0;
  (*gpu_Pk_init)[148] = 0.0;
  (*gpu_Pk_init)[164] = 0.0;
  (*gpu_Pk_init)[180] = 0.0;
  (*gpu_Pk_init)[196] = 0.0;
  (*gpu_Pk_init)[212] = 0.0;
  (*gpu_Pk_init)[228] = 0.0;
  (*gpu_Pk_init)[244] = T;
  (*gpu_Pk_init)[6] = 0.0;
  (*gpu_Pk_init)[22] = 0.0;
  (*gpu_Pk_init)[38] = 0.0;
  (*gpu_Pk_init)[54] = 0.0;
  (*gpu_Pk_init)[70] = 0.0;
  (*gpu_Pk_init)[86] = 0.0;
  (*gpu_Pk_init)[102] = 1.0;
  (*gpu_Pk_init)[118] = 0.0;
  (*gpu_Pk_init)[134] = T;
  (*gpu_Pk_init)[150] = 0.0;
  (*gpu_Pk_init)[166] = 0.0;
  (*gpu_Pk_init)[182] = 0.0;
  (*gpu_Pk_init)[198] = 0.0;
  (*gpu_Pk_init)[214] = 0.0;
  (*gpu_Pk_init)[230] = 0.0;
  (*gpu_Pk_init)[246] = 0.0;
  (*gpu_Pk_init)[8] = 0.0;
  (*gpu_Pk_init)[24] = 0.0;
  (*gpu_Pk_init)[40] = 0.0;
  (*gpu_Pk_init)[56] = 0.0;
  (*gpu_Pk_init)[72] = 0.0;
  (*gpu_Pk_init)[88] = 0.0;
  (*gpu_Pk_init)[104] = 0.0;
  (*gpu_Pk_init)[120] = 0.0;
  (*gpu_Pk_init)[136] = 1.0;
  (*gpu_Pk_init)[152] = 0.0;
  (*gpu_Pk_init)[168] = 0.0;
  (*gpu_Pk_init)[184] = T;
  (*gpu_Pk_init)[200] = 0.0;
  (*gpu_Pk_init)[216] = 0.0;
  (*gpu_Pk_init)[232] = 0.0;
  (*gpu_Pk_init)[248] = 0.0;
  (*gpu_Pk_init)[10] = 0.0;
  (*gpu_Pk_init)[26] = 0.0;
  (*gpu_Pk_init)[42] = 0.0;
  (*gpu_Pk_init)[58] = 0.0;
  (*gpu_Pk_init)[74] = 0.0;
  (*gpu_Pk_init)[90] = 0.0;
  (*gpu_Pk_init)[106] = 0.0;
  (*gpu_Pk_init)[122] = 0.0;
  (*gpu_Pk_init)[138] = 0.0;
  (*gpu_Pk_init)[154] = 0.0;
  (*gpu_Pk_init)[170] = 1.0;
  (*gpu_Pk_init)[186] = 0.0;
  (*gpu_Pk_init)[202] = 0.0;
  (*gpu_Pk_init)[218] = 0.0;
  (*gpu_Pk_init)[234] = T;
  (*gpu_Pk_init)[250] = 0.0;
  (*gpu_Pk_init)[12] = 0.0;
  (*gpu_Pk_init)[28] = 0.0;
  (*gpu_Pk_init)[44] = 0.0;
  (*gpu_Pk_init)[60] = 0.0;
  (*gpu_Pk_init)[76] = 0.0;
  (*gpu_Pk_init)[92] = 0.0;
  (*gpu_Pk_init)[108] = 0.0;
  (*gpu_Pk_init)[124] = 0.0;
  (*gpu_Pk_init)[140] = T;
  (*gpu_Pk_init)[156] = 0.0;
  (*gpu_Pk_init)[172] = 0.0;
  (*gpu_Pk_init)[188] = 0.0;
  (*gpu_Pk_init)[204] = 1.0;
  (*gpu_Pk_init)[220] = 0.0;
  (*gpu_Pk_init)[236] = 0.0;
  (*gpu_Pk_init)[252] = 0.0;
  (*gpu_Pk_init)[14] = 0.0;
  (*gpu_Pk_init)[30] = 0.0;
  (*gpu_Pk_init)[46] = 0.0;
  (*gpu_Pk_init)[62] = 0.0;
  (*gpu_Pk_init)[78] = 0.0;
  (*gpu_Pk_init)[94] = 0.0;
  b_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(T,
    *gpu_Pk_init);
  c_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_iv, *gpu_iv1, *gpu_iv2, *gpu_iv3, *gpu_iv4, *gpu_iv5, *gpu_iv6,
     *gpu_iv7, *gpu_ipiv, *gpu_Pk_init);
  syncIsDirty = true;
  for (j = 0; j < 15; j++) {
    c = j * 17;
    jp1j = c - 12;
    b_c = 14 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_Pk_init)[c]);
    for (k = 0; k <= b_c; k++) {
      ix++;
      s = std::abs((*gpu_Pk_init)[ix]);
      if (s > smax) {
        iy = k + 1;
        smax = s;
      }
    }

    if ((*gpu_Pk_init)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        b_c = iy;
        for (k = 0; k < 16; k++) {
          ix = j + k * 16;
          iy = b_c + k * 16;
          smax = (*gpu_Pk_init)[ix];
          (*gpu_Pk_init)[ix] = (*gpu_Pk_init)[iy];
          (*gpu_Pk_init)[iy] = smax;
        }
      }

      i1 = (c - j) + 2;
      for (i = 0; i <= i1 - jp1j; i++) {
        iy = (c + i) + 1;
        (*gpu_Pk_init)[iy] /= (*gpu_Pk_init)[c];
      }
    }

    b_c = 14 - j;
    iy = c + 18;
    jy = c + 16;
    for (ia = 0; ia <= b_c; ia++) {
      smax = (*gpu_Pk_init)[jy];
      if ((*gpu_Pk_init)[jy] != 0.0) {
        ix = c;
        i1 = iy - 14;
        i2 = iy - j;
        for (jp1j = 0; jp1j <= i2 - i1; jp1j++) {
          i = (iy + jp1j) - 1;
          (*gpu_Pk_init)[i] += (*gpu_Pk_init)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 16;
      iy += 16;
    }
  }

  d_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_p);
  syncIsDirty = true;
  for (k = 0; k < 15; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*gpu_ipiv)[k] > k + 1) {
      iy = (*b_gpu_p)[(*gpu_ipiv)[k] - 1];
      (*b_gpu_p)[(*gpu_ipiv)[k] - 1] = (*b_gpu_p)[k];
      (*b_gpu_p)[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 16; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    b_i = (*b_gpu_p)[k];
    (*gpu_Fk_inv)[k + (((*b_gpu_p)[k] - 1) << 4)] = 1.0;
    for (j = 0; j <= 15 - k; j++) {
      ia = k + j;
      if ((*gpu_Fk_inv)[ia + ((b_i - 1) << 4)] != 0.0) {
        for (i = 0; i <= 14 - ia; i++) {
          iy = (ia + i) + 1;
          (*gpu_Fk_inv)[iy + ((b_i - 1) << 4)] -= (*gpu_Fk_inv)[ia + ((b_i - 1) <<
            4)] * (*gpu_Pk_init)[iy + (ia << 4)];
        }
      }
    }
  }

  for (j = 0; j < 16; j++) {
    jp1j = (j << 4) - 1;
    for (k = 0; k < 16; k++) {
      iy = 16 - k;
      jy = (15 - k) << 4;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Fk_inv)[(jp1j - k) + 16] != 0.0) {
        (*gpu_Fk_inv)[(jp1j - k) + 16] /= (*gpu_Pk_init)[(jy - k) + 15];
        for (i = 0; i <= iy - 2; i++) {
          (*gpu_Fk_inv)[(i + jp1j) + 1] -= (*gpu_Fk_inv)[(jp1j - k) + 16] *
            (*gpu_Pk_init)[i + jy];
        }
      }
    }
  }

  // 4*4
  //  Noise matrix
  if (syncIsDirty) {
    cudaDeviceSynchronize();
  }

  (*gpu_Gk)[0] = T * T / 2.0;
  (*gpu_Gk)[16] = 0.0;
  (*gpu_Gk)[32] = T * T / 2.0;
  (*gpu_Gk)[48] = 0.0;
  (*gpu_Gk)[64] = T * T / 2.0;
  (*gpu_Gk)[80] = 0.0;
  (*gpu_Gk)[96] = T * T / 2.0;
  (*gpu_Gk)[112] = 0.0;
  (*gpu_Gk)[1] = T;
  (*gpu_Gk)[17] = 0.0;
  (*gpu_Gk)[33] = T;
  (*gpu_Gk)[49] = 0.0;
  (*gpu_Gk)[65] = T * T / 2.0;
  (*gpu_Gk)[81] = 0.0;
  (*gpu_Gk)[97] = T * T / 2.0;
  (*gpu_Gk)[113] = 0.0;
  (*gpu_Gk)[2] = 0.0;
  (*gpu_Gk)[18] = T * T / 2.0;
  (*gpu_Gk)[34] = 0.0;
  (*gpu_Gk)[50] = T * T / 2.0;
  (*gpu_Gk)[66] = T * T / 2.0;
  (*gpu_Gk)[82] = 0.0;
  (*gpu_Gk)[98] = T * T / 2.0;
  (*gpu_Gk)[114] = 0.0;
  (*gpu_Gk)[3] = 0.0;
  (*gpu_Gk)[19] = T;
  (*gpu_Gk)[35] = 0.0;
  (*gpu_Gk)[51] = T;
  (*gpu_Gk)[67] = 0.0;
  (*gpu_Gk)[83] = T;
  (*gpu_Gk)[99] = 0.0;
  (*gpu_Gk)[115] = T;
  (*gpu_Gk)[4] = T * T / 2.0;
  (*gpu_Gk)[20] = 0.0;
  (*gpu_Gk)[36] = T * T / 2.0;
  (*gpu_Gk)[52] = 0.0;
  (*gpu_Gk)[68] = 0.0;
  (*gpu_Gk)[84] = T;
  (*gpu_Gk)[100] = 0.0;
  (*gpu_Gk)[116] = T;
  (*gpu_Gk)[5] = T;
  (*gpu_Gk)[21] = 0.0;
  (*gpu_Gk)[37] = T;
  (*gpu_Gk)[53] = 0.0;
  (*gpu_Gk)[69] = 0.0;
  (*gpu_Gk)[85] = T;
  (*gpu_Gk)[101] = 0.0;
  (*gpu_Gk)[117] = T;
  (*gpu_Gk)[6] = 0.0;
  (*gpu_Gk)[22] = T * T / 2.0;
  (*gpu_Gk)[38] = 0.0;
  (*gpu_Gk)[54] = T * T / 2.0;
  (*gpu_Gk)[70] = 0.0;
  (*gpu_Gk)[86] = T;
  (*gpu_Gk)[102] = 0.0;
  (*gpu_Gk)[118] = T;
  (*gpu_Gk)[7] = 0.0;
  (*gpu_Gk)[23] = T;
  (*gpu_Gk)[39] = 0.0;
  (*gpu_Gk)[55] = T;
  (*gpu_Gk)[71] = 0.0;
  (*gpu_Gk)[87] = T;
  (*gpu_Gk)[103] = 0.0;
  (*gpu_Gk)[119] = T;
  (*gpu_Gk)[8] = T * T / 2.0;
  (*gpu_Gk)[24] = 0.0;
  (*gpu_Gk)[40] = T * T / 2.0;
  (*gpu_Gk)[56] = 0.0;
  (*gpu_Gk)[72] = 0.0;
  (*gpu_Gk)[88] = T;
  (*gpu_Gk)[104] = 0.0;
  (*gpu_Gk)[120] = T;
  (*gpu_Gk)[9] = T;
  (*gpu_Gk)[25] = 0.0;
  (*gpu_Gk)[41] = T;
  (*gpu_Gk)[57] = 0.0;
  (*gpu_Gk)[73] = 0.0;
  (*gpu_Gk)[89] = T;
  (*gpu_Gk)[105] = 0.0;
  (*gpu_Gk)[121] = T;
  (*gpu_Gk)[10] = 0.0;
  (*gpu_Gk)[26] = T * T / 2.0;
  (*gpu_Gk)[42] = 0.0;
  (*gpu_Gk)[58] = T * T / 2.0;
  (*gpu_Gk)[74] = 0.0;
  (*gpu_Gk)[90] = T;
  (*gpu_Gk)[106] = 0.0;
  (*gpu_Gk)[122] = T;
  (*gpu_Gk)[11] = 0.0;
  (*gpu_Gk)[27] = T;
  (*gpu_Gk)[43] = 0.0;
  (*gpu_Gk)[59] = T;
  (*gpu_Gk)[75] = 0.0;
  (*gpu_Gk)[91] = T;
  (*gpu_Gk)[107] = 0.0;
  (*gpu_Gk)[123] = T;
  (*gpu_Gk)[12] = T * T / 2.0;
  (*gpu_Gk)[28] = 0.0;
  (*gpu_Gk)[44] = T * T / 2.0;
  (*gpu_Gk)[60] = 0.0;
  (*gpu_Gk)[76] = 0.0;
  (*gpu_Gk)[92] = T;
  (*gpu_Gk)[108] = 0.0;
  (*gpu_Gk)[124] = T;
  (*gpu_Gk)[13] = T;
  (*gpu_Gk)[29] = 0.0;
  (*gpu_Gk)[45] = T;
  (*gpu_Gk)[61] = 0.0;
  (*gpu_Gk)[77] = 0.0;
  (*gpu_Gk)[93] = T;
  (*gpu_Gk)[109] = 0.0;
  (*gpu_Gk)[125] = T;
  (*gpu_Gk)[14] = 0.0;
  (*gpu_Gk)[30] = T * T / 2.0;
  (*gpu_Gk)[46] = 0.0;
  (*gpu_Gk)[62] = T * T / 2.0;
  (*gpu_Gk)[78] = 0.0;
  (*gpu_Gk)[94] = T;
  (*gpu_Gk)[110] = 0.0;
  (*gpu_Gk)[126] = T;
  (*gpu_Gk)[15] = 0.0;
  (*gpu_Gk)[31] = T;
  (*gpu_Gk)[47] = 0.0;
  (*gpu_Gk)[63] = T;
  (*gpu_Gk)[79] = 0.0;
  (*gpu_Gk)[95] = T;
  (*gpu_Gk)[111] = 0.0;
  (*gpu_Gk)[127] = T;

  //  Prediction step alternative 2. Gives easier matrix to invert
  e_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
    (*gpu_Fk_inv, *gpu_Pk_init);
  syncIsDirty = true;
  for (c = 0; c < 16; c++) {
    iy = c << 4;
    i1 = iy - 15;
    for (ic = 0; ic <= iy - i1; ic++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_C)[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 16; c++) {
    jp1j = c * 16 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 15;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_C)[k] += (*b_gpu_Ik)[b_c - 1] * (*gpu_Pk_init)[ia];
      }

      jy += 16;
    }

    iy = c << 4;
    i1 = iy - 15;
    for (ic = 0; ic <= iy - i1; ic++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_Ih)[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 16; c++) {
    jp1j = c * 16 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 15;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_Ih)[k] += (*gpu_Fk_inv)[b_c - 1] * (*gpu_C)[ia];
      }

      jy += 16;
    }
  }

  // 4*4
  for (c = 0; c < 8; c++) {
    iy = c << 4;
    i1 = iy - 15;
    for (ic = 0; ic <= iy - i1; ic++) {
      C[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 8; c++) {
    jp1j = c * 16 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 15;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        C[k] += (*gpu_Gk)[b_c - 1] * (*gpu_Ih)[ia];
      }

      jy += 16;
    }
  }

  f_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*gpu_Gk, *gpu_A);
  syncIsDirty = true;
  for (c = 0; c < 16; c++) {
    iy = c << 3;
    i1 = iy - 7;
    for (ic = 0; ic <= iy - i1; ic++) {
      b_C[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 16; c++) {
    jp1j = c * 16 + 1;
    iy = c << 3;
    jy = -1;
    i1 = jp1j - 15;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 6;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        b_C[k] += (*gpu_Ih)[b_c - 1] * (*gpu_A)[ia];
      }

      jy += 8;
    }
  }

  for (c = 0; c < 8; c++) {
    iy = c << 3;
    i1 = iy - 7;
    for (ic = 0; ic <= iy - i1; ic++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_Rw)[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 8; c++) {
    jp1j = c * 16 + 1;
    iy = c << 3;
    jy = -1;
    i1 = jp1j - 15;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 6;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_Rw)[k] += (*gpu_Gk)[b_c - 1] * b_C[ia];
      }

      jy += 8;
    }
  }

  g_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
    (*gpu_Rw_inv, *gpu_Rw, *gpu_y);
  h_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 7; j++) {
    c = j * 9;
    jp1j = c - 4;
    b_c = 6 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_Rw)[c]);
    for (k = 0; k <= b_c; k++) {
      ix++;
      s = std::abs((*gpu_Rw)[ix]);
      if (s > smax) {
        iy = k + 1;
        smax = s;
      }
    }

    if ((*gpu_Rw)[c + iy] != 0.0) {
      if (iy != 0) {
        (*b_gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        b_c = iy;
        for (k = 0; k < 8; k++) {
          ix = j + k * 8;
          iy = b_c + k * 8;
          smax = (*gpu_Rw)[ix];
          (*gpu_Rw)[ix] = (*gpu_Rw)[iy];
          (*gpu_Rw)[iy] = smax;
        }
      }

      i1 = (c - j) + 2;
      for (i = 0; i <= i1 - jp1j; i++) {
        iy = (c + i) + 1;
        (*gpu_Rw)[iy] /= (*gpu_Rw)[c];
      }
    }

    b_c = 6 - j;
    iy = c + 10;
    jy = c + 8;
    for (ia = 0; ia <= b_c; ia++) {
      smax = (*gpu_Rw)[jy];
      if ((*gpu_Rw)[jy] != 0.0) {
        ix = c;
        i1 = iy - 6;
        i2 = iy - j;
        for (jp1j = 0; jp1j <= i2 - i1; jp1j++) {
          i = (iy + jp1j) - 1;
          (*gpu_Rw)[i] += (*gpu_Rw)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 8;
      iy += 8;
    }
  }

  i_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_p);
  syncIsDirty = true;
  for (k = 0; k < 7; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*b_gpu_ipiv)[k] > k + 1) {
      iy = (*gpu_p)[(*b_gpu_ipiv)[k] - 1];
      (*gpu_p)[(*b_gpu_ipiv)[k] - 1] = (*gpu_p)[k];
      (*gpu_p)[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 8; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    b_i = (*gpu_p)[k];
    (*gpu_y)[k + (((*gpu_p)[k] - 1) << 3)] = 1.0;
    for (j = 0; j <= 7 - k; j++) {
      ia = k + j;
      if ((*gpu_y)[ia + ((b_i - 1) << 3)] != 0.0) {
        for (i = 0; i <= 6 - ia; i++) {
          iy = (ia + i) + 1;
          (*gpu_y)[iy + ((b_i - 1) << 3)] -= (*gpu_y)[ia + ((b_i - 1) << 3)] * (*
            gpu_Rw)[iy + (ia << 3)];
        }
      }
    }
  }

  for (j = 0; j < 8; j++) {
    jp1j = (j << 3) - 1;
    for (k = 0; k < 8; k++) {
      iy = 8 - k;
      jy = (7 - k) << 3;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_y)[(jp1j - k) + 8] != 0.0) {
        (*gpu_y)[(jp1j - k) + 8] /= (*gpu_Rw)[(jy - k) + 7];
        for (i = 0; i <= iy - 2; i++) {
          (*gpu_y)[(i + jp1j) + 1] -= (*gpu_y)[(jp1j - k) + 8] * (*gpu_Rw)[i +
            jy];
        }
      }
    }

    c = j << 4;
    i1 = c - 15;
    for (ic = 0; ic <= c - i1; ic++) {
      c_C[c + ic] = 0.0;
    }
  }

  for (c = 0; c < 8; c++) {
    jp1j = c * 8 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 7;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        c_C[k] += (*gpu_y)[b_c - 1] * C[ia];
      }

      jy += 16;
    }
  }

  j_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
    (*gpu_Gk, *gpu_A);
  syncIsDirty = true;
  for (c = 0; c < 16; c++) {
    iy = c << 4;
    i1 = iy - 15;
    for (ic = 0; ic <= iy - i1; ic++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_Pk_init)[iy + ic] = 0.0;
    }
  }

  // 4*4
  for (c = 0; c < 16; c++) {
    jp1j = c * 8 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 7;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_Pk_init)[k] += (*gpu_A)[b_c - 1] * c_C[ia];
      }

      jy += 16;
    }

    iy = c << 4;
    i1 = iy - 15;
    for (ic = 0; ic <= iy - i1; ic++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*gpu_C)[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 16; c++) {
    jp1j = c * 16 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 15;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*gpu_C)[k] += (*gpu_Ih)[b_c - 1] * (*gpu_Pk_init)[ia];
      }

      jy += 16;
    }
  }

  // 4*4
  if (syncIsDirty) {
    cudaDeviceSynchronize();
  }

  (*gpu_ih)[0] = initialization_vec[2];
  (*gpu_ih)[1] = 0.0;
  (*gpu_ih)[2] = 0.0;
  (*gpu_ih)[3] = 0.0;
  (*gpu_ih)[4] = initialization_vec[2];
  (*gpu_ih)[5] = 0.0;
  (*gpu_ih)[6] = 0.0;
  (*gpu_ih)[7] = 0.0;
  (*gpu_ih)[8] = initialization_vec[2];
  (*gpu_ih)[9] = 0.0;
  (*gpu_ih)[10] = 0.0;
  (*gpu_ih)[11] = 0.0;
  (*gpu_ih)[12] = initialization_vec[2];
  k_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih);
  l_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ih, *b_gpu_Ik, *c_gpu_Ik);
  m_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*c_gpu_Ik, *gpu_Fk_inv, *gpu_ih);

  // 4*1
  n_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_Pk_init, *gpu_ih, *gpu_op);

  // 4*1
  o_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(256U, 1U, 1U)>>>
    (*gpu_C, *gpu_Ih, *gpu_Pk_init);
  p_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_ipiv);
  syncIsDirty = true;
  for (j = 0; j < 15; j++) {
    c = j * 17;
    jp1j = c - 12;
    b_c = 14 - j;
    iy = 0;
    ix = c;
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    smax = std::abs((*gpu_Ih)[c]);
    for (k = 0; k <= b_c; k++) {
      ix++;
      s = std::abs((*gpu_Ih)[ix]);
      if (s > smax) {
        iy = k + 1;
        smax = s;
      }
    }

    if ((*gpu_Ih)[c + iy] != 0.0) {
      if (iy != 0) {
        (*gpu_ipiv)[j] = static_cast<signed char>((j + iy) + 1);
        iy += j;
        b_c = iy;
        for (k = 0; k < 16; k++) {
          ix = j + k * 16;
          iy = b_c + k * 16;
          smax = (*gpu_Ih)[ix];
          (*gpu_Ih)[ix] = (*gpu_Ih)[iy];
          (*gpu_Ih)[iy] = smax;
        }
      }

      i1 = (c - j) + 2;
      for (i = 0; i <= i1 - jp1j; i++) {
        iy = (c + i) + 1;
        (*gpu_Ih)[iy] /= (*gpu_Ih)[c];
      }
    }

    b_c = 14 - j;
    iy = c + 18;
    jy = c + 16;
    for (ia = 0; ia <= b_c; ia++) {
      smax = (*gpu_Ih)[jy];
      if ((*gpu_Ih)[jy] != 0.0) {
        ix = c;
        i1 = iy - 14;
        i2 = iy - j;
        for (jp1j = 0; jp1j <= i2 - i1; jp1j++) {
          i = (iy + jp1j) - 1;
          (*gpu_Ih)[i] += (*gpu_Ih)[ix + 1] * -smax;
          ix++;
        }
      }

      jy += 16;
      iy += 16;
    }
  }

  q_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_p);
  syncIsDirty = true;
  for (k = 0; k < 15; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    if ((*gpu_ipiv)[k] > k + 1) {
      iy = (*b_gpu_p)[(*gpu_ipiv)[k] - 1];
      (*b_gpu_p)[(*gpu_ipiv)[k] - 1] = (*b_gpu_p)[k];
      (*b_gpu_p)[k] = static_cast<signed char>(iy);
    }
  }

  for (k = 0; k < 16; k++) {
    if (syncIsDirty) {
      cudaDeviceSynchronize();
      syncIsDirty = false;
    }

    b_i = (*b_gpu_p)[k];
    (*gpu_Pk_init)[k + (((*b_gpu_p)[k] - 1) << 4)] = 1.0;
    for (j = 0; j <= 15 - k; j++) {
      ia = k + j;
      if ((*gpu_Pk_init)[ia + ((b_i - 1) << 4)] != 0.0) {
        for (i = 0; i <= 14 - ia; i++) {
          iy = (ia + i) + 1;
          (*gpu_Pk_init)[iy + ((b_i - 1) << 4)] -= (*gpu_Pk_init)[ia + ((b_i - 1)
            << 4)] * (*gpu_Ih)[iy + (ia << 4)];
        }
      }
    }
  }

  for (j = 0; j < 16; j++) {
    jp1j = (j << 4) - 1;
    for (k = 0; k < 16; k++) {
      iy = 16 - k;
      jy = (15 - k) << 4;
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      if ((*gpu_Pk_init)[(jp1j - k) + 16] != 0.0) {
        (*gpu_Pk_init)[(jp1j - k) + 16] /= (*gpu_Ih)[(jy - k) + 15];
        for (i = 0; i <= iy - 2; i++) {
          (*gpu_Pk_init)[(i + jp1j) + 1] -= (*gpu_Pk_init)[(jp1j - k) + 16] *
            (*gpu_Ih)[i + jy];
        }
      }
    }
  }

  // 4*4
  r_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_op, *gpu_Pk_init, *gpu_xk_m_out);

  // 4*1
  // ---- Measurement step --------------------------
  //  Extract front and rear track widths
  //  drive axle 1
  //  drive axle 2
  smax = L_axlePos[0] + L_geometricWheelbase;

  // 1*1
  s_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_B_usedMeas_vec, *gpu_Re, *gpu_Re_inv);

  // 1*13
  t_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(192U, 1U, 1U)>>>
    (*b_gpu_Re_inv);
  u_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
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
  (*gpu_H_linear)[105] = 0.0;
  (*gpu_H_linear)[118] = 0.0;
  (*gpu_H_linear)[131] = 0.0;
  (*gpu_H_linear)[144] = L_imuToRear;
  (*gpu_H_linear)[157] = 0.0;
  (*gpu_H_linear)[170] = 0.0;
  (*gpu_H_linear)[183] = 0.0;
  (*gpu_H_linear)[196] = L_imuToRear;
  (*gpu_H_linear)[6] = 1.0;
  (*gpu_H_linear)[19] = 0.0;
  (*gpu_H_linear)[32] = -L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[45] = 0.0;
  (*gpu_H_linear)[58] = 1.0;
  (*gpu_H_linear)[71] = 0.0;
  (*gpu_H_linear)[84] = -L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[97] = 0.0;
  (*gpu_H_linear)[110] = 1.0;
  (*gpu_H_linear)[123] = 0.0;
  (*gpu_H_linear)[136] = 0.0;
  (*gpu_H_linear)[149] = 0.0;
  (*gpu_H_linear)[162] = 1.0;
  (*gpu_H_linear)[175] = 0.0;
  (*gpu_H_linear)[188] = 0.0;
  (*gpu_H_linear)[201] = 0.0;
  (*gpu_H_linear)[7] = 1.0;
  (*gpu_H_linear)[20] = 0.0;
  (*gpu_H_linear)[33] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[46] = 0.0;
  (*gpu_H_linear)[59] = 1.0;
  (*gpu_H_linear)[72] = 0.0;
  (*gpu_H_linear)[85] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[98] = 0.0;
  (*gpu_H_linear)[111] = 1.0;
  (*gpu_H_linear)[124] = 0.0;
  (*gpu_H_linear)[137] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[150] = 0.0;
  (*gpu_H_linear)[163] = 1.0;
  (*gpu_H_linear)[176] = 0.0;
  (*gpu_H_linear)[189] = L_trackWidth[2] / 2.0;
  (*gpu_H_linear)[202] = 0.0;
  (*gpu_H_linear)[8] = 1.0;
  (*gpu_H_linear)[21] = 0.0;
  (*gpu_H_linear)[34] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[47] = 0.0;
  (*gpu_H_linear)[60] = 1.0;
  (*gpu_H_linear)[73] = 0.0;
  (*gpu_H_linear)[86] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[99] = 0.0;
  (*gpu_H_linear)[112] = 1.0;
  (*gpu_H_linear)[125] = 0.0;
  (*gpu_H_linear)[138] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[151] = 0.0;
  (*gpu_H_linear)[164] = 1.0;
  (*gpu_H_linear)[177] = 0.0;
  (*gpu_H_linear)[190] = -L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[203] = 0.0;
  (*gpu_H_linear)[9] = 1.0;
  (*gpu_H_linear)[22] = 0.0;
  (*gpu_H_linear)[35] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[48] = 0.0;
  (*gpu_H_linear)[61] = 1.0;
  (*gpu_H_linear)[74] = 0.0;
  (*gpu_H_linear)[87] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[100] = 0.0;
  (*gpu_H_linear)[113] = 1.0;
  (*gpu_H_linear)[126] = 0.0;
  (*gpu_H_linear)[139] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[152] = 0.0;
  (*gpu_H_linear)[165] = 1.0;
  (*gpu_H_linear)[178] = 0.0;
  (*gpu_H_linear)[191] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[204] = 0.0;
  (*gpu_H_linear)[10] = 1.0;
  (*gpu_H_linear)[23] = 0.0;
  (*gpu_H_linear)[36] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[49] = 0.0;
  (*gpu_H_linear)[62] = 1.0;
  (*gpu_H_linear)[75] = 0.0;
  (*gpu_H_linear)[88] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[101] = 0.0;
  (*gpu_H_linear)[114] = 1.0;
  (*gpu_H_linear)[127] = 0.0;
  (*gpu_H_linear)[140] = L_trackWidth[3] / 2.0;
  (*gpu_H_linear)[153] = 0.0;
  (*gpu_H_linear)[166] = 1.0;
  (*gpu_H_linear)[179] = 0.0;
  (*gpu_H_linear)[192] = L_trackWidth[3] / 2.0;
  v_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_H_linear);
  w_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_iv8, *gpu_iv9, *gpu_iv10, *gpu_iv11, *gpu_H_linear);

  //  Nonlinear parts
  x_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_xk_m_out, *gpu_H_linear, *gpu_hk);

  // 13*1
  // the new hk 13*1 matrix is formed from this calculation
  cudaDeviceSynchronize();
  (*gpu_hk)[4] = ((*gpu_xk_m_out)[0] - L_trackWidth[0] * (*gpu_xk_m_out)[2] /
                  2.0) * std::cos(delta) + smax * (*gpu_xk_m_out)[2] * std::sin
    (delta);
  (*gpu_hk)[5] = ((*gpu_xk_m_out)[0] + L_trackWidth[0] * (*gpu_xk_m_out)[2] /
                  2.0) * std::cos(delta) + smax * (*gpu_xk_m_out)[2] * std::sin
    (delta);
  (*gpu_H_linear)[30] = -L_trackWidth[0] * std::cos(delta) / 2.0 + smax * std::
    sin(delta);
  (*gpu_H_linear)[5] = std::cos(delta);
  (*gpu_H_linear)[31] = L_trackWidth[0] * std::cos(delta) / 2.0 + smax * std::
    sin(delta);
  y_InformationFilterUpdate_kerne<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(smax,
    delta, L_imuToRear, *gpu_xk_m_out, *gpu_H_linear, *gpu_hk);
  ab_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
    (*gpu_H_linear, *b_gpu_A);
  syncIsDirty = true;
  for (c = 0; c < 13; c++) {
    iy = c << 4;
    i1 = iy - 15;
    for (ic = 0; ic <= iy - i1; ic++) {
      if (syncIsDirty) {
        cudaDeviceSynchronize();
        syncIsDirty = false;
      }

      (*b_gpu_C)[iy + ic] = 0.0;
    }
  }

  for (c = 0; c < 13; c++) {
    jp1j = c * 13 + 1;
    iy = c << 4;
    jy = -1;
    i1 = jp1j - 12;
    for (i = 0; i <= jp1j - i1; i++) {
      b_c = jp1j + i;
      ia = jy;
      i2 = iy - 14;
      ix = iy + 1;
      for (ic = 0; ic <= ix - i2; ic++) {
        k = iy + ic;
        ia++;
        if (syncIsDirty) {
          cudaDeviceSynchronize();
          syncIsDirty = false;
        }

        (*b_gpu_C)[k] += (*b_gpu_Re_inv)[b_c - 1] * (*b_gpu_A)[ia];
      }

      jy += 16;
    }
  }

  bb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*gpu_xk_m_out, *gpu_H_linear, *gpu_hk, *gpu_y_meas, *b_gpu_y_meas);
  cb_InformationFilterUpdate_kern<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*b_gpu_y_meas, *b_gpu_C, *gpu_op);

  // Hk is always a 13*4 matrix and hk is always a 13*1 matrix
  cudaDeviceSynchronize();
  cudaMemcpy(&op[0], gpu_op, 128ULL, cudaMemcpyDeviceToHost);
  cudaMemcpy(&xk_m_out[0], gpu_xk_m_out, 128ULL, cudaMemcpyDeviceToHost);
  cudaFree(*gpu_y_meas);
  cudaFree(*gpu_iv11);
  cudaFree(*gpu_iv10);
  cudaFree(*gpu_iv9);
  cudaFree(*gpu_iv8);
  cudaFree(*gpu_Re);
  cudaFree(*gpu_B_usedMeas_vec);
  cudaFree(*gpu_iv7);
  cudaFree(*gpu_iv6);
  cudaFree(*gpu_iv5);
  cudaFree(*gpu_iv4);
  cudaFree(*gpu_iv3);
  cudaFree(*gpu_iv2);
  cudaFree(*gpu_iv1);
  cudaFree(*gpu_iv);
  cudaFree(*b_gpu_Rw);
  cudaFree(*gpu_Ik);
  cudaFree(*gpu_op);
  cudaFree(*gpu_xk_m_out);
  cudaFree(*gpu_Pk_init);
  cudaFree(*gpu_ipiv);
  cudaFree(*b_gpu_Ik);
  cudaFree(*gpu_Rw);
  cudaFree(*gpu_Rw_inv);
  cudaFree(*b_gpu_ipiv);
  cudaFree(*gpu_p);
  cudaFree(*gpu_Fk_inv);
  cudaFree(*b_gpu_p);
  cudaFree(*gpu_Gk);
  cudaFree(*gpu_A);
  cudaFree(*gpu_y);
  cudaFree(*gpu_ih);
  cudaFree(*c_gpu_Ik);
  cudaFree(*gpu_C);
  cudaFree(*gpu_Ih);
  cudaFree(*gpu_Re_inv);
  cudaFree(*b_gpu_Re_inv);
  cudaFree(*gpu_H_linear);
  cudaFree(*gpu_hk);
  cudaFree(*b_gpu_A);
  cudaFree(*b_gpu_y_meas);
  cudaFree(*b_gpu_C);
}

//
// Arguments    : void
// Return Type  : void
//
void InformationFilterUpdate_initialize()
{
  state_not_empty = false;
  eml_rand_init();
  eml_rand_mcg16807_stateful_init();
  eml_rand_shr3cong_stateful_init();
  isInitialized_InformationFilterUpdate = true;
}

//
// Arguments    : void
// Return Type  : void
//
void InformationFilterUpdate_terminate()
{
  // (no terminate code required)
  isInitialized_InformationFilterUpdate = false;
}

//
// File trailer for InformationFilterUpdate.cu
//
// [EOF]
//
