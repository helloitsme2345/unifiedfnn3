//
// File: InformationFilterUpdate.h
//
// GPU Coder version                    : 1.5
// CUDA/C/C++ source code generated on  : 07-Sep-2020 10:58:04
//
#ifndef INFORMATIONFILTERUPDATE_H
#define INFORMATIONFILTERUPDATE_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "InformationFilterUpdate_types.h"

// Function Declarations
extern void InformationFilterUpdate(const double y_meas[13], const double
  B_usedMeas_vec[13], const double initialization_vec[4], double delta, const
  double Rw[8], const double Re[13], double L_imuToRear, double
  L_geometricWheelbase, const double L_trackWidth[5], const double L_axlePos[5],
  double T, double xk_m_out[16], double op[16]);
extern void InformationFilterUpdate_initialize();
extern void InformationFilterUpdate_terminate();

#endif

//
// File trailer for InformationFilterUpdate.h
//
// [EOF]
//
