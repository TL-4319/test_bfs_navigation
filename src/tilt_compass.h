/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/
#ifndef NAVIGATION_SRC_TILT_COMPASS_H_  // NOLINT
#define NAVIGATION_SRC_TILT_COMPASS_H_

#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {
/*
* Yaw, pitch, and roll from 3-axis accelerometer and 
* 3-axis magnetometer measurements
*/
inline Eigen::Vector3f TiltCompass(const Eigen::Vector3f &accel,
                                   const Eigen::Vector3f &mag) {
  Eigen::Vector3f ypr;
  Eigen::Vector3f a = accel;
  Eigen::Vector3f m = mag;
  /* Normalize accel and mag */
  a.normalize();
  m.normalize();
  /* Pitch */
  ypr(1) = std::asin(a(0));
  /* Roll */
  ypr(2) = std::asin(-a(1) / std::cos(ypr(1)));
  /* Yaw */
  ypr(0) = std::atan2(m(2) * std::sin(ypr(2)) - m(1) * std::cos(ypr(2)),
           m(0) * std::cos(ypr(1)) + m(1) * std::sin(ypr(1)) * std::sin(ypr(2))
           + m(2) * std::sin(ypr(1)) * std::cos(ypr(2)));
  return ypr;
}

}  // namespace bfs

#endif  // NAVIGATION_SRC_TILT_COMPASS_H_ NOLINT
