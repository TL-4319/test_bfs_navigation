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
#ifndef NAVIGATION_SRC_GNSS_COMPASS_H_  // NOLINT
#define NAVIGATION_SRC_GNSS_COMPASS_H_

#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {
/*
* Heading from true north using moving baseline
*/

inline float GnssCompass(const Eigen::Vector3f &body_baseline,
                                   const Eigen::Vector3f &nav_baseline) {
  Eigen::Vector3f body = body_baseline;
  Eigen::Vector3f nav = nav_baseline;
  float heading_rad;
  float dot_prod = nav.dot(body);
  if (abs(dot_prod) < 0.001f){
    return heading_rad;
  } 
  Eigen::Vector3f cross_prod = body.cross(nav);
  /* Heading from true north in radian */
  heading_rad = atan2f(cross_prod[2], dot_prod);
  return heading_rad;
}

}  // namespace bfs

#endif  // NAVIGATION_SRC_GNSS_COMPASS_H_ NOLINT
