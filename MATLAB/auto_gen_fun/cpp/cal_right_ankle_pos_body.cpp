//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cal_right_ankle_pos_body.cpp
//
// Code generation for function 'cal_right_ankle_pos_body'
//

// Include files
#include "cal_right_ankle_pos_body.h"
#include <cmath>

// Function Definitions
void cal_right_ankle_pos_body(const double in1[5],
                              double right_ankle_pos_body[3])
{
  double b_right_ankle_pos_body_tmp;
  double b_t37_tmp;
  double right_ankle_pos_body_tmp;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t24;
  double t24_tmp;
  double t32;
  double t33;
  double t34;
  double t35;
  double t37;
  double t37_tmp;
  double t6;
  double t7;
  double t8;
  double t9;
  // CAL_RIGHT_ANKLE_POS_BODY
  //     RIGHT_ANKLE_POS_BODY = CAL_RIGHT_ANKLE_POS_BODY(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 9.2.
  //     26-Dec-2023 16:43:16
  t6 = std::cos(in1[0]);
  t7 = std::cos(in1[1]);
  t8 = std::cos(in1[2]);
  t9 = std::cos(in1[3]);
  t10 = std::sin(in1[0]);
  t11 = std::sin(in1[1]);
  t12 = std::sin(in1[2]);
  t13 = std::sin(in1[3]);
  t14 = t6 * t11;
  t32 = t12 * 0.25881867051728752 + t8 * 0.96592592665880117;
  t34 = t6 * 0.89253905378626919 + 0.073386872872531927;
  t35 = t6 * 0.1573787853566756 - 0.416197455873963;
  t37_tmp = t10 * t11;
  b_t37_tmp = t6 * t7;
  t37 = (t37_tmp * 0.1736482513311082 + t7 * 0.89253905378626919) +
        b_t37_tmp * 0.073386872872531927;
  t24_tmp = t7 * t10;
  t24 = t14 - t24_tmp * 0.42261797806762619;
  t33 = t8 * 0.25881867051728752 - t12 * 0.96592592665880117;
  t8 = (t37_tmp * 0.98480774002322313 + b_t37_tmp * 0.416197455873963) -
       t7 * 0.1573787853566756;
  right_ankle_pos_body_tmp = t33 * t34;
  b_right_ankle_pos_body_tmp = t32 * t8;
  t12 = t32 * t34;
  t8 *= t33;
  right_ankle_pos_body[0] =
      (((t6 * 0.0155012981125982 + t7 * 0.0021796961771899559) +
        (t11 * -0.00044695575041295863 + t14 * 0.0011820007746820549)) +
       (((((b_t37_tmp * -0.005764334763854388 -
            t24_tmp * 0.002796853981665954) -
           t37_tmp * 0.013639587199321641) -
          t12 * 0.01306) +
         right_ankle_pos_body_tmp * 0.24916) -
        b_right_ankle_pos_body_tmp * 0.24916)) +
      (((t8 * -0.01306 +
         t9 * (right_ankle_pos_body_tmp - b_right_ankle_pos_body_tmp) *
             0.2785) -
        t13 * (t12 + t8) * 0.2785) +
       0.022768080323065449);
  right_ankle_pos_body_tmp = t10 * t33;
  b_right_ankle_pos_body_tmp = t24 * t32;
  t12 = t10 * t32;
  t8 = t24 * t33;
  right_ankle_pos_body[1] =
      (((((t10 * 0.015740430829912711 + t14 * 0.01385) + b_t37_tmp * 0.00284) -
         t24_tmp * 0.0058532589962366228) +
        t37_tmp * 0.001200235057712058) -
       t12 * 0.011836381426260719) +
      (((((right_ankle_pos_body_tmp * 0.22581568117665551 +
           b_right_ankle_pos_body_tmp * 0.24916) +
          t8 * 0.01306) +
         t9 *
             (right_ankle_pos_body_tmp * 0.90630791931552235 +
              b_right_ankle_pos_body_tmp) *
             0.2785) -
        t13 * (t12 * 0.90630791931552235 - t8) * 0.2785) -
       0.082);
  right_ankle_pos_body_tmp = t33 * t35;
  b_right_ankle_pos_body_tmp = t32 * t37;
  t12 = t32 * t35;
  t8 = t33 * t37;
  right_ankle_pos_body[2] =
      (((t6 * 0.0027332982888126061 - t7 * 0.01236166589493983) +
        (t11 * 0.0025348109127530049 + t14 * 0.0002084187189579907)) +
       ((((b_t37_tmp * -0.0010164081892845671 -
           t24_tmp * 0.00049316103378034732) -
          t37_tmp * 0.0024050282809358488) -
         t12 * 0.01306) +
        right_ankle_pos_body_tmp * 0.24916)) +
      ((((b_right_ankle_pos_body_tmp * -0.24916 - t8 * 0.01306) +
         t9 * (right_ankle_pos_body_tmp - b_right_ankle_pos_body_tmp) *
             0.2785) -
        t13 * (t12 + t8) * 0.2785) -
       0.28290810159992358);
}

// End of code generation (cal_right_ankle_pos_body.cpp)
