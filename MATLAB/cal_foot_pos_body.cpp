//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cal_foot_pos_body.cpp
//
// Code generation for function 'cal_foot_pos_body'
//

// Include files
#include "cal_foot_pos_body.h"
#include <cmath>

// Function Definitions
void cal_foot_pos_body(const double in1[5], double foot_pos_body[6])
{
  double b_t26[6];
  double t10;
  double t105;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t26;
  double t27;
  double t28;
  double t29;
  double t32;
  double t34;
  double t35;
  double t37;
  double t39;
  double t43;
  double t45;
  double t46;
  double t47;
  double t49;
  double t51;
  double t52;
  double t53;
  double t55;
  double t56;
  double t57;
  double t6;
  double t62;
  double t64;
  double t65;
  double t7;
  double t72;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
  double t8;
  double t80;
  double t81;
  double t87;
  double t89;
  double t9;
  double t93;
  double t95;
  // CAL_FOOT_POS_BODY
  //     FOOT_POS_BODY = CAL_FOOT_POS_BODY(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 9.2.
  //     24-Dec-2023 19:50:34
  t6 = std::cos(in1[0]);
  t7 = std::cos(in1[1]);
  t8 = std::cos(in1[2]);
  t9 = std::cos(in1[3]);
  t10 = std::sin(in1[0]);
  t11 = std::sin(in1[1]);
  t12 = std::sin(in1[2]);
  t13 = std::sin(in1[3]);
  t14 = t6 * t11;
  t93 = t6 * t7;
  t15 = t93 * 0.00284;
  t105 = t7 * t10;
  t26 = t105 * 0.002796853981665954;
  t27 = t10 * 0.015740430829912711;
  t28 = t105 * 0.0058532589962366228;
  t95 = t10 * t11;
  t29 = t95 * 0.013639587199321641;
  t32 = t95 * 0.001200235057712058;
  t34 = t105 * 0.00049316103378034732;
  t35 = t95 * 0.0024050282809358488;
  t37 = t6 * 0.89253905378626919;
  t39 = t6 * 0.1573787853566756;
  t43 = t11 * 0.0025348109127530049;
  t45 = t7 * 0.01236166589493983;
  t49 = t6 * 0.0155012981125982;
  t51 = t93 * 0.005764334763854388;
  t52 = t11 * 0.00044695575041295863;
  t53 = t7 * 0.0021796961771899559;
  t55 = t6 * 0.0027332982888126061;
  t56 = t93 * 0.0010164081892845671;
  t16 = t14 * 0.01385;
  t46 = t14 * 0.0011820007746820549;
  t47 = t12 * 0.25881867051728752 + t8 * 0.96592592665880117;
  t57 = t14 * 0.0002084187189579907;
  t72 = (t95 * 0.1736482513311082 + t7 * 0.89253905378626919) +
        t93 * 0.073386872872531927;
  t11 = t14 - t105 * 0.42261797806762619;
  t14 = t8 * 0.25881867051728752 - t12 * 0.96592592665880117;
  t8 = t10 * t47;
  t62 = t8 * 0.011836381426260719;
  t6 = (t95 * 0.98480774002322313 + t93 * 0.416197455873963) -
       t7 * 0.1573787853566756;
  t73 = t47 * (t37 + 0.073386872872531927);
  t75 = t47 * (t39 - 0.416197455873963);
  t89 = t47 * t72;
  t105 = t10 * t14;
  t64 = t105 * 0.22581568117665551;
  t65 = t11 * t47;
  t11 *= t14;
  t74 = t14 * (t37 + 0.073386872872531927);
  t76 = t14 * (t39 - 0.416197455873963);
  t77 = t73 * 0.01306;
  t80 = t75 * 0.01306;
  t87 = t47 * t6;
  t12 = t14 * t6;
  t6 = t14 * t72;
  t37 = t89 * 0.24916;
  t39 = t65 * 0.24916;
  t47 = t11 * 0.01306;
  t72 = t74 * 0.24916;
  t81 = t76 * 0.24916;
  t7 = t13 * (t8 * 0.90630791931552235 - t11) * -0.2785;
  t93 = t87 * 0.24916;
  t95 = t12 * 0.01306;
  t10 = t6 * 0.01306;
  t8 = t9 * (t105 * 0.90630791931552235 + t65) * 0.2785;
  t105 = t13 * (t73 + t12) * 0.2785;
  t14 = t13 * (t75 + t6) * 0.2785;
  t6 = t9 * (t76 - t89) * 0.2785;
  t11 = t9 * (t74 - t87) * 0.2785;
  b_t26[0] =
      ((((((((((((t26 - t29) - t46) + t49) + t52) + t53) - t51) + t72) - t77) -
          t93) -
         t95) -
        t105) +
       t11) +
      0.022768080323065449;
  b_t26[1] =
      ((((((((((-t15 + t16) + t27) - t28) - t32) - t62) + t64) + t39) + t47) +
        t8) +
       t7) +
      0.082;
  b_t26[2] =
      ((((((((((((t34 - t35) - t43) - t45) + t55) - t57) - t56) + t81) - t80) -
          t37) -
         t10) -
        t14) +
       t6) -
      0.28290810159992358;
  b_t26[3] =
      ((((((((((((-t26 - t29) + t46) + t49) - t52) + t53) - t51) + t72) - t77) -
          t93) -
         t95) -
        t105) +
       t11) +
      0.022768080323065449;
  b_t26[4] =
      ((((((((((t15 + t16) + t27) - t28) + t32) - t62) + t64) + t39) + t47) +
        t8) +
       t7) -
      0.082;
  b_t26[5] =
      ((((((((((((-t34 - t35) + t43) - t45) + t55) + t57) - t56) + t81) - t80) -
          t37) -
         t10) -
        t14) +
       t6) -
      0.28290810159992358;
  for (int i{0}; i < 6; i++) {
    foot_pos_body[i] = b_t26[i];
  }
}

// End of code generation (cal_foot_pos_body.cpp)