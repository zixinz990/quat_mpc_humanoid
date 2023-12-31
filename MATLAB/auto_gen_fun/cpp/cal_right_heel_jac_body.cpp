//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// cal_right_heel_jac_body.cpp
//
// Code generation for function 'cal_right_heel_jac_body'
//

// Include files
#include "cal_right_heel_jac_body.h"
#include <cmath>

// Function Definitions
void cal_right_heel_jac_body(const double in1[5],
                             double right_heel_jac_body[15])
{
  double t10;
  double t101;
  double t103;
  double t104;
  double t105;
  double t106;
  double t107;
  double t11;
  double t118;
  double t119;
  double t12;
  double t120;
  double t121;
  double t122;
  double t123;
  double t124;
  double t125;
  double t126;
  double t127;
  double t128;
  double t129_tmp;
  double t13;
  double t130_tmp;
  double t131_tmp;
  double t134;
  double t137;
  double t14;
  double t143;
  double t144;
  double t146;
  double t147;
  double t148;
  double t15;
  double t150;
  double t152;
  double t154;
  double t159;
  double t16;
  double t161;
  double t171;
  double t18;
  double t180;
  double t183;
  double t186;
  double t187;
  double t19;
  double t190;
  double t192;
  double t194;
  double t195;
  double t198;
  double t20;
  double t201;
  double t21;
  double t211;
  double t215;
  double t23;
  double t24;
  double t26;
  double t68;
  double t7;
  double t70;
  double t71;
  double t71_tmp;
  double t72;
  double t72_tmp;
  double t73;
  double t74;
  double t77;
  double t8;
  double t9;
  double t90;
  double t91;
  double t92;
  double t93;
  double t93_tmp;
  double t94;
  double t97_tmp;
  // CAL_RIGHT_HEEL_JAC_BODY
  //     RIGHT_HEEL_JAC_BODY = CAL_RIGHT_HEEL_JAC_BODY(IN1)
  //     This function was generated by the Symbolic Math Toolbox version 9.2.
  //     26-Dec-2023 16:46:15
  t7 = std::cos(in1[0]);
  t8 = std::cos(in1[1]);
  t9 = std::cos(in1[2]);
  t10 = std::cos(in1[3]);
  t11 = std::cos(in1[4]);
  t12 = std::sin(in1[0]);
  t13 = std::sin(in1[1]);
  t14 = std::sin(in1[2]);
  t15 = std::sin(in1[3]);
  t16 = std::sin(in1[4]);
  t18 = in1[2] + in1[3];
  t19 = std::cos(in1[0] + in1[1]);
  t20 = std::cos(t18);
  t21 = std::sin(t18);
  t23 = t7 * t13;
  t24 = t12 * t13;
  t74 = t7 * 0.89253905378626919;
  t77 = t7 * 0.1573787853566756;
  t26 = std::cos(in1[0] - in1[1]);
  t71_tmp = t7 * t8;
  t71 = t24 + t71_tmp * 0.42261797806762619;
  t90 = t14 * 0.25881867051728752 + t9 * 0.96592592665880117;
  t68 = t21 * 1.165615668098183E+15 + t20 * 4.350143643368078E+15;
  t72_tmp = t8 * t12;
  t72 = t23 - t72_tmp * 0.42261797806762619;
  t91 = t9 * 0.25881867051728752 - t14 * 0.96592592665880117;
  t119 = t9 * t15;
  t93_tmp = t10 * t14;
  t9 *= t10;
  t18 = t14 * t15;
  t93 = ((t18 * 1.165615668098183E+15 + t119 * 4.350143643368078E+15) +
         t93_tmp * 4.350143643368078E+15) -
        t9 * 1.165615668098183E+15;
  t94 = ((t119 * 1.165615668098183E+15 + t93_tmp * 1.165615668098183E+15) +
         t9 * 4.350143643368078E+15) -
        t18 * 4.350143643368078E+15;
  t18 = t23 * 0.98480774002322313 - t72_tmp * 0.416197455873963;
  t97_tmp = t12 * t90;
  t9 = t23 * 0.1736482513311082 - t72_tmp * 0.073386872872531927;
  t103 = (t13 * 5.792909656367617E+32 + t23 * 4.7630803684217777E+31) -
         t72_tmp * 1.127041587345724E+32;
  t104 = t71 * t90;
  t14 = (t24 * 0.98480774002322313 + t71_tmp * 0.416197455873963) -
        t8 * 0.1573787853566756;
  t119 = (t24 * 0.1736482513311082 + t8 * 0.89253905378626919) +
         t71_tmp * 0.073386872872531927;
  t120 = (-(t72_tmp * 0.1736482513311082) + t13 * 0.89253905378626919) +
         t23 * 0.073386872872531927;
  t125 = t90 * (t74 + 0.073386872872531927);
  t127 = t90 * (t77 - 0.416197455873963);
  t70 = t20 * 1.165615668098183E+15 - t21 * 4.350143643368078E+15;
  t73 = t19 * 1.040118983530025E+16 + t26 * 2.562760718366372E+16;
  t92 = t19 * 0.28869101096618688 + t26 * 0.71130898903381312;
  t101 = (t13 * 6.3840419750585418E+30 + t72_tmp * 3.9948547927995041E+31) -
         t23 * 1.6882974552066919E+31;
  t105 = t71 * t91;
  t106 = t72 * t90;
  t107 = t72 * t91;
  t118 = (t72_tmp * 0.98480774002322313 + t13 * 0.1573787853566756) -
         t23 * 0.416197455873963;
  t121 = t90 * t18;
  t122 = t91 * t18;
  t123 = t90 * t9;
  t124 = t91 * t9;
  t126 = t91 * (t74 + 0.073386872872531927);
  t128 = t91 * (t77 - 0.416197455873963);
  t146 = t90 * t14;
  t147 = t91 * t14;
  t148 = t90 * t119;
  t150 = t91 * t119;
  t129_tmp = t7 * t90;
  t18 = t129_tmp * 0.90630791931552235 + t105;
  t130_tmp = t12 * t91;
  t14 = t130_tmp * 0.90630791931552235 + t106;
  t131_tmp = t7 * t91;
  t9 = t131_tmp * 0.90630791931552235 - t104;
  t93_tmp = t97_tmp * 0.90630791931552235 - t107;
  t21 = t10 * t93_tmp;
  t144 = t21 * -0.2785;
  t152 = t130_tmp * 0.89253905378626919 + t121;
  t159 = t130_tmp * 0.1573787853566756 + t123;
  t119 = t125 + t147;
  t77 = t127 + t150;
  t134 = t15 * t18;
  t20 = t15 * t14;
  t137 = t10 * t9;
  t154 = t10 * t152;
  t161 = t10 * t159;
  t74 = t126 - t146;
  t26 = t10 * t119;
  t171 = t128 - t148;
  t71 = t10 * t77;
  t143 = t20 * 0.2785;
  t19 = t15 * t74;
  t72 = t15 * t171;
  t180 = t10 * t18 + t15 * t9;
  t9 = t10 * t14 + -t15 * t93_tmp;
  t183 = t26 * 0.2785;
  t187 = t71 * 0.2785;
  t18 = t20 + t21;
  t194 = t11 * t18 * -0.04;
  t198 = t16 * t18 * 0.0225;
  t93_tmp = t97_tmp * 0.89253905378626919 - t122;
  t21 = t154 + -t15 * t93_tmp;
  t20 = t97_tmp * 0.1573787853566756 - t124;
  t201 = t161 + -t15 * t20;
  t186 = t19 * 0.2785;
  t190 = t72 * 0.2785;
  t192 = t16 * t9 / 25.0;
  t195 = t11 * t9 * 0.0225;
  t9 = t26 + t19;
  t14 = t71 + t72;
  t18 = t15 * t119 - t10 * t74;
  t211 = t16 * t18 / 25.0;
  t215 = t11 * t18 * 0.0225;
  t18 = t15 * t77 - t10 * t171;
  t171 = t16 * t18 / 25.0;
  t74 = t11 * t18 * 0.0225;
  t77 = t11 * t9 / 25.0;
  t72 = t16 * t9 * 0.0225;
  t119 = t11 * t14 / 25.0;
  t71 = t16 * t14 * 0.0225;
  t19 = t15 * t152 + t10 * t93_tmp;
  right_heel_jac_body[0] =
      ((((t12 * -0.0155012981125982 - t23 * 0.013639587199321641) -
         t24 * 0.0011820007746820549) -
        t121 * 0.24916) +
       (((((t122 * -0.01306 - t154 * 0.2785) + t11 * t19 * 0.0225) +
          t16 * t19 / 25.0) -
         t71_tmp * 0.002796853981665954) +
        t72_tmp * 0.005764334763854388)) +
      ((((t97_tmp * 0.01165656004244868 - t130_tmp * 0.22238503064138679) -
         t11 * t21 / 25.0) +
        t16 * t21 * 0.0225) +
       t15 * t93_tmp * 0.2785);
  t19 = t134 - t137;
  right_heel_jac_body[1] =
      (((((((t7 * 0.015740430829912711 + t23 * 0.001200235057712058) -
            t24 * 0.01385) -
           t104 * 0.24916) -
          t105 * 0.01306) -
         t134 * 0.2785) +
        t137 * 0.2785) -
       t71_tmp * 0.0058532589962366228) +
      ((((((t72_tmp * -0.00284 - t129_tmp * 0.011836381426260719) +
           t131_tmp * 0.22581568117665551) -
          t11 * t180 * 0.0225) -
         t16 * t180 / 25.0) -
        t11 * t19 / 25.0) +
       t16 * t19 * 0.0225);
  t19 = t15 * t159 + t10 * t20;
  right_heel_jac_body[2] =
      (((t12 * -0.0027332982888126061 - t23 * 0.0024050282809358488) -
        t24 * 0.0002084187189579907) +
       ((((((t123 * -0.24916 - t124 * 0.01306) - t161 * 0.2785) +
           t11 * t19 * 0.0225) +
          t16 * t19 / 25.0) -
         t71_tmp * 0.00049316103378034732) +
        t72_tmp * 0.0010164081892845671)) +
      ((((t97_tmp * 0.0020553669367581831 - t130_tmp * 0.039212498159469283) -
         t11 * t201 / 25.0) +
        t16 * t201 * 0.0225) +
       t15 * t20 * 0.2785);
  t19 = t10 * t90;
  t21 = t15 * t91;
  t20 = t11 * t68;
  t26 = t11 * t70;
  t93_tmp = t16 * t68;
  t14 = t16 * t70;
  right_heel_jac_body[3] =
      (((t8 * -0.00044695575041295863 - t13 * 0.0021796961771899559) +
        ((t23 * 0.005764334763854388 + t24 * 0.002796853981665954) +
         t71_tmp * 0.0011820007746820549)) +
       ((((t72_tmp * -0.013639587199321641 - t90 * t118 * 0.24916) -
          t91 * t118 * 0.01306) -
         t20 * t101 / 4.5671926166590723E+48) -
        t26 * t101 * 1.2316099784104841E-49)) +
      (((t93_tmp * t101 * 1.2316099784104841E-49 -
         t14 * t101 / 4.5671926166590723E+48) -
        t19 * t118 * 0.2785) -
       t21 * t118 * 0.2785);
  t9 = t11 * t73;
  t18 = t16 * t73;
  right_heel_jac_body[4] =
      ((((((t23 * -0.00284 + t24 * 0.0058532589962366228) + t71_tmp * 0.01385) +
          t72_tmp * 0.001200235057712058) +
         t90 * t92 * 0.24916) +
        t91 * t92 * 0.01306) -
       t9 * t93 * 1.38666955995881E-34) +
      ((((t9 * t94 / 4.0564819207303341E+33 -
          t18 * t93 / 4.0564819207303341E+33) -
         t18 * t94 * 1.38666955995881E-34) +
        t19 * t92 * 0.2785) +
       t21 * t92 * 0.2785);
  right_heel_jac_body[5] =
      (((t8 * 0.0025348109127530049 + t13 * 0.01236166589493983) +
        ((t23 * 0.0010164081892845671 + t24 * 0.00049316103378034732) +
         t71_tmp * 0.0002084187189579907)) +
       ((((t72_tmp * -0.0024050282809358488 + t90 * t120 * 0.24916) +
          t91 * t120 * 0.01306) +
         t20 * t103 / 7.3075081866545146E+49) +
        t26 * t103 * 7.6975623650655232E-51)) +
      (((t93_tmp * t103 * -7.6975623650655232E-51 +
         t14 * t103 / 7.3075081866545146E+49) +
        t19 * t120 * 0.2785) +
       t21 * t120 * 0.2785);
  right_heel_jac_body[6] =
      ((((((((t125 * -0.24916 - t126 * 0.01306) + t146 * 0.01306) -
            t147 * 0.24916) -
           t183) -
          t186) -
         t77) +
        t72) +
       t211) +
      t215;
  right_heel_jac_body[7] =
      ((((((((t106 * -0.01306 + t107 * 0.24916) + t144) - t143) - t192) +
          t194) -
         t195) +
        t198) -
       t97_tmp * 0.22581568117665551) -
      t130_tmp * 0.011836381426260719;
  right_heel_jac_body[8] =
      ((((((((t127 * -0.24916 - t128 * 0.01306) + t148 * 0.01306) -
            t150 * 0.24916) -
           t187) -
          t190) -
         t119) +
        t71) +
       t171) +
      t74;
  right_heel_jac_body[9] = ((((-t183 - t186) - t77) + t72) + t211) + t215;
  right_heel_jac_body[10] = ((((t144 - t143) - t192) + t194) - t195) + t198;
  right_heel_jac_body[11] = ((((-t187 - t190) - t119) + t71) + t171) + t74;
  right_heel_jac_body[12] = ((-t77 + t72) + t211) + t215;
  right_heel_jac_body[13] = ((-t192 + t194) - t195) + t198;
  right_heel_jac_body[14] = ((-t119 + t71) + t171) + t74;
}

// End of code generation (cal_right_heel_jac_body.cpp)
