function left_foot_jac = cal_left_foot_jac(in1)
%CAL_LEFT_FOOT_JAC
%    LEFT_FOOT_JAC = CAL_LEFT_FOOT_JAC(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    23-Dec-2023 19:48:15

t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t4 = in1(4,:);
t6 = cos(t1);
t7 = cos(t2);
t8 = cos(t3);
t9 = cos(t4);
t10 = sin(t1);
t11 = sin(t2);
t12 = sin(t3);
t13 = sin(t4);
t14 = t6.*t7;
t15 = t6.*t11;
t16 = t10.*t11;
t17 = t8.*2.588186705172875e-1;
t18 = t12.*2.588186705172875e-1;
t19 = t8.*9.659259266588012e-1;
t20 = t12.*9.659259266588012e-1;
t23 = t7.*t10.*9.848077400232231e-1;
t26 = t7.*t10.*1.736482513311082e-1;
t29 = t7.*t10.*4.226179780676262e-1;
t36 = t6.*8.925390537862692e-1;
t37 = t7.*8.925390537862692e-1;
t38 = t11.*8.925390537862692e-1;
t39 = t6.*1.573787853566756e-1;
t40 = t7.*1.573787853566756e-1;
t41 = t11.*1.573787853566756e-1;
t44 = t7.*t10.*4.16197455873963e-1;
t50 = t7.*t10.*7.338687287253193e-2;
t21 = -t20;
t22 = t15.*9.848077400232231e-1;
t24 = t16.*9.848077400232231e-1;
t25 = t15.*1.736482513311082e-1;
t27 = t16.*1.736482513311082e-1;
t28 = t14.*4.226179780676262e-1;
t30 = t16.*4.226179780676262e-1;
t31 = -t26;
t32 = -t29;
t42 = t14.*4.16197455873963e-1;
t43 = t15.*4.16197455873963e-1;
t45 = -t40;
t47 = -t44;
t48 = t14.*7.338687287253193e-2;
t49 = t15.*7.338687287253193e-2;
t51 = -t50;
t52 = t18+t19;
t61 = t36+7.338687287253193e-2;
t62 = t39-4.16197455873963e-1;
t33 = t14+t30;
t34 = t16+t28;
t35 = t15+t32;
t46 = -t43;
t53 = t17+t21;
t54 = t22+t47;
t55 = t10.*t52.*9.063079193155223e-1;
t58 = t25+t51;
t63 = t24+t42+t45;
t65 = t27+t37+t48;
t66 = t31+t38+t49;
t67 = t52.*t61;
t69 = t52.*t62;
t56 = t10.*t53.*9.063079193155223e-1;
t57 = -t55;
t59 = t35.*t52;
t60 = t35.*t53;
t64 = t23+t41+t46;
t68 = t53.*t61;
t70 = t53.*t62;
t76 = t52.*t63;
t77 = t53.*t63;
t78 = t52.*t65;
t80 = t53.*t65;
t71 = t56+t59;
t72 = t57+t60;
t74 = t9.*(t55-t60).*(-2.785e-1);
t79 = -t76;
t81 = -t78;
t82 = t67+t77;
t83 = t69+t80;
t73 = t13.*t71.*2.785e-1;
t84 = t68+t79;
t85 = t70+t81;
t86 = t9.*t82.*2.785e-1;
t89 = t9.*t83.*2.785e-1;
t75 = -t73;
t87 = -t86;
t88 = t13.*t84.*2.785e-1;
t91 = -t89;
t92 = t13.*t85.*2.785e-1;
t90 = -t88;
t93 = -t92;
et1 = t10.*(-1.55012981125982e-2)+t14.*2.796853981665954e-3-t15.*1.363958719932164e-2;
et2 = t16.*1.182000774682055e-3+t7.*t10.*5.764334763854388e-3;
et3 = t10.*t52.*1.165656004244868e-2-t10.*t53.*2.223850306413868e-1-t52.*t54.*2.4916e-1-t53.*t54.*1.306e-2;
et4 = t9.*(t10.*t53.*8.925390537862692e-1+t52.*t54).*(-2.785e-1)+t13.*(t10.*t52.*8.925390537862692e-1-t53.*t54).*2.785e-1;
et5 = t7.*4.469557504129586e-4-t11.*2.179696177189956e-3;
et6 = t14.*(-1.182000774682055e-3)+t15.*5.764334763854388e-3-t16.*2.796853981665954e-3;
et7 = t7.*t10.*(-1.363958719932164e-2)-t52.*t64.*2.4916e-1-t53.*t64.*1.306e-2-t9.*t52.*t64.*2.785e-1-t13.*t53.*t64.*2.785e-1;
et8 = t6.*1.574043082991271e-2-t14.*5.853258996236623e-3-t15.*1.200235057712058e-3-t16.*1.385e-2+t7.*t10.*2.84e-3-t6.*t52.*1.183638142626072e-2;
et9 = t6.*t53.*2.258156811766555e-1-t34.*t52.*2.4916e-1-t34.*t53.*1.306e-2+t9.*(t6.*t53.*9.063079193155223e-1-t34.*t52).*2.785e-1-t13.*(t6.*t52.*9.063079193155223e-1+t34.*t53).*2.785e-1;
et10 = t10.*(-2.733298288812606e-3)+t14.*4.931610337803473e-4-t15.*2.405028280935849e-3;
et11 = t16.*2.084187189579907e-4+t7.*t10.*1.016408189284567e-3;
et12 = t10.*t52.*2.055366936758183e-3-t10.*t53.*3.921249815946928e-2-t52.*t58.*2.4916e-1-t53.*t58.*1.306e-2;
et13 = t9.*(t10.*t53.*1.573787853566756e-1+t52.*t58).*(-2.785e-1)+t13.*(t10.*t52.*1.573787853566756e-1-t53.*t58).*2.785e-1;
et14 = t7.*(-2.534810912753005e-3)+t11.*1.236166589493983e-2;
et15 = t14.*(-2.084187189579907e-4)+t15.*1.016408189284567e-3-t16.*4.931610337803473e-4;
et16 = t7.*t10.*(-2.405028280935849e-3)+t52.*t66.*2.4916e-1+t53.*t66.*1.306e-2+t9.*t52.*t66.*2.785e-1+t13.*t53.*t66.*2.785e-1;
mt1 = [et1+et2+et3+et4,et8+et9,et10+et11+et12+et13,et5+et6+et7,t14.*1.385e-2+t15.*2.84e-3+t16.*5.853258996236623e-3-t7.*t10.*1.200235057712058e-3+t33.*t52.*2.4916e-1+t33.*t53.*1.306e-2+t9.*t33.*t52.*2.785e-1+t13.*t33.*t53.*2.785e-1,et14+et15+et16];
mt2 = [t67.*(-2.4916e-1)-t68.*1.306e-2+t76.*1.306e-2-t77.*2.4916e-1+t87+t90,t59.*(-1.306e-2)+t60.*2.4916e-1+t74+t75-t10.*t52.*2.258156811766555e-1-t10.*t53.*1.183638142626072e-2];
mt3 = [t69.*(-2.4916e-1)-t70.*1.306e-2+t78.*1.306e-2-t80.*2.4916e-1+t91+t93,t87+t90,t74+t75,t91+t93,0.0,0.0,0.0];
left_foot_jac = reshape([mt1,mt2,mt3],3,5);
