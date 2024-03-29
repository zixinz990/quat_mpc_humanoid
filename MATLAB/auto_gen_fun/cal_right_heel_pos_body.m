function right_heel_pos_body = cal_right_heel_pos_body(in1)
%CAL_RIGHT_HEEL_POS_BODY
%    RIGHT_HEEL_POS_BODY = CAL_RIGHT_HEEL_POS_BODY(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    02-Mar-2024 23:21:07

t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t4 = in1(4,:);
t5 = in1(5,:);
t7 = cos(t1);
t8 = cos(t2);
t9 = cos(t3);
t10 = cos(t4);
t11 = cos(t5);
t12 = sin(t1);
t13 = sin(t2);
t14 = sin(t3);
t15 = sin(t4);
t16 = sin(t5);
t17 = t7.*t13;
t18 = t9.*2.588186705172875e-1;
t19 = t14.*2.588186705172875e-1;
t20 = t9.*9.659259266588012e-1;
t21 = t14.*9.659259266588012e-1;
t23 = t12.*t13.*9.848077400232231e-1;
t24 = t12.*t13.*1.736482513311082e-1;
t25 = t8.*t12.*4.226179780676262e-1;
t28 = t7.*8.925390537862692e-1;
t29 = t8.*8.925390537862692e-1;
t30 = t7.*1.573787853566756e-1;
t31 = t8.*1.573787853566756e-1;
t32 = t7.*t8.*4.16197455873963e-1;
t34 = t7.*t8.*7.338687287253193e-2;
t22 = -t21;
t26 = -t25;
t33 = -t31;
t35 = t19+t20;
t42 = t28+7.338687287253193e-2;
t43 = t30-4.16197455873963e-1;
t45 = t24+t29+t34;
t27 = t17+t26;
t36 = t18+t22;
t37 = t12.*t35.*9.063079193155223e-1;
t44 = t23+t32+t33;
t46 = t35.*t42;
t48 = t35.*t43;
t59 = t35.*t45;
t38 = t12.*t36.*9.063079193155223e-1;
t39 = -t37;
t40 = t27.*t35;
t41 = t27.*t36;
t47 = t36.*t42;
t49 = t36.*t43;
t57 = t35.*t44;
t58 = t36.*t44;
t61 = t36.*t45;
t62 = -t59;
t50 = t38+t40;
t51 = t39+t41;
t54 = -t10.*(t37-t41);
t55 = -t15.*(t37-t41);
t60 = -t57;
t63 = t46+t58;
t64 = t48+t61;
t68 = t49+t62;
t52 = t10.*t50;
t53 = t15.*t50;
t65 = t47+t60;
t66 = t10.*t63;
t67 = t15.*t63;
t72 = t10.*t64;
t73 = t15.*t64;
t74 = t10.*t68;
t75 = t15.*t68;
t56 = -t53;
t69 = t10.*t65;
t70 = t15.*t65;
t71 = -t67;
t76 = -t73;
t77 = t52+t55;
t81 = t72+t75;
t78 = t54+t56;
t79 = t66+t70;
t80 = t69+t71;
t82 = t74+t76;
et1 = t7.*1.55012981125982e-2+t8.*2.179696177189956e-3;
et2 = t13.*(-4.469557504129586e-4)+t17.*1.182000774682055e-3-t46.*1.306e-2+t47.*2.4916e-1-t57.*2.4916e-1-t58.*1.306e-2;
et3 = t67.*(-2.785e-1)+t69.*2.785e-1-t7.*t8.*5.764334763854388e-3-t8.*t12.*2.796853981665954e-3-t12.*t13.*1.363958719932164e-2-t11.*t79.*(9.0./4.0e+2)-(t16.*t79)./2.5e+1;
et4 = t11.*(t67-t69).*(-1.0./2.5e+1)+t16.*(t67-t69).*(9.0./4.0e+2)+2.505308032306545e-2;
et5 = t12.*1.574043082991271e-2+t17.*1.385e-2+t40.*2.4916e-1+t41.*1.306e-2+t52.*2.785e-1-t11.*(t53+t10.*(t37-t41)).*(9.0./4.0e+2)-(t16.*(t53+t10.*(t37-t41)))./2.5e+1+t7.*t8.*2.84e-3-t8.*t12.*5.853258996236623e-3;
et6 = t12.*t13.*1.200235057712058e-3-t12.*t35.*1.183638142626072e-2+t12.*t36.*2.258156811766555e-1+(t11.*t77)./2.5e+1-t16.*t77.*(9.0./4.0e+2)-t15.*(t37-t41).*2.785e-1-4.1e+1./5.0e+2;
et7 = t7.*2.733298288812606e-3-t8.*1.236166589493983e-2;
et8 = t13.*2.534810912753005e-3+t17.*2.084187189579907e-4-t48.*1.306e-2+t49.*2.4916e-1-t59.*2.4916e-1-t61.*1.306e-2;
et9 = t73.*(-2.785e-1)+t74.*2.785e-1-t7.*t8.*1.016408189284567e-3-t8.*t12.*4.931610337803473e-4-t12.*t13.*2.405028280935849e-3-t11.*t81.*(9.0./4.0e+2);
et10 = t16.*t81.*(-1.0./2.5e+1)-(t11.*(t73-t74))./2.5e+1+t16.*(t73-t74).*(9.0./4.0e+2)-2.223521015999236e-1;
right_heel_pos_body = [et1+et2+et3+et4;et5+et6;et7+et8+et9+et10];
