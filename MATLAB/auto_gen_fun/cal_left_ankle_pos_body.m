function left_ankle_pos_body = cal_left_ankle_pos_body(in1)
%CAL_LEFT_ANKLE_POS_BODY
%    LEFT_ANKLE_POS_BODY = CAL_LEFT_ANKLE_POS_BODY(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    26-Dec-2023 16:43:15

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
t14 = t6.*t11;
t15 = t8.*2.588186705172875e-1;
t16 = t12.*2.588186705172875e-1;
t17 = t8.*9.659259266588012e-1;
t18 = t12.*9.659259266588012e-1;
t20 = t10.*t11.*9.848077400232231e-1;
t21 = t10.*t11.*1.736482513311082e-1;
t22 = t7.*t10.*4.226179780676262e-1;
t25 = t6.*8.925390537862692e-1;
t26 = t7.*8.925390537862692e-1;
t27 = t6.*1.573787853566756e-1;
t28 = t7.*1.573787853566756e-1;
t29 = t6.*t7.*4.16197455873963e-1;
t31 = t6.*t7.*7.338687287253193e-2;
t19 = -t18;
t23 = -t22;
t30 = -t28;
t32 = t16+t17;
t34 = t25+7.338687287253193e-2;
t35 = t27-4.16197455873963e-1;
t37 = t21+t26+t31;
t24 = t14+t23;
t33 = t15+t19;
t36 = t20+t29+t30;
et1 = t6.*1.55012981125982e-2+t7.*2.179696177189956e-3;
et2 = t11.*4.469557504129586e-4-t14.*1.182000774682055e-3;
et3 = t6.*t7.*(-5.764334763854388e-3)+t7.*t10.*2.796853981665954e-3-t10.*t11.*1.363958719932164e-2-t32.*t34.*1.306e-2+t33.*t34.*2.4916e-1-t32.*t36.*2.4916e-1;
et4 = t33.*t36.*(-1.306e-2)+t9.*(t33.*t34-t32.*t36).*2.785e-1-t13.*(t32.*t34+t33.*t36).*2.785e-1+2.276808032306545e-2;
et5 = t10.*1.574043082991271e-2+t14.*1.385e-2-t6.*t7.*2.84e-3-t7.*t10.*5.853258996236623e-3-t10.*t11.*1.200235057712058e-3-t10.*t32.*1.183638142626072e-2;
et6 = t10.*t33.*2.258156811766555e-1+t24.*t32.*2.4916e-1+t24.*t33.*1.306e-2+t9.*(t10.*t33.*9.063079193155223e-1+t24.*t32).*2.785e-1-t13.*(t10.*t32.*9.063079193155223e-1-t24.*t33).*2.785e-1+4.1e+1./5.0e+2;
et7 = t6.*2.733298288812606e-3-t7.*1.236166589493983e-2;
et8 = t11.*(-2.534810912753005e-3)-t14.*2.084187189579907e-4;
et9 = t6.*t7.*(-1.016408189284567e-3)+t7.*t10.*4.931610337803473e-4-t10.*t11.*2.405028280935849e-3-t32.*t35.*1.306e-2+t33.*t35.*2.4916e-1;
et10 = t32.*t37.*(-2.4916e-1)-t33.*t37.*1.306e-2+t9.*(t33.*t35-t32.*t37).*2.785e-1-t13.*(t32.*t35+t33.*t37).*2.785e-1-2.829081015999236e-1;
left_ankle_pos_body = [et1+et2+et3+et4;et5+et6;et7+et8+et9+et10];