clear; clc;
syms t1 t2 t3 t4 t5 real
theta = [t1; t2; t3; t4; t5];

%% LEFT FOOT
% p01 = [0.003261; 0.082; -0.172278];
p01 = [0.005546; 0.082; -0.111722];
R01 = basicRotMat("y", -0.174533) * basicRotMat("z", t1);
T01 = [R01, p01; 0, 0, 0, 1];

p12 = [-0.06435; 0; -0.07499];
R12 = basicRotMat("y", 0.436332) * basicRotMat("x", t2);
T12 = [R12, p12; 0, 0, 0, 1];

p23 = [0.08837; -0.00284; -0.01385];
R23 = basicRotMat("y", -0.261799) * basicRotMat("y", t3);
T23 = [R23, p23; 0, 0, 0, 1];

p34 = [-0.01306; 0; -0.24916];
R34 = basicRotMat("y", t4);
T34 = [R34, p34; 0, 0, 0, 1];

p45 = [0; 0; -0.2785];
R45 = basicRotMat("y", t5);
T45 = [R45, p45; 0, 0, 0, 1];

p5_toe = [0.02 + 0.17 / 4; 0; -0.04];
R5_toe = eye(3);
T5_toe = [R5_toe, p5_toe; 0, 0, 0, 1];

p5_heel = [0.02 - 0.17 / 4; 0; -0.04];
R5_heel = eye(3);
T5_heel = [R5_heel, p5_heel; 0, 0, 0, 1];

T0_toe = simplify(T01 * T12 * T23 * T34 * T45 * T5_toe);
left_toe_pos = T0_toe(1:3, 4);
left_toe_jac = simplify(jacobian(left_toe_pos, theta));

T0_heel = simplify(T01 * T12 * T23 * T34 * T45 * T5_heel);
left_heel_pos = T0_heel(1:3, 4);
left_heel_jac = simplify(jacobian(left_heel_pos, theta));

%% RIGHT FOOT
% p01 = [0.003261; -0.082; -0.172278];
p01 = [0.005546; -0.082; -0.111722];
R01 = basicRotMat("y", -0.174533) * basicRotMat("z", t1);
T01 = [R01, p01; 0, 0, 0, 1];

p12 = [-0.06435; 0; -0.07499];
R12 = basicRotMat("y", 0.436332) * basicRotMat("x", t2);
T12 = [R12, p12; 0, 0, 0, 1];

p23 = [0.08837; 0.00284; -0.01385];
R23 = basicRotMat("y", -0.261799) * basicRotMat("y", t3);
T23 = [R23, p23; 0, 0, 0, 1];

p34 = [-0.01306; 0; -0.24916];
R34 = basicRotMat("y", t4);
T34 = [R34, p34; 0, 0, 0, 1];

p45 = [0; 0; -0.2785];
R45 = basicRotMat("y", t5);
T45 = [R45, p45; 0, 0, 0, 1];

p5_toe = [0.02 + 0.17 / 4; 0; -0.04];
R5_toe = eye(3);
T5_toe = [R5_toe, p5_toe; 0, 0, 0, 1];

p5_heel = [0.02 - 0.17 / 4; 0; -0.04];
R5_heel = eye(3);
T5_heel = [R5_heel, p5_heel; 0, 0, 0, 1];

T0_toe = simplify(T01 * T12 * T23 * T34 * T45 * T5_toe);
right_toe_pos = T0_toe(1:3, 4);
right_toe_jac = simplify(jacobian(right_toe_pos, theta));

T0_heel = simplify(T01 * T12 * T23 * T34 * T45 * T5_heel);
right_heel_pos = T0_heel(1:3, 4);
right_heel_jac = simplify(jacobian(right_heel_pos, theta));

%%
matlabFunction(left_toe_pos, ...
               'file', 'auto_gen_fun/cal_left_toe_pos_body.m', ...
               'vars', {theta}, ...
               'outputs', {'left_toe_pos_body'});
matlabFunction(left_heel_pos, ...
               'file', 'auto_gen_fun/cal_left_heel_pos_body.m', ...
               'vars', {theta}, ...
               'outputs', {'left_heel_pos_body'});
matlabFunction(right_toe_pos, ...
               'file', 'auto_gen_fun/cal_right_toe_pos_body.m', ...
               'vars', {theta}, ...
               'outputs', {'right_toe_pos_body'});
matlabFunction(right_heel_pos, ...
               'file', 'auto_gen_fun/cal_right_heel_pos_body.m', ...
               'vars', {theta}, ...
               'outputs', {'right_heel_pos_body'});
matlabFunction(left_toe_jac, ...
               'file', 'auto_gen_fun/cal_left_toe_jac_body.m', ...
               'vars', {theta}, ...
               'outputs', {'left_toe_jac_body'});
matlabFunction(left_heel_jac, ...
               'file', 'auto_gen_fun/cal_left_heel_jac_body.m', ...
               'vars', {theta}, ...
               'outputs', {'left_heel_jac_body'});
matlabFunction(right_toe_jac, ...
               'file', 'auto_gen_fun/cal_right_toe_jac_body.m', ...
               'vars', {theta}, ...
               'outputs', {'right_toe_jac_body'});
matlabFunction(right_heel_jac, ...
               'file', 'auto_gen_fun/cal_right_heel_jac_body.m', ...
               'vars', {theta}, ...
               'outputs', {'right_heel_jac_body'});