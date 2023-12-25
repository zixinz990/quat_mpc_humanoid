clear; clc;

syms t1 t2 t3 t4 t5 real

theta = [t1; t2; t3; t4; t5];

p01 = [0.003261; 0.082; -0.172278];
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

T05 = simplify(T01 * T12 * T23 * T34 * T45);
p05 = T05(1:3, 4);

left_foot_pos = simplify(p05);
left_foot_jac = simplify(jacobian(p05, [t1; t2; t3; t4; t5]));

%%
p01 = [0.003261; -0.082; -0.172278];
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

T05 = simplify(T01 * T12 * T23 * T34 * T45);
p05 = T05(1:3, 4);

right_foot_pos = simplify(p05);
right_foot_jac = simplify(jacobian(p05, [t1; t2; t3; t4; t5]));

%%
foot_pos_body = [left_foot_pos, right_foot_pos];
matlabFunction(foot_pos_body, ...
               'file', 'cal_foot_pos_body.m', ...
               'vars', {theta}, ...
               'outputs', {'foot_pos_body'});
% matlabFunction(left_foot_jac, ...
%                'file', 'cal_left_foot_jac.m', ...
%                'vars', {theta}, ...
%                'outputs', {'left_foot_jac'});
% matlabFunction(right_foot_jac, ...
%                'file', 'cal_right_foot_jac.m', ...
%                'vars', {theta}, ...
%                'outputs', {'right_foot_jac'});