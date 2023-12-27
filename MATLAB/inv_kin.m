clear all;

theta = [0; 0; -deg2rad(8); deg2rad(23.9); 0];

left_foot_pos_body = left_foot_pos(theta)
right_foot_pos_body = right_foot_pos(theta)

theta_ = [0; 0; -deg2rad(8); deg2rad(23.9); deg2rad(-15.9)];

left_toe_pos_body = left_toe_pos(theta_)
left_heel_pos_body = left_heel_pos(theta_)

right_toe_pos_body = right_toe_pos(theta_)
right_heel_pos_body = right_heel_pos(theta_)