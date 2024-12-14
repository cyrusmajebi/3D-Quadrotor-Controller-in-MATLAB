function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

%robot params, mass, gravity and inertia
m = params.mass;
g = params.gravity;
I = params.I;

%extract des acc from des_state.acc
r_ddot_T_1 = des_state.acc(1);
r_ddot_T_2 = des_state.acc(2);
r_ddot_T_3 = des_state.acc(3);

%extract des vel from des_state.vel
r_dot_T_1 = des_state.vel(1);
r_dot_T_2 = des_state.vel(2);
r_dot_T_3 = des_state.vel(3);

%extract des pos from des_state.pos
r_T_1 = des_state.pos(1);
r_T_2 = des_state.pos(2);
r_T_3 = des_state.pos(3);


%extract current vel from state.vel
r_dot_1 = state.vel(1);
r_dot_2 = state.vel(2);
r_dot_3 = state.vel(3);

%extract current pos from state.pos
r_1 = state.pos(1);
r_2 = state.pos(2);
r_3 = state.pos(3);


%gains
kd_1 = 10;
kd_2 = 10;
kd_3 = 10;

kp_1 = 10;
kp_2 = 10;
kp_3 = 10;

kd_phi = 30;
kd_theta = 30;
kd_psi = 30;

kp_phi = 500;
kp_theta = 500;
kp_psi = 500;


%extract psi des from des_state.yaw
psi_T = des_state.yaw;

%extract current angles from state.rot
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);

%define the des velocities
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

%extract current velocities from state.omega
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

%define desired acc (3 components)
r_ddot_1_des = r_ddot_T_1 + (kd_1*(r_dot_T_1 - r_dot_1)) + (kp_1*(r_T_1 - r_1));
r_ddot_2_des = r_ddot_T_2 + (kd_2*(r_dot_T_2 - r_dot_2)) + (kp_2*(r_T_2 - r_2));
r_ddot_3_des = r_ddot_T_3 + (kd_3*(r_dot_T_3 - r_dot_3)) + (kp_3*(r_T_3 - r_3));

%define desired angles
phi_des = (1/g) * ( (r_ddot_1_des * sin(psi_T)) - (r_ddot_2_des * cos(psi_T)) );
theta_des = (1/g) * ( (r_ddot_1_des * cos(psi_T)) + (r_ddot_2_des * sin(psi_T)) );
psi_des = des_state.yaw;

%u1
u1 = m*(g + r_ddot_3_des);

%u2
u2 = [ ((kp_phi * (phi_des - phi)) + (kd_phi * (p_des - p)) ); ...
       ((kp_theta * (theta_des - theta)) + (kd_theta * (q_des - q)) ); ... 
       ((kp_psi * (psi_des - psi)) + (kd_psi * (r_des - r)) )];

% Thrust
F = u1;

% Moment
M = I * u2;

% =================== Your code ends here ===================

end
