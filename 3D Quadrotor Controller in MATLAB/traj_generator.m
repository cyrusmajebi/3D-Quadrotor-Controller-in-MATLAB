function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

function [T] = polyT(n,k,t)
%n is the polynom number of coefficients, k is the requested derivative and
%t is the actual value of t (this can be anything, not just 0 or 1).
T = zeros(n,1);
D = zeros(n,1);
%Init:
for i=1:n
D(i) = i-1;
T(i) = 1;
end
%Derivative:
for j=1:k
    for i=1:n
        T(i) = T(i) * D(i);

        if D(i) > 0
            D(i) = D(i) - 1;
        end
    end
end
%put t value
for i=1:n
T(i) = T(i) * t^D(i);
end
T = T';
end

function [coeff] = getCoeff(waypoints)
n = size(waypoints,1)-1; % number of segments P1..n
A = zeros(8*n, 8*n);
b = zeros(1,8*n);

%disp(n)

% Fill A and b matices with values using loops

% First 4 equations
% P1(0)=W1; P2(0)=W2;  P3(0)=W3; P4(0)=W4
for i=1:n
    b(1,i) = waypoints(i);
end

row = 1;
for i=1:n
    A(row, 1+(8*(i-1)):8*i) = polyT(8,0,0); 
    row = row + 1;
end
%row = 5, after first 4 equations

% Next 4 equations
% P1(1)=W2; P2(1)=W3; P3(1)=W4; P4(1)=W5
for i=1:n
    b(1,i+4) = waypoints(i+1);
end

for i=1:n
    A(row, 1+(8*(i-1)):8*i) = polyT(8,0,1); 
    row = row + 1;
end
%row = 9, after first 8 equations


% Next 3 equations
% P1(k)(t=0) = 0 for all k = 1..3
for i=1:3
    b(1,i+8) = 0;
end

for k=1:3
    A(row, 1:8) = polyT(8,k,0); 
    row = row + 1;
end
%row = 12, after first 11 equations



% Next 3 equations
% Pn(k)(t=1) = 0 for all k = 1..3
for i=1:3
    b(1,i+11) = 0;
end

for k=1:3
    A(row, 25:32) = polyT(8,k,1); 
    row = row + 1;
end
%row = 15, after first 14 equations


% Next 18 equations
% Pi-1(k)(t=1) = Pi(k)(t=0) for all i=2..n and k=1..6 
for i=1:18
    b(1,i+14) = 0;
end

for i=2:n
    for k=1:6
        A(row, 1+(8*(i-2)):i*8) = [polyT(8,k,1), -polyT(8,k,0)]; 
        row = row + 1;
    end
end
%row = 32, after 32nd equation.

%disp(A)
%disp(b)

coeff = A\b';
end


persistent waypoints0 traj_time d0 coeffx coeffy coeffz
if nargin > 2
    desired_state.pos = zeros(3,1);
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    xway = waypoints(1,:)';
    coeffx = getCoeff(xway);
    yway = waypoints(2,:)';
    coeffy = getCoeff(yway);
    zway = waypoints(3,:)';
    coeffz = getCoeff(zway);

    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;

    
else
    if(t >= traj_time(end))
        t = traj_time(end)-0.0001;
    end

    if (t == 0) 
        t_index = 1;
    else
        t_index = find(traj_time > t,1) - 1;
        t_index = max(t_index, 1);
    end
    scale = (t - traj_time(t_index))/d0(t_index);


    t0 = polyT(8,0,scale)';
    t1 = polyT(8,1,scale)';
    t2 = polyT(8,2,scale)';

    index = (t_index-1)*8+1:t_index*8;

    desired_state.pos = [coeffx(index)'*t0;...
                         coeffy(index)'*t0;...
                         coeffz(index)'*t0 ];
    desired_state.vel = [coeffx(index)'*t1.*(1/d0(t_index));...
                         coeffy(index)'*t1.*(1/d0(t_index));...
                         coeffz(index)'*t1.*(1/d0(t_index)) ];
    desired_state.acc = [coeffx(index)'*t2.*(1/d0(t_index)^2);...
                         coeffy(index)'*t2.*(1/d0(t_index)^2);...
                         coeffz(index)'*t2.*(1/d0(t_index)^2) ];
    desired_state.yaw = 0;
    desired_state.yawdot = 0;

    %if(t_index > 1)
        %t = t - traj_time(t_index-1);
    %end

    %if(t == 0)
        %desired_state.pos = waypoints0(:,1);
    %else
        %scale = t/d0(t_index-1);
        %desired_state.pos = (1 - scale) * waypoints0(:,t_index-1)...
            %+ scale * waypoints0(:,t_index);
    %end

    %desired_state.vel = zeros(3,1);
    %desired_state.acc = zeros(3,1);
    %desired_state.yaw = 0;
    %desired_state.yawdot = 0;
end
%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

