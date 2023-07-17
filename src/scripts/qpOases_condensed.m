%% Init script for Data-driven Predicitive Control of a 3-DoF Helicopter.
%  See documentation for physical and mathematical background.
%  
%  Attached files: 
%  - Data_driven_MPC_qpOASES_uncondensed.slx   <-- Simulation Model
%  - hankel_c.m                                <-- used in controller
%  - Travel_traj.mat                           <-- contains QP matrices
%  - Elevation_traj.mat                        <-- contains QP matrices
%  - Data-driven MPC of 3-DoF Helicopters.pdf  <-- Documentation
%
% ATTENTION: qpOASES has to be compiled first!

%% Setup
run('setup.m')

%% Compile qpOASES

error('First add qpOASES_SQProblem.cpp from core folder to simulink subfolder in qpOASES submodule')
addpath('../core/qpOASES/interfaces/simulink')
run('make.m')

%% Compile qpOASES

currentPath = pwd;
error('First add your qpOASES path in init script (line 23)')
%cd 'C:\Users\Fabian\Documents\qpOASES\interfaces\simulink'
run('make.m')
cd(currentPath);
error('Here also (line 27)')
%addpath('C:\Users\Fabian\Documents\qpOASES\interfaces\simulink')

warning('Dont forget to update the header of "qpOASES_SQProblem.cpp"')

%% System Setup 

x0 = [0*pi/180; 0; -30*pi/180; 0; 0; 0;]; % Starting position

n  = 6;  % system dimension
nu = 6;  % estimated dimension
m  = 2;  % Input dimension
p  = 3;  % Output dimension

%% Reference Trajectory Setup

% Setpoints to be reached (tracking trajectory)
u_T = [1.1447/2;  1.1447/2]; % input corresponding to y_T = 0 (see DataGen.m)

% Angle setpoints:
y_T1 = [0*pi/180; 0*pi/180; 0];    % alpha_s, beta_s, gamma_s
y_T2 = [90*pi/180; 0*pi/180; 0];   % alpha_s, beta_s, gamma_s
y_T3 = [0*pi/180; 0*pi/180; 0];    % alpha_s, beta_s, gamma_s
y_T4 = [0*pi/180; -30*pi/180; 0];  % alpha_s, beta_s, gamma_s

% Set point transition times:
T_Track = 5;
T_Track2 = 20;
T_Track3 = 30;

y_goal = [0; 0; 0];

%% Data-driven Predictive Control Setup

L_true = 50;     % Prediction Horizon (L_true*dt seconds)
L = L_true + nu; % for convenience
t_data = 31;
dt = 0.1;
N = tsim/dt + 1;  % Data length (N*dt seconds)

% Cost Matrices
Q = blkdiag(15,15,15);%  60*eye(p);
R = 10*eye(m);

% Artificial Setpoint Matrices 
S_y = diag([10000;500;0]);

% Regularization Parameters
if NOISE
    lambda_alpha = 150;
    lambda_sigma = 1e7;
else
    lambda_alpha = 20;
    %lambda_sigma = 1e5;
    lambda_sigma = 1e7;
end

% Not important for now
lambda_beta = 0;


%% Quadratic Program Setup
%
%                 min    1/2 z'Hz + f'z
%                 s.t.  lbA <= A*z <= ubA
%                        lb <= z <= ub
%
%           z = [alpha; us; ys; sigma]

%%%%%%%%%%%%%%%%%%%%% For travel task %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('datatraj_travel') %-> ud, yd
Hu = hankel_c(ud(:), L, N-L+1,m);
Hy = hankel_c(yd(:), L, N-L+1,p);

Hu_init = Hu(1:m*nu,:);
Hy_init = Hy(1:p*nu,:);
Hu_tec  = Hu(end-m*nu+1:end,:);
Hy_tec  = Hy(end-p*nu+1:end,:);

% Cost function
H_alpha = Hu'*kron(R,eye(L))*Hu+Hy'*kron(Q,eye(L))*Hy+lambda_alpha*eye(N-L+1);
H_sigma = kron(Q,eye(L))+lambda_sigma*eye(p*L);
H = 2*[H_alpha,             -Hu'*repmat(R,L,1), -Hy'*repmat(Q,L,1), -Hy'*kron(Q,eye(L));...
      (-Hu'*repmat(R,L,1))', L*R,                zeros(m,p+p*L); ...
      (-Hy'*repmat(Q,L,1))', zeros(p,m), L*Q+S_y, repmat(Q,L,1)'; ...
      -kron(Q,eye(L))'*Hy,   zeros(p*L,m), repmat(Q,L,1), H_sigma];
H = (H+H')/2; %for numerical issues
H_t = H;

% Constraints
A_QP = [Hu_init, zeros(nu*m,m+p+p*L);...
        Hy_init, zeros(nu*p,m+p) [-eye(p*nu) zeros(p*nu, p*(L-nu))];...
        Hu_tec, -kron(ones(nu,1),eye(m)) zeros(nu*m,p+p*L);...
        Hy_tec, zeros(nu*p,m) -kron(ones(nu,1),eye(p)), [zeros(p*nu, p*(L-nu)) -eye(p*nu)];...
        ones(1,N-L+1), zeros(1,m+p+p*L);...
        Hu, zeros(m*L,m+p+p*L)];
A_t = A_QP;
Hu_t = Hu;


%%%%%%%%%%%%%%%%%%%%%%%%%% For elevation task %%%%%%%%%%%%%%%%%%%%%%%%%%
load('datatraj_elevation') %-> ud, yd
Hu = hankel_c(ud(:), L, N-L+1,m);
Hy = hankel_c(yd(:), L, N-L+1,p);

Hu_init = Hu(1:m*nu,:);
Hy_init = Hy(1:p*nu,:);
Hu_tec  = Hu(end-m*nu+1:end,:);
Hy_tec  = Hy(end-p*nu+1:end,:);

% Cost function
H_alpha = Hu'*kron(R,eye(L))*Hu+Hy'*kron(Q,eye(L))*Hy+lambda_alpha*eye(N-L+1);
H_sigma = kron(Q,eye(L))+lambda_sigma*eye(p*L);
H = 2*[H_alpha,             -Hu'*repmat(R,L,1), -Hy'*repmat(Q,L,1), -Hy'*kron(Q,eye(L));...
      (-Hu'*repmat(R,L,1))', L*R,                zeros(m,p+p*L); ...
      (-Hy'*repmat(Q,L,1))', zeros(p,m), L*Q+S_y, repmat(Q,L,1)'; ...
      -kron(Q,eye(L))'*Hy,   zeros(p*L,m), repmat(Q,L,1), H_sigma];
H = (H+H')/2; %for numerical issues
H_e = H;

% Constraints
A_QP = [Hu_init, zeros(nu*m,m+p+p*L);...
        Hy_init, zeros(nu*p,m+p) [-eye(p*nu) zeros(p*nu, p*(L-nu))];...
        Hu_tec, -kron(ones(nu,1),eye(m)) zeros(nu*m,p+p*L);...
        Hy_tec, zeros(nu*p,m) -kron(ones(nu,1),eye(p)), [zeros(p*nu, p*(L-nu)) -eye(p*nu)];...
        ones(1,N-L+1), zeros(1,m+p+p*L);...
        Hu, zeros(m*L,m+p+p*L)];
A_e = A_QP;
Hu_e = Hu;

% See documentation for force restrictions
u_min  = [0; 0];
u_max  = [0.98; 0.98];
lb = [-inf*ones(N-L+1,1); u_min; -inf*ones(p+p*L,1)];
ub = [inf*ones(N-L+1,1); u_max; inf*ones(p+p*L,1)];

%% Simulation

T_sim = 40;
dt = 0.1;

NOISE = 0;
noise_bound_cl   = 1*pi/180; % noise in closed loop measurements

simout = sim('Data_driven_MPC_condensed.slx');

% Read out signals
t        = simout.tout;
u_cl     = simout.input.Data;
y_cl     = simout.output.Data;
y_s      = simout.setpoint.Data;
y_T      = simout.trajectory.Data;

% Convert to Degree
y_cl  = y_cl*180/pi;
y_s   = y_s*180/pi;
y_T   = y_T*180/pi;

%% Plots

% Control inputs:
figure(1)
subplot(2,1,1)
stairs(t,u_cl(:,1))
hold on
grid on
yline(u_T(1),'r')
legend('u_1','u_{1,s}')
%
subplot(2,1,2)
stairs(t,u_cl(:,2))
hold on
grid on
yline(u_T(2),'r')
legend('u_2','u_{2,s}')
%
sgtitle('Control Inputs')

% Plant outputs:
figure(2)
subplot(3,1,1)
hold all
plot(t,y_cl(:,1),'Linewidth',1.5);
plot(t,y_T(:,1),'Linewidth',1);
plot(t,y_s(:,1))
grid on
ylim([-10,100])
legend('$\alpha$','$\alpha_{goal}$','$\alpha_s$','Interpreter','latex','Location','SouthEast')
%
subplot(3,1,2)
hold all
plot(t,y_cl(:,2),'Linewidth',1.5);
plot(t,y_T(:,2),'Linewidth',1);
plot(t,y_s(:,2))
grid on
legend('$\beta$','$\beta_{goal}$','$\beta_s$','Interpreter','latex','Location','South')
%
subplot(3,1,3)
plot(t,y_cl(:,3),'Linewidth',1.5);
hold on
grid on
legend('\gamma','Location','South')
xlabel('time in s')
%
sgtitle('Plant Outputs')