
set(0,'defaulttextInterpreter','latex');

%% Script for (simulative) Data Generation
%  Will be called in init files where Data Generation is needed.
%
%  See documentation for physical and mathematical background.
%  
%  Attached files: 
%   - DataGenerationNonlin.slx     <-- Simulink-Model
%   - hankel_r.m                   <-- Hankel-matrix calculation

%% Model parameters for 3-DoF Helicopter

m_H = 1.322;
m_A = 2.638;

Jxx = 1.1455;
Jyy = 1.1184;
Jzz = 0.032;

r = 0.1775;

rHx = -0.089;
rHz = 0.655;
rAHx = -0.034;
rAHz = 0.0193;

g = 9.81;

%% Nonlinear equations of motion

syms alph dalph bet dbet gam dgam %short for alpha, alpha_dot, beta, beta_dot, gamma, gamma_dot
syms Fsum Fdiff

f = [dalph;...
     1/Jxx * Fsum*sin(gam) * (rHx*sin(bet) - rHz*cos(bet));...
     dbet;...
     1/Jyy * (-(m_A+m_H)*g *(rAHz*cos(bet) - rAHx*sin(bet)) + Fsum * rHz*cos(gam));...
     dgam;...
     -1/Jzz * r * Fdiff];
 
%% Linearization around set point
 
x = [alph; dalph; bet; dbet; gam; dgam];
u = [Fsum; Fdiff];

A_t = jacobian(f,x);
B_t = jacobian(f,u);

A_fct = matlabFunction(A_t,'Vars',[x; u]);
B_fct = matlabFunction(B_t,'Vars',[x; u]);

% Set point
Fdiff_lin = 0;
Fsum_lin = (m_A+m_H)*g * rAHz/rHz;
F_lin = [Fsum_lin; Fdiff_lin];

% Linearize matrices around zero equilibrium
A = A_fct(0,0,0,0,0,0,Fsum_lin,Fdiff_lin);
B = B_fct(0,0,0,0,0,0,Fsum_lin,Fdiff_lin);
C = [1 0 0 0 0 0;...
     0 0 1 0 0 0;...
     0 0 0 0 1 0];
D = zeros(3,2);

n = 6; %system order
m = 2; %number of inputs
p = 3; %number of outputs

clear alph dalph bet dbet gam dgam Fsum Fdiff x u
clear A_fct B_fct A_t B_t
 
%% System analysis

poles = eig(A);
rank_P = rank(ctrb(A,B));
rank_Q = rank(obsv(A,C));

%% Pre stabilizing controller

%State feedback:
%Q = blkdiag(500,1,5,1,1,1);% <- for elevation traj (u_dist=0.2, tdist=0.3)
Q = 5*eye(6);               % <- for travel traj (u_dist=0.2, tdist = 0.4)
R = eye(2);
K = lqr(A,B,Q,R);

%Observer:
Qhat = 50*eye(6);
Rhat = eye(3);
Lobs = lqr(A',C',Qhat,Rhat)';

clear Q R Qhat Rhat

%% Data Generation

x0 = [0/180*pi; 0; 0/180*pi; 0; 0; 0];

simout = sim('DataGenerationNonlin.slx');

t  = simout.tout;
ud = simout.input.Data';
yd = simout.output.Data';

% Convert to Front and Backmotor Force
ud = 1/2*[1 1;1 -1] * ud;
% ( Attention: if data is generated from a linearized model, the linearization
%   offset F_lin has to be added afterwards )

if plotData
    figure(4)
    
    subplot(2,2,1)
    plot(t,ud(1,:))
    title('Input Data')
    ylabel('$u_1$ in N')
    xlim([0,tsim])
    grid on
    subplot(2,2,3)
    plot(t,ud(2,:))
    ylabel('$u_2$ in N')
    grid on
    xlabel('time in s')
    xlim([0,tsim])
    
    subplot(3,2,2)
    plot(t,yd(1,:)*180/pi)
    title('Output Data')
    ylabel('$\alpha$')
    xlim([0,tsim])
    grid on
    subplot(3,2,4)
    plot(t,yd(2,:)*180/pi)
    ylabel('$\beta$')
    xlim([0,tsim])
    grid on
    subplot(3,2,6)
    plot(t,yd(3,:)*180/pi)
    ylabel('$\gamma$')
    xlim([0,tsim])
    grid on
    xlabel('time in s')
    
    set(gcf,'position',[0,0,750,200])
end

%% Data Analysis

% N = length(t);      %Data length = tsim/dt + 1
% L_true = 50;        %Simulation length
% 
% if N >= (m+1)*(L_true+n)
%     disp('Minimum data length fullfilled')
% else
%     error('Data not large enough for Simulation horizon')
% end
% 
% Hu = hankel_r(ud(:), L_true+n, N-(L_true+n)+1,m);
% Hy = hankel_r(yd(:), L_true+n, N-(L_true+n)+1,p);
% 
% if abs(min(svd(Hu))) > 1e-3
%     disp('u(t) is persistently exciting')
%     disp('Minimum singular value of Hu:')
%     disp(num2str(min(svd(Hu))))
% else
%     disp('Minimum singular value of Hu:')
%     disp(num2str(min(svd(Hu))))
%     warning('u might not be persistently exciting')
% end
