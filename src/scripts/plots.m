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
ylabel('$\alpha$')
legend('$\alpha$','$\alpha_{goal}$','$\alpha_s$','Interpreter','latex','Location','NorthEast')
%
subplot(3,1,2)
hold all
plot(t,y_cl(:,2),'Linewidth',1.5);
plot(t,y_T(:,2),'Linewidth',1);
plot(t,y_s(:,2))
grid on
ylabel('$\beta$')
legend('$\beta$','$\beta_{goal}$','$\beta_s$','Interpreter','latex','Location','NorthEast')
%
subplot(3,1,3)
plot(t,y_cl(:,3),'Linewidth',1.5);
hold on
grid on
legend('\gamma','Location','SouthEast')
ylabel('$\gamma$')
xlabel('time in s')
%
sgtitle('Plant Outputs')

% Real-time ability:
figure
stairs(t,compTime)
grid on
hold on
yline(dt)
ylabel('Computation time')
xlabel('Simulation time')
title('Real-time ability')

% % Slack variable sigma
% figure
% stairs(t,sigma)
% grid on
% title('Slack Variable $\sigma$')
% ylabel('$\sigma$')
% xlabel('time in s')

% Operational Coverage:
% figure
% hold all
% plot(y_cl(:,1),y_cl(:,2),'r-x')
% plot(y_T(:,1),y_T(:,2),'b')
% plot(yd(1,:),yd(2,:),'g-x')
% grid on
% title('Operational Coverage')
%legend('Position','Reference position','Collected data')