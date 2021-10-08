function [ ] = PrintPlot( plt, uvms )

% some predefined plots
% you can add your own

figure(1);
% subplot (2,1,1);
plot(plt.t, plt.ee_pos(:,:),'LineWidth', 2);
yline(uvms.goalPosition(1,1),'m--','Tool x goal');
yline(uvms.goalPosition(2,1),'m--','Tool y goal');
yline(uvms.goalPosition(3,1),'m--','Tool z goal');
%set(hplot, 'LineWidth', 1);
legend('ee_x','ee_y','ee_z');
xlabel('Time(s)');

figure(2);
% subplot (2,1,2);
% hplot = plot(plt.t, plt.a(10,:));
% set(hplot, 'LineWidth', 1);
% legend('A.md');
% ylabel('ha act. fn.');
% xlabel('Time(s)');

subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
xlabel('Time(s)');

subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
xlabel('Time(s)');

figure(3);

subplot(2,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
xlabel('Time(s)');

subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
xlabel('Time(s)');
% 
% subplot(3,2,5);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
% xlabel('Time(s)');
% 
% subplot(3,2,6);
% hplot = plot(plt.t, plt.a(8:9,:));
% set(hplot, 'LineWidth', 2);
% legend('Amu', 'Aha');
% xlabel('Time(s)');
    
figure(4)
plot(plt.t, plt.tool_err(:,:),'LineWidth', 2);
%set(hplot, 'LineWidth', 1);
legend('ee_x','ee_y','ee_z');
xlabel('Time(s)');

figure(5)
plot(plt.t, plt.vel(:,:),'LineWidth', 2);
%set(hplot, 'LineWidth', 1);
legend('sin');
xlabel('Time(s)');
    

end

