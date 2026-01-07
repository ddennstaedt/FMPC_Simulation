clear all
close all

%% FMPC vs Quadratic short horizon
% 
% load('data_fmpc_short.mat')
% load('data_mpc_quadratic.mat')
% 
% figure
% hold on
%     stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
%     stairs(tcontrol_m,umpc_m,'color','#D95319','linewidth',2)
%     axis tight
%     xlim([InitialTime,FinalTime])
%     ylim([350 425])
%     xlim([0 4])
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([350 375 400 425])
%     legend('$u_{\rm FMPC}(t)$','$u_{\rm MPC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_fmpc_vs_mpc_short', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
% plot(tstate_m,SystemOutput_m,'color','#D95319','linewidth',2)
% plot(tstate,Reference,'color','#77AC30','linewidth',2)
% plot(tstate,Reference+Psi,'k--','linewidth',1.5)
% plot(tstate,Reference-Psi,'k--','linewidth',1.5)
% xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([250 300 350])
% legend('$y_{\rm FMPC}(t)$','$y_{\rm MPC}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
% box off
%     hold off
%     saveas(gcf, 'Output_fmpc_vs_mpc_short', 'epsc')

%% FMPC vs Quadratic long horizon

% load('data_fmpc_long.mat')
% load('data_mpc_quadratic_long.mat')
% figure
% hold on
%     stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
%     stairs(tcontrol_m,umpc_m,'color','#D95319','linewidth',2)
%     axis tight
%     xlim([InitialTime,FinalTime])
%     ylim([330 425])
%     xlim([0 4])
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([350 375 400 425])
%     legend('$u_{\rm MPC}(t)$','$u_{\rm MPC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_fmpc_vs_mpc_long', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
% plot(tstate_m,SystemOutput_m,'color','#D95319','linewidth',2)
% plot(tstate,Reference,'color','#77AC30','linewidth',2)
% plot(tstate,Reference+Psi,'k--','linewidth',1.5)
% plot(tstate,Reference-Psi,'k--','linewidth',1.5)
% xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([250 300 350])
% legend('$y_{\rm FMPC}(t)$','$y_{\rm MPC}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
% box off
%     hold off
%     saveas(gcf, 'Output_fmpc_vs_mpc_long', 'epsc')

%% FMPC vs FC
% 
%   load('data_fmpc_long.mat')
%   load('data_fc.mat')
% set(gcf, 'Renderer', 'Painters');
% figure
% hold on
%     stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
%     plot(tstate_fc,ufc,'color','#D95319','linewidth',2)
%     axis tight
%     xlim([InitialTime,FinalTime])
%     ylim([0 425])
%     xlim([0 4])
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([0 100 200 300 400])
%     legend('$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
%     box off
% hold off
% saveas(gcf, 'Control_fmpc_vs_fc', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
% plot(tstate_fc,SystemOutput_fc,'color','#D95319','linewidth',2)
% plot(tstate,Reference,'color','#77AC30','linewidth',2)
% plot(tstate,Reference+Psi,'k--','linewidth',1.5)
% plot(tstate,Reference-Psi,'k--','linewidth',1.5)
% xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([0 250 300 350])
% legend('$y_{\rm FMPC}(t)$','$y_{\rm FC}(t)$','$y_{\rm re f}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
% box off
%     hold off
%     saveas(gcf, 'Output_fmpc_vs_fc', 'epsc')

%% FMPC vs FC fail
%   load('data_fmpc_long.mat')
%   load('data_fc_fail.mat')
% set(gcf, 'Renderer', 'Painters');
% figure
% hold on
%     stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
%     plot(tstate_fc,ufc,'color','#D95319','linewidth',2)
%     axis tight
%     xlim([InitialTime,FinalTime])
%     ylim([-425 425])
%     xlim([0 4])
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([-400 -300 -200 -100 0 100 200 300 400])
%     legend('$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
%     box off
% hold off
% saveas(gcf, 'Control_fmpc_vs_fc_fail', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
% plot(tstate_fc,SystemOutput_fc,'color','#D95319','linewidth',2)
% plot(tstate,Reference,'color','#77AC30','linewidth',2)
% plot(tstate,Reference+Psi,'k--','linewidth',1.5)
% plot(tstate,Reference-Psi,'k--','linewidth',1.5)
% xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([0 100 200 300 350])
% legend('$y_{\rm FMPC}(t)$','$y_{\rm FC}(t)$','$y_{\rm re f}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','east')
% box off
%     hold off
%     saveas(gcf, 'Output_fmpc_vs_fc_fail', 'epsc')

%% Figures robust FMPC
%% robust FMPC non robust
% load('data_robustfmpc_nonrobust.mat')
% figure;
% plot(tstate,u_applied,'color','k','linewidth',2)
%     hold on
%     axis tight
%     xlim([0,4])
%     ylim([300 620])
%     xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([0 300 400 500 600])
%     legend('$u(t)=u_{\rm FMPC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_robustfmpc_nonrobust', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
%     plot(tstate,ModelOutput,'color','#D95319','linewidth',2)
%     plot(tstate,Reference,'color','#77AC30','linewidth',2)
%     plot(tstate,Reference+Psi,'k--','linewidth',1.5)
%     plot(tstate,Reference-Psi,'k--','linewidth',1.5)
%     xlim([0 4])
%     ylim([245 510])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([0 250 300 350 400 450 500])
%     legend('$y(t)$','$y_{\rm M}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
%saveas(gcf, 'Output_robustfmpc_nonrobust', 'epsc')

%% robust FMPC trivial
% load('data_robustfmpc_trivial.mat')
% figure;
%     hold on
%     plot(tstate,u_applied,'color','k','linewidth',2)
%     stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
%     plot(tstate,uFC,'color','#0072BD','linewidth',2)
%     axis tight
%     xlim([0,4])
%     ylim([-250 610])
%     xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([-200 0 200 400 600])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
% saveas(gcf, 'Control_robustfmpc_trivial', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
%     plot(tstate,ModelOutput,'color','#D95319','linewidth',2)
%     plot(tstate,Reference,'color','#77AC30','linewidth',2)
%     plot(tstate,Reference+Psi,'k--','linewidth',1.5)
%     plot(tstate,Reference-Psi,'k--','linewidth',1.5)
%     xlim([0 4])
%     ylim([260 350])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([275 300 325 350])
%     legend('$y(t)$','$y_{\rm M}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
% saveas(gcf, 'Output_robustfmpc_trivial', 'epsc')

%% robust FMPC reinit
% load('data_robustfmpc_reinit.mat')
% figure;
%     hold on
%     plot(tstate,u_applied,'color','k','linewidth',2)
%     stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
%     plot(tstate,uFC,'color','#0072BD','linewidth',2)
%     axis tight
%     xlim([0,4])
%     ylim([-250 610])
%     xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([-200 0 200 400 600])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
% saveas(gcf, 'Control_robustfmpc_reinit', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
%     plot(tstate,ModelOutput,'color','#D95319','linewidth',2)
%     plot(tstate,Reference,'color','#77AC30','linewidth',2)
%     plot(tstate,Reference+Psi,'k--','linewidth',1.5)
%     plot(tstate,Reference-Psi,'k--','linewidth',1.5)
%     xlim([0 4])
%     ylim([260 350])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([275 300 325 350])
%     legend('$y(t)$','$y_{\rm M}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
% saveas(gcf, 'Output_robustfmpc_reinit', 'epsc')


%% Learning fmpc
% load('data_learning_fmpc.mat')
% figure;
%     hold on
%     plot(tstate,u_applied,'color','k','linewidth',2)
%     stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
%     plot(tstate,uFC,'color','#0072BD','linewidth',2)
%     axis tight
%     xlim([0,4])
%     ylim([-60 450])
%     xlim([0 4])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([-50 0 100 200 300 400])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
% saveas(gcf, 'Control_learning_fmpc', 'epsc')
% 
% figure;
% plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
% hold on
%     plot(tstate,ModelOutput,'color','#D95319','linewidth',2)
%     plot(tstate,Reference,'color','#77AC30','linewidth',2)
%     plot(tstate,Reference+Psi,'k--','linewidth',1.5)
%     plot(tstate,Reference-Psi,'k--','linewidth',1.5)
%     xlim([0 4])
%     ylim([245 350])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 1 2 3 4])
%     yticks([250 275 300 325 350])
%     legend('$y(t)$','$y_{\rm M}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','east')
%     box off
% hold off
% saveas(gcf, 'Output_learning_fmpc', 'epsc')

load('data_learning_fmpc_often.mat')
figure;
    hold on
    plot(tstate,u_applied,'color','k','linewidth',2)
    stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
    plot(tstate,uFC,'color','#0072BD','linewidth',2)
    axis tight
    xlim([0,4])
    ylim([-50 450])
    xlim([0 4])
    xlabel('Time t','interpreter','latex','FontSize',16)
    ylabel('Input u','interpreter','latex','FontSize',16)
    xticks([0 1 2 3 4])
    yticks([-50 0 100 200 300 400])
    legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','east')
    box off
hold off
saveas(gcf, 'Control_learning_fmpc_often', 'epsc')

figure;
plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
hold on
    plot(tstate,ModelOutput,'color','#D95319','linewidth',2)
    plot(tstate,Reference,'color','#77AC30','linewidth',2)
    plot(tstate,Reference+Psi,'k--','linewidth',1.5)
    plot(tstate,Reference-Psi,'k--','linewidth',1.5)
    xlim([0 4])
    ylim([245 350])
    xlabel('Time t','interpreter','latex','FontSize',16)
    ylabel('Temperature y','interpreter','latex','FontSize',16)
    xticks([0 1 2 3 4])
    yticks([250 275 300 325 350])
    legend('$y(t)$','$y_{\rm M}(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','east')
    box off
hold off
saveas(gcf, 'Output_learning_fmpc_often', 'epsc')
