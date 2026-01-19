clear all
close all

Scenario = 0;

% Scenario 0: ZoH FC

%% ZoH FC

if 0 == Scenario
    load('data_fc_zoh_scen1.mat')
    load('data_fc_zoh_scen2.mat')
    
    
    figure
    hold on
        plot(t_con,u_ZoH,'color','#D95319','linewidth',2)
        plot(t_con_m,u_ZoH_m,'color','#77AC30','linewidth',2)
        plot(t_FC,u_FC,'color','#0072BD','linewidth',2)
        axis tight
        xlim([0,1])
        ylim([-40 100])
        ylabel('Input u','interpreter','latex','FontSize',16)
        xlabel('Time t','interpreter','latex','FontSize',16)
        xticks([0 0.5 1])
        yticks([-40 0 50 100])
        legend('$u_{\rm ZoH}(t)$','$\hat{u}_{\rm ZoH}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
        box off
    hold off
    saveas(gcf, 'Control_massoncar_ZoH', 'epsc')
    
    figure;
    hold on
    plot(t_con,y_ZoH,'color','#D95319','linewidth',2)
    plot(t_con_m,y_ZoH_m,'color','#77AC30','linewidth',2)
    plot(t_FC,y_FC,'color','#0072BD','linewidth',2)
    plot(t_con,ref+psi,'k--','linewidth',1.5)
    plot(t_con,ref-psi,'k--','linewidth',1.5)
    xlim([0 1])
    ylim([-0.15 0.55])
        xlabel('Time t','interpreter','latex','FontSize',16)
        ylabel('System output y','interpreter','latex','FontSize',16)
        xticks([0 0.5 1])
        yticks([-0.15 0 0.15 0.5])
    legend('$y_{\rm ZoH}(t)$','$\hat{y}_{\rm ZoH}(t)$','$y_{\rm FC}(t)$','$y_{\rm ref}(t)\pm0.15$','Interpreter','latex','FontSize',26,'Location','southeast')
    box off
    hold off
    saveas(gcf, 'Output_massoncar_ZoH', 'epsc')

elseif Scenario == 1

% load('data_fc.mat')
% load('data_fmpc_10-3.mat')
% 
% 
% figure
% hold on
%     stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
%     plot(t_fc,ufc,'color','#D95319','linewidth',2)
%     axis tight
%     xlim([0,10])
%     ylim([-30 20])
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-30 -20 -10 0 10 20])
%     legend('$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_massoncar_FC', 'epsc')
% 
% figure;
% hold on
% plot(tstate,TrackingError,'color','#0072BD','linewidth',2)
% plot(t_fc,TrackingError_FC,'color','#D95319','linewidth',2)
% plot(tstate,Psi,'k--','linewidth',1.5)
% plot(tstate,-Psi,'k--','linewidth',1.5)
% xlim([0 10])
% ylim([-1 1])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Tracking error e','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-1 -0.5 0 0.5 1])
% legend('$y_{\rm FMPC}(t)-y_{\rm ref}(t)$','$y_{\rm FC}(t)-y_{\rm ref}(t)$','$\pm \psi(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
% box off
%     hold off
%     saveas(gcf, 'Output_massoncar_FC', 'epsc')

end

% FMPC vs MPC
% 
% load('data_fmpc_10-4.mat')
% load('data_mpc_quadratic.mat')
% 
% figure
% hold on
%     stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
%     stairs(tcontrol_m,umpc_m,'color','#D95319','linewidth',2)
%     axis tight
%     xlim([0,10])
%     ylim([-30 32])
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-30 -15 0 15 30])
%     legend('$u_{\rm FMPC}(t)$','$u_{\rm MPC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_massoncar_FMPCvsMPC', 'epsc')
% 
% 
% figure;
% hold on
% box off
% plot(tstate,TrackingError,'color','#0072BD','linewidth',2)
% plot(tstate_m,TrackingError_m,'color','#D95319','linewidth',2)
% plot(tstate,Psi,'k--','linewidth',1.5)
% plot(tstate,-Psi,'k--','linewidth',1.5)
% xlim([0 10])
% ylim([-1 1])
% xlabel('Time t','interpreter','latex','FontSize',16)
% ylabel('Tracking error e','interpreter','latex','FontSize',16)
% xticks([0 2 4 6 8 10])
% yticks([-1 -0.5 0 0.5 1])
% legend('$y_{\rm FMPC}(t)-y_{\rm ref}(t)$','$y_{\rm MPC}(t)-y_{\rm ref}(t)$','$\pm \psi(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
% 
% annotation('arrow', [0.61 0.5], [0.39 0.47],'linewidth',2);
% insetAx = axes('Position', [0.36, 0.18, 0.5, 0.2], 'NextPlot', 'add');  % [x, y, width, height]
% box on;
% 
% xRange = [4.7, 5.0];
% yRange = [-0.1005, -0.0998];
% xlim(insetAx, xRange);
% ylim(insetAx, yRange);
% xticks(insetAx,[4.7 4.85 5])
% yticks(insetAx,[-0.1005 -0.1002 -0.0998])
% plot(insetAx,tstate,TrackingError,'color','#0072BD','linewidth',2)
% plot(insetAx,tstate_m,TrackingError_m,'color','#D95319','linewidth',2)
% plot(insetAx,tstate,-Psi,'k--','linewidth',1.5)
% 
% 
% hold off
%  
% saveas(gcf, 'Output_massoncar_FMPCvsMPC', 'epsc')

%% robust FMPC trivial
% 
% load('data_robustfmpc_trivial.mat')
% figure;
%     hold on
%     plot(tstate,u_applied,'color','k','linewidth',2)
%     stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
%     plot(tstate,uFC,'color','#0072BD','linewidth',2)
%     axis tight
%     xlim([0,10])
%     ylim([-32 32])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-30 -15 0 15 30])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
%     box off
% hold off
% saveas(gcf, 'Control_robustfmpc_trivial', 'epsc')
% 
% figure;
% hold on
%     plot(tstate,TrackingError,'color','#0072BD','linewidth',2)
%     plot(tstate,ModelError,'color','#D95319','linewidth',2)
%     plot(tstate,Psi,'k--','linewidth',1.5)
%     plot(tstate,-Psi,'k--','linewidth',1.5)
%     xlim([0 10])
%     ylim([-1 1])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-1 -0.5 0 0.5 1])
%     legend('$y(t)-y_{\rm ref}(t)$','$y_{\rm M}(t)-y_{\rm ref}(t)$','$\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
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
%     xlim([0,10])
%     ylim([-18 32])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-30 -15 0 15 30])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_robustfmpc_reinit', 'epsc')
% 
% figure;
% hold on
%     plot(tstate,TrackingError,'color','#0072BD','linewidth',2)
%     plot(tstate,ModelError,'--','color','#D95319','linewidth',2)
%     plot(tstate,Psi,'k--','linewidth',1.5)
%     plot(tstate,-Psi,'k--','linewidth',1.5)
%     xlim([0 10])
%     ylim([-1 1])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-1 -0.5 0 0.5 1])
%     legend('$y(t)-y_{\rm ref}(t)$','$y_{\rm M}(t)-y_{\rm ref}(t)$','$\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
%     annotation('arrow', [0.36 0.28], [0.39 0.545],'linewidth',2);
%     insetAx = axes('Position', [0.36, 0.18, 0.5, 0.2], 'NextPlot', 'add');  % [x, y, width, height]
%     box on;
%     
%     xRange = [1.67, 2.35];
%     yRange = [0.0735, 0.09];
%     xlim(insetAx, xRange);
%     ylim(insetAx, yRange);
%     xticks(insetAx,[1.8 2 2.2])
%     yticks(insetAx,[0.075 0.08 0.085 0.09])
%     plot(insetAx,tstate,TrackingError,'color','#0072BD','linewidth',2)
%     plot(insetAx,tstate,ModelError,'color','#D95319','linewidth',2)
% saveas(gcf, 'Output_robustfmpc_reinit', 'epsc')
%
%% learning robust FMPC
% 
% load('data_learning_fmpc.mat')
% figure;
%     hold on
%     plot(tstate,u_applied,'color','k','linewidth',2)
%     stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
%     plot(tstate,uFC,'color','#0072BD','linewidth',2)
%     axis tight
%     xlim([0,10])
%     ylim([-31 31])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-30 -15 0 15 30])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_learning_fmpc', 'epsc')
% 
% figure;
% hold on
%     plot(tstate,TrackingError,'color','#0072BD','linewidth',2)
%     plot(tstate,ModelError,'--','color','#D95319','linewidth',2)
%     plot(tstate,Psi,'k--','linewidth',1.5)
%     plot(tstate,-Psi,'k--','linewidth',1.5)
%     xlim([0 10])
%     ylim([-1 1])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-1 -0.5 0 0.5 1])
%     legend('$y(t)-y_{\rm ref}(t)$','$y_{\rm M}(t)-y_{\rm ref}(t)$','$\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Output_learning_fmpc', 'epsc')

% load('data_learning_fmpc_dist.mat')
% figure;
%     hold on
%     plot(tstate,u_applied,'color','k','linewidth',2)
%     stairs(tcontrol,umpc,'color','#D95319','linewidth',2)
%     plot(tstate,uFC,'color','#0072BD','linewidth',2)
%     axis tight
%     xlim([0,10])
%     ylim([-31 31])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Input u','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-30 -15 0 15 30])
%     legend('$u(t)=u_{\rm FMPC}(t)+u_{\rm FC}(t)$','$u_{\rm FMPC}(t)$','$u_{\rm FC}(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Control_learning_fmpc_dist', 'epsc')
% 
% figure;
% hold on
%     plot(tstate,TrackingError,'color','#0072BD','linewidth',2)
%     plot(tstate,ModelError,'--','color','#D95319','linewidth',2)
%     plot(tstate,Psi,'k--','linewidth',1.5)
%     plot(tstate,-Psi,'k--','linewidth',1.5)
%     xlim([0 10])
%     ylim([-1 1])
%     xlabel('Time t','interpreter','latex','FontSize',16)
%     ylabel('Temperature y','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8 10])
%     yticks([-1 -0.5 0 0.5 1])
%     legend('$y(t)-y_{\rm ref}(t)$','$y_{\rm M}(t)-y_{\rm ref}(t)$','$\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','northeast')
%     box off
% hold off
% saveas(gcf, 'Output_learning_fmpc_dist', 'epsc')
