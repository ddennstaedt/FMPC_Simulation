clear all
close all

load('data_fmpc_original.mat')
load('data_fmpc_mani')


% 
% figure
% hold on
%     stairs(tcontrol,umpc,'linewidth',1.5)
%     axis tight
%     xlim([InitialTime,FinalTime])
%     ylim([0 30])
%     xlim([0 8])
%     ylabel('Input','interpreter','latex','FontSize',16)
%     xlabel('time $t$','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8])
%     yticks([0 10 20 30])
%     legend('$u_{\rm FMPC}(t)$','Interpreter','latex','FontSize',26,'Location','northwest')
% hold off
% 
% figure;
% plot(tstate,SystemOutput,'r')
% hold on
% plot(tstate,Reference,'b')
% plot(tstate,Reference+Psi,'k--')
% plot(tstate,Reference-Psi,'k--')
% xlim([0 8])
%     xlabel('time $t$','interpreter','latex','FontSize',16)
%     ylabel('Angular velocity~$\dot z_1(t)$','interpreter','latex','FontSize',16)
%     xticks([0 2 4 6 8])
%     yticks([0 100 200 300])
% legend('$y(t)$','$y_{\rm ref}(t)$','$\psi$','Interpreter','latex','FontSize',26,'Location','northwest')
%     hold off

%% Figures with mani
set(gcf, 'Renderer', 'Painters');
figure
hold on
    stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
    stairs(tcontrol_m,umpc_m,'color','#D95319','linewidth',2)
    axis tight
    xlim([InitialTime,FinalTime])
    ylim([-2 30])
    xlim([0 8])
    ylabel('Input u','interpreter','latex','FontSize',16)
    xlabel('Time t','interpreter','latex','FontSize',16)
    xticks([0 2 4 6 8])
    yticks([0 10 20 30])
    legend('$u_{\rm MPC}(t)$','$\hat u_{\rm MPC}(t)$','Interpreter','latex','FontSize',26,'Location','northwest')
    box off
hold off
saveas(gcf, 'Control', 'epsc')

figure;
plot(tstate,SystemOutput,'color','#0072BD','linewidth',2)
hold on
plot(tstate_m,SystemOutput_m,'color','#D95319','linewidth',2)
plot(tstate,Reference,'color','#77AC30','linewidth',2)
plot(tstate,Reference+Psi,'k--','linewidth',1.5)
plot(tstate,Reference-Psi,'k--','linewidth',1.5)
xlim([0 8])
    xlabel('Time t','interpreter','latex','FontSize',16)
    ylabel('Angular velocity y','interpreter','latex','FontSize',16)
    xticks([0 2 4 6 8])
    yticks([0 100 200 300])
legend('$y(t)$','$\hat y(t)$','$y_{\rm ref}(t)$','$y_{\rm ref}(t)\pm\psi(t)$','Interpreter','latex','FontSize',26,'Location','southeast')
box off
    hold off
    saveas(gcf, 'Output', 'epsc')
