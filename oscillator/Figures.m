clear all
close all

load('data_fmpc_scen0.mat')
load('data_fmpc_scen1.mat')

%% Figures of scenario 0 & scenario 1

set(gcf, 'Renderer', 'Painters');
figure
hold on
stairs(tcontrol,umpc,'color','#0072BD','linewidth',2)
stairs(tcontrol_1,umpc_1,'color','#D95319','linewidth',2)
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
plot(tstate_1,SystemOutput_1,'color','#D95319','linewidth',2)
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
