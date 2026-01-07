close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

%% Parameters
M= 4.1*10^4;%10;          %Number of discretisation steps of integrator
InitialTime = 0;       %Initial time of simulation
FinalTime   = 4.1;     %End time of simulation

InitialSystemState = [0.02; 0.9; 270];

%% ODE (system)

f_System = @ode_chemicalreactor;
f_SystemOutput = @ode_chemicalreactor_output;

%% Control Loop

Iteration    = 1;
SystemStates = InitialSystemState;

CurrentSystemState = InitialSystemState;

%[tstate,SystemTrajectory] = ode15s(@(t,x) f_System(t,x,FunnelControl(t,f_SystemOutput(x))), [InitialTime FinalTime], InitialSystemState,odeset('RelTol',1e-8,'AbsTol',1e-8));

SystemTrajectory=rk4Integrator(@(t,x) f_System(t,x,FunnelControl(t,f_SystemOutput(x))),InitialTime,FinalTime,M,InitialSystemState);

SystemStates = [InitialSystemState,SystemTrajectory];

%% Data for plots

TimeWindow  = [InitialTime FinalTime];
tstate_fc      = InitialTime:(FinalTime-InitialTime)/M:FinalTime-(FinalTime-InitialTime)/M;                              %Sampletimes of system states


%SystemOutput_fc  = f_SystemOutput(SystemStates);
SystemOutput_fc = zeros(1,length(tstate_fc));
ufc = zeros(1,length(tstate_fc));

for i = 1:length(tstate_fc)
    SystemOutput_fc(i)= f_SystemOutput(SystemStates(:,i));
    ufc(i) = FunnelControl(tstate_fc(i),f_SystemOutput(SystemStates(:,i)));
end
%TrackingError = SystemOutput_fc - ReferenceSignal(tstate_fc);

Reference=ReferenceSignal(tstate_fc);
Psi_fc = Funnel(tstate_fc);

save('data_fc.mat', 'tstate_fc', 'SystemOutput_fc','ufc');

%% Plots:

figure
hold on
    plot(tstate_fc,SystemOutput_fc,'linewidth',1.5)
    plot(tstate_fc,Reference);
    plot(tstate_fc,Reference + Psi_fc,'Color', 'r');
    plot(tstate_fc,Reference - Psi_fc,'Color', 'r');
    xlim([InitialTime,4])
    %ylim([-2 2])

    ylabel('$y$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y$ of FMPC','$y_{\mathrm{ref}}$','Interpreter','latex')
hold off

% figure
% hold on
%     plot(tstate,TrackingError,'linewidth',1.5)
%     plot(tstate,Psi,'Color', 'r');
%     plot(tstate,- Psi,'Color', 'r');
%     xlim([InitialTime,4])
%     %ylim([-1 1])
%     
%     ylabel('$y-y_{\mathrm{ref}}$','interpreter','latex')
%     xlabel('time $t$','interpreter','latex')
% 
%     legend('$y-y_{\mathrm{ref}}$ of FMPC','$\psi(t)$','Interpreter','latex')
% hold off

figure
hold on
    plot(tstate_fc,ufc,'linewidth',2.5)
    axis tight
    xlim([InitialTime,4])

    ylabel('$u$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$u(t)$ of FC','Interpreter','latex')
hold off


function a = FunnelControlAlpha(x)
    a= 1./(1-x);
end

function u_FC = FunnelControl(time, Cur_Output)
    e = (Cur_Output-ReferenceSignal(time));
    w = e./Funnel(time);
    u_FC = - 1*FunnelControlAlpha(norm(w)^2)*e;
end
