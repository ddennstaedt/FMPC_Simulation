close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

Scenario = 1;

if Scenario == 0
    M= 4.1*10^4;       %Number of discretisation steps of integrator
else 
    M= 4.1*10^3;
end

%% Parameters

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

SystemTrajectory=rk4Integrator(@(t,x) f_System(t,x,FunnelControl(t,f_SystemOutput(x))),InitialTime,FinalTime,M,InitialSystemState);

SystemStates = [InitialSystemState,SystemTrajectory];

%% Data for plots

TimeWindow  = [InitialTime FinalTime];
tstate_fc      = InitialTime:(FinalTime-InitialTime)/M:FinalTime-(FinalTime-InitialTime)/M;                              %Sampletimes of system states

SystemOutput_fc = zeros(1,length(tstate_fc));
ufc = zeros(1,length(tstate_fc));

for i = 1:length(tstate_fc)
    SystemOutput_fc(i)= f_SystemOutput(SystemStates(:,i));
    ufc(i) = FunnelControl(tstate_fc(i),f_SystemOutput(SystemStates(:,i)));
end

Reference=ReferenceSignal(tstate_fc);
Psi_fc = Funnel(tstate_fc);

if Scenario== 0
    save('data_fc.mat', 'tstate_fc', 'SystemOutput_fc','ufc');
else
    save('data_fc_fail.mat', 'tstate_fc', 'SystemOutput_fc','ufc');
end
%% Plots:

figure
hold on
    plot(tstate_fc,SystemOutput_fc,'linewidth',1.5)
    plot(tstate_fc,Reference);
    plot(tstate_fc,Reference + Psi_fc,'Color', 'r');
    plot(tstate_fc,Reference - Psi_fc,'Color', 'r');
    xlim([InitialTime,4])

    ylabel('$y$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y$ of FMPC','$y_{\mathrm{ref}}$','Interpreter','latex')
hold off

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
