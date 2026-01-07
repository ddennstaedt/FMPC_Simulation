close all
clear all

%addpath('~/Documents/workspace/casadi-3.7.0');
%addpath('../util/')
import casadi.*
import casadi.tools.*

%% Parameters
M= 10^7;%10;          %Number of discretisation steps of integrator
InitialTime = 0;       %Initial time of simulation
FinalTime   = 10;     %End time of simulation

InitialSystemState = [0; 0; 0; 0];

%% ODE (system)

f_System = @ode_massoncar;
f_SystemOutput = @ode_massoncar_output;
f_SystemOutputDot = @ode_massoncar_outputdot;

SystemStates = InitialSystemState;

TimeWindow  = [InitialTime FinalTime];
[t_fc,state_fc]=ode15s(@(t,x) f_System(t,x,FunnelControl(t,f_SystemOutput(x),f_SystemOutputDot(x))), TimeWindow, InitialSystemState,odeset('AbsTol',1e-6,'RelTol',1e-6));


%% Data for plots

%SystemOutput_fc  = f_SystemOutput(SystemStates);
SystemOutput_fc = zeros(1,length(t_fc));
ufc = zeros(1,length(t_fc));
TrackingError_FC = zeros(1,length(t_fc));
for i = 1:length(t_fc)
    SystemOutput_fc(i)= f_SystemOutput(state_fc(i,:)');
    ufc(i) = FunnelControl(t_fc(i),f_SystemOutput(state_fc(i,:)'),f_SystemOutputDot(state_fc(i,:)'));
    TrackingError_FC(i) = f_SystemOutput(state_fc(i,:)')-ReferenceSignal(t_fc(i));
end
%TrackingError_FC = SystemOutput_fc - ReferenceSignal(t_fc);

Reference_fc=ReferenceSignal(t_fc);
Psi_fc = Funnel(t_fc);

save('data_fc.mat', 't_fc','ufc','Psi_fc','TrackingError_FC');

%% Plots:

figure
hold on
    plot(t_fc,SystemOutput_fc,'linewidth',1.5)
    plot(t_fc,Reference_fc);
    plot(t_fc,Reference_fc + Psi_fc,'Color', 'r');
    plot(t_fc,Reference_fc - Psi_fc,'Color', 'r');
    xlim([InitialTime,10])
    %ylim([-2 2])

    ylabel('$y$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y$ of FMPC','$y_{\mathrm{ref}}$','Interpreter','latex')
hold off

figure
hold on 
    plot(t_fc,TrackingError_FC,'linewidth',1.5)
    plot(t_fc,Psi_fc,'Color', 'r');
    plot(t_fc,- Psi_fc,'Color', 'r');
    xlim([InitialTime,4])
    %ylim([-1 1])
    
    ylabel('$y-y_{\mathrm{ref}}$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y-y_{\mathrm{ref}}$ of FMPC','$\psi(t)$','Interpreter','latex')
hold off

figure
hold on
    plot(t_fc,ufc,'linewidth',2.5)
    axis tight
    xlim([InitialTime,10])

    ylabel('$u$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$u(t)$ of FC','Interpreter','latex')
hold off


function a = FunnelControlAlpha(x)
    a= 1./(1-x);
end

function u_FC = FunnelControl(time, Cur_Output,Cur_OutputDot)
    Phi = 1./Funnel(time);
    e = (Cur_Output-ReferenceSignal(time));
    edot = (Cur_OutputDot- ReferenceSignalDot(time));
    w = Phi*edot + FunnelControlAlpha(Phi.^2*e.^2)*Phi*e;
    u_FC = - 1*FunnelControlAlpha(w.^2)*w;
end

%State [y, yd, eta1, eta2]
function dz = ode_massoncar(Time, State, ControlValue)
    y = State(1);
    dy = State(2);
    eta = State(3:4);
    
    R1 =0;
    R2 = 8/9;
    S = -4*sqrt(2)/9* [2 1];
    Gam =1/9;
    Q =[0 1;
        -4 -2];
    P = 2*sqrt(2)*[1;0];
    dz = [dy;
          R1*y + R2*dy + S*eta + Gam .* ControlValue;
          Q*eta + P*y];
end

function y = ode_massoncar_output(State)
    y = [1 0 0 0]*State;
end

function y = ode_massoncar_outputdot(State)
    y = [0 1 0 0]*State;
end

