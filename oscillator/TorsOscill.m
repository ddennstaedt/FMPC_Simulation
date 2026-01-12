close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

Scenario = 1;

%% MPC parameters
if Scenario == 0
    tau = 0.002;
    MaxControl = 267;
    T = 10*tau;            %Length of prediction horizion
else 
    tau = 0.2;
    MaxControl = 30;
    T = 1;
end

N=floor(T/tau)+1;      %Number of discretisation steps of MPC control
DiscretSteps= 300;     %Number of discretisation steps of integrator
h=tau;                 %MPC sampling rate
InitialTime = 0;       %Initial time of simulation
FinalTime   = 8.2;     %End time of simulation



%% Paramter Torsional Oscillator

I_1     = 0.136;
I_2     = 0.12;
k       = 0.1;
d       = 0.16;

M = [1 0 0 
     0 I_1 0
     0 0 I_2];

M_inv = M\eye(size(M));

A_bar = [0 1 -1
        -k -d d
         k d -d];

B_bar = [0;1;0];

A = M_inv * A_bar ;
B = M_inv * B_bar;

C = [0 1 0];


%% Initialisation
InitialControlValue = 0;

InitialModelState  = [0; 0; 0];
InitialSystemState = [0; 0; 0];


%% ODE (model and system)

f_System = @(t,x,u) A*x+B*u;
f_SystemOutput = @(x) C*x;

f_Model       = @(t,x,u) A*x+B*u;
f_ModelOutput = @(x) C*x;

%% Cost function
EnergyCoeff = 10^(-1);          %Amplification of the energy/control term
f_error = @(time, state) f_ModelOutput(state)-ReferenceSignal(time);
f_stagecost =  @(time, state, control) f_error(time,state)^2/(1+ min([sign(Funnel(time)^2-f_error(time,state)^2),0])) + EnergyCoeff*control^2;



%% Optimisation problem
ModelStateDimension = length(InitialModelState);
ControlDimension = length(C*B);

[ocp, X, U, J,OCPInit,t0] = BuildOCP(f_Model,f_stagecost,ModelStateDimension,ControlDimension,MaxControl, N, T);

%% MPC Loop

Iteration    = 1;
umpc         = [];
SystemStates = InitialSystemState;
ModelStates  = InitialModelState;

CurrentTime  = InitialTime;

CurrentSystemState = InitialSystemState;
CurrentModelState = InitialModelState;

InitialControl = repmat(InitialControlValue,1,N);
%Set initial values for the solver
ocp.set_initial(X, 0*repmat(CurrentModelState,1,N+1));
ocp.set_initial(U, InitialControl);
while true

    %Set OCP initial state and time
    ocp.set_value(OCPInit, CurrentModelState);
    ocp.set_value(t0, CurrentTime);
    
    %Solve OCP
    sol = ocp.solve();
    
    %Compute solution
    xopt  =   sol.value(X);         %State trajectory
    uopt  =   sol.value(U);         %Control values
    Jopt  =   sol.value(J);         %Optimal costs
    
    umpc = [umpc, uopt(1)];          
    
    %Compute next system state
    SystemTrajectory=rk4Integrator(@(t,x) f_System(t,x,uopt(1)),CurrentTime,CurrentTime+h,DiscretSteps,CurrentSystemState);
       
    SystemStates = [SystemStates,SystemTrajectory];
    CurrentSystemState = SystemTrajectory(:,end);
    %Compute next model state 
    ModelTrajectory = SystemTrajectory;
    %ModelStates  = [ModelStates, ModelTrajectory];
    CurrentModelState = ModelTrajectory(:,end);

    
    %Set last control values and model state
    %as initial guesses for the next iteration to help the optimiser
    ocp.set_initial(X,[xopt(:,2:end),xopt(:,end)]);
    ocp.set_initial(U,[uopt(2:end),uopt(end)]);
    
    CurrentTime=CurrentTime+h;
    disp(CurrentTime);
    
    Iteration = Iteration + 1;

    if CurrentTime+h>=FinalTime
        break
    end    
end

%% Data for plots
umpc = [umpc, umpc(end)];                                                   %adding last control again to have a better plot with stairs function

TimeWindow  = [InitialTime FinalTime];
tstate      = InitialTime:h/DiscretSteps:(Iteration-1)*h;                   %Sampletimes of system states
tcontrol    = InitialTime:h:(Iteration-1)*h;                                %Sampletimes of control

SystemOutput  = f_SystemOutput(SystemStates);
TrackingError = SystemOutput - ReferenceSignal(tstate);

Reference=ReferenceSignal(tstate);
Psi = Funnel(tstate);


if Scenario == 0
    save('data_fmpc_scen0.mat', 'tstate', 'SystemOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc');
else
    tstate_1 = tstate;
    SystemOutput_1 = SystemOutput;
    Reference_1 = Reference;
    Psi_1 = Psi;
    tcontrol_1 = tcontrol;
    umpc_1 = umpc;
    InitialTime_1 = InitialTime;
    FinalTime_1 = FinalTime;
    save('data_fmpc_scen1.mat', 'tstate_1', 'SystemOutput_1', 'Reference_1', 'Psi_1', 'tcontrol_1', 'InitialTime_1','FinalTime_1','umpc_1');
end
%% Plots:

figure
hold on
    stairs(tcontrol,umpc,'linewidth',1.5)
    axis tight
    xlim([InitialTime,FinalTime])
    ylim([0 30])
    xlim([0 8])
    ylabel('Input','interpreter','latex','FontSize',16)
    xlabel('time $t$','interpreter','latex','FontSize',16)
    xticks([0 2 4 6 8])
    yticks([0 10 20 30])
    legend('$u_{\rm FMPC}(t)$','Interpreter','latex','FontSize',26,'Location','northwest')
hold off

figure
    plot(tstate,SystemOutput,'r')
    hold on
    plot(tstate,Reference,'b')
    plot(tstate,Reference+Psi,'k--')
    plot(tstate,Reference-Psi,'k--')
    xlim([0 8])
        xlabel('time $t$','interpreter','latex','FontSize',16)
        ylabel('Angular velocity~$\dot z_1(t)$','interpreter','latex','FontSize',16)
        xticks([0 2 4 6 8])
        yticks([0 100 200 300])
    legend('$y(t)$','$y_{\rm ref}(t)$','$\psi$','Interpreter','latex','FontSize',26,'Location','northwest')
hold off
