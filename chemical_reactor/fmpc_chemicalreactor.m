close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

Scenario = 2;
% Scenario 0: FMPC with horizon 0.01, 
% Scenario 1: FMPC with horizon 1,
% Scenario 2: MPC with horizon  0.01,
% Scenario 3: MPC with horizon  1,
%% MPC parameters
if 0 == Scenario || 2 == Scenario
    T = 0.01;                     %Length of prediction horizion
    N = 20;                       %Number of discretisation steps of MPC control
    M = 10;                       %Number of discretisation steps of integrator
    EnergyCoeff = 10^(-1);        %Amplification of the energy/control term
elseif 1 == Scenario || 3 == Scenario
    T = 1;                        %Length of prediction horizion
    N = 10;                       %Number of discretisation steps of MPC control
    M = 100;                      %Number of discretisation steps of integrator
    EnergyCoeff = 10^(-4);        %Amplification of the energy/control term
end

h=T/N;                 %MPC sampling rate
InitialTime = 0;       %Initial time of simulation
FinalTime   = 4.1;     %End time of simulation

MaxControl = 600;

%% Initialisation
InitialControlValue = 410;

InitialModelState  = [0.02; 0.9; 270];
InitialSystemState = [0.02; 0.9; 270];

%% ODE (model and system)

f_System = @ode_chemicalreactor;
f_SystemOutput = @ode_chemicalreactor_output;

f_Model       = @ode_chemicalreactor;
f_ModelOutput = @ode_chemicalreactor_output;

%% Cost function
FMPCCoeff   = 1;                %Amplification of the funnel term
f_error     = @(time,state) f_ModelOutput(state) - ReferenceSignal(time);
if 0 == Scenario || 1 == Scenario
    f_stagecost = GetFunnelCost(f_error,FMPCCoeff,EnergyCoeff);
else
    % quadratic stage cost
    f_stagecost =  @(time, state, control) FMPCCoeff*f_error(time,state)^2 + EnergyCoeff*norm(control-360)^2;
end

%% Optimisation problem
ModelStateDimension = length(InitialModelState);
ControlDimension = 1;

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
ocp.set_initial(X, repmat(CurrentModelState,1,N+1));
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
    SystemTrajectory=rk4Integrator(@(t,x) f_System(t,x,uopt(1)),CurrentTime,CurrentTime+h,M,CurrentSystemState);
       
    SystemStates = [SystemStates,SystemTrajectory];
    CurrentSystemState = SystemTrajectory(:,end);
    %Compute next model state 
    ModelTrajectory = rk4Integrator(@(t,x) f_Model(t,x,uopt(1)),CurrentTime,CurrentTime+h,M,CurrentModelState);
    ModelStates  = [ModelStates, ModelTrajectory];
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
tstate      = InitialTime:h/M:(Iteration-1)*h;                              %Sampletimes of system states
tcontrol    = InitialTime:h:(Iteration-1)*h;                                %Sampletimes of control

SystemOutput  = f_SystemOutput(SystemStates);
ModelOutput = f_ModelOutput(ModelStates);

ModelMismatch = SystemOutput- ModelOutput;

ModelError    = ModelOutput - ReferenceSignal(tstate);
TrackingError = SystemOutput - ReferenceSignal(tstate);

Reference=ReferenceSignal(tstate);
Psi = Funnel(tstate);

tstate_m = tstate;
SystemOutput_m = SystemOutput;
Reference_m = Reference;
Psi_m = Psi;
tcontrol_m = tcontrol;
umpc_m = umpc;
InitialTime_m = InitialTime;
FinalTime_m = FinalTime;

if 0 == Scenario
    save('data_fmpc_short.mat', 'tstate', 'SystemOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc');
elseif 1 == Scenario
    save('data_fmpc_long.mat', 'tstate', 'SystemOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc');
elseif 2 == Scenario
    save('data_mpc_quadratic_short.mat', 'tstate_m', 'SystemOutput_m', 'Reference_m', 'Psi_m', 'tcontrol_m', 'InitialTime_m','FinalTime_m','umpc_m');
elseif 3 == Scenario
    save('data_mpc_quadratic_long.mat', 'tstate_m', 'SystemOutput_m', 'Reference_m', 'Psi_m', 'tcontrol_m', 'InitialTime_m','FinalTime_m','umpc_m');
end

%% Plots:

figure
hold on
    plot(tstate,SystemOutput,'linewidth',1.5)
    plot(tstate,Reference);
    plot(tstate,Reference + Psi,'Color', 'r');
    plot(tstate,Reference - Psi,'Color', 'r');
    xlim([InitialTime,4])

    ylabel('$y$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y$ of FMPC','$y_{\mathrm{ref}}$','Interpreter','latex')
hold off

figure
hold on
    plot(tstate,TrackingError,'linewidth',1.5)
    plot(tstate, ModelError,'linewidth',1.5, 'Color', 'r','LineStyle',':')
    plot(tstate,Psi,'Color', 'r');
    plot(tstate,- Psi,'Color', 'r');
    xlim([InitialTime,4])

    ylabel('$y-y_{\mathrm{ref}}$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y-y_{\mathrm{ref}}$ of FMPC','$\psi(t)$','Interpreter','latex')
hold off

figure
hold on
    stairs(tcontrol,umpc,'linewidth',1.5)
    axis tight
    xlim([InitialTime,4])

    ylabel('$u$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$u(t)$ of FMPC','Interpreter','latex')
hold off
