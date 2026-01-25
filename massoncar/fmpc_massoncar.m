close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

Scenario = 2;
%Scenario 0: FMPC with lambda = 10^-3
%Scenario 1: FMPC with lambda = 10^-4
%Scenario 2: MPC  with lambda = 10^-4

%% MPC parameters
T = 1;                 %Length of prediction horizion
N = 10;                %Number of discretisation steps of MPC control
M = 100;               %Number of discretisation steps of integrator
h=T/N;                 %MPC sampling rate
InitialTime = 0;       %Initial time of simulation
FinalTime   = 10.1;    %End time of simulation
if 0 == Scenario
    EnergyCoeff = 10^(-3);  %Amplification of the energy/control term
else
    EnergyCoeff = 10^(-4);
end


MaxControl = 30;
%% Initialisation
InitialControlValue = 0;

InitialModelState  = [0; 0; 0; 0];
InitialSystemState = [0; 0; 0; 0];

%% ODE (model and system)

f_System = @ode_massoncar;
f_SystemOutput = @ode_massoncar_output;

f_Model       = @ode_massoncar;
f_ModelOutput = @ode_massoncar_output;
f_ModelOutputDot = @ode_massoncar_outputdot;

%% Auxilliary funnel
f_error = @(time, state) f_ModelOutput(state) - ReferenceSignal(time);
f_errordot = @(time, state) f_ModelOutputDot(state) - ReferenceSignalDot(time);

InitialError = abs(f_error(InitialTime,InitialModelState));
InitialErrorDot = abs(f_errordot(InitialTime,InitialModelState));
InitialFunnel = Funnel(InitialTime);

alpha = 2;
bet = 0.2;
Gam =0.2;
k_1 =2*InitialErrorDot/(Gam*(1-Gam)*InitialFunnel)+2*(alpha+1/Gam)/(1-Gam);

if InitialError>= Gam*InitialFunnel
    disp('error')
    return;
end
f_auxfunnel = @(t) 1/Gam*(InitialErrorDot+k_1*InitialError)*exp(-alpha * t ) + bet/(alpha*Gam);

%% Cost function
FMPCCoeff   = 1;                 %Amplification of the funnel term

f_aux_error     = @(time,state) f_errordot(time,state) - k_1.*f_error(time,state);
%% Optimisation problem
ModelStateDimension = length(InitialModelState);
ControlDimension = 1;
if 2 == Scenario
    %quadratic stagecost
    f_stagecost = @(time, state, control) FMPCCoeff*abs(f_error(time,state))^2 + EnergyCoeff*abs(control)^2;
    f_constraint = @(time,state) ReferenceSignal(time)-Funnel(time)<=f_ModelOutput(state)<=ReferenceSignal(time)+Funnel(time);
    [ocp, X, U, J,OCPInit,t0] = BuildOCP(f_Model,f_stagecost,ModelStateDimension,ControlDimension,MaxControl, N, T,f_constraint);
else
    f_stagecost = @(time, state, control) FMPCCoeff*abs(f_aux_error(time,state))^2/((f_auxfunnel(time)^2-abs(f_aux_error(time,state))^2)) + EnergyCoeff*abs(control)^2;
    [ocp, X, U, J,OCPInit,t0] = BuildOCP(f_Model,f_stagecost,ModelStateDimension,ControlDimension,MaxControl, N, T);
end 

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

if 0 == Scenario
    save('data_fmpc_10-3.mat', 'tstate', 'TrackingError', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc');
elseif 1 == Scenario
    save('data_fmpc_10-4.mat', 'tstate', 'TrackingError', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc');
else
    tstate_m = tstate;
    SystemOutput_m = SystemOutput;
    Reference_m = Reference;
    Psi_m = Psi;
    tcontrol_m = tcontrol;
    umpc_m = umpc;
    TrackingError_m = TrackingError;
    InitialTime_m = InitialTime;
    FinalTime_m = FinalTime;
    save('data_mpc_quadratic.mat', 'tstate_m', 'TrackingError_m', 'Psi_m', 'tcontrol_m', 'InitialTime_m','FinalTime_m','umpc_m')
end


%% Plots:

figure
hold on
    plot(tstate,SystemOutput,'linewidth',1.5)
    plot(tstate,Reference);
    plot(tstate,Reference + Psi,'Color', 'r');
    plot(tstate,Reference - Psi,'Color', 'r');
    xlim([InitialTime,10])
    %ylim([-2 2])

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
    xlim([InitialTime,10])
    ylim([-1 1])
    
    ylabel('$y-y_{\mathrm{ref}}$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y-y_{\mathrm{ref}}$ of FMPC','$\psi(t)$','Interpreter','latex')
hold off

figure
hold on
    stairs(tcontrol,umpc,'linewidth',1.5)
    axis tight
    xlim([InitialTime,10])

    ylabel('$u$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$u(t)$ of FMPC','Interpreter','latex')
hold off
