close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

%% MPC Parameter
T = 1;                 %Length of prediction horizion
N = 10;                %Number of discretisation steps of MPC control
M= 10000;              %Number of discretisation steps of integrator
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

f_Model       = @ode_chemicalreactor_lin;
f_ModelOutput = @ode_chemicalreactor_output;

%% Cost function
FMPCCoeff   = 1;                %Amplification of the funnel term
EnergyCoeff = 10^(-4);%10^(-4);  %10^(-1)        %Amplification of the energy/control term
f_error     = @(time,state) f_ModelOutput(state) - ReferenceSignal(time);
f_stagecost = GetFunnelCost(f_error,FMPCCoeff,EnergyCoeff);

f_constraint = @(time,state) ReferenceSignal(time)-Funnel(time)<=f_ModelOutput(state)<=ReferenceSignal(time)+Funnel(time);


%% Optimisation problem
%Dimensions
ModelStateDimension = length(InitialModelState);
ControlDimension = 1;


[ocp, X, U, J,OCPInit,t0] = BuildOCP(f_Model,f_stagecost, ModelStateDimension, ...
                                     ControlDimension,MaxControl, N, T, f_constraint);

%Set initial values for the solver


Iteration    = 1;
umpc         = [];
SystemStates = InitialSystemState;
ModelStates  = InitialModelState;

uFC          = [];
u_applied    = [];

CurrentTime  = InitialTime;

CurrentSystemState = InitialSystemState;
CurrentModelState = InitialModelState;
j = 1;
ControlInit = repmat(InitialControlValue,1,N);
ModelStatesInit = repmat(InitialModelState,1,N+1);
%% MPC Loop
while true

    %Set OCP initial state and time
    ocp.set_value(OCPInit, CurrentModelState);
    ocp.set_value(t0, CurrentTime);
    
    %Set initial values has hints for the solver
    %ocp.set_initial(X, ModelStatesInit);
    ocp.set_initial(U, ControlInit);

    %Solve OCP
    sol = ocp.solve();
    
    %Compute solution
    xopt  =   sol.value(X);         %State trajectory
    uopt  =   sol.value(U);         %Control values
    Jopt  =   sol.value(J);         %Optimal costs
    
    umpc = [umpc, uopt(1)];          
    
    %Predict model trajectory
    ModelTrajectory = rk4Integrator(@(t,x) f_Model(t,x,uopt(1)),CurrentTime,CurrentTime+h,M,CurrentModelState);
    ModelStates  = [ModelStates, ModelTrajectory];
    CurrentModelState = ModelTrajectory(:,end);
    
    %Callback to compute the current control value;
    %Avoids recomputation of current model output based on current time
    f_ControlCB = @(t,x,i) uopt(1)+ FunnelControlValue( ...
        AdaptiveFunnel(t,f_ModelOutput(ModelTrajectory(:,i))), ...
        f_SystemOutput(x)-f_ModelOutput(ModelTrajectory(:,i)));

    %Compute next system state
    SystemTrajectory = rk4Integrator(@(t,x,u) f_System(t,x,u) ...
        ,CurrentTime,CurrentTime+h,M,CurrentSystemState,f_ControlCB);
    SystemStates = [SystemStates,SystemTrajectory];
    CurrentSystemState = SystemTrajectory(:,end);
    
    %Store applied control values for plot
    tmp_h=h/M;
    for i=1:M 
        cur_FC =   FunnelControlValue( ...
            AdaptiveFunnel(CurrentTime+i*tmp_h,f_ModelOutput(ModelTrajectory(:,i))),...
            f_SystemOutput(SystemTrajectory(:,i))-f_ModelOutput(ModelTrajectory(:,i)));
        uFC(j) = cur_FC;
        u_applied(j) = cur_FC + uopt(1);
        j = j+1;
    end

    
    %Update model state with measurement
    %Only update model output with system output 
    %CurrentModelState(3) = f_SystemOutput(CurrentSystemState);

    %Set last control values as initial guesses for the next control to
    %help the optimiser
    ControlInit = [uopt(2:end),uopt(end)];
    ModelStatesInit = [xopt(:,2:end),xopt(:,end)];
    CurrentTime=CurrentTime+h;
    disp(CurrentTime);
    
    Iteration = Iteration + 1;

    if CurrentTime+h>=FinalTime
        break
    end    
end

%% Data for plots
%adding last control again to have a better plots with stairs function
umpc = [umpc, umpc(end)];                                                  
uFC = [uFC, uFC(end)];
u_applied = [u_applied, u_applied(end)];

TimeWindow  = [InitialTime FinalTime];
tstate      = InitialTime:h/M:(Iteration-1)*h;                              %Sampletimes of system states
tcontrol    = InitialTime:h:(Iteration-1)*h;                                %Sampletimes of control

SystemOutput  = f_SystemOutput(SystemStates);
ModelOutput = f_ModelOutput(ModelStates);

Reference=ReferenceSignal(tstate);
Psi = Funnel(tstate);

%Errors:
ModelError    = ModelOutput - ReferenceSignal(tstate);
TrackingError = SystemOutput - ReferenceSignal(tstate);


save('data_robustfmpc_nonrobust.mat', 'tstate', 'SystemOutput','ModelOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc','uFC','u_applied');




%% Plots:

figure
hold on
    plot(tstate,SystemOutput,'linewidth',1.5)
    plot(tstate,ModelOutput,'linewidth',1.5)
    plot(tstate,Reference);
    plot(tstate,Reference + Psi,'Color', 'r');
    plot(tstate,Reference - Psi,'Color', 'r');
    xlim([InitialTime,4])
    %ylim([-2 2])

    ylabel('$y$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y$ of FMPC','$y_{\mathrm{ref}}$','Interpreter','latex')
hold off

figure
hold on
    plot(tstate,TrackingError,'linewidth',1.5)
    plot(tstate, ModelError,'linewidth',1.5, 'Color', 'r','LineStyle',':')
    plot(tstate, + Psi,'Color', 'r');
    plot(tstate,- Psi,'Color', 'r');
    xlim([InitialTime,4])
    %ylim([-1 1])
    
    ylabel('$y-y_{\mathrm{ref}}$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y-y_{\mathrm{ref}}$ of FMPC','$\psi(t)$','Interpreter','latex')
hold off

figure
hold on
    stairs(tstate,u_applied,'linewidth',1.5, 'Color', 'k','LineStyle','-')
    stairs(tstate,uFC,'linewidth',1.5, 'Color', 'b','LineStyle','-')
    stairs(tcontrol,umpc,'linewidth',1.5, 'Color', 'r','LineStyle','-')
    axis tight
    xlim([InitialTime,4])

    ylabel('$u$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$u(t)$ of FMPC','Interpreter','latex')
hold off

%% Helper functions

function Phi = AdaptiveFunnel(t, ModelOutput)
    ModelError = norm(ModelOutput - ReferenceSignal(t));
    Phi = 1./(Funnel(t) - ModelError);
end

function a = FunnelControlAlpha(x)
    a= 1./(1-x);
end

function beta=ActivationFunction(Phi,CurError)
    if Phi * norm(CurError) < 0.4
        beta =0;
    else
        beta = Phi * norm(CurError) - 0.4;
    end
    beta = 0;
end

function u_FC = FunnelControlValue (Cur_Funnel_Phi, Cur_Error)
    w = Cur_Funnel_Phi * Cur_Error;
    u_FC = - ActivationFunction(Cur_Funnel_Phi, Cur_Error)*FunnelControlAlpha(norm(w)^2)*w;
end