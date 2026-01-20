close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*
Scenario = 1;
% Scenario 0: Model update every 5 iterations
% Scenario 1: Model update every 3 iterations

%% MPC Parameter
T = 1;                 %Length of prediction horizion
N = 10;                %Number of discretisation steps of MPC control
M= 1000;               %Number of discretisation steps of integrator
h=T/N;                 %MPC sampling rate
InitialTime = 0;       %Initial time of simulation
FinalTime   = 4.1;     %End time of simulation
MaxControl = 600;
if 0 == Scenario
    ModelUpdateIter = 5;   %Update Model every X iterations
else
    ModelUpdateIter = 3;   %Update Model every X iterations
end
%% Initialisation
InitialControlValue = 450;

%% ODE system

f_System = @ode_chemicalreactor;
f_SystemOutput = @ode_chemicalreactor_output;

%% Cost function
FMPCCoeff   = 1;                %Amplification of the funnel term
EnergyCoeff = 10^(-4);          %Amplification of the energy/control term

InitialSystemState = [0.02; 0.9; 270];

%% ODE model x^+ = Ax +Bu +D; y =Cx
% Initial model 
A_mod = [0 0 0;
         0 0 0;
         0 0 0];
B_mod = [0; 0; 1];
D_mod = [0; 0; 0];
C_mod = [0 0 1];
InitialModelState  = [0.02; 0.9; 270];



Iteration    = 1;
SystemStates = InitialSystemState;
ModelStates  = InitialModelState;

umpc         = []; %control from the MPC component
uFC          = []; %control from the FC component
u_applied    = []; %control applied to the system

u_optim      = []; %control applied to the system in last iteration

OutputOptim  = f_SystemOutput(InitialSystemState);

CurrentTime  = InitialTime;

CurrentSystemState = InitialSystemState;
CurrentModelState = InitialModelState;
j = 1;

ControlInit = repmat(InitialControlValue,1,N);
ModelStatesInit = repmat(InitialModelState,1,N+1);
%% MPC Loop
while true
    %% Optimisation problem

    f_Model       = @(t,x,u) A_mod*x + B_mod*u + D_mod;
    f_ModelOutput = @(x) C_mod*x;
    
    f_error     = @(time,state) f_ModelOutput(state) - ReferenceSignal(time);
    f_stagecost = GetFunnelCost(f_error,FMPCCoeff,EnergyCoeff);

    f_constraint = @(time,state) ReferenceSignal(time)-Funnel(time)<=f_ModelOutput(state)<=ReferenceSignal(time)+Funnel(time);


    %Dimensions

    ModelStateDimension = length(InitialModelState);
    ControlDimension = 1;
    
    [ocp, X, U, J,OCPInit,t0] = BuildOCP(f_Model,f_stagecost, ...
                                         ModelStateDimension, ControlDimension,MaxControl, N, T,f_constraint);
   

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
        if i == 1
            u_optim= u_applied(j);
            OutputOptim = [OutputOptim(end),f_SystemOutput(CurrentSystemState)]; %[f_SystemOutput(SystemTrajectory)]; %
        end
        j = j+1;
    end

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
    if mod(Iteration, ModelUpdateIter) == 0 %update model every  iterations
        [A_mod, B_mod, C_mod, D_mod, x0, XN] = LearnModel(OutputOptim, u_optim, CurrentTime-h, CurrentTime, M);
        CurrentModelState = XN;
    else
        CurrentModelState = ModelTrajectory(:,end);
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

%Errors:
ModelError    = ModelOutput - ReferenceSignal(tstate);
TrackingError = SystemOutput - ReferenceSignal(tstate);

Reference=ReferenceSignal(tstate);
Psi = Funnel(tstate);

if 0 == Scenario
    save('data_learning_fmpc.mat', 'tstate', 'SystemOutput','ModelOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc','uFC','u_applied');
else
    save('data_learning_fmpc_often.mat', 'tstate', 'SystemOutput','ModelOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc','uFC','u_applied');
end 
%% Plots:

figure
hold on
    plot(tstate,SystemOutput,'linewidth',1.5)
    plot(tstate,ModelOutput,'linewidth',1.5)
    fplot(@(x) ReferenceSignal(x)                   ,TimeWindow);
    fplot(@(x) ReferenceSignal(x) + Funnel(x),TimeWindow,'Color', 'r');
    fplot(@(x) ReferenceSignal(x) - Funnel(x),TimeWindow,'Color', 'r');
    xlim([InitialTime,4])

    ylabel('$y$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$y$ of FMPC','$y_{\mathrm{ref}}$','Interpreter','latex')
hold off

figure
hold on
    plot(tstate,TrackingError,'linewidth',1.5)
    plot(tstate, ModelError,'linewidth',1.5, 'Color', 'r','LineStyle',':')
    fplot(@(x)   Funnel(x),TimeWindow,'Color', 'r');
    fplot(@(x) - Funnel(x),TimeWindow,'Color', 'r');
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

function [c,ceq] = Contraints(OParam)
    rbar = 1.3;
    sbar = 1.4;
    %gammabar = 1;
    pbar = 1/400;
    etabar = 0.91;

    dbar= 2.5;
    ybar =  400;

    c(1) = OParam(10)^2 + OParam(11)^2 - dbar^2;
    c(2) = OParam(12)^2 - dbar^2;

    c(3) = OParam(9)^2-rbar^2;
    c(4) = OParam(7)^2 + OParam(8)^2-sbar^2;

    c(5) = OParam(3)^2+OParam(6)^2 -pbar^2;


    c(6) = eigs([OParam(1) OParam(2);
         OParam(4) OParam(5)],1) + (pbar*ybar+dbar)/etabar;
    ceq =[];
end


function [A,B,C,D,x0, xN] = LearnModel(SystemOutput, ControlInput, t0, tf, M)

    OParam = zeros(1,12); 

    OptimOptions = optimset('UseParallel',true);
    
    nonlcon = @Contraints;
    OParam = fmincon(@(Cur_Param)PredictionError(Cur_Param, t0, tf, M, ControlInput, SystemOutput), ...
                                                 OParam,[],[],[],[],[],[],nonlcon,OptimOptions);
    [A,B,C,D,x0] = Param2Matrix(OParam, SystemOutput(1));
    f_control = @(t,x,i) ControlInput(:,1);
    ModelStates = rk4Integrator(@(t,x,u) A*x+B*u+D, t0, tf, M , x0, f_control);
    xN=ModelStates(:,end);
end

function E = PredictionError(OParam, t0, tf,  M, u_applied, SystemOutput)
    [A,B,C,D,x_0] = Param2Matrix(OParam, SystemOutput(1));
    f_ode = @(t,x,u) A*x+B*u+D;
    f_Output = @(x) C*x;
    f_control = @(t,x,i) u_applied(:,1);
    ModelStates = rk4Integrator(f_ode, t0, tf, M , x_0, f_control);
    ModelOutput = [f_Output(ModelStates)];
    E = norm(SystemOutput(end) - ModelOutput(end))^2;
end

function [A,B,C,D,x_0] = Param2Matrix(OParam, y_0)
    A = [OParam(1) OParam(2) OParam(3);
         OParam(4) OParam(5) OParam(6);
         OParam(7) OParam(8) OParam(9)];
    D = [OParam(10);OParam(11);OParam(12)];

    %We do not update B and C
    B = [0;0;1]; 
    C = [0 0 1]; 
    x_0 = [0.02; 0.9; y_0];
end

function Phi = AdaptiveFunnel(t, ModelOutput)
    ModelError = norm(ModelOutput - ReferenceSignal(t));
    Phi = 1./(Funnel(t) - ModelError);
end

function a = FunnelControlAlpha(x)
    a= 1./(1-x);
end

function u_FC = FunnelControlValue (Cur_Funnel_Phi, Cur_Error)
    w = Cur_Funnel_Phi * Cur_Error;
    u_FC = - 10*FunnelControlAlpha(norm(w)^2)*w;
end