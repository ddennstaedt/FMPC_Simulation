close all
clear all

addpath('~/Documents/workspace/casadi-3.7.0');
addpath('../util/')
import casadi.*
import casadi.tools.*

%% MPC Parameter
T = 0.5;%1;%0.01;                 %Length of prediction horizion
N = 20;%10;%20;                %Number of discretisation steps of MPC control
M = 100;%100;%10;              %Number of discretisation steps of integrator
h=T/N;                 %MPC sampling rate
InitialTime = 0;       %Initial time of simulation
FinalTime   = 10;     %End time of simulation

MaxControl = 30;

%% Initialisation
InitialControlValue = 0;

InitialModelState  = [0; 0; 0; 0];
InitialSystemState = [0; 0; 0; 0];


%% ODE system

f_System = @ode_massoncar_dist;
f_SystemOutput = @ode_massoncar_output;
f_SystemOutputDot = @ode_massoncar_outputdot;
%% Cost function
FMPCCoeff   = 1;                %Amplification of the funnel term
EnergyCoeff = 10^(-4);%10^(-4);  %10^(-1)        %Amplification of the energy/control term

%% Auxilliary funnel
InitialError = 1;
InitialErrorDot = 0;
InitialFunnel = Funnel(InitialTime);
global k_1;
alpha = 2;
bet = 0.2;
Gam =0.2;
k_1 =2*InitialErrorDot/(Gam*(1-Gam)*InitialFunnel)+2*(alpha+1/Gam)/(1-Gam);

if InitialError>= Gam*InitialFunnel
    disp('error')
    return;
end
f_auxfunnel = @(t) 1/Gam*(InitialErrorDot+k_1*InitialError)*exp(-alpha * t ) + bet/(alpha*Gam);


%% ODE model x^+ = Ax +Bu +D; y =Cx
% Initial model 
CurParams = [1; 4; 4; 0.5];
Params= [CurParams];
[R1,R2,S,Gam,Q,P] = MassOnCarMatrices(6, 2, 3, 0.75);



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

    f_Model       = @(time,state,control) [state(2);...
                               R1*state(1)+R2*state(2)+ S*state(3:4) + Gam .* control;...
                               Q*state(3:4) + P*state(1)];
    f_ModelOutput = @(state)   [1 0 0 0]*state;
    f_ModelOutputDot =@(state) [0 1  0 0]*state;

    
    f_error     = @(time,state) f_ModelOutput(state) - ReferenceSignal(time);
    f_errordot = @(time, state) f_ModelOutputDot(state) - ReferenceSignalDot(time);
    f_aux_error     = @(time,state) f_errordot(time,state) - k_1.*f_error(time,state);
    f_stagecost = @(time, state, control) FMPCCoeff*abs(f_aux_error(time,state))^2/((f_auxfunnel(time)^2-abs(f_aux_error(time,state))^2)) + EnergyCoeff*abs(control)^2;

    %f_constraint = @(time,state) ReferenceSignal(time)-Funnel(time)<=f_ModelOutput(state)<=ReferenceSignal(time)+Funnel(time);


    %Dimensions

    ModelStateDimension = length(InitialModelState);
    ControlDimension = 1;
    
    [ocp, X, U, J,OCPInit,t0] = BuildOCP(f_Model,f_stagecost, ...
                                         ModelStateDimension, ControlDimension,MaxControl, N, T);
   

    %Set OCP initial state and time
    ocp.set_value(OCPInit, CurrentModelState);
    ocp.set_value(t0, CurrentTime);

    %Set initial values has hints for the solver
    ocp.set_initial(X, ModelStatesInit);
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
        f_SystemOutput(x)-f_ModelOutput(ModelTrajectory(:,i)), ...
        f_SystemOutputDot(x)-f_ModelOutputDot(ModelTrajectory(:,i)));
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
            f_SystemOutput(SystemTrajectory(:,i))-f_ModelOutput(ModelTrajectory(:,i)), ...
            f_SystemOutputDot(SystemTrajectory(:,i))-f_ModelOutputDot(ModelTrajectory(:,i)));
        uFC(j) = cur_FC;
        u_applied(j) = cur_FC + uopt(1);
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
    if mod(Iteration, 20) == 0 %update model every 10 iterations
        [R1,R2,S,Gam,Q,P, Mod,CurParams] = LearnModel(f_SystemOutput(SystemStates), u_applied, CurrentTime, M*(Iteration-1),CurParams);
        Params=[Params,CurParams];
        
        CurrentModelState = Mod;
        %if ~CanInitialiseWithData(CurrentTime, AuxFunnel,CurrentModelState(1),CurrentModelState(2),SystemOutput,SystemOutputDot)
            [CurrentModelState(1), CurrentModelState(2)] = InitialiseModel(CurrentTime, f_auxfunnel(CurrentTime),f_SystemOutput(CurrentSystemState),f_SystemOutputDot(CurrentSystemState));
        %end
    else
        CurrentModelState = ModelTrajectory(:,end);
        %CurrentModelState(1) = CurrentSystemState(3);
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



save('data_learning_fmpc_dist_1.mat', 'tstate', 'SystemOutput','ModelOutput', 'Reference', 'Psi', 'tcontrol', 'InitialTime','FinalTime','umpc','uFC','u_applied','ModelError','TrackingError','Params');

%% Plots:

figure
hold on
    plot(tstate,SystemOutput,'linewidth',1.5)
    plot(tstate,ModelOutput,'linewidth',1.5)
    fplot(@(x) ReferenceSignal(x)                   ,TimeWindow);
    fplot(@(x) ReferenceSignal(x) + Funnel(x),TimeWindow,'Color', 'r');
    fplot(@(x) ReferenceSignal(x) - Funnel(x),TimeWindow,'Color', 'r');
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
    plot(tstate,-Psi,'Color', 'r');
    xlim([InitialTime,10])
    ylim([-1 1])
    
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
    xlim([InitialTime,10])

    ylabel('$u$','interpreter','latex')
    xlabel('time $t$','interpreter','latex')

    legend('$u(t)$ of FMPC','Interpreter','latex')
hold off

%% Helper functions

function [R1,R2,S,Gam,Q,P, Mod, Param] = LearnModel(SystemOutput, ControlInput, time, M,CurParam)
    Param = CurParam;
    lb = [0.5; 0.5; 0.5; 0.5];
    ub = [10; 10;  5; 5];
   
    OptimOptions = optimset('MaxFunEvals',10000, 'MaxIter',1000);

    Param = fmincon(@(Cur_Param)PredictionError(Cur_Param, time, M, ControlInput, SystemOutput), ...
                                                Param,[],[],[],[],lb,ub,[],OptimOptions);
    
    [R1,R2,S,Gam,Q,P] = MassOnCarMatrices(Param(1),Param(2),Param(3),Param(4));
    f_Model  = @(time,state,control) [state(2);...
                                   R1*state(1)+R2*state(2)+ S*state(3:4) + Gam .* control;...
                                   Q*state(3:4) + P*state(1)];
    f_ControlCB = @(t,x,i) ControlInput(i);
    InitialModelState  = [0; 0; 0; 0];
    ModelStates=rk4Integrator(@(t,x, u) f_Model(t,x,u), ...
                                                0, ...
                                                time, ...
                                                M, ...
                                                InitialModelState, ...
                                                f_ControlCB);    
    Mod=ModelStates(:,end);
end

function E = PredictionError(OParam, CurrentTime,  M, u_applied, SystemOutput)
    [R1,R2,S,Gam,Q,P] = MassOnCarMatrices(OParam(1),OParam(2),OParam(3),OParam(4));
    f_Model  = @(time,state,control) [state(2);...
                                   R1*state(1)+R2*state(2)+ S*state(3:4) + Gam .* control;...
                                   Q*state(3:4) + P*state(1)];
    f_ModelOutput = @(state)   [1 0 0 0]*state;
    f_ControlCB = @(t,x,i) u_applied(i);
    InitialModelState  = [0; 0; 0; 0];
    ModelStates=rk4Integrator(@(t,x, u) f_Model(t,x,u), ...
                                        0, ...
                                        CurrentTime, ...
                                        M, ...
                                        InitialModelState, ...
                                        f_ControlCB);    
    ModelOutput = [f_ModelOutput(InitialModelState),f_ModelOutput(ModelStates)];
    E = norm(SystemOutput - ModelOutput)^2;
end


function Phi = AdaptiveFunnel(t, ModelOutput)
    ModelError = norm(ModelOutput - ReferenceSignal(t));
    Phi = 1./(Funnel(t) - ModelError);
end

function a = FunnelControlAlpha(x)
    a= 1./(1-x);
end

function beta=ActivationFunction(Phi,CurError)
    beta = 2;
end

function u_FC = FunnelControlValue (Cur_Funnel_Phi, Cur_Error, Cur_ErrorDot)
    w = Cur_Funnel_Phi*Cur_ErrorDot + FunnelControlAlpha(Cur_Funnel_Phi.^2*Cur_Error.^2)*Cur_Funnel_Phi*Cur_Error;
    u_FC = - ActivationFunction(Cur_Funnel_Phi, Cur_Error)*FunnelControlAlpha(w.^2)*w;
end

function b = CanInitialiseWithData(time, AuxFunnel,ModOutput, ModOutputDot,SystemOutput,SystemOutputDot)
    global k_1;
    b = true;
    lambda = 0.99;
    eps =0.99;
    ErrorMod = ModOutput-ReferenceSignal(time);
    ErrorModDot = ModOutputDot-ReferenceSignalDot(time);
    AuxErrorMod = ErrorModDot+k_1*ErrorMod;    
    if abs(ErrorMod)>=lambda*Funnel(time)
        b = false;
    elseif abs(AuxErrorMod)>=AuxFunnel
        b = false;
    end
    Cur_Fun = Funnel(time);
    AdaptedFun = Funnel(time)-ErrorMod;
    AdaptedPhi = 1./AdaptedFun;
    Error = ModOutput-SystemOutput;
    ErrorDot = ModOutputDot-SystemOutputDot;
    if AdaptedPhi*abs(Error)>= eps
        b = false;
    elseif (AdaptedPhi*ErrorDot + FunnelControlAlpha(AdaptedPhi.^2*Error.^2)*AdaptedPhi*Error)>=eps
        b = false;
    end
end

function [c,ceq] = InitContraints(time,Param,AuxFunnel,SysOutput,SysOutputDot)
    global k_1;
    lambda = 0.99;
    eps =0.99;

    CurErrorMod =  Param(1)-ReferenceSignal(time);
    CurErrorModDot = Param(2)-ReferenceSignalDot(time);
    CurAuxErrorMod = CurErrorModDot+ k_1*CurErrorMod;

    c(1) = abs(CurErrorMod).^2-lambda*Funnel(time).^2;
    c(2) = abs(CurAuxErrorMod).^2-AuxFunnel.^2;

    AdaptedFunnel = Funnel(time) - abs(CurErrorMod);
    AdaptedPhi= 1./AdaptedFunnel;

    CurErrorSys = SysOutput - Param(1);
    CurErrorSysDot = SysOutputDot - Param(2);

    c(3) = (AdaptedPhi*abs(CurErrorSys)).^2- eps.^2;
    
    w =AdaptedPhi*(CurErrorSysDot) + FunnelControlAlpha(AdaptedPhi.^2*CurErrorSys.^2)*AdaptedPhi*CurErrorSys;
    c(4) = abs(w).^2-eps.^2;
    ceq =[];
end

function [ModOutput,ModOutputDot] = InitialiseModel(time,AuxFunnel,SystemOutput,SystemOutputDot)
    if CanInitialiseWithData(time, AuxFunnel,SystemOutput,SystemOutputDot,SystemOutput,SystemOutputDot)
        %we don't need to optimise if we can just put the model directly on
        %the  system data
        ModOutput = SystemOutput;
        ModOutputDot = SystemOutputDot;
    else
        
        Ref = ReferenceSignal(time);
        RefDot = ReferenceSignalDot(time);

        if ~CanInitialiseWithData(time, AuxFunnel,Ref,RefDot,SystemOutput,SystemOutputDot)
            disp('fatal error');
        end

        nonlcon = @(cur_param) InitContraints(time,cur_param,AuxFunnel,SystemOutput,SystemOutputDot);
    
        OParam =[Ref,RefDot];
        options = optimoptions("fmincon",...
        Algorithm="interior-point",...
        EnableFeasibilityMode=true,...
        SubproblemAlgorithm="cg");
        OParam = fmincon(@(Cur_Param)abs(Cur_Param(1)-SystemOutput)^2+1*abs(Cur_Param(2)-SystemOutputDot)^2, ...
                                                     OParam,[],[],[],[],[],[],nonlcon,options);
        ModOutput = OParam(1);
        ModOutputDot = OParam(2);
        if ~CanInitialiseWithData(time, AuxFunnel,ModOutput,ModOutputDot,SystemOutput,SystemOutputDot)
            [c,ceq] = InitContraints(time,OParam,AuxFunnel,SystemOutput,SystemOutputDot);
            disp('fatal error');
        end
    end

end