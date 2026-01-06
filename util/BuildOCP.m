function [ocp, X, U, J, OCPInit,t0] = BuildOCP(f_Model,f_stagecost, ModelStateDimension,ControlDimension,MaxControl, ...
                                               MPCDiscretisation, Horizon, f_constraint)
    ocp = casadi.Opti();
    %Define decision variables
    X = ocp.variable(ModelStateDimension,MPCDiscretisation+1);
    U = ocp.variable(ControlDimension,MPCDiscretisation);
    t0=ocp.parameter();
    OCPInit=ocp.parameter(ModelStateDimension,1);

    h=Horizon/MPCDiscretisation;                 %MPC sampling rate
    %time instances
    t = 0:h:Horizon;
    
    %Cost function
    J=0;
    
    %Initial Constraint
    ocp.subject_to(X(:,1) == OCPInit);
    
    for i=1:MPCDiscretisation
        %Dynamic constraints
        CurTime = t0+t(i);
        xnext = rk4(@(t,x)f_Model(t,x,U(:,i)),h,CurTime,X(:,i));
        
        eval(['con_', num2str(i), '=  X(:,i+1) == xnext;']);
        eval(['ocp.subject_to( con_', num2str(i),' );']);
        
        %FMPC-term
        %Cur_Error =Error(f_ModelOutput(xnext),CurTime);
        J = J + f_stagecost(CurTime,xnext,U(i));
    end
    
    %Input constraints
    ocp.subject_to( -MaxControl <= U(:) <= MaxControl );
    
    %Output constraints
    if exist('f_constraint','var')
        for i=1:MPCDiscretisation+1
             ocp.subject_to(f_constraint(t0+t(i), X(:,i)));
             %ocp.subject_to(ReferenceSignal(t0+t(i))-Funnel(t0+t(i))<=f_ModelOutput(X(:,i))<=ReferenceSignal(t0+t(i))+Funnel(t0+t(i)));
        end
    end
    
    ocp.solver('ipopt',OCPPluginOptions(),OCPSolverOptions());
    
    ocp.minimize(J);                % Set cost function
end 


function plugin_options = OCPPluginOptions
    plugin_options = struct;
end

function solver_options = OCPSolverOptions
    solver_options = struct;
    solver_options.max_iter =1000;
    solver_options.print_level =0;
    solver_options.acceptable_tol = 1e-8;
    solver_options.acceptable_obj_change_tol = 1e-6;
end