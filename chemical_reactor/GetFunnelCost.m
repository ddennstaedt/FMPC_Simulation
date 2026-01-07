function j = GetFunnelCost(f_error,MPCCoeff,EnergyCoeff)
    j = @(time, state, control) MPCCoeff*f_error(time,state)^2/((Funnel(time)^2-f_error(time,state)^2)) + EnergyCoeff*norm(control-360)^2;
end

%*((1+ min(sign(Funnel(time)^2-error^2),0)))