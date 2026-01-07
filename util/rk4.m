%% Runge-Kutte 4 Integrator
% @ode (Time, State) -- callback to get value of ode
% h step length
% t current time
% x current state
function xf = rk4(ode,h,t,x)
    k1 = ode(t,x);
    k2 = ode(t+h/2,x+h/2*k1);
    k3 = ode(t+h/2,x+h/2*k2);
    k4 = ode(t+h,x+h*k3);
    xf = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end