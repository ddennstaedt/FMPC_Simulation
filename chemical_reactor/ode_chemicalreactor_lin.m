%State [x_1, x_2, y]
function dtX = ode_chemicalreactor_lin(Time, State, ControlValue)
    dtX = SystemDrift_Lin(State) + [0;0;1]* ControlValue;
end

function FX  = SystemDrift_Lin(State)
    x_1_in   = 1;
    x_2_in   = 0;
    C_1      = -1;
    C_2      = 1;
    b        = 209.2;
    d        = 1.1;
    q        = 1.25;
    bar_y    = 337.1;

    k_0 = exp(25);
    k_1 = 8700;
    
    a_1 = k_0*k_1*exp(-k_1/bar_y)/bar_y^2*x_1_in/2;
    a_2 = k_0*exp(-k_1/bar_y);
    
    A = [C_1*a_2-d 0 C_1*a_1;
         C_2*a_2 -d C_2*a_1;
         b*a_2 0 b*a_1-q];

    D = [-C_1*a_1*bar_y+d*x_1_in;
         -C_2*a_1*bar_y+d*x_2_in;
         -b*a_1*bar_y];
    FX = A*State + D;
end
