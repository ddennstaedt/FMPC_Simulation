%State [x_1, x_2, y]
function dtX = ode_chemicalreactor(Time, State, ControlValue)
    dtX = SystemDrift(State) + [0;0;1]* ControlValue;
end

function FX  = SystemDrift(State)
    x_1_in   = 1;
    x_2_in   = 0;
    C_1      = -1;
    C_2      = 1;
    b        = 209.2;
    d        = 1.1;
    q        = 1.25;
    
    FX = [C_1 * Arrhenius(State) + d * (x_1_in-State(1));...
          C_2 * Arrhenius(State) + d * (x_2_in-State(2));...
          b   * Arrhenius(State) - (q * State(3))];
end


function r = Arrhenius(State)
    k_0 = exp(25);
    k_1 = 8700;
    r = k_0 * exp(-k_1 / State(3)) * State(1);
end