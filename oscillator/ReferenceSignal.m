function Yref = ReferenceSignal(t)
    Yref = 250*(1/2)*(1+erf((t-3)/sqrt(2)));
end
