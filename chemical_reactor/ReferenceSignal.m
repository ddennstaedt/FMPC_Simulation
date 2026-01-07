function Yref = ReferenceSignal(t)
    % Workaround since Casadi does not support piecwise defined functions
    TempStart = 270;
    TempEnd = 337.1;
    FirstPart = @(t) max(sign(sin(t.*pi/3+pi/3)),0);
    SecondPart = @(t) max(sign(sin(t.*pi/3-2*pi/3)),0);
    Yref= FirstPart(t).*(TempStart+t.*(TempEnd-TempStart)/2)+SecondPart(t).*TempEnd;
    %Yref = 337.1;
end