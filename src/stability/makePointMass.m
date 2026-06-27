function comp = makePointMass(name, mass_kg, r_m)
% makePointMass
%
% Purpose:
%   Create a point-mass component record for aircraft mass properties.
%
% Inputs:
%   name    : char/string
%   mass_kg : scalar [kg]
%   r_m     : [x y z] absolute aircraft coordinates [m]
%
% Output:
%   comp    : struct

    arguments
        name
        mass_kg (1,1) double {mustBeNonnegative}
        r_m (1,3) double
    end

    comp = struct();
    comp.type    = 'pointMass';
    comp.name    = char(name);
    comp.mass_kg = mass_kg;
    comp.r_m     = r_m;
end