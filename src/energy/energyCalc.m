function [E_climb, E_cruise, E_f, E_design, E_f_Wh, E_design_Wh] = ...
    energyCalc(Wg, eta_p, LD, delta_h, R_cruise, reserve_factor)
% energyCalc
% Computes mission energy estimates from gross weight and mission assumptions.
%
% Inputs:
%   Wg             [N]  Gross weight
%   eta_p          [-]  Propulsion efficiency
%   LD             [-]  Lift-to-drag ratio
%   delta_h        [m]  Climb altitude change
%   R_cruise       [m]  Cruise range
%   reserve_factor [-]  Energy margin multiplier
%
% Outputs:
%   E_climb        [J]  Climb energy
%   E_cruise       [J]  Cruise energy
%   E_f            [J]  Total mission energy
%   E_design       [J]  Design energy with reserve
%   E_f_Wh         [Wh] Total mission energy
%   E_design_Wh    [Wh] Design energy with reserve

    E_climb  = (Wg * delta_h) / eta_p;
    E_cruise = (Wg / LD) * (R_cruise / eta_p);

    E_f      = E_climb + E_cruise;
    E_design = reserve_factor * E_f;

    E_f_Wh      = E_f / 3600;
    E_design_Wh = E_design / 3600;
end