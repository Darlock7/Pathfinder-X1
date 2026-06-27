function [E_climb, E_cruise, E_f, E_design, E_f_Wh, E_design_Wh] = ...
    EnergyConsumption(Wg, delta_h, R_cruise, LD, eta_p, reserve_factor)

E_climb = (Wg*delta_h)/eta_p;
E_cruise = (Wg/LD)*(R_cruise/eta_p);

E_f = E_climb + E_cruise;
E_design = reserve_factor*E_f;

E_f_Wh = E_f/3600;
E_design_Wh = E_design/3600;

end