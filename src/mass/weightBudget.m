function mass = weightBudget(varargin)
%WEIGHTBUDGET  Sub-250 g all-up mass budget for Pathfinder X1.
%   The 249 g cap (FAA sub-250 g exemption) is the binding design constraint.
%   Edit the component table as parts are selected; the function sums the
%   all-up mass and warns/errors if the budget is exceeded.
%
%   mass = weightBudget()        uses the placeholder estimates below
%   mass = weightBudget(cap_g)   override the cap (default 249)
%
%   OUTPUT struct:
%     mass.items     table of components [name, grams, status]
%     mass.total_g   summed all-up mass [g]
%     mass.margin_g  cap - total  (negative => over budget)
%     mass.feasible  logical
%
%   STATUS: placeholder masses — replace with measured/spec values.

    cap_g = 249;
    if nargin >= 1 && ~isempty(varargin{1}); cap_g = varargin{1}; end

    % name                       grams   status  (E=estimate, S=spec, M=measured)
    items = {
        "Airframe (CF spar+ribs+skin)", 70,  "E"
        "Battery (2S/3S LiPo)",         55,  "E"
        "Motor x2 (pusher)",            24,  "E"
        "ESC x2",                       10,  "E"
        "Propeller x2",                  6,  "E"
        "Tilt servo x2 (vectoring)",    18,  "E"
        "Flight controller",             9,  "E"
        "GPS + compass",                 8,  "E"
        "Radio receiver",                4,  "E"
        "FPV camera + VTX",             16,  "E"
        "Wiring + connectors + misc",   15,  "E"
    };

    names  = string(items(:,1));
    grams  = cell2mat(items(:,2));
    status = string(items(:,3));
    total  = sum(grams);

    mass.items    = table(names, grams, status, ...
                          'VariableNames', {'Component','grams','source'});
    mass.total_g  = total;
    mass.margin_g = cap_g - total;
    mass.feasible = total <= cap_g;

    fprintf('Pathfinder mass budget: %.0f g of %.0f g cap (margin %+.0f g)\n', ...
            total, cap_g, mass.margin_g);
    if ~mass.feasible
        warning('OVER BUDGET by %.0f g — cut mass before proceeding.', -mass.margin_g);
    end
end
