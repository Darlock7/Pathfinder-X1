function missionOut = missionProfileCigar(mission)

%% =========================
%   INPUTS
%% =========================
nLaps             = mission.nLaps;
lapLengthTarget_m = mission.lapLengthTarget_m;
V_pattern         = mission.V_pattern;
h_ground          = mission.h_ground;
h_cruise          = mission.h_cruise;

runwayLength_m    = mission.runwayLength_m;
straightLength_m  = mission.straightLength_m;

liftoffFrac       = mission.liftoffFrac;
touchdownFrac     = mission.touchdownFrac;
n_turn            = mission.n_turn;

climbRate_mps     = mission.climbRate_mps;
descentRate_mps   = mission.descentRate_mps;

g = 9.81;

fprintf('================ Mission Profile ======================\n');

%% =========================
%   BASIC CHECKS
%% =========================
if nLaps < 1 || floor(nLaps) ~= nLaps
    error('mission.nLaps must be a positive integer.');
end
if lapLengthTarget_m <= 2*straightLength_m
    error('mission.lapLengthTarget_m must be greater than 2*straightLength_m.');
end
if runwayLength_m <= 0 || straightLength_m <= 0
    error('mission.runwayLength_m and mission.straightLength_m must be positive.');
end
if straightLength_m < runwayLength_m
    error('mission.straightLength_m must be >= mission.runwayLength_m.');
end
if V_pattern <= 0
    error('mission.V_pattern must be positive.');
end
if h_cruise < h_ground
    error('mission.h_cruise must be >= mission.h_ground.');
end
if liftoffFrac <= 0 || liftoffFrac >= 1
    error('mission.liftoffFrac must be between 0 and 1.');
end
if touchdownFrac <= 0 || touchdownFrac >= 1
    error('mission.touchdownFrac must be between 0 and 1.');
end
if n_turn <= 1
    error('mission.n_turn must be > 1 for a coordinated level turn.');
end
if climbRate_mps <= 0 || descentRate_mps <= 0
    error('mission.climbRate_mps and mission.descentRate_mps must be positive.');
end
if climbRate_mps >= V_pattern || descentRate_mps >= V_pattern
    error('Vertical rates must be less than mission.V_pattern.');
end

%% =========================
%   TURN PHYSICS / GEOMETRY
%% =========================
% Path geometry uses straightLength_m, not runwayLength_m
R_geom = (lapLengthTarget_m - 2*straightLength_m) / (2*pi);
R_phys = V_pattern^2 / (g * sqrt(n_turn^2 - 1));

R_turn = max(R_geom, R_phys);

lapLength_m    = 2*straightLength_m + 2*pi*R_turn;
missionRange_m = nLaps * lapLength_m;

fprintf('Runway length                      = %.2f m\n', runwayLength_m);
fprintf('Straight length used              = %.2f m\n', straightLength_m);
fprintf('Target lap length                 = %.2f m\n', lapLengthTarget_m);
fprintf('Geometric turn radius             = %.2f m\n', R_geom);
fprintf('Physics-based min turn radius     = %.2f m\n', R_phys);
fprintf('Turn radius used                  = %.2f m\n', R_turn);
fprintf('Lap length used                   = %.2f m\n', lapLength_m);
fprintf('Total mission distance            = %.2f m\n\n', missionRange_m);

%% =========================
%   BUILD ONE LAP
%% =========================
% Segment 1: lower straight, y = 0, x: 0 -> straightLength_m
% Segment 2: right turn upward
% Segment 3: upper straight, y = 2R, x: straightLength_m -> 0
% Segment 4: left turn downward back to y = 0

N = 800;
s = linspace(0, lapLength_m, N).';

x_lap = zeros(size(s));
y_lap = zeros(size(s));

s1 = straightLength_m;
s2 = s1 + pi*R_turn;
s3 = s2 + straightLength_m;

for i = 1:length(s)
    si = s(i);

    if si <= s1
        % lower straight
        x_lap(i) = si;
        y_lap(i) = 0;

    elseif si <= s2
        % right turn upward
        th = -pi/2 + (si - s1)/R_turn;
        x_lap(i) = straightLength_m + R_turn*cos(th);
        y_lap(i) = R_turn + R_turn*sin(th);

    elseif si <= s3
        % upper straight
        x_lap(i) = straightLength_m - (si - s2);
        y_lap(i) = 2*R_turn;

    else
        % left turn downward
        th = pi/2 + (si - s3)/R_turn;
        x_lap(i) = R_turn*cos(th);
        y_lap(i) = R_turn + R_turn*sin(th);
    end
end

%% =========================
%   STACK LAPS
%% =========================
x = [];
y = [];
s_total = [];

for k = 0:(nLaps-1)
    if k < nLaps-1
        idx = 1:(N-1);
    else
        idx = 1:N;
    end

    x = [x; x_lap(idx)];
    y = [y; y_lap(idx)];
    s_total = [s_total; s(idx) + k*lapLength_m];
end

%% =========================
%   EVENTS
%% =========================
% Liftoff referenced to actual runway length
s_liftoff = liftoffFrac * runwayLength_m;

% Payload drop on lap 2, halfway down the lower straight
if nLaps >= 2
    s_deploy = lapLength_m + 0.5*runwayLength_m;
else
    s_deploy = 0.5*runwayLength_m;
end

%% =========================
%   PHYSICS-BASED CLIMB / DESCENT
%% =========================
deltaH = h_cruise - h_ground;

Vh_climb   = sqrt(V_pattern^2 - climbRate_mps^2);
Vh_descent = sqrt(V_pattern^2 - descentRate_mps^2);

t_climb_s   = deltaH / climbRate_mps;
t_descent_s = deltaH / descentRate_mps;

climb_len   = Vh_climb   * t_climb_s;
descent_len = Vh_descent * t_descent_s;

climbAngle_deg   = atan2d(climbRate_mps, Vh_climb);
descentAngle_deg = atan2d(descentRate_mps, Vh_descent);

% Climb starts after liftoff
s_climb_end = s_liftoff + climb_len;

% Touchdown occurs on the FINAL LOWER STRAIGHT at y = 0
s_touchdown = (nLaps - 1)*lapLength_m + touchdownFrac*runwayLength_m;

% Descent begins descent_len before touchdown
s_desc_start = s_touchdown - descent_len;

if s_climb_end >= s_desc_start
    error('Climb and descent regions overlap. Adjust mission sizing or rates.');
end

fprintf('Climb rate                        = %.2f m/s\n', climbRate_mps);
fprintf('Descent rate                      = %.2f m/s\n', descentRate_mps);
fprintf('Climb angle                       = %.2f deg\n', climbAngle_deg);
fprintf('Descent angle                     = %.2f deg\n', descentAngle_deg);
fprintf('Climb distance                    = %.2f m\n', climb_len);
fprintf('Descent distance                  = %.2f m\n', descent_len);
fprintf('Liftoff station                   = %.2f m\n', s_liftoff);
fprintf('Touchdown station                 = %.2f m\n\n', s_touchdown);

%% =========================
%   ALTITUDE PROFILE
%% =========================
z = zeros(size(s_total));

for i = 1:length(s_total)
    si = s_total(i);

    if si <= s_liftoff
        z(i) = h_ground;

    elseif si <= s_climb_end
        z(i) = h_ground + (h_cruise - h_ground) * ...
            ((si - s_liftoff)/(s_climb_end - s_liftoff));

    elseif si <= s_desc_start
        z(i) = h_cruise;

    elseif si <= s_touchdown
        z(i) = h_ground + (h_cruise - h_ground) * ...
            (1 - (si - s_desc_start)/(s_touchdown - s_desc_start));

    else
        z(i) = h_ground;
    end
end

%% =========================
%   TIME
%% =========================
t = s_total / V_pattern;
lapEndTimes = (1:nLaps)' * lapLength_m / V_pattern;

%% =========================
%   TRIM AT TOUCHDOWN
%% =========================
validIdx = s_total <= s_touchdown;

x = x(validIdx);
y = y(validIdx);
z = z(validIdx);
t = t(validIdx);
s_total = s_total(validIdx);

%% =========================
%   RECOMPUTE PAYLOAD INDEX AFTER TRIM
%% =========================
[~, idxDeploy] = min(abs(s_total - s_deploy));

%% =========================
%   PHASE INDICES
%% =========================
idx_ground_roll = s_total <= s_liftoff;
idx_climb       = s_total > s_liftoff    & s_total <= s_climb_end;
idx_cruise      = s_total > s_climb_end  & s_total <= s_desc_start;
idx_descent     = s_total > s_desc_start & s_total <= s_touchdown;

%% =========================
%   3D PLOT
%% =========================
fig3D = figure('Name','Mission Profile (3D)','NumberTitle','off','Color','w');
hold on;
grid on;

hGroundProj = plot3(x, y, zeros(size(z)), 'k-', 'LineWidth', 1.0);

hGroundRoll = plot3(x(idx_ground_roll), y(idx_ground_roll), z(idx_ground_roll), ...
    'g-', 'LineWidth', 2.5);

hClimb = plot3(x(idx_climb), y(idx_climb), z(idx_climb), ...
    '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5);

hCruise = plot3(x(idx_cruise), y(idx_cruise), z(idx_cruise), ...
    'c-', 'LineWidth', 2.5);

hDescent = plot3(x(idx_descent), y(idx_descent), z(idx_descent), ...
    'r-', 'LineWidth', 2.5);

hTakeoff = plot3(x(1), y(1), 0, 'ko', ...
    'MarkerSize', 8, 'LineWidth', 1.8, 'MarkerFaceColor', 'w');

hLanding = plot3(x(end), y(end), 0, 'ks', ...
    'MarkerSize', 8, 'LineWidth', 1.8, 'MarkerFaceColor', 'k');

hDeploy = plot3(x(idxDeploy), y(idxDeploy), z(idxDeploy), 'ks', ...
    'MarkerSize', 8, 'LineWidth', 1.8, 'MarkerFaceColor', 'w');

text(x(1), y(1), 0, '  Takeoff', 'FontWeight', 'bold');
text(x(idxDeploy), y(idxDeploy), z(idxDeploy), '  Payload Drop', ...
    'FontWeight', 'bold');
text(x(end), y(end), 0, '  Landing', 'FontWeight', 'bold');

xlabel('x [m]');
ylabel('y [m]');
zlabel('Altitude [m]');
title('Mission Profile');

dx = max(x) - min(x);
dy = max(y) - min(y);
dz = max(z) - min(z);
pad = 0.10;

xlim([min(x)-pad*dx, max(x)+pad*dx]);
ylim([min(y)-pad*dy, max(y)+pad*dy]);
zlim([0, max(z)+pad*max(dz,1)]);

axis equal
daspect([1 1 0.4])
view(45,25)
rotate3d on

legend([hGroundProj, hGroundRoll, hClimb, hCruise, hDescent, hTakeoff, hLanding, hDeploy], ...
    {'Ground Projection', 'Ground Roll', 'Climb', 'Cruise', 'Descent', ...
     'Takeoff', 'Landing', 'Payload Drop'}, ...
    'Location', 'best');

%% =========================
%   2D ALTITUDE VS TIME
%% =========================
% fig2D = figure('Color','w');
% hold on;
% grid on;
% box on;
% 
% plot(t(idx_ground_roll), z(idx_ground_roll), 'g-', 'LineWidth', 2.5);
% plot(t(idx_climb),       z(idx_climb),       '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 2.5);
% plot(t(idx_cruise),      z(idx_cruise),      'c-', 'LineWidth', 2.5);
% plot(t(idx_descent),     z(idx_descent),     'r-', 'LineWidth', 2.5);
% 
% plot(t(1), z(1), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'w');
% plot(t(idxDeploy), z(idxDeploy), 'ks', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'w');
% plot(t(end), z(end), 'ks', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerFaceColor', 'k');
% 
% for k = 1:nLaps
%     if lapEndTimes(k) <= t(end)
%         xline(lapEndTimes(k), '--', sprintf('End Lap %d', k), ...
%             'LabelVerticalAlignment', 'bottom');
%     end
% end
% 
% text(t(1), z(1)+0.03*max(h_cruise,1), 'Takeoff', 'FontWeight', 'bold');
% text(t(idxDeploy), z(idxDeploy)+0.03*max(h_cruise,1), 'Payload Drop', ...
%     'FontWeight', 'bold', 'HorizontalAlignment', 'center');
% text(t(end), z(end)+0.03*max(h_cruise,1), 'Landing', 'FontWeight', 'bold');
% 
% xlabel('Time [s]');
% ylabel('Altitude [m]');
% title('Altitude vs Time');
% 
% legend({'Ground Roll', 'Climb', 'Cruise', 'Descent'}, ...
%     'Location', 'best');
% 

%% =========================
%   2D ALTITUDE VS TIME (SIMPLE)
%% =========================
fig2D = figure('Name','Altitude vs Time','NumberTitle','off','Color','w');
hold on;

% clean single mission profile curve
plot(t, z, 'k-', 'LineWidth', 2.2);

% important points
plot(t(1), z(1), 'ko', 'MarkerFaceColor','w', 'MarkerSize', 6, 'LineWidth', 1.1);
plot(t(idxDeploy), z(idxDeploy), 'ko', 'MarkerFaceColor','w', 'MarkerSize', 6, 'LineWidth', 1.1);
plot(t(end), z(end), 'ks', 'MarkerFaceColor','k', 'MarkerSize', 6, 'LineWidth', 1.1);

% minimal labels
text(t(1), z(1) + 0.04*max(h_cruise,1), 'Takeoff', ...
    'HorizontalAlignment','left', 'VerticalAlignment','bottom');

text(t(idxDeploy), z(idxDeploy) + 0.04*max(h_cruise,1), 'Payload Drop', ...
    'HorizontalAlignment','center', 'VerticalAlignment','bottom');

text(t(end), z(end) + 0.04*max(h_cruise,1), 'Landing', ...
    'HorizontalAlignment','right', 'VerticalAlignment','bottom');

% optional cruise-altitude reference
plot([t(1) t(end)], [h_cruise h_cruise], '--', ...
    'Color', [0.65 0.65 0.65], 'LineWidth', 0.9);

text(t(round(0.08*length(t))), h_cruise + 0.04*max(h_cruise,1), ...
    sprintf('h_{cruise} = %.0f m', h_cruise), ...
    'HorizontalAlignment','left');

title('Altitude vs Time');
xlabel('Time [s]');
ylabel('Altitude [m]');

xlim([t(1) t(end)]);
ylim([h_ground - 1, h_cruise + 0.12*max(h_cruise,1)]);

ax = gca;
ax.Box = 'off';
ax.LineWidth = 0.8;
ax.FontSize = 12;
ax.TickDir = 'out';
grid off;

% cleaner ticks
yticks([h_ground h_cruise]);

hold off;

%% =========================
%   OUTPUT
%% =========================
missionOut.x = x;
missionOut.y = y;
missionOut.z = z;
missionOut.t = t;
missionOut.s = s_total;

missionOut.turnRadius_m = R_turn;
missionOut.lapLengthUsed_m = lapLength_m;
missionOut.totalDistance_m = s_total(end);
missionOut.totalTime_s = t(end);

missionOut.climbRate_mps = climbRate_mps;
missionOut.descentRate_mps = descentRate_mps;
missionOut.climbTime_s = t_climb_s;
missionOut.descentTime_s = t_descent_s;
missionOut.climbDistance_m = climb_len;
missionOut.descentDistance_m = descent_len;
missionOut.climbAngle_deg = climbAngle_deg;
missionOut.descentAngle_deg = descentAngle_deg;

missionOut.idxDeploy = idxDeploy;
missionOut.s_liftoff = s_liftoff;
missionOut.s_touchdown = s_touchdown;
missionOut.s_deploy = s_deploy;
missionOut.lapEndTimes = lapEndTimes;

missionOut.fig3D = fig3D;
missionOut.fig2D = fig2D;

end