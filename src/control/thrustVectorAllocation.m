function u = thrustVectorAllocation(wreq, state, cfg)
%THRUSTVECTORALLOCATION  Map a desired body wrench to twin throttle + tilt.
%   Pathfinder X1 has NO control surfaces. All force/moment authority comes
%   from two thrust-vectored pusher motors:
%       u.T  = [T_L; T_R]      thrust per motor              [N]
%       u.d  = [d_L; d_R]      tilt (vector) angle per motor [rad]
%
%   Actuation map (body axes, motors at +/- y_m from CG, moment arms l_*):
%     Each motor produces thrust T_i along its tilted axis (tilt about the
%     pitch axis by d_i). Resolve to body X (forward) and Z (up):
%       Fx_i =  T_i*cos(d_i)
%       Fz_i =  T_i*sin(d_i)
%     Body forces/moments:
%       Fx   =  Fx_L + Fx_R
%       Fz   =  Fz_L + Fz_R
%       Mpitch =  (Fz_L + Fz_R)*l_x        (tilt -> pitch)        [thrust vector]
%       Mroll  =  (Fz_R - Fz_L)*y_m        (differential tilt -> roll)
%       Myaw   =  (Fx_L - Fx_R)*y_m        (differential thrust -> yaw)
%
%   INPUTS
%     wreq  struct: desired .Fx, .Fz, .Mroll, .Mpitch, .Myaw   (SI)
%     state struct: airspeed, attitude, regime ("cruise"|"STOL"|"hover")
%     cfg   struct: geometry (.y_m, .l_x), actuator limits (.Tmax, .dMax)
%
%   OUTPUT
%     u     struct: .T = [T_L;T_R] [N], .d = [d_L;d_R] [rad], .feasible (bool)
%
%   NOTE: the system is OVER/UNDER-actuated depending on regime. In hover the
%   wing aero is ~zero and the two motors must hold attitude alone
%   (bicopter/tailsitter) — yaw authority is the weak axis. This function is
%   where Pathfinder's core control problem lives.
%
%   STATUS: STUB. TODO: implement the nonlinear allocation solve below.

    arguments
        wreq  struct
        state struct
        cfg   struct
    end

    % --- TODO: implement ---
    % 1. Build the actuation Jacobian from cfg geometry at current tilt.
    % 2. Solve for [T_L,T_R,d_L,d_R] given wreq (nonlinear in d -> lsqnonlin
    %    or small-angle linearization + iterate).
    % 3. Saturate to [0,Tmax] and [-dMax,dMax]; set feasible=false if clipped.
    % 4. Per regime: cruise prioritizes Fx; hover prioritizes Fz + attitude.

    error('thrustVectorAllocation: not yet implemented (X1 control core).');
end
