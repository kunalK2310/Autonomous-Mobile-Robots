function [cmdV, cmdW] = limitCmds(fwdVel, angVel, maxV, wheel2Center)
% Scale forward and angular velocity commands to avoid saturating motors.
%
% Inputs:
%  - fwdVel        Desired forward velocity (m/s).
%  - angVel        Desired angular velocity (rad/s).
%  - maxV          Maximum motor velocity (assumes symmetric saturation).
%  - wheel2Center  Distance from the wheels to the center of the robot (in
%                  meters).
%
% Outputs:
%  - cmdV          Scaled forward velocity command (m/s).
%  - cmdW          Scaled angular velocity command (rad/s).

L2 = wheel2Center;
L = L2 * 2;
VrminusVl = angVel * L;
VrplusVl = 2 * fwdVel;
Vr = (VrminusVl + VrplusVl) / 2;
Vl = VrplusVl - Vr;
scaleVr = 1;
scaleVl = 1;
if abs(Vr) > maxV
    scaleVr = maxV / abs(Vr);
end
if abs(Vl) > maxV
    scaleVl = maxV / abs(Vl);
end
scale = min(scaleVr, scaleVl);
Vr_scaled = scale * Vr;
Vl_scaled = scale * Vl;
cmdV = 1 / 2 * (Vr_scaled + Vl_scaled);
cmdW = 1 / L * (Vr_scaled - Vl_scaled);

end