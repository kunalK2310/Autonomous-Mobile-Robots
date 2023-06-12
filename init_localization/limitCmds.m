function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Liang, Leo
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