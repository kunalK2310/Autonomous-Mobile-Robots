function [isDebug] = isDebug(setDebug)
% Check if the application is running in debug mode.
%
% Inputs:
%  - setDebug      Used to set debug value. Should only be used by
%                  finalCompetition.
%
% Outputs:
%  - debug         True if application is in debug mode, false if not.

persistent debug;

if exist("setDebug", "var")
    if setDebug
        debug = true;
    else
        debug = false;
    end
end

if debug
    isDebug = true;
else
    isDebug = false;
end

end
