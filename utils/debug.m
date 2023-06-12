function debug(varargin)
% Just like puts, but only print if debug is on.
%
% Inputs:
%  - ...           Things to print.

if isDebug()
    puts(varargin{:});
end

end
