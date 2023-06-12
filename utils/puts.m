function puts(varargin)
% Print to console.
%
% Inputs:
%  - ...           Things to print.

o = "";

for i=1:nargin
    if i > 1
        o = strcat(o, " ");
    end
    n = varargin{i};
    if ~isstring(n)
        n = num2str(n);
    end
    o = strcat(o, n);
end

disp(o);

end
