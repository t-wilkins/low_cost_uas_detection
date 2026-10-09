function s = ridHex(b, spaced)
% RIDHEX  Convert a uint8 byte array to a hex string.
%   ridHex(b)        -> no separators, e.g. 'FA0BBC0D' (what addIE wants)
%   ridHex(b, true)  -> space-separated, e.g. 'FA 0B BC 0D' (for display)

if nargin < 2
    spaced = false;
end
if spaced
    s = strtrim(sprintf('%02X ', uint8(b)));
else
    s = sprintf('%02X', uint8(b));
end
end
