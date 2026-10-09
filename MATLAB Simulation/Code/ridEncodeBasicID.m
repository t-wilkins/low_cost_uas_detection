function msg = ridEncodeBasicID(messageType, idType, uaType, uasID, protoVer)
% RIDENCODEBASICID  25-byte Basic ID message, ASTM F3411.
%   msg = ridEncodeBasicID(messageType, idType, uaType, uasID, protoVer)
%
%   messageType : 0 = Basic ID
%   idType      : e.g. 1 = Serial Number
%   uaType      : e.g. 2 = Helicopter / Multirotor
%   uasID       : char, <=20 chars
%   protoVer    : protocol version nibble (e.g. 2 = F3411-22a)

msg = zeros(1, 25, 'uint8');

% byte 1: Message Type (4b) | Protocol Version (4b)
msg(1) = bitor(bitshift(uint8(messageType), 4), uint8(protoVer));

% byte 2: ID Type (4b) | UA Type (4b)
msg(2) = bitor(bitshift(uint8(idType), 4), uint8(uaType));

% bytes 3-22: UAS ID, ASCII, null-padded to 20 bytes
b = uint8(uasID);
n = min(numel(b), 20);
msg(3:2+n) = b(1:n);

% bytes 23-25: reserved, already zero
end
