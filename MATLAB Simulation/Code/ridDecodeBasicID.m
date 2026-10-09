function d = ridDecodeBasicID(msg)
% RIDDECODEBASICID  Decode a 25-byte Basic ID message back into fields.

msg = uint8(msg(:)).';
d.protocolVersion = double(bitand(msg(1), 15));
d.idType          = double(bitshift(msg(2), -4));
d.uaType          = double(bitand(msg(2), 15));
raw               = char(msg(3:22));
d.uasID           = raw(raw ~= char(0));
end
