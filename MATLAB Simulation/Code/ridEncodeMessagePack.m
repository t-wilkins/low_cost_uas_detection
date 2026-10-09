function pack = ridEncodeMessagePack(msgs, protoVer)
% RIDENCODEMESSAGEPACK  Wrap N-by-25 uint8 messages into a Message Pack.
%   pack = ridEncodeMessagePack(msgs, protoVer)
%
%   msgs : N-by-25 uint8, one row per message
%   Returns the Message Pack (type 0xF) as a row vector:
%   [ type(0xF)|protoVer , singleMessageSize(25) , messageCount , msg_1 ... msg_N ]

n = size(msgs, 1);
hdr = [bitor(bitshift(uint8(15), 4), uint8(protoVer)), uint8(25), uint8(n)];
pack = [hdr, reshape(msgs.', 1, [])];
end
