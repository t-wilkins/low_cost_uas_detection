function [payload, ok] = decodeRidMsg(pack, origin)
% DECODERIDMSG  Decode an ASTM F3411-22a Message Pack (inverse of encodeRidMsg).
%   [payload, ok] = decodeRidMsg(pack, origin)
%
%   pack   : uint8 Message Pack bytes [F2 19 N msg_1 ... msg_N]
%   origin : [lat0 lon0 h0] ENU origin, for converting the position back
%            with lla2enu(..., 'flat') (UAV Toolbox)
%
%   payload fields (NaN / '' if the message is absent):
%     msgTypes  : message types found in the pack, e.g. [0 1 4 5]
%     uasID     : Basic ID UAS ID
%     idType, uaType
%     status    : Location status (2 = airborne)
%     lat, lon  : degrees
%     altGeo    : geodetic altitude (m)
%     height    : height above ground (m)
%     pos       : 3x1 ENU position (m) from lat, lon and height
%     heading   : degrees clockwise from north
%     speed     : horizontal speed (m/s)
%     vertSpeed : vertical speed (m/s)
%     timestamp : tenths of a second since the hour, in seconds
%   ok : true if the pack header is valid and every message is 25 bytes

payload = struct('msgTypes', [], 'uasID', '', 'idType', NaN, 'uaType', NaN, ...
    'status', NaN, 'lat', NaN, 'lon', NaN, 'altGeo', NaN, ...
    'height', NaN, 'pos', nan(3,1), 'heading', NaN, 'speed', NaN, ...
    'vertSpeed', NaN, 'timestamp', NaN);
ok = false;

pack = uint8(pack(:).');
if numel(pack) < 3 || bitshift(pack(1), -4) ~= 15 || pack(2) ~= 25
    return
end
nMsg = double(pack(3));
if numel(pack) < 3 + 25*nMsg
    return
end

for m = 1:nMsg
    msg  = pack(3 + 25*(m-1) + (1:25));
    type = double(bitshift(msg(1), -4));
    payload.msgTypes(end+1) = type;
    switch type
        case 0
            payload = decodeBasicID(payload, msg);
        case 1
            payload = decodeLocation(payload, msg, origin);
    end
end
ok = true;
end

% -------------------------------------------------------------------------

function p = decodeBasicID(p, msg)
    p.idType = double(bitshift(msg(2), -4));
    p.uaType = double(bitand(msg(2), 15));
    id = msg(3:22);
    p.uasID = char(id(id ~= 0));
end

function p = decodeLocation(p, msg, origin)
    b2 = msg(2);
    p.status = double(bitshift(b2, -4));
    ewBit    = double(bitand(bitshift(b2, -1), 1));
    multBit  = double(bitand(b2, 1));
    
    p.heading = double(msg(3)) + 180*ewBit;
    if multBit == 0
        p.speed = double(msg(4)) * 0.25;
    else
        p.speed = double(msg(4)) * 0.75 + 63.75;
    end
    p.vertSpeed = double(typecast(msg(5), 'int8')) * 0.5;
    
    p.lat    = double(typecast(msg(6:9),   'int32')) * 1e-7;
    p.lon    = double(typecast(msg(10:13), 'int32')) * 1e-7;
    p.altGeo = decodeAlt(msg(16:17));
    p.height = decodeAlt(msg(18:19));
    
    p.timestamp = double(typecast(msg(22:23), 'uint16')) / 10;
    
    % Back to the simulation frame; z is height above ground, as encoded
    xyz   = lla2enu([p.lat p.lon p.altGeo], origin, 'flat');
    p.pos = [xyz(1); xyz(2); p.height];
end

function alt = decodeAlt(b)
    alt = double(typecast(b, 'uint16')) * 0.5 - 1000;
end