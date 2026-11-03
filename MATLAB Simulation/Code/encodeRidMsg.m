function pack = encodeRidMsg(cfg)
% ENCODERIDMSG  ASTM F3411-22a Message Pack, matching the sxjack ESP32 scheduler.
%   pack = encodeRidMsg(cfg)
%
%   Returns the Message Pack as a 1-by-(3 + 25*N) uint8 row vector, ready to
%   follow the counter byte in the vendor IE (DD len FA 0B BC 0D counter pack).
%
%   Two message packs, alternating every beacon (500 ms), A first:
%     A (even msgCounter): Basic ID, Location, System, Operator ID  (103 bytes)
%     B (odd  msgCounter): Location, Self ID                         (53 bytes)
%   Location is sent at 2 Hz; all other messages at 1 Hz.
%
%   Basic ID and Location are properly encoded. System, Operator ID and
%   Self ID carry the correct header byte followed by 24 random bytes.
%
%   Position is given in the simulation's local ENU frame and converted to
%   WGS84 latitude/longitude/altitude about cfg.origin with
%   enu2lla(..., 'ellipsoid') (UAV Toolbox, R2020b+). The ground is the ENU plane 
%   z = 0, so the Height field (above ground) is z. Convert back on the 
%   receive side with lla2enu(..., origin, 'flat').
%
%   cfg fields (all optional, defaults shown):
%       uasID      = 'A.00001'      UAS serial number, <= 20 characters
%       pos        = [100 100 10]   ENU [x y z] (m), z = height above ground
%       origin     = [53.3438 -6.2546 60]  ENU origin [lat lon h] (deg, deg, m
%                                   ellipsoidal); set to Trinity College
%       heading    = 0              track direction, degrees clockwise from north
%       speed      = 0              horizontal ground speed (m/s), 0-254.25
%       vertSpeed  = 0              vertical speed (m/s), +/-63, positive up
%       t          = 0              simulation time (s), for the timestamp
%       msgCounter = 0              beacon counter; parity selects pack A or B

%% Apply defaults
if nargin < 1 || isempty(cfg), cfg = struct(); end
def = struct('uasID', 'A.00001', ...
    'pos', [100 100 10], ...
    'origin', [53.3438 -6.2546 60], ...
    'heading', 0, ...
    'speed', 0, ...
    'vertSpeed', 0, ...
    't', 0, ...
    'msgCounter', 0, ...
    'visualise', 0);
f = fieldnames(def);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = def.(f{i}); end
end

%% Encode Message Pack

if mod(cfg.msgCounter, 2) == 0      % Pack A
    msgs = [encodeBasicID(cfg);
            encodeLocMsg(cfg);
            randomMsg(4);           % System
            randomMsg(5)];          % Operator ID
else                                % Pack B
    msgs = [encodeLocMsg(cfg);
            randomMsg(3)];          % Self ID
end

pack = ridEncodeMessagePack(msgs, 2);

if cfg.visualise
    fprintf('Message Pack (%d bytes): %s\n', numel(pack), ...
            strtrim(sprintf('%02X ', uint8(pack))));
end 
end

% -------------------------------------------------------------------------

function hdr = msgHeader(messageType)
    % Message Type (4b) | Protocol Version (4b), version 2 = F3411-22a
    hdr = bitor(bitshift(uint8(messageType), 4), uint8(2));
end

function msg = randomMsg(messageType)
    % 25-byte message with a valid header and random content
    msg = [msgHeader(messageType), randi([0 255], 1, 24, 'uint8')];
end

function basicIdMsg = encodeBasicID(cfg)
    idType = 1;                     % Serial Number
    uaType = 2;                     % Helicopter / Multirotor

    basicIdMsg = zeros(1, 25, 'uint8');
    basicIdMsg(1) = msgHeader(0);   % Basic ID
    basicIdMsg(2) = bitor(bitshift(uint8(idType), 4), uint8(uaType));

    % bytes 3-22: UAS ID, ASCII, null-padded to 20 bytes
    b = uint8(cfg.uasID);
    n = min(numel(b), 20);
    basicIdMsg(3:2+n) = b(1:n);
    % bytes 23-25: reserved, zero
end

function locMsg = encodeLocMsg(cfg)
    status     = 2;                 % Airborne
    heightType = 1;                 % 1 = above ground level, 0 = above take-off

    locMsg = zeros(1, 25, 'uint8');
    locMsg(1) = msgHeader(1);       % Location / Vector

    % Track direction: 0-179 plus E/W bit adding 180
    hdg = mod(round(cfg.heading), 360);
    ewBit = uint8(hdg >= 180);
    dirVal = uint8(hdg - 180 * double(ewBit));

    % Speed: 0.25 m/s steps up to 63.75, then 0.75 m/s steps with multiplier bit
    spd = max(cfg.speed, 0);
    if spd <= 63.75
        multBit = uint8(0);
        spdVal  = uint8(round(spd / 0.25));
    else
        multBit = uint8(1);
        spdVal  = uint8(min(round((spd - 63.75) / 0.75), 254));
    end

    % Byte 2: Status (4b) | reserved (1b) | Height type (1b) | E/W (1b) | Speed mult (1b)
    locMsg(2) = bitor(bitor(bitshift(uint8(status), 4), bitshift(uint8(heightType), 2)), ...
                      bitor(bitshift(ewBit, 1), multBit));

    % Byte 3: Direction
    locMsg(3) = dirVal;

    % Byte 4: Speed
    locMsg(4) = spdVal;

    % Byte 5: vertical speed, int8, 0.5 m/s steps, +/-63 m/s
    vs = max(min(round(cfg.vertSpeed / 0.5), 126), -126);
    locMsg(5) = typecast(int8(vs), 'uint8');

    % Bytes 6-13: latitude, longitude, int32 in 1e-7 degrees (~1.1 cm)
    lla = enu2lla(cfg.pos(:).', cfg.origin, 'flat');
    lat = lla(1); lon = lla(2); hEll = lla(3);
    locMsg(6:9)   = typecast(int32(round(lat * 1e7)), 'uint8');
    locMsg(10:13) = typecast(int32(round(lon * 1e7)), 'uint8');

    % Bytes 14-19: pressure altitude, geodetic altitude, height (0.5 m steps, -1000 m offset)
    locMsg(14:15) = encodeAlt(-1000);       % pressure altitude: invalid
    locMsg(16:17) = encodeAlt(hEll);        % geodetic altitude (WGS84 ellipsoidal)
    locMsg(18:19) = encodeAlt(cfg.pos(3));  % height above ground

    % Accuracy fields, fixed as in the ESP32 code
    locMsg(20) = uint8(4*16 + 10);  % vertical <10 m | horizontal <10 m
    locMsg(21) = uint8(4*16 + 1);   % baro <10 m     | speed <10 m/s

    % Bytes 22-23: timestamp, tenths of a second since the start of the hour
    locMsg(22:23) = typecast(uint16(mod(round(cfg.t * 10), 36000)), 'uint8');
    locMsg(24) = uint8(15);         % timestamp accuracy 1.5 s (0.1 s units)
    % byte 25: reserved, zero
end

function b = encodeAlt(alt)
    b = typecast(uint16(max(min(round((alt + 1000) / 0.5), 65535), 0)), 'uint8');
end

function pack = ridEncodeMessagePack(msgs, protoVer)
    % [type(0xF)|protoVer, singleMessageSize(25), messageCount, msg_1 ... msg_N]
    n = size(msgs, 1);
    hdr = [bitor(bitshift(uint8(15), 4), uint8(protoVer)), uint8(25), uint8(n)];
    pack = [hdr, reshape(msgs.', 1, [])];
end

