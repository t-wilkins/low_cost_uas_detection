function tx = buildRidBeaconWaveform(opts)
% BUILDRIDBEACONWAVEFORM  Build an ASTM F3411 Basic ID message, wrap it in
% an 802.11 beacon vendor-specific IE, and generate the non-HT OFDM
% waveform. Pure build step - no channel, no plotting.
%
%   tx = buildRidBeaconWaveform(opts)
%
%   opts fields (all optional, defaults shown):
%     uasID          = 'N.123456'   char, <=20 chars
%     idType         = 1            1 = Serial Number
%     uaType         = 2            2 = Helicopter / Multirotor
%     protoVer       = 2            2 = F3411-22a
%     msgCounter     = 0            vendor IE message counter byte
%     ssid           = 'DroneIDTest'
%     beaconInterval = 100          TUs
%     mcs            = 0            BPSK 1/2, 6 Mbps
%     channelBW      = 'CBW20'
%     idleTime       = 20e-6        trailing idle time appended by wlanWaveformGenerator
%
%   Returns struct tx with fields:
%     waveform   : generated non-HT OFDM waveform (no channel applied)
%     cfgPHY     : wlanNonHTConfig used
%     ind        : wlanFieldIndices(cfgPHY)
%     fs         : sample rate
%     mpduBits   : transmitted MPDU bits (ground truth for BER)
%     mpduLen    : MPDU length in bytes
%     uasID      : the UAS ID actually encoded (ground truth for ID match)
%     basicIDMsg : the raw 25-byte Basic ID message
%     ieData     : the full vendor IE payload bytes

opts = applyDefaults(opts);

%% Remote ID payload
basicID = ridEncodeBasicID(0, opts.idType, opts.uaType, opts.uasID, opts.protoVer);
pack    = ridEncodeMessagePack(basicID, opts.protoVer);
ieData  = [uint8([250 11 188 13]), uint8(opts.msgCounter), pack];   % FA 0B BC 0D = ASTM RID

%% Beacon MAC frame
frameBody = wlanMACManagementConfig;
frameBody.SSID           = opts.ssid;
frameBody.BeaconInterval = opts.beaconInterval;
frameBody = addIE(frameBody, 221, ridHex(ieData));

cfgMAC = wlanMACFrameConfig('FrameType', 'Beacon', 'ManagementConfig', frameBody);
[mpduBits, mpduLen] = wlanMACFrame(cfgMAC, 'OutputFormat', 'bits');

%% PHY: non-HT OFDM waveform
cfgPHY = wlanNonHTConfig( ...
    'ChannelBandwidth', opts.channelBW, ...
    'MCS',              opts.mcs, ...
    'PSDULength',       mpduLen);

fs = wlanSampleRate(cfgPHY);
waveform = wlanWaveformGenerator(mpduBits, cfgPHY, 'IdleTime', opts.idleTime);

%% Apply Tx Output Power Scaling
ind = wlanFieldIndices(cfgPHY);
txPowerDbm = 20;
airLen = double(ind.NonHTData(2));
avgPower = mean(abs(waveform(1:airLen)).^2);
waveform = waveform * sqrt(10^((txPowerDbm - 30) / 10) / avgPower); % Watts

%% Package output
tx.waveform   = waveform;
tx.cfgPHY     = cfgPHY;
tx.ind        = ind;
tx.fs         = fs;
tx.mpduBits   = mpduBits;
tx.mpduLen    = mpduLen;
tx.uasID      = opts.uasID;
tx.basicIDMsg = basicID;
tx.ieData     = ieData;
end

function opts = applyDefaults(opts)
if nargin < 1 || isempty(opts), opts = struct(); end
defaults = struct( ...
    'uasID',          'N.123456', ...
    'idType',         1, ...
    'uaType',         2, ...
    'protoVer',       2, ...
    'msgCounter',     0, ...
    'ssid',           'DroneIDTest', ...
    'beaconInterval', 100, ...
    'mcs',            0, ...
    'channelBW',      'CBW20', ...
    'idleTime',       20e-6);
fn = fieldnames(defaults);
for i = 1:numel(fn)
    if ~isfield(opts, fn{i})
        opts.(fn{i}) = defaults.(fn{i});
    end
end
end
