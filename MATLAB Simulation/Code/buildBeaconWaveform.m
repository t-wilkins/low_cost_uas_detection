function tx = buildBeaconWaveform(msg, cfg)
% BUILDBEACONWAVEFORM  Wrap an ASTM F3411 Message Pack in an 802.11 beacon
% and generate the non-HT OFDM waveform. No channel, no power scaling.
%
%   tx = buildBeaconWaveform(msg, cfg)
%
%   Inputs
%     msg            : 1-by-N uint8 Message Pack, from encodeRidMsg
%     cfg (only requiring following fields):
%        beaconInterval : beacon interval header field (TU), default 500 as in the
%                         ESP32 code. Transmit timing is set by the caller.
%        msgCounter     : vendor IE counter, written mod 256 (and 802.11 sequence
%                         number mod 4096). Use the same counter as encodeRidMsg.
%        srcMAC         : 12 hex characters, transmitter address and BSSID
%
%   The waveform is complex baseband at 20 MHz; the carrier (channel 6,
%   2.437 GHz, as in the ESP32 code) is applied in the top level Tx function.
%
%   Output struct tx
%     waveform : non-HT OFDM waveform, 6 Mbps (MCS 0), 20 MHz, not power-scaled
%     fs       : sample rate (Hz)
%     cfgPHY   : wlanNonHTConfig used (needed to demodulate)
%     mpduBits : transmitted MPDU bits incl. FCS (ground truth for BER)
%
%   Frame body: SSID 'UAS_ID_OPEN', beacon interval, WLAN Toolbox default
%   capability and rates, and the Remote ID vendor IE
%   (DD len FA 0B BC 0D counter pack).

%% Apply defaults
if nargin < 1 || isempty(cfg), cfg = struct(); end
def = struct('msgCounter', 0, ...
             'srcMAC', '0A0000000001', ...
             'beaconInterval', 500);
f = fieldnames(def);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = def.(f{i}); end
end

%% Beacon frame body
mgmt = wlanMACManagementConfig;
mgmt.FrameType      = 'Beacon';
mgmt.BeaconInterval = cfg.beaconInterval;
mgmt.SSID           = 'UAS_ID_OPEN';

ridIE = [uint8([250 11 188 13 mod(cfg.msgCounter, 256)]), msg]; % FA 0B BC 0D counter pack
mgmt  = addIE(mgmt, 221, reshape(dec2hex(ridIE, 2).', 1, []));

%% MAC frame
cfgMAC = wlanMACFrameConfig('FrameType', 'Beacon', 'ManagementConfig', mgmt);
cfgMAC.Address1       = 'FFFFFFFFFFFF';                     % broadcast
cfgMAC.Address2       = upper(cfg.srcMAC);                      % transmitter
cfgMAC.Address3       = upper(cfg.srcMAC);                      % BSSID
cfgMAC.SequenceNumber = mod(cfg.msgCounter, 4096);

[mpduBits, mpduLen] = wlanMACFrame(cfgMAC, 'OutputFormat', 'bits');

%% PHY: non-HT OFDM, 6 Mbps, 20 MHz
cfgPHY = wlanNonHTConfig('ChannelBandwidth', 'CBW20', 'MCS', 0, 'PSDULength', mpduLen);

tx.waveform = wlanWaveformGenerator(mpduBits, cfgPHY, 'IdleTime', 0);
tx.fs       = wlanSampleRate(cfgPHY);
tx.cfgPHY   = cfgPHY;
tx.mpduBits = mpduBits;
end
