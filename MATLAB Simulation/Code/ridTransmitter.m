function tx = ridTransmitter(cfg)
% RIDTRANSMITTER  One Remote ID beacon transmission from one drone.
%   Encodes the Message Pack (encodeRidMsg), builds the beacon waveform
%   (buildBeaconWaveform), scales it to transmit power and applies the
%   drone's carrier frequency offset. No channel.
%
%   tx = ridTransmitter(cfg)
%
%   cfg fields (all optional, defaults shown)
%       uasID          = 'A.00001'      UAS serial number, <= 20 characters
%       pos            = [100 100 10]   ENU [x y z] (m), z = height above ground
%       origin         = [53.3438 -6.2546 60]  ENU origin [lat lon h]
%       heading        = 0              degrees clockwise from north
%       speed          = 0              horizontal ground speed (m/s)
%       vertSpeed      = 0              vertical speed (m/s), positive up
%       t              = 0              transmit time (s)
%       msgCounter     = 0              beacon counter; even = pack A, odd = pack B
%       visualise      = 1              plot the transmitted baseband signal
%       srcMAC         = '0A0000000001' 12 hex characters
%       cfo            = 0              carrier frequency offset (Hz), fixed per drone
%       txPowerDbm     = 20             transmit power (dBm)
%       beaconInterval = 500            beacon interval header field (TU)
%
%   Output struct tx
%     waveform : baseband waveform (sqrt(W) scale), transmit power and CFO applied
%     fs       : sample rate (Hz)
%     cfgPHY   : wlanNonHTConfig (needed to demodulate)
%     mpduBits : transmitted MPDU bits (ground truth for BER)
%     msg      : Message Pack bytes (ground truth for decoding)
%     t        : transmit start time (s)
%     duration : waveform duration (s)
%     pos      : 3x1 ENU position (m), input to the channel
%     vel      : 3x1 ENU velocity (m/s) from heading, speed, vertSpeed
%
%   When cfg.visualise is true, plots the transmitted complex baseband
%   signal's in-phase and quadrature components.
        
%% Apply defaults
% Encoder defaults match encodeRidMsg; set here too because pos, heading,
% speed and vertSpeed are also needed for tx.pos and tx.vel.
if nargin < 1 || isempty(cfg), cfg = struct(); end
def = struct('uasID', 'A.00001', ...
             'pos', [100 100 10], ...
             'origin', [53.3438 -6.2546 60], ...
             'heading', 0, ...
             'speed', 0, ...
             'vertSpeed', 0, ...
             't', 0, ...
             'msgCounter', 1, ...
             'visualise', 1, ...
             'srcMAC', '0A0000000001', ...
             'cfo', 0, ...
             'txPowerDbm', 20, ...
             'beaconInterval', 500);
f = fieldnames(def);
for i = 1:numel(f)
    if ~isfield(cfg, f{i}), cfg.(f{i}) = def.(f{i}); end
end

%% Message Pack and beacon waveform
msg = encodeRidMsg(cfg);
b   = buildBeaconWaveform(msg, cfg);
x   = b.waveform;
n   = (0:numel(x)-1).';

%% Transmit power and CFO
x = x * sqrt(10^((cfg.txPowerDbm - 30)/10) / mean(abs(x).^2));
x = x .* exp(1j*2*pi*cfg.cfo*(cfg.t + n/b.fs));    % phase continuous over time

%% Output
tx.waveform = x;
tx.fs       = b.fs;
tx.cfgPHY   = b.cfgPHY;
tx.mpduBits = b.mpduBits;
tx.msg      = msg;
tx.t        = cfg.t;
tx.duration = numel(x) / b.fs;
tx.pos      = cfg.pos(:);
tx.vel      = [cfg.speed * sind(cfg.heading);
               cfg.speed * cosd(cfg.heading);
               cfg.vertSpeed];

if cfg.visualise
    tUs = (0:numel(tx.waveform)-1).' / tx.fs * 1e6;
    figure('Name', 'Transmitted signal');
    plot(tUs, real(tx.waveform), tUs, imag(tx.waveform));
    grid on;
    xlabel('Time (\mus)');
    ylabel('Amplitude (sqrt(W))');
    title('Transmitted baseband signal');
    legend('In-phase', 'Quadrature');
end
end
