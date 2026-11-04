function rxOut = ridReceiver(rxWaveform, rxCfg)
% RIDRECEIVER  Non-HT beacon receiver for Remote ID frames (WLAN Toolbox).
%   Applies the receiver's CFO, then detects the packet, measures RSSI,
%   estimates timing and CFO, estimates the channel and noise,
%   recovers L-SIG and the PSDU, checks the FCS, extracts the Remote ID
%   vendor IE and decodes it with decodeRidMsg.
%
%   rxOut = ridReceiver(rxWaveform, rxCfg)
%
%   Inputs
%     rxWaveform : received baseband waveform at 20 MHz (e.g. chnOut.waveform),
%                  one packet per window
%     rxCfg      : receiver struct (same as applyChannel). Uses
%                    cfo    = 0                    receiver LO offset (Hz)
%                    origin = [53.3438 -6.2546 60] ENU origin for decoded position
%                    visualise = 1                 plot the receiver input
%
%   Output struct rxOut
%     detected    : true if a packet was detected and timing found
%     rssiDbm     : received power over the L-STF (signal + noise), dBm;
%                   NaN if not detected
%     startOffset : packet start (first L-STF sample) as a 0-based sample
%                   offset into rxWaveform; NaN if not detected
%     cfoEst      : estimated CFO, coarse + fine (Hz): transmitter CFO +
%                   Doppler - receiver CFO; NaN if not detected
%     decoded     : true if L-SIG, FCS and Remote ID pack all decoded
%     payload     : struct from decodeRidMsg plus .msgCounter and .srcMAC;
%                   empty if not decoded
%
%   No gain control is applied: the waveform is floating point with no ADC
%   model, and every WLAN Toolbox step used here is insensitive to signal
%   scale (packet detection is normalised, CFO uses phase only, and the
%   channel and noise estimates scale together).

arguments
    rxWaveform (:,1) double {mustBeNonempty}
    rxCfg      (1,1) struct
end

if ~isfield(rxCfg, 'cfo'),    rxCfg.cfo    = 0;                    end
if ~isfield(rxCfg, 'origin'), rxCfg.origin = [53.3438 -6.2546 60]; end
if ~isfield(rxCfg, 'visualise'), rxCfg.visualise = 1;              end

cbw = 'CBW20';
fs  = 20e6;
ind = wlanFieldIndices(wlanNonHTConfig);  % L-STF, L-LTF, L-SIG indices

rxOut = struct('detected', false, 'rssiDbm', NaN, 'startOffset', NaN, 'cfoEst', NaN, ...
               'decoded', false, 'payload', []);

%% Receiver LO offset
n  = (0:numel(rxWaveform)-1).';
rx = rxWaveform .* exp(-1j*2*pi*rxCfg.cfo*n/fs);
N  = numel(rx);

if rxCfg.visualise
    tUs = (0:N-1).' / fs * 1e6;
    figure('Name', 'Receiver input signal');
    plot(tUs, real(rx), tUs, imag(rx));
    grid on;
    xlabel('Time (\mus)');
    ylabel('Amplitude (sqrt(W))');
    title('Receiver input after receiver-LO correction');
    legend('In-phase', 'Quadrature');
end

%% Packet detection (L-STF autocorrelation)
pktOffset = wlanPacketDetect(rx, cbw);
if isempty(pktOffset) || pktOffset + ind.LSIG(2) > N
    rxOut.detected = false;
    return
end

%% RSSI: mean power over the L-STF (waveform is in sqrt(W))
lstf          = rx(pktOffset + (ind.LSTF(1):ind.LSTF(2)));
% Rounded to integer values as ESP32 reports
rxOut.rssiDbm = round(10*log10(mean(abs(lstf).^2)) + 30);

%% Coarse CFO (L-STF)
lstf      = rx(pktOffset + (ind.LSTF(1):ind.LSTF(2)));
coarseCFO = wlanCoarseCFOEstimate(lstf, cbw);
rx        = rx .* exp(-1j*2*pi*coarseCFO*n/fs);

%% Symbol timing (L-LTF cross-correlation)
nonHT     = rx(pktOffset + (ind.LSTF(1):ind.LSIG(2)));
pktOffset = pktOffset + wlanSymbolTimingEstimate(nonHT, cbw);
if pktOffset < 0 || pktOffset + ind.LSIG(2) > N
    return
end

%% Fine CFO (L-LTF)
lltf    = rx(pktOffset + (ind.LLTF(1):ind.LLTF(2)));
fineCFO = wlanFineCFOEstimate(lltf, cbw);
rx      = rx .* exp(-1j*2*pi*fineCFO*n/fs);

rxOut.detected    = true;
rxOut.startOffset = pktOffset;
rxOut.cfoEst      = coarseCFO + fineCFO;

%% Channel and noise estimation (L-LTF)
lltf      = rx(pktOffset + (ind.LLTF(1):ind.LLTF(2)));
lltfDemod = wlanLLTFDemodulate(lltf, cbw);
chanEst   = wlanLLTFChannelEstimate(lltfDemod, cbw);
noiseVar  = wlanLLTFNoiseEstimate(lltfDemod);

%% L-SIG: rate and length
lsig = rx(pktOffset + (ind.LSIG(1):ind.LSIG(2)));
[lsigBits, failCheck] = wlanLSIGRecover(lsig, chanEst, noiseVar, cbw);
if failCheck
    return
end
[mcs, psduLen] = interpretLSIG(lsigBits);
if isnan(mcs) || psduLen == 0
    return
end

%% Data field
cfgRx   = wlanNonHTConfig('ChannelBandwidth', cbw, 'MCS', mcs, 'PSDULength', psduLen);
indData = wlanFieldIndices(cfgRx, 'NonHT-Data');
if pktOffset + indData(2) > N
    return
end
psduBits = wlanNonHTDataRecover(rx(pktOffset + (indData(1):indData(2))), ...
                                chanEst, noiseVar, cfgRx);

%% MAC: FCS check, then Remote ID vendor IE
[cfgMAC, ~, status] = wlanMPDUDecode(psduBits, cfgRx, 'DataFormat', 'bits');
if ~strcmp(char(status), 'Success') || ~strcmp(cfgMAC.FrameType, 'Beacon')
    return
end

psdu = bitsToBytes(psduBits);
[counter, pack] = findRidIE(psdu);
if isempty(pack)
    return
end

[payload, ok] = decodeRidMsg(pack, rxCfg.origin);
if ~ok
    return
end
payload.msgCounter = counter;
payload.srcMAC     = cfgMAC.Address2;

rxOut.decoded = true;
rxOut.payload = payload;
end

% -------------------------------------------------------------------------

function [mcs, len] = interpretLSIG(bits)
    % L-SIG: RATE (4 bits, R1 first), reserved, LENGTH (12 bits, LSB first)
    rates = {'1101', '1111', '0101', '0111', '1001', '1011', '0001', '0011'}; % 6..54 Mbps
    mcs = find(strcmp(rates, char('0' + double(bits(1:4).')))) - 1;
    if isempty(mcs), mcs = NaN; end
    len = sum(double(bits(6:17)) .* 2.^(0:11).');
end

function bytes = bitsToBytes(bits)
    % Column of bits, LSB first per byte -> uint8 row
    bytes = uint8(2.^(0:7) * reshape(double(bits), 8, []));
end

function [counter, pack] = findRidIE(psdu)
    % Beacon: 24-byte MAC header, 12 bytes of fixed fields, IEs, 4-byte FCS.
    % Remote ID IE: DD len FA 0B BC 0D counter pack
    counter = []; pack = [];
    k = 37;
    last = numel(psdu) - 4;
    while k + 1 <= last
        id  = psdu(k);
        len = double(psdu(k+1));
        if k + 1 + len > last, return, end
        body = psdu(k+2 : k+1+len);
        if id == 221 && len >= 6 && isequal(body(1:4), uint8([250 11 188 13]))
            counter = double(body(5));
            pack    = body(6:end);
            return
        end
        k = k + 2 + len;
    end
end