function out = ridSimulateScenario(cfg)
% RIDSIMULATESCENARIO  Several Remote ID beacon transmitters, each with its
% own distance, CFO and beacon interval, heard by one receiver.
%
%   out = ridSimulateScenario(cfg)
%
% Each transmitter repeats the same beacon waveform (own UAS ID) at its own
% interval. Transmissions are placed on a common time axis. Packets that
% overlap in time are summed at the receiver; packets that do not are
% simulated in their own short buffer, so long simulated times stay cheap.
% Every transmission gets its own TGn channel draw (path loss, shadowing,
% fading) and the transmitter's CFO. Thermal noise is added once per buffer.
%
% Receiver model: scan the buffer for a preamble, try to decode, then stay
% busy for one packet duration after every lock, whether or not the decode
% worked. A packet that starts while the receiver is busy is lost. The
% transmitters do not carrier-sense (pure ALOHA).
%
% "Detected" = at least one packet from that transmitter passed the FCS
% and its UAS ID was recovered.
%
% cfg fields (all optional):
%   seed           2026         RNG seed ([] leaves the RNG alone)
%   nTx            10           number of transmitters
%   simTime        10           s of air time
%   distRange      [10 300]     m, initial Tx-Rx distance, uniform
%   speedMax       10           m/s, radial speed drawn uniform in +/-speedMax
%   intervalRange  [0.1024 1]   s, nominal beacon interval per transmitter
%   jitterFrac     0.05         per-beacon timing jitter, +/- fraction of interval
%   txPowerDBm     20           transmit power (waveform is rescaled to this)
%   cfoPpmMax      10           per-transmitter CFO drawn uniform in +/-ppm
%   noiseFigureDB  4.5          receiver noise figure
%   delayProfile   'Model-D'    TGn delay profile
%   carrierFreq    2.4e9        Hz
%   padSamples     400          noise-only samples either side of each buffer
%
% out fields:
%   tx    table, one row per transmitter
%   pkt   table, one row per transmission
%   nTx, nTxDetected, nPackets, nDecoded, nLocked, nCollided
%   nFalseAlarms  locks that do not line up with any transmission
%   nRxErrors     receive-chain exceptions (should be 0)
%   airtime (s), channelLoad (fraction), noiseVar (W), fs (Hz), cfg

if nargin < 1, cfg = struct(); end
cfg = applyDefaults(cfg);
if ~isempty(cfg.seed), rng(cfg.seed); end
nTx = cfg.nTx;

%% Transmitter parameters
ids      = cellstr(compose('UAS%04d', (1:nTx)'));
dist0    = cfg.distRange(1) + diff(cfg.distRange) * rand(nTx, 1);
speed    = cfg.speedMax * (2*rand(nTx, 1) - 1);          % radial, m/s (+ = moving away)
interval = cfg.(1) + diff(cfg.intervalRange) * rand(nTx, 1);
phase    = interval .* rand(nTx, 1);                      % time of first beacon
cfoHz    = cfg.cfoPpmMax * 1e-6 * cfg.carrierFreq * (2*rand(nTx, 1) - 1);

%% Waveform bank (one per transmitter, scaled to the requested Tx power)
wave = cell(nTx, 1);
for i = 1:nTx
    tu = min(65535, max(1, round(interval(i) / 1.024e-3)));   % keep the beacon-interval field consistent
    built = buildRidBeaconWaveform(struct('uasID', ids{i}, 'beaconInterval', tu));
    if i == 1
        cfgPHY  = built.cfgPHY;
        ind     = built.ind;
        fs      = built.fs;
        airLen  = double(ind.NonHTData(2));                       % packet length without trailing idle
        pAvg    = mean(abs(built.waveform(1:airLen)).^2);
        txScale = sqrt(10^((cfg.txPowerDBm - 30)/10) / pAvg);   % watts
    end
    wave{i} = built.waveform * txScale;
end
Lw  = numel(wave{1});
pad = cfg.padSamples;

noiseVar = 10^((-228.6 + 10*log10(290) + 10*log10(fs) + cfg.noiseFigureDB)/10);   % kTBF, watts

%% Transmission schedule
pk = zeros(0, 2);                                         % [txIdx, startTime]
for i = 1:nTx
    nPk = floor((cfg.simTime - phase(i)) / interval(i)) + 1;
    k   = (0:nPk-1)';
    t   = phase(i) + k*interval(i) + interval(i)*cfg.jitterFrac*(2*rand(nPk, 1) - 1);
    t   = t(t >= 0 & t < cfg.simTime);
    pk  = [pk; repmat(i, numel(t), 1), t]; %#ok<AGROW>
end
if isempty(pk)
    error('ridSimulateScenario:noPackets', 'No transmissions scheduled - increase simTime.');
end
pk     = sortrows(pk, 2);
nPk    = size(pk, 1);
pkTx   = pk(:, 1);
pkT    = pk(:, 2);
pkDist = max(1, dist0(pkTx) + speed(pkTx) .* pkT);        % radial motion only for now
pkS    = round(pkT * fs);                                 % start sample on the common axis

%% Group overlapping transmissions into clusters
cid  = zeros(nPk, 1);
c    = 1;
cid(1) = 1;
cEnd = pkS(1) + Lw;
for k = 2:nPk
    if pkS(k) > cEnd + 2*pad
        c    = c + 1;
        cEnd = pkS(k) + Lw;
    else
        cEnd = max(cEnd, pkS(k) + Lw);
    end
    cid(k) = c;
end
nClusters = c;

%% Build each buffer, scan it, score the result
pkCollided = false(nPk, 1);
pkLocked   = false(nPk, 1);
pkDecoded  = false(nPk, 1);
cfoEstPk   = nan(nPk, 1);
nFalse     = 0;
nRxErrors  = 0;

for c = 1:nClusters
    m  = find(cid == c);
    sm = pkS(m);

    % Time overlap between packets in this cluster
    ov = (sm < sm.' + airLen) & (sm.' < sm + airLen);
    ov(1:numel(m)+1:end) = false;
    pkCollided(m) = any(ov, 2);

    % Superpose the propagated packets, then add thermal noise once
    bufStart = sm(1) - pad;
    bufLen   = max(sm) + Lw + pad - bufStart;
    buf      = zeros(bufLen, 1);
    for j = 1:numel(m)
        k = m(j);
        y = propagate(wave{pkTx(k)}, fs, pkDist(k), cfoHz(pkTx(k)), cfg);
        o = pkS(k) - bufStart;
        buf(o+1:o+Lw) = buf(o+1:o+Lw) + y;
    end
    buf = buf + sqrt(noiseVar/2) * complex(randn(bufLen, 1), randn(bufLen, 1));

    % Receiver
    [locks, nErr] = scanBuffer(buf, cfgPHY, ind, fs, noiseVar, airLen);
    nRxErrors = nRxErrors + nErr;

    % Match locks to transmissions
    trueLocal = sm - bufStart;                            % true start of each packet in this buffer
    for q = 1:numel(locks)
        lk = locks(q);
        d  = lk.start - trueLocal;                        % lock minus true start
        cand = find(d >= -160 & d < airLen);              % allow one L-STF of early timing error
        if isempty(cand)
            nFalse = nFalse + 1;
        else
            [~, j] = min(abs(d(cand)));
            pkLocked(m(cand(j))) = true;
        end
        if lk.decoded
            tIdx  = find(strcmp(ids, lk.id), 1);
            cand2 = m(pkTx(m) == tIdx & ~pkDecoded(m));
            if ~isempty(cand2)
                [~, j2] = min(abs(pkS(cand2) - bufStart - lk.start));
                pkDecoded(cand2(j2)) = true;
                pkLocked(cand2(j2))  = true;
                cfoEstPk(cand2(j2))  = lk.cfoEst;
            end
        end
    end
end

if nRxErrors > 0
    warning('ridSimulateScenario:rxErrors', ...
        '%d receive-chain exception(s); affected buffers were cut short.', nRxErrors);
end

%% Per-transmitter summary
nSent    = accumarray(pkTx, 1, [nTx 1]);
nLocked  = accumarray(pkTx, double(pkLocked),  [nTx 1]);
nDecoded = accumarray(pkTx, double(pkDecoded), [nTx 1]);
meanDist = accumarray(pkTx, pkDist, [nTx 1], @mean);
decodeRate = nDecoded ./ max(nSent, 1);
tFirst = nan(nTx, 1);
for i = 1:nTx
    k = find(pkTx == i & pkDecoded, 1);                   % pk is time-sorted
    if ~isempty(k), tFirst(i) = pkT(k); end
end

%% Output
out.cfg  = cfg;
out.fs   = fs;
out.noiseVar = noiseVar;
out.airtime  = airLen / fs;
out.channelLoad = nPk * out.airtime / cfg.simTime;
out.nTx = nTx;
out.nTxDetected = sum(nDecoded > 0);
out.nPackets  = nPk;
out.nDecoded  = sum(pkDecoded);
out.nLocked   = sum(pkLocked);
out.nCollided = sum(pkCollided);
out.nFalseAlarms = nFalse;
out.nRxErrors = nRxErrors;

out.tx = table((1:nTx)', ids, dist0, speed, interval, cfoHz, meanDist, nSent, nLocked, nDecoded, decodeRate, tFirst, ...
    'VariableNames', {'idx','id','dist0','speed','interval','cfoHz','meanDist','nSent','nLocked','nDecoded','decodeRate','tFirst'});
out.pkt = table(pkTx, pkT, pkDist, cid, pkCollided, pkLocked, pkDecoded, cfoEstPk, ...
    'VariableNames', {'tx','t','dist','cluster','collided','locked','decoded','cfoEst'});
end


%% ==================================================================
%  Local helpers
%  ==================================================================
function y = propagate(x, fs, dist, cfoHz, cfg)
% One transmission through its own TGn channel draw, plus the transmitter CFO.
chan = wlanTGnChannel( ...
    'SampleRate',              fs, ...
    'DelayProfile',            cfg.delayProfile, ...
    'CarrierFrequency',        cfg.carrierFreq, ...
    'NumTransmitAntennas',     1, ...
    'NumReceiveAntennas',      1, ...
    'LargeScaleFadingEffect',  'Pathloss and shadowing', ...
    'TransmitReceiveDistance', dist);
y = chan(x);
y = frequencyOffset(y, fs, cfoHz);
end

function [locks, nErr] = scanBuffer(buf, cfgPHY, ind, fs, noiseVar, airLen)
% Repeatedly detect and decode. After each lock the receiver skips one
% packet duration, decoded or not.
locks = struct('start', {}, 'decoded', {}, 'id', {}, 'cfoEst', {});
nErr = 0;
pos  = 0;
while numel(buf) - pos >= airLen
    try
        r = receiveRidBeacon(buf(pos+1:end), cfgPHY, ind, fs, noiseVar, []);
    catch ME
        nErr = nErr + 1;
        if nErr == 1
            warning('ridSimulateScenario:rxException', '%s', ME.message);
        end
        break;      % lock position unknown, abandon the rest of this buffer
    end
    if ~r.detected, break; end

    lk = struct('start', pos + r.startOffset, 'decoded', false, 'id', '', 'cfoEst', NaN);
    if r.decodeOK && r.ridFound && ~isempty(r.basicID)
        lk.decoded = true;
        lk.id      = r.basicID.uasID;
        lk.cfoEst  = r.coarseCFO + r.fineCFO;
    end
    locks(end+1) = lk; %#ok<AGROW>
    pos = lk.start + airLen;
end
end

function cfg = applyDefaults(cfg)
defaults = struct( ...
    'seed',          2026, ...
    'nTx',           10, ...
    'simTime',       10, ...
    'distRange',     [10 300], ...
    'speedMax',      10, ...
    'intervalRange', [0.1024 1.0], ...
    'jitterFrac',    0.05, ...
    'txPowerDBm',    20, ...
    'cfoPpmMax',     10, ...
    'noiseFigureDB', 4.5, ...
    'delayProfile',  'Model-D', ...
    'carrierFreq',   2.4e9, ...
    'padSamples',    400);
fn = fieldnames(defaults);
for i = 1:numel(fn)
    if ~isfield(cfg, fn{i})
        cfg.(fn{i}) = defaults.(fn{i});
    end
end
end
