function results = simulateScenario(scnCfg)
% SIMULATESCENARIO  Multi-node Remote ID scenario: static drones beaconing
% every 500 ms, received by several fixed receivers.
%   results = simulateScenario(scnCfg)
%
%   Each beacon is transmitted once (ridTransmitter) and passed separately
%   through the channel to every receiver (applyChannel, ridReceiver): one
%   isolated receive window per beacon per receiver, no overlapping frames
%   or collisions.
%
%   scnCfg fields (all optional, defaults shown)
%     seed         = 1                     rng seed, set once at the start
%     simTime      = 10                    simulated time (s)
%     fs           = 20e6                  sample rate (Hz); schedule unit is 1/fs
%     fc           = 2.437e9               carrier frequency (Hz) (Chn 6)
%     origin       = [53.3438 -6.2546 60]  ENU origin [lat lon h]
%     chCfg        : n = 2, d0 = 1, shadowSigmaDb = 0 (see applyChannel);
%                    shadowing drawn once per link, fixed for the run
%     txPos        = [500 1200 50; 1500 300 80; 5000 4000 120]
%                                          ENU (m), one row per drone
%     txPowerDbm   = 20                    scalar or one per drone
%     maxTxCfoPpm  = 10                    drone CFO, uniform in +/- maxTxCfoPpm
%     rxPos        = [0 0 2; 2000 0 2; 2000 2000 2; 0 2000 2]
%                                          ENU (m), one row per receiver
%     nfDb         = 4.5                   scalar or one per receiver
%     maxRxCfoPpm  = 10                    receiver CFO, uniform in +/- maxRxCfoPpm
%     beaconPeriod = 0.5                   beacon period (s), ESP32 / sxjack scheduler
%     minSep       = 1e-3                  minimum spacing between frames of
%                                          different drones (s)
%     verbose      = true                  print one line per beacon
%     plot         = true                  call plotScenario at the end
%
%   Output struct results
%     scnCfg   : configuration used, defaults filled in
%     nodes    : .tx (uasID, srcMAC, pos, cfo, txPowerDbm, t0Tick),
%                .rx (pos, nfDb, cfo), .shadowDb (nTx x nRx, dB)
%     schedule : table [tTick, txIdx, msgCounter] in time order; tTick is
%                int64, in samples at fs
%     log      : table, one row per (beacon, receiver)
%                IDs       eventIdx, tTick, tS, txIdx, rxIdx, msgCounter, pack
%                truth     dist, shadowDb, snrDb, toa, trueCFO, truePos
%                estimates detected, decoded, rssiDbm, cfoEst, cfoErr, toaEst,
%                          decPos, posErr, decUasID, decSrcMAC
%                NaN or "" where not detected or not decoded. Times (tS, toa,
%                toaEst) are absolute seconds and the decoded IDs identify the
%                drone, so a continuous per-receiver buffer can replace the
%                per-beacon windows later.
%
%   Re-plot a saved run with plotScenario(results).

%% 1. Parameters
if nargin < 1 || isempty(scnCfg)
    scnCfg = struct(); 
end
scnCfg = applyDefaults(scnCfg);
rng(scnCfg.seed);

periodTicks = toTicks(scnCfg.beaconPeriod, scnCfg.fs);
minSepTicks = toTicks(scnCfg.minSep, scnCfg.fs);
simTicks    = toTicks(scnCfg.simTime, scnCfg.fs);

%% 2. Nodes
nodes = configureNodes(scnCfg);
nTx   = numel(nodes.tx);
nRx   = numel(nodes.rx);

rxCfgs = struct('pos', {nodes.rx.pos}, 'vel', [0 0 0], 'nfDb', {nodes.rx.nfDb}, ...
                'cfo', {nodes.rx.cfo}, 'origin', scnCfg.origin, 'visualise', 0);

%% 3. Schedule
[schedule, t0Ticks] = buildSchedule(nTx, periodTicks, minSepTicks, simTicks);
for i = 1:nTx
    nodes.tx(i).t0Tick = t0Ticks(i);
end
nEv = height(schedule);

%% 5. Simulation loop
% Results log
rows = cell(nEv*nRx, 1);
r = 0;
for e = 1:nEv
    i = schedule.txIdx(e);
    t = double(schedule.tTick(e)) / scnCfg.fs;

    % a. transmitter state at t
    s = getTxState(nodes.tx(i), t);
    txCfg = struct('uasID', nodes.tx(i).uasID, 'srcMAC', nodes.tx(i).srcMAC, ...
                   'pos', s.pos, 'heading', s.heading, 'speed', s.speed, ...
                   'vertSpeed', s.vertSpeed, 't', t, ...
                   'msgCounter', schedule.msgCounter(e), 'origin', scnCfg.origin, ...
                   'fc', scnCfg.fc, 'cfo', nodes.tx(i).cfo, ...
                   'txPowerDbm', nodes.tx(i).txPowerDbm, 'visualise', 0);

    % b. one transmission per beacon
    tx = ridTransmitter(txCfg);

    % c. each receiver: own channel realisation and receive window
    nDec = 0;
    rxPayloads = cell(1, nRx);                 % payload structs, [] if not decoded
    for m = 1:nRx
        chCfg = scnCfg.chCfg;
        chCfg.shadowDb = nodes.shadowDb(i, m);
        chn = applyChannel(tx.waveform, txCfg, rxCfgs(m), chCfg);
        rx  = ridReceiver(chn.waveform, rxCfgs(m));
        rxPayloads{m} = rx.payload;

        r = r + 1;
        rows{r} = logRow(e, schedule(e,:), m, txCfg, rxCfgs(m), chn, rx);
        nDec = nDec + rx.decoded;
    end

    % d. every receiver that decoded this beacon must recover the same payload
    % (isequaln: NaN fields, e.g. no Basic ID in pack B, count as equal)
    decPayloads = rxPayloads(~cellfun(@isempty, rxPayloads));
    if numel(decPayloads) > 1 && ~isequaln(decPayloads{:})
        error('simulateScenario:payloadMismatch', ...
              'Beacon %d (drone %d): receivers decoded different payloads.', e, i);
    end

    if scnCfg.verbose
        fprintf('%4d/%d  t = %8.4f s  drone %d  pack %s  decoded at %d/%d receivers\n', ...
                e, nEv, t, i, rows{r}.pack, nDec, nRx);
    end
end
logTbl = struct2table(vertcat(rows{:}));
logTbl.pack = categorical(logTbl.pack, ["A" "B"]);

%% Output
results.scnCfg   = scnCfg;
results.nodes    = nodes;
results.schedule = schedule;
results.log      = logTbl;

%% 6. Plot
if scnCfg.plot
    plotScenario(results);
end
end

% -------------------------------------------------------------------------

function cfg = applyDefaults(cfg)
    def = struct('seed', 1, ...
                 'simTime', 10, ...
                 'fs', 20e6, ...
                 'fc', 2.437e9, ...
                 'origin', [53.0105 -7.1555 88], ...
                 'chCfg', struct(), ...
                 'txPos', [-400 600 50; 1000 -1500 80; -5000 -4000 120], ...
                 'txPowerDbm', 20, ...
                 'maxTxCfoPpm', 10, ...
                 'rxPos', [0 0 2; -2000 0 2; -2000 -2000 2; 0 -2000 2], ...
                 'nfDb', 4.5, ...
                 'maxRxCfoPpm', 10, ...
                 'beaconPeriod', 0.5, ...
                 'minSep', 1e-3, ...
                 'verbose', true, ...
                 'plot', true);
    f = fieldnames(def);
    for i = 1:numel(f)
        if ~isfield(cfg, f{i}), cfg.(f{i}) = def.(f{i}); end
    end
    chDef = struct('n', 2, 'd0', 1, 'shadowSigmaDb', 0);   % as applyChannel
    f = fieldnames(chDef);
    for i = 1:numel(f)
        if ~isfield(cfg.chCfg, f{i}), cfg.chCfg.(f{i}) = chDef.(f{i}); end
    end
end

function n = toTicks(tSec, fs)
    % Seconds to an integer number of samples at fs; must be exact
    x = tSec * fs;
    if abs(x - round(x)) > 1e-6
        error('simulateScenario:ticks', '%g s is not a whole number of samples at %g Hz.', tSec, fs);
    end
    n = int64(round(x));
end

function nodes = configureNodes(cfg)
    nTx = size(cfg.txPos, 1);
    nRx = size(cfg.rxPos, 1);
    txPow = expand(cfg.txPowerDbm, nTx, 'txPowerDbm');
    nf    = expand(cfg.nfDb, nRx, 'nfDb');

    for i = 1:nTx
        tx(i, 1).uasID      = sprintf('A.%05d', i);              %#ok<AGROW>
        tx(i, 1).srcMAC     = sprintf('0A%010X', i);
        tx(i, 1).pos        = cfg.txPos(i,:);
        tx(i, 1).cfo        = cfg.maxTxCfoPpm * 1e-6 * cfg.fc * (2*rand - 1);
        tx(i, 1).txPowerDbm = txPow(i);
        tx(i, 1).t0Tick     = int64(0);                          % set by buildSchedule
    end
    for m = 1:nRx
        rx(m, 1).pos  = cfg.rxPos(m,:);                          %#ok<AGROW>
        rx(m, 1).nfDb = nf(m);
        rx(m, 1).cfo  = cfg.maxRxCfoPpm * 1e-6 * cfg.fc * (2*rand - 1);
    end

    nodes.tx       = tx;
    nodes.rx       = rx;
    nodes.shadowDb = cfg.chCfg.shadowSigmaDb * randn(nTx, nRx);  % one draw per link
end

function v = expand(v, n, name)
    if isscalar(v)
        v = repmat(v, n, 1);
    elseif numel(v) ~= n
        error('simulateScenario:size', '%s must be scalar or have %d elements.', name, n);
    end
    v = v(:);
end

function [schedule, t0] = buildSchedule(nTx, periodTicks, minSepTicks, simTicks)
    % Random start offset per drone in [0, period), at least minSep from
    % every other drone's offset (circularly, as all share the period).
    t0 = zeros(nTx, 1, 'int64');
    for i = 1:nTx
        ok = false;
        for attempt = 1:1000 % Number of start offset trials
            c = int64(randi([0, double(periodTicks) - 1])); % Random start
            d = mod(c - t0(1:i-1), periodTicks); % Samples since last start
            if all(min(d, periodTicks - d) >= minSepTicks) % If start offset acceptable
                ok = true;
                break
            end
        end
        if ~ok
            error('simulateScenario:schedule', ...
                  'Could not place drone %d; reduce the number of drones or minSep.', i);
        end
        t0(i) = c;
    end

    % t_k = t0 + k*period, merged over all drones and sorted by time
    tTick      = zeros(0, 1, 'int64');
    txIdx      = zeros(0, 1);
    msgCounter = zeros(0, 1);
    for i = 1:nTx
        k = (0:floor(double(simTicks - 1 - t0(i)) / double(periodTicks))).';
        tTick      = [tTick; t0(i) + int64(k)*periodTicks];      %#ok<AGROW>
        txIdx      = [txIdx; repmat(i, numel(k), 1)];            %#ok<AGROW>
        msgCounter = [msgCounter; k];                            %#ok<AGROW>
    end
    schedule = sortrows(table(tTick, txIdx, msgCounter), {'tTick', 'txIdx'});
    assert(all(diff(schedule.tTick) >= minSepTicks), 'simulateScenario:overlap', ...
           'Schedule has frames closer than minSep.');
end

function s = getTxState(txNode, t) %#ok<INUSD>
    % Transmitter kinematic state at time t (s). Static for now: fixed
    % position, zero velocity. Trajectories go here without changing the loop.
    s.pos       = txNode.pos;
    s.heading   = 0;
    s.speed     = 0;
    s.vertSpeed = 0;
end

function row = logRow(e, ev, m, txCfg, rxCfg, chn, rx)
    % One log row per (received beacon): IDs, truth, estimates
    row.eventIdx   = e;
    row.tTick      = ev.tTick;
    row.tS         = txCfg.t;
    row.txIdx      = ev.txIdx;
    row.rxIdx      = m;
    row.msgCounter = ev.msgCounter;
    row.pack       = string(char('A' + mod(ev.msgCounter, 2)));

    row.dist     = chn.dist;
    row.shadowDb = chn.shadowDb;
    row.snrDb    = chn.snrDb;
    row.toa      = chn.toa;
    row.trueCFO  = txCfg.cfo + chn.fD - rxCfg.cfo;
    row.truePos  = txCfg.pos(:).';

    row.detected  = rx.detected;
    row.decoded   = rx.decoded;
    row.rssiDbm   = rx.rssiDbm;
    row.cfoEst    = rx.cfoEst;
    row.decPos    = nan(1, 3);
    row.decLla    = nan(1, 3);
    row.posErr    = nan(1, 3);
    row.decUasID  = "";
    row.decSrcMAC = "";
    row.timestamp = nan;
    row.macTimestamp = nan;                    % beacon TSF (us)
    if rx.decoded
        row.decPos    = rx.payload.pos(:).';
        row.decLla    = rx.payload.lla(:).';
        row.posErr    = row.decPos - row.truePos;
        row.decUasID  = string(rx.payload.uasID); % "" for pack B (no Basic ID)
        row.decSrcMAC = string(rx.payload.srcMAC);
        row.timestamp = rx.payload.timestamp;
        row.macTimestamp = rx.payload.macTimestamp;
    end
end
