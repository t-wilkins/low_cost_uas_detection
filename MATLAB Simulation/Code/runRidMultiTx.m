%% runRidMultiTx.m
% Several Remote ID transmitters at different distances and beacon
% intervals, one receiver. Part 1 runs a single scenario and reports which
% transmitters were decoded. Part 2 repeats it over a range of one config
% field (default: number of transmitters) to see how detection changes.

clear; close all;

%% Scenario (all fields optional - defaults are listed in ridSimulateScenario.m)
cfg = struct( ...
    'seed',          2026, ...
    'nTx',           10, ...
    'simTime',       10, ...            % s
    'distRange',     [10 300], ...      % m, initial Tx-Rx distance
    'speedMax',      10, ...            % m/s, radial
    'intervalRange', [0.1024 1.0], ...  % s between beacons, per transmitter
    'jitterFrac',    0.05, ...
    'txPowerDBm',    20, ...
    'cfoPpmMax',     10);

%% Part 1: single scenario
out = ridSimulateScenario(cfg);

fprintf('Packet airtime         : %.0f us (channel load %.2f%%)\n', 1e6*out.airtime, 100*out.channelLoad);
fprintf('Transmitters detected  : %d of %d\n', out.nTxDetected, out.nTx);
fprintf('Packets decoded        : %d of %d (%.1f%%)\n', out.nDecoded, out.nPackets, 100*out.nDecoded/out.nPackets);
fprintf('Preamble locks         : %d of %d packets\n', out.nLocked, out.nPackets);
fprintf('Packets overlapping    : %d (%.1f%%)\n', out.nCollided, 100*out.nCollided/out.nPackets);
fprintf('Locks with no packet   : %d\n', out.nFalseAlarms);
fprintf('Receive exceptions     : %d\n\n', out.nRxErrors);
disp(out.tx)

tx = out.tx;
p  = out.pkt;

figure('Name', 'Single scenario');
subplot(2,1,1);
scatter(tx.meanDist, 100*tx.decodeRate, 40, 'filled'); grid on;
xlabel('Mean distance (m)'); ylabel('Packets decoded (%)');
title('Decode rate by transmitter'); ylim([-5 105]);

subplot(2,1,2); hold on; grid on;
[~, order] = sort(tx.meanDist);
rankOf = zeros(height(tx), 1);
rankOf(order) = 1:height(tx);
y = rankOf(p.tx);
dec  = p.decoded;
miss = ~p.decoded & ~p.collided;
col  = ~p.decoded & p.collided;
plot(p.t(dec),  y(dec),  '.', 'Color', [0 0.6 0], 'MarkerSize', 14);
plot(p.t(col),  y(col),  'x', 'Color', [1 0.5 0], 'MarkerSize', 8);
plot(p.t(miss), y(miss), 'X', 'Color', [0.8 0 0], 'MarkerSize', 8);
xlabel('Time (s)'); ylabel('Transmitter (1 = nearest)');
title('Packet outcomes');
legend('Decoded', 'Missed, overlapped another packet', 'Missed', 'Location', 'eastoutside');

%% Part 2: sweep one config field
runSweep    = true;
sweepField  = 'nTx';                % any scalar field of cfg
sweepValues = [1 5 10 25 50];
nTrials     = 3;                    % independent draws per value
sweepCfg    = cfg;
sweepCfg.simTime = 5;               % shorter than part 1 to keep the sweep quick

if runSweep
    nV = numel(sweepValues);
    detPct = zeros(nV, nTrials);
    decPct = zeros(nV, nTrials);
    colPct = zeros(nV, nTrials);

    for vi = 1:nV
        for tr = 1:nTrials
            c = sweepCfg;
            c.(sweepField) = sweepValues(vi);
            c.seed = cfg.seed + 1000*vi + tr;
            o = ridSimulateScenario(c);
            detPct(vi, tr) = 100 * o.nTxDetected / o.nTx;
            decPct(vi, tr) = 100 * o.nDecoded   / o.nPackets;
            colPct(vi, tr) = 100 * o.nCollided  / o.nPackets;
        end
        fprintf('%s = %g done\n', sweepField, sweepValues(vi));
    end

    figure('Name', 'Sweep');
    plot(sweepValues, mean(detPct, 2), '-o', ...
         sweepValues, mean(decPct, 2), '-s', ...
         sweepValues, mean(colPct, 2), '-^');
    grid on;
    xlabel(sweepField); ylabel('%');
    legend('Transmitters detected', 'Packets decoded', 'Packets overlapping another', 'Location', 'best');
    title(sprintf('Mean over %d trials', nTrials));
end
