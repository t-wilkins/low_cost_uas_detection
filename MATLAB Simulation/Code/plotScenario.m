function plotScenario(results)
% PLOTSCENARIO  Plots for one simulateScenario run, built from the results 
% struct (scnCfg, nodes, log), so a saved run can be re-plotted without 
% re-simulating.
%   plotScenario(results)
%
%   1. Starting node map: receivers, drones, at t=0 links coloured by decode rate
%   2. Trajectory map: true vs decoded vs estimated node positions throughout simulation 
%   3. Decode rate per link (drone x receiver)
%   4. Timestamps per drone over the run: transmission time, MAC (TSF)
%      timestamp, RID Location timestamp, true ToA at each receiver

cfg   = results.scnCfg;
nodes = results.nodes;
L     = results.log;
nTx   = numel(nodes.tx);
nRx   = numel(nodes.rx);

txPos     = vertcat(nodes.tx.pos);
rxPos     = vertcat(nodes.rx.pos);
txDecPos  = vertcat(L.decPos);
txTruePos = vertcat(L.truePos);

% Convert positions to LLA
txLla    = enu2lla(txPos, cfg.origin, 'flat');
rxLla    = enu2lla(rxPos, cfg.origin, 'flat');
txDecLla = enu2lla(txDecPos, cfg.origin, 'flat');
txTrueLla = enu2lla(txTruePos, cfg.origin, 'flat');

txNames  = "TX " + (1:nTx);
rxNames  = "RX " + (1:nRx);

rate = accumarray([L.txIdx L.rxIdx], double(L.decoded), [nTx nRx], @mean, NaN);

%% 1. Node map
figure('Name', 'Node map');
gx = geoaxes;
hold(gx, 'on');
cmap = parula(256);
for i = 1:nTx
    for m = 1:nRx
        if isnan(rate(i,m)), continue, end
        geoplot([txLla(i,1) rxLla(m,1)], [txLla(i,2) rxLla(m,2)], '-', ...
             'Color', cmap(1 + round(rate(i,m)*255), :), 'LineWidth', 1.5, ...
             'HandleVisibility', 'off');
    end
end
hRx = geoplot(rxLla(:,1), rxLla(:,2), 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 9);
hTx = geoplot(txLla(:,1), txLla(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
text(rxLla(:,1), rxLla(:,2), "  " + rxNames);
text(txLla(:,1), txLla(:,2), "  " + txNames);
colormap(cmap);
clim([0 1]);
cb = colorbar;
cb.Label.String = 'Decode rate';
title('Node map');
legend([hRx hTx], {'Receiver', 'Drone'}, 'Location', 'best');
geobasemap streets

%% 2. Decode rate per link
figure('Name', 'Decode rate per link');
h = heatmap(rxNames, txNames, rate);
h.Title           = 'Decode rate per link';
h.XLabel          = 'Receiver';
h.YLabel          = 'Drone';
h.ColorLimits     = [0 1];
h.CellLabelFormat = '%.2f';

%% 3. Trajectory Map: True vs Transmitted
% True position once per beacon (each beacon is logged once per receiver);
% decoded position once per successful reception, i.e. the transmitted coordinates
[~, ev]  = unique(L.eventIdx);             % first row of each beacon, in time order

figure('Name', 'Trajectory map');
gx = geoaxes;
hold(gx, 'on');
for i = 1:nTx
    b = ev(L.txIdx(ev) == i);
    geoplot(txTrueLla(b,1), txTrueLla(b,2), '-b', 'LineWidth', 1.5, ...
        'DisplayName', txNames(i) + " true");
    d = L.txIdx == i & L.decoded;
    geoplot(txDecLla(d,1), txDecLla(d,2), '--r', 'LineWidth', 1.5, ...
        'DisplayName', txNames(i) + " decoded");
end
hRx = geoplot(rxLla(:,1), rxLla(:,2), 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 9);
hTx = geoplot(txTrueLla(:,1), txTrueLla(:,2), 'x', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
text(rxLla(:,1), rxLla(:,2), "  " + rxNames);
text(txLla(:,1), txLla(:,2), "  " + txNames + " @ t0");
title('True vs decoded drone positions');
legend([hRx hTx], {'Receiver', 'Drone'}, 'Location', 'best');
geobasemap streets

%% 4. Timestamps
% One row per drone. The timestamps of one beacon are microseconds to
% milliseconds apart, so at full scale they coincide: each type sits at its
% own vertical offset in the drone's row; zoom in on a beacon to see the gaps.
dec = find(L.decoded);
[~, k] = unique(L.eventIdx(dec));          % first decoded row of each beacon
dec = dec(k);
T   = cfg.beaconPeriod;
ts  = 1/cfg.fs;
nT  = floor(cfg.simTime / T);

figure('Name', 'Timestamps', 'Position', [100 100 1100 750]);
ax = axes('Position', [0.08 0.40 0.88 0.50]);
hold(ax, 'on'); grid(ax, 'on');
xline(ax, T*(1:nT), '--', "T_{" + (1:nT) + "}", 'Color', [0.5 0.5 0.5], ...
      'HandleVisibility', 'off');
plot(ax, L.tS(ev), L.txIdx(ev) + 0.24, 'o', 'Color', [0 0.6 0], ...
     'MarkerFaceColor', [0 0.6 0], 'DisplayName', '(1) Transmission time t_k');
plot(ax, L.macTimestamp(dec)*1e-6, L.txIdx(dec) + 0.08, 'o', 'Color', [0 0.3 0.9], ...
     'MarkerFaceColor', [0 0.3 0.9], 'DisplayName', '(2) MAC timestamp (TSF, 1 \mus)');
plot(ax, L.timestamp(dec), L.txIdx(dec) - 0.08, 'd', 'Color', [0.9 0.5 0], ...
     'MarkerFaceColor', [0.9 0.5 0], 'DisplayName', '(3) RID Location timestamp (0.1 s)');
plot(ax, L.toa, L.txIdx - 0.24, 'o', 'Color', [0.85 0 0.55], ...
     'MarkerFaceColor', [0.85 0 0.55], 'MarkerSize', 4, 'DisplayName', '(4) True ToA at each Rx');
xlim(ax, [0 cfg.simTime]);
ylim(ax, [0.5 nTx + 0.5]);
yticks(ax, 1:nTx);
yticklabels(ax, txNames);
xlabel(ax, 't (s)');
title(ax, 'Timestamps per beacon');
legend(ax, 'Location', 'northoutside', 'Orientation', 'horizontal');

% Legend notes: what each timestamp represents and the assumptions behind it
notes = {
    ['(1) Transmission time t_k = t_0 + kT_b (log.tS), T_b = ' num2str(T*1e3) ...
     ' ms, integer samples at f_s. One instant at which the Message Pack is encoded, ' ...
     'the beacon waveform built and the whole frame transmitted: no processing, queueing ' ...
     'or CSMA/CA delay and no clock drift. t_k is the first L-STF sample.']
    ['(2) MAC timestamp: beacon Timestamp field (TSF) decoded at the receiver, floor(t_k 10^6) \mus, ' ...
     '1 \mus resolution. Taken from the shared simulation clock (no per-drone TSF offset or drift) ' ...
     'and stamped at frame start; 802.11 stamps it when the symbol carrying the field is sent ' ...
     '(~52 \mus later for this frame at 6 Mb/s) plus PHY delay. One per beacon, if any receiver decoded it.']
    ['(3) RID Location timestamp (ASTM F3411, bytes 22-23) decoded at the receiver: tenths of a ' ...
     'second since the hour, round(10 t_k) as in opendroneid-core-c, 0.1 s resolution, so up to ' ...
     '\pm50 ms from t_k. Simulation t = 0 taken as the top of the hour; position fix time = t_k ' ...
     '(no GNSS latency). Carried in both pack A and pack B.']
    ['(4) True ToA from the channel: t_k + d/c at each receiver, line of sight, exact (not ' ...
     'quantised to samples). One per receiver, whether or not the beacon was detected.']
    'T_n: beacon period boundaries, n T_b.'};
annotation('textbox', [0.08 0.01 0.88 0.30], 'String', notes, 'EdgeColor', 'none', ...
           'FontSize', 8, 'VerticalAlignment', 'top');

