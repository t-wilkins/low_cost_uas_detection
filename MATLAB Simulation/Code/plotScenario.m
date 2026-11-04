function plotScenario(results)
% PLOTSCENARIO  Plots for one simulateScenario run, built only from the
% results struct (scnCfg, nodes, log), so a saved run can be re-plotted
% without re-simulating.
%   plotScenario(results)
%
%   1. Node map: receivers, drones, links coloured by decode rate
%   2. Decode rate per link (drone x receiver)
%   3. RSSI vs distance, with the log-distance path loss model
%   4. CFO estimate error vs SNR, and its distribution

cfg   = results.scnCfg;
nodes = results.nodes;
L     = results.log;
nTx   = numel(nodes.tx);
nRx   = numel(nodes.rx);

txPos   = vertcat(nodes.tx.pos);
rxPos   = vertcat(nodes.rx.pos);
txNames = "TX " + (1:nTx);
rxNames = "RX " + (1:nRx);
colors  = lines(nRx);

rate = accumarray([L.txIdx L.rxIdx], double(L.decoded), [nTx nRx], @mean, NaN);

%% 1. Node map
figure('Name', 'Node map');
hold on; grid on; axis equal;
cmap = parula(256);
for i = 1:nTx
    for m = 1:nRx
        if isnan(rate(i,m)), continue, end
        plot([txPos(i,1) rxPos(m,1)], [txPos(i,2) rxPos(m,2)], '-', ...
             'Color', cmap(1 + round(rate(i,m)*255), :), 'LineWidth', 1.5, ...
             'HandleVisibility', 'off');
    end
end
hRx = plot(rxPos(:,1), rxPos(:,2), 'k^', 'MarkerFaceColor', 'k', 'MarkerSize', 9);
hTx = plot(txPos(:,1), txPos(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
text(rxPos(:,1), rxPos(:,2), "  " + rxNames);
text(txPos(:,1), txPos(:,2), "  " + txNames);
colormap(cmap);
clim([0 1]);
cb = colorbar;
cb.Label.String = 'Decode rate';
xlabel('East (m)');
ylabel('North (m)');
title('Node map');
legend([hRx hTx], {'Receiver', 'Drone'}, 'Location', 'best');

%% 2. Decode rate per link
figure('Name', 'Decode rate per link');
h = heatmap(rxNames, txNames, rate);
h.Title           = 'Decode rate per link';
h.XLabel          = 'Receiver';
h.YLabel          = 'Drone';
h.ColorLimits     = [0 1];
h.CellLabelFormat = '%.2f';

%% 3. RSSI vs distance
lambda = physconst('LightSpeed') / cfg.fc;
kB     = physconst('Boltzmann');
ch     = cfg.chCfg;
plDb   = @(d) 20*log10(4*pi*ch.d0/lambda) + 10*ch.n*log10(d/ch.d0);
nfDbm  = 10*log10(kB * 290 * cfg.fs * 10^(mean([nodes.rx.nfDb])/10)) + 30;   % kTBF, B = fs
d      = logspace(log10(max(1, min(L.dist)/2)), log10(max(L.dist)*2), 200);

figure('Name', 'RSSI vs distance');
hold on; grid on;
set(gca, 'XScale', 'log');
for m = 1:nRx
    sel = L.rxIdx == m & L.detected;
    scatter(L.dist(sel), L.rssiDbm(sel), 24, colors(m,:), 'filled', 'DisplayName', rxNames(m));
end
und = L.detected & ~L.decoded;
if any(und)
    scatter(L.dist(und), L.rssiDbm(und), 60, 'k', 'x', 'DisplayName', 'Detected, not decoded');
end
for p = unique([nodes.tx.txPowerDbm])
    S = p - plDb(d);                                % mean received signal power, dBm
    plot(d, S, 'k-', 'DisplayName', sprintf('Model signal, %g dBm tx', p));
    plot(d, 10*log10(10.^(S/10) + 10^(nfDbm/10)), 'k--', 'DisplayName', 'Model signal + noise');
    if ch.shadowSigmaDb > 0
        plot(d, S + ch.shadowSigmaDb, 'k:', 'DisplayName', sprintf('\\pm%g dB shadowing', ch.shadowSigmaDb));
        plot(d, S - ch.shadowSigmaDb, 'k:', 'HandleVisibility', 'off');
    end
end
yline(nfDbm, '-.', 'Color', 'r', 'DisplayName', 'Noise floor');
xlabel('Distance (m)');
ylabel('RSSI (dBm)');
title(sprintf('RSSI vs distance (n = %g, d_0 = %g m)', ch.n, ch.d0));
legend('Location', 'southwest');

%% 4. CFO error
figure('Name', 'CFO error');
tiledlayout(1, 2);
nexttile;
hold on; grid on;
for m = 1:nRx
    sel = L.rxIdx == m & L.detected;
    scatter(L.snrDb(sel), L.cfoErr(sel), 24, colors(m,:), 'filled', 'DisplayName', rxNames(m));
end
xlabel('SNR (dB)');
ylabel('CFO error (Hz)');
title('CFO estimate error vs SNR');
legend('Location', 'best');

nexttile;
err = L.cfoErr(L.detected);
histogram(err);
grid on;
xlabel('CFO error (Hz)');
ylabel('Count');
title(sprintf('CFO error: mean %.0f Hz, std %.0f Hz', mean(err), std(err)));
end
