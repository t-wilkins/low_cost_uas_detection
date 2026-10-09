%% ridBeaconDemo.m
% Single end-to-end run of the ASTM F3411 Remote ID over WiFi beacon link,
% with the full set of diagnostic plots. Uses the same modular functions
% as runRidSweep.m so the demo and the sweep can never drift apart.

clear; close all; rng(2026);

%% Build transmit waveform
tx = buildRidBeaconWaveform(struct('uasID', 'N.123456'));
fprintf('Basic ID message (%d bytes): %s\n', numel(tx.basicIDMsg), ridHex(tx.basicIDMsg, true));
fprintf('Vendor IE payload (%d bytes): %s\n\n', numel(tx.ieData), ridHex(tx.ieData, true));
fprintf('Beacon MPDU length: %d bytes\n', tx.mpduLen);

plotBeaconWaveform(tx.waveform, tx.ind);

%% Channel
chanOpts = struct('distance', 5, 'delayProfile', 'Model-D', 'cfoHz', 500, 'maxSilenceSamp', 500);
rxOut = applyRidChannel(tx.waveform, tx.fs, chanOpts);

figure; spectrogram(tx.waveform, 50, "yaxis"); title("Tx Signal Spectrogram");
figure; spectrogram(rxOut.waveform, 50, "yaxis"); title("Rx Signal Spectrogram");

impulse = zeros(100, 1); impulse(1) = 1;
CIR = rxOut.tgnChan(impulse);
figure; stem(real(CIR));
title('IEEE 802.11n Multipath Fading Channel Impulse Response');
xlabel('Taps'); ylabel('Amplitude');

saScope = spectrumAnalyzer(SampleRate=tx.fs, ShowLegend=true, ...
    AveragingMethod='exponential', ForgettingFactor=0.99, ...
    Title='20 MHz Non-HT Waveform Before and After 802.11n Channel', ...
    ChannelNames={'Before','After'});
saScope([tx.waveform; zeros(numel(rxOut.waveform)-numel(tx.waveform),1)], rxOut.waveform);

%% Receive
truth = struct('mpduBits', tx.mpduBits, 'uasID', tx.uasID);
res = receiveRidBeacon(rxOut.waveform, tx.cfgPHY, tx.ind, tx.fs, rxOut.noiseVar, truth);

plotPacketDetect(rxOut.waveform, tx.cfgPHY, res.startOffset);

fprintf('\nBit errors in PSDU: %d of %d\n', res.bitErrors, res.numBits);
fprintf('MPDU decode status: %s\n', string(res.decodeStatus));

if res.ridFound && ~isempty(res.basicID)
    d = res.basicID;
    fprintf('\nRecovered Basic ID | protoVer %d | IDType %d | UAType %d | UAS ID "%s"\n', ...
        d.protocolVersion, d.idType, d.uaType, d.uasID);
    fprintf('Match with transmitted ID: %d\n', res.idMatch);
else
    fprintf('\nRemote ID not recovered (detected=%d, decodeOK=%d, ridFound=%d)\n', ...
        res.detected, res.decodeOK, res.ridFound);
end


%% ==================================================================
%  Local plotting helpers (kept out of the core pipeline functions)
%  ==================================================================
function plotBeaconWaveform(waveform, ind)
t = 0:numel(waveform)-1;
figure;
plot(t, real(waveform), 'b'); hold on;
plot(t, imag(waveform), 'r');
lo = -max(abs(waveform(real(waveform)<0))); hi = max(real(waveform));
fields = {ind.LSTF, ind.LLTF, ind.LSIG, ind.NonHTData};
colors = {[131,50,168]/255, [94,204,193]/255, [230,133,37]/255, [60,199,69]/255};
labels = {'L-STF','L-LTF','L-SIG','NonHTData (RID Packet)'};
for i = 1:numel(fields)
    f = fields{i};
    fill([f(2) f(1) f(1) f(2)], [lo lo hi hi], colors{i}, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    text(f(1)+1, hi*0.9, labels{i}, 'FontSize', 12, 'FontWeight', 'bold');
end
xlabel('Samples'); ylabel('Amplitude'); title('20 MHz Non-HT Beacon Waveform');
legend('Real','Imag'); grid on;
end

function plotPacketDetect(rxWaveform, cfgPHY, startOffset)
[~, M] = wlanPacketDetect(rxWaveform, cfgPHY.ChannelBandwidth);
figure; plot(M); hold on;
plot(real(rxWaveform ./ max(rxWaveform)));
if ~isempty(startOffset)
    xline(startOffset, '--k');
end
yline(0.5, '--k');
title('STF Decision Statistics vs Normalised Waveform');
legend('Decision Statistics', 'Normalised Waveform', 'Detected Start Offset', 'Decision Threshold');
end
