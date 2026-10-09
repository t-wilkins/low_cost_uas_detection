%% runRidSweep.m
% Sensitivity sweep over distance and CFO for the ASTM F3411 Remote ID
% over WiFi beacon link. Builds the waveform once, then for each
% (distance, CFO) grid point runs nTrials independent channel/noise
% realizations through applyRidChannel + receiveRidBeacon and records
% packet detection rate, FCS decode rate, ID match rate, and mean BER.

clear; close all;

%% Sweep grid - edit these
distanceGrid = 100:50:800;        % metres
cfoGrid      = [0 250 500 1000 2000];    % Hz
nTrials      = 200;                       % independent noise/channel draws per grid point

%% Fixed transmit configuration (built once - payload doesn't depend on channel)
tx = buildRidBeaconWaveform(struct('uasID', 'N.123456'));
truth = struct('mpduBits', tx.mpduBits, 'uasID', tx.uasID);

%% Preallocate result grids
nD = numel(distanceGrid);
nC = numel(cfoGrid);
detectRate = zeros(nD, nC);
decodeRate = zeros(nD, nC);
idMatchRate = zeros(nD, nC);
meanBER     = zeros(nD, nC);

%% Sweep
for di = 1:nD
    for ci = 1:nC
        nDetected = 0; nDecoded = 0; nIDMatch = 0; berAccum = 0; berCount = 0;

        for t = 1:nTrials
            chanOpts = struct( ...
                'distance',     distanceGrid(di), ...
                'cfoHz',        cfoGrid(ci), ...
                'delayProfile', 'Model-D', ...
                'maxSilenceSamp', 500);

            rxOut = applyRidChannel(tx.waveform, tx.fs, chanOpts);
            res = receiveRidBeacon(rxOut.waveform, tx.cfgPHY, tx.ind, tx.fs, rxOut.noiseVar, truth);

            nDetected = nDetected + res.detected;
            nDecoded  = nDecoded  + res.decodeOK;
            nIDMatch  = nIDMatch  + res.idMatch;
            if ~isnan(res.bitErrors)
                berAccum = berAccum + res.bitErrors / res.numBits;
                berCount = berCount + 1;
            end
        end

        detectRate(di, ci)  = nDetected / nTrials;
        decodeRate(di, ci)  = nDecoded  / nTrials;
        idMatchRate(di, ci) = nIDMatch  / nTrials;
        meanBER(di, ci)     = berAccum / max(berCount, 1);
    end
    fprintf('Distance %g m done.\n', distanceGrid(di));
end

%% Plot results
figure('Name', 'RID Beacon Sensitivity Sweep');
plot(distanceGrid, detectRate(:,1));
grid on;
hold on;
plot(distanceGrid, decodeRate(:,1));
plot(distanceGrid, idMatchRate(:,1));
xlabel("Distance (m)")
ylabel("Success Rate")
legend("Successful Preamble Detection", ...
    "Successful PSDU Decoding", ...
    "Successful RID Match")

figure('Name', 'RID Beacon Sensitivity Sweep');

subplot(2,2,1);
imagesc(cfoGrid, distanceGrid, detectRate); set(gca,'YDir','normal');
colorbar; xlabel('CFO (Hz)'); ylabel('Distance (m)'); title('Packet Detection Rate');

subplot(2,2,2);
imagesc(cfoGrid, distanceGrid, decodeRate); set(gca,'YDir','normal');
colorbar; xlabel('CFO (Hz)'); ylabel('Distance (m)'); title('FCS Decode Success Rate');

subplot(2,2,3);
imagesc(cfoGrid, distanceGrid, idMatchRate); set(gca,'YDir','normal');
colorbar; xlabel('CFO (Hz)'); ylabel('Distance (m)'); title('Correct UAS ID Recovery Rate');

subplot(2,2,4);
imagesc(cfoGrid, distanceGrid, log10(meanBER + eps)); set(gca,'YDir','normal');
colorbar; xlabel('CFO (Hz)'); ylabel('Distance (m)'); title('log_{10}(Mean BER)');

sgtitle(sprintf('RID Beacon Link Sensitivity (%d trials/point)', nTrials));
