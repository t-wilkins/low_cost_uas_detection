function result = ridDemo(seed)
% RIDDEMO  Single Remote ID beacon: transmit, line-of-sight channel, receive.
%   result = ridDemo(seed)
%
%   Builds one beacon with ridTransmitter, passes it through applyChannel to
%   one receive antenna, recovers it with ridReceiver, and prints the
%   estimates against the ground truth. seed (default 1) sets the noise.
%
%   result fields: txCfg, rxCfg, tx, chnOut, rxOut, and
%     trueStartOffset : true packet start as a 0-based sample offset
%     trueCFO         : txCfg.cfo + Doppler - rxCfg.cfo (Hz)
%     posErr          : decoded minus true ENU position (m)

if nargin < 1, seed = 1; end
rng(seed);

%% Scenario
fc = 2.437e9;
visualise = true;

txCfg            = struct();
txCfg.uasID      = 'A.00001';
txCfg.pos        = [4000 1000 30];        % ENU (m), 30 m above ground
txCfg.heading    = 225;                 % flying towards the receiver (SW)
txCfg.speed      = 10;                  % m/s
txCfg.vertSpeed  = 0;
txCfg.t          = 0;
txCfg.msgCounter = 0;                   % pack A
txCfg.fc         = fc;
txCfg.cfo        = 5e-6 * fc;           % +5 ppm transmitter crystal error
txCfg.srcMAC     = '0A0000000001';
txCfg.visualise  = visualise;

rxCfg            = struct();
rxCfg.pos        = [0 0 2];             % receiver 2 m above ground at the origin
rxCfg.nfDb       = 4.5;
rxCfg.cfo        = -3e-6 * fc;          % -3 ppm receiver crystal error
rxCfg.visualise  = visualise;

%% Transmit, channel, receive
tx     = ridTransmitter(txCfg);
chnOut = applyChannel(tx.waveform, txCfg, rxCfg);
rxOut  = ridReceiver(chnOut.waveform, rxCfg);

%% Ground truth
trueStartOffset = (chnOut.toa - chnOut.tStart) * chnOut.fs;
trueCFO         = txCfg.cfo + chnOut.fD - rxCfg.cfo;

%% Report
fprintf('\n--- Link ---\n');
fprintf('Distance        %8.2f m\n',   chnOut.dist);
fprintf('Delay           %8.2f ns\n',  chnOut.delay*1e9);
fprintf('Path gain       %8.2f dB\n',  chnOut.pathGainDb);
fprintf('SNR             %8.2f dB\n',  chnOut.snrDb);
fprintf('Doppler         %8.2f Hz\n',  chnOut.fD);

fprintf('\n--- Receiver ---\n');
fprintf('Detected        %d\n', rxOut.detected);
if rxOut.detected
    fprintf('RSSI            %8.2f dBm      (signal %.2f dBm, noise %.2f dBm)\n', rxOut.rssiDbm, ...
        10*log10(chnOut.noiseVar) + 30 + chnOut.snrDb, 10*log10(chnOut.noiseVar) + 30);
    fprintf('Start offset    %8d samples   (true %.2f)\n', rxOut.startOffset, trueStartOffset);
    fprintf('CFO             %8.1f Hz        (true %.1f)\n', rxOut.cfoEst, trueCFO);
end
fprintf('Decoded         %d\n', rxOut.decoded);

posErr = nan(3,1);
if rxOut.decoded
    p = rxOut.payload;
    posErr = p.pos - txCfg.pos(:);
    fprintf('\n--- Payload ---\n');
    fprintf('Messages        %s\n', mat2str(p.msgTypes));
    fprintf('UAS ID          %s   (sent %s)\n', p.uasID, txCfg.uasID);
    fprintf('Source MAC      %s\n', p.srcMAC);
    fprintf('Counter         %d\n', p.msgCounter);
    fprintf('Position        [%.2f %.2f %.2f] m   (true %s)\n', p.pos, mat2str(txCfg.pos));
    fprintf('Position error  [%.3f %.3f %.3f] m\n', posErr);
    fprintf('Heading         %g deg   Speed %g m/s   Vert %g m/s\n', ...
        p.heading, p.speed, p.vertSpeed);
end
fprintf('\n');

%% Output
result = struct('txCfg', txCfg, 'rxCfg', rxCfg, 'tx', tx, 'chnOut', chnOut, ...
    'rxOut', rxOut, 'trueStartOffset', trueStartOffset, ...
    'trueCFO', trueCFO, 'posErr', posErr);
end