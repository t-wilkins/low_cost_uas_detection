%% ridBeaconLink.m
clear; close all; rng(2026);

visualise = true;

%% ------------------------------------------------------------------
%  Build the Remote ID payload
%  ------------------------------------------------------------------
%   Vendor Specific IE (element ID 221 / 0xDD)
%     +-- OUI            : FA 0B BC        (ASTM)
%     +-- Vendor type    : 0D              (Remote ID)
%     +-- Message counter: 1 byte
%     +-- Message Pack   : [0xF | version | msg_size | N | msg_1 ... msg_N]
%
%   Basic ID message (25 bytes):
%     byte 1     : MessageType=0x0 | ProtocolVersion
%     byte 2     : IDType | UAType
%     bytes 3-22 : UAS ID, ASCII, null-padded to 20 bytes
%     bytes 23-25: reserved (0)

messageType = 0;                 % 0 = Basic ID
protoVer    = 2;                 % 2 = F3411-22a
idType      = 1;                 % 1 = Serial Number 
uaType      = 2;                 % 2 = Helicopter / Multirotor
uasID       = 'N.123456';        % <=20 chars (Example from standard)

basicID = ridEncodeBasicID(messageType, idType, uaType, uasID, protoVer);
pack    = ridEncodeMessagePack(basicID, protoVer);   % one message in the pack

msgCounter = 0;
ieData = [uint8([250 11 188 13]), uint8(msgCounter), pack];   % Vendor-Specific

fprintf('Basic ID message (%d bytes): %s\n', numel(basicID), ridHex(basicID));
fprintf('Vendor IE payload (%d bytes): %s\n\n', numel(ieData), ridHex(ieData));

%% ------------------------------------------------------------------
%  Build the WiFi beacon MAC frame with the vendor-specific IE attached
%  ------------------------------------------------------------------
beaconInterval = 100; % TUs (1024us) between packets, used in MathWorks examples
frameBody = wlanMACManagementConfig; % default beacon body
frameBody.SSID = 'DroneIDTest';
frameBody.BeaconInterval = beaconInterval; % TUs
ieHexData = ridHex(ieData); ieHexData = ieHexData(~isspace(ieHexData));
frameBody = addIE(frameBody, 221, ieHexData); % 221 = vendor specific

cfgMAC = wlanMACFrameConfig( ...
    'FrameType',        'Beacon', ...
    'ManagementConfig', frameBody);

[mpduBits, mpduLen] = wlanMACFrame(cfgMAC, 'OutputFormat', 'bits');
fprintf('Beacon MPDU length: %d bytes\n', mpduLen);

%% ------------------------------------------------------------------
%  PHY: non-HT (802.11a/g) OFDM waveform
%  ------------------------------------------------------------------
cfgPHY = wlanNonHTConfig( ...
    'ChannelBandwidth', 'CBW20', ...
    'MCS',              0, ...       % BPSK 1/2 -> 6 Mbps, most robust
    'PSDULength',       mpduLen);

fs = wlanSampleRate(cfgPHY);

% Appending 50us of silence time to each packet
txWaveform = wlanWaveformGenerator(mpduBits, cfgPHY, 'IdleTime', 50e-6);

ind = wlanFieldIndices(cfgPHY);

% Plot transmitted WiFi Beacon packet in time, highlighting different 
% frame components
if visualise
    t = (0:numel(txWaveform)-1);
    figure;
    plot(t, real(txWaveform), 'b');
    hold on
    plot(t, imag(txWaveform), 'r');
    fill([ind.LSTF(2), ind.LSTF(1), ind.LSTF(1), ind.LSTF(2)], ...
         [-max(abs(txWaveform(txWaveform<0))) -max(abs(txWaveform(txWaveform<0))) max(txWaveform) max(txWaveform)], ...
         [131, 50, 168]/255, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    fill([ind.LLTF(2), ind.LLTF(1), ind.LLTF(1), ind.LLTF(2)], ...
         [-max(abs(txWaveform(txWaveform<0))) -max(abs(txWaveform(txWaveform<0))) max(txWaveform) max(txWaveform)], ...
         [94, 204, 193]/255, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    fill([ind.LSIG(2), ind.LSIG(1), ind.LSIG(1), ind.LSIG(2)], ...
         [-max(abs(txWaveform(txWaveform<0))) -max(abs(txWaveform(txWaveform<0)))  max(txWaveform) max(txWaveform)], ...
         [230, 133, 37]/255, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    fill([ind.NonHTData(2), ind.NonHTData(1), ind.NonHTData(1), ind.NonHTData(2)], ...
         [-max(abs(txWaveform(txWaveform<0))) -max(abs(txWaveform(txWaveform<0))) max(txWaveform) max(txWaveform)], ...
         [60, 199, 69]/255, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    text(ind.LSTF(1)+1, max(txWaveform)*0.9, 'L-STF', 'FontSize', 12, 'FontWeight', 'bold');
    text(ind.LLTF(1)+1, max(txWaveform)*0.9, 'L-LTF', 'FontSize', 12, 'FontWeight', 'bold');
    text(ind.LSIG(1)+1, max(txWaveform)*0.9, 'L-SIG', 'FontSize', 12, 'FontWeight', 'bold');
    text(ind.NonHTData(1)+75, max(txWaveform)*0.9, 'NonHTData (RID Packet)', 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Samples'); ylabel('Amplitude');
    title('20 MHz Non-HT Beacon Waveform');
    legend('Real', 'Imag')
    grid on;
end

%% Sensitivity Sweep
maxD = 200;
stepD = 10;
nD = maxD/stepD;
distance = 1:stepD:maxD;
numPackets = 20;
meanBER = zeros(nD,1);

for di = 1:nD % meters
    bitErrors = 0;
    for n = 1:numPackets % Number of independent noise/channel draws
        %------------------------------------------------------------------
        %  Channel: CFO + AWGN + Multipath + Doppler
        %  ------------------------------------------------------------------
        % Add random silence time to signal
        silence = randi(500);
        % Insert a random idle interval before the transmitted packet
        tx = [zeros(silence, size(txWaveform, 2)); txWaveform];
        
        if visualise
            figure
            spectrogram(tx, 50, "yaxis")
            title("Tx Signal Spectrogram")
        end
        
        % Add multipath channel with pathloss
        dist = distance(di);
        tgnChan = wlanTGnChannel( ...
            'SampleRate',              fs, ...
            'DelayProfile',            'Model-D', ...   % office
            'CarrierFrequency',        2.4e9, ...
            'NumTransmitAntennas',     1, ...
            'NumReceiveAntennas',      1, ...
            'LargeScaleFadingEffect',  'Pathloss and shadowing', ...
            'TransmitReceiveDistance', dist);  
        
        rxWaveform = tgnChan(tx);
        
        % Plot channel impulse response
        if visualise
            impulse = zeros(100, 1);
            impulse(1) = 1;
            CIR = tgnChan(impulse);
            figure
            stem(real(CIR))
            title('IEEE 802.11n Multipath Fading Channel Impulse Response')
            xlabel('Taps'); ylabel('Amplitude')
        end
        
        % Add receiver noise with variance = kTBF, k = Boltzmann's Constant, T =
        % Temperature, B = Bandwidth and F = Receiver Noise Figure
        k = -228.6; % dBW/(Hz*K)
        T = 290; % K
        B = fs; % Hz
        F = 4.5; % dB - Noise Figure of ESP32
        noiseVar = 10^((k + 10*log10(T) + 10*log10(B) + F)/10);
        rxNoise = comm.AWGNChannel('NoiseMethod','Variance','Variance',noiseVar);
        rxWaveform = rxNoise(rxWaveform);
        
        % Add CFO 
        cfoHz = 500;
        rxWaveform = frequencyOffset(rxWaveform, fs, cfoHz);
        
        % Add spectrum analyser plot with before- and after- channel waveforms
        if visualise
            saScope = spectrumAnalyzer(SampleRate=fs,ShowLegend=true,...
                AveragingMethod='exponential',ForgettingFactor=0.99, ...
                Title='20 MHz Non-HT Waveform Before and After 802.11n Channel',...
                ChannelNames={'Before','After'});
            saScope([tx,rxWaveform]);
        
            figure
            spectrogram(rxWaveform, 50, "yaxis");
            title("Rx Signal Spectrogram");
        end
        % ------------------------------------------------------------------
        %  Receiver: sync, channel estimate, data recovery
        %  ------------------------------------------------------------------
        
        [startOffset,M] = wlanPacketDetect(rxWaveform, cfgPHY.ChannelBandwidth);
        
        if visualise
            figure;
            plot(M); 
            hold on
            rxNormalised = rxWaveform./max(rxWaveform);
            plot(real(rxNormalised))
            xline(startOffset, '--k')
            yline(0.5, '--k')
            title("STF Decision Statistics vs Normalised Waveform");
            legend("Decision Statistics", "Normalised Waveform", ...
                "Decision Threshold", "Detected Preamble Start Offset")
        end 
        
        if isempty(startOffset)
            error('Packet not detected.');
        end
        rx = rxWaveform(startOffset+1:end, :);
        
        % Coarse CFO from L-STF
        coarse = wlanCoarseCFOEstimate(rx(ind.LSTF(1):ind.LSTF(2), :), cfgPHY.ChannelBandwidth);
        rx = frequencyOffset(rx, fs, -coarse);
        
        % Fine CFO from L-LTF
        fine = wlanFineCFOEstimate(rx(ind.LLTF(1):ind.LLTF(2), :), cfgPHY.ChannelBandwidth);
        rx = frequencyOffset(rx, fs, -fine);
        
        % Channel estimate from L-LTF
        demodLLTF = wlanLLTFDemodulate(rx(ind.LLTF(1):ind.LLTF(2), :), cfgPHY);
        chEst     = wlanLLTFChannelEstimate(demodLLTF, cfgPHY);
        
        if visualise
            scatterplot(chEst);
        end
        
        % Recover the PSDU
        rxBits = wlanNonHTDataRecover(rx(ind.NonHTData(1):ind.NonHTData(2), :), ...
            chEst, noiseVar, cfgPHY);
        
        fprintf('Bit errors in PSDU: %d of %d\n', sum(rxBits ~= mpduBits), numel(mpduBits));
        
        % ------------------------------------------------------------------
        %  Decode the MPDU and pull the Remote ID back out
        %  ------------------------------------------------------------------
        [rxCfgMAC, ~, status] = wlanMPDUDecode(rxBits, cfgPHY);
        fprintf('MPDU decode status: %s\n', string(status));
        
        if status ~= wlanMACDecodeStatus.Success
            error('FCS check failed - increase SNR.');
        end
        
        ies = rxCfgMAC.ManagementConfig.InformationElements;
        ridPayload = [];
        for k = 1:size(ies, 1)
            if ies{k,1}(1) == 221 % Vendor-specific IE detected
                octets = uint8(ies{k,2}(:)).'; % Extract IE
                % Check if OUI corresponds to ASTM RID packet
                if numel(octets) >= 5 && isequal(octets(1:4), uint8([250 11 188 13]))
                    ridPayload = octets; % Extract payload
                    break;
                end
            end
        end
        
        if isempty(ridPayload)
            error('No Remote ID vendor IE found in received beacon.');
        end
        
        msgs = ridDecodeMessagePack(ridPayload(6:end));
        fprintf('\nRecovered %d Remote ID message(s).\n', size(msgs, 1));
        
        for k = 1:size(msgs, 1)
            m = msgs(k, :);
            msgType = bitshift(m(1), -4);
            if msgType == 0
                d = ridDecodeBasicID(m);
                fprintf('  Basic ID  | protoVer %d | IDType %d | UAType %d | UAS ID "%s"\n', ...
                    d.protocolVersion, d.idType, d.uaType, d.uasID);
                fprintf('  Match with transmitted ID: %d\n', strcmp(d.uasID, uasID));
            else
                fprintf('  Message type %d (not decoded here)\n', msgType);
            end
        end
        visualise = false;
        bitErrors = bitErrors + sum(rxBits ~= mpduBits);
    end
    meanBER(di) = bitErrors/numPackets;
end

% Plot BER vs Distance Curve
figure;
plot(distance, meanBER)
title("BER vs Distance")
xlabel("Distance (m)")
ylabel("BER")

%% ==================================================================
%  Remote ID encode/decode helpers
%  ==================================================================
function msg = ridEncodeBasicID(messageType, idType, uaType, uasID, protoVer)
    % 25-byte Basic ID message, ASTM F3411. Packed as column vector
    msg = zeros(1, 25, 'uint8');
    % Header
    % Message type (4 bits) + Protocol Version (4 bits)
    msg(1) = bitor(bitshift(messageType, 4), uint8(protoVer)); 
    % Message
    % ID Type (4 bits) + UA Type (4 bits)
    msg(2) = bitor(bitshift(uint8(idType), 4), uint8(uaType));
    % UAS ID (20 bytes)
    b = uint8(uasID);
    n = min(numel(b), 20);
    msg(3:2+n) = b(1:n); % null padded
    % msg(23:25) reserved, already zero
end

function pack = ridEncodeMessagePack(msgs, protoVer)
    % Returns the encoded Message Pack as a row vector.
    n = size(msgs, 1);
    % Message Type (MsgPck = 0xF) + Protocol Version + Message Size (25
    % bytes) + Number of Messages (n)
    hdr = [bitor(bitshift(uint8(15), 4), uint8(protoVer)), uint8(25), uint8(n)];
    % Pack as row vector
    pack = [hdr, reshape(msgs.', 1, [])];
end

function msgs = ridDecodeMessagePack(pack)
% Returns the decoded messages from the pack
    pack = uint8(pack(:)).';
    if bitshift(pack(1), -4) ~= 15 % Check Message Type 
        error('Not a Message Pack (type 0x%X).', bitshift(pack(1), -4));
    end
    % Create matrix of transmitted messages
    singleSize = double(pack(2));
    n          = double(pack(3));
    body       = pack(4 : 3 + singleSize*n);
    msgs       = reshape(body, singleSize, n).';
end

function d = ridDecodeBasicID(msg)
    msg = uint8(msg(:)).';
    d.protocolVersion = double(bitand(msg(1), 15));
    d.idType          = double(bitshift(msg(2), -4));
    d.uaType          = double(bitand(msg(2), 15));
    raw               = char(msg(3:22));
    d.uasID           = raw(raw ~= char(0));
end

function s = ridHex(b)
    s = strtrim(sprintf('%02X ', uint8(b)));
end