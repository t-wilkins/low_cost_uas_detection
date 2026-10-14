function res = receiveRidBeacon(rxWaveform, cfgPHY, ind, fs, noiseVar, truth)
% RECEIVERIDBEACON  Full receive chain: packet detection, coarse/fine CFO
% correction, L-LTF channel estimation, PSDU recovery, MPDU decode, and
% Remote ID extraction. Pure processing - no plotting.
%
%   res = receiveRidBeacon(rxWaveform, cfgPHY, ind, fs, noiseVar, truth)
%
%   truth : struct with ground-truth fields used only for scoring:
%             .mpduBits  (for BER)
%             .uasID     (for ID match check)
%           Pass [] to skip scoring (BER/idMatch come back NaN/false).
%
%   Returns struct res with fields:
%     detected     : logical, packet found at all
%     startOffset  : sample offset returned by wlanPacketDetect
%     coarseCFO, fineCFO : Hz
%     bitErrors, numBits : BER inputs (NaN if truth not supplied)
%     decodeStatus : wlanMACDecodeStatus
%     decodeOK     : logical, FCS passed
%     ridFound     : logical, ASTM vendor IE located
%     basicID      : decoded Basic ID struct (ridDecodeBasicID output), or [] if not found
%     idMatch      : logical, decoded UAS ID matches truth.uasID (false if truth not supplied)

res = struct('detected', false, 'startOffset', [], 'coarseCFO', NaN, 'fineCFO', NaN, ...
    'bitErrors', NaN, 'numBits', NaN, 'decodeStatus', [], 'decodeOK', false, ...
    'ridFound', false, 'basicID', [], 'idMatch', false);

%% Packet detection
startOffset = wlanPacketDetect(rxWaveform, cfgPHY.ChannelBandwidth);
if isempty(startOffset)
    return;   % detected stays false, everything else NaN/empty
end
res.detected    = true;
res.startOffset = startOffset;
rx = rxWaveform(startOffset+1:end, :);

% Lock too close to the end of the buffer for a full packet: nothing to decode
if size(rx, 1) < ind.NonHTData(2)
    return;
end

%% Coarse CFO from L-STF
coarse = wlanCoarseCFOEstimate(rx(ind.LSTF(1):ind.LSTF(2), :), cfgPHY.ChannelBandwidth);
rx = frequencyOffset(rx, fs, -coarse);
res.coarseCFO = coarse;

%% Fine CFO from L-LTF
fine = wlanFineCFOEstimate(rx(ind.LLTF(1):ind.LLTF(2), :), cfgPHY.ChannelBandwidth);
rx = frequencyOffset(rx, fs, -fine);
res.fineCFO = fine;

%% Channel estimate from L-LTF
demodLLTF = wlanLLTFDemodulate(rx(ind.LLTF(1):ind.LLTF(2), :), cfgPHY);
chEst     = wlanLLTFChannelEstimate(demodLLTF, cfgPHY);

%% Recover PSDU
rxBits = wlanNonHTDataRecover(rx(ind.NonHTData(1):ind.NonHTData(2), :), chEst, noiseVar, cfgPHY);

if nargin >= 6 && ~isempty(truth) && isfield(truth, 'mpduBits')
    res.bitErrors = sum(rxBits ~= truth.mpduBits);
    res.numBits   = numel(truth.mpduBits);
end

%% MPDU decode
[rxCfgMAC, ~, status] = wlanMPDUDecode(rxBits, cfgPHY);
res.decodeStatus = status;
res.decodeOK = (status == wlanMACDecodeStatus.Success);
if ~res.decodeOK
    return;
end

%% Extract Remote ID vendor IE
ies = rxCfgMAC.ManagementConfig.InformationElements;
ridPayload = [];
for k = 1:size(ies, 1)
    if ies{k,1}(1) == 221
        octets = uint8(ies{k,2}(:)).';
        if numel(octets) >= 5 && isequal(octets(1:4), uint8([250 11 188 13]))
            ridPayload = octets;
            break;
        end
    end
end
if isempty(ridPayload)
    return;   % ridFound stays false
end
res.ridFound = true;

%% Decode Basic ID (first Basic ID message found in the pack)
msgs = ridDecodeMessagePack(ridPayload(6:end));
for k = 1:size(msgs, 1)
    m = msgs(k, :);
    if bitshift(m(1), -4) == 0   % Basic ID
        res.basicID = ridDecodeBasicID(m);
        if nargin >= 6 && ~isempty(truth) && isfield(truth, 'uasID')
            res.idMatch = strcmp(res.basicID.uasID, truth.uasID);
        end
        break;
    end
end
end
