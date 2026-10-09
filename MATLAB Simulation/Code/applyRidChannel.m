function rxOut = applyRidChannel(txWaveform, fs, opts)
% APPLYRIDCHANNEL  Push a transmitted waveform through a TGn multipath
% channel with path loss, thermal noise, a CFO, and a random leading
% silence (unknown packet start, as a real receiver would see).
%
%   rxOut = applyRidChannel(txWaveform, fs, opts)
%
%   opts fields (all optional, defaults shown):
%     distance        = 5          metres, transmitter-receiver distance
%     delayProfile    = 'Model-D'  TGn delay profile (A-F)
%     carrierFreq     = 2.4e9      Hz
%     noiseFigureDB   = 4.5        receiver noise figure (dB)
%     cfoHz           = 500        residual carrier frequency offset
%     maxSilenceSamp  = 500        max random leading silence (samples); 0 to disable
%     seed            = []         RNG seed for repeatable sweeps; [] = don't touch RNG
%
%   Returns struct rxOut with fields:
%     waveform  : received waveform after channel + noise + CFO
%     noiseVar  : noise variance actually injected (ground truth for recovery/LLR)
%     silence   : number of leading silence samples actually inserted
%     tgnChan   : the channel object used (so CIR can be queried, e.g. tgnChan(impulse))

opts = applyDefaults(opts);

if ~isempty(opts.seed)
    rng(opts.seed);
end

%% Leading silence - unknown packet start time
if opts.maxSilenceSamp > 0
    silence = randi(opts.maxSilenceSamp);
else
    silence = 0;
end
txPadded = [zeros(silence, size(txWaveform, 2)); txWaveform];

%% Multipath + path loss
tgnChan = wlanTGnChannel( ...
    'SampleRate',              fs, ...
    'DelayProfile',            opts.delayProfile, ...
    'CarrierFrequency',        opts.carrierFreq, ...
    'NumTransmitAntennas',     1, ...
    'NumReceiveAntennas',      1, ...
    'LargeScaleFadingEffect',  'Pathloss and shadowing', ...
    'TransmitReceiveDistance', opts.distance);

rx = tgnChan(txPadded);

%% Thermal noise: variance = kTBF
k = -228.6;                          % dBW/(Hz*K), Boltzmann's constant
noiseVar = 10^((k + 10*log10(290) + 10*log10(fs) + opts.noiseFigureDB)/10);
rxNoise = comm.AWGNChannel('NoiseMethod', 'Variance', 'Variance', noiseVar);
rx = rxNoise(rx);

%% CFO
rx = frequencyOffset(rx, fs, opts.cfoHz);

%% Package output
rxOut.waveform = rx;
rxOut.noiseVar = noiseVar;
rxOut.silence  = silence;
rxOut.tgnChan  = tgnChan;
end

function opts = applyDefaults(opts)
if nargin < 1 || isempty(opts), opts = struct(); end
defaults = struct( ...
    'distance',       5, ...
    'delayProfile',   'Model-D', ...
    'carrierFreq',    2.4e9, ...
    'noiseFigureDB',  4.5, ...
    'cfoHz',          500, ...
    'maxSilenceSamp', 500, ...
    'seed',           []);
fn = fieldnames(defaults);
for i = 1:numel(fn)
    if ~isfield(opts, fn{i})
        opts.(fn{i}) = defaults.(fn{i});
    end
end
end
