function chnOut = applyChannel(txWaveform, txCfg, rxCfg, chCfg)
% APPLYCHANNEL  Propagation channel from one transmitter to one receive
% antenna: delay, path loss, shadowing, Doppler and thermal noise.
% Log-distance path loss, line of sight; multipath to be added.
%
%   chnOut = applyChannel(txWaveform, txCfg, rxCfg)
%   chnOut = applyChannel(txWaveform, txCfg, rxCfg, chCfg)
%
%   Inputs
%     txWaveform : baseband waveform from ridTransmitter (sqrt(W) scale,
%                  transmit power already applied), sampled at 20 MHz
%     txCfg      : same struct as ridTransmitter. Uses
%                    fc = 2.437e9        carrier frequency (Hz)
%                    pos = [100 100 10]  ENU position (m)
%                    heading = 0, speed = 0, vertSpeed = 0  (velocity)
%                    t = 0               transmit time (s)
%                    antGainDb = 0       transmit antenna gain (dBi)
%     rxCfg      : receiver struct
%                    pos                 ENU position (m), required
%                    vel = [0 0 0]       ENU velocity (m/s)
%                    nfDb = 4.5          noise figure (dB), ESP32 datasheet
%                    antGainDb = 0       receive antenna gain (dBi)
%                    visualise = 1       plot the channel output signal
%     chCfg      : channel struct (optional)
%                    n = 2               path loss exponent (2 = free space)
%                    d0 = 1              reference distance (m), free space up to d0
%                    shadowSigmaDb = 0   log-normal shadowing std (dB), drawn per
%                                        call when shadowDb is not given
%                    shadowDb            fixed shadowing loss (dB), e.g. one draw
%                                        per link held for a whole scenario;
%                                        overrides shadowSigmaDb
%
%   Fixed: fs = 20 MHz, T0 = 290 K,
%   20 us of noise before the frame and after it.
%
%   Output struct chnOut
%     waveform   : received waveform (sqrt(W) scale), signal + noise
%     fs         : sample rate (Hz)
%     noiseVar   : true noise variance per complex sample, kTBF (W)
%     snrDb      : received signal power / noise power over fs (dB)
%     dist       : transmitter-receiver distance (m)
%     delay      : propagation delay (s)
%     fD         : Doppler shift (Hz), positive when closing
%     pathGainDb : path gain incl. shadowing and antenna gains (dB)
%     shadowDb   : shadowing loss applied (dB), positive = extra loss
%     tStart     : absolute time of sample 1 (s)
%     toa        : absolute true time of arrival of the frame start (s)
%
%   When rxCfg.visualise is true, plots the channel output signal's in-phase
%   and quadrature components.
%
%   Model: log-distance path loss
%     PL(d) = 20*log10(4*pi*d0/lambda) + 10*n*log10(d/d0) + shadowDb  (dB),
%   i.e. Friis free space to d0 then exponent n; n = 2 with no shadowing is
%   exactly Friis for any d0. Fractional delay applied in the frequency domain, carrier phase exp(-j*2*pi*fc*tau),
%   Doppler as a phase ramp. Noise N = k*T0*B*F with B = fs, i.e. over the
%   whole simulated band, so the per-subcarrier SNR after OFDM demodulation
%   is correct. The delay change within one frame is neglected (~6 mm at
%   20 m/s over 300 us).

arguments
    txWaveform (:,1) double {mustBeNonempty}
    txCfg      (1,1) struct
    rxCfg      (1,1) struct {mustHavePos}
    chCfg      (1,1) struct = struct()
end

%% Constants
fs   = 20e6;                            % CBW20
c    = physconst('LightSpeed');
kB   = physconst('Boltzmann');
T0   = 290;
tPad = 20e-6;                           % noise before and after the frame

[txCfg, rxCfg, chCfg] = applyDefaults(txCfg, rxCfg, chCfg);
fc = txCfg.fc;                          % carrier, defined by the transmitter

%% Geometry
pTx = txCfg.pos(:);
vTx = [txCfg.speed * sind(txCfg.heading);
       txCfg.speed * cosd(txCfg.heading);
       txCfg.vertSpeed];
pRx = rxCfg.pos(:);
vRx = rxCfg.vel(:);

r      = pRx - pTx;
dist   = norm(r);
u      = r / dist;                      % unit vector tx -> rx
tau    = dist / c;
lambda = c / fc;
fD     = dot(vTx - vRx, u) / lambda;    % positive when closing

%% Path gain (log-distance + shadowing)
if isfield(chCfg, 'shadowDb')
    shadowDb = chCfg.shadowDb;
elseif chCfg.shadowSigmaDb > 0
    shadowDb = chCfg.shadowSigmaDb * randn;
else
    shadowDb = 0;                       % no draw, keeps the noise sequence unchanged
end
plDb    = 20*log10(4*pi*chCfg.d0/lambda) + 10*chCfg.n*log10(dist/chCfg.d0) + shadowDb;
gainAmp = 10^((txCfg.antGainDb + rxCfg.antGainDb - plDb)/20);

%% Window: [pre-roll | delay | frame | tail], sample nPad+1 = transmit time
N    = numel(txWaveform);
nPad = round(tPad * fs);
L    = nPad + ceil(tau * fs) + N + nPad;

x = zeros(L, 1);
x(nPad+1 : nPad+N) = txWaveform;

% Fractional delay in the frequency domain (zero-padded, no wrap-around)
Nfft = 2^nextpow2(L);
k    = (0:Nfft-1).';
f    = (k - Nfft*(k >= Nfft/2)) * fs / Nfft;
y    = ifft(fft(x, Nfft) .* exp(-1j*2*pi*f*tau));
y    = y(1:L);

% Path gain, carrier phase, Doppler
t = (0:L-1).' / fs;
y = y * gainAmp * exp(-1j*2*pi*fc*tau) .* exp(1j*2*pi*fD*t);

%% Thermal noise, N = kTBF with B = fs
noiseVar = kB * T0 * fs * 10^(rxCfg.nfDb/10);
noise    = sqrt(noiseVar/2) * (randn(L, 1) + 1j*randn(L, 1));

%% Output
chnOut.waveform   = y + noise;
chnOut.fs         = fs;
chnOut.noiseVar   = noiseVar;
chnOut.snrDb      = 10*log10(mean(abs(txWaveform).^2) * gainAmp^2 / noiseVar);
chnOut.dist       = dist;
chnOut.delay      = tau;
chnOut.fD         = fD;
chnOut.pathGainDb = 20*log10(gainAmp);
chnOut.shadowDb   = shadowDb;
chnOut.tStart     = txCfg.t - tPad;
chnOut.toa        = txCfg.t + tau;

if rxCfg.visualise
    tUs = (0:L-1).' / fs * 1e6;
    figure('Name', 'Channel output signal');
    plot(tUs, real(chnOut.waveform), tUs, imag(chnOut.waveform));
    grid on;
    xlabel('Time (\mus)');
    ylabel('Amplitude (sqrt(W))');
    title('Channel output signal (signal + noise)');
    legend('In-phase', 'Quadrature');
end
end

% -------------------------------------------------------------------------

function [txCfg, rxCfg, chCfg] = applyDefaults(txCfg, rxCfg, chCfg)
    txDef = struct('fc', 2.437e9, 'pos', [100 100 10], 'heading', 0, 'speed', 0, ...
                   'vertSpeed', 0, 't', 0, 'antGainDb', 0, 'visualise', 1);
    rxDef = struct('vel', [0 0 0], 'nfDb', 4.5, 'antGainDb', 0, 'visualise', 1);
    chDef = struct('n', 2, 'd0', 1, 'shadowSigmaDb', 0);   % shadowDb has no default
    f = fieldnames(chDef);
    for i = 1:numel(f)
        if ~isfield(chCfg, f{i}), chCfg.(f{i}) = chDef.(f{i}); end
    end
    f = fieldnames(txDef);
    for i = 1:numel(f)
        if ~isfield(txCfg, f{i}), txCfg.(f{i}) = txDef.(f{i}); end
    end
    f = fieldnames(rxDef);
    for i = 1:numel(f)
        if ~isfield(rxCfg, f{i}), rxCfg.(f{i}) = rxDef.(f{i}); end
    end
end

function mustHavePos(rxCfg)
    if ~isfield(rxCfg, 'pos') || numel(rxCfg.pos) ~= 3
        error('applyChannel:rxCfg', 'rxCfg.pos must be a 3-element ENU position (m).');
    end
end