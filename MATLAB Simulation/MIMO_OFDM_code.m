function MIMO_OFDM_MMSE_Only
clc; clear; close all;

% =========================================================
% All-in-one 2x2 MIMO-OFDM simulation for MATLAB R2016a
% MMSE equalizer only
% No external helper files
% No phased.* objects
% Includes:
%   - QPSK and 16-QAM
%   - Frequency-selective Rayleigh channel
%   - Perfect channel knowledge
%   - MMSE equalization only
%   - BER, constellation, channel response, PSD, MSE plots
% =========================================================

rng(2025);

%% Parameters
nTx = 2;
nRx = 2;
Nfft = 64;
Ncp = 16;
L = 4;                      % Number of channel taps
NofdmSym = 100;             % Number of OFDM symbols
SNRdB = 0:2:20;
numSNR = length(SNRdB);

modOrderList = [4 16];      % QPSK, 16-QAM
numMods = length(modOrderList);

% BER(modulation, snr)
BER = zeros(numMods, numSNR);

% Storage for plots
detectedConst = cell(numMods, 1);
mseStore = cell(numMods, 1);
Hstore = [];
psdSignal = [];

%% Main simulation
for mIdx = 1:numMods
    M = modOrderList(mIdx);
    bitsPerSymbol = log2(M);

    fprintf('Starting modulation M = %d\n', M);

    for sIdx = 1:numSNR
        snrDb = SNRdB(sIdx);
        snrLin = 10^(snrDb/10);
        noiseVar = 1/snrLin;

        errMMSE = 0;
        totalBits = 0;

        mseMMSE = zeros(Nfft,1);
        mseCount = zeros(Nfft,1);

        constMMSE = [];

        for blk = 1:NofdmSym

            %% Generate random bits for each TX antenna
            bits_tx = randi([0 1], nTx, Nfft * bitsPerSymbol);

            %% Map bits to symbols
            S_tx = zeros(nTx, Nfft);
            for tx = 1:nTx
                S_tx(tx,:) = mapSymbols(bits_tx(tx,:), M);
            end

            %% OFDM modulation (for PSD display only)
            txTime = zeros(nTx, Nfft + Ncp);
            for tx = 1:nTx
                x_ifft = ifft(S_tx(tx,:), Nfft);
                txTime(tx,:) = [x_ifft(end-Ncp+1:end) x_ifft];
            end

            if isempty(psdSignal) && mIdx == 1 && sIdx == numSNR
                psdSignal = txTime(1,:).';
            end

            %% Frequency-selective MIMO channel
            H_taps = zeros(nRx, nTx, L);
            for l = 1:L
                H_taps(:,:,l) = (randn(nRx,nTx) + 1j*randn(nRx,nTx)) / sqrt(2*L);
            end

            % Frequency response on each subcarrier
            H_freq = zeros(nRx, nTx, Nfft);
            for rx = 1:nRx
                for tx = 1:nTx
                    h = squeeze(H_taps(rx,tx,:));
                    h_padded = [h; zeros(Nfft-L,1)];
                    H_freq(rx,tx,:) = fft(h_padded, Nfft);
                end
            end

            if isempty(Hstore) && mIdx == 1 && sIdx == numSNR
                Hstore = H_freq;
            end

            %% Channel transmission on each subcarrier
            Y_rx = zeros(nRx, Nfft);
            for sc = 1:Nfft
                Hk = squeeze(H_freq(:,:,sc));
                xk = S_tx(:,sc);
                nk = sqrt(noiseVar/2) * (randn(nRx,1) + 1j*randn(nRx,1));
                yk = Hk * xk + nk;
                Y_rx(:,sc) = yk;
            end

            %% MMSE equalization only
            Xhat_MMSE = zeros(nTx, Nfft);

            for sc = 1:Nfft
                Hk = squeeze(H_freq(:,:,sc));
                yk = Y_rx(:,sc);

                Wmmse = (Hk' * Hk + noiseVar * eye(nTx)) \ Hk';
                Xhat_MMSE(:,sc) = Wmmse * yk;

                mseMMSE(sc) = mseMMSE(sc) + mean(abs(Xhat_MMSE(:,sc) - S_tx(:,sc)).^2);
                mseCount(sc) = mseCount(sc) + 1;
            end

            %% Demap bits
            rxBitsMMSE = zeros(nTx, Nfft * bitsPerSymbol);
            for tx = 1:nTx
                rxBitsMMSE(tx,:) = demapSymbols(Xhat_MMSE(tx,:), M);
            end

            %% BER accumulation
            errMMSE = errMMSE + sum(bits_tx(:) ~= rxBitsMMSE(:));
            totalBits = totalBits + numel(bits_tx);

            %% Save constellation at highest SNR
            if sIdx == numSNR
                constMMSE = [constMMSE; Xhat_MMSE(1,:).']; %#ok<AGROW>
            end

        end % blk

        BER(mIdx, sIdx) = errMMSE / totalBits;

        fprintf('M = %2d | SNR = %2d dB | BER MMSE = %.6f\n', ...
            M, snrDb, BER(mIdx, sIdx));

        if sIdx == numSNR
            detectedConst{mIdx} = constMMSE;
            mseStore{mIdx} = mseMMSE ./ mseCount;
        end

    end % sIdx

    fprintf('Done modulation M = %d\n', M);
end

%% =======================
% Plot 1: BER comparison
%% =======================
figure;
semilogy(SNRdB, squeeze(BER(1,:)), '-o', 'LineWidth', 1.6); hold on;
semilogy(SNRdB, squeeze(BER(2,:)), '-s', 'LineWidth', 1.6);
grid on;
xlabel('SNR (dB)');
ylabel('BER');
title('BER Comparison for 2x2 MIMO-OFDM with MMSE');
legend('QPSK - MMSE', '16QAM - MMSE', 'Location', 'southwest');

%% =======================
% Plot 2: Constellations
%% =======================
figure;
subplot(1,2,1);
plot(real(detectedConst{1}(1:min(500,end))), imag(detectedConst{1}(1:min(500,end))), '.');
grid on; axis equal;
title('QPSK - MMSE');
xlabel('In-Phase'); ylabel('Quadrature');

subplot(1,2,2);
plot(real(detectedConst{2}(1:min(500,end))), imag(detectedConst{2}(1:min(500,end))), '.');
grid on; axis equal;
title('16-QAM - MMSE');
xlabel('In-Phase'); ylabel('Quadrature');

%% =======================
% Plot 3: Channel frequency response
%% =======================
figure;
hold on;
leg = {};
for rx = 1:nRx
    for tx = 1:nTx
        plot(0:Nfft-1, 20*log10(abs(squeeze(Hstore(rx,tx,:))) + eps), 'LineWidth', 1.2);
        leg{end+1} = sprintf('H_{%d%d}', rx, tx); %#ok<AGROW>
    end
end
grid on;
xlabel('Subcarrier index');
ylabel('|H(f)| (dB)');
title('Channel Frequency Response');
legend(leg, 'Location', 'best');

%% =======================
% Plot 4: Relative gain per subcarrier
%% =======================
Hpower = squeeze(sum(sum(abs(Hstore).^2,1),2));
Hpower_dB = 10*log10(Hpower + eps);
Hpower_dB = Hpower_dB - max(Hpower_dB);

figure;
plot(0:Nfft-1, Hpower_dB, '-o', 'LineWidth', 1.4);
grid on;
xlabel('Subcarrier index');
ylabel('Relative gain (dB)');
title('Relative Channel Gain per Subcarrier');

%% =======================
% Plot 5: OFDM spectrum / PSD
%% =======================
figure;
try
    pwelch(psdSignal, hamming(64), 32, 512, 1, 'centered');
    title('PSD of Transmitted OFDM Signal');
catch
    Npsd = 512;
    Pxx = abs(fft(psdSignal, Npsd)).^2 / length(psdSignal);
    f = linspace(-0.5, 0.5, Npsd);
    plot(f, fftshift(10*log10(Pxx + eps)), 'LineWidth', 1.2);
    grid on;
    xlabel('Normalized Frequency');
    ylabel('Power (dB)');
    title('Power Spectrum of OFDM Signal');
end

%% =======================
% Plot 6: Per-subcarrier MSE
%% =======================
figure;
subplot(1,2,1);
plot(0:Nfft-1, 10*log10(mseStore{1} + eps), '-o');
grid on;
xlabel('Subcarrier index'); ylabel('MSE (dB)');
title('QPSK - MMSE MSE');

subplot(1,2,2);
plot(0:Nfft-1, 10*log10(mseStore{2} + eps), '-s');
grid on;
xlabel('Subcarrier index'); ylabel('MSE (dB)');
title('16-QAM - MMSE MSE');

disp('Simulation finished successfully.');

end

% =========================================================
% Subfunctions
% =========================================================

function sym = mapSymbols(bits, M)
bits = bits(:).';
bps = log2(M);
bmat = reshape(bits, bps, []).';

if M == 4
    sym = (1/sqrt(2)) * ((1 - 2*bmat(:,1)) + 1j*(1 - 2*bmat(:,2)));

elseif M == 16
    nSym = size(bmat,1);
    Ilev = zeros(nSym,1);
    Qlev = zeros(nSym,1);

    ibits = bmat(:,1:2);
    qbits = bmat(:,3:4);

    Ilev(ibits(:,1)==0 & ibits(:,2)==0) = -3;
    Ilev(ibits(:,1)==0 & ibits(:,2)==1) = -1;
    Ilev(ibits(:,1)==1 & ibits(:,2)==1) =  1;
    Ilev(ibits(:,1)==1 & ibits(:,2)==0) =  3;

    Qlev(qbits(:,1)==0 & qbits(:,2)==0) = -3;
    Qlev(qbits(:,1)==0 & qbits(:,2)==1) = -1;
    Qlev(qbits(:,1)==1 & qbits(:,2)==1) =  1;
    Qlev(qbits(:,1)==1 & qbits(:,2)==0) =  3;

    sym = (Ilev + 1j*Qlev) / sqrt(10);
else
    error('Only M = 4 and M = 16 are supported.');
end

sym = sym.';
end

function bits = demapSymbols(sym, M)
sym = sym(:);

if M == 4
    b0 = real(sym) < 0;
    b1 = imag(sym) < 0;
    bits = zeros(1, 2*length(sym));
    bits(1:2:end) = b0;
    bits(2:2:end) = b1;

elseif M == 16
    levels = [-3 -1 1 3];
    const = [];
    constBits = [];

    for ii = 1:4
        for qq = 1:4
            c = (levels(ii) + 1j*levels(qq)) / sqrt(10);
            const = [const; c]; %#ok<AGROW>

            switch ii
                case 1, Ibits = [0 0];
                case 2, Ibits = [0 1];
                case 3, Ibits = [1 1];
                case 4, Ibits = [1 0];
            end

            switch qq
                case 1, Qbits = [0 0];
                case 2, Qbits = [0 1];
                case 3, Qbits = [1 1];
                case 4, Qbits = [1 0];
            end

            constBits = [constBits; Ibits Qbits]; %#ok<AGROW>
        end
    end

    bits = zeros(1, 4*length(sym));
    for k = 1:length(sym)
        [~, idx] = min(abs(sym(k) - const));
        bits((k-1)*4+1:k*4) = constBits(idx,:);
    end
else
    error('Only M = 4 and M = 16 are supported.');
end
end