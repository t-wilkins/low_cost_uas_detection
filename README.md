# Low-Cost UAS Detection

MATLAB simulation of ASTM F3411 Remote ID drone beacons transmitted over IEEE 802.11 (Wi-Fi), built toward a distributed MIMO sensing system for passive UAS detection and localisation.

**MAI Final Year Project — Electronic Engineering, Trinity College Dublin**

---

## Overview

Remote ID (ASTM F3411-22a) requires drones to broadcast their identity and status over Wi-Fi as standard 802.11 beacon frames. This project simulates the full transmission and reception chain — from encoding compliant F3411 payloads to recovering them through a realistic channel — and characterises detection performance across multiple simultaneous transmitters as a baseline for a later distributed MIMO/DoA extension.

**Current stage:** multi-transmitter link-level simulation  
**Planned:** UAV trajectory integration, colocated MUSIC/ESPRIT DoA estimation, distributed array with timing and CFO synchronisation

---

## Repository Structure

```
MATLAB Simulation/
├── Code/                        # Modular function library
│   ├── ridEncodeBasicID.m       # Encode 25-byte F3411 Basic ID message
│   ├── ridEncodeMessagePack.m   # Wrap messages in F3411 Message Pack
│   ├── ridDecodeMessagePack.m   # Unpack received Message Pack
│   ├── ridDecodeBasicID.m       # Decode Basic ID back to struct
│   ├── ridHex.m                 # uint8 → hex string helper
│   ├── buildRidBeaconWaveform.m # Full TX chain: F3411 → MAC beacon → non-HT OFDM waveform
│   ├── applyRidChannel.m        # Channel stage: TGn Model-D, path loss, CFO, AWGN
│   ├── receiveRidBeacon.m       # Full RX chain: detect, CFO correct, equalise, decode
│   ├── ridBeaconDemo.m          # Single-link demo with diagnostic plots
│   ├── runRidSweep.m            # 2-D sensitivity sweep (distance × CFO)
│   ├── ridSimulateScenario.m    # Multi-transmitter scenario engine
│   └── runRidMultiTx.m          # Multi-Tx driver: single run + nTx sweep
├── droneIdSimulation.m          # Original monolithic prototype
└── MIMO_OFDM_code.m             # Early MIMO/OFDM reference
```

---

## Signal Chain

```
F3411 Basic ID message (25 bytes)
    → Message Pack (type 0xF)
    → Vendor-specific IE (OUI FA:0B:BC, type 0x0D)
    → 802.11 Beacon frame (wlanMACFrameConfig)
    → Non-HT OFDM waveform (MCS 0, CBW20, 6 Mbps)
    → TGn Model-D channel (path loss, shadowing, fading)
    → Carrier frequency offset
    → Thermal noise (kTBF)
    → Packet detect (wlanPacketDetect)
    → Coarse CFO estimate (L-STF)
    → Fine CFO estimate (L-LTF)
    → Channel estimate (L-LTF)
    → PSDU recovery (wlanNonHTDataRecover)
    → MPDU decode + FCS (wlanMPDUDecode)
    → Vendor IE extraction → F3411 decode → UAS ID
```

---

## Key Parameters

| Parameter | Value | Basis |
|---|---|---|
| PHY | Non-HT OFDM, CBW20 | 802.11-2020 Clause 17 |
| MCS | 0 (BPSK 1/2, 6 Mbps) | Minimum mandatory rate |
| Carrier | 2.4 GHz | F3411-22a Wi-Fi method |
| Channel model | TGn Model-D (50 ns RMS) | IEEE 802.11n indoor/office |
| Tx power | 20 dBm | Configurable |
| Noise figure | 4.5 dB | Typical 802.11 receiver |
| Multi-Tx model | Pure ALOHA | Conservative; no carrier sense |

---

## Usage

### Single demo link
```matlab
ridBeaconDemo
```

### Sensitivity sweep (distance × CFO)
```matlab
runRidSweep
```
Produces four heatmaps: detection rate, FCS decode rate, ID match rate, log BER.

### Multi-transmitter scenario
```matlab
runRidMultiTx
```
Part 1 runs one scenario (default: 10 Tx, 10 s) and prints per-transmitter statistics.  
Part 2 sweeps a config field (default: `nTx` in [1 5 10 25 50]) and plots detection, decode and collision rates.

### Custom scenario
```matlab
cfg = struct('nTx', 20, 'simTime', 30, 'distRange', [10 500]);
out = ridSimulateScenario(cfg);
disp(out.tx)
```

---

## Dependencies

- MATLAB (R2023a or later recommended)
- WLAN Toolbox
- Communications Toolbox

---

## Standards References

- ASTM F3411-22a — *Standard Specification for Remote ID and Tracking*
- IEEE Std 802.11-2020 — *Wireless LAN Medium Access Control and Physical Layer*
- IEEE 802.11n Amendment (TGn channel models) — Erceg et al., 2004
