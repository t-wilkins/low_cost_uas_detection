# Low-Cost UAS Detection

MATLAB simulation of ASTM F3411 Remote ID drone beacons transmitted over IEEE 802.11 (Wi-Fi), building toward a distributed MIMO sensing system for passive UAS detection and localisation.

**MAI Final Year Project — Electronic Engineering, Trinity College Dublin**

---

## Overview

Remote ID (ASTM F3411-22a) requires drones to broadcast their identity and status over Wi-Fi as standard 802.11 beacon frames. This project simulates the full transmission and reception chain, from encoding compliant F3411 payloads to recovering them through a realistic channel, and characterises detection performance across multiple scenarios as a baseline for a later distributed MIMO/DoA extension.

**Current stage:** 
 - Have developed end-to-end simulation of the broadcast and reception of a single F3411 Basic ID packet, printing and plotting the message and waveform as it moves through the signal chain
 - Have used Claude to split simulation into several different function files and develop a multiTx simulation. This code is temporary and was just used as a reference. Currently developing personal simulation code

**Planned:**
 - Using AI generated code as a reference, split functions into different modular files
 - Integrate a more representative, outdoor, drone-to-ground, wireless channel
 - Encode UAV location message into RID packet
 - Define a DxD space, with Rx at origin, and Tx at random coordinate within space, and plot the received vs true location
 - Sweep distance vs PPR with a single Tx/Rx pair to evaluate range
 - Introduce multiple transmitters, and simulate their RID beacon signals over a defined sim time, plotting packet receptions vs time and distance, and received vs true Tx locations
 - Increase the number of Rx nodes, initially in a synchronised ULA configuration and perform localisation using Capon/MUSIC/ESPRIT algorithms, plotting estimated vs received vs true locations
 - Investigate if this can be done for increasing numbers of transmitters, bearing in mind hardware/latency requirements for later implementation
 - Sweep CFO and distance values between Rx nodes and guage the degradation of the estimated location accuracy
 - Investigate synchronisation of Rx nodes using beacon timestamps, implement and determine reliability of this synchronisation for localisation
 - Using synchronised nodes, simulate distributed Rx array, plotting performance (PPR rate, Est vs Rec vs Tr location, etc.) against number of Rx nodes and varying D (location space)

---

## Repository Structure

```
MATLAB Simulation/
├── ClaudeRefCode/               # AI generated modular function library - proof of concept and reference
│   ├── ridEncodeMessage.m       # Encode 25-byte F3411 Basic ID message
│   ├── ridEncodeMessagePack.m   # Wrap messages in F3411 Message Pack
│   ├── ridDecodeMessagePack.m   # Unpack received Message Pack
│   ├── ridDecodeBasicID.m       # Decode Basic ID back to struct
│   ├── ridHex.m                 # uint8 → hex string helper
│   ├── buildRidBeaconWaveform.m # Full TX chain: F3411 message → MAC beacon → non-HT OFDM waveform
│   ├── applyRidChannel.m        # Channel stage: TGn Model-D, path loss, CFO, AWGN
│   ├── receiveRidBeacon.m       # Full RX chain: detect, CFO correct, equalise, decode
│   ├── ridBeaconDemo.m          # Single-link demo with diagnostic plots
│   ├── runRidSweep.m            # 2-D sensitivity sweep (distance × CFO)
│   ├── ridSimulateScenario.m    # Multi-transmitter scenario engine
│   └── runRidMultiTx.m          # Multi-Tx driver: single run + nTx sweep
├── droneIdSimulation.m          # Original prototype to be developed
```

---

## Signal Chain

```
F3411 Basic ID message (25 bytes)
    → Message Pack (type 0xF)
    → Vendor-specific IE (OUI FA:0B:BC, type 0x0D)
    → 802.11 Beacon frame (wlanMACFrameConfig)
    → Non-HT OFDM waveform (MCS 0, CBW20, 6 Mbps)
    → TGn Model-D channel (path loss, shadowing, fading) (Temp)
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
| PHY | Non-HT OFDM, CBW20 | 802.11 Clause 17 |
| MCS | 0 (BPSK 1/2, 6 Mbps) | Minimum mandatory rate |
| Carrier | 2.4 GHz | F3411 Wi-Fi method |
| Channel model | TGn Model-D (50 ns RMS) | IEEE 802.11n indoor/office (Placeholder - need to replace with more representative channel) |
| Tx power | 20 dBm | Configurable |
| Noise figure | 4.5 dB | ESP32 WiFi Receiver (https://esp32.com/viewtopic.php?t=3096) |

---

## Usage

### <ins>Personal:</ins>
### Single demo link
```matlab
droneIdSimulation
```

### <ins>Claude:</ins>
### Single demo link 
```matlab
ridBeaconDemo
```

### Sensitivity sweep (distance × CFO)
```matlab
runRidSweep
```
Produces a plot of detection rate, FCS decode rate, and ID match rate against distance, four heatmaps: detection rate, FCS decode rate, ID match rate, log BER.

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
