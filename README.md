# Low-Cost UAS Detection

MATLAB simulation of ASTM F3411 Remote ID drone beacons transmitted over IEEE 802.11 (Wi-Fi) Beacon frames, building toward a distributed MIMO sensing system for passive UAS detection and localisation.

**MAI Final Year Project, Electronic Engineering, Trinity College Dublin**

---

## Overview

Remote ID (ASTM F3411-22a) requires drones to broadcast their identity and status over Wi-Fi as standard 802.11 beacon frames. The goal for this project is to track the transmitted GPS coordinates of a drone and to cross-validate this data by localising the signal, thus ensuring that the drone is compliant with the standard and is not spoofing their location. This project simulates the full transmission and reception chain, from encoding compliant F3411 payloads to recovering them through a realistic channel. 


## Repository Structure

```
MATLAB Simulation/
├── Code/                        # Modular function library 
│   ├── encodeRidMsg.m           # Encode F3411 Message Pack
│   ├── decodeRidMsg.m           # Unpack received Message Pack
│   ├── buildBeaconWaveform.m    # Takes F3411 message → MAC beacon → non-HT OFDM waveform
│   ├── ridTransmitter.m         # Top-level Tx function: encodes msg and builds waveform before scaling Tx power and applying CFO
│   ├── applyChannel.m           # Channel stage: Applies geometrically accurate channel - LOS only for now, plans to add multipath later
│   ├── ridReceiver.m            # Top-level Rx function: applies Rx CFO, detects, RSSI measurement, CFO corrects, noise measurement, equalises, decodes
│   ├── ridDemo.m                # Single-link demo with diagnostic plots
│   ├── simulateScenario.m       # Multi-Tx/Rx simulation
│   ├── plotScenario.m           # Plotting of logged results from simulateScenario
├── droneIdSimulation.m          # Original prototype 
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
    → RSSI calculation (1dBm resolution)
    → Coarse CFO estimate (L-STF)
    → Fine CFO estimate (L-LTF)
    → Channel estimate (L-LTF)
    → PSDU recovery (wlanNonHTDataRecover)
    → MPDU decode + FCS (wlanMPDUDecode)
    → Vendor IE extraction → F3411 decode → UAS ID
```

