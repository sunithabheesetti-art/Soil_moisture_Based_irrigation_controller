# Soil_moisture_Based_irrigation_controller
 Soilmoisture based irrigation controller that samples soil moisture,temperature on non-blocking schedule filters out spikes, then triggers the pump only after several consecutive out-of-range readings to avoid false alarms. It detects sensor faults and switches to a safe fault state instead of acting on bad data the data is preserved even  offline
## Problem Description
Farmers irrigate on a fixed schedule because there is no way to know what the soil actually holds. Water is applied when the field is already wet, wasting scarce groundwater and pumping cost, and withheld when the crop is under stress because the schedule says the next turn is tomorrow. The measurement that would settle it is simple, but no affordable controller that acts on it locally is available.
## Proposed Solution
So i have  build a autonomous soil moisture based irrigation controller that measures soil moisture, decides on its own whether irrigation is needed, operates the pump accordingly, and continues to work correctly when the network is unavailable or a sensor fails. Tools You May Use

## Requirement Table
| Parameter | Value | Reasoning |
|---|---|---|
| Sampling interval | 2000 ms | Fast enough to react, slow enough not to flood the log/network |
| Safe range (soil moisture) | 30% – 70% | Below = drought stress, above = waterlogging/root rot risk |
| Action threshold — irrigate ON | < 30% | Genuine dry condition |
| Action threshold — irrigate OFF | ≥ 55% | Hysteresis gap (30→55) stops relay chatter at the boundary |
| High alert (waterlogged) | > 70% | Pump forced off, alert raised |
| Consecutive readings before acting | 3 (≈6 s) | A single noisy/spiked sample cannot trigger the pump |
| Plausible sensor range | moisture 0–100%, temp −10…60 °C | Anything outside is physically impossible → rejected, not smoothed |
| Stuck-value rule | 6 identical consecutive raw samples | Distinguishes "sensor frozen" from "soil genuinely stable" |
| Store-and-forward buffer | 30 records, FIFO | Bridges typical outage lengths in this simulation |
