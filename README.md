# Soil_moisture_Based_irrigation_controller
 Soilmoisture based irrigation controller that samples soil moisture,temperature on non-blocking schedule filters out spikes, then triggers the pump only after several consecutive out-of-range readings to avoid false alarms. It detects sensor faults and switches to a safe fault state instead of acting on bad data the data is preserved even  offline
