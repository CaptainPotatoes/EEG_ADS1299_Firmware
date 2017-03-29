# nRF51 ADS1299 Bluetooth LE Firmware
Bluetooth LE Firmware for nRF51 and ADS1299 SDK 11.0.0

Download the NRF SDK 11.0.0, and copy master folder to \examples\ble_peripheral\.
It should work.

## 3/29/17 Update & Notes
Tested using nRF51 development kit (PCA_10028) with an ADS1299-4 breakout board. It works fine for four channels at 250Hz. 
This is only for demonstration, use at your own risk. Configuration may be modified to work with any ADS1299 or ADS1299-x board.
The configuration set up in this firmware is for a single ended (referential montage setup) for 4 channels (ADS1299-4), using the SRB1 pin as negative electrode for all four channels.

The Ads1299_defRegs.xlsx file will help with configuration. 

See circuit schematics [here](https://github.com/CaptainPotatoes/KiCAD-EEG-Design/blob/master/Schematics.pdf)
