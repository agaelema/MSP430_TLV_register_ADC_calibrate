# MSP430_TLV_register_ADC_calibrate

CCS project showing how to get TLV registers and use them to calibrate internal reference and ADC samples


### FR6989 - TLV - CRC ref and adc12 correction - v01

Based in the Launchpad EXP430FR6989
* ADC12 Channel 3 (P1.3) - single-endded mode
* CRC16 HW - To get CRC16 CCITT of TLV Table
* CRC16 in software - to devices without CRC HW
* Ref_out can be enable (same pin of SW1 in launchpad)