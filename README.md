# MSP430_TLV_register_ADC_calibrate

CCS project showing how to get TLV registers and use them to calibrate internal reference and ADC samples

* Accuracy improvment after TLV correction
![alt text](https://github.com/agaelema/MSP430_TLV_register_ADC_calibrate/blob/master/msp430-tlv-accuracy-test.png?raw=true?raw=true "Accuracy test")


### FR6989 - TLV - CRC ref and adc12 correction - v01

Based in the Launchpad EXP430FR6989
* ADC12 Channel 3 (P1.3) - single-endded mode
* CRC16 HW - To get CRC16 CCITT of TLV Table
* CRC16 in software - to devices without CRC HW
* Ref_out can be enable (same pin of SW1 in launchpad)

Article published in: https://www.embarcados.com.br/aumentando-a-exatidao-do-adc-do-msp430/

### DISCLAIMER

This software is provided 'as is' with no explicit or implied warranties in respect of its properties, including, but not limited to, correctness and/or fitness for purpose.