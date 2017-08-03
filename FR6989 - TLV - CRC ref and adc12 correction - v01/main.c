//******************************************************************************
//  MSP430FR6989 Demo - ADC12B, Sample A3, 2.5V Shared Ref, TLV, CRC16
//
//  Based in example "tlv_ex3_calibrateTempSensor" of "MSP430 DriverLib - TI"
//  This example show how to get and use TLV data to increase accuracy of
//  internal reference value (real value) and ADC data using it as reference
//
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//        >---|P1.3/A3          |
//
//  Author: Haroldo Amaral - agaelema@gmail.com
//  2017/08/01
//******************************************************************************

#include    <msp430.h>
#include    "driverlib.h"

#define     SAMPLES     64

#define     CRC_HW              // calculate CRC using CRC16 HW of MSP430
#define     CRC_SW              // calculate CRC by software

/*
 * TLV -ADC Variables
 */
struct s_TLV_ADC_Cal_Data *pADCCAL;
uint8_t bADCCAL_bytes;
uint16_t ADCGain;
int16_t ADCOffset;
uint16_t ADC_12V_Ref_30C, ADC_12V_Ref_85C, ADC_20V_Ref_30C;
uint16_t ADC_20V_Ref_85C, ADC_25V_Ref_30C, ADC_25V_Ref_85C;

/*
 * TLV - REF Variables
 */
struct s_TLV_REF_Cal_Data *pREFCAL;
uint8_t bREFCAL_bytes;
uint16_t REF_12V, REF_20V, REF_25V;

/*
 * other variables
 */
const float Vref_ideal = 2.5f;
float Vref_correct = 0;

float Vbit_ideal = 0;
float Vbit_correct = 0;
float Gain_correct = 0;

uint16_t ADC_raw = 0;
uint32_t ADC_average = 0;
float ADC_correct1 = 0;
float ADC_correct2 = 0;
float ADC_correct3 = 0;
float ADC_correct4 = 0;
float ADC_correct5 = 0;

uint16_t counter = 0;

/*
 * Prototype of functions
 */
void readADCCAL(void);
void readREFCAL(void);
uint16_t crc16(const uint8_t* data_p, uint8_t length);

void main(void)
{
    /* Stop WDT */
    WDT_A_hold(WDT_A_BASE);


#ifdef      CRC_HW
    /*
     * Calculate the CRC16 using MSP430 CRC16 HW
     * Based in this post
     * - https://e2e.ti.com/support/microcontrollers/msp430/f/166/p/52174/713993#713993
     */
    uint16_t i;
    uint8_t CRCRESULT_LSB, CRCRESULT_MSB;
    CRCINIRES = 0xFFFF;
    i = 0x01A04;

    for (i = 0x01A04; i <= 0x01AFF; i++) {
        CRCDIRB_L = *(unsigned char*)(i);
    };

    CRCRESULT_LSB = CRCINIRES_L;
    CRCRESULT_MSB = CRCINIRES_H;
    if (CRCRESULT_LSB != *(unsigned char*)(0x1A02))
        while (1); //crc fail
    if (CRCRESULT_MSB != *(unsigned char*)(0x1A03))
        while (1); // crc fail
    __no_operation();
#endif

#ifdef      CRC_SW
    /*
     * calculate the CRC based in Software
     */
    volatile uint8_t *ptr = (uint8_t *)0x01A04;
    volatile uint16_t crc_value = 0;

    crc_value = crc16(ptr, (0x01AFF - 0x01a04 + 1));            // 252 bytes

    if ((crc_value & 0xFF) != *(unsigned char*)(0x1A02))
        while (1); //crc fail
    if (((crc_value >> 8) & 0xFF) != *(unsigned char*)(0x1A03))
        while (1); // crc fail
    __no_operation();                   // add breakpoint to debug value
#endif


    /*
     * Read TLV specific values
     */
    readREFCAL();
    readADCCAL();

    /* Calculate the real/correct Vref value */
    Vref_correct = ((float)REF_25V * Vref_ideal) / (uint16_t)(1<<15);
    __no_operation();                   // add breakpoint to debug value


    /*
     * Select Port 1
     * Set Pin 0 to output Ternary Module Function, (V_ref+).
     * Set Pin 3 to output Ternary Module Function, (A3, C3).
     */
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        GPIO_PORT_P1,
        GPIO_PIN0 + GPIO_PIN3,
        GPIO_TERNARY_MODULE_FUNCTION
        );

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    /* If ref generator busy, WAIT  */
    while(Ref_A_isRefGenBusy(REF_A_BASE));

    /* Select internal reference to 2.5V */
    Ref_A_setReferenceVoltage(REF_A_BASE, REF_A_VREF2_5V);

    /* Turn on Reference Voltage */
    Ref_A_enableReferenceVoltage(REF_A_BASE);

//    /* Enable Ref_out in P1.0 */
//    Ref_A_enableReferenceVoltageOutput(REF_A_BASE);

    /*
     * Configure ADC12 module - Base address of ADC12B Module
     *
     * Use internal ADC12B bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider/pre-divider of 1
     * Not use internal channel
     */
    ADC12_B_initParam initParam = {0};
    initParam.sampleHoldSignalSourceSelect = ADC12_B_SAMPLEHOLDSOURCE_SC;
    initParam.clockSourceSelect = ADC12_B_CLOCKSOURCE_ADC12OSC;
    initParam.clockSourceDivider = ADC12_B_CLOCKDIVIDER_1;
    initParam.clockSourcePredivider = ADC12_B_CLOCKPREDIVIDER__1;
    initParam.internalChannelMap = ADC12_B_NOINTCH;
    ADC12_B_init(ADC12_B_BASE, &initParam);

    /* Enable the ADC12B module */
    ADC12_B_enable(ADC12_B_BASE);

    /*
     * Base address of ADC12B Module
     * For memory buffers 0-7 sample/hold for 64 clock cycles
     * For memory buffers 8-15 sample/hold for 4 clock cycles (default)
     * Disable Multiple Sampling
     */
    ADC12_B_setupSamplingTimer(ADC12_B_BASE,
                               ADC12_B_CYCLEHOLD_16_CYCLES,
                               ADC12_B_CYCLEHOLD_4_CYCLES,
                               ADC12_B_MULTIPLESAMPLESDISABLE);

    /*
     * Configure Memory Buffer
     *
     * Base address of the ADC12B Module
     * Configure memory buffer 0
     * Map input A3 to memory buffer 0
     * Vref+ = IntBuffer
     * Vref- = AVss
     * Memory buffer 0 is not the end of a sequence
     */
    ADC12_B_configureMemoryParam configureMemoryParam = {0};
    configureMemoryParam.memoryBufferControlIndex = ADC12_B_MEMORY_0;
    configureMemoryParam.inputSourceSelect = ADC12_B_INPUT_A3;
    configureMemoryParam.refVoltageSourceSelect = ADC12_B_VREFPOS_INTBUF_VREFNEG_VSS;
    configureMemoryParam.endOfSequence = ADC12_B_NOTENDOFSEQUENCE;
    configureMemoryParam.windowComparatorSelect = ADC12_B_WINDOW_COMPARATOR_DISABLE;
    configureMemoryParam.differentialModeSelect = ADC12_B_DIFFERENTIAL_MODE_DISABLE;
    ADC12_B_configureMemory(ADC12_B_BASE, &configureMemoryParam);

    ADC12_B_clearInterrupt(ADC12_B_BASE, 0, ADC12_B_IFG0);

    /* Enable memory buffer 0 interrupt */
    ADC12_B_enableInterrupt(ADC12_B_BASE, ADC12_B_IE0, 0, 0);

    __delay_cycles(75);                       // reference settling ~75us

    while(1)
    {
        /*
         * Enable/Start sampling and conversion
         *
         * Base address of ADC12B Module
         * Start the conversion into memory buffer 0
         * Use the single-channel, single-conversion mode
         */
        ADC12_B_startConversion(ADC12_B_BASE,
                                ADC12_B_MEMORY_0,
                                ADC12_B_SINGLECHANNEL);

        __bis_SR_register(LPM0_bits + GIE);    // LPM0, ADC10_ISR will force exit
        __no_operation();                      // For debug only

        if (counter < SAMPLES)
        {
            ADC_average += (uint32_t)ADC_raw;
            counter++;
        }
        else
        {
            ADC_average /= SAMPLES;

            Vbit_ideal = Vref_ideal / 4096;                     // ideal adc volts/bit resolution
            Vbit_correct = Vref_correct / 4096;                 // real adc volts/bit resolution
            Gain_correct = (float)ADCGain / 32768;              // gain factor correction

            /* 1 - Wihtout any calibration */
            ADC_correct1 = (float)ADC_average * Vbit_ideal;
            /* 2 - Correcting Vref */
            ADC_correct2 = (float)ADC_average * Vbit_correct;
            /* 3 - Correcting Vref and ADC Gain */
            ADC_correct3 = (float)ADC_average * Vbit_correct * Gain_correct;
            /* 4 - Correcting Vref, ADC Gain and ADC offset */
            ADC_correct4 = (float)ADC_average * Vbit_correct * Gain_correct + (ADCOffset * Vbit_correct);
            /* 5 - Correcting Vref, ADC Gain and ADC offset - less computer intensive */
            ADC_correct5 = (Vbit_ideal) * ((((((int32_t)(ADC_average << 1) * (int32_t)REF_25V) >> 16) * (int32_t)ADCGain) >> 15) + (int32_t)ADCOffset);

            ADC_average = 0;
            counter = 0;
            __no_operation();
        }
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
#endif
void ADC12_ISR(void)
{
    switch(__even_in_range(ADC12IV,12))
    {
    case  0: break;                         // Vector  0:  No interrupt
    case  2: break;                         // Vector  2:  ADC12BMEMx Overflow
    case  4: break;                         // Vector  4:  Conversion time overflow
    case  6: break;                         // Vector  6:  ADC12BHI
    case  8: break;                         // Vector  8:  ADC12BLO
    case 10: break;                         // Vector 10:  ADC12BIN
    case 12:                                // Vector 12:  ADC12BMEM0 Interrupt
        ADC_raw = ADC12_B_getResults(ADC12_B_BASE, ADC12_B_MEMORY_0);
        __bic_SR_register_on_exit(LPM0_bits);           // Exit active CPU
        break;                              // Clear CPUOFF bit from 0(SR)
    case 14: break;                         // Vector 14:  ADC12BMEM1
    case 16: break;                         // Vector 16:  ADC12BMEM2
    case 18: break;                         // Vector 18:  ADC12BMEM3
    case 20: break;                         // Vector 20:  ADC12BMEM4
    case 22: break;                         // Vector 22:  ADC12BMEM5
    case 24: break;                         // Vector 24:  ADC12BMEM6
    case 26: break;                         // Vector 26:  ADC12BMEM7
    case 28: break;                         // Vector 28:  ADC12BMEM8
    case 30: break;                         // Vector 30:  ADC12BMEM9
    case 32: break;                         // Vector 32:  ADC12BMEM10
    case 34: break;                         // Vector 34:  ADC12BMEM11
    case 36: break;                         // Vector 36:  ADC12BMEM12
    case 38: break;                         // Vector 38:  ADC12BMEM13
    case 40: break;                         // Vector 40:  ADC12BMEM14
    case 42: break;                         // Vector 42:  ADC12BMEM15
    case 44: break;                         // Vector 44:  ADC12BMEM16
    case 46: break;                         // Vector 46:  ADC12BMEM17
    case 48: break;                         // Vector 48:  ADC12BMEM18
    case 50: break;                         // Vector 50:  ADC12BMEM19
    case 52: break;                         // Vector 52:  ADC12BMEM20
    case 54: break;                         // Vector 54:  ADC12BMEM21
    case 56: break;                         // Vector 56:  ADC12BMEM22
    case 58: break;                         // Vector 58:  ADC12BMEM23
    case 60: break;                         // Vector 60:  ADC12BMEM24
    case 62: break;                         // Vector 62:  ADC12BMEM25
    case 64: break;                         // Vector 64:  ADC12BMEM26
    case 66: break;                         // Vector 66:  ADC12BMEM27
    case 68: break;                         // Vector 68:  ADC12BMEM28
    case 70: break;                         // Vector 70:  ADC12BMEM29
    case 72: break;                         // Vector 72:  ADC12BMEM30
    case 74: break;                         // Vector 74:  ADC12BMEM31
    case 76: break;                         // Vector 76:  ADC12BRDY
    default: break;
    }
}


/*******************************************************************************
 * crc16 - return the crc16 CCITT of data
 *
 *  *data_p: data pointer
 *  length: length of data (array) in bytes
 *
 *  Based in this post: https://stackoverflow.com/a/23726131
 ******************************************************************************/
uint16_t crc16(const uint8_t* data_p, uint8_t length)
{
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}


/******************************************************************************
 * This function will search the TLV block for the ADC CAL tag and store the
 * address of the first data in the block in the reference pointer passed as an
 * argument. The data is then saved to local variables.
 ******************************************************************************/
void readADCCAL(void)
{
    /* Read ADC12 Calibration Values */
    TLV_getInfo(TLV_TAG_ADCCAL,
                0,
                &bADCCAL_bytes,
                (uint16_t **)&pADCCAL
                );

    ADCGain = pADCCAL->adc_gain_factor;
    ADCOffset = pADCCAL->adc_offset;
    ADC_12V_Ref_30C = pADCCAL->adc_ref15_30_temp;
    ADC_12V_Ref_85C = pADCCAL->adc_ref15_85_temp;
    ADC_20V_Ref_30C = pADCCAL->adc_ref20_30_temp;
    ADC_20V_Ref_85C = pADCCAL->adc_ref20_85_temp;
    ADC_25V_Ref_30C = pADCCAL->adc_ref25_30_temp;
    ADC_25V_Ref_85C = pADCCAL->adc_ref25_85_temp;
}


/******************************************************************************
 * This function will search the TLV block for the REF CAL tag and store the
 * address of the first data in the block in the reference pointer passed as an
 * argument. The data is then saved to local variables.
 ******************************************************************************/
void readREFCAL(void)
{
    /* Read REF Calibration Values */
    TLV_getInfo(TLV_TAG_REFCAL,
                0,
                &bREFCAL_bytes,
                (uint16_t **)&pREFCAL
                );

    REF_12V = pREFCAL->ref_ref15;
    REF_20V = pREFCAL->ref_ref20;
    REF_25V = pREFCAL->ref_ref25;
}
