
#include <msp430.h>
#include "Board.h"
#include "driverlib.h"
#include "stdbool.h"
#include "hal_LCD.h"
#include "string.h"
#include "main.h"


char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result


void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN); //1.2
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN); //2.6

    //Set LED1 and LED2 as outputs
    //GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
}

/* Clock System Initialization */
void Init_Clock(void)
{

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
    }
}

/* Buzzer PWM Initialization */
void Init_Buzzer(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //BZ1 (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(BZ1_PORT, BZ1_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}

//void Init_ADC(void)
//{
//    /*
//     * To use the ADC, you need to tell a physical pin to be an analog input instead
//     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
//     * these in main.h for A9 on P8.1.
//     */
//
//    //Set ADC_IN to input direction
//    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_IN_PORT, ADC_IN_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
//
//    //Initialize the ADC Module
//    /*
//     * Base Address for the ADC Module
//     * Use internal ADC bit as sample/hold signal to start conversion
//     * USE MODOSC 5MHZ Digital Oscillator as clock source
//     * Use default clock divider of 1
//     */
//    ADC_init(ADC_BASE,
//             ADC_SAMPLEHOLDSOURCE_SC,
//             ADC_CLOCKSOURCE_ADCOSC,
//             ADC_CLOCKDIVIDER_1);
//
//    ADC_enable(ADC_BASE);
//
//    /*
//     * Base Address for the ADC Module
//     * Sample/hold for 16 clock cycles
//     * Do not enable Multiple Sampling
//     */
//    ADC_setupSamplingTimer(ADC_BASE,
//                           ADC_CYCLEHOLD_16_CYCLES,
//                           ADC_MULTIPLESAMPLESDISABLE);
//
//    //Configure Memory Buffer
//    /*
//     * Base Address for the ADC Module
//     * Use input ADC_IN_CHANNEL
//     * Use positive reference of AVcc
//     * Use negative reference of AVss
//     */
//    ADC_configureMemory(ADC_BASE,
//                        ADC_IN_CHANNEL,
//                        ADC_VREFPOS_AVCC,
//                        ADC_VREFNEG_AVSS);
//
//    ADC_clearInterrupt(ADC_BASE,
//                       ADC_COMPLETED_INTERRUPT);
//
//    //Enable Memory Buffer interrupt
//    ADC_enableInterrupt(ADC_BASE,
//                        ADC_COMPLETED_INTERRUPT);
//}
//
////ADC interrupt service routine
//#pragma vector=ADC_VECTOR
//__interrupt
//void ADC_ISR(void)
//{
//    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);
//
//    ADC_clearInterrupt(ADC_BASE, ADCStatus);
//
//    if (ADCStatus)
//    {
//        ADCState = 0; //Not busy anymore
//        ADCResult = ADC_getResults(ADC_BASE);
//    }
//}

// LCD memory map for numeric digits
const char digit[10][2] =
{
    {0xFC, 0x28},  /* "0" LCD segments a+b+c+d+e+f+k+q */
    {0x60, 0x20},  /* "1" */
    {0xDB, 0x00},  /* "2" */
    {0xF3, 0x00},  /* "3" */
    {0x67, 0x00},  /* "4" */
    {0xB7, 0x00},  /* "5" */
    {0xBF, 0x00},  /* "6" */
    {0xE4, 0x00},  /* "7" */
    {0xFF, 0x00},  /* "8" */
    {0xF7, 0x00}   /* "9" */
};

// LCD memory map for uppercase letters
const char alphabetBig[26][2] =
{
    {0xEF, 0x00},  /* "A" LCD segments a+b+c+e+f+g+m */
    {0xF1, 0x50},  /* "B" */
    {0x9C, 0x00},  /* "C" */
    {0xF0, 0x50},  /* "D" */
    {0x9F, 0x00},  /* "E" */
    {0x8F, 0x00},  /* "F" */
    {0xBD, 0x00},  /* "G" */
    {0x6F, 0x00},  /* "H" */
    {0x90, 0x50},  /* "I" */
    {0x78, 0x00},  /* "J" */
    {0x0E, 0x22},  /* "K" */
    {0x1C, 0x00},  /* "L" */
    {0x6C, 0xA0},  /* "M" */
    {0x6C, 0x82},  /* "N" */
    {0xFC, 0x00},  /* "O" */
    {0xCF, 0x00},  /* "P" */
    {0xFC, 0x02},  /* "Q" */
    {0xCF, 0x02},  /* "R" */
    {0xB7, 0x00},  /* "S" */
    {0x80, 0x50},  /* "T" */
    {0x7C, 0x00},  /* "U" */
    {0x0C, 0x28},  /* "V" */
    {0x6C, 0x0A},  /* "W" */
    {0x00, 0xAA},  /* "X" */
    {0x00, 0xB0},  /* "Y" */
    {0x90, 0x28}   /* "Z" */
};

void Init_LCD()
{
    // L0~L26 & L36~L39 pins selected
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_SEGMENT_LINE_26);
    LCD_E_setPinAsLCDFunctionEx(LCD_E_BASE, LCD_E_SEGMENT_LINE_36, LCD_E_SEGMENT_LINE_39);

    LCD_E_initParam initParams = {0};
        initParams.clockSource  = LCD_E_CLOCKSOURCE_XTCLK;
        initParams.clockDivider = LCD_E_CLOCKDIVIDER_3;
        initParams.muxRate      = LCD_E_4_MUX;
        initParams.waveforms    = LCD_E_STANDARD_WAVEFORMS;
        initParams.segments     = LCD_E_SEGMENTS_ENABLED;

    // Init LCD as 4-mux mode
    LCD_E_init(LCD_E_BASE, &initParams);

    // LCD Operation - Mode 3, internal 3.02v, charge pump 256Hz
    LCD_E_setVLCDSource(LCD_E_BASE, LCD_E_INTERNAL_REFERENCE_VOLTAGE, LCD_E_EXTERNAL_SUPPLY_VOLTAGE);
    LCD_E_setVLCDVoltage(LCD_E_BASE, LCD_E_REFERENCE_VOLTAGE_2_96V);

    LCD_E_enableChargePump(LCD_E_BASE);
    LCD_E_setChargePumpFreq(LCD_E_BASE, LCD_E_CHARGEPUMP_FREQ_16);

    // Clear LCD memory
    LCD_E_clearAllMemory(LCD_E_BASE);

    // Configure COMs and SEGs
    // L0 = COM0, L1 = COM1, L2 = COM2, L3 = COM3
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_0, LCD_E_MEMORY_COM0);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_1, LCD_E_MEMORY_COM1);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_2, LCD_E_MEMORY_COM2);
    LCD_E_setPinAsCOM(LCD_E_BASE, LCD_E_SEGMENT_LINE_3, LCD_E_MEMORY_COM3);

    // Select to display main LCD memory
    LCD_E_selectDisplayMemory(LCD_E_BASE, LCD_E_DISPLAYSOURCE_MEMORY);

    // Turn on LCD
    LCD_E_on(LCD_E_BASE);
}

/*
 * Scrolls input string across LCD screen from left to right
 */
void displayScrollText(char *msg)
{
    int length = strlen(msg);
    int i;
    int s = 5;
    char buffer[6] = "      ";
    for (i=0; i<length+7; i++)
    {
        int t;
        for (t=0; t<6; t++)
            buffer[t] = ' ';
        int j;
        for (j=0; j<length; j++)
        {
            if (((s+j) >= 0) && ((s+j) < 6))
                buffer[s+j] = msg[j];
        }
        s--;

        showChar(buffer[0], pos1);
        showChar(buffer[1], pos2);
        showChar(buffer[2], pos3);
        showChar(buffer[3], pos4);
        showChar(buffer[4], pos5);
        showChar(buffer[5], pos6);

        __delay_cycles(200000);
    }
}

/*
 * Displays input character at given LCD digit/position
 * Only spaces, numeric digits, and uppercase letters are accepted characters
 */
void showChar(char c, int position)
{
    if (c == ' ')
    {
        // Display space
        LCDMEMW[position/2] = 0;
    }
    else if (c >= '0' && c <= '9')
    {
        // Display digit
        LCDMEMW[position/2] = digit[c-48][0] | (digit[c-48][1] << 8);
    }
    else if (c >= 'A' && c <= 'Z')
    {
        // Display alphabet
        LCDMEMW[position/2] = alphabetBig[c-65][0] | (alphabetBig[c-65][1] << 8);
    }
    else
    {
        // Turn all segments on if character is not a space, digit, or uppercase letter
        LCDMEMW[position/2] = 0xFFFF;
    }
}

/*
 * Displays hex value
 */
void showHex(int hex)
{
    showChar((char)((0xC0 & hex) >> 6) + '0', pos3);
    showChar((char)((0x30 & hex) >> 4) + '0', pos4);
    showChar((char)((0x0C & hex) >> 2) + '0', pos5);
    showChar((char)(0x03 & hex) + '0', pos6);
}

/*
 * Clears memories to all 6 digits on the LCD
 */
void clearLCD()
{
    LCDMEMW[pos1/2] = 0;
    LCDMEMW[pos2/2] = 0;
    LCDMEMW[pos3/2] = 0;
    LCDMEMW[pos4/2] = 0;
    LCDMEMW[pos5/2] = 0;
    LCDMEMW[pos6/2] = 0;
    LCDMEM[12] = LCDMEM[13] = 0;
}




int main( void )
{
    WDTCTL = WDTPW + WDTHOLD;               // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;                   // Disable the GPIO power-on default high-impedance mode
                                            // to activate previously configured port settings

    char buttonState = 0; //Current button press state (to allow edge detection)

     //Turn off interrupts during initialization
     __disable_interrupt();

     //        P1DIR |= 0x01;                          // Set P1.0 to output direction
     //        P8DIR |= BIT3;                          // Set P8.3 to output direction
     //        P1DIR |= BIT5;

      P8DIR |= BIT2;                         // Set P8.2 to output direction, door_sensor
      P1DIR |= BIT0;                         // Set P1.0 to output direction, window_sensor

      P8DIR |= BIT3;                          // set P8.3 to output direction, Red Led and Buzzer
      P1DIR |= BIT5;                          // Set P1.5 to output direction, Green Led



      //testing purposes
//      P8OUT |= BIT3;
//      P1OUT |= BIT5;

     //Stop watchdog timer unless you plan on using it
//     WDT_A_hold(WDT_A_BASE);

     // Initializations - see functions for more detail
     Init_GPIO();    //Sets all pins to output low as a default
//     Init_Buzzer();  //Sets up a PWM output
//     Init_ADC();     //Sets up the ADC to sample
     Init_Clock();   //Sets up the necessary system clocks
     Init_UART();    //Sets up an echo over a COM port
     Init_LCD();     //Sets up the LaunchPad LCD display

//     PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

     //All done initializations - turn interrupts back on.
     __enable_interrupt();



        char* msg = "PLEASE SELECT ZONE";
        char* msg1 = "DEFAULT ZONE 1 SELECTED";
        char* msg2 = "ZONE 2 SELECTED";
        char* msg3 = "ZONE 3 SELECTED";
        char* msg4 = "ZONE 4 SELECTED";
        char* conf1 = "ARE YOU SURE";
        char* conf2 = "CONFIRMED";
        char* conf3 = "PLEASE RESELECT";
        char* conf4 = "WAITING TO CONFIRM";
        char* change = "PRESS TO CHANGE ZONE";

        int zone1 = 0; //which zone is selected
        int zone2 = 0;
        int zone3 = 0;
        int zone4 = 0;
        int conf = 0; //0 is not confirmed, 1 is confirmed
        int wait = 0; //0 is wait, 1 is finished
        const int repeat = 0;

        while (repeat == 0){
            conf = 0;
            wait = 0;
            zone1 =1;
            zone2 = 0;
            zone3= 0;
            zone4 = 0;

            while (conf == 0){
                wait = 0;
                zone1 = 0;
                zone2 = 0;
                zone3 = 0;
                zone4 = 0;
                displayScrollText(msg);
//                __delay_cycles(500000);
                if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) &(GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1)){
                    displayScrollText(msg2);
                    zone1 = 0;
                    zone2 = 1;
                    zone3 = 0;
                    zone4 = 0;
                } else if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1)& (buttonState == 0) &(GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0)){
                    displayScrollText(msg3);
                    zone1 = 0;
                    zone2 = 0;
                    zone3 = 1;
                    zone4 = 0;
                } else if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) &(GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0)){
                    displayScrollText(msg4);
                    zone1 = 0;
                    zone2 = 0;
                    zone3 = 0;
                    zone4 = 1;
                } else {
                    displayScrollText(msg1);
                    zone1 = 1;
                    zone2 = 0;
                    zone3 = 0;
                    zone4 = 0;
                }


                while(wait == 0){
                    displayScrollText(conf1);
                    if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) & (GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1)){
                        displayScrollText(conf2);
                        conf = 1;
                        wait = 1;
                    } else if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1)& (buttonState == 0) &(GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 0)){
                        displayScrollText(conf3);
                        conf = 0;
                        wait = 1;
                    } else {
                        displayScrollText(conf4);
                        conf = 0;
                        wait = 0;
                    }
                }
            }

                if((P8IN & BIT2) == 0 && (P1IN & BIT0) == 0 ){   //detect magnet, both door, window are closed
                    P8OUT &= ~BIT3;                                //green light ok
                    P1OUT |= BIT5;
                    displayScrollText("SAFE");
                } else if ((P8IN & BIT2) == 1 && (P1IN & BIT0) == 0 ){
                    P8OUT |= BIT3;                              //red light
                    P1OUT &= ~BIT5;
                    displayScrollText("DOOR NOT CLOSED");
                }else if ((P8IN & BIT2) == 0 && (P1IN & BIT0) == 1 ){
                    P8OUT |= BIT3;                              //red light
                    P1OUT &= ~BIT5;
                    displayScrollText("WINDOW NOT CLOSED");
                } else {
                    P8OUT |= BIT3;                              //red light
                    P1OUT &= ~BIT5;
                    displayScrollText("BOTH NOT CLOSED");
                }
                displayScrollText(change);
                if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) & (GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1)){
                    zone1 = 0;
                }
            }
//            while(zone1 == 1){
            //zone 2
            while(zone2 == 1){
                if((P1IN & BIT0) == 0 ){   //detect magnet, window is closed
                    P8OUT &= ~BIT3;
                    P1OUT |= BIT5;
                    displayScrollText("SAFE");
                 } else{
                    P8OUT |= BIT3;
                    P1OUT &= ~BIT5;
                    displayScrollText("WINDOW NOT CLOSED");
                 }
                displayScrollText(change);
                if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) & (GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1)){
                    zone2 = 0;
                }
            }

            //zone 3
            while(zone3 ==1){
                if((P8IN & BIT2) == 0 ){   //detect magnet, door is closed
                    P8OUT &= ~BIT3;
                    P1OUT |= BIT5;
                    displayScrollText("SAFE");
                 } else{
                    P8OUT |= BIT3;
                    P1OUT &= ~BIT5;
                    displayScrollText("DOOR NOT CLOSED");
                }
                displayScrollText(change);
                if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) & (GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1)){
                    zone3 = 0;
                }
            }

            //zone 4
            while(zone4 == 1){
                displayScrollText("SAFE");
                P8OUT &= ~BIT3;
                P1OUT |= BIT5;
                displayScrollText(change);
                if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0)& (buttonState == 0) & (GPIO_getInputPinValue(SW2_PORT, SW2_PIN) == 1)){
                    zone4 = 0;
                }

                volatile unsigned int i;
                 i=10000;
                 P8OUT &= ~BIT3;
                 while(i !=0){
                     i--;
                     P1OUT |= BIT5;
                 }
            }


    clearLCD();




    __bis_SR_register(LPM3_bits | GIE);                        // Enter LPM3.5
    __no_operation();                                          // For debugger




}





