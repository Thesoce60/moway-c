//
// Created by Tom JUMEL on 04/11/2020.
//
//notes : TRIS = 0, pin OUTPUT, tris = 1 pin INPUT
#include "adc.h"        //A/D conversion library
#include "delays.h"
#include "lib_mot_moway.h"
#include "p18f86j50.h"
#ifndef MOWAYCORE_H
#define MOWAYCORE_H

//****************************************************************************************************************//
// JUMPING THE BOOTLOADER
//****************************************************************************************************************//
#define REMAPPED_RESET_VECTOR_ADDRESS            0x1000    //Reset adress for the correct bootloader jump
#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS    0x1008    //High priority interruption adress for the correct bootloader jump
#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS    0x1018    //Low priority interruption adress for the correct bootloader jump

void YourHighPriorityISRCode(); //Function that executes the needed code in case of the high priority interruption jumps.
void YourLowPriorityISRCode(); //Function that executes the needed code in case of the low priority interruption jumps.
//*********************RESET, HIGH AND LOW PRIORITY INTERRUPTION REMAP****************************//
extern void _startup(void);

#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS

void _reset(void) {
    _asm
            goto _startup
            _endasm
}

#pragma code

#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS

void Remapped_High_ISR(void) {
    _asm
            goto YourHighPriorityISRCode
            _endasm
}

#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS

void Remapped_Low_ISR(void) {
    _asm
            goto YourLowPriorityISRCode
            _endasm
}

#pragma code

#pragma interrupt YourHighPriorityISRCode

void YourHighPriorityISRCode() {
}

#pragma interruptlow YourLowPriorityISRCode

void YourLowPriorityISRCode() {
}
#define 	XOUT8		0x06
#define 	YOUT8		0x07
#define 	ZOUT8		0x08
#define 	DETSRC		0x0A
#define 	MCTL		0x16
#define 	INTRST		0x17
#define 	CTL1		0x18
#define 	PDTH		0x1B
#define 	PW			0x1C
#define 	LT			0x1D
#define 	TW			0x1E
//#####################################################################SENSOR_START########################################################
typedef unsigned char byte;
//---------------------------------------------------------------FUNCTION---------------------------------------------------//
//-----------FUNCTION DECLARATION
void ACCE_WRITE(unsigned char, unsigned char);
unsigned char ACCE_READ(unsigned char);

//*****************************************************
//*                  delay()                          *
//*****************************************************
//*Descripcion: wait millis time                      *
//*****************************************************
//*Input variables: millis (max: 65535)               *
//*****************************************************

void delay(unsigned int millis) {
    if (millis == 0)
        return;
    do {
        if (millis <= 255) {
            Delay1KTCYx(millis);
            millis = millis - millis;
        } else {
            if (millis <= 2550) {
                Delay10KTCYx(millis / 10);
                millis = millis % 10;
            } else {
                Delay10KTCYx(255);
                millis = millis - 2550;
            }
        }
    } while (millis != 0);
}
//*****************************************************
//*                delayInMicros()                    *
//*****************************************************
//*Descripcion: wait micros time                      *
//*****************************************************
//*Input variables: micros (max: 65535)               *
//*****************************************************

void delayInMicros(unsigned int micros) {
    if (micros == 0)
        return;
    do {
        if (micros <= 2550) {
            Delay10TCYx(micros / 10);
            micros = micros - micros;
        } else {
            Delay10TCYx(255);
            micros = micros - 2550;
        }
    } while (micros != 0);
}

//-------------------LED Function;

const struct {
    const byte LED_TOP_GREEN, LED_TOP_RED, LED_BRAKE, LED_FRONT;
} LED = {1, 2, 3, 4};

#define HIGH 0
#define LOW 1

void writeLedState(byte targetLed, byte state) {
    switch (targetLed) {
        case 3:
            PORTEbits.RE5 = state;
            //TRISEbits.TRISE5
            break;
        case 4:
            PORTCbits.RC7 = state;
            break;
        case 1:
            PORTBbits.RB6 = state;
            break;
        case 2:
            PORTBbits.RB5 = state;
            break;
    }
}

void blinkLed(byte targetLed) {
    writeLedState(targetLed, HIGH);
    delay(50);
    writeLedState(targetLed, LOW);
    delay(50);
}
//-------------------------SPEAKER FUNCTION;

void setSpeakerState(byte state) {
    PORTBbits.RB3 = (state == HIGH);
}

byte toneSpeaker(byte frequence, unsigned int millis) {

    if (millis == 0)
        return (0);

    PR2 = frequence;
    frequence = frequence & 0b11111110;
    CCPR2L = (frequence >> 1); //CCPR2L est un CCP en mode PWM

    setSpeakerState(LOW); //Speaker pin as a gnd

    T2CONbits.TMR2ON = 1;
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2CKPS1 = 1;

    delay(millis);

    CCP2CON = 0; // truning PWM to GND
    return (1);
}
//---------------------------------------------FRONT_SENSOR FUNCTION

const enum {
    CENTER_LEFT_S = 10, CENTER_RIGHT_S, SIDE_LEFT_S, SIDE_RIGHT_S
} Ir_Sensor_enum;

const struct {
    const byte CENTER_LEFT, CENTER_RIGHT, SIDE_LEFT, SIDE_RIGHT;
} IR = {CENTER_LEFT_S, CENTER_RIGHT_S, SIDE_LEFT_S, SIDE_RIGHT_S};

const byte IR_ADC_CONFIG_channel[] = {
    ADC_CH1 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
    ADC_CH11 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
    ADC_CH3 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS,
    ADC_CH10 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS
};
const byte IR_ADC_CONFIG_Channel_choose[] = {
    ADC_2ANA,
    ADC_12ANA,
    ADC_4ANA,
    ADC_11ANA
};

byte getIRState(byte sensor) {
    switch (sensor) {
        case SIDE_LEFT_S:
            return PORTAbits.RA3;
        case CENTER_LEFT_S:
            return PORTAbits.RA1;
        case CENTER_RIGHT_S:
            return PORTFbits.RF6;
        case SIDE_RIGHT_S:
            return PORTFbits.RF5;
        default:
            return 0xff;
    }
}

void setIRLed(byte state) {
    PORTJbits.RJ6 = (state == HIGH); //PORT IR LED
}
//    if ((getIRState(sensor)) != 0) {
//        delayInMicros(500);
//        return 0;
//    }
//    //Openning ADC Convert with channel ADC_CH3
//    OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD,
//            IR_ADC_CONFIG_channel[sensor],
//            IR_ADC_CONFIG_Channel_choose[sensor]
//            );
//    //Turn on IR led
//    setIRLed(HIGH);
//    delayInMicros(120);
//    //requesting analog
//    ConvertADC();
//    while (BusyADC());
//    //turn off IR led
//    setIRLed(LOW);
//    delayInMicros(500);
//
//    CloseADC();
//
//    if ((getIRState(sensor)) != 0) {
//        delayInMicros(500);
//        return 0;
//    }
//    return ADRESH;
#define   IR_RX_R_2     PORTFbits.RF5
#define   IR_RX_R_1     PORTFbits.RF6
#define   IR_RX_L_1     PORTAbits.RA1	       		//PORTA,1
#define   IR_RX_L_2     PORTAbits.RA3	       		//PORTA,3
byte checkObstacle(byte sensor) {
    byte SENSOR=0;

    //It checks which of the four sensors is going to be tested
    switch(sensor){

        case SIDE_LEFT_S:

            //Verification of non-existence of external signal
            if((IR_RX_L_2)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                return 0;
            }

            //The ADC channel 3 is selected
            OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH3 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS ,ADC_4ANA);

            //An infrared light pulse is sent for the obstacle detection
            setIRLed(HIGH);
            Delay10TCYx (12);		//Se espera 120 us

            //It is necessary to wait until the conversion is completely done
            ConvertADC();
            while( BusyADC() );

            //Reading of the left side sensor
            SENSOR=ADRESH;

            //The end of the infrared light pulse
            setIRLed(LOW);
            Delay10TCYx (50);		//Delay of 500 us

            //A/D conversion is disabled
            CloseADC();

            //Verification of non-existence of external signal
            if((IR_RX_L_2)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            break;

        case CENTER_LEFT_S:

            //Verification of non-existence of external signal
            if((IR_RX_L_1)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            //The ADC channel 1 is selected
            OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH1 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS ,ADC_2ANA);

            //An infrared light pulse is sent for the obstacle detection
            setIRLed(HIGH);
            Delay10TCYx (12);		//Delay of 120 us

            //It is necessary to wait until the conversion is completely done
            ConvertADC();
            while( BusyADC() );

            //Reading of the left central sensor
            SENSOR=ADRESH;

            //The end of the infrared light pulse
            setIRLed(LOW);
            Delay10TCYx (50);		//Delay of 500 us

            //A/D conversion is disabled
            CloseADC();

            //Verification of non-existence of external signal
            if((IR_RX_L_1)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            break;

        case CENTER_RIGHT_S:

            //Verification of non-existence of external signal
            if((IR_RX_R_1)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            //The ADC channel 11 is selected
            OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH11 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS ,ADC_12ANA);

            //An infrared light pulse is sent for the obstacle detection
            setIRLed(HIGH);
            Delay10TCYx (12);		//Delay of 120 us

            //It is necessary to wait until the conversion is completely done
            ConvertADC();
            while( BusyADC() );

            //Reading of the right central sensor
            SENSOR=ADRESH;

            //The end of the infrared light pulse
            setIRLed(LOW);
            Delay10TCYx (50);		//Delay of 500 us


            //A/D conversion is disabled
            CloseADC();

            //Verification of non-existence of external signal
            if((IR_RX_R_1)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            break;

        case SIDE_RIGHT_S:

            //Verification of non-existence of external signal
            if((IR_RX_R_2)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            //The ADC channel 10 is selected
            OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH10 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS ,ADC_11ANA);

            //An infrared light pulse is sent for the obstacle detection
            setIRLed(HIGH);
            Delay10TCYx (12);		//Delay of 120 us

            //It is necessary to wait until the conversion is completely done
            ConvertADC();
            while( BusyADC() );

            //Reading of the right side sensor
            SENSOR=ADRESH;

            //The end of the infrared light pulse
            setIRLed(LOW);
            Delay10TCYx (50);		//Delay of 500 us

            //A/D conversion is disabled
            CloseADC();

            //Verification of non-existence of external signal
            if((IR_RX_R_2)!=0){
                Delay10TCYx (50);		//Delay of 500 us
                SENSOR=0;
                break;
            }

            break;
        default:
            return(0);
    }
    //The value of the sensor is returned
    return (SENSOR);

}
byte checkObstacleWithOffsetD(byte sensor,byte offset){
    return (checkObstacle(sensor) > offset ? 1 : 0);
}
byte checkObstacleD(byte sensor) {
    byte res = 0;

    WDTCONbits.ADSHR = 1;
    ANCON0 = 0b11111111;
    ANCON1 = 0b11111111;
    WDTCONbits.ADSHR = 0;

    if (getIRState(sensor) != 0) {
        Delay10TCYx(50);
        return 0;
    }
    setIRLed(HIGH);
    delayInMicros(12);

    res = getIRState(sensor);

    setIRLed(LOW);
    delayInMicros(50);
    if (getIRState(sensor) != 0) {
        Delay10TCYx(50);
        return 0;
    }
    return res;
}
void (*IRCallback)(byte sensor);

void setIRSensorCallback(void (*Callback)(byte sensor)) {
    IRCallback = Callback;
}
void irRoutine();

void irRoutine() {
    byte i = 0;
    if (&IRCallback == 0x00)
        return;
    for (i = 0; i < 4; i++)if (checkObstacleD(i) != 0)(*IRCallback)(i);
}

const enum {
    LINE_RIGHT_S = 20, LINE_LEFT_S
} Line_Sensor_enum;

const struct {
    const byte LINE_RIGHT, LINE_LEFT;
} LINE = {LINE_RIGHT_S, LINE_LEFT_S};
//----------------------------LINE_FUNCTION

void setLineLed(byte state) {
    PORTDbits.RD1 = (state == HIGH); //PORT IR LED
}

byte getLineState(byte sensor) {
    switch (sensor) {
        case LINE_RIGHT_S:
            return PORTAbits.RA2;
        case LINE_LEFT_S:
            return PORTAbits.RA5;
    }
}
byte checkLine(byte line) {
    byte res = 0;
    switch (line) {
        case LINE_RIGHT_S:
            OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH2 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_3ANA); //analogCH2
            break;
        case LINE_LEFT_S:
            OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH4 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_5ANA); //analogCH4
            break;
    }

    setLineLed(HIGH); //Turning on IR line led

    delayInMicros(900); //Waiting 

    ConvertADC();
    while (BusyADC());

    res = ADRESH;
    setLineLed(LOW);
    CloseADC();
    return (res);
}

byte checkLineWithOffsetD(byte line,byte offset){
    return (checkLine(line) > offset ?1:0);
}

byte checkLineD(byte line) {

    //Variables
    byte res = 0;


    WDTCONbits.ADSHR = 1;
    ANCON0 = 0b11111111;
    ANCON1 = 0b11111111;
    WDTCONbits.ADSHR = 0;

    setLineLed(HIGH);
    delayInMicros(900); //Delay of 900 us

    res = getLineState(line);

    setLineLed(LOW);

    return res;
}

void (*LINECallback)(byte sensor);

void setLineSensorCallback(void (*Callback)(byte sensor)) {
    LINECallback = Callback;
}

void lineRoutine() {
    byte i = 0;
    if (&LINECallback == 0x00)
        return;
    for (i = 0; i < 2; i++)if (checkLineD(i) != 0)(*LINECallback)(i);
}

//----------------------------SENSOR_EASY_FUNCTION
//copy of official function SEN_LIGHT(void)

byte getLightLVL() {
    byte lightRes = 0;
    //Select channel 0ADC  
    OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_1ANA);
    ConvertADC();
    while (BusyADC());
    lightRes = ADRESH / 2;
    CloseADC();
    return (lightRes);
}
//copy of official function SEN_TEMPERATURE (void)

byte getTempC(void) {
    //Variables
    int SEN_TEMPERATURE_C = 0;

    //The ADC channel 13 is selected 
    OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH13 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_14ANA);
    ConvertADC();
    while (BusyADC()); //It is necessary to wait until the conversion is completely done

    //Reading and calculation of temperature
    SEN_TEMPERATURE_C = ADRESH;

    //It checks if the value is ok and it calculates the percentage
    SEN_TEMPERATURE_C = (SEN_TEMPERATURE_C - 69);
    if (SEN_TEMPERATURE_C > 0) {
        SEN_TEMPERATURE_C = (SEN_TEMPERATURE_C * 10) / 23;
    } else {
        SEN_TEMPERATURE_C = 0;
    }

    //A/D conversion is disabled
    CloseADC();

    //It returns the battery level
    return (SEN_TEMPERATURE_C);

}
//copy of official function SEN_BATTERY(void)
#define   MIN_BAT_LVL 132			//Mínimun of battery equal to 3.3 V
#define   MAX_BAT_LVL 162			//Máximun of battery equal to 4.2 V
#define   ADC_BAT_PRECISION   3

byte getBatteryLVL() {
    //Variables
    byte batPer = 0;
    OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH14 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_15ANA);
    ConvertADC();
    while (BusyADC());
    batPer = ADRESH;

    if (batPer < MAX_BAT_LVL) {
        if (batPer > MIN_BAT_LVL) {
            batPer = (batPer - MIN_BAT_LVL) * ADC_BAT_PRECISION;
        } else {
            batPer = 0;
        }
    } else {
        batPer = 100;
    }
    CloseADC();
    return (batPer);
}

byte getMicVal() {
    byte micValue = 0;
    OpenADC(ADC_FOSC_4 & ADC_LEFT_JUST & ADC_0_TAD, ADC_CH15 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, ADC_15ANA);
    ConvertADC();
    while (BusyADC());
    micValue = ADRESH;
    CloseADC();
    return (micValue);
}

byte getMicValD() {
    WDTCONbits.ADSHR = 1;
    ANCON0 = 0b11111111;
    ANCON1 = 0b11111111;
    WDTCONbits.ADSHR = 0;
    return PORTHbits.RH7; //MIC_PORT
}
// ACCELEROMETRE
#define   ACCE_TAP_SENSI		   			0b01110000			
#define   ACCE_TAP_WINDOW		   			0b01111111			
#define   ACCE_TAP_LAT		   				0b00000100			

const enum {
    SDO_S = 30, //MOSI
    CLOCK_S,
    CSN_S
} SPI_enum;

const struct {
    const byte SDO, CLOCK, CSN;
} SPI = {SDO_S, CLOCK_S, CSN_S};

void writeSpiState(byte spiPin, byte state) {
    state = (state == HIGH);
    switch (spiPin) {
        case SDO_S:
            PORTDbits.RD4 = state;
            break;
        case CLOCK_S:
            PORTDbits.RD6 = state;
            break;
        case CSN_S:
            PORTDbits.RD7 = state;
            break;
    }
}

const enum {
    X_S = 40, Y_S, Z_S
} AXIS_enum;

const struct {
    const byte X, Y, Z;
} AXIS = {X_S, Y_S, Z_S};
#define   ACCE_INT1_DRDY	PORTBbits.RB1		   	
#define   ACCE_INT2         PORTBbits.RB2		   	

byte getAxisValue(byte ACCE_TO_CHECK) {
    //Variables
    byte SENSOR = 0;
    byte ACCE_ADRESS = 0;
    byte ACCE_DATA_OUT = 0;
    byte ACCE_NO_HIGH = 255;
    byte ACCE_NO_LOW = 255;
    byte ACCE_DATA_IN = 0;

    //The accelerometer is turn on "measure" mode
    ACCE_ADRESS = MCTL;
    ACCE_DATA_OUT = 0b00000101;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //It checks if the axes are calculated on time
    while (ACCE_NO_HIGH >= 0) {
        ACCE_NO_LOW--;
        while (ACCE_NO_LOW >= 0) {
            if (ACCE_INT1_DRDY == 1) {
                break;
            } else {
                ACCE_NO_LOW--;
            }
        }
        if (ACCE_INT1_DRDY == 1) {
            break;
        }
        ACCE_NO_HIGH--;
        if (ACCE_NO_HIGH == 0) {
            //The accelerometer goes back to "pulse detection" mode
            ACCE_ADRESS = MCTL;
            ACCE_DATA_OUT = 0b01000011;
            ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

            return (0);

        } else {
            ACCE_NO_LOW = 255;
        }
    }

    //It selects the axes to be calculated
    switch (ACCE_TO_CHECK) {
        case X_S:

            //X reading 
            ACCE_ADRESS = XOUT8;
            ACCE_DATA_IN = ACCE_READ(ACCE_ADRESS);
            SENSOR = ACCE_DATA_IN ^ 0b10000000;

            break;

        case Y_S:

            //Y reading
            ACCE_ADRESS = YOUT8;
            ACCE_DATA_IN = ACCE_READ(ACCE_ADRESS);
            SENSOR = ACCE_DATA_IN ^ 0b10000000;

            break;

        case Z_S:

            //Z reading
            ACCE_ADRESS = ZOUT8;
            ACCE_DATA_IN = ACCE_READ(ACCE_ADRESS);
            SENSOR = ACCE_DATA_IN ^ 0b10000000;

            break;
    }


    //The accelerometer goes back to "pulse detection" mode
    ACCE_ADRESS = MCTL;
    ACCE_DATA_OUT = 0b01000011;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    return (SENSOR);

}
byte checkIsTap();

byte checkIsTap() {
    //Variables
    byte ACCE_ADRESS = 0;
    byte SEN_ACCE_TAP = 0;
    byte ACCE_DATA_OUT = 0;
    byte ACCE_DATA_IN = 0;

    //Reading of the interruptions register
    ACCE_ADRESS = DETSRC;
    ACCE_DATA_IN = ACCE_READ(ACCE_ADRESS);
    SEN_ACCE_TAP = ACCE_DATA_IN & 0b00000011;

    //It checks if it is a Tap or a Tap tap
    if (SEN_ACCE_TAP == 0b00000011 || SEN_ACCE_TAP == 0b00000010) {
        SEN_ACCE_TAP = SEN_ACCE_TAP & 0b00000010;
    }
    //Interruptions cleaning
    ACCE_ADRESS = INTRST;
    ACCE_DATA_OUT = 0b00000011;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //Interruptions cleaning
    ACCE_ADRESS = INTRST;
    ACCE_DATA_OUT = 0b00000000;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);
    delay(250);
    //It return the value: 0 if it is a tap_tap and 1 if it is an only tap
    return (SEN_ACCE_TAP);
}

void (*onTapCallback)(byte state);

void setOnTapCallback(void (*onTap)(byte state)) {
    onTapCallback = onTap;
}

void onTapRoutine() {
    byte i = 0;
    if (&onTapCallback == 0x00)
        return;
    if (checkIsTap() != 0)
        (*onTapCallback)(HIGH == 0);
}

//---------------------
//Copy from the official leb_sen_moway.h SEN_CONFIG
//init_core(), put all at init state
//---------------------
void init_core();

void init_core() {
    //Variables
    byte ACCE_ADRESS = 0;
    byte ACCE_DATA_OUT = 0;

    //Disable vref circuit
    CVRCON = 0b00000000;

    //Disable parallel port
    PMCONH = 0b00000000;

    //Disable the comparator
    CM1CON = 0b00000000;
    CM2CON = 0b00000000;

    //ADC configuration
    WDTCONbits.ADSHR = 0;
    ADCON0 = 0b00000000;
    ADCON1 = 0b00100011;

    WDTCONbits.ADSHR = 1;
    ANCON0 = 0b11111111;
    ANCON1 = 0b11111111;
    WDTCONbits.ADSHR = 0;

    //Disable PLL
    OSCTUNEbits.PLLEN = 0;
    //***********TRIS_CONFIG************//

    //*****TRIS_OUTPUT*****//
    TRISBbits.TRISB5 = 0; //LED_RED
    TRISBbits.TRISB6 = 0; //LED_GREEN
    TRISCbits.TRISC7 = 0; //LED_FRONT
    TRISEbits.TRISE5 = 0; //LED_BRAKE
    TRISBbits.TRISB3 = 0; //SPEAKER

    TRISDbits.TRISD1 = 0; //LINE_TX_TRIS

    TRISJbits.TRISJ6 = 0; //IR_LED_TX
    TRISJbits.TRISJ7 = 0; //IR_LED --?

    //*****TRIS_INPUT*****//
    TRISFbits.TRISF6 = 1; //IR_CENTER_RIGHT
    TRISFbits.TRISF5 = 1; //IR_SIDE_RIGHT
    TRISAbits.TRISA1 = 1; //IR_CENTER_LEFT
    TRISAbits.TRISA3 = 1; //IR_SIDE_LEFT
    TRISAbits.TRISA0 = 1; //LIGHT_SENSOR
    TRISHbits.TRISH6 = 1; //BAT_SENSOR
    TRISHbits.TRISH5 = 1; //TEMP_SENSOR
    TRISHbits.TRISH7 = 1; //MIC_SENSOR
    TRISAbits.TRISA2 = 1; //LINE_RX_RIGHT
    TRISAbits.TRISA5 = 1; //LINE_RX_LEFT

    //SPI configuration
    //I/O Configuration
    //----------TRIS_CONFIG----------//
    TRISDbits.TRISD5 = 1; //SPI SD INPUT PIN (MISO)
    TRISDbits.TRISD4 = 0; //SPI SD OUTPUT PIN (MOSI)
    TRISDbits.TRISD6 = 0; //SPI CLOCK OUTPUT PIN
    TRISDbits.TRISD7 = 0; //SPI CSN OUTPUT PIN

    TRISBbits.TRISB1 = 1; //ACCE_INT1_DRDY_TRIS = 1;??
    TRISBbits.TRISB2 = 1; //ACCE_INT2_TRIS = 1;
    PMCONH = 0b00000000;

    //Turning on ACCE CSN 
    PORTDbits.RD7 = 1;

    //SPI module configuration
    SSP2STAT = 0b11000000;
    SSP2CON2 = 0b00000000;
    SSP2CON1 = 0b00100000;

    //Accelerometer configuration for the pulse detection
    ACCE_ADRESS = MCTL;
    ACCE_DATA_OUT = 0b01000011; //b'01000011';
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //Configuratio for TAP
    //Sense configuration
    ACCE_ADRESS = PDTH;
    ACCE_DATA_OUT = ACCE_TAP_SENSI;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //Pulse window
    ACCE_ADRESS = PW;
    ACCE_DATA_OUT = ACCE_TAP_WINDOW;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //Latency with the second pulse
    ACCE_ADRESS = LT;
    ACCE_DATA_OUT = ACCE_TAP_LAT;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //Second pulse window
    ACCE_ADRESS = TW;
    ACCE_DATA_OUT = ACCE_TAP_WINDOW;
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);

    //Interruptions assignment
    ACCE_ADRESS = CTL1;
    ACCE_DATA_OUT = 0b00000100; //b'00000100';
    ACCE_WRITE(ACCE_ADRESS, ACCE_DATA_OUT);
    MOT_CONFIG();
    

}
//*****************************************************
//*              ACCE_WRITE(dato3)			          *
//*****************************************************
//*		Description:Reading condition to 			  *
//* 	make possible the SPI communication.     	  *
//*****************************************************

void ACCE_WRITE(unsigned char data1, unsigned char data2) {

    static unsigned char SEN_ACCE_BYTE_OUT_1;
    static unsigned char SEN_ACCE_BYTE_OUT_2;
    static unsigned char SEN_ACCE_BYTE_IN_2;
    static unsigned char SEN_ACCE_BYTE_IN_1;

    TRISDbits.TRISD7 = 0; //FORCE CSN as output
    writeSpiState(SPI.CSN, LOW);
    delayInMicros(900);
    SEN_ACCE_BYTE_OUT_1 = (data1 << 1);
    SEN_ACCE_BYTE_OUT_1 = SEN_ACCE_BYTE_OUT_1 | 0b10000000;
    SEN_ACCE_BYTE_OUT_2 = data2;
    SSP2BUF = SEN_ACCE_BYTE_OUT_1; //Load send data
    while (SSP2STATbits.BF != 1) {
    }; //wait

    SEN_ACCE_BYTE_IN_1 = SSP2BUF; //Store received data

    SSP2BUF = SEN_ACCE_BYTE_OUT_2; //Load send data
    while (SSP2STATbits.BF != 1) {
    }; //wait

    SEN_ACCE_BYTE_IN_2 = SSP2BUF;
    writeSpiState(SPI.CSN, HIGH);
    
}
//*****************************************************


//*****************************************************
//*              ACCE_READ(dato3)			          *
//*****************************************************
//*		Description:Reading condition to 			  *
//* 	make possible the SPI communication.     	  *
//*****************************************************

unsigned char ACCE_READ(unsigned char data) {
    static unsigned char SEN_ACCE_BYTE_OUT_1;
    static unsigned char SEN_ACCE_BYTE_OUT_2;
    static unsigned char SEN_ACCE_BYTE_IN_2;
    static unsigned char SEN_ACCE_BYTE_IN_1;
    TRISDbits.TRISD7 = 0; //FORCE CSN as output
    writeSpiState(SPI.CSN, LOW);
    delayInMicros(900); //Delay_900us
    SEN_ACCE_BYTE_OUT_1 = (data << 1);
    SEN_ACCE_BYTE_OUT_1 = SEN_ACCE_BYTE_OUT_1 & 0b01111111;
    SEN_ACCE_BYTE_OUT_2 = 0b00000000;
    SSP2BUF = SEN_ACCE_BYTE_OUT_1; //Load send data
    while (SSP2STATbits.BF != 1) {
    }; //wait

    SEN_ACCE_BYTE_IN_1 = SSP2BUF; //Store received data

    SSP2BUF = SEN_ACCE_BYTE_OUT_2; //Load send data
    while (SSP2STATbits.BF != 1) {
    }; //wait

    SEN_ACCE_BYTE_IN_2 = SSP2BUF;
    writeSpiState(SPI.CSN, HIGH);
    return (SEN_ACCE_BYTE_IN_2);
}
//#####################################################################SENSOR_END#########################################################


//#####################################################################BRAIN_START########################################################
#define abs(x) ((x) > 0 ? (x) : -(x))
unsigned int getMotMM();

unsigned int getMotMM() {
    byte *dist_ptr;
    dist_ptr = MOT_FDBCK(STATUS_KM);
    return ((*(dist_ptr + 1)) << 8) | (*(dist_ptr + 0));
}

byte getMotAngle(int angle) {
    return (byte)((abs(angle)*3.3f) / 12);
}
// speed in mm/s

unsigned int getSpeed() {
    unsigned int mm1, mm2;
    mm1 = getMotMM();
    delay(100);
    mm2 = getMotMM();
    return (mm2 - mm1)*10;
}
const enum {
    CENTER_S = 50, LEFT_WHEEL_S, RIGHT_WHEEL_S
} ROTPOINT_enum;
const struct {
    const byte CENTER, LEFT_WHEEL, RIGHT_WHEEL;
} ROTPOINT = {CENTER_S, LEFT_WHEEL_S, RIGHT_WHEEL_S};

//VELOCITY 0-100
//ANGLE
//rotationPoint ROTPOINT

byte rotateA(byte velocity, int angle, byte rotationPoint) {
    byte returnCode = 0;
    if (angle != 360 && angle != -360)
        angle %= 360;
    if (rotationPoint == ROTPOINT.CENTER)
        returnCode = MOT_ROT(velocity, FWD, Center, ((angle < 0) ? LEFT : RIGHT), ANGLE, getMotAngle(angle));
    if (rotationPoint == ROTPOINT.LEFT_WHEEL)
        returnCode = MOT_ROT(velocity, ((angle < 0) ? FWD : BACK), WHEEL, LEFT, ANGLE, getMotAngle(angle));
    if (rotationPoint == ROTPOINT.RIGHT_WHEEL)
        returnCode = MOT_ROT(velocity, ((angle < 0) ? BACK : FWD), WHEEL, RIGHT, ANGLE, getMotAngle(angle));
    return returnCode;
}

//Time in millis, min 100ms max 25500ms(25,5s)

byte rotateT(char velocity, unsigned int timeInMillis, byte rotationPoint) {
    if (abs(velocity) > 100)
        velocity = (velocity < 0 ? -100 : 100);
    timeInMillis = (timeInMillis > 25500 && timeInMillis < 100) ? (timeInMillis > 25500 ? 25500 : 100) : timeInMillis;
    return MOT_ROT(abs(velocity), velocity < 0 ? BACK : FWD, rotationPoint == ROTPOINT.CENTER ? Center : WHEEL, rotationPoint == ROTPOINT.LEFT_WHEEL ? LEFT : RIGHT, TIME, timeInMillis / 100);
}

byte moveD(char velocity, byte distanceInCM) {
    if (abs(velocity) > 100)
        velocity = (velocity < 0 ? -100 : 100);
    return MOT_STR(abs(velocity), velocity < 0 ? BACK : FWD, DISTANCE, distanceInCM);
}
//Time in millis, min 100ms max 25500ms(25,5s)

byte moveT(char velocity, byte timeInMillis) {
    if (abs(velocity) > 100)
        velocity = (velocity < 0 ? -100 : 100);
    timeInMillis = (timeInMillis > 25500 && timeInMillis < 100) ? (timeInMillis > 25500 ? 25500 : 100) : timeInMillis;
    return MOT_STR(abs(velocity), velocity < 0 ? BACK : FWD, TIME, timeInMillis);
}

//SUIS LE COTE GAUCHE D'UNE LIGNE NOIR
byte trackWhiteLine(byte topLedDebug) {
    byte bit_state = ((!checkLineD(LINE.LINE_LEFT)) << 1) | (!checkLineD(LINE.LINE_RIGHT));
    if (topLedDebug) {
        writeLedState(LED.LED_TOP_GREEN, LOW);
        writeLedState(LED.LED_TOP_RED, LOW);
    }
    switch (bit_state) {
        case 0x00:
            if (topLedDebug)
                writeLedState(LED.LED_TOP_RED, HIGH);
            rotateT(50, 25500, ROTPOINT.RIGHT_WHEEL); //RIGHT
            return  0;
        case 0x01:
            if (topLedDebug)
                writeLedState(LED.LED_TOP_GREEN, HIGH);
            moveT(50, 25500);
            return  1;
        case 0x02:
            if (topLedDebug)
                writeLedState(LED.LED_TOP_RED, HIGH);
            rotateT(50, 25500, ROTPOINT.LEFT_WHEEL); //LEFT
            return 0;
        case 0x03:
            if (topLedDebug)
                writeLedState(LED.LED_TOP_RED, HIGH);
            rotateT(50, 25500, ROTPOINT.LEFT_WHEEL); //LEFT
            return 0;
    }
}
byte getIrSensorState();

byte getIrSensorState() {
    return checkObstacleD(IR.SIDE_LEFT) << 3 | checkObstacleD(IR.CENTER_LEFT) << 2 | checkObstacleD(IR.CENTER_RIGHT) << 1 | checkObstacleD(IR.SIDE_RIGHT);
}
byte getIrSensorStateOffset(byte offset) {
    return checkObstacleWithOffsetD(IR.SIDE_LEFT,offset) << 3 | checkObstacleWithOffsetD(IR.CENTER_LEFT,offset) << 2 | checkObstacleWithOffsetD(IR.CENTER_RIGHT,offset) << 1 | checkObstacleWithOffsetD(IR.SIDE_RIGHT,offset);
}

byte checkAngles();

byte checkAngles() {
    byte state_left, state_right;
    rotateA(50, -45, ROTPOINT.CENTER);
    delay(20);
    state_left = getIrSensorState();
    rotateA(50, 90, ROTPOINT.CENTER);
    delay(20);
    state_right = getIrSensorState();
    rotateA(50, -45, ROTPOINT.CENTER);
    return state_left << 4 | state_right;
}

byte avoidObjects(byte topLedDebug,byte preferedTurn) {
    byte checkAngleReturn, bit_state = getIrSensorStateOffset(254);
    if (topLedDebug) {
        writeLedState(LED.LED_TOP_GREEN, LOW);
        writeLedState(LED.LED_TOP_RED, LOW);
    }
    if (bit_state) {
        if (topLedDebug)
            writeLedState(LED.LED_TOP_RED, HIGH);
        if (bit_state == 0x0F) {
            if (checkAngleReturn = checkAngles()) {
                switch (checkAngleReturn) {
                    case 0xff://Cul-de-sac
                        moveD(-50, 5);
                        break;
                    case 0xf0://angle à gauche donc tourner à droite
                        rotateA(50, 5, ROTPOINT.CENTER);
                        break;
                    case 0x0f://angle à droite donc tourner à gauche
                        rotateA(50, -5, ROTPOINT.CENTER);
                        break;
                }
            }
            return 1;
        }
        if (bit_state == 0x08) {//1000
            rotateA(50, 1, ROTPOINT.CENTER);
            return 1;
        }
        if (bit_state == 0x01) {//1000
            rotateA(50, -1, ROTPOINT.CENTER);
            return 1;
        }
        rotateA(50,preferedTurn == ROTPOINT.LEFT_WHEEL? -1:1, ROTPOINT.CENTER);
    }
    else{
        if (topLedDebug)
            writeLedState(LED.LED_TOP_GREEN, HIGH);
        moveT(80,25000);
    }
    return 0;
}
void stop(){
    MOT_STOP();
}

#endif
