/*---------------------------------------------------------
 *  SISTEMAS DE CONTROL 1 SECCION 20
 *  LABORATORIO 7
 *  ANGEL ORELLANA 19095
 *  JULIO LOPEZ    18211
 *
 *
 * ---------------------------------------------------------
 * */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line){}
#endif


#define FREQ_MUESTREO 5000

#define NUM_SPI_DATA    1  // Número de palabras que se envían cada vez
#define VALOR_MAX    4095  // Valor máximo del contador
#define SPI_FREC  4000000  // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16  // Número de bits que se envían cada vez, entre 4 y 16



// Estos valores fueron ajustados en vivo, los valores finales estan en el
// reporte
float KI = 245.878;
float KP = 4.28666;
float KD = 0.01868;
//volatile float v0, v1a;
volatile int32_t error=0, errorAnterior,errorAcu = 0;
volatile int32_t y;
volatile float vRef, vRet;
volatile int32_t yInt;
void timer0Handler(){


    uint32_t adcValues[2];
    uint32_t spiData;

    //limpiar las banderas de las interrupciones
    TimerIntClear(TIMER0_BASE , TIMER_TIMA_TIMEOUT);
    //inicial la conversion del adc
    ADCProcessorTrigger(ADC0_BASE, 2);
    while(!ADCIntStatus(ADC0_BASE, 2, false));
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, adcValues);
    //PID
    //vRef = adcValues[1]*3.3/4095;
    //vRet = adcValues[0]*3.3/4095;
    errorAnterior = error;
    error = (adcValues[1]- adcValues[0]);

    errorAcu += error;
    // no se trabaja en voltios ya que los voltajes de referencia y el adc son los mismos
    // por lo que para minimizar el procesamiento no se convierte a voltaje el valor ingresado
    y = KP*error + KI*errorAcu/FREQ_MUESTREO + KD*(error-errorAnterior)*FREQ_MUESTREO;
    /*yInt = y*4095/3.3;
    // salida al DAC
    if(yInt>4095) yInt = 4095;
    else if(yInt<0) yInt = 0;*/

    if(y>4095) y = 4095;
    else if(y<0) y = 0;

    spiData = (uint32_t)((0b0111 << 12) | (0x0FFF & y));

    SSIDataPut(SSI0_BASE, spiData);
    while(SSIBusy(SSI0_BASE));

}


int main(void){
    // Reloj a 80 MHz
   SysCtlClockSet( SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ
                        | SYSCTL_SYSDIV_2_5);


   //------------------ ADC -----------------------------
   SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0); //habilitar el adc0
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // se usara AIN0 y AIN1, que estan en PE3/PE2

   GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
   //configurar secuenciador
   ADCSequenceConfigure(ADC0_BASE, 2 , ADC_TRIGGER_PROCESSOR, 0);
   //paso 0 del sequencer 2
   ADCSequenceStepConfigure(ADC0_BASE, 2, 0 , ADC_CTL_CH0);
   //paso 1 del sequencer 2, aqui termina la secuencia
   ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
   //habilitar la secuencia
   ADCSequenceEnable(ADC0_BASE, 2);
   ADCIntClear(ADC0_BASE, 2); //limpiar la bandera de interrupcion

   IntMasterEnable();

   //------------------ TIMER --------------------------
   //habilitar modulo timer 0
   SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
   // TIMER 0 como periodico
   TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
   // tiempo con el cual se va a dar un overflow del timer
   TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/FREQ_MUESTREO));

   //-------------------- SPI ---------------------------
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
   GPIOPinConfigure(GPIO_PA2_SSI0CLK);
   GPIOPinConfigure(GPIO_PA3_SSI0FSS);
   GPIOPinConfigure(GPIO_PA4_SSI0RX);
   GPIOPinConfigure(GPIO_PA5_SSI0TX);
   GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                      GPIO_PIN_2);
   SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                          SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
   SSIEnable(SSI0_BASE);


   //------------------ INTERRUPCIONES ------------------
   IntMasterEnable();

   IntEnable(INT_TIMER0A);
   TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

   // --------------- START MODULES ----------------------
   TimerEnable(TIMER0_BASE, TIMER_A);

   while(1);

}
