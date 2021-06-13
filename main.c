#include "mcu_support_package/inc/stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

#define TIMER_PRESCALER 720
#define PERIOD 50000
#define PAUSE_TIMER_PRESCALER 72
#define PAUSE_PERIOD 1
#define PAUSE_US 36
#define MEASUREMENTS 100
#define FREQUENCY_MEASUREMENTS 3
#define MAX_FREQUENCY 1000
#define MIN_FREQUENCY 100

// =====================Global variables=====================

static int16_t adcOut[MEASUREMENTS] = {0};
static int16_t measurementsQuantity = 0;
static volatile bool risingEdge = true;
static volatile int16_t frequency = 0;                        // instant frequency
static int16_t frequencyRow[FREQUENCY_MEASUREMENTS] = {0};    // array of instant frequencies
static int8_t frequencyQuantity = 0;

// =====================Global functions=====================

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

// =====================Static functions=====================

static void pause(uint16_t time);



int main(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    
// =====================Ports settings=====================
    
    GPIO_InitTypeDef compTrigPort;
    GPIO_InitTypeDef dacOutPort;
    GPIO_InitTypeDef speakerPort;

// This port sends impulse to URM37 v4.0
    GPIO_StructInit(&compTrigPort);
    compTrigPort.GPIO_Mode = GPIO_Mode_Out_PP;
    compTrigPort.GPIO_Pin = GPIO_Pin_1;
    compTrigPort.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &compTrigPort);

// This port gets analog voltage from URM37 v4.0    
    GPIO_StructInit(&dacOutPort);
    dacOutPort.GPIO_Mode = GPIO_Mode_AIN;
    dacOutPort.GPIO_Pin = GPIO_Pin_4;
    dacOutPort.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &dacOutPort);
    
    GPIO_StructInit(&speakerPort);
    speakerPort.GPIO_Mode = GPIO_Mode_Out_PP;
    speakerPort.GPIO_Pin = GPIO_Pin_6;
    speakerPort.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &speakerPort);
    
// =====================ADC settings=====================

    ADC_InitTypeDef adc1;
    
    ADC_StructInit(&adc1);
    adc1.ADC_ContinuousConvMode = DISABLE;
    adc1.ADC_DataAlign = ADC_DataAlign_Right;
    adc1.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc1.ADC_Mode = ADC_Mode_Independent;
    adc1.ADC_NbrOfChannel = 1;
    adc1.ADC_ScanConvMode = DISABLE;
    ADC_Init(ADC1,&adc1);
    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_13Cycles5);

// =====================ADC calibration=====================

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1)) {;}
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1)) {;}

// =====================Timer settings to generate sounds=====================

    TIM_TimeBaseInitTypeDef timer3;

    __disable_irq();
        
    TIM_TimeBaseStructInit(&timer3);
    timer3.TIM_Prescaler = TIMER_PRESCALER - 1;
    timer3.TIM_Period = PERIOD - 1;
    timer3.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &timer3);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    TIM_Cmd(TIM3,ENABLE);
    
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
//    NVIC_EnableIRQ(TIM3_IRQn);
    
    __enable_irq();
        
// =====================Timer settings to pause=====================
   
   TIM_TimeBaseInitTypeDef timer2;

    __disable_irq();
        
    TIM_TimeBaseStructInit(&timer2);
    timer2.TIM_Prescaler = PAUSE_TIMER_PRESCALER - 1;
    timer2.TIM_Period = PAUSE_PERIOD - 1;
    timer2.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &timer2);
    TIM_Cmd(TIM2,ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    __enable_irq();
    
// =====================Main=====================
    
    while(1) {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
        pause(PAUSE_US);    
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
        pause(PAUSE_US);
        
        if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
            continue;
        }
        
        adcOut[measurementsQuantity] = ADC_GetConversionValue(ADC1);
        measurementsQuantity++;
 
        if(measurementsQuantity != MEASUREMENTS) {
            continue;
        }
        
        //enter if all measurements are made
        measurementsQuantity = 0;
        int32_t adcOutSum = 0;
                
        for(int16_t i = 0; i < MEASUREMENTS; i++) {
            adcOutSum += adcOut[i];
        }
                
        //output sound frequency calculation
        //approximating voltage change law is V = 5*x + 24, 
        //so to calculate a distance we can use x = (V - 24) / 5
        frequency = 10000 / (((adcOutSum / MEASUREMENTS) - 24) / 5); 
                
        if(frequency > MAX_FREQUENCY) {
            frequency = MAX_FREQUENCY;
        }
                
        // if frequency is larger than 100 we won't see an object
        if(frequency < MIN_FREQUENCY) {
            NVIC_DisableIRQ(TIM3_IRQn);
            TIM_Cmd(TIM3,DISABLE);
            TIM3->CNT = 0;
            continue;
        }
        
        //filling the array of instant frequencies
        frequencyRow[frequencyQuantity] = frequency;
        frequencyQuantity++;
        
        if(FREQUENCY_MEASUREMENTS != frequencyQuantity) {
            continue;
        }
        
        frequencyQuantity = 0;
        
        //if all instant frequencies are equal to each other output frequency will change
        //it creates a semblance of hysteresis   
        if(frequencyRow[0] != frequencyRow[1] || frequencyRow[0] != frequencyRow[2]) {
            continue;
        }
        
        __disable_irq();
        NVIC_DisableIRQ(TIM3_IRQn);
        TIM_Cmd(TIM3,DISABLE);
        TIM3->CNT = 0;
        TIM3->ARR = PERIOD;
        TIM3->ARR /= frequencyRow[0];
        TIM_Cmd(TIM3,ENABLE);
        __enable_irq();
        NVIC_EnableIRQ(TIM3_IRQn);       
    }

    return 0;
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if(risingEdge) {
            GPIO_SetBits(GPIOC, GPIO_Pin_6);
            risingEdge = false;
        } else {
            GPIO_ResetBits(GPIOC, GPIO_Pin_6);
            risingEdge = true;
        }
    }
}

// time is in microseconds
void pause(uint16_t time) {
    TIM_Cmd(TIM2,DISABLE);
    TIM2->CNT = 0;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM2->ARR = PAUSE_PERIOD * time;
    TIM_Cmd(TIM2,ENABLE);
    while(TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET) {
        __NOP();
    }
}





#ifdef USE_FULL_ASSERT

// эта функция вызывается, если assert_param обнаружил ошибку
void assert_failed(uint8_t * file, uint32_t line) { 
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     
    (void)file;
    (void)line;

    __disable_irq();
    while(1) {
        // это ассемблерная инструкция "отладчик, стой тут"
        // если вы попали сюда, значит вы ошиблись в параметрах вызова функции из SPL. 
        // Смотрите в call stack, чтобы найти ее
        __BKPT(0xAB);
    }
}

#endif
