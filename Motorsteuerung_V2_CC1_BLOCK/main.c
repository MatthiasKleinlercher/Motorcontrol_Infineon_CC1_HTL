// Green U
// Blue V
// Yellow W am Kondensator
//
// Rot
// Orange (an Blau)
// Grün (an Gelb)
// Gelb (an Braun)
// Schwarz

/*******************************************************************************
 * Header Files
 *******************************************************************************/
#include "cy_cordic.h"
#include "cy_device.h"
#include "cy_dma.h"
#include "cy_hppass_ac.h"
#include "cy_hppass_sar.h"
#include "cy_sysclk.h"
#include "cy_sysint.h"
#include "cy_syslib.h"
#include "cy_tcpwm.h"
#include "cy_tcpwm_counter.h"
#include "cy_tcpwm_pwm.h"
#include "cy_utils.h"
#include "cycfg_clocks.h"
#include "cycfg_peripherals.h"
#include "cycfg_pins.h"
#include "cyip_flashc.h"
#include "gpio_psc3_e_lqfp_80.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "psc3m5fds2afq1_s.h"
#include <stdint.h>
#include <stdio.h>
#include <cy_retarget_io.h>
#include <math.h>
#include <string.h>
#include "cy_gpio.h"
#include "cy_hppass.h"
#include "cy_cordic.h"
#include "cy_pdl.h"
#include <stdbool.h>

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define LUT_SIZE                            6
#define SET_VALUE_COUNTER_ISR_FREQ_HZ       10000
#define DIVIDER                             1220

#define MAX_ADC_MEASURMENT_VALUES_PER_PERIOD 2040

#define SYSTEMCFREQUENCY                    240000000

#define HIGH                                1
#define LOW                                -1
#define OFF                                 0

#define U                                   0
#define V                                   1
#define W                                   2

#define POOLPAARZAHL                        4
#define MAX_SAMPLES_PER_PHASE               64

#define sqrt3                               1.732050808f
#define TWO_M_PI                            6.283185307f
#define ONE_M_PI_THIRD                      1.04719755f
#define TWO_M_PI_THIRD                      2.094395102f
#define FOUR_M_PI_THIRD                     4.188790205f
#define sqrt3o2                             0.8660254038f
#define k2o3                                0.666666667f
#define Vdc                                 36.0f
#define ONE_o_SQRT3                         0.57735026919f
#define M_PIo30                             0.1047197551197f
#define PI									3.1415926536
		
#define C30                                 0.994521895f
#define S30                                 0.104528463f

#define TWO_PIo_1250                        0.0050265f

/* Injection Timing @ 25 kHz (1 Periode = 40us) */
#define INJ_ON_CYCLES_COARSE                5      /* 200 us on */
#define INJ_OFF_CYCLES_COARSE_HIZ           6      /* 240..320 us off if High-Z; increase if needed */
#define INJ_OFF_CYCLES_COARSE_LS            12     /* 480 us off if Low-Side-ON */

#define INJ_ON_CYCLES_FINE                  5
#define INJ_OFF_CYCLES_FINE_HIZ             6
#define INJ_OFF_CYCLES_FINE_LS              12

#define INJ_ON_CYCLES_POL                   10
#define INJ_OFF_CYCLES_POL_HIZ              8
#define INJ_OFF_CYCLES_POL_LS               15

#define M_COARSE                            0.15f
#define M_FINE                              0.35f
#define M_POL                               0.60f

#define OMEGA_LOCK_MIN     400.0f      // ab ~64 el. Hz handover zulässig
#define OBS_READY_COUNT    100         // 100 * 0.2ms = 20 ms stabil

/*******************************************************************************
 * Globals
 *******************************************************************************/

uint16_t  lut_index        = 0;
float_t Duty             = 0.0f;
float_t w_e			   = 0.0;
float_t Idc			   = 0.0;
float_t Idc_filt = 0.0;
bool w_e_hall_counter_running = false;

const uint8_t hall_to_lut[8] =
{
    255, // 000
    5,   // 001
    3,   // 010
    4,   // 011
    1,   // 100
    0,   // 101
    2,   // 110
    255  // 111
};

int16_t lut[LUT_SIZE][3] =
{
	{ HIGH, LOW,  OFF },   // Zustand 1: U+ V- W offen
    { HIGH, OFF, LOW },    // Zustand 2: U+ W- V offen
    { OFF, HIGH, LOW },    // Zustand 3: V+ W- U offen
    { LOW, HIGH, OFF },    // Zustand 4: V+ U- W offen
    { LOW, OFF, HIGH },    // Zustand 5: W+ U- V offen
    { OFF, LOW, HIGH }     // Zustand 6: W+ V- U offen
};

int16_t cclut[LUT_SIZE][3] = 
{
	{HIGH, OFF, LOW},
	{OFF, HIGH, LOW},
	{LOW, HIGH, OFF},
	{LOW, OFF, HIGH},
	{OFF, LOW, HIGH},
	{HIGH, LOW,  OFF}
};


int16_t adc_sar_result[16] = {0,0,0};

uint16_t adc_current_result[16];
uint8_t  chanIdx0 = 0;
uint8_t  chanIdx1 = 0;
uint8_t  chanIdx2 = 0;

float32_t currentU = 0.0f;
float32_t currentV = 0.0f;
float32_t currentW = 0.0f;

float32_t voltageU = 0.0f;
float32_t voltageV = 0.0f;
float32_t voltageW = 0.0f;

float32_t Duty_U = 0.0f, Duty_V = 0.0f, Duty_W = 0.0f;

uint8_t button_pressed_before = 0U;

static uint32_t samples_for_offset = 0;
static uint32_t offset_acc_u = 0, offset_acc_v = 0, offset_acc_w = 0;
static uint32_t adc_off_u = 2048, adc_off_v = 2048, adc_off_w = 2048;

/* PI Controller */
typedef struct { float32_t kp, ki, integrator, prevI; } pi_t;
typedef struct {
    pi_t d, q;
    float32_t psi_f;
    float32_t Ts;
} foc_curr_ctrl_t;

static foc_curr_ctrl_t Ictrl =
{
    .d = {.kp = 1.35f, .ki = 550.0f, .integrator = 0.0, .prevI = 0.0},
    .q = {.kp = 1.35f, .ki = 550.0f, .integrator = 0.0, .prevI = 0.0},
    .psi_f = 0.06f,
    .Ts = 0.00004f
};

/* Motorparameter */
static const float32_t Rs = 0.035f;        // Ohm
static const float32_t Ld = 86e-6f;        // H
static const float32_t Lq = 86e-6f;        // H
static const float32_t Ls = 86e-6f;        // H

typedef enum {BLOCK_ADC_CALIBRATION, BLOCK_READY_TO_START, BLOCK_RUNNING, BLOCK_OFF } block_state_t;
volatile block_state_t block_state = BLOCK_OFF;

uint32_t period_PWM = 0;
uint32_t prevCountValueSpeMeaCou = 0;

static float32_t theta_e_rad     = 0.0f;     // elektrischer Winkel [rad]

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void init(void);
static inline float32_t adc_to_amp(uint16_t adc, uint16_t adc_off, float32_t k);
void Poti_read(void);

/* ISRs */
void adc_group0_done_intr_handler(void);
void controller_counter_intr_handler(void);
void speed_measurment_intr_handler(void);
void position_estimate_intr_handler(void);
void pwm_reload_irq_handler(void);
void hall_interrupt_handler(void);

/*******************************************************************************
	SL FOC
 *******************************************************************************/
static inline float32_t _sat(float32_t x, float32_t min, float32_t max)
{
	if(x < min) return min;
	if(x > max) return max;
	return x;
}

static inline float32_t _wrap_0_2pi(float32_t a)
{
	while(a >= TWO_M_PI) a -= TWO_M_PI;
	while(a < 0.0f) a += TWO_M_PI;
	return a;
}

static inline float32_t wrap_pi(float32_t a)
{
    while (a >  PI)       a -= TWO_M_PI;
    while (a <= -PI)      a += TWO_M_PI;
    return a;
}

float32_t clampf(float32_t x, float32_t min, float32_t max)
{
	if (x > max) x = max;
	else if (x < min) x = min;
	return x;
}


/*******************************************************************************
 * Helpers & math
 *******************************************************************************/

static inline void outputs_all_off(void)
{      
    Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_U_HW, PWM_Counter_U_NUM, 0);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_V_HW, PWM_Counter_V_NUM, 0);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_W_HW, PWM_Counter_W_NUM, 0);
}

static inline void low_sides_all_on(void)
{
    outputs_all_off(); // Placeholder
}

/*******************************************************************************
 * ISRs
 *******************************************************************************/
cy_stc_sysint_t Controller_counter_intr_config = { .intrSrc = Controller_Counter_IRQ, .intrPriority = 4U };
cy_stc_sysint_t Speed_Measurment_intr_config   = { .intrSrc = Speed_Measurment_Counter_IRQ, .intrPriority = 5U };
cy_stc_sysint_t hall_isr3_config                = { .intrSrc = ioss_interrupts_sec_gpio_8_IRQn, .intrPriority = 2U };
cy_stc_sysint_t hall_isr12_config                = { .intrSrc = ioss_interrupts_sec_gpio_9_IRQn, .intrPriority = 2U };
cy_stc_sysint_t fifo_isr_cfg             	   = { .intrSrc = pass_interrupt_fifos_IRQn, .intrPriority = 3U };
cy_stc_sysint_t set_value_intr_config          = { .intrSrc = Test_Counter_IRQ, .intrPriority = 1U };


void controller_counter_intr_handler()
{
    uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Controller_Counter_HW, Controller_Counter_NUM);
	

    Cy_TCPWM_ClearInterrupt(Controller_Counter_HW, Controller_Counter_NUM, intrStatus);
}

void set_PWM(TCPWM_Type* hw, uint32_t num, uint8_t phase)
{
	uint32_t compare = 0;
	compare = (uint32_t)(Cy_TCPWM_PWM_GetPeriod0(hw, num) * Duty);
	
	switch(cclut[lut_index][phase])
		{
			case HIGH:
				Cy_TCPWM_PWM_Enable(hw, num);
				Cy_TCPWM_PWM_SetCompare0BufVal(hw, num, compare);
				Cy_TCPWM_TriggerCaptureOrSwap_Single(hw, num);
				break;
			
			case LOW:
				Cy_TCPWM_PWM_Enable(hw, num);
				Cy_TCPWM_PWM_SetCompare0BufVal(hw, num, 0);
				Cy_TCPWM_TriggerCaptureOrSwap_Single(hw, num);
				break;
			
			case OFF:
				Cy_TCPWM_PWM_Disable(hw, num);
				break;
				
		}
}

void set_value()
{
	uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Test_Counter_HW, Test_Counter_NUM);
	set_PWM(PWM_Counter_U_HW, PWM_Counter_U_NUM, U);
	set_PWM(PWM_Counter_V_HW, PWM_Counter_V_NUM, V);
	set_PWM(PWM_Counter_W_HW, PWM_Counter_W_NUM, W);
	Cy_TCPWM_ClearInterrupt(Test_Counter_HW, Test_Counter_NUM, intrStatus);
}


uint8_t read_hall_state(void)
{
    uint8_t H1 = Cy_GPIO_Read(HALL1_PORT, HALL1_PIN);
    uint8_t H2 = Cy_GPIO_Read(HALL2_PORT, HALL2_PIN);
    uint8_t H3 = Cy_GPIO_Read(HALL3_PORT, HALL3_PIN);
    return (H1 << 2) | (H2 << 1) | H3;
}

void hall_interrupt_handler(void)
{
	
    uint32_t intr_status1 = Cy_GPIO_GetInterruptStatusMasked(HALL1_PORT, HALL1_PIN);
    uint32_t intr_status2 = Cy_GPIO_GetInterruptStatusMasked(HALL2_PORT, HALL2_PIN);
    uint32_t intr_status3 = Cy_GPIO_GetInterruptStatusMasked(HALL3_PORT, HALL3_PIN);
	
    if (intr_status1 || intr_status2 || intr_status3)
    {
		uint8_t hall_state = read_hall_state();
		lut_index = hall_to_lut[hall_state];
		static uint32_t prevCount;
		if (CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING & Cy_TCPWM_Counter_GetStatus(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM))
		{
			uint8_t Divider = 100;
			uint32_t countValueSpeMeaCou = Cy_TCPWM_Counter_GetCounter(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
			float32_t freq = (float32_t)SYSTEMCFREQUENCY/(float32_t)Divider;
			w_e = TWO_M_PI * freq/ ((float32_t)(countValueSpeMeaCou-prevCount)* (float32_t)LUT_SIZE) * 4.0;
			prevCount = countValueSpeMeaCou;
		}
		else
		{
			prevCount = 0;
			Cy_TCPWM_PWM_Enable(Speed_Measurment_Counter_HW,Speed_Measurment_Counter_NUM);
			Cy_TCPWM_Counter_SetCounter(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM, 0U);
			Cy_TCPWM_TriggerStart_Single(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
		}
	}
    if (intr_status1) Cy_GPIO_ClearInterrupt(HALL1_PORT, HALL1_PIN);
    if (intr_status2) Cy_GPIO_ClearInterrupt(HALL2_PORT, HALL2_PIN);
    if (intr_status3) Cy_GPIO_ClearInterrupt(HALL3_PORT, HALL3_PIN);
    
    NVIC_ClearPendingIRQ(ioss_interrupts_sec_gpio_8_IRQn);
    NVIC_ClearPendingIRQ(ioss_interrupts_sec_gpio_9_IRQn);
}

/* --- ADC FIFO ISR --- */
void pass_0_sar_0_fifo_0_buffer_0_callback(void)
{
    uint32_t intrStatus = Cy_HPPASS_FIFO_GetInterruptStatusMasked();
    if (intrStatus & CY_HPPASS_INTR_FIFO_0_LEVEL) adc_group0_done_intr_handler();
    Cy_HPPASS_FIFO_ClearInterrupt(CY_HPPASS_INTR_FIFO_0_LEVEL);
}

static inline float32_t adc_to_amp(uint16_t adc, uint16_t adc_off, float32_t k)
{
    return k * ((float32_t)adc -(float32_t)adc_off);
}

void current_offset_calibrate_step(uint16_t au, uint16_t av, uint16_t aw)
{
    if (samples_for_offset == 0) 
    {
        Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_U_HW, PWM_Counter_U_NUM, 0);
        Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_V_HW, PWM_Counter_V_NUM, 0);
        Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_W_HW, PWM_Counter_W_NUM, 0);

        Cy_TCPWM_TriggerStart_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
        Cy_TCPWM_TriggerStart_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
        Cy_TCPWM_TriggerStart_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);
    }

    if (samples_for_offset < 2048) 
    {
        offset_acc_u += au; offset_acc_v += av; offset_acc_w += aw;
        samples_for_offset++;
        if (samples_for_offset == 2048) 
        {
            adc_off_u = (uint16_t)(offset_acc_u / samples_for_offset);
            adc_off_v = (uint16_t)(offset_acc_v / samples_for_offset);
            adc_off_w = (uint16_t)(offset_acc_w / samples_for_offset);
            block_state = BLOCK_READY_TO_START;
        }
    }
}

void adc_group0_done_intr_handler(void)
{
    uint8_t chanIdx;
    adc_sar_result[0] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[1] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[2] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);

    const float32_t k = -0.018f;
    currentU = adc_to_amp(adc_sar_result[0], adc_off_u, k);
    currentV = adc_to_amp(adc_sar_result[1], adc_off_v, k);
    currentW = adc_to_amp(adc_sar_result[2], adc_off_w, k);
    
    Idc = 0.5 *(currentU*lut[lut_index][0] + currentV*lut[lut_index][1]+ currentW*lut[lut_index][2]);
	
	if (block_state == BLOCK_ADC_CALIBRATION)
	{
	    if (samples_for_offset < 2048) 
	    {
	        current_offset_calibrate_step(adc_sar_result[0], adc_sar_result[1], adc_sar_result[2]);
	    }
	    else 
		{
			samples_for_offset = 0;
			block_state = BLOCK_READY_TO_START;
		}
	}
}

static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;

void Poti_read(void)
{
    int32_t raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(12U);
    Duty = (float32_t)raw_value_idPot / 4095.0f;
}


void init(void)
{
    cy_rslt_t result;

    /* UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    /* HPPASS/SAR */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_Init(&pass_0_config)) { CY_ASSERT(0); }
    Cy_HPPASS_AC_Start(0U, 100U);

    /* Position Estimate Counter */
    if (result != Cy_TCPWM_Counter_Init(Position_Estimate_Counter_HW, Position_Estimate_Counter_NUM, &Position_Estimate_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Position_Estimate_Counter_HW, Position_Estimate_Counter_NUM);
    //Cy_SysInt_Init(&position_estimate_intr_config, position_estimate_intr_handler);
    //NVIC_EnableIRQ(position_estimate_intr_config.intrSrc);

    /* Test Counter */
    if (result != Cy_TCPWM_Counter_Init(Test_Counter_HW, Test_Counter_NUM, &Test_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Test_Counter_HW, Test_Counter_NUM);
    Cy_SysInt_Init(&set_value_intr_config, set_value);

    /* Speed Counter */
    if(result != Cy_TCPWM_Counter_Init(Speed_Measurment_Counter_HW,Speed_Measurment_Counter_NUM, &Speed_Measurment_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
    //Cy_SysInt_Init(&Speed_Measurment_intr_config, speed_measurment_intr_handler);
    //NVIC_EnableIRQ(Speed_Measurment_intr_config.intrSrc);

    /* Measurment Counter */
    if (result != Cy_TCPWM_Counter_Init(Measurment_Counter_HW, Measurment_Counter_NUM, &Measurment_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Measurment_Counter_HW, Measurment_Counter_NUM);

    /* Controller Counter */
    if(result != Cy_TCPWM_Counter_Init(Controller_Counter_HW,Controller_Counter_NUM, &Controller_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Controller_Counter_HW, Controller_Counter_NUM);
    Cy_SysInt_Init(&Controller_counter_intr_config, controller_counter_intr_handler);
    NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);

    /* PWM U/V/W */
    if (result != Cy_TCPWM_PWM_Init(PWM_Counter_U_HW, PWM_Counter_U_NUM, &PWM_Counter_U_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_U_HW, PWM_Counter_U_NUM);

    if (result != Cy_TCPWM_PWM_Init(PWM_Counter_V_HW, PWM_Counter_V_NUM, &PWM_Counter_V_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_V_HW, PWM_Counter_V_NUM);

    if (result != Cy_TCPWM_PWM_Init(PWM_Counter_W_HW, PWM_Counter_W_NUM, &PWM_Counter_W_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
    
    if (result != Cy_TCPWM_PWM_Init(SevSw_Counter_HW, SevSw_Counter_NUM, &SevSw_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(SevSw_Counter_HW, SevSw_Counter_NUM);

    /* Halls & Button */
    Cy_GPIO_Pin_Init(HALL1_PORT, HALL1_PIN, &HALL1_config);
    Cy_GPIO_Pin_Init(HALL2_PORT, HALL2_PIN, &HALL2_config);
    Cy_GPIO_Pin_Init(HALL3_PORT, HALL3_PIN, &HALL3_config);
    Cy_GPIO_Pin_Init(SW1_PORT, SW1_NUM, &SW1_config);
    Cy_SysInt_Init(&hall_isr3_config, hall_interrupt_handler);
    NVIC_EnableIRQ(hall_isr3_config.intrSrc);
    Cy_SysInt_Init(&hall_isr12_config, hall_interrupt_handler);
    NVIC_EnableIRQ(hall_isr12_config.intrSrc);

    /* ADC FIFO ISR */
    Cy_HPPASS_FIFO_SetInterruptMask(CY_HPPASS_INTR_FIFO_0_LEVEL);
    Cy_SysInt_Init(&fifo_isr_cfg, pass_0_sar_0_fifo_0_buffer_0_callback);
    NVIC_EnableIRQ(fifo_isr_cfg.intrSrc);
}

/*******************************************************************************
 * Main
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    init();

    __enable_irq();

    /* PWM Perioden angleichen (optional) */
    period_PWM = Cy_TCPWM_Counter_GetPeriod(Test_Counter_HW, Test_Counter_NUM)/2;
    Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_U_HW, PWM_Counter_U_NUM, period_PWM);
    Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_V_HW, PWM_Counter_V_NUM, period_PWM);
    Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_W_HW, PWM_Counter_W_NUM, period_PWM);
    
    Cy_TCPWM_PWM_SetPeriod0(SevSw_Counter_HW, SevSw_Counter_NUM, period_PWM);
    Cy_TCPWM_PWM_SetCompare0Val(SevSw_Counter_HW, SevSw_Counter_NUM, period_PWM);
    Cy_TCPWM_TriggerStart_Single(SevSw_Counter_HW, SevSw_Counter_NUM);
	
    printf("Init done\r\n");

    for (;;)
    {
		if (block_state == BLOCK_READY_TO_START)
		{
			outputs_all_off();
		}
		

		if (Cy_GPIO_Read(SW1_PORT, SW1_NUM) == 1UL && button_pressed_before == false)
		{
			button_pressed_before = true;
			if (block_state == BLOCK_READY_TO_START) 
			{
				block_state = BLOCK_RUNNING;
				NVIC_EnableIRQ(set_value_intr_config.intrSrc);
				NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);
			}
			else if (block_state == BLOCK_OFF)
			{
				Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
				block_state = BLOCK_ADC_CALIBRATION;
			}
		}
		else if (Cy_GPIO_Read(SW1_PORT, SW1_NUM) == 0UL)
		{
			button_pressed_before = false;
		}
		
		Poti_read();
		
        /* UART Demo */
        char print_Array[9];
        float32_t data_Array[2] = {Idc, w_e};
        print_Array[0] = 0xAA;
        memcpy(&print_Array[1], data_Array, sizeof(data_Array));
        for (int i = 0; i < (int)sizeof(print_Array); i++) 
        { 
			printf("%c", print_Array[i]); 
		}

    }
}

/* [] END OF FILE */