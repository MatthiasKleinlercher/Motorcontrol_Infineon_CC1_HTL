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
#include "cy_tcpwm_pwm.h"
#include "cy_utils.h"
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

// include for TCPWM
#include "cy_tcpwm_counter.h"
#include "cy_tcpwm_pwm.h"
#include "cy_sysclk.h"
#include "cycfg_routing.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#include "cyhal.h"
#endif /* defined (CY_USING_HAL) */

#if defined (CY_USING_HAL_LITE)
#include "cyhal_hw_types.h"
#endif /* defined (CY_USING_HAL_LITE) */

#if defined (COMPONENT_MTB_HAL)
#include "mtb_hal.h"
#include "cycfg_clocks.h"
#include "mtb_hal_hw_types.h"
#endif /* defined (COMPONENT_MTB_HAL) */

//includes for DEBUG_UART
#include "cy_scb_uart.h"
#include "cy_sysclk.h"

#if defined (CY_USING_HAL)
#include "cyhal_hwmgr.h"
#include "cyhal.h"
#endif /* defined (CY_USING_HAL) */

#if defined (COMPONENT_MTB_HAL)
#include "mtb_hal.h"
#include "cycfg_clocks.h"
#include "mtb_hal_hw_types.h"
#endif /* defined (COMPONENT_MTB_HAL) */

#if defined (CY_USING_HAL_LITE)
#include "cyhal_hw_types.h"
#endif /* defined (CY_USING_HAL_LITE) */


/*******************************************************************************
* defines
*******************************************************************************/

// ADC - Channels
#define ADC_CH_IMS_U		0U
#define ADC_CH_IMS_V  	 	1U
#define ADC_CH_IMS_W   		2U
#define ADC_CH_IMS_DCBUS	3U
#define ADC_CH_VMS_U		4U
#define ADC_CH_VMS_V		5U
#define ADC_CH_VMS_W		6U
#define ADC_CH_VMS_DCBUS	7U

// ADC
#define VM_IU  (1u<<0)
#define VM_IV  (1u<<1)
#define VM_IW  (1u<<2)
#define VM_IDC (1u<<3)
#define VM_VU  (1u<<4)
#define VM_VV  (1u<<5)
#define VM_VW  (1u<<6)
#define VM_VDC (1u<<7)

// ADC calibartion type
typedef struct
{
    uint32_t target;        // e.g. 2048
    uint32_t warmup;        // e.g. 64

    volatile bool running;
    volatile bool done;

    uint32_t cnt_iu, cnt_iv, cnt_iw, cnt_idc;
    uint32_t cnt_vu, cnt_vv, cnt_vw, cnt_vdc;

    uint32_t wu_iu,  wu_iv,  wu_iw,  wu_idc;
    uint32_t wu_vu,  wu_vv,  wu_vw,  wu_vdc;

    int64_t acc_iu, acc_iv, acc_iw, acc_idc;
    int64_t acc_vu, acc_vv, acc_vw, acc_vdc;

    // result for offset at 0A / 0V
    int16_t off_iu, off_iv, off_iw, off_idc;
    int16_t off_vu, off_vv, off_vw;

    // Busvoltage can not be calibrated with 0V
    int16_t base_vdc_raw;

} adc_cal_ls_clamp_t;

// raw values (ISR/Frame)
typedef struct
{
    int16_t iu, iv, iw, idc;
    int16_t vu, vv, vw, vdc;
    uint8_t valid_mask; // check if channels were there
} adc_frame_t;

// Amps and Volts values
typedef struct
{
    float32_t iu, iv, iw, idc;   // A
    float32_t vu, vv, vw, vdc;   // V
} meas_si_t;

// Transformation struct
typedef struct { float32_t alpha, beta; } ab_t;
typedef struct { float32_t d, q; } dq_t;

// states of pre alignment
typedef enum {INJ_IDLE, INJ_NEXT, INJ_ON, INJ_OFF, INJ_DONE} inj_state_t;
typedef struct
{
    inj_state_t st;

    float32_t theta_list[16];
    float32_t peaks[16];
    uint8_t  theta_count;
    uint8_t  idx;

    float32_t inj_m;
    int32_t on_cycles;
    int32_t off_cycles;

    uint8_t stage;              // 0 coarse, 1 fine15, 2 fine7p5, 3 polarity
    float32_t theta_est_rad;    // Result (rad)
    bool theta_polar_is_north;
    
    float32_t theta_e_rad_final;
    bool theta_e_rad_final_ready;

} prealign_t;

// Params for pre alignment
#define M_COARSE              (0.1f)
#define M_FINE                (0.1f)
#define M_POL                 (0.1f)

#define INJ_ON_CYCLES_COARSE  (3)
#define INJ_OFF_CYCLES_COARSE (12)

#define INJ_ON_CYCLES_FINE    (4)
#define INJ_OFF_CYCLES_FINE   (16)

#define INJ_ON_CYCLES_POL     (8)
#define INJ_OFF_CYCLES_POL    (32)

// Values
#define TWO_PI          6.2831853072f
#define PI              3.1415926536f
#define TWO_o_THREE     0.6666666667f
#define SQRT3_o_2       0.8660254038f
#define ONE_o_SQRT3     0.5773502692f
#define ONE_o_THREE     0.3333333333f

// Block defines
// LUT_SIZE for Block
#define LUT_SIZE 6

// Values for High, Low and High Z
#define HIGH                                1
#define LOW                                -1
#define OFF                                 0

// Values for Phases 
#define A                                   0
#define B                                   1
#define C                                   2
// Test

/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/

// Finite State Machine
typedef enum {SYSTEM_OFF, SYSTEM_INIT, SYSTEM_CALIBRATION, SYSTEM_READY, SYSTEM_PRE_ALIGNMENT} system_state_t;
volatile system_state_t system_state = SYSTEM_OFF;

// Interruptconfig
cy_stc_sysint_t Controller_counter_intr_config = { .intrSrc = Controller_Counter_IRQ, .intrPriority = 4U };
cy_stc_sysint_t Speed_Measurment_intr_config   = { .intrSrc = Speed_Measurment_Counter_IRQ, .intrPriority = 5U };
cy_stc_sysint_t hall_isr3_config                = { .intrSrc = ioss_interrupts_sec_gpio_8_IRQn, .intrPriority = 2U };
cy_stc_sysint_t hall_isr12_config                = { .intrSrc = ioss_interrupts_sec_gpio_9_IRQn, .intrPriority = 2U };
cy_stc_sysint_t fifo_isr_cfg             	   = { .intrSrc = pass_interrupt_fifos_IRQn, .intrPriority = 3U };
cy_stc_sysint_t set_value_intr_config          = { .intrSrc = Test_Counter_IRQ, .intrPriority = 1U };
cy_stc_sysint_t pwm_reload_intr_config         = { .intrSrc = PWM_Counter_U_IRQ, .intrPriority =6U};

// DEBUG_UART
static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;

// ADC - Calibration
static adc_cal_ls_clamp_t adc_calibration;

// ADC - meas values with voltage and current data
static volatile  meas_si_t adc_meas;
static volatile bool adc_meas_valid = false;

// calculated voltages for transformation
static volatile ab_t invClarkeVoltage;
static volatile dq_t invParkVoltage;

// prealignment
static prealign_t preAlignment;
volatile bool inj_collect_now = false;     // gets true in ON
static volatile float32_t inj_theta_now = 0.0f;
volatile float32_t last_imag = 0.0f;       // ADC-ISR writes values in Peak
/*******************************************************************************
* Functions
*******************************************************************************/
void init();
void calibration_init();
void pass_0_sar_0_fifo_0_buffer_0_callback();
void pwm_reload_intr_handler();
void controller_counter_intr_handler();
void speed_measurment_intr_handler();
void adc_calibration_start_lowside_clamp(uint32_t target_samples, uint32_t warmup_samples);
static inline bool _cal_all_done(const adc_cal_ls_clamp_t *c);
void adc_calibration_feed_lowside_clamp(const int16_t *data, const uint8_t *chan, uint8_t n);
static inline int16_t _adc_apply_offset(int16_t raw, int16_t off);
void low_sides_all_on(void);
void high_sides_all_on(void);
static inline float32_t _adc_to_amp_i16(int16_t adc_raw, int16_t adc_off, float32_t k);
static inline float32_t _adc_to_volt_i16(int16_t adc_raw, int16_t adc_off, float32_t k, float32_t b);
static inline adc_frame_t _adc_unpack_frame(const int16_t *data, const uint8_t *chan, uint8_t n);
static inline bool _adc_frame_to_si(const adc_frame_t *f, meas_si_t *out);
static inline void _clarke_ab(float32_t ia, float32_t ib, float32_t ic, float32_t *alpha, float32_t *beta);
static inline void _park_dq(float32_t alpha, float32_t beta, float32_t s, float32_t c, float32_t *d, float32_t *q);
static inline void _inv_park_ab(float32_t d, float32_t q, float32_t s, float32_t c, float32_t *alpha, float32_t *beta);
static inline void _inv_clarke_abc(float32_t alpha, float32_t beta, float32_t *a, float32_t *b, float32_t *c);
static inline float32_t _wrap_pi(float32_t x);
static inline int32_t _argmax_f32(const float32_t *a, int n);
static inline void _apply_vector_rad(float32_t theta_rad, float32_t m);
static inline void _prepare_coarse(prealign_t *p);
static inline void _prepare_six_around(prealign_t *p, float32_t theta0, float32_t delta_rad);
static inline void _prepare_polarity(prealign_t *p, float32_t theta0);
static inline void _prealign_start(void);
static inline void _prealign_step_pwm(void);
static inline float32_t _DEG2RAD_F(float32_t x);
static inline void _svpwm(float32_t Va, float32_t Vb, float32_t Vc, float32_t overmod);
void set_PWM(TCPWM_Type* hw, uint32_t num, uint8_t phase, int16_t lut_array[LUT_SIZE][3],uint8_t lut_idx, float32_t duty);
inline float32_t _sat(float32_t x, float32_t min, float32_t max);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/

/*******************************************************************************
* Function Name: void low_sides_all_on(void)

* sets all three phases to Ground
*******************************************************************************/
void low_sides_all_on(void)
{
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_U_HW, PWM_Counter_U_NUM, 0U);
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_V_HW, PWM_Counter_V_NUM, 0U);
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_W_HW, PWM_Counter_W_NUM, 0U);
	
	Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
}

/******************************************************************************/

/*******************************************************************************
* Function Name: void high_sides_all_on(void)

* sets all three phases to VDC
*******************************************************************************/
void high_sides_all_on(void)
{
	uint32_t PWM_U_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	uint32_t PWM_V_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_V_HW, PWM_Counter_V_NUM);
	uint32_t PWM_W_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_W_HW, PWM_Counter_W_NUM);
	
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_U_HW, PWM_Counter_U_NUM, PWM_U_PeriodVal);
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_V_HW, PWM_Counter_V_NUM, PWM_V_PeriodVal);
	Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_W_HW, PWM_Counter_W_NUM, PWM_W_PeriodVal);
	
	Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _clarke_ab(float32_t ia, float32_t ib, float32_t ic, float32_t *alpha, float32_t *beta)

* Clarke Transformation from a,b,c to alpha, beta
*******************************************************************************/
static inline void _clarke_ab(float32_t ia, float32_t ib, float32_t ic, float32_t *alpha, float32_t *beta)
{
    // power-invariant Clarke (2/3 Skalierung)
    *alpha = TWO_o_THREE * (ia - 0.5*ib - 0.5*ic);
    *beta  = TWO_o_THREE * (SQRT3_o_2 * (ib - ic));
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _park_dq(float32_t alpha, float32_t beta, float32_t s, float32_t c, float32_t *d, float32_t *q)

* Park Transformation from alpha, beta to d, q
*******************************************************************************/
static inline void _park_dq(float32_t alpha, float32_t beta, float32_t s, float32_t c, float32_t *d, float32_t *q)
{
    *d = c*alpha + s*beta;
    *q = -s*alpha + c*beta;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _park_dq(float32_t alpha, float32_t beta, float32_t s, float32_t c, float32_t *d, float32_t *q)

* inverse Park Transformation from d, q to alpha, beta
*******************************************************************************/
static inline void _inv_park_ab(float32_t d, float32_t q, float32_t s, float32_t c, float32_t *alpha, float32_t *beta)
{
    *alpha = c*d - s*q;
    *beta  = s*d + c*q;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _inv_clarke_abc(float32_t alpha, float32_t beta, float32_t *a, float32_t *b, float32_t *c)

* inverse Park Transformation from alpha, beta to a, b, c
*******************************************************************************/
static inline void _inv_clarke_abc(float32_t alpha, float32_t beta, float32_t *a, float32_t *b, float32_t *c)
{
    *a = alpha;
    *b = -0.5*alpha + SQRT3_o_2*beta;
    *c = -0.5*alpha - SQRT3_o_2*beta;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _svpwm()

* -
*******************************************************************************/
/* TODO: Has to be documented */
static inline void _svpwm(float32_t Va, float32_t Vb, float32_t Vc, float32_t overmod)
{	
	float32_t Vmax = fmaxf(fmaxf(Va, Vb), Vc);
	float32_t Vmin = fminf(fminf(Va, Vb), Vc);
	float32_t Voff = -0.5*(Vmax+Vmin);
	
	Va += Voff; Vb += Voff; Vc += Voff;
	
	if (overmod > 0.0f)
	{
		float32_t scale = 1.0f + 0.15f * _sat(overmod,0.0f,1.0f);
		Va *= scale; Vb *= scale; Vc *= scale;
	}
	
	float32_t Vdc = adc_meas.vdc;
	
	float32_t da = _sat(0.5f + Va / Vdc, 0.0f, 1.0f);
	float32_t db = _sat(0.5f + Vb / Vdc, 0.0f, 1.0f);
	float32_t dc = _sat(0.5f + Vc / Vdc, 0.0f, 1.0f);
	
	uint32_t PWM_Period = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	
	uint32_t compareU = PWM_Period * da;
	uint32_t compareV = PWM_Period * db;
	uint32_t compareW = PWM_Period * dc;
	
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_U_HW, PWM_Counter_U_NUM, compareU);
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_V_HW, PWM_Counter_V_NUM, compareV);
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_W_HW, PWM_Counter_W_NUM, compareW);
	
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);	
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _svpwm()

* -
*******************************************************************************/
/* TODO: Has to be documented */
static inline void _Block(float32_t Duty, uint8_t lut_idx, bool direction_is_math_pos)
{	
	/*TODO: Check if Arrays are correct for clock and countclockwise*/
	static int16_t lut[LUT_SIZE][3] =
	{
		{ HIGH, LOW,  OFF },   // Zustand 1: U+ V- W offen
	    { HIGH, OFF, LOW },    // Zustand 2: U+ W- V offen
	    { OFF, HIGH, LOW },    // Zustand 3: V+ W- U offen
	    { LOW, HIGH, OFF },    // Zustand 4: V+ U- W offen
	    { LOW, OFF, HIGH },    // Zustand 5: W+ U- V offen
	    { OFF, LOW, HIGH }     // Zustand 6: W+ V- U offen
	};

	static int16_t cclut[LUT_SIZE][3] = 
	{
		{HIGH, OFF, LOW},
		{OFF, HIGH, LOW},
		{LOW, HIGH, OFF},
		{LOW, OFF, HIGH},
		{OFF, LOW, HIGH},
		{HIGH, LOW,  OFF}
	};
	
	if (direction_is_math_pos == true)
	{
		set_PWM(PWM_Counter_U_HW, PWM_Counter_U_NUM, A, cclut, lut_idx, Duty);
		set_PWM(PWM_Counter_V_HW, PWM_Counter_V_NUM, B, cclut, lut_idx, Duty);
		set_PWM(PWM_Counter_W_HW, PWM_Counter_W_NUM, C, cclut, lut_idx, Duty);
	}
	else 
	{
		set_PWM(PWM_Counter_U_HW, PWM_Counter_U_NUM, A, lut, lut_idx, Duty);
		set_PWM(PWM_Counter_V_HW, PWM_Counter_V_NUM, B, lut, lut_idx, Duty);
		set_PWM(PWM_Counter_W_HW, PWM_Counter_W_NUM, C, lut, lut_idx, Duty);
	}
}

void set_PWM(TCPWM_Type* hw, uint32_t num, uint8_t phase, int16_t lut_array[LUT_SIZE][3],uint8_t lut_idx, float32_t duty)
{
	uint32_t compare = (uint32_t)(Cy_TCPWM_PWM_GetPeriod0(hw, num) * duty);
	
	switch(lut_array[lut_idx][phase])
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
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _apply_vector_rad(float32_t theta_rad, float32_t m)

* Applies a voltage vector based on theta_rad and factor m
*******************************************************************************/
static inline void _apply_vector_rad(float32_t theta_rad, float32_t m)
{
    theta_rad = _wrap_pi(theta_rad);

    float32_t Vmag = m * (adc_meas.vdc * ONE_o_SQRT3);
    invClarkeVoltage.alpha = Vmag * cosf(theta_rad);
    invClarkeVoltage.beta  = Vmag * sinf(theta_rad);
	float32_t va, vb, vc;
	_inv_clarke_abc(invClarkeVoltage.alpha, invClarkeVoltage.beta, &va, &vb, &vc);
    _svpwm(va, vb, vc, 0.0);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _prepare_coarse(prealign_t *p)
				 static inline void _prepare_six_around(prealign_t *p, float32_t theta0, float32_t delta_rad)
				 static inline void _prepare_polarity(prealign_t *p, float32_t theta0)

* Functions for pre Alignement
*******************************************************************************/
static inline void _prepare_coarse(prealign_t *p)
{
    // 12 angle: 0..330° with 30°- steps (π/6)
    p->theta_count = 12;
    p->idx = 0;

    for (int i = 0; i < 12; i++)
    {
        float32_t th = (float32_t)i * (PI/6.0f);  // 30°
        p->theta_list[i] = _wrap_pi(th);
        p->peaks[i] = 0.0f;
    }
}

static inline void _prepare_six_around(prealign_t *p, float32_t theta0, float32_t delta_rad)
{
    static const float32_t k[6] = {-2.5f, -1.5f, -0.5f, +0.5f, +1.5f, +2.5f};
    p->theta_count = 6;
    p->idx = 0;

    for (int i = 0; i < 6; i++)
    {
        p->theta_list[i] = _wrap_pi(theta0 + k[i] * delta_rad);
        p->peaks[i] = 0.0f;
    }
}

static inline void _prepare_polarity(prealign_t *p, float32_t theta0)
{
    p->theta_count = 2;
    p->idx = 0;
    p->theta_list[0] = _wrap_pi(theta0);
    p->theta_list[1] = _wrap_pi(theta0 + PI);
    p->peaks[0] = p->peaks[1] = 0.0f;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline float32_t _wrap_pi(float32_t x)

* Function used to wrap angle in [-pi; +pi] 
*******************************************************************************/
static inline float32_t _wrap_pi(float32_t x)
{
    while (x >= PI)  x -= TWO_PI;
    while (x < -PI)  x += TWO_PI;
    return x;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: inline float32_t _sat(float32_t x, float32_t min, float32_t max)

* Function satuats between max and min value
*******************************************************************************/
inline float32_t _sat(float32_t x, float32_t min, float32_t max)
{
	if(x < min) return min;
	if(x > max) return max;
	return x;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline float32_t _DEG2RAD_F(float32_t x)

* Function used to calcultate deg out of rad
*******************************************************************************/
static inline float32_t _DEG2RAD_F(float32_t x)
{
    return x*(PI/180.0f);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline int32_t _argmax_f32(const float32_t *a, int n)

* Function used to get the maximum value out of multiple values
*******************************************************************************/
static inline int32_t _argmax_f32(const float32_t *a, int n)
{
    int k = 0;
    for (int i = 1; i < n; i++)
        if (a[i] > a[k]) k = i;
    return k;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void speed_measurment_intr_handler(void)

* Interrupt handler uses Hall signals to calculate the electical and machincal speed of the machine
*******************************************************************************/
void speed_measurment_intr_handler()
{
	uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
	
	Cy_TCPWM_ClearInterrupt(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM, intrStatus);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void speed_measurment_intr_handler(void)

* Interrupt handler uses Hall signals to calculate the electical and machincal speed of the machine
*******************************************************************************/
void controller_counter_intr_handler()
{
	uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Controller_Counter_HW, Controller_Counter_NUM);
	
	Cy_TCPWM_ClearInterrupt(Controller_Counter_HW, Controller_Counter_NUM, intrStatus);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void speed_measurment_intr_handler(void)

* Interrupt handler used for pre alligenment (TODO: enable and disable for pwm interrupt has to be done)
*******************************************************************************/
void pwm_reload_intr_handler()
{
	uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	if (intrStatus & CY_TCPWM_INT_ON_TC)
    {
        if (system_state == SYSTEM_PRE_ALIGNMENT)
        {
            _prealign_step_pwm();
        }
        // else: später SYSTEM_READY -> FOC Control Loop
    }
	Cy_TCPWM_ClearInterrupt(PWM_Counter_U_HW, PWM_Counter_U_NUM, intrStatus);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _prealign_start(void)

* Has to be called when system_state = SYSTEM_PRE_ALIGNMENT (TODO: Has to be called when system_state = SYSTEM_PRE_ALIGNMENT)
*******************************************************************************/
static inline void _prealign_start(void)
{
    memset(&preAlignment, 0, sizeof(preAlignment));

    preAlignment.st = INJ_NEXT;
    preAlignment.stage = 0;
	inj_theta_now = 0.0f;
	preAlignment.theta_e_rad_final_ready = false;
	NVIC_EnableIRQ(pwm_reload_intr_config.intrSrc);
    preAlignment.inj_m = M_COARSE;
    _prepare_coarse(&preAlignment);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _prealign_step_pwm(void)

* function to do pre alignment (TODO: function has to be checked and tested, is called in pwm_reload)
*******************************************************************************/
static inline void _prealign_step_pwm(void)
{
    switch (preAlignment.st)
    {
        case INJ_IDLE:
            return;

        case INJ_NEXT:
        {
            if (preAlignment.idx >= preAlignment.theta_count)
            {
                preAlignment.st = INJ_DONE;
                break;
            }
			
			last_imag = 0.0f;
            float32_t th = preAlignment.theta_list[preAlignment.idx];
     		
            inj_theta_now = th;
            _apply_vector_rad(th, preAlignment.inj_m);

            // per cycle
            if (preAlignment.stage == 0) 
            {
                preAlignment.on_cycles  = INJ_ON_CYCLES_COARSE;
                preAlignment.off_cycles = INJ_OFF_CYCLES_COARSE;
            } 
            else if (preAlignment.stage == 1 || preAlignment.stage == 2) 
            {
                preAlignment.on_cycles  = INJ_ON_CYCLES_FINE;
                preAlignment.off_cycles = INJ_OFF_CYCLES_FINE;
            } 
            else 
            {
                preAlignment.on_cycles  = INJ_ON_CYCLES_POL;
                preAlignment.off_cycles = INJ_OFF_CYCLES_POL;
            }

            inj_collect_now = false;
            preAlignment.st = INJ_ON;
        } break;

        case INJ_ON:
        {
            if (preAlignment.on_cycles == 2) 
            {
                inj_collect_now = true;   // ADC-ISR nimmt jetzt last_imag
            }

            if (--preAlignment.on_cycles <= 0)
            {
                low_sides_all_on();       // OFF-Phase (Phasen clamp)
                inj_collect_now = false;
                preAlignment.st = INJ_OFF;
            }
        } break;

        case INJ_OFF:
        {
            if (--preAlignment.off_cycles <= 0)
            {
                preAlignment.peaks[preAlignment.idx] = last_imag;
                preAlignment.idx++;
                preAlignment.st = INJ_NEXT;
            }
        } break;

        case INJ_DONE:
        {
            // Bestes theta0 finden
            int k = _argmax_f32(preAlignment.peaks, preAlignment.theta_count);
            float32_t theta0 = preAlignment.theta_list[k];

            if (preAlignment.stage == 0)
            {
                // fein um theta0, delta=15° (in rad)
                preAlignment.stage = 1;
                preAlignment.inj_m = M_FINE;
                _prepare_six_around(&preAlignment, theta0, _DEG2RAD_F(15.0f));
                preAlignment.st = INJ_NEXT;
            }
            else if (preAlignment.stage == 1)
            {
                // noch feiner, delta=7.5°
                preAlignment.stage = 2;
                preAlignment.inj_m = M_FINE;
                _prepare_six_around(&preAlignment, theta0, _DEG2RAD_F(7.5f));
                preAlignment.st = INJ_NEXT;
            }
            else if (preAlignment.stage == 2)
            {
                // Polaritätstest: theta0 und theta0+pi
                preAlignment.stage = 3;
                preAlignment.inj_m = M_POL;
                preAlignment.theta_est_rad = theta0;
                _prepare_polarity(&preAlignment, theta0);
                preAlignment.st = INJ_NEXT;
            }
            else
            {
                // stage==3 -> Polarität auswerten
                int k2 = _argmax_f32(preAlignment.peaks, 2);
                preAlignment.theta_polar_is_north = (k2 == 0);

                // Wenn das größere Peak bei theta+pi ist -> Winkel um pi drehen
                float32_t theta_final = preAlignment.theta_est_rad;
                if (k2 == 1) theta_final = _wrap_pi(theta_final + PI);

                // >>> Ergebniswinkel in [-pi,pi) <<<
                // Hier in deine FOC/Estimator-Variablen übernehmen:
                preAlignment.theta_e_rad_final = theta_final;     // deine globale elektrische Anfangslage
				preAlignment.theta_e_rad_final_ready = true;
				NVIC_DisableIRQ(pwm_reload_intr_config.intrSrc);
                // Prealignment beendet:
                preAlignment.st = INJ_IDLE;
                //system_state =;

                // Cy_TCPWM_TriggerStart_Single(Controller_Counter_HW, Controller_Counter_NUM);
            }
        } break;
    }
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void pass_0_sar_0_fifo_0_buffer_0_callback()

* Interrupt when ADC finished
*******************************************************************************/
void pass_0_sar_0_fifo_0_buffer_0_callback()
{
	uint32_t intrStatus = Cy_HPPASS_FIFO_GetInterruptStatusMasked();
	
	const uint8_t ADC_FIFO_IDX = 0U;
	static int16_t adc_data[16];
	static uint8_t adc_chan[16];
	
	if (intrStatus & CY_HPPASS_INTR_FIFO_0_LEVEL)
    {
        /* reads all the voltage and current data */
        uint8_t count = Cy_HPPASS_FIFO_ReadAll(ADC_FIFO_IDX, adc_data, adc_chan);
		
		if (count == 16U) adc_meas_valid = true;
		else adc_meas_valid = false;
		
		/* adc calibration */
		if (system_state == SYSTEM_CALIBRATION)
		{
			adc_calibration_feed_lowside_clamp(adc_data, adc_chan, count);
			/* TODO: Check with Infineon, adc_chan Array always 0 */
			if (adc_calibration.done)
            {
				system_state = SYSTEM_READY;
				_prealign_start();
            }
		}
		else
        {
            adc_frame_t frame = _adc_unpack_frame(adc_data, adc_chan, count);
            _adc_frame_to_si(&frame, &adc_meas);
        }
        
        if (system_state == SYSTEM_PRE_ALIGNMENT && inj_collect_now)
        {
			float32_t ialpha, ibeta;
		    _clarke_ab(adc_meas.iu, adc_meas.iv, adc_meas.iw, &ialpha, &ibeta);
		
		    float32_t s = sinf(inj_theta_now);
		    float32_t c = cosf(inj_theta_now);
		
		    float32_t id = c*ialpha + s*ibeta;
		    last_imag = fabsf(id);
		
		    inj_collect_now = false;
		}
		
        Cy_HPPASS_FIFO_ClearInterrupt(CY_HPPASS_INTR_FIFO_0_LEVEL);
    }
	Cy_HPPASS_FIFO_ClearInterrupt(intrStatus);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline adc_frame_t _adc_unpack_frame(const int16_t *data, const uint8_t *chan, uint8_t n)

* puts adc values in a frame
*******************************************************************************/
static inline adc_frame_t _adc_unpack_frame(const int16_t *data, const uint8_t *chan, uint8_t n)
{
    adc_frame_t f = {0};

    for (uint8_t i = 0; i < n; i++)
    {
        switch (i)
        {
            case ADC_CH_IMS_U:     f.iu  = data[i]; f.valid_mask |= VM_IU;  break;
            case ADC_CH_IMS_V:     f.iv  = data[i]; f.valid_mask |= VM_IV;  break;
            case ADC_CH_IMS_W:     f.iw  = data[i]; f.valid_mask |= VM_IW;  break;
            case ADC_CH_IMS_DCBUS: f.idc = data[i]; f.valid_mask |= VM_IDC; break;

            case ADC_CH_VMS_U:     f.vu  = data[i]; f.valid_mask |= VM_VU;  break;
            case ADC_CH_VMS_V:     f.vv  = data[i]; f.valid_mask |= VM_VV;  break;
            case ADC_CH_VMS_W:     f.vw  = data[i]; f.valid_mask |= VM_VW;  break;
            case ADC_CH_VMS_DCBUS: f.vdc = data[i]; f.valid_mask |= VM_VDC; break;
			// for more values, add antoher case and make shure there is a engouh space to save the data(nothing else to do)
            default: break;
        }
    }
    return f;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline bool _adc_frame_to_si(const adc_frame_t *f, meas_si_t *out)

* frame to Amps and Volts
*******************************************************************************/
static inline bool _adc_frame_to_si(const adc_frame_t *f, meas_si_t *out)
{
	static const float32_t K_IU  = -0.0171f;
	static const float32_t K_IV  = -0.0199f;
	static const float32_t K_IW  = -0.0166f;
	static const float32_t K_IDC = -0.0157f;
	
	static const float32_t K_VPH = 0.0228f; 
	static const float32_t B_VPH = -0.8265f;
	
	static const float32_t K_VDC = 0.0228f;  
	static const float32_t B_VDC = -0.8265f;
	
    const uint8_t need = VM_IU|VM_IV|VM_IW|VM_IDC|VM_VU|VM_VV|VM_VW|VM_VDC;
    if ((f->valid_mask & need) != need) return false;

    out->iu  = _adc_to_amp_i16(f->iu,  adc_calibration.off_iu,  K_IU);
    out->iv  = _adc_to_amp_i16(f->iv,  adc_calibration.off_iv,  K_IV);
    out->iw  = _adc_to_amp_i16(f->iw,  adc_calibration.off_iw,  K_IW);
    out->idc = _adc_to_amp_i16(f->idc, adc_calibration.off_idc, K_IDC);

    out->vu = _adc_to_volt_i16(f->vu, adc_calibration.off_vu, K_VPH, B_VPH);
    out->vv = _adc_to_volt_i16(f->vv, adc_calibration.off_vv, K_VPH, B_VPH);
    out->vw = _adc_to_volt_i16(f->vw, adc_calibration.off_vw, K_VPH, B_VPH);

    out->vdc = K_VDC * (float32_t)f->vdc + B_VDC;

    return true;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void adc_calibration_start_lowside_clamp(uint32_t target_samples, uint32_t warmup_samples)

* Sets up the adc calibration vars.
*******************************************************************************/
void adc_calibration_start_lowside_clamp(uint32_t target_samples, uint32_t warmup_samples)
{
    adc_calibration = (adc_cal_ls_clamp_t){0};
    adc_calibration.target  = target_samples;
    adc_calibration.warmup  = warmup_samples;
    adc_calibration.running = true;
    adc_calibration.done    = false;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline bool _cal_all_done(const adc_cal_ls_clamp_t *c)

*
*******************************************************************************/
static inline bool _cal_all_done(const adc_cal_ls_clamp_t *c)
{
    return (c->cnt_iu  >= c->target) &&
           (c->cnt_iv  >= c->target) &&
           (c->cnt_iw  >= c->target) &&
           (c->cnt_idc >= c->target) &&
           (c->cnt_vu  >= c->target) &&
           (c->cnt_vv  >= c->target) &&
           (c->cnt_vw  >= c->target) &&
           (c->cnt_vdc >= c->target);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline float32_t _adc_to_amp_i16(int16_t adc_raw, int16_t adc_off, float32_t k)

*
*******************************************************************************/
static inline float32_t _adc_to_amp_i16(int16_t adc_raw, int16_t adc_off, float32_t k)
{
    return k * ((float32_t)adc_raw - (float32_t)adc_off);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline float32_t _adc_to_volt_i16(int16_t adc_raw, int16_t adc_off, float32_t k, float32_t b)

*
*******************************************************************************/
static inline float32_t _adc_to_volt_i16(int16_t adc_raw, int16_t adc_off, float32_t k, float32_t b)
{
    // generische lineare Kennlinie: V = k*(raw-off) + b
    return k * ((float32_t)adc_raw - (float32_t)adc_off) + b;
}
/******************************************************************************/


/*******************************************************************************
* Function Name: static inline int16_t _adc_apply_offset(int16_t raw, int16_t off)

*
*******************************************************************************/
static inline int16_t _adc_apply_offset(int16_t raw, int16_t off)
{
    return (int16_t)(raw - off);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void adc_calibration_feed_lowside_clamp(const int16_t *data, const uint8_t *chan, uint8_t n)

*
*******************************************************************************/
void adc_calibration_feed_lowside_clamp(const int16_t *data, const uint8_t *chan, uint8_t n)
{
    if (!adc_calibration.running || adc_calibration.done) return;

    for (uint8_t i = 0; i < n; i++)
    {
        const int16_t x = data[i];

        switch (i)
        {
            case ADC_CH_IMS_U:
                if (adc_calibration.wu_iu < adc_calibration.warmup) { adc_calibration.wu_iu++; break; }
                if (adc_calibration.cnt_iu < adc_calibration.target) { adc_calibration.acc_iu += x; adc_calibration.cnt_iu++; }
                break;

            case ADC_CH_IMS_V:
                if (adc_calibration.wu_iv < adc_calibration.warmup) { adc_calibration.wu_iv++; break; }
                if (adc_calibration.cnt_iv < adc_calibration.target) { adc_calibration.acc_iv += x; adc_calibration.cnt_iv++; }
                break;

            case ADC_CH_IMS_W:
                if (adc_calibration.wu_iw < adc_calibration.warmup) { adc_calibration.wu_iw++; break; }
                if (adc_calibration.cnt_iw < adc_calibration.target) { adc_calibration.acc_iw += x; adc_calibration.cnt_iw++; }
                break;

            case ADC_CH_IMS_DCBUS:
                if (adc_calibration.wu_idc < adc_calibration.warmup) { adc_calibration.wu_idc++; break; }
                if (adc_calibration.cnt_idc < adc_calibration.target) { adc_calibration.acc_idc += x; adc_calibration.cnt_idc++; }
                break;

            case ADC_CH_VMS_U:
                if (adc_calibration.wu_vu < adc_calibration.warmup) { adc_calibration.wu_vu++; break; }
                if (adc_calibration.cnt_vu < adc_calibration.target) { adc_calibration.acc_vu += x; adc_calibration.cnt_vu++; }
                break;

            case ADC_CH_VMS_V:
                if (adc_calibration.wu_vv < adc_calibration.warmup) { adc_calibration.wu_vv++; break; }
                if (adc_calibration.cnt_vv < adc_calibration.target) { adc_calibration.acc_vv += x; adc_calibration.cnt_vv++; }
                break;

            case ADC_CH_VMS_W:
                if (adc_calibration.wu_vw < adc_calibration.warmup) { adc_calibration.wu_vw++; break; }
                if (adc_calibration.cnt_vw < adc_calibration.target) { adc_calibration.acc_vw += x; adc_calibration.cnt_vw++; }
                break;

            case ADC_CH_VMS_DCBUS:
                if (adc_calibration.wu_vdc < adc_calibration.warmup) { adc_calibration.wu_vdc++; break; }
                if (adc_calibration.cnt_vdc < adc_calibration.target) { adc_calibration.acc_vdc += x; adc_calibration.cnt_vdc++; }
                break;
			
			// for more values, add antoher case and make shure there is a engouh space to save the data(nothing else to do)
			
            default:
                break;
        }
    }

    if (!_cal_all_done(&adc_calibration)) return;

    // meanvalue (RAW) -> Offsets for 0A/0V
    adc_calibration.off_iu  = (int16_t)(adc_calibration.acc_iu  / (int64_t)adc_calibration.cnt_iu);
    adc_calibration.off_iv  = (int16_t)(adc_calibration.acc_iv  / (int64_t)adc_calibration.cnt_iv);
    adc_calibration.off_iw  = (int16_t)(adc_calibration.acc_iw  / (int64_t)adc_calibration.cnt_iw);
    adc_calibration.off_idc = (int16_t)(adc_calibration.acc_idc / (int64_t)adc_calibration.cnt_idc);

    adc_calibration.off_vu  = (int16_t)(adc_calibration.acc_vu  / (int64_t)adc_calibration.cnt_vu);
    adc_calibration.off_vv  = (int16_t)(adc_calibration.acc_vv  / (int64_t)adc_calibration.cnt_vv);
    adc_calibration.off_vw  = (int16_t)(adc_calibration.acc_vw  / (int64_t)adc_calibration.cnt_vw);

    // Is under voltage -> not 0V
    adc_calibration.base_vdc_raw = (int16_t)(adc_calibration.acc_vdc / (int64_t)adc_calibration.cnt_vdc);

    adc_calibration.done    = true;
    adc_calibration.running = false;
}
/******************************************************************************/


/*******************************************************************************
* Function Name: void calibration_init(void)

* Sets up the PWM counter for ADC calibaration.
*******************************************************************************/
void calibration_init()
{
	uint32_t SevSw_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(SevSw_Counter_HW, SevSw_Counter_NUM);
	Cy_TCPWM_PWM_SetCompare0Val(SevSw_Counter_HW, SevSw_Counter_NUM, SevSw_PeriodVal);
	
	uint32_t PWM_U_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	uint32_t PWM_V_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_V_HW, PWM_Counter_V_NUM);
	uint32_t PWM_W_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(PWM_Counter_W_HW, PWM_Counter_W_NUM);
	
	uint32_t PWM_PeriodVal = 4800; // If clock divider = 1
	
	if (PWM_U_PeriodVal != PWM_PeriodVal || PWM_V_PeriodVal != PWM_PeriodVal || PWM_W_PeriodVal != PWM_PeriodVal)
	{	
		Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_U_HW, PWM_Counter_U_NUM, PWM_PeriodVal);
		Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_V_HW, PWM_Counter_V_NUM, PWM_PeriodVal);
		Cy_TCPWM_PWM_SetPeriod0(PWM_Counter_W_HW, PWM_Counter_W_NUM, PWM_PeriodVal);
	}
	
	NVIC_EnableIRQ(fifo_isr_cfg.intrSrc);
	adc_calibration_start_lowside_clamp(2048u, 64u);
	low_sides_all_on();
}
/******************************************************************************/



/*******************************************************************************
* Function Name: void init(void)

* that function initalizes all essential componants to run the system.
* Has to be called at the beginning.
*******************************************************************************/
void init()
{
	system_state = SYSTEM_INIT;
    cy_rslt_t result;
	
    /* UART init ofr DEBUG_UART */
	result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);
    
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    if (result != CY_RSLT_SUCCESS) { CY_ASSERT(0); }

    /* HPPASS/SAR init for ADC, FIFO */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_Init(&pass_0_config)) { CY_ASSERT(0); }
    Cy_HPPASS_AC_Start(0U, 100U);

    /* Test Counter init, used to trigger PWM */
	if (result != Cy_TCPWM_Counter_Init(Test_Counter_HW, Test_Counter_NUM, &Test_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Test_Counter_HW, Test_Counter_NUM);

    /* Speed Counter init, used calculate speed in Blockkommuation, sensored FOC */
	if(result != Cy_TCPWM_Counter_Init(Speed_Measurment_Counter_HW,Speed_Measurment_Counter_NUM, &Speed_Measurment_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
    Cy_SysInt_Init(&Speed_Measurment_intr_config, speed_measurment_intr_handler);
    NVIC_EnableIRQ(Speed_Measurment_intr_config.intrSrc);

    /* Controller Counter init, used for PI Controller */
	if(result != Cy_TCPWM_Counter_Init(Controller_Counter_HW,Controller_Counter_NUM, &Controller_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Controller_Counter_HW, Controller_Counter_NUM);
    Cy_SysInt_Init(&Controller_counter_intr_config, controller_counter_intr_handler);
    NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);

    /* PWM U/V/W init */
	if (result != Cy_TCPWM_PWM_Init(PWM_Counter_U_HW, PWM_Counter_U_NUM, &PWM_Counter_U_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    // PWM Terminalcount Interrupt for rotor pre alligment
    Cy_SysInt_Init(&pwm_reload_intr_config, pwm_reload_intr_handler);

	if (result != Cy_TCPWM_PWM_Init(PWM_Counter_V_HW, PWM_Counter_V_NUM, &PWM_Counter_V_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_V_HW, PWM_Counter_V_NUM);

	if (result != Cy_TCPWM_PWM_Init(PWM_Counter_W_HW, PWM_Counter_W_NUM, &PWM_Counter_W_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
    
	if (result != Cy_TCPWM_PWM_Init(SevSw_Counter_HW, SevSw_Counter_NUM, &SevSw_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_PWM_Enable(SevSw_Counter_HW, SevSw_Counter_NUM);

    /* Halls & Button init*/
    Cy_GPIO_Pin_Init(HALL1_PORT, HALL1_PIN, &HALL1_config);
    Cy_GPIO_Pin_Init(HALL2_PORT, HALL2_PIN, &HALL2_config);
    Cy_GPIO_Pin_Init(HALL3_PORT, HALL3_PIN, &HALL3_config);
    Cy_GPIO_Pin_Init(SW1_PORT, SW1_NUM, &SW1_config);
    Cy_GPIO_Pin_Init(SW2_PORT, SW2_NUM, &SW2_config);

    /* ADC FIFO ISR */
    Cy_HPPASS_FIFO_SetInterruptMask(CY_HPPASS_INTR_FIFO_0_LEVEL);
    Cy_SysInt_Init(&fifo_isr_cfg, pass_0_sar_0_fifo_0_buffer_0_callback);
    system_state = SYSTEM_CALIBRATION;
    calibration_init();
}
/*******************************************************************************/


int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();
    
    init();
    static bool button_pressed_before = false;

    for (;;)
    {
		if (Cy_GPIO_Read(SW2_PORT, SW2_NUM) == 1UL && button_pressed_before == false)
		{
			button_pressed_before = true;
	        switch (system_state)
	        {
	            case SYSTEM_READY:
	            	system_state = SYSTEM_PRE_ALIGNMENT;
					NVIC_EnableIRQ(pwm_reload_intr_config.intrSrc);
	                break;
	            
	            // only for testing
	            case SYSTEM_PRE_ALIGNMENT:
					_prealign_start();
	            	NVIC_EnableIRQ(pwm_reload_intr_config.intrSrc);
	            	break;

	
	            default:
	                break;
	        }
		}
		else if (Cy_GPIO_Read(SW2_PORT, SW2_NUM) == 0UL)
		{
			button_pressed_before = false;
		}
		
		if (preAlignment.theta_e_rad_final_ready)
		{
		    preAlignment.theta_e_rad_final_ready = false;
		    printf("theta_e = %.3f rad\r\n", preAlignment.theta_e_rad_final);
		}
    }
}


/* [] END OF FILE */
