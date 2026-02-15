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
    
    volatile float32_t theta_e_rad_final;
    volatile bool theta_e_rad_final_ready;

} prealign_t;


// Slide Mode Observer Variablen
typedef struct
{
	// ---- Motor/System Konstanten ----
	float32_t R; //Strangwiderstand
	float32_t L; //Stanginduktivität
	float32_t Ts; // Abtastzeit
	float32_t fs; // Abtastfrequenz
	float32_t p; //Poolparzahl
	
	// ---- LPF auf Back-EMF ----
	float32_t w_e0_emf; // LPF-Bandbreite
	float32_t k0; //LPF-Koeffizient k0 = w_e0_emf/fs
	float32_t alpha_emf; // EMF-Filter-Grenzfrequenz [rad/s]
	float32_t ealpha_f; //LPF Zustand alpha
	float32_t ebeta_f; //LPF-Zustand beta
	
	// ---- Vorherige Werte ----
	float32_t i_alpha_prev;
	float32_t i_beta_prev;
	float32_t theta_hat;
	float32_t omega_hat; 
	float32_t theta_prev;
	float32_t Kp_pll;
	float32_t Ki_pll;
	float32_t integ_e;
	float32_t theta_emf_raw;
	// ---- Ergebnisse ----
	float32_t theta_rad; //{0;2PI}
	float32_t w_rad_s;
	float32_t n;
	float32_t omega_hat_f;
	float32_t alpha_omega;
	int8_t direction;
	float32_t theta_shift;
} smo_t;
smo_t SMO;

// PI controller struct
typedef struct 
{
    float32_t kp;       // proportional gain
    float32_t ki;       // integral gain (per second)
    float32_t Ts;       // sample time [s]
    float32_t out_min;  // actuator min
    float32_t out_max;  // actuator max
    float32_t prevU;
    float32_t prevE;
    bool useIntFlag;
} PI_t;

// Hall measurmente
typedef struct
{
    volatile uint8_t   lut_idx;    
    volatile uint8_t   prev_lut_idx;
    volatile int8_t    direction;      // +1 = math. pos / -1 = math. negative / 0
    
    volatile uint32_t  prev_Count;
    volatile bool	   overflow;
	
	volatile bool	   correct_angle_set;
	volatile float32_t correct_theta;
	
	volatile float32_t we_hall;
	
	volatile uint32_t last_edge_cnt;
	volatile float32_t theta_sector;
} hall_meas_t;

enum {
    HALL_FAULT_NONE          = 0,
    HALL_FAULT_INVALID_STATE = 1u << 0,
    HALL_FAULT_INVALID_TRANS = 1u << 1,
    HALL_FAULT_GLITCH        = 1u << 2,
    HALL_FAULT_TIMEOUT       = 1u << 3,
};

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

// system frequenzy
#define SysFreq 240000000

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

//Controller Interrupt Time
#define CONTROLLER_INTERRUPT_TIME 0.00004

// Voltage supoply
#define VDD 36.0

// print global vars
typedef struct
{
	float32_t ibus;
	float32_t vbus;
	float32_t id;
	float32_t iq;
	float32_t we_ref;
	float32_t Duty;
	float32_t we;
	float32_t we_hall;
	float32_t we_smo;
}Print_t;
/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/

// ===== Control selection / FSM requests =====
typedef enum {	SYSTEM_OFF, SYSTEM_INIT, SYSTEM_CALIBRATION, SYSTEM_READY, 
				SYSTEM_STARTUP_FOC_PREALIGN, SYSTEM_STARTUP_FOC_OPENLOOP, SYSTEM_RUN_FOC, SYSTEM_SHUTDOWN_FOC_CMD, SYSTEM_SHUTDOWN_FOC,
				SYSTEM_STARTUP_BLOCK, SYSTEM_RUN_BLOCK, SYSTEM_SHUTDOWN_BLOCK_CMD, SYSTEM_SHUTDOWN_BLOCK, 
				SYSTEM_FAULT} system_state_t;
volatile system_state_t system_state = SYSTEM_OFF;

typedef enum {	FOC_SENSORED, FOC_SENSORLESS} foc_type_t;
volatile foc_type_t foc_type = FOC_SENSORLESS;

typedef enum {	SWITCH_CTRL_METHODE,
				START_FOC,
				START_BLOCK,
				STOP_FOC,
				STOP_BLOCK,
				NO_REQ }system_req_t;
volatile system_req_t system_req = NO_REQ;

// Interruptconfig
cy_stc_sysint_t Controller_counter_intr_config = { .intrSrc = Controller_Counter_IRQ, .intrPriority = 4U };
cy_stc_sysint_t Speed_Measurment_intr_config   = { .intrSrc = Speed_Measurment_Counter_IRQ, .intrPriority = 5U };
cy_stc_sysint_t hall_isr3_config               = { .intrSrc = ioss_interrupts_sec_gpio_8_IRQn, .intrPriority = 2U };
cy_stc_sysint_t hall_isr12_config              = { .intrSrc = ioss_interrupts_sec_gpio_9_IRQn, .intrPriority = 2U };
cy_stc_sysint_t fifo_isr_cfg             	   = { .intrSrc = pass_interrupt_fifos_IRQn, .intrPriority = 3U };
cy_stc_sysint_t set_value_intr_config          = { .intrSrc = Test_Counter_IRQ, .intrPriority = 1U };
cy_stc_sysint_t pwm_reload_intr_config         = { .intrSrc = PWM_Counter_U_IRQ, .intrPriority =6U};
cy_stc_sysint_t Safety_intr_config			   = { .intrSrc = Safety_Counter_IRQ, .intrPriority = 7U};

// DEBUG_UART
static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;

// ADC - Calibration
adc_cal_ls_clamp_t adc_calibration;

// ADC - meas values with voltage and current data
volatile  meas_si_t adc_meas;
volatile bool adc_meas_valid = false;

// calculated voltages for transformation
volatile ab_t invClarkeVoltage;
volatile dq_t invParkVoltage;

// prealignment
prealign_t preAlignment;
volatile bool inj_collect_now = false;     // gets true in ON
volatile float32_t inj_theta_now = 0.0f;
volatile float32_t last_imag = 0.0f;       // ADC-ISR writes values in Peak

// Hall measurement
static hall_meas_t hall_values = {};

// Block pi controller
static PI_t g_pi_block_speed;

// Test Vars
volatile Print_t PrintVars;
float32_t we_ref = 800.0;
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
static inline bool _cal_all_done(adc_cal_ls_clamp_t *c);
void adc_calibration_feed_lowside_clamp(const int16_t *data, const uint8_t *chan, uint8_t n);
static inline int16_t _adc_apply_offset(int16_t raw, int16_t off);
void low_sides_all_on(void);
void high_sides_all_on(void);
static inline float32_t _adc_to_amp_i16(int16_t adc_raw, int16_t adc_off, float32_t k);
static inline float32_t _adc_to_volt_i16(int16_t adc_raw, int16_t adc_off, float32_t k, float32_t b);
static inline adc_frame_t _adc_unpack_frame(const int16_t *data, const uint8_t *chan, uint8_t n);
static inline bool _adc_frame_to_si(const adc_frame_t *f, volatile meas_si_t *out);
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
static inline float32_t _sat(float32_t x, float32_t min, float32_t max);
void SMO_Init(smo_t *smo);
static inline void _SMO_Update(smo_t *smo, float32_t iAlpha, float32_t iBeta, float32_t vAlpha, float32_t vBeta);
static inline float32_t _SMO_GetTheta(const smo_t* smo);
static inline float32_t _SMO_GetOmega(const smo_t* smo);
static inline float32_t _SMO_GetSpeed(const smo_t* smo);
static inline float32_t _pi_controller_step(PI_t *c, float32_t setpoint, float32_t measurement);
void init_pi_d(PI_t *d, float32_t vd);
void init_pi_q(PI_t *q, float32_t vq);
void init_pi_speed(PI_t *s);
static inline void _open_loop_step(float32_t *vd, float32_t *vq, float32_t we);
void Poti_read(void);
void safety_intr_handler();
static inline void _pwm_all_enable();
static inline void _pwm_all_disable();
static inline void _motor_outputs_stop_now();
uint8_t read_hall_state(void);
inline int8_t _get_direction(uint8_t lut_idx, uint8_t prev_lut_idx);
inline float32_t _get_correct_theta(uint8_t lut_idx, uint8_t prev_lut_idx);
void hall_interrupt_handler(void);
static inline float32_t _HALL_get_theta(const hall_meas_t* h);
static inline float32_t _HALL_get_we(hall_meas_t* hall);
static void init_pi_block_speed(PI_t *s);
static inline float32_t block_get_duty_from_speed(float32_t we_ref_cmd, float32_t we_meas);
static void system_fsm_step();
static inline void _Block(float32_t Duty, uint8_t lut_idx, bool direction_is_math_pos);
static inline void foc_switch_ctrl_methode();

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
* Function Name: static inline void _pwm_all_enable()

* enables all three PWM Counter
*******************************************************************************/

static inline void _pwm_all_enable()
{
    Cy_TCPWM_PWM_Enable(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_PWM_Enable(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_PWM_Enable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
}

/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _pwm_all_disable()

* disables all three PWM Counter
*******************************************************************************/

static inline void _pwm_all_disable()
{
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_U_HW, PWM_Counter_U_NUM, 0u);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_V_HW, PWM_Counter_V_NUM, 0u);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_W_HW, PWM_Counter_W_NUM, 0u);

    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);

    Cy_TCPWM_PWM_Disable(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
}

/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void _motor_outputs_stop_now()

* Stops the motor
*******************************************************************************/

static inline void _motor_outputs_stop_now()
{

    _pwm_all_enable();
    low_sides_all_on();
}

/******************************************************************************/

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
	
	float32_t da = _sat(0.5f + Va / VDD, 0.0f, 1.0f);
	float32_t db = _sat(0.5f + Vb / VDD, 0.0f, 1.0f);
	float32_t dc = _sat(0.5f + Vc / VDD, 0.0f, 1.0f);
	
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

static void block_start(void)
{   
    init_pi_block_speed(&g_pi_block_speed);
    
    _pwm_all_enable();
    NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);
    Cy_TCPWM_TriggerStart_Single(Controller_Counter_HW, Controller_Counter_NUM);
}

static inline float32_t block_get_duty_from_speed(float32_t we_ref_cmd, float32_t we_meas)
{
    float32_t duty = _pi_controller_step(&g_pi_block_speed, we_ref_cmd, fabsf(we_meas));
    return _sat(duty, 0.0f, 0.95f);
}

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
static inline float32_t _sat(float32_t x, float32_t min, float32_t max)
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
	
	hall_values.overflow = true;
	
	Cy_TCPWM_ClearInterrupt(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM, intrStatus);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void safety_intr_handler()

* Interrupt handler to turn off the machine
*******************************************************************************/
void safety_intr_handler()
{
	uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Safety_Counter_HW, Safety_Counter_NUM);
	
    system_state = SYSTEM_FAULT;
	
	Cy_TCPWM_ClearInterrupt(Safety_Counter_HW, Safety_Counter_NUM, intrStatus);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline void foc_switch_ctrl_methode()

* 
*******************************************************************************/
static inline void foc_switch_ctrl_methode()
{
    if (foc_type == FOC_SENSORLESS)
    {
        foc_type = FOC_SENSORED;
    }
    else
    {
        float32_t th_h = _HALL_get_theta(&hall_values);
        float32_t we_h = _HALL_get_we(&hall_values);

        __disable_irq();
        SMO.theta_hat   = _wrap_pi(th_h - SMO.theta_shift);
        SMO.theta_rad   = th_h;
        SMO.omega_hat   = we_h;
        SMO.omega_hat_f = we_h;
        SMO.integ_e     = we_h;
        __enable_irq();

        foc_type = FOC_SENSORLESS;
    }
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void SMO_Init(smo_t *smo)
				 void inline _SMO_Update(smo_t *smo, float32_t iAlpha, float32_t iBeta, float32_t vAlpha, float32_t vBeta)
				 static inline float32_t _SMO_GetTheta(const smo_t* smo)
				 static inline float32_t _SMO_GetOmega(const smo_t* smo)
				 static inline float32_t _SMO_GetSpeed(const smo_t* smo)
				 
* All function for Slidemode Observer to calc. the rotor angle
*******************************************************************************/

void SMO_Init(smo_t *smo)
{
	smo->R = 0.035;
    smo->L = 0.000086;
    smo->Ts = CONTROLLER_INTERRUPT_TIME;
    smo->fs = 1.0/CONTROLLER_INTERRUPT_TIME;
    smo->p  = 1.0;
    
    smo->ealpha_f = 0.0f;  smo->ebeta_f = 0.0f;
    smo->i_alpha_prev = 0.0f;  smo->i_beta_prev = 0.0f;
    smo->theta_prev = 0.0f;
    
    // PLL-Startwerte (z.B. aus deinem Open-Loop)
    smo->theta_hat = 0.0f;
    smo->omega_hat = 0.0f;
    smo->integ_e   = 0.0f;

    //Kp ~ 2*w_pll, Ki ~ (w_pll)^2; w_pll << w0_emf
    const float w_pll = 2.0f * 3.14159265f * 15.0f;   // 15 Hz
    smo->Kp_pll = 1.4f * w_pll;
    smo->Ki_pll = (w_pll*w_pll);
    smo->theta_emf_raw = 0.0f;
    
    smo->w_e0_emf = w_pll * 6.0;
    smo->alpha_emf = 1.0f - expf(-smo->w_e0_emf * smo->Ts);
    if (smo->alpha_emf > 1.0f) smo->alpha_emf = 1.0f;
    if (smo->alpha_emf < 0.0f) smo->alpha_emf = 0.0f;
    
    smo->theta_rad = 0.0;
    smo->w_rad_s = 0.0f;
    
    smo->alpha_omega = 0.3;
    
    smo->omega_hat_f = 0.0f;
    
    smo->theta_shift = 0.36f;
}

static inline void _SMO_Update(smo_t *smo, float32_t iAlpha, float32_t iBeta, float32_t vAlpha, float32_t vBeta)
{
	const float32_t di_alpha = (iAlpha - smo->i_alpha_prev) * smo->fs;
    const float32_t di_beta  = (iBeta  - smo->i_beta_prev)  * smo->fs;
	
	// EMF = v - R*i - L*di/dt
    float32_t e_alpha = vAlpha - (smo->R * iAlpha) - (smo->L * di_alpha);
    float32_t e_beta  = vBeta  - (smo->R * iBeta)  - (smo->L * di_beta);
	
	const float32_t a = smo->alpha_emf;
    smo->ealpha_f += a * (e_alpha - smo->ealpha_f);
    smo->ebeta_f  += a * (e_beta  - smo->ebeta_f);
	
	float32_t e_theta = -((smo->ebeta_f * sinf(smo->theta_hat)) + (smo->ealpha_f * cosf(smo->theta_hat)));
	
	// PLL mit einfachem Antiwindup
    const float32_t OMEGA_MAX = 6720.0f;
    float32_t omega_u = smo->Kp_pll * e_theta + smo->integ_e;
    float32_t omega_limited = fminf(fmaxf(omega_u, -OMEGA_MAX), OMEGA_MAX);
    smo->omega_hat = omega_limited;
    
    // Test:
    PrintVars.we_smo = omega_limited;
    
    // Back-calculation (k_aw ≈ ω_n)
    const float k_aw = 2.0f * 3.14159265f * 15.0f;
    smo->integ_e += (smo->Ki_pll * e_theta + (omega_limited - omega_u) * k_aw) * smo->Ts;
	
    smo->theta_hat = _wrap_pi(smo->theta_hat + smo->Ts * smo->omega_hat);
    smo->theta_rad = _wrap_pi(smo->theta_hat + smo->theta_shift);

    // w-Glättung (für Anzeige)
    smo->omega_hat_f += smo->alpha_omega * (smo->omega_hat - smo->omega_hat_f);
	
    smo->i_alpha_prev = iAlpha;
    smo->i_beta_prev  = iBeta;
}


static inline float32_t _SMO_GetTheta(const smo_t* smo){ return smo->theta_rad; }
static inline float32_t _SMO_GetOmega(const smo_t* smo){ return smo->omega_hat; }
static inline float32_t _SMO_GetSpeed(const smo_t* smo){ return (smo->omega_hat / (TWO_PI * smo->p)) * 60.0f;}
/******************************************************************************/


/*******************************************************************************
* Function Name: 

				 
* 
*******************************************************************************/

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

const float32_t lut_to_theta[6] = 
{
	0.26179939,
	1.57079633,
	2.87979327,
	-2.87979327,
	-1.57079633,
	-0.26179939
};

const float32_t lut_to_theta_sector[6] = 
{
	0.0,
	1.04719755,
	2.0943951,
	3.14159265,
	-2.0943951,
	-1.04719755
};

uint8_t read_hall_state(void)
{
    uint8_t H1 = Cy_GPIO_Read(HALL1_PORT, HALL1_PIN);
    uint8_t H2 = Cy_GPIO_Read(HALL2_PORT, HALL2_PIN);
    uint8_t H3 = Cy_GPIO_Read(HALL3_PORT, HALL3_PIN);
    return (H1 << 2) | (H2 << 1) | H3;
}

inline int8_t _get_direction(uint8_t lut_idx, uint8_t prev_lut_idx)
{
	if 		((int8_t)lut_idx - (int8_t)prev_lut_idx == 1 || (int8_t)lut_idx - (int8_t)prev_lut_idx == -5) return 1;
	else if ((int8_t)lut_idx - (int8_t)prev_lut_idx == -1 || (int8_t)lut_idx - (int8_t)prev_lut_idx == 5) return -1;
	else return 0;
}

inline float32_t _get_correct_theta(uint8_t lut_idx, uint8_t prev_lut_idx)
{
	if ((lut_idx == 1 && prev_lut_idx == 0) || (lut_idx == 0 && prev_lut_idx == 1)) return lut_to_theta[0];
	else if ((lut_idx == 2 && prev_lut_idx == 1) || (lut_idx == 1 && prev_lut_idx == 2)) return lut_to_theta[1];
	else if ((lut_idx == 3 && prev_lut_idx == 2) || (lut_idx == 2 && prev_lut_idx == 3)) return lut_to_theta[2];
	else if ((lut_idx == 4 && prev_lut_idx == 3) || (lut_idx == 3 && prev_lut_idx == 4)) return lut_to_theta[3];
	else if ((lut_idx == 5 && prev_lut_idx == 4) || (lut_idx == 4 && prev_lut_idx == 5)) return lut_to_theta[4];
	else if ((lut_idx == 0 && prev_lut_idx == 5) || (lut_idx == 5 && prev_lut_idx == 0)) return lut_to_theta[5];
	else return 0.0;
}


void hall_interrupt_handler(void)
{
	
	uint8_t lut_index = hall_to_lut[read_hall_state()];
    if (lut_index == 255U)
    {
        Cy_GPIO_ClearInterrupt(HALL1_PORT, HALL1_PIN);
        Cy_GPIO_ClearInterrupt(HALL2_PORT, HALL2_PIN);
        Cy_GPIO_ClearInterrupt(HALL3_PORT, HALL3_PIN);
        return;
    }
	
	
	hall_values.lut_idx = lut_index;
	//hall_values.direction = _get_direction(lut_index, hall_values.prev_lut_idx);
	hall_values.theta_sector = lut_to_theta_sector[lut_index];
	//hall_values.correct_theta = _get_correct_theta(lut_index, hall_values.prev_lut_idx);
	if (hall_values.correct_theta != 0.0f) hall_values.correct_angle_set = true;
	else hall_values.correct_angle_set = false;
	hall_values.prev_lut_idx = lut_index;
	
	if (Cy_TCPWM_Counter_GetStatus(Safety_Counter_HW, Safety_Counter_NUM) & CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING)
	{
		Cy_TCPWM_TriggerStopOrKill_Single(Safety_Counter_HW, Safety_Counter_NUM);
		Cy_TCPWM_Counter_SetCounter(Safety_Counter_HW, Safety_Counter_NUM, 0U);
	}
	Cy_TCPWM_TriggerStart_Single(Safety_Counter_HW, Safety_Counter_NUM);
	
	if (Cy_TCPWM_Counter_GetStatus(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM) & CY_TCPWM_COUNTER_STATUS_COUNTER_RUNNING)
	{
		uint32_t count = Cy_TCPWM_Counter_GetCounter(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
		if (hall_values.overflow == false)
        {
            uint32_t diff_count = (count - hall_values.prev_Count);
            if (diff_count > 0u)
            {
                float32_t timePerHall = (float32_t)diff_count / (float32_t)SysFreq;
                float32_t we_mag = TWO_PI / (timePerHall * (float32_t)LUT_SIZE);

                int8_t dir = hall_values.direction;
                if (dir == 0) dir = +1;
                hall_values.we_hall = (float32_t)dir * we_mag;
                
                // Test:
                PrintVars.we_hall = hall_values.we_hall;
            }
            hall_values.prev_Count = count;
        }
		else 
		{
			hall_values.prev_Count = count;
			hall_values.overflow = false;	
		}
		hall_values.last_edge_cnt = count;
	}
	else 
	{
		Cy_TCPWM_TriggerStart_Single(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
        hall_values.prev_Count = Cy_TCPWM_Counter_GetCounter(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
        hall_values.last_edge_cnt = hall_values.prev_Count;
	}
	
    Cy_GPIO_ClearInterrupt(HALL1_PORT, HALL1_PIN);
    Cy_GPIO_ClearInterrupt(HALL2_PORT, HALL2_PIN);
    Cy_GPIO_ClearInterrupt(HALL3_PORT, HALL3_PIN);
}

static inline float32_t _HALL_get_theta(const hall_meas_t* h)
{
    uint32_t now = Cy_TCPWM_Counter_GetCounter(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
    uint32_t diff = (now - h->last_edge_cnt);
    float32_t dt = (float32_t)diff / (float32_t)SysFreq;

    return _wrap_pi(h->theta_sector + h->we_hall * dt);
}

static inline float32_t _HALL_get_we(hall_meas_t* hall) {return hall->we_hall;}
/******************************************************************************/


/*******************************************************************************
* Function Name: float32_t inline _pi_controller_step(PI_t *c, float32_t setpoint, float32_t measurement)

* PI COntroller function
*******************************************************************************/
static inline float32_t _pi_controller_step(PI_t *c, float32_t setpoint, float32_t measurement)
{
	float32_t e = setpoint - measurement;
	
	float32_t u_unsat; 

	if (c->useIntFlag) u_unsat = c->prevU + c->kp * (e - c->prevE + c->ki * c->Ts * e);
	//else u_unsat = c->prevU + c->kp * (e - c->prevE);
	else u_unsat = c->kp * e;
	
	float32_t u_sat = _sat(u_unsat, c->out_min, c->out_max);
	
	if (u_unsat - u_sat > 0.01 || u_unsat - u_sat < -0.01) c->useIntFlag = false;
	else c->useIntFlag = true;

	c->prevE = e;
	c->prevU = u_sat;
	
	return u_sat;
}
/******************************************************************************/


/*******************************************************************************
* Function Name: static inline void _open_loop_step(float32_t *vd, float32_t *vq, float32_t we)

* Interrupt handler uses Hall signals to calculate the electical and machincal speed of the machine
*******************************************************************************/
static inline void _open_loop_step(float32_t *vd, float32_t *vq, float32_t we)
{
	const float32_t vq_boost = 1.0;
	const float32_t we_ramp = 0.002; //[V/rad/s]
	
	*vd = 0.0;
	*vq = vq_boost + we_ramp * we;
	if (*vq < 0.0) *vq = 0.0;
}
/******************************************************************************/

/*******************************************************************************
* Function Name: void controller_counter_intr_handler()

* Interrupt handler for controller
*******************************************************************************/
void controller_counter_intr_handler()
{
	uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Controller_Counter_HW, Controller_Counter_NUM);
	
	const float32_t OL_TIME = 5.0;
	const float32_t OL_WE = 800.0;
	const float32_t OL_DUTY = 0.2;
	const float32_t CALL_TIME = 1.0/25000.0;
	
	// =========================
    // BLOCK
    // =========================
	
	if (system_state == SYSTEM_STARTUP_BLOCK || system_state == SYSTEM_RUN_BLOCK || system_state == SYSTEM_SHUTDOWN_BLOCK_CMD || system_state == SYSTEM_SHUTDOWN_BLOCK)
	{
		static float32_t duty = 0.0;
		float32_t we_m = 0.0;
		if (system_state == SYSTEM_STARTUP_BLOCK)
		{
			we_m = _HALL_get_we(&hall_values);
			duty += OL_DUTY * CALL_TIME/OL_TIME;
			if (duty >= OL_DUTY) system_state = SYSTEM_RUN_BLOCK;
		}
		
		else if (system_state == SYSTEM_RUN_BLOCK || system_state == SYSTEM_SHUTDOWN_BLOCK_CMD)
	    {	
	        we_m = _HALL_get_we(&hall_values);
	        if (system_state == SYSTEM_SHUTDOWN_BLOCK_CMD) 
	        {
				we_ref = OL_WE;
				if (we_m < OL_WE + 10.0) system_state = SYSTEM_SHUTDOWN_BLOCK;
			}
	        duty = block_get_duty_from_speed(we_ref, we_m);
	    }
		
		else if (system_state == SYSTEM_SHUTDOWN_BLOCK)
		{
			we_m = _HALL_get_we(&hall_values);
			duty -= OL_DUTY * CALL_TIME/OL_TIME;
			if (duty <= 0.0)
		    {
				duty = 0.0;
				we_m = 0.0;
		        system_state = SYSTEM_READY;
		    }
		}
		bool dir_math_pos = (hall_values.direction >= 0);
	    _Block(duty, hall_values.lut_idx, dir_math_pos);
		// Test:
		PrintVars.we = we_m;
		PrintVars.Duty = duty;
	}
	
	// =========================
    // FOC
    // =========================
	
	if (system_state == SYSTEM_STARTUP_FOC_OPENLOOP || system_state == SYSTEM_RUN_FOC || system_state == SYSTEM_SHUTDOWN_FOC_CMD || system_state == SYSTEM_SHUTDOWN_FOC)
	{
		meas_si_t measurment_data = adc_meas;
		
		float32_t ialpha = 0.0, ibeta = 0.0;
		_clarke_ab(measurment_data.iu, measurment_data.iv, measurment_data.iw, &ialpha, &ibeta);
		
		float32_t valpha_measured = 0.0, vbeta_measured = 0.0;
		_clarke_ab(measurment_data.vu, measurment_data.vv, measurment_data.vw, &valpha_measured, &vbeta_measured);
		
		_SMO_Update(&SMO, ialpha, ibeta, valpha_measured, vbeta_measured);
		
		static float32_t th = 0.0;
		static float32_t we = 0.0;
		static PI_t d;
		static PI_t q;
		static PI_t speed;
		float32_t vd = 0.0, vq = 0.0;
		float32_t id = 0.0, iq = 0.0;
		float32_t c = 0.0;
		float32_t s = 0.0;
		
		if (system_state == SYSTEM_STARTUP_FOC_OPENLOOP)
		{
			we += OL_WE * CALL_TIME/OL_TIME;
			th = _wrap_pi(th + CALL_TIME * we);
			
			c = cosf(th);
			s = sinf(th);
			_park_dq(ialpha, ibeta, s, c, &id, &iq);
			
			if (we <= OL_WE) _open_loop_step(&vd, &vq, we);
		    else
		    {
		        _open_loop_step(&vd, &vq, we);
		        system_state = SYSTEM_RUN_FOC;
		        init_pi_d(&d, vd);
		        init_pi_q(&q, vq);
		        init_pi_speed(&speed);
		    }
		}
		
		else if (system_state == SYSTEM_RUN_FOC || system_state == SYSTEM_SHUTDOWN_FOC_CMD)
		{
			if (foc_type == FOC_SENSORED)
			{
				we = _HALL_get_we(&hall_values);
            	th = _HALL_get_theta(&hall_values);
			}
			else if (foc_type == FOC_SENSORLESS)
			{
				we = _SMO_GetOmega(&SMO);
            	th = _SMO_GetTheta(&SMO);		
			}
			
			c = cosf(th);
			s = sinf(th);
			_park_dq(ialpha, ibeta, s, c, &id, &iq);
			
			static float32_t iq_ref = 0.0f;
		    static uint8_t count = 10;
		    if (count >= 9)
		    {
				if (system_state == SYSTEM_SHUTDOWN_FOC_CMD) 
				{
					we_ref = OL_WE;
					if (we <= OL_WE + 10.0) system_state = SYSTEM_SHUTDOWN_FOC;
				}
		        iq_ref = _pi_controller_step(&speed, we_ref, we);
		        count = 0;
		    }
		    else count++;
		
			vd = _pi_controller_step(&d, 0.0f, id);
			vq = _pi_controller_step(&q, iq_ref, iq);
		}
		
		else if (system_state == SYSTEM_SHUTDOWN_FOC)
		{
			we -= OL_WE * CALL_TIME/OL_TIME;
			th = _wrap_pi(th + CALL_TIME * we);
			
			c = cosf(th);
			s = sinf(th);
			_park_dq(ialpha, ibeta, s, c, &id, &iq);
			
			if (we >= 0.0) _open_loop_step(&vd, &vq, we);
		    else
		    {
				vd = 0.0;
				vq = 0.0;
				we = 0.0;
		        system_state = SYSTEM_READY;
		    }
		}
		
		//Only for testing Print vars
		PrintVars.we = we;
		PrintVars.id = id;
		PrintVars.iq = iq;
		PrintVars.we_ref = we_ref;
		
		float32_t valpha, vbeta;
		_inv_park_ab(vd, vq, s, c, &valpha, &vbeta);
		
		float32_t va, vb, vc;
		_inv_clarke_abc(valpha, vbeta, &va, &vb, &vc);

	    _pwm_all_enable();
	    _svpwm(va, vb, vc, 0.5f);
	}
	
	Cy_TCPWM_ClearInterrupt(Controller_Counter_HW, Controller_Counter_NUM, intrStatus);
}

void init_pi_d(PI_t *d, float32_t vd)
{
	d->ki 			= 219.9;
	d->kp			= 0.54;
	d->out_max		= VDD/2.0;
	d->out_min		= -VDD/2.0;
	d->Ts			= CONTROLLER_INTERRUPT_TIME;
	d->prevE		= 0.0;
	d->prevU		= vd;
	d->useIntFlag	= true;
}

void init_pi_q(PI_t *q, float32_t vq)
{
	q->ki 			= 219.9;
	q->kp			= 0.54;
	q->out_max		= VDD/2.0;
	q->out_min		= -VDD/2.0;
	q->Ts			= CONTROLLER_INTERRUPT_TIME;
	q->prevE		= 0.0;
	q->prevU		= vq;
	q->useIntFlag	= true;
}

void init_pi_speed(PI_t *s)
{
	s->ki 			= 2.7;
	s->kp			= 0.1;
	s->out_max		= 3.0;
	s->out_min		= -3.0;
	s->Ts			= CONTROLLER_INTERRUPT_TIME*10.0;
	s->prevE		= 0.0;
	s->prevU		= 0.0;
	s->useIntFlag	= true;
}

static void init_pi_block_speed(PI_t *s)
{
    s->ki = 3.0;
    s->kp = 0.0004;
    s->out_max = 0.99;
    s->out_min = 0.0f;
    s->Ts = CONTROLLER_INTERRUPT_TIME;
    s->prevE = 0.0f;
    s->prevU = 0.0f;
    s->useIntFlag = true;
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
        if (system_state == SYSTEM_STARTUP_FOC_PREALIGN)
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
	NVIC_EnableIRQ(pwm_reload_intr_config.intrSrc);
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
            int k = _argmax_f32(preAlignment.peaks, preAlignment.theta_count);
            float32_t theta0 = preAlignment.theta_list[k];

            if (preAlignment.stage == 0)
            {
                preAlignment.stage = 1;
                preAlignment.inj_m = M_FINE;
                _prepare_six_around(&preAlignment, theta0, _DEG2RAD_F(15.0f));
                preAlignment.st = INJ_NEXT;
            }
            else if (preAlignment.stage == 1)
            {
                preAlignment.stage = 2;
                preAlignment.inj_m = M_FINE;
                _prepare_six_around(&preAlignment, theta0, _DEG2RAD_F(7.5f));
                preAlignment.st = INJ_NEXT;
            }
            else if (preAlignment.stage == 2)
            {
                preAlignment.stage = 3;
                preAlignment.inj_m = M_POL;
                preAlignment.theta_est_rad = theta0;
                _prepare_polarity(&preAlignment, theta0);
                preAlignment.st = INJ_NEXT;
            }
            else
            {
                int k2 = _argmax_f32(preAlignment.peaks, 2);
                preAlignment.theta_polar_is_north = (k2 == 0);

                float32_t theta_final = preAlignment.theta_est_rad;
                if (k2 == 1) theta_final = _wrap_pi(theta_final + PI);

                preAlignment.theta_e_rad_final = theta_final;
				preAlignment.theta_e_rad_final_ready = true;
				NVIC_DisableIRQ(pwm_reload_intr_config.intrSrc);
                // Prealignment edning:
				foc_type = FOC_SENSORLESS;
                system_state = SYSTEM_STARTUP_FOC_OPENLOOP;
                SMO_Init(&SMO);
                NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);
                Cy_TCPWM_TriggerStart_Single(Controller_Counter_HW, Controller_Counter_NUM);
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
		// Test:
		Poti_read();
		
        /* reads all the voltage and current data */
        uint8_t count = Cy_HPPASS_FIFO_ReadAll(ADC_FIFO_IDX, adc_data, adc_chan);
		
		if (count == 16U) adc_meas_valid = true;
		else adc_meas_valid = false;
		
		/* adc calibration */
		if (system_state == SYSTEM_CALIBRATION)
		{
			adc_calibration_feed_lowside_clamp(adc_data, adc_chan, count);
			if (adc_calibration.done)
            {
				system_state = SYSTEM_READY;
				_motor_outputs_stop_now();
            }
		}
		else
        {
            adc_frame_t frame = _adc_unpack_frame(adc_data, adc_chan, count);
            
            meas_si_t tmp;
            if (_adc_frame_to_si(&frame, &tmp))
			{
			    adc_meas = tmp;     
			    adc_meas_valid = true;
			}
			else adc_meas_valid = false;
        }
        
        if (system_state == SYSTEM_STARTUP_FOC_PREALIGN && inj_collect_now)
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
inline adc_frame_t _adc_unpack_frame(const int16_t *data, const uint8_t *chan, uint8_t n)
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
inline bool _adc_frame_to_si(const adc_frame_t *f, volatile meas_si_t *out)
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
	
	// Test: 
	PrintVars.ibus = out->idc;
	
    out->vu = _adc_to_volt_i16(f->vu, adc_calibration.off_vu, K_VPH, B_VPH);
    out->vv = _adc_to_volt_i16(f->vv, adc_calibration.off_vv, K_VPH, B_VPH);
    out->vw = _adc_to_volt_i16(f->vw, adc_calibration.off_vw, K_VPH, B_VPH);

    out->vdc = K_VDC * (float32_t)f->vdc + B_VDC;
    
    // Test: 
	PrintVars.vbus = out->vdc;
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
static inline bool _cal_all_done(adc_cal_ls_clamp_t *c)
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
* Function Name: static void system_fsm_step()


*******************************************************************************/
static void system_fsm_step()
{
    if (system_req == START_FOC)
    {
        system_state = SYSTEM_STARTUP_FOC_PREALIGN;
        _pwm_all_enable();
        NVIC_EnableIRQ(pwm_reload_intr_config.intrSrc);
        _prealign_start();
        system_req = NO_REQ;
    }
   	else if(system_req == START_BLOCK)
    {	
		system_state = SYSTEM_STARTUP_BLOCK;
		block_start();
		system_req = NO_REQ;
	}
	else if(system_req == STOP_FOC)
    {	
		system_state = SYSTEM_SHUTDOWN_FOC_CMD;
		system_req = NO_REQ;
	}
	else if(system_req == STOP_BLOCK)
    {	
		system_state = SYSTEM_SHUTDOWN_BLOCK_CMD;
		system_req = NO_REQ;
	}

	
}
/******************************************************************************/

void Poti_read(void)
{

    int32_t raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(12U);
    we_ref = (float32_t)raw_value_idPot / 4095.0f * 6400.0;
    
    // Test:
    PrintVars.we_ref = we_ref;
    
    raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(10U);
	// = (float32_t)raw_value_idPot / 4095.0f * 1000.0;

}

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
    
    /* Safety Counter turns off the machine if something goes wrong */
   	if(result != Cy_TCPWM_Counter_Init(Safety_Counter_HW,Safety_Counter_NUM, &Safety_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Safety_Counter_HW, Safety_Counter_NUM);
    Cy_SysInt_Init(&Safety_intr_config, safety_intr_handler);
    //NVIC_EnableIRQ(Safety_intr_config.intrSrc);

    /* Controller Counter init, used for PI Controller */
	if(result != Cy_TCPWM_Counter_Init(Controller_Counter_HW,Controller_Counter_NUM, &Controller_Counter_config)) { CY_ASSERT(0); }
    Cy_TCPWM_Counter_Enable(Controller_Counter_HW, Controller_Counter_NUM);
    Cy_SysInt_Init(&Controller_counter_intr_config, controller_counter_intr_handler);

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
    Cy_SysInt_Init(&hall_isr12_config, hall_interrupt_handler);
    Cy_SysInt_Init(&hall_isr3_config, hall_interrupt_handler);
    NVIC_EnableIRQ(hall_isr12_config.intrSrc);
    NVIC_EnableIRQ(hall_isr3_config.intrSrc);
    
    /* Fan PWM Pin*/
    Cy_GPIO_Pin_Init(FAN_PWM_PORT, FAN_PWM_PIN, &FAN_PWM_config);
    
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
    static uint8_t sw1_prev = 0, sw2_prev = 0;
	
    for (;;)
    {
		/*
		
		READY:
		SW1 FOC
		SW2 BLOCK
		
		RUN_FOC:
		SW1 toggelt sensorless <-> hall
		SW2: Stop
		
		RUN_BLOCK:
		SW1: --		
		SW2: Stop
		
		*/
		
		uint8_t sw1 = (uint8_t)Cy_GPIO_Read(SW1_PORT, SW1_NUM);
	    uint8_t sw2 = (uint8_t)Cy_GPIO_Read(SW2_PORT, SW2_NUM);
	
	    bool sw1_rise = (sw1 == 1u && sw1_prev == 0u);
	    bool sw2_rise = (sw2 == 1u && sw2_prev == 0u);
		
		// --- READY: select mode + start ---
	    if (system_state == SYSTEM_READY)
	    {
	        if (sw1_rise)
	        {
	            system_req = START_FOC;
	        }
	        if (sw2_rise)
	        {
	            system_req = START_BLOCK;
	        }
	    }
	
	    // --- RUN_FOC ---
	    if (system_state == SYSTEM_RUN_FOC)
	    {
	        if (sw1_rise)
	        {
				foc_switch_ctrl_methode();

	        }
	
	        if (sw2_rise)
	        {
				system_req = STOP_FOC;
	        }

	    }
	
	    // --- RUN_BLOCK ---
	    if (system_state == SYSTEM_RUN_BLOCK)
	    {
	        if (sw1_rise)
	        {
				
	        }
	        else if (sw2_rise)
	        {
				system_req = STOP_BLOCK;
	        }
	    }
	
	    sw1_prev = sw1;
	    sw2_prev = sw2;
	
	    // FSM progresses here
	    system_fsm_step();
				
		/* UART */
        char print_Array[37];
        float32_t data_Array[9] = {PrintVars.Duty*100.0, PrintVars.ibus, PrintVars.vbus, PrintVars.id, PrintVars.iq, PrintVars.we, PrintVars.we_hall, PrintVars.we_ref, PrintVars.we_smo};
        print_Array[0] = 0xAA;
        memcpy(&print_Array[1], data_Array, sizeof(data_Array));
        for (int i = 0; i < (int)sizeof(print_Array); i++) 
        { 
			printf("%c", print_Array[i]); 
		}
    }
}



/* [] END OF FILE */
