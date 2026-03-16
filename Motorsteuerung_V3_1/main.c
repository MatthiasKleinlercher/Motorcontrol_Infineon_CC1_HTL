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
#include "cyip_hppass.h"
#include "gpio_psc3_e_lqfp_80.h"
#include "mtb_hal.h"
#include "cybsp.h"
#include "psc3m5fds2afq1_s.h"
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <cy_retarget_io.h>
#include <math.h>
#include <stdlib.h>
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

// Communication
typedef struct
{
	float32_t target_speed; //[U/min]
	float32_t target_torque;	   // [Nm]
	uint8_t control_mode;
	uint8_t enable_motor;
	uint8_t start;
	uint8_t direction_cc;
	uint8_t foc;
	uint8_t sensorless;
}rx_t;

typedef struct
{
    uint16_t speed;         // speed *10
    uint8_t  torgue;        // torque *100 (aus iq/12)

    uint16_t i_bus;         // value *10
    uint16_t v_bus;         // value *10
    uint16_t pcb_temp;      // value *10  (noch nicht vorhanden -> 0)
    uint16_t winding_temp;  // value *10  (noch nicht vorhanden -> 0)
    uint16_t id;            // value *10
    uint16_t iq;            // value *10
    uint16_t vd;            // value *10
    uint16_t vq;            // value *10

    uint8_t  signal;        // bit0=error, bit1=warning
} tx_t;


// motor suite gui
typedef struct
{
    volatile uint8_t drive_enable;
    volatile int8_t  direction_cmd;     // +1 / -1
    volatile float   speed_ref_rpm;
    volatile uint8_t estop;

    volatile uint16_t state;
    volatile float speed_meas_rpm;
    volatile float id_meas;
    volatile float iq_meas;
    volatile float ia;
    volatile float ib;
    volatile float ic;
} motor_gui_if_t;

motor_gui_if_t g_gui = {0};

// Params for pre alignment
#define M_COARSE              (0.002f)
#define M_FINE                (0.003f)
#define M_POL                 (0.004f)

#define INJ_ON_CYCLES_COARSE  (300)
#define INJ_OFF_CYCLES_COARSE (400)

#define INJ_ON_CYCLES_FINE    (300)
#define INJ_OFF_CYCLES_FINE   (400)

#define INJ_ON_CYCLES_POL     (300)
#define INJ_OFF_CYCLES_POL    (400)

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
#define CONTROLLER_INTERRUPT_TIME 0.00016  // 160 µs

// Voltage supoply
#define VDD 36.0f

// Rated Current
#define Ir 35.0f

//Communication
#define DBG_STARTBYTE   (0xAAu)
#define DBG_WORDS       (8u)
#define DBG_PAYLOAD_LEN (DBG_WORDS * 4u)   // 32 Bytes

// print global vars
typedef struct
{
	float32_t ibus;
	float32_t vbus;
	float32_t id;
	float32_t iq;
	float32_t we_ref;
	float32_t we;
	float32_t we_hall;
	float32_t we_OBS;
	float32_t vd;
	float32_t vq;
	float32_t iq_ref;
	float32_t Duty;
}Print_t;
/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/

// ===== Control selection / FSM requests =====
typedef enum {	SYSTEM_OFF, SYSTEM_INIT, SYSTEM_ENABLE, SYSTEM_READY, 
				SYSTEM_STARTUP_FOC_PREALIGN, SYSTEM_STARTUP_FOC_OPENLOOP, SYSTEM_RUN_FOC, SYSTEM_SHUTDOWN_FOC_CMD, SYSTEM_SHUTDOWN_FOC,
				SYSTEM_STARTUP_BLOCK, SYSTEM_RUN_BLOCK, SYSTEM_SHUTDOWN_BLOCK_CMD, SYSTEM_SHUTDOWN_BLOCK, 
				SYSTEM_DISABLE, SYSTEM_FAULT} system_state_t;
volatile system_state_t system_state = SYSTEM_INIT;

typedef enum {	FOC_SENSORED, FOC_SENSORLESS} foc_type_t;
volatile foc_type_t foc_type = FOC_SENSORED;

typedef enum {	FOC_IQ_CONTROLLED, FOC_SPEED_CONTROLLED} foc_controller_type_t;
volatile foc_controller_type_t foc_controller_type = FOC_SPEED_CONTROLLED;

volatile bool counterClockwise = 0U; //CCW = true; CW = false;
volatile float32_t We_ref = 0.0f;
volatile float32_t Iq_ref = 0.0f;

typedef enum {	ENABLE_SYSTEM,
				DISABLE_SYSTEM,
				SWITCH_CTRL_METHODE,
				SWITCH_CONTROLLER_METHODE,
				START_FOC,
				START_BLOCK,
				STOP_FOC,
				STOP_BLOCK,
				ENABLE,
				DISABLE,
				NO_REQ }system_req_t;
volatile system_req_t system_req = NO_REQ;

// Interruptconfig
cy_stc_sysint_t Controller_counter_intr_config = { .intrSrc = Controller_Counter_IRQ, .intrPriority = 2U };
cy_stc_sysint_t Speed_Measurment_intr_config   = { .intrSrc = Speed_Measurment_Counter_IRQ, .intrPriority = 3U };
cy_stc_sysint_t hall_isr3_config               = { .intrSrc = ioss_interrupts_sec_gpio_8_IRQn, .intrPriority = 0U };
cy_stc_sysint_t hall_isr12_config              = { .intrSrc = ioss_interrupts_sec_gpio_9_IRQn, .intrPriority = 0U };
cy_stc_sysint_t fifo_isr_cfg             	   = { .intrSrc = pass_interrupt_fifos_IRQn, .intrPriority = 1U };
cy_stc_sysint_t pwm_reload_intr_config         = { .intrSrc = PWM_Counter_U_IRQ, .intrPriority =4U};
cy_stc_sysint_t Safety_intr_config			   = { .intrSrc = Safety_Counter_IRQ, .intrPriority = 5U};
cy_stc_sysint_t Uart_intr_config			   = { .intrSrc = Message_Counter_IRQ, .intrPriority = 6U};

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

// Communication
volatile rx_t rxData;
volatile tx_t txData;
static rx_t rxPrev = {0};

// Test Vars
volatile Print_t PrintVars;
float32_t alpha = 0.0;
/*******************************************************************************
* Functions
*******************************************************************************/
void enable();
void disable();
void init();
void pass_0_sar_0_fifo_0_buffer_0_callback();
void pwm_reload_intr_handler();
void controller_counter_intr_handler();
void speed_measurment_intr_handler();
void adc_calibration_start_lowside_clamp(uint32_t target_samples, uint32_t warmup_samples);
void adc_calibration_feed_lowside_clamp(const int16_t *data, const uint8_t *chan, uint8_t n);
void low_sides_all_on(void);
void high_sides_all_on(void);
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
uint8_t read_hall_state(void);
static inline int8_t _get_direction(uint8_t lut_idx, uint8_t prev_lut_idx);
static inline float32_t _get_correct_theta(uint8_t lut_idx, uint8_t prev_lut_idx);
void hall_interrupt_handler(void);
static inline float32_t _HALL_get_theta(const hall_meas_t* h);
static inline float32_t _HALL_get_we(hall_meas_t* hall);
static void init_pi_block_speed(PI_t *s);
static void system_fsm_step();
static inline void _Block(float32_t Duty, uint8_t lut_idx, bool direction_is_math_pos);
static inline void foc_switch_ctrl_methode();
static inline uint16_t le16(const uint8_t *p);
bool DebugUart_TryReadPacket(void);
static inline void wr16le(uint8_t *p, uint16_t v);
void DebugUart_SendTxPacket(void);
void sevSw_on();
void sevSw_off();
static inline bool _is_foc_group(system_state_t st);
static inline bool _is_block_group(system_state_t st);
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
* Function Name: static inline float32_t _dir_sign_f(void)
				 static inline bool _is_foc_group(system_state_t st)
				 static inline bool _is_block_group(system_state_t st)

* Helpers: State groups
*******************************************************************************/


static inline bool _is_foc_group(system_state_t st)
{
    return (st == SYSTEM_STARTUP_FOC_PREALIGN) || (st == SYSTEM_STARTUP_FOC_OPENLOOP) ||
           (st == SYSTEM_RUN_FOC) || (st == SYSTEM_SHUTDOWN_FOC_CMD) || (st == SYSTEM_SHUTDOWN_FOC);
}

static inline bool _is_block_group(system_state_t st)
{
    return (st == SYSTEM_STARTUP_BLOCK) || (st == SYSTEM_RUN_BLOCK) ||
           (st == SYSTEM_SHUTDOWN_BLOCK_CMD) || (st == SYSTEM_SHUTDOWN_BLOCK);
}

/******************************************************************************/

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
* Function Name: static inline void _Block(float32_t Duty, uint8_t lut_idx, bool direction_is_math_pos)

* -
*******************************************************************************/

static inline void _Block(float32_t Duty, uint8_t lut_idx, bool CounterClockwise)
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
	
	if (CounterClockwise == true)
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
	
    //system_state = SYSTEM_FAULT;
	
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
    
    // Back-calculation (k_aw ≈ ω_n)
    const float k_aw = 2.0f * 3.14159265f * 15.0f;
    smo->integ_e += (smo->Ki_pll * e_theta + (omega_limited - omega_u) * k_aw) * smo->Ts;
	
    smo->theta_hat = _wrap_pi(smo->theta_hat + smo->Ts * smo->omega_hat);
    smo->theta_rad = _wrap_pi(smo->theta_hat + smo->theta_shift);

    // w-Glättung (für Anzeige)
    smo->omega_hat_f += alpha*(smo->omega_hat - smo->omega_hat_f);//smo->alpha_omega * (smo->omega_hat - smo->omega_hat_f);
	PrintVars.we_OBS = smo->omega_hat_f;
	
    smo->i_alpha_prev = iAlpha;
    smo->i_beta_prev  = iBeta;
}


static inline float32_t _SMO_GetTheta(const smo_t* smo){ return smo->theta_rad; }
static inline float32_t _SMO_GetOmega(const smo_t* smo){ return smo->omega_hat_f; }
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

static inline int8_t _get_direction(uint8_t lut_idx, uint8_t prev_lut_idx)
{
    if (((int8_t)lut_idx - (int8_t)prev_lut_idx == 1) || ((int8_t)lut_idx - (int8_t)prev_lut_idx == -5)) return 1;
    else if (((int8_t)lut_idx - (int8_t)prev_lut_idx == -1) || ((int8_t)lut_idx - (int8_t)prev_lut_idx == 5)) return -1;
    else return 0;
}

static inline float32_t _get_correct_theta(uint8_t lut_idx, uint8_t prev_lut_idx)
{
    if ((lut_idx == 1 && prev_lut_idx == 0) || (lut_idx == 0 && prev_lut_idx == 1)) return lut_to_theta[0];
    else if ((lut_idx == 2 && prev_lut_idx == 1) || (lut_idx == 1 && prev_lut_idx == 2)) return lut_to_theta[1];
    else if ((lut_idx == 3 && prev_lut_idx == 2) || (lut_idx == 2 && prev_lut_idx == 3)) return lut_to_theta[2];
    else if ((lut_idx == 4 && prev_lut_idx == 3) || (lut_idx == 3 && prev_lut_idx == 4)) return lut_to_theta[3];
    else if ((lut_idx == 5 && prev_lut_idx == 4) || (lut_idx == 4 && prev_lut_idx == 5)) return lut_to_theta[4];
    else if ((lut_idx == 0 && prev_lut_idx == 5) || (lut_idx == 5 && prev_lut_idx == 0)) return lut_to_theta[5];
    else return 0.0f;
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
	
	
	hall_values.lut_idx    = lut_index;
	hall_values.direction  = _get_direction(lut_index, hall_values.prev_lut_idx);
	hall_values.theta_sector = lut_to_theta_sector[lut_index];
	
	hall_values.correct_theta = _get_correct_theta(lut_index, hall_values.prev_lut_idx);
	hall_values.correct_angle_set = (hall_values.correct_theta != 0.0f);
	
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
                hall_values.we_hall = we_mag;
                PrintVars.we_hall = we_mag;
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
	const float32_t vq_boost = 4.5;
	const float32_t we_ramp = 0.0009; //[V/rad/s]
	
	*vd = 0.0;
	*vq = vq_boost + we_ramp * we;
	if (counterClockwise == true) *vq = -*vq;
	if (fabsf(*vq) < 0.0) *vq = 0.0;
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
	const float32_t OL_WE = 200.0;
	const float32_t OL_DUTY = 0.2;
	const float32_t CALL_TIME = CONTROLLER_INTERRUPT_TIME;
	
	float32_t we_ref = _sat(We_ref, 200.0f, 628.0f);
	
	// =========================
    // BLOCK
    // =========================
	
	if (system_state == SYSTEM_STARTUP_BLOCK || system_state == SYSTEM_RUN_BLOCK || system_state == SYSTEM_SHUTDOWN_BLOCK_CMD || system_state == SYSTEM_SHUTDOWN_BLOCK)
	{
		static PI_t c;
		static float32_t duty = 0.0;
		static float32_t we_m = 0.0;
		if(system_state != SYSTEM_STARTUP_BLOCK && system_state == SYSTEM_SHUTDOWN_BLOCK) we_m = _HALL_get_we(&hall_values);
		if (system_state == SYSTEM_STARTUP_BLOCK)
		{
			we_m += OL_WE * CALL_TIME/OL_TIME;
			const float32_t duty_boost = 0.15;
			const float32_t duty_ramp = 0.0009;
			duty = duty_boost + duty_ramp*we_m;
			
			if (we_m >= OL_DUTY) 
			{
				init_pi_block_speed(&c);
				system_state = SYSTEM_RUN_BLOCK;
			}
		}
		
		else if (system_state == SYSTEM_RUN_BLOCK || system_state == SYSTEM_SHUTDOWN_BLOCK_CMD)
	    {	
			//if (we_m < OL_WE * 0.8f && system_state == SYSTEM_RUN_BLOCK) system_state = SYSTEM_SHUTDOWN_BLOCK_CMD;
			
	        if (system_state == SYSTEM_SHUTDOWN_BLOCK_CMD) 
	        {
				we_ref = OL_WE;
				if (fabsf(we_m) < (OL_WE + 10.0f)) system_state = SYSTEM_SHUTDOWN_BLOCK;
			}
	        duty =  _pi_controller_step(&c, we_ref, we_m);
	    }
		
		else if (system_state == SYSTEM_SHUTDOWN_BLOCK)
		{
			we_m -= OL_WE * CALL_TIME/OL_TIME;
			const float32_t duty_boost = 0.15;
			const float32_t duty_ramp = 0.0009;
			duty = duty_boost + duty_ramp*we_m;
			if (we_m <= 0.0)
		    {
				duty = 0.0;
				we_m = 0.0;
		        system_state = SYSTEM_READY;
		    }
		}
		
		_Block(duty, hall_values.lut_idx, counterClockwise);
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
		static float32_t iq_ref = 0.01;
		
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
			
			//if (we < OL_WE * 0.8f && system_state == SYSTEM_RUN_FOC) system_state = SYSTEM_SHUTDOWN_FOC_CMD;
			
			c = cosf(th);
			s = sinf(th);
			_park_dq(ialpha, ibeta, s, c, &id, &iq);
			
		    static uint8_t count = 10;
		    if (count >= 9)
		    {
				if (system_state == SYSTEM_SHUTDOWN_FOC_CMD) 
				{
					we_ref = OL_WE;
					if (fabsf(we) <= (OL_WE + 10.0f)) system_state = SYSTEM_SHUTDOWN_FOC;
				}
		        if (foc_controller_type == FOC_SPEED_CONTROLLED)
				{
				    iq_ref = _pi_controller_step(&speed, we_ref, we);
				}
		        count = 0;
		    }
		    else count++;
			
			if (foc_controller_type == FOC_IQ_CONTROLLED) iq_ref = Iq_ref;
			
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
		PrintVars.iq += alpha*(iq - PrintVars.iq);
		PrintVars.vq = vq;
		PrintVars.we_ref = we_ref;
		PrintVars.iq_ref = iq_ref;
		
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
	d->ki 			= 406.5;
	d->kp			= 0.0015;
	d->out_max		= VDD/2.0;
	d->out_min		= -VDD/2.0;
	d->Ts			= CONTROLLER_INTERRUPT_TIME;
	d->prevE		= 0.0;
	d->prevU		= vd;
	d->useIntFlag	= true;
}

void init_pi_q(PI_t *q, float32_t vq)
{
	q->ki 			= 406.5;
	q->kp			= 0.0015;
	q->out_max		= VDD/2.0;
	q->out_min		= 0.001;
	q->Ts			= CONTROLLER_INTERRUPT_TIME;
	q->prevE		= 0.0;
	q->prevU		= vq;
	q->useIntFlag	= true;
}

void init_pi_speed(PI_t *s)
{
	s->ki 			= 0.9;
	s->kp			= 0.3;
	s->out_max		= 70.0;
	s->out_min		= 0.001;
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
    preAlignment.inj_m = M_COARSE;
    _prepare_coarse(&preAlignment);
    
    Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
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
                system_state = SYSTEM_STARTUP_FOC_OPENLOOP;
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

		float32_t iu = -0.0189f * (float32_t)adc_data[0] + 38.352;
		float32_t iv = -0.0184f * (float32_t)adc_data[1] + 37.684;
		float32_t iw = -0.0172f * (float32_t)adc_data[2] + 35.205;
		float32_t offset = (iu + iv + iw) / 3.0;
		iu -= offset;iv -= offset; iw -= offset;
		adc_meas.iu = iu; adc_meas.iv = iv; adc_meas.iw = iw;
		adc_meas.idc = 0.028 * (float32_t)adc_data[3] - 57.082;
		
		float32_t vu = 0.022 * (float32_t)adc_data[4] - 0.2378;
		float32_t vv = 0.0219 * (float32_t)adc_data[5] - 0.1068;
		float32_t vw = 0.0219 * (float32_t)adc_data[6] - 0.1021;
		offset = (vu + vv + vw) / 3.0;
		vu -= offset; vv -= offset; vw -= offset;
		adc_meas.vu = vu; adc_meas.vv = vu; adc_meas.vu = vw;
		adc_meas.vdc = 0.022 * (float32_t)adc_data[7] - 0.1133;
		
		// Tester:
		PrintVars.ibus = adc_meas.idc;
		PrintVars.vbus = adc_meas.vdc;
		
		
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
* Function Name: void sevSw_on()
				 void sevSw_off()

* Enables/disables half bridge
*******************************************************************************/
void sevSw_on()
{
	uint32_t SevSw_PeriodVal = Cy_TCPWM_PWM_GetPeriod0(SevSw_Counter_HW, SevSw_Counter_NUM);
	Cy_TCPWM_PWM_SetCompare0Val(SevSw_Counter_HW, SevSw_Counter_NUM, SevSw_PeriodVal);
	
	Cy_TCPWM_TriggerStart_Single(SevSw_Counter_HW, SevSw_Counter_NUM);
}

void sevSw_off()
{
	Cy_TCPWM_PWM_SetCompare0Val(SevSw_Counter_HW, SevSw_Counter_NUM, 0);
	
	Cy_TCPWM_TriggerStart_Single(SevSw_Counter_HW, SevSw_Counter_NUM);
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static void system_fsm_step()


*******************************************************************************/
static void system_fsm_step()
{
	switch (system_state) 
	{
		case SYSTEM_OFF:
		{
			if (system_req == ENABLE) system_state = SYSTEM_ENABLE;
		}break;
		
		case SYSTEM_INIT:
		{
			init();
			system_state = SYSTEM_OFF;
		}break;
		
		case SYSTEM_ENABLE:
		{
			enable();
			sevSw_on();
			high_sides_all_on();
			system_state = SYSTEM_READY;
			system_req = NO_REQ;
			
		}break;
		
		case SYSTEM_READY:
		{
			if (system_req == START_FOC) 
			{
				Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
				Cy_TCPWM_TriggerStart_Single(Controller_Counter_HW, Controller_Counter_NUM);
				SMO_Init(&SMO);
				_prealign_start();
                NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);
				system_state = SYSTEM_STARTUP_FOC_PREALIGN;
				system_req = NO_REQ;
			}
			else if (system_req == START_BLOCK)
			{
				Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
				Cy_TCPWM_TriggerStart_Single(Controller_Counter_HW, Controller_Counter_NUM);
                NVIC_EnableIRQ(Controller_counter_intr_config.intrSrc);
				system_state = SYSTEM_STARTUP_BLOCK;
				system_req = NO_REQ;
			}
			
			else if (system_req == DISABLE) system_state = SYSTEM_DISABLE;
		}break;
		
		case SYSTEM_STARTUP_FOC_PREALIGN:
		{}break;
		
		case SYSTEM_STARTUP_FOC_OPENLOOP:
		{}break;
		
		case SYSTEM_RUN_FOC:
		{
			if (system_req == STOP_FOC)
			{
				system_state = SYSTEM_SHUTDOWN_FOC_CMD;
				system_req = NO_REQ;
			}
			else if(system_req == START_BLOCK)
			{
				system_state = SYSTEM_SHUTDOWN_FOC_CMD;
			}
			
			else if(system_req == SWITCH_CTRL_METHODE)
			{
				foc_switch_ctrl_methode();
			}
			
			else if(system_req == SWITCH_CONTROLLER_METHODE)
			{
				if (foc_controller_type == FOC_IQ_CONTROLLED) foc_controller_type = FOC_SPEED_CONTROLLED;
				else if (foc_controller_type == FOC_SPEED_CONTROLLED) foc_controller_type = FOC_IQ_CONTROLLED;				
			}
		}break;
		
		case SYSTEM_SHUTDOWN_FOC_CMD:
		{}break;
		
		case SYSTEM_SHUTDOWN_FOC:
		{}break;	
		
		case SYSTEM_STARTUP_BLOCK:
		{}break;
		
		case SYSTEM_RUN_BLOCK:
		{
			if (system_req == STOP_BLOCK)
			{
				system_state = SYSTEM_SHUTDOWN_BLOCK_CMD;
				system_req = NO_REQ;
			}
			else if(system_req == START_FOC)
			{
				system_state = SYSTEM_SHUTDOWN_BLOCK_CMD;
			}
		}break;
		
		case SYSTEM_SHUTDOWN_BLOCK_CMD:
		{}break;
		
		case SYSTEM_SHUTDOWN_BLOCK:
		{}break;
		
		case SYSTEM_DISABLE:
		{
			sevSw_off();
			system_state = SYSTEM_OFF;
			system_req = NO_REQ;
		}break;
		
		case SYSTEM_FAULT:
		{
			if (system_state == SYSTEM_RUN_FOC) system_state = SYSTEM_SHUTDOWN_FOC_CMD;
			else if (system_state == SYSTEM_RUN_BLOCK) system_state = SYSTEM_SHUTDOWN_BLOCK_CMD;
			else system_state = SYSTEM_DISABLE;
		}break;

	}
}
/******************************************************************************/

/*******************************************************************************
* Function Name: static inline uint16_t le16(const uint8_t *p)
				 bool DebugUart_TryReadPacket(void)
				 static inline void wr16le(uint8_t *p, uint16_t v)
				 static inline void wr32le(uint8_t *p, uint32_t v)
				 void DebugUart_SendTxPacket(void)
				 
* That functions are your to transmit und recieve message between µC and Motorpanel
*******************************************************************************/
static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool DebugUart_TryReadPacket(void)
{
    typedef enum
    {
        RX_WAIT_CMD = 0,
        RX_READ_DATA
    } rx_parse_state_t;

    static rx_parse_state_t state = RX_WAIT_CMD;
    static uint8_t cmd = 0u;
    static uint8_t payload[2];
    static uint8_t idx = 0u;
    static uint8_t expected_len = 0u;

    bool updated = false;

    while (Cy_SCB_UART_GetNumInRxFifo(DEBUG_UART_HW) != 0u)
    {
        uint8_t b = (uint8_t)Cy_SCB_UART_Get(DEBUG_UART_HW);

        if (state == RX_WAIT_CMD)
        {
            cmd = b;
            idx = 0u;

            switch (cmd)
            {
                case 0xA0:  // speed -> 16 Bit
                    expected_len = 2u;
                    state = RX_READ_DATA;
                    break;
                    
                case 0xA1:  // torque -> 8 Bit
                    expected_len = 1u;
                    state = RX_READ_DATA;
                    break;

                case 0xAA:  // packed flags -> 8 Bit
                    expected_len = 1u;
                    state = RX_READ_DATA;
                    break;

                default:
                    // unbekanntes Startbyte ignorieren
                    state = RX_WAIT_CMD;
                    break;
            }
        }
        else // RX_READ_DATA
        {
            payload[idx++] = b;

            if (idx >= expected_len)
            {
                switch (cmd)
                {
                    case 0xA0:
                        rxData.target_speed = (float32_t)(le16(payload))/10.0;
                        updated = true;
                        
                        break;

                    case 0xA1:
					    rxData.target_torque = (float32_t)(payload[0]) / 100.0f;
					    updated = true;
					    break;

                    case 0xAA:
                    {
                        uint8_t flags = payload[0];

                        // bit0 = ControlMode (1 = speed, 0 = torque)
                        // bit1 = EnableMotor
                        // bit2 = StartStop
                        // bit3 = Direction
                        // bit4 = FOC/Block
                        // bit5 = SensorMode
                        // bit6..7 reserved

                        rxData.control_mode = (flags & (1u << 0)) ? 1u : 0u;
                        rxData.enable_motor = (flags & (1u << 1)) ? 1u : 0u;
                        rxData.start        = (flags & (1u << 2)) ? 1u : 0u;
                        rxData.direction_cc = (flags & (1u << 3)) ? 1u : 0u;
                        rxData.foc          = (flags & (1u << 4)) ? 1u : 0u;
                        rxData.sensorless   = (flags & (1u << 5)) ? 1u : 0u;
						
                        updated = true;
                    }
                    break;

                    default:
                        break;
                }

                // für nächstes Telegramm zurücksetzen
                state = RX_WAIT_CMD;
                idx = 0u;
                expected_len = 0u;
            }
        }
    }
    return updated;
}

static inline void wr16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static inline uint16_t _u16_from_x10(float32_t x)
{
    if (x <= 0.0f) return 0u;
    float32_t y = x * 10.0f + 0.5f;
    if (y > 65535.0f) y = 65535.0f;
    return (uint16_t)y;
}

static inline uint8_t _u8_from_x100(float32_t x)
{
    if (x <= 0.0f) return 0u;
    float32_t y = x * 100.0f + 0.5f;
    if (y > 255.0f) y = 255.0f;
    return (uint8_t)y;
}

static void DebugUart_UpdateTxData(void)
{
    // Snapshot (kurz IRQ aus, damit keine “halb” geschriebenen floats)
    float32_t we, ibus, vbus, id, iq, vd, vq;
    system_state_t st;
    bool err, warn;
    float32_t pole_pairs;

    we   = PrintVars.we;      // electrical rad/s (aus ISR gesetzt)
    ibus = PrintVars.ibus;    // A
    vbus = PrintVars.vbus;    // V
    id   = PrintVars.id;      // A (FOC)
    iq   = PrintVars.iq;      // A (FOC)
    pole_pairs = (SMO.p > 0.0f) ? SMO.p : 1.0f;

    // Speed aus we -> rpm (mechanisch): rpm = we/(2*pi*p) * 60
    float32_t speed_rpm = (fabsf(we) / (TWO_PI * pole_pairs)) * 60.0f;

    // torque = iq / 12 (nur sinnvoll in FOC)
    float32_t torque_nm = 0.0f;

    bool foc_valid = _is_foc_group(st);
    if (foc_valid)
    {
        torque_nm = fabsf(iq) / 12.0f;
    }
    else
    {
        // Block: iq/torque nicht verfügbar -> 0
        id = 0.0f; iq = 0.0f; vd = 0.0f; vq = 0.0f;
    }

    // txData füllen (deine Skalierungsvorgaben)
    txData.speed = _u16_from_x10(speed_rpm);
    txData.torgue = _u8_from_x100(torque_nm);

    txData.i_bus = _u16_from_x10(fabsf(ibus));
    txData.v_bus = _u16_from_x10(fabsf(vbus));

    // Temperaturen sind im Code noch nicht vorhanden
    txData.pcb_temp = 0u;
    txData.winding_temp = 0u;

    txData.id = _u16_from_x10(fabsf(id));
    txData.iq = _u16_from_x10(fabsf(iq));
    txData.vd = _u16_from_x10(fabsf(vd));
    txData.vq = _u16_from_x10(fabsf(vq));

    // signal bits
    uint8_t sig = 0u;
    if (err)  sig |= (1u << 0);   // LSB = error
    if (warn) sig |= (1u << 1);   // bit1 = warning
    txData.signal = sig;
}

void DebugUart_SendTxPacket(void)
{
    // txData aus aktuellen Laufwerten berechnen
    DebugUart_UpdateTxData();

    const uint8_t TX_PAYLOAD_LEN = 20U;
    uint8_t buf[1u + TX_PAYLOAD_LEN];
    uint8_t i = 0u;

    buf[i++] = DBG_STARTBYTE;

    wr16le(&buf[i], txData.speed);        i += 2u;   // uint16_t
    buf[i++] = txData.torgue;                       // uint8_t

    wr16le(&buf[i], txData.i_bus);        i += 2u;
    wr16le(&buf[i], txData.v_bus);        i += 2u;
    wr16le(&buf[i], txData.pcb_temp);     i += 2u;
    wr16le(&buf[i], txData.winding_temp); i += 2u;
    wr16le(&buf[i], txData.id);           i += 2u;
    wr16le(&buf[i], txData.iq);           i += 2u;
    wr16le(&buf[i], txData.vd);           i += 2u;
    wr16le(&buf[i], txData.vq);           i += 2u;

    buf[i++] = txData.signal;

    Cy_SCB_UART_PutArray(DEBUG_UART_HW, buf, i);
}


void Poti_read(void)
{

    int32_t raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(12U);
    We_ref = (float32_t)raw_value_idPot / 4095.0f * 428.0 + 200.0;
    
    raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(10U);
	alpha = (float32_t)raw_value_idPot / 4095.0f;

}

/*******************************************************************************
* Function Name: void enable()

* enables all parts.
* Has to be called at the beginning.
*******************************************************************************/
void enable()
{
	Cy_TCPWM_Counter_Enable(Test_Counter_HW, Test_Counter_NUM);
	Cy_TCPWM_Counter_Enable(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
    NVIC_EnableIRQ(Speed_Measurment_intr_config.intrSrc);
    Cy_TCPWM_Counter_Enable(Safety_Counter_HW, Safety_Counter_NUM);
    NVIC_EnableIRQ(Safety_intr_config.intrSrc);
    Cy_TCPWM_Counter_Enable(Controller_Counter_HW, Controller_Counter_NUM);
    Cy_TCPWM_PWM_Enable(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_PWM_Enable(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_PWM_Enable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
    Cy_TCPWM_PWM_Enable(SevSw_Counter_HW, SevSw_Counter_NUM);
    Cy_TCPWM_Counter_Enable(Message_Counter_HW, Message_Counter_NUM);
    NVIC_EnableIRQ(hall_isr12_config.intrSrc);
    NVIC_EnableIRQ(hall_isr3_config.intrSrc);
    NVIC_EnableIRQ(fifo_isr_cfg.intrSrc);
}

/******************************************************************************/

/*******************************************************************************
* Function Name: void enable()

* enables all parts.
* Has to be called at the beginning.
*******************************************************************************/
void disable()
{
	Cy_TCPWM_Counter_Disable(Test_Counter_HW, Test_Counter_NUM);
	Cy_TCPWM_Counter_Disable(Speed_Measurment_Counter_HW, Speed_Measurment_Counter_NUM);
    NVIC_DisableIRQ(Speed_Measurment_intr_config.intrSrc);
    Cy_TCPWM_Counter_Disable(Safety_Counter_HW, Safety_Counter_NUM);
    NVIC_DisableIRQ(Safety_intr_config.intrSrc);
    Cy_TCPWM_Counter_Disable(Controller_Counter_HW, Controller_Counter_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
    Cy_TCPWM_PWM_Disable(SevSw_Counter_HW, SevSw_Counter_NUM);
    Cy_TCPWM_Counter_Disable(Message_Counter_HW, Message_Counter_NUM);
    NVIC_DisableIRQ(hall_isr12_config.intrSrc);
    NVIC_DisableIRQ(hall_isr3_config.intrSrc);
    NVIC_DisableIRQ(fifo_isr_cfg.intrSrc);
}

/******************************************************************************/

/*******************************************************************************
* Function Name: void init(void)

* that function initalizes all essential componants to run the system.
* Has to be called at the beginning.
*******************************************************************************/
void init()
{
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

    /* Speed Counter init, used calculate speed in Blockkommuation, sensored FOC */
	if(result != Cy_TCPWM_Counter_Init(Speed_Measurment_Counter_HW,Speed_Measurment_Counter_NUM, &Speed_Measurment_Counter_config)) { CY_ASSERT(0); }
    Cy_SysInt_Init(&Speed_Measurment_intr_config, speed_measurment_intr_handler);
    
    /* Safety Counter turns off the machine if something goes wrong */
   	if(result != Cy_TCPWM_Counter_Init(Safety_Counter_HW,Safety_Counter_NUM, &Safety_Counter_config)) { CY_ASSERT(0); }
	Cy_SysInt_Init(&Safety_intr_config, safety_intr_handler);

    /* Controller Counter init, used for PI Controller */
	if(result != Cy_TCPWM_Counter_Init(Controller_Counter_HW,Controller_Counter_NUM, &Controller_Counter_config)) { CY_ASSERT(0); }
    Cy_SysInt_Init(&Controller_counter_intr_config, controller_counter_intr_handler);

    /* PWM U/V/W init */
	if (result != Cy_TCPWM_PWM_Init(PWM_Counter_U_HW, PWM_Counter_U_NUM, &PWM_Counter_U_config)) { CY_ASSERT(0); }
    // PWM Terminalcount Interrupt for rotor pre alligment
    Cy_SysInt_Init(&pwm_reload_intr_config, pwm_reload_intr_handler);
	if (result != Cy_TCPWM_PWM_Init(PWM_Counter_V_HW, PWM_Counter_V_NUM, &PWM_Counter_V_config)) { CY_ASSERT(0); }
	if (result != Cy_TCPWM_PWM_Init(PWM_Counter_W_HW, PWM_Counter_W_NUM, &PWM_Counter_W_config)) { CY_ASSERT(0); }
    
	if (result != Cy_TCPWM_PWM_Init(SevSw_Counter_HW, SevSw_Counter_NUM, &SevSw_Counter_config)) { CY_ASSERT(0); }
    
    /*for sending Messages to Motorpanel*/
    if(result != Cy_TCPWM_Counter_Init(Message_Counter_HW,Message_Counter_NUM, &Message_Counter_config)) { CY_ASSERT(0); }
    Cy_SysInt_Init(&Uart_intr_config, DebugUart_SendTxPacket);
    //NVIC_EnableIRQ(Uart_intr_config.intrSrc);

    /* Halls & Button init*/
    Cy_GPIO_Pin_Init(HALL1_PORT, HALL1_PIN, &HALL1_config);
    Cy_GPIO_Pin_Init(HALL2_PORT, HALL2_PIN, &HALL2_config);
    Cy_GPIO_Pin_Init(HALL3_PORT, HALL3_PIN, &HALL3_config);
    Cy_SysInt_Init(&hall_isr12_config, hall_interrupt_handler);
    Cy_SysInt_Init(&hall_isr3_config, hall_interrupt_handler);
    
    /* Fan PWM Pin*/
    Cy_GPIO_Pin_Init(FAN_PWM_PORT, FAN_PWM_PIN, &FAN_PWM_config);
    
    /* Fan PWM Pin*/
    Cy_GPIO_Pin_Init(ERROR_PORT, ERROR_PIN, &ERROR_config);
    
    Cy_GPIO_Pin_Init(SW1_PORT, SW1_NUM, &SW1_config);
    Cy_GPIO_Pin_Init(SW2_PORT, SW2_NUM, &SW2_config);

    /* ADC FIFO ISR */
    Cy_HPPASS_FIFO_SetInterruptMask(CY_HPPASS_INTR_FIFO_0_LEVEL);
    Cy_SysInt_Init(&fifo_isr_cfg, pass_0_sar_0_fifo_0_buffer_0_callback);
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
	// Test:
	foc_controller_type = FOC_SPEED_CONTROLLED;
	foc_type = FOC_SENSORED;
	Iq_ref = 0.0;

	uint8_t sw1_prev = 1u;
    uint8_t sw2_prev = 1u;
    static uint8_t count = 0U;
	
    for (;;)
    {
        uint8_t sw1 = (uint8_t)Cy_GPIO_Read(SW1_PORT, SW1_NUM);
        uint8_t sw2 = (uint8_t)Cy_GPIO_Read(SW2_PORT, SW2_NUM);

        bool sw1_press = (sw1_prev == 1u) && (sw1 == 0u);
        bool sw2_press = (sw2_prev == 1u) && (sw2 == 0u);

        sw1_prev = sw1;
        sw2_prev = sw2;
        
        if (sw1_press)
        {
			count++;
			if(count > 3U) count = 0U;
		}
		
		if (system_req == NO_REQ)
		{
			switch (system_state) 
			{
				case SYSTEM_OFF:
				{
					if (count == 0U && sw2_press) system_req = ENABLE;
				}break;
				
				case SYSTEM_READY:
				{
					if (count == 0U && sw2_press) system_req = START_FOC;
					else if (count == 1U && sw2_press) system_req = START_BLOCK;
					else if (count == 2U && sw2_press) system_req = DISABLE;
				}break;
				
				case SYSTEM_RUN_FOC:
				{
					if (count == 0U && sw2_press) system_req = STOP_FOC;
					else if (count == 1U && sw2_press) system_req = SWITCH_CONTROLLER_METHODE;
					else if (count == 2U && sw2_press) system_req = SWITCH_CTRL_METHODE;
					else if (count == 3U && sw2_press) system_req = START_BLOCK;
				}break;
				
				case SYSTEM_RUN_BLOCK:
				{
					if (count == 0U && sw2_press) system_req = STOP_BLOCK;
					else if (count == 1U && sw2_press) system_req = START_BLOCK;
				}break;
			}
		}

		
		
		
	    system_fsm_step();
	    //DebugUart_SendTxPacket();
	    char print_Array[41];
    	float32_t data_Array[10] = {PrintVars.we_ref, PrintVars.we, PrintVars.iq_ref, PrintVars.iq, PrintVars.vq, PrintVars.vbus, PrintVars.ibus, (float32_t)count, PrintVars.we_hall, PrintVars.we_OBS};
    	print_Array[0] = 0xAA;
    	memcpy(&print_Array[1], data_Array, sizeof(data_Array));
    	for (int i = 0; i < (int)sizeof(print_Array); i++) 
    	{ 
			printf("%c", print_Array[i]); 
		}
    }
}



/* [] END OF FILE */
