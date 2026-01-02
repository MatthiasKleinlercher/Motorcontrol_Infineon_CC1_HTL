/// Green U
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

#define MAX_SAMPLES_PER_PHASE               64

#define sqrt3                               1.732050808f
#define TWO_M_PI                            6.283185307f
#define ONE_M_PI_THIRD                      1.04719755f
#define TWO_M_PI_THIRD                      2.094395102f
#define PI_OVER_TWO							1.57079632679489
#define TWO_OVER_PI							0.63661977236758
#define FOUR_M_PI_THIRD                     4.188790205f
#define sqrt3o2                             0.8660254038f
#define sqrt2								1.41421356237
#define k2o3                                0.666666667f
#define Vdc                                 36.0f
#define Ir									3.0f
#define ONE_o_SQRT3                         0.57735026919f
#define M_PIo30                             0.1047197551197f
#define PI									3.1415926536
		
#define C30                                 0.994521895f
#define S30                                 0.104528463f

#define TWO_PIo_1250                        0.0050265f

/* Injection Timing @ 25 kHz (1 Periode = 40us) */
#define INJ_ON_CYCLES_COARSE                5      /* 200 us on */
#define INJ_OFF_CYCLES_COARSE_HIZ           10      /* 240..320 us off if High-Z; increase if needed */
#define INJ_OFF_CYCLES_COARSE_LS            12     /* 480 us off if Low-Side-ON */

#define INJ_ON_CYCLES_FINE                  5
#define INJ_OFF_CYCLES_FINE_HIZ             10
#define INJ_OFF_CYCLES_FINE_LS              12

#define INJ_ON_CYCLES_POL                   10
#define INJ_OFF_CYCLES_POL_HIZ              10
#define INJ_OFF_CYCLES_POL_LS               15

#define M_COARSE                            0.1f
#define M_FINE                              0.1f
#define M_POL                               0.1f

#define OMEGA_LOCK_MIN     400.0f      // ab ~64 el. Hz handover zulässig
#define OBS_READY_COUNT    100         // 100 * 0.2ms = 20 ms stabil

/*******************************************************************************
 * Globals
 *******************************************************************************/

float32_t w_e = 0.0;
float32_t w_e_ref = 0.0f;

int16_t adc_sar_result[16] = {0,0,0, 0, 0, 0,0};

uint16_t adc_current_result[16];
uint8_t  chanIdx0 = 0;
uint8_t  chanIdx1 = 0;
uint8_t  chanIdx2 = 0;

float32_t I_d = 0.0f;
float32_t I_q = 0.0f;
float32_t I_d_ref = 0.0f;
float32_t I_q_ref = 0.0f;
float32_t vq = 0.0f;
float32_t vd = 0.0f;
//float32_t I_alpha = 0.0f, I_beta = 0.0f;

typedef struct{
	float32_t ialpha;
	float32_t ibeta;
	float32_t valpha;
	float32_t vbeta;
	float32_t vdc;
}clarke_vals_t;

volatile clarke_vals_t measValues = 
{
	.ialpha = 0.0,
	.ibeta = 0.0,
	.valpha = 0.0,
	.vbeta = 0.0,
	.vdc = 0.0
};

float32_t v_alpha = 0.0f;
float32_t v_beta  = 0.0f;
float32_t sin_ra  = 0.0f, cos_ra = 1.0f;

float32_t Duty_U = 0.0f, Duty_V = 0.0f, Duty_W = 0.0f;

uint8_t button_pressed_before = 0U;

static uint32_t samples_for_offset = 0;
static uint32_t offset_acc_u = 0, offset_acc_v = 0, offset_acc_w = 0;
static uint32_t adc_off_u = 2048, adc_off_v = 2048, adc_off_w = 2048;

/* PI Controller */
typedef struct { float32_t kp, ki, integrator, prevI; } pi_t;
typedef struct {
    pi_t d, q;
    float32_t Ts;
} foc_curr_ctrl_t;

static foc_curr_ctrl_t Ictrl =
{
    .d = {.kp = 1.1f, .ki = 5.4f, .integrator = 0.0, .prevI = 0.0},
    .q = {.kp = 1.1f, .ki = 5.4f, .integrator = 0.0, .prevI = 0.0},
    .Ts = 0.00004f
};


typedef struct {
	float32_t ki;
	float32_t kp;
	float32_t integrator;
	float32_t w_e_tgt;
	float32_t Ts;
}foc_speed_ctrl_t;

static foc_speed_ctrl_t Speed_Control = 
{
	.integrator = 0.0,
	.ki = 0.00,
	.kp = 0.019f,
	.w_e_tgt = 400.0,
	.Ts = 0.00004f
};

/* Motorparameter */
static const float32_t Rs = 0.035f;        // Ohm
static const float32_t Ld = 86e-6f;        // H
static const float32_t Lq = 86e-6f;        // H
static const float32_t Ls = 86e-6f;        // H
static const float32_t Pp = 4.0;

typedef enum {FOC_ADC_CALIBRATION, FOC_POSITION_ESTIMATION, FOC_READY_TO_START, FOC_RUNNING_OPEN_LOOP, FOC_RUNNING_OL_TO_CL, FOC_RUNNING_CLOSED_LOOP, FOC_OFF } block_state_t;
volatile block_state_t foc_state = FOC_OFF;

typedef enum { FIRST_STEP_ESTIMATION, SECOND_STEP_ESTIMATION} rotor_pos_est_t;
volatile rotor_pos_est_t foc_init = FIRST_STEP_ESTIMATION;

/* --- Injection State Machine --- */
typedef enum { INJ_IDLE, INJ_NEXT, INJ_ON, INJ_OFF, INJ_DONE, INJ_POL_A, INJ_POL_B, INJ_POL_DONE } inj_state_t;
static volatile inj_state_t inj_state = INJ_IDLE;

static int  inj_on_cycles = 0, inj_off_cycles = 0;
static int  inj_idx = 0;                 // Index im aktuellen Winkelsatz
static float32_t inj_m = M_COARSE;           // Modulationsgrad
static float32_t inj_theta_deg = 0.0f;

static const float theta_coarse[12] = {0,30,60,90,120,150,180,210,240,270,300,330};
static float32_t theta_list[12];             // aktiver Satz (12 oder 6 Einträge)
static int   theta_count = 0;

static float32_t peaks[12];                  // Peaks für den aktuellen Satz
static float32_t theta_est_deg = 0.0f;
static bool  theta_polar_is_north = false;  // true: θ zeigt auf N-Pol

/* Messwert aus ADC-ISR */
volatile bool inj_collect_now = false;
volatile bool have_sample = false;
volatile float32_t last_imag = 0.0f;

uint32_t period_PWM = 0;
uint32_t prevCountValueSpeMeaCou = 0;

static float32_t theta_e_rad     = 0.0f;     // elektrischer Winkel [rad]

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

typedef struct
{
	float32_t v_boost;
	float32_t w_e_target;
	float32_t t_start;
	float32_t Kvf;
	pi_t d, q;
}ols_t;
ols_t OLS;

static float32_t a_th = 0.0f;         // 0..1 Blend-Faktor
const  float32_t Tblend_th = 10.0f;   // z.B. 30 ms

// Hilfsvariablen
float32_t t_ol = 0.0;
float32_t alpha = 0.0;

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

static inline float32_t _wrap_pi(float32_t a)
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

static inline float32_t unwrap_delta(float32_t th_now, float32_t th_prev)
{
    // kleinstes Winkelinkrement in [-pi, +pi]
    float32_t d = th_now - th_prev;
    if (d >  (float32_t)PI) d -= 2.0f*(float32_t)PI;
    if (d < -(float32_t)PI) d += 2.0f*(float32_t)PI;
    return d;
}


/*******************************************************************************
 * Helpers & math
 *******************************************************************************/

static inline void outputs_all_off(void)
{      
    Cy_TCPWM_PWM_Disable(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_PWM_Disable(PWM_Counter_W_HW, PWM_Counter_W_NUM);
}

static inline void low_sides_all_on(void)
{
	
    Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_U_HW, PWM_Counter_U_NUM, 0);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_V_HW, PWM_Counter_V_NUM, 0);
    Cy_TCPWM_PWM_SetCompare0Val(PWM_Counter_W_HW, PWM_Counter_W_NUM, 0);
    
    Cy_TCPWM_TriggerStart_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    Cy_TCPWM_TriggerStart_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
    Cy_TCPWM_TriggerStart_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);
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
cy_stc_sysint_t pwm_reload_intr_config         = { .intrSrc = PWM_Counter_U_IRQ, .intrPriority =6U};


//===========================
// Slide Mode Observer
//===========================

void SMO_Init(smo_t *smo, float32_t rs, float32_t ls, float32_t ts, float32_t pol_pairs)
{
	smo->R = rs;
    smo->L = ls;
    smo->fs = 1.0/ts;
    smo->Ts = ts;
    smo->p  = pol_pairs;
    
    smo->ealpha_f = 0.0f;  smo->ebeta_f = 0.0f;
    smo->i_alpha_prev = 0.0f;  smo->i_beta_prev = 0.0f;
    smo->theta_prev = 0.0f;
    
    // PLL-Startwerte (z.B. aus deinem Open-Loop)
    smo->theta_hat = 0.0f;
    smo->omega_hat = 0.0f;
    smo->integ_e   = 0.0f;

    // Faustregler (für Start): Kp ~ 2*w_pll, Ki ~ (w_pll)^2; w_pll << w0_emf
    const float w_pll = 2.0f * 3.14159265f * 15.0f;   // 15 Hz
    smo->Kp_pll = 1.4f * w_pll;
    smo->Ki_pll = (w_pll*w_pll);
    smo->theta_emf_raw = 0.0f;
    
    smo->w_e0_emf = w_pll * 6.0;
    smo->alpha_emf = 1.0f - expf(-smo->w_e0_emf * ts);
    if (smo->alpha_emf > 1.0f) smo->alpha_emf = 1.0f;
    if (smo->alpha_emf < 0.0f) smo->alpha_emf = 0.0f;
    
    smo->theta_rad = 0.0;
    smo->w_rad_s = 0.0f;
    
    // optionale ω-Glättung
    const float w_omega = 2.0f * 3.14159265f * 20.0f; // 20 Hz
    smo->alpha_omega = 1.0f - expf(-w_omega * ts);
    
    smo->omega_hat_f = 0.0f;
    
    smo->theta_shift = 0.36f;
}

void SMO_Update(smo_t *smo, float32_t iAlpha, float32_t iBeta, float32_t vAlpha, float32_t vBeta)
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
    const float OMEGA_MAX = 6720.0f;
    float32_t omega_u = smo->Kp_pll * e_theta + smo->integ_e;
    float32_t omega_limited = fminf(fmaxf(omega_u, -OMEGA_MAX), OMEGA_MAX);
    smo->omega_hat = omega_limited;
    
    // Back-calculation (k_aw ≈ ω_n)
    const float k_aw = 2.0f * 3.14159265f * 15.0f;
    smo->integ_e += (smo->Ki_pll * e_theta + (omega_limited - omega_u) * k_aw) * smo->Ts;
	
    smo->theta_hat = _wrap_pi(smo->theta_hat + smo->Ts * smo->omega_hat);
    smo->theta_rad = _wrap_pi(smo->theta_hat + smo->theta_shift);

    // optionale ω-Glättung (für Anzeige/Reglerübergabe)
    smo->omega_hat_f += smo->alpha_omega * (smo->omega_hat - smo->omega_hat_f);

    smo->i_alpha_prev = iAlpha;
    smo->i_beta_prev  = iBeta;
}


// ===== Hilfsfunktionen für Zugriff =====
static inline float SMO_GetTheta(const smo_t* smo){ return smo->theta_rad; }
static inline float SMO_GetOmega(const smo_t* smo){ return smo->omega_hat; }
static inline float SMO_GetSpeed(const smo_t* smo){ return smo->omega_hat/(TWO_M_PI*smo->p*smo->p)*60;}

//=========================
// FOC Functions
//=========================
static inline void _clarke_ab(float32_t ia, float32_t ib, float32_t ic, volatile float32_t *alpha, volatile float32_t *beta)
{
	*alpha = 2.0/3.0*(ia-0.5*ib-0.5*ic);
	*beta = 2.0/3.0*(ib*sqrt3o2 - ic*sqrt3o2);
}

static inline void _park_dq(float32_t alpha, float32_t  beta, float32_t sra, float32_t cra, float32_t *d, float32_t *q)
{
	*d = cra*alpha + sra*beta;
	*q = -sra*alpha + cra*beta;
}

static inline void _inv_park_ab(float32_t d, float32_t q, float32_t sra, float32_t cra, float32_t *alpha, float32_t *beta)
{
	*alpha = cra*d - sra*q;
	*beta = sra*d + cra*q;
}

//=========================
// SVPWM
//=========================

static inline void _svpwm(float32_t v_alpha, float32_t v_beta, float32_t overmod)
{
	float32_t Va = (2.0f/3.0f)*v_alpha;
	float32_t Vb = (-1.0f/3.0f)*v_alpha + (ONE_o_SQRT3*v_beta);
	float32_t Vc = (-1.0f/3.0f)*v_alpha - (ONE_o_SQRT3*v_beta);
	
	float32_t Vmax = fmaxf(fmaxf(Va, Vb), Vc);
	float32_t Vmin = fminf(fminf(Va, Vb), Vc);
	float32_t Voff = -0.5*(Vmax+Vmin);
	
	Va += Voff; Vb += Voff; Vc += Voff;
	
	if (overmod > 0.0f)
	{
		float32_t scale = 1.0f + 0.15f * _sat(overmod,0.0f,1.0f);
		Va *= scale; Vb *= scale; Vc *= scale;
	}
	
	float32_t da = _sat(0.5f + Va / Vdc, 0.0f, 1.0f);
	float32_t db = _sat(0.5f + Vb / Vdc, 0.0f, 1.0f);
	float32_t dc = _sat(0.5f + Vc / Vdc, 0.0f, 1.0f);
	
	uint32_t compareU = period_PWM * da;
	uint32_t compareV = period_PWM * db;
	uint32_t compareW = period_PWM * dc;
	
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_U_HW, PWM_Counter_U_NUM, compareU);
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_V_HW, PWM_Counter_V_NUM, compareV);
	Cy_TCPWM_PWM_SetCompare0BufVal(PWM_Counter_W_HW, PWM_Counter_W_NUM, compareW);
	
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_U_HW, PWM_Counter_U_NUM);
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_V_HW, PWM_Counter_V_NUM);
	Cy_TCPWM_TriggerCaptureOrSwap_Single(PWM_Counter_W_HW, PWM_Counter_W_NUM);	
}

//==========================
// Controller ISR
//==========================

void open_loop_init(ols_t *c, float32_t we_target, float32_t start_time)
{
	float32_t Ke = 0.011;
	float32_t alphaVF = 0.8;
	float32_t alphaIR = 0.4;
	
	c->Kvf = alphaVF * Ke;
	c->v_boost = Ir * Rs * alphaIR;
	c->w_e_target = we_target;
	c->t_start = start_time;
}

void foc_open_loop_step(ols_t *c, foc_curr_ctrl_t *s, foc_speed_ctrl_t *y)
{	
	if (t_ol < c->t_start) 
	{
		w_e = c->w_e_target * t_ol/c->t_start;
		vq = c->v_boost + c->Kvf * w_e;
		vd = 0.0f;
	}
	else if (t_ol > 2.0*c->t_start)
	{
		foc_state = FOC_RUNNING_OL_TO_CL;
		Ictrl.d.integrator = vd;
		Ictrl.q.integrator = vq;
		Speed_Control.integrator = I_q;
	}
}

void speed_controller_step(foc_speed_ctrl_t *s)
{
	float32_t e = w_e_ref - w_e;
	float32_t I_q_ref_unsat = e*s->kp + s->integrator;
	
	I_q_ref = _sat(I_q_ref_unsat, 0.0, Ir);
	I_d_ref = 0.0;
	s->integrator = _sat(s->integrator + e*s->ki*s->Ts, -Ir, Ir);
}

void current_controller_step(foc_curr_ctrl_t *c)
{
	I_q_ref = _sat(I_q_ref, 0.0, Ir);
	I_d_ref = _sat(I_d_ref, 0.0, Ir);	
    float32_t eq = I_q_ref - I_q;
    float32_t ed = I_d_ref - I_d;

    float32_t vq_unsat = eq*c->q.kp + c->q.integrator;
    float32_t vd_unsat = ed*c->d.kp + c->d.integrator;
	
    float32_t vmag = sqrtf(vq_unsat*vq_unsat + vd_unsat*vd_unsat);
    if (vmag > Vdc) {
        float32_t s = Vdc / vmag;
        vq = vq_unsat * s;   
        vd = vd_unsat * s; 
    } else {
        vq = vq_unsat;
        vd = vd_unsat;
    }
	
    c->q.integrator = _sat(c->q.integrator + eq*c->q.ki*c->Ts, -Vdc, Vdc);
    c->d.integrator = _sat(c->d.integrator + ed*c->d.ki*c->Ts, -Vdc, Vdc);
	
}

void controller_counter_intr_handler()
{
    uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(Controller_Counter_HW, Controller_Counter_NUM);
    
	clarke_vals_t m = measValues;  // kurzer Copy
	if(foc_state != FOC_READY_TO_START) SMO_Update(&SMO, m.ialpha, m.ibeta, m.valpha, m.vbeta);
	
	Poti_read();
	
	if (foc_state == FOC_RUNNING_OPEN_LOOP)
	{	
		theta_e_rad = _wrap_pi(theta_e_rad + w_e * Ictrl.Ts);
		sin_ra = sinf(theta_e_rad), cos_ra = cosf(theta_e_rad);
		_park_dq(measValues.ialpha, measValues.ibeta, sin_ra, cos_ra, &I_d, &I_q);
		foc_open_loop_step(&OLS, &Ictrl, &Speed_Control);
		t_ol += Ictrl.Ts;
	}
	
	if (foc_state == FOC_RUNNING_OL_TO_CL)
	{
		w_e = SMO_GetOmega(&SMO);
		theta_e_rad = SMO_GetTheta(&SMO);
		sin_ra = sinf(theta_e_rad), cos_ra = cosf(theta_e_rad);
		_park_dq(measValues.ialpha, measValues.ibeta, sin_ra, cos_ra, &I_d, &I_q);
		t_ol += Ictrl.Ts;
		if (t_ol > 3.0*OLS.t_start) foc_state = FOC_RUNNING_CLOSED_LOOP;
	}
	
	if (foc_state == FOC_RUNNING_CLOSED_LOOP)
	{
		theta_e_rad = SMO_GetTheta(&SMO);
		w_e = SMO_GetOmega(&SMO);
		sin_ra = sinf(theta_e_rad), cos_ra = cosf(theta_e_rad);
		_park_dq(measValues.ialpha, measValues.ibeta, sin_ra, cos_ra, &I_d, &I_q);
		speed_controller_step(&Speed_Control);
		current_controller_step(&Ictrl);
	}
	
	_inv_park_ab(vd, vq, sin_ra, cos_ra, &v_alpha, &v_beta);
	_svpwm(v_alpha, v_beta, 0.0);
	
    Cy_TCPWM_ClearInterrupt(Controller_Counter_HW, Controller_Counter_NUM, intrStatus);
}


/* --- ADC FIFO ISR --- */
void pass_0_sar_0_fifo_0_buffer_0_callback(void)
{
    uint32_t intrStatus = Cy_HPPASS_FIFO_GetInterruptStatusMasked();
    if (intrStatus & CY_HPPASS_INTR_FIFO_0_LEVEL) adc_group0_done_intr_handler();
    Cy_HPPASS_FIFO_ClearInterrupt(CY_HPPASS_INTR_FIFO_0_LEVEL);
}



/*******************************************************************************
 * PWM Reload IRQ: Puls-Zustandsmaschine (25 kHz Timing)
 *******************************************************************************/
static inline void apply_vector_deg(float32_t theta_deg, float32_t m)
{
    /* statorfester Vektor */
    float32_t theta = theta_deg * PI/180.0;
    float32_t Vmag  = m * (Vdc * ONE_o_SQRT3);
    v_alpha = Vmag * cosf(theta);
    v_beta  = Vmag * sinf(theta);
	_svpwm(v_alpha, v_beta, 0.0);
}

static inline int argmax(const float32_t* a, int n)
{
    int k=0; 
    for (int i=1;i<n;i++) 
   	{
		if (a[i]>a[k]) k=i;
	}
	return k;
}

static inline float wrap_deg(float32_t d)
{
    while (d>=360.0f) d-=360.0f;
    while (d<0.0f) d+=360.0f;
    return d;
}

static inline void prepare_series_coarse(void)
{
    memcpy(theta_list, theta_coarse, sizeof(theta_coarse));
    theta_count = 12;
    memset(peaks, 0, sizeof(peaks));
    inj_idx = 0;
    inj_m = M_COARSE;
    inj_state = INJ_NEXT;
}

static inline void prepare_series_15deg(float theta0)
{
    float list[6] = { theta0-37.5f, theta0-22.5f, theta0-7.5f, theta0+7.5f, theta0+22.5f, theta0+37.5f };
    for (int i=0;i<6;i++){ theta_list[i]=wrap_deg(list[i]); }
    theta_count = 6;
    memset(peaks, 0, sizeof(float)*6);
    inj_idx = 0;
    inj_m = M_FINE;
    inj_state = INJ_NEXT;
}

static inline void prepare_series_7p5deg(float theta0)
{
    float list[6] = { theta0-18.75f, theta0-11.25f, theta0-3.75f, theta0+3.75f, theta0+11.25f, theta0+18.75f };
    for (int i=0;i<6;i++){ theta_list[i]=wrap_deg(list[i]); }
    theta_count = 6;
    memset(peaks, 0, sizeof(float)*6);
    inj_idx = 0;
    inj_m = M_FINE;
    inj_state = INJ_NEXT;
}

void pwm_reload_intr_handler(void)
{
    uint32_t intrStatus = Cy_TCPWM_GetInterruptStatusMasked(PWM_Counter_U_HW, PWM_Counter_U_NUM);
    
    if (intrStatus & CY_TCPWM_INT_ON_TC)
    {
	    /* Für Off-Variante wählen: */
	    const bool useLowSideON = true;
	
	    switch (inj_state)
	    {
	    case INJ_IDLE:
	        break;
	
	    case INJ_NEXT:
	        if (inj_idx >= theta_count) 
	        {
	            inj_state = INJ_DONE;
	            break;
	        }
	        
	        inj_theta_deg = theta_list[inj_idx];
	        apply_vector_deg(inj_theta_deg, inj_m);
	        inj_on_cycles  = (inj_m==M_POL)? INJ_ON_CYCLES_POL : INJ_ON_CYCLES_COARSE;
	        
	        if (theta_count==6 && inj_m==M_FINE) inj_on_cycles = INJ_ON_CYCLES_FINE;
	
	        /* OFF-Zeit vorab setzen */
	        if (!useLowSideON) 
	        {
	            inj_off_cycles = (inj_m==M_POL)? INJ_OFF_CYCLES_POL_HIZ : (theta_count==6 ? INJ_OFF_CYCLES_FINE_HIZ : INJ_OFF_CYCLES_COARSE_HIZ);
	        } else 
	        {
	            inj_off_cycles = (inj_m==M_POL)? INJ_OFF_CYCLES_POL_LS : (theta_count==6 ? INJ_OFF_CYCLES_FINE_LS : INJ_OFF_CYCLES_COARSE_LS);
	        }
	
	        have_sample = false;
	        inj_state = INJ_ON;
	        break;
	
	    case INJ_ON:
	        /* In der letzten ON-Periode das ADC-Center-Sample freigeben */
	        if (inj_on_cycles == 2) 
	        {
	            inj_collect_now = true;   // ADC-ISR nimmt das Sample 
	        }
	
	        if (--inj_on_cycles <= 0) 
	        {
	            if (useLowSideON) low_sides_all_on(); 
	            else outputs_all_off();
	            inj_state = INJ_OFF;
	        }
	        break;
	
	    case INJ_OFF:
	        if (--inj_off_cycles <= 0) 
	        {
	            float32_t imag = last_imag;     // vom ADC-ISR
	            peaks[inj_idx] = imag;
	            inj_idx++;
	            inj_state = INJ_NEXT;
	        }
	        break;
	
	    case INJ_DONE: 
	    {
	        /* Serie auswerten und nächste vorbereiten */
	        int k = argmax(peaks, theta_count);
	        float32_t theta0 = theta_list[k];
	
	        static int stage = 0;  // 0:30°, 1:15°, 2:7.5°, 3:Pol
	        if (stage == 0) 
	        {
	            prepare_series_15deg(theta0); stage = 1;
	        } 
	        else if (stage == 1) 
	        {
	            prepare_series_7p5deg(theta0); stage = 2;
	        } 
	        else if (stage == 2) 
	        {
	            /* Polaritätstest: θ und θ+180° */
	            theta_est_deg = theta0;
	            inj_m = M_POL;
	            inj_idx = 0;
	            theta_list[0] = wrap_deg(theta_est_deg);
	            theta_list[1] = wrap_deg(theta_est_deg + 180.0f);
	            theta_count = 2;
	            memset(peaks,0,sizeof(float32_t)*2);
	            inj_state = INJ_NEXT;
	            stage = 3;
	        } 
	        else 
	        {
	            /* stage==3 → Polarität fertig */
	            int k2 = argmax(peaks, 2);
	            /* k2==0 → θ (d+) größer → θ zeigt auf N-Pol */
	            theta_polar_is_north = (k2 == 0);
	            /* Ergebnis steht: theta_est_deg, theta_polar_is_north */
	            foc_state = FOC_READY_TO_START;           // oder FOC_CLOSED_LOOP nach Bedarf
	            inj_state = INJ_IDLE;
	            
	            //open_loop_start(theta_est_deg, 50.0f);  // 300 rad/s elektrisch als Ziel
	            theta_e_rad = theta_est_deg * PI/180;
	            theta_e_rad = _wrap_pi(theta_e_rad);
	            NVIC_DisableIRQ(pwm_reload_intr_config.intrSrc);
	            SMO_Init(&SMO, Rs, Ls, Ictrl.Ts, Pp);
	            Cy_TCPWM_TriggerStart_Single(Controller_Counter_HW, Controller_Counter_NUM);
	        }
	        break;
	    }
	
	    case INJ_POL_A:
	    case INJ_POL_B:
	    case INJ_POL_DONE:
	        /* Nicht benötigt, Polarität wird mit normaler Statemachine erledigt */
	        break;
	    }
    }
    Cy_TCPWM_ClearInterrupt(PWM_Counter_U_HW, PWM_Counter_U_NUM, intrStatus);
}



static inline float32_t adc_to_amp(uint16_t adc, uint16_t adc_off, float32_t k)
{
    return k * ((float32_t)adc -(float32_t)adc_off);
}

static inline float32_t adc_to_volt(uint16_t adc)
{
	return 0.0228*(float32_t)adc-0.8265;
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
            foc_state = FOC_POSITION_ESTIMATION;
            low_sides_all_on();
			prepare_series_coarse();
			open_loop_init(&OLS, 400.0, 2.0);
            NVIC_EnableIRQ(pwm_reload_intr_config.intrSrc);
            samples_for_offset = 2048;
        }
    }
}

void adc_group0_done_intr_handler(void)
{
    uint8_t chanIdx;
    adc_sar_result[0] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[1] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[2] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[3] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[4] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[5] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[6] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);
    adc_sar_result[7] = Cy_HPPASS_FIFO_Read(0U, &chanIdx);

    const float32_t kiU = -0.0171f;
    const float32_t kiV = -0.0199f;
    const float32_t kiW = -0.0166f;
   	const float32_t kiBUS = -0.0157;
    float32_t currentU = adc_to_amp(adc_sar_result[0], adc_off_u, kiU);
    float32_t currentV = adc_to_amp(adc_sar_result[1], adc_off_v, kiV);
    float32_t currentW = adc_to_amp(adc_sar_result[2], adc_off_w, kiW);
    float32_t currentBUS = adc_to_amp(adc_sar_result[3], 2038, kiBUS);
    float32_t VoltageUtoGND = adc_to_volt(adc_sar_result[4]);
    float32_t VoltageVtoGND = adc_to_volt(adc_sar_result[5]);
    float32_t VoltageWtoGND = adc_to_volt(adc_sar_result[6]);
    measValues.vdc = adc_to_volt(adc_sar_result[7]);
   
    
   	float32_t Vcm = (VoltageUtoGND+VoltageVtoGND+VoltageWtoGND)/3.0;
   	VoltageUtoGND -= Vcm;
   	VoltageVtoGND -= Vcm;
   	VoltageWtoGND -= Vcm;
     
    _clarke_ab(currentU, currentV, currentW, &measValues.ialpha, &measValues.ibeta);
    _clarke_ab(VoltageUtoGND, VoltageVtoGND,VoltageWtoGND, &measValues.valpha, &measValues.vbeta);
    
    if (inj_collect_now && foc_state == FOC_POSITION_ESTIMATION) 
    {
        last_imag = sqrtf(measValues.ialpha*measValues.ialpha + measValues.ibeta*measValues.ibeta);
        have_sample = true;
        inj_collect_now = false;
    }
	
	if (foc_state == FOC_ADC_CALIBRATION)
	{	
	    if (samples_for_offset < 2048) 
	    {
	        current_offset_calibrate_step(adc_sar_result[0], adc_sar_result[1], adc_sar_result[2]);
	    }
	}
}


static cy_stc_scb_uart_context_t DEBUG_UART_context;
static mtb_hal_uart_t DEBUG_UART_hal_obj;

void Poti_read(void)
{
    int32_t raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(12U);
    w_e_ref = (float32_t)raw_value_idPot / 4095.0f * 6400;
    
    raw_value_idPot = Cy_HPPASS_SAR_Result_ChannelRead(10U);
    I_d_ref = (float32_t)raw_value_idPot / 4095.0f * 3.0;
}

void motor_stop(void)
{
    /* --- FOC-Zustände & Referenzen ------------------------------------ */
    foc_state = FOC_OFF;
    foc_init  = FIRST_STEP_ESTIMATION;

    w_e     = 0.0f;
    w_e_ref = 400.0f;      /* Startwert wie beim Global-Init */

    theta_e_rad = 0.0f;
    sin_ra = 0.0f;
    cos_ra = 1.0f;

    I_d     = 0.0f;
    I_q     = 0.0f;
    I_d_ref = 0.0f;
    I_q_ref = 0.0f;

    vq = 0.0f;
    vd = 0.0f;
    v_alpha = 0.0f;
    v_beta  = 0.0f;

    /* --- PI-Stromregler ----------------------------------------------- */
    Ictrl.d.integrator = 0.0f;
    Ictrl.d.prevI      = 0.0f;
    Ictrl.q.integrator = 0.0f;
    Ictrl.q.prevI      = 0.0f;

    /* --- Speed-Controller --------------------------------------------- */
    Speed_Control.integrator = 0.0f;
    Speed_Control.w_e_tgt    = 400.0f;   /* wie initial */

    /* --- Open-Loop-Start-Variablen ------------------------------------ */
    t_ol = 0.0f;
    a_th = 0.0f;

    OLS.v_boost     = 0.0f;
    OLS.w_e_target  = 0.0f;
    OLS.t_start     = 0.0f;
    OLS.Kvf         = 0.0f;
    OLS.d.integrator = 0.0f;
    OLS.q.integrator = 0.0f;

    /* --- Messwerte & ADC-Offsets -------------------------------------- */
    measValues.ialpha = 0.0f;
    measValues.ibeta  = 0.0f;
    measValues.valpha = 0.0f;
    measValues.vbeta  = 0.0f;
    measValues.vdc    = 0.0f;

    samples_for_offset = 0U;
    offset_acc_u = 0U;
    offset_acc_v = 0U;
    offset_acc_w = 0U;
    adc_off_u = 2048U;
    adc_off_v = 2048U;
    adc_off_w = 2048U;

    /* --- Injektions-/Positionsschätzungs-Statemachine ----------------- */
    inj_state = INJ_IDLE;
    inj_on_cycles  = 0;
    inj_off_cycles = 0;
    inj_idx        = 0;
    inj_m          = M_COARSE;
    inj_theta_deg  = 0.0f;
    theta_est_deg  = 0.0f;
    theta_polar_is_north = false;
    memset(peaks, 0, sizeof(peaks));

    inj_collect_now = false;
    have_sample     = false;
    last_imag       = 0.0f;

    /* --- Slide-Mode-Observer wieder auf Anfang ------------------------ */
    SMO_Init(&SMO, Rs, Ls, Ictrl.Ts, Pp);

    /* --- Button-State zurücksetzen (für sauberes Debouncing) ---------- */
    button_pressed_before = 0U;

    /* Optional: Duty auf neutralen Vektor erzwingen (kein Drehmoment) */
    v_alpha = 0.0f;
    v_beta  = 0.0f;
    
    low_sides_all_on();
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
    Cy_SysInt_Init(&pwm_reload_intr_config, pwm_reload_intr_handler);

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
    Cy_GPIO_Pin_Init(SW2_PORT, SW2_NUM, &SW2_config);

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

		if (Cy_GPIO_Read(SW2_PORT, SW2_NUM) == 1UL && button_pressed_before == false)
		{
			button_pressed_before = true;
	        switch (foc_state)
	        {
	            case FOC_OFF:
	                /* wie bisher: ADC-Offset-Kalibrierung starten */
	                Cy_TCPWM_TriggerStart_Single(Test_Counter_HW, Test_Counter_NUM);
	                foc_state = FOC_ADC_CALIBRATION;
	                break;
	
	            case FOC_READY_TO_START:
	                t_ol = 0.0f;              
	                foc_state = FOC_RUNNING_OPEN_LOOP;
	                break;
	
	            default:
	                motor_stop();
	                break;
	        }
		}
		else if (Cy_GPIO_Read(SW2_PORT, SW2_NUM) == 0UL)
		{
			button_pressed_before = false;
		}

		Poti_read();
		
        /* UART */
        char print_Array[25];
        float32_t data_Array[6] = {I_q, I_q_ref, Speed_Control.kp, w_e, w_e_ref,Speed_Control.ki};
        print_Array[0] = 0xAA;
        memcpy(&print_Array[1], data_Array, sizeof(data_Array));
        for (int i = 0; i < (int)sizeof(print_Array); i++) 
        { 
			printf("%c", print_Array[i]); 
		}

    }
}

/* [] END OF FILE */