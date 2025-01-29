#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/pwm.h"

static int brightness = 0;  // Current brightness level (duty cycle)
static int fade_direction = 1;  // 1 = increasing brightness, -1 = decreasing
/* !!! PART 2 & 3 !!! */

/* !!! MIGHT WANT TO CHANGE THIS !!! */
#define BUTTON_DEBOUNCE_DELAY   50
#define EVENT_QUEUE_LENGTH      10 


typedef enum _event_t 
{
    b1_evt = 0,
    b2_evt = 1,
    b3_evt = 2,
    no_evt = 3
} event_t;

const static uint GP0 = 0; 
const static uint GP1 = 1; 
const static uint GP2 = 2;
const static uint GP3 = 3;

const static uint B1 = 20;
const static uint B2 = 21; 
const static uint B3 = 22;

queue_t evt_queue;

/* Function pointer primitive */ 
typedef void (*state_func_t)( void );

typedef struct _state_t
{
    uint8_t id;
    state_func_t Enter;
    state_func_t Do;
    state_func_t Exit;
    uint32_t delay_ms;
} state_t;


/* !!! PART 2 & 3 !!! */
/* Define event queue */

unsigned long button_time = 0;

void button_isr(uint gpio, uint32_t events) 
{
    /* !!! PART 2 !!! */
    if ((to_ms_since_boot(get_absolute_time())-button_time) > BUTTON_DEBOUNCE_DELAY) 
    {
        button_time = to_ms_since_boot(get_absolute_time());
        
        event_t evt;
        switch(gpio)
        {
            case B1: 
                evt = b1_evt; 
                queue_try_add(&evt_queue, &evt); 
            break; 

            case B2: 
                evt = b2_evt; 
                queue_try_add(&evt_queue, &evt); 
            break;

            case B3: 
                evt = b3_evt; 
                queue_try_add(&evt_queue, &evt); 
            break;
        }
    }
    return; 
}

void private_init() 
{

    /* !!! PART 2 !!! */
    /* Button setup */
    gpio_init(B1); 
    gpio_init(B2);
    gpio_init(B2);
    gpio_set_dir(B1, GPIO_IN); 
    gpio_set_dir(B2, GPIO_IN); 
    gpio_set_dir(B2, GPIO_IN);

    /* !!! PART 1 !!! */
    /* LED setup */
    gpio_init(GP0); 
    gpio_init(GP1); 
    gpio_init(GP2); 
    gpio_init(GP3);  
    gpio_set_dir(GP0, GPIO_OUT);
    gpio_set_dir(GP1, GPIO_OUT);
    gpio_set_dir(GP2, GPIO_OUT);
    gpio_set_dir(GP3, GPIO_OUT);

    gpio_set_irq_enabled_with_callback(B1, GPIO_IRQ_EDGE_FALL, true, &button_isr); 
    gpio_set_irq_enabled_with_callback(B2, GPIO_IRQ_EDGE_FALL, true, &button_isr); 
    gpio_set_irq_enabled_with_callback(B3, GPIO_IRQ_EDGE_FALL, true, &button_isr); 

    
   

    /* Event queue setup */
    queue_init(&evt_queue, sizeof(event_t), EVENT_QUEUE_LENGTH); 
}

/* The next three methods are for convenience, you might want to use them. */
event_t get_event(void)
{
    /* !!!! PART 2 !!!! */
    event_t evt = no_evt; 
    if (queue_try_remove(&evt_queue, &evt))
    { 
        return evt; 
    }
    return no_evt; 
}

void on_pwm_wrap() {
    static int fade = 0;
    static bool going_up = true;
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(GP0));

    if (going_up) {
        ++fade;
        if (fade > 255) {
            fade = 255;
            going_up = false;
        }
    } else {
        --fade;
        if (fade < 0) {
            fade = 0;
            going_up = true;
        }
    }
    // Square the fade value to make the LED's brightness appear more linear
    // Note this range matches with the wrap value
    pwm_set_gpio_level(GP0, fade * fade);
}

void leds_off () 
{
    /* !!! PART 1 !!! */
    gpio_put(GP0, 0);
    gpio_put(GP1, 0);
    gpio_put(GP2, 0);
    gpio_put(GP3, 0);

}

void leds_on () 
{
    /* !!! PART 2 !!! */
    gpio_put(GP0, 1);
    gpio_put(GP1, 1);
    gpio_put(GP2, 1);
    gpio_put(GP3, 1);
}

void do_state_0(void)
{

    /* !!! PART 1 !!! */
    uint slice_num = pwm_gpio_to_slice_num(GP0);
    pwm_set_irq_enabled(slice_num, true);
    gpio_put(GP0, 1);
    

}


void do_state_1(void) {
    gpio_put(GP1, 1);

}


void do_state_2(void) {
    
    gpio_put(GP2, 1);
}


void do_state_3(void) {
    gpio_set_function(GP0, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(GP0);
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), on_pwm_wrap);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 4.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

}


/* !!! PART 1 !!! */
const state_t state0 = {
    0, 
    leds_off,
    do_state_0,
    leds_off, 
    500
};

const state_t state1 = {
    1, 
    leds_off,
    do_state_1,
    leds_off, 
    500
};

const state_t state2 = {
    2, 
    leds_off,
    do_state_2,
    leds_off, 
    500
};

const state_t state3 = {
    3, 
    leds_off,
    do_state_3,
    leds_off, 
    1000
};

/* !!! PART 2 !!! */
 state_t state_table[4][4] = {
    /*  STATE       B1              B2      B3      NO-EVT   */
    {/* S0 */      state2,    state1,  state3,   state0},
    {/* S1 */   state0,   state2,  state3,  state1},    
    {/* S2 */    state1,    state0,  state3,  state2},
    {/* S3 */    state0, state0, state0, state3 }
};

/* !!! ALL PARTS !!! */
int main() 
{
    private_init(); 

    state_t current_state = state0;
    event_t evt = no_evt;

    for(;;) 
    {
        current_state.Enter(); 

        while (1)  
        {
            current_state.Do();
            evt = get_event();

            if (evt != no_evt && current_state.id != state_table[current_state.id][evt].id) {
                break;  // Transition to next state
            }
        }

        current_state.Exit(); 
        current_state = state_table[current_state.id][evt];
    }
}

