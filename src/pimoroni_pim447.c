/* SPDX-License-Identifier: MIT */
/* pimoroni_pim447.c - Driver for Pimoroni PIM447 Trackball */

#define DT_DRV_COMPAT zmk_pimoroni_pim447

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/keymap.h>
#include <math.h>

#include "pimoroni_pim447.h"
#include "pimoroni_pim447_led.h"
#include "pimoroni_pim447_rgb.h"

LOG_MODULE_REGISTER(zmk_pimoroni_pim447, CONFIG_ZMK_PIMORONI_PIM447_LOG_LEVEL);

/* A missing INT GPIO selects I2C polling; no header struct changes are needed. */
#define PIM447_HAS_IRQ DT_INST_NODE_HAS_PROP(0, int_gpios)

#if !PIM447_HAS_IRQ
static void pimoroni_pim447_poll_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(pim447_poll_work, pimoroni_pim447_poll_handler);
static atomic_t pim447_polling_active = ATOMIC_INIT(0);
#endif

/* Statically initialize the shared settings mutex. */
K_MUTEX_DEFINE(pim447_settings_lock);

struct pim447_settings pim447_settings = {
    /* Moderate maximum gain, accelerated over a 20 ms reference window. */
    .mouse_max_speed = 8,
    .mouse_max_time = 20,
    .mouse_smoothing_factor = 1.3f,
    .scroll_max_speed = 1,
    .scroll_max_time = 1,
    .scroll_smoothing_factor = 0.5f,
    .hue_increment_factor = 0.3f,
};

/*
 * Motion acceleration is per-driver-instance state.  This driver declares
 * exactly one PIM447 device (DEVICE_DT_INST_DEFINE(0)), so a single state
 * object is sufficient.  Protect all accesses with data->data_lock.
 *
 * Keep fractional counts so small acceleration gains are not rounded away.
 */
#define PIM447_ACCEL_START_COUNTS 0.5f
#define PIM447_ACCEL_FULL_COUNTS  5.0f

static struct {
    float gain;
    float remainder_x;
    float remainder_y;
} pim447_motion = {
    .gain = 1.0f,
};

#define AUTOMOUSE_LAYER (DT_PROP(DT_DRV_INST(0), automouse_layer))

/* Forward declarations */
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                          gpio_port_pins_t pins);
static int pimoroni_pim447_enable_interrupt(const struct pimoroni_pim447_config *config,
                                            struct pimoroni_pim447_data *data, bool enable);
#if AUTOMOUSE_LAYER > 0
static void activate_automouse_layer(void);
static void automouse_deactivate_work_handler(struct k_work *work);
static struct k_work automouse_deactivate_work;
#else
static inline void activate_automouse_layer(void) {}
#endif

const struct device *pim447_get_device(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_INST(0, zmk_pimoroni_pim447));

    if (!device_is_ready(dev)) {
        return NULL;
    }

    return dev;
}

/*
 * Sleep enable/disable run from the ZMK activity listener thread and race
 * against the workqueue's I2C traffic. Take i2c_lock for the whole sequence
 * so the CTRL register update and the LED shutdown are atomic with respect
 * to movement reads. The LED write uses the _unlocked helper to avoid a
 * recursive lock.
 */
void pim447_enable_sleep(const struct device *dev)
{
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    uint8_t ctrl_reg_value;

    k_mutex_lock(&data->i2c_lock, K_FOREVER);

    if (i2c_reg_read_byte_dt(&config->i2c, REG_CTRL, &ctrl_reg_value) != 0) {
        LOG_ERR("Failed to read PIM447 control register");
        k_mutex_unlock(&data->i2c_lock);
        return;
    }

    ctrl_reg_value |= MSK_CTRL_SLEEP;

    if (i2c_reg_write_byte_dt(&config->i2c, REG_CTRL, ctrl_reg_value) != 0) {
        LOG_ERR("Failed to write PIM447 control register");
        k_mutex_unlock(&data->i2c_lock);
        return;
    }

    set_leds_unlocked(config, 0, 0, 0, 0);

#if !PIM447_HAS_IRQ
    /* Gate the already-submitted input work before releasing the I2C lock. */
    atomic_clear(&pim447_polling_active);
#endif
    k_mutex_unlock(&data->i2c_lock);
#if !PIM447_HAS_IRQ
    (void)k_work_cancel_delayable(&pim447_poll_work);
#endif

    pim447_rgb_stop();

    LOG_DBG("PIM447 sleep enabled");
}

void pim447_disable_sleep(const struct device *dev)
{
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    uint8_t ctrl_reg_value;

    k_mutex_lock(&data->i2c_lock, K_FOREVER);

    if (i2c_reg_read_byte_dt(&config->i2c, REG_CTRL, &ctrl_reg_value) != 0) {
        LOG_ERR("Failed to read PIM447 control register");
        k_mutex_unlock(&data->i2c_lock);
        return;
    }

    ctrl_reg_value &= ~MSK_CTRL_SLEEP;

    if (i2c_reg_write_byte_dt(&config->i2c, REG_CTRL, ctrl_reg_value) != 0) {
        LOG_ERR("Failed to write PIM447 control register");
        k_mutex_unlock(&data->i2c_lock);
        return;
    }

    /* Leave the LED off on wake; movement re-colours it from the hue cycle. */
    set_leds_unlocked(config, 0, 0, 0, 0);

#if !PIM447_HAS_IRQ
    atomic_set(&pim447_polling_active, 1);
#endif
    k_mutex_unlock(&data->i2c_lock);
#if !PIM447_HAS_IRQ
    (void)k_work_reschedule(&pim447_poll_work,
                            K_MSEC(CONFIG_ZMK_PIMORONI_PIM447_POLL_INTERVAL_MS));
#endif

    pim447_rgb_start(dev);

    LOG_DBG("PIM447 sleep disabled");
}

void pim447_toggle_mode(const struct device *dev)
{
    struct pimoroni_pim447_data *data = dev->data;

    k_mutex_lock(&data->data_lock, K_FOREVER);
    if (data->mode == PIM447_MODE_MOUSE) {
        data->mode = PIM447_MODE_SCROLL;
    } else {
        data->mode = PIM447_MODE_MOUSE;
    }
    enum pim447_mode new_mode = data->mode;

    /* Reset smoothing state on mode switch to avoid a stale first sample. */
    data->previous_x = 0;
    data->previous_y = 0;
    data->smoothed_x = 0;
    data->smoothed_y = 0;
    /* Do not carry mouse movement fractions into scroll mode or vice versa. */
    pim447_motion.gain = 1.0f;
    pim447_motion.remainder_x = 0.0f;
    pim447_motion.remainder_y = 0.0f;
    k_mutex_unlock(&data->data_lock);

    LOG_DBG("PIM447 mode switched to %s",
            (new_mode == PIM447_MODE_MOUSE) ? "MOUSE" : "SCROLL");
}

static int activity_state_changed_handler(const zmk_event_t *eh)
{
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    const struct device *dev = pim447_get_device();

    if (dev == NULL) {
        LOG_ERR("PIM447 device not ready");
        return -ENODEV;
    }

    /*
     * Only ZMK_ACTIVITY_ACTIVE should keep the sensor awake. Everything else
     * (IDLE and especially SLEEP/deep sleep) must put it to sleep - testing
     * for IDLE alone woke the sensor back up on deep sleep.
     */
    if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        pim447_disable_sleep(dev);
    } else {
        pim447_enable_sleep(dev);
    }

    return 0;
}

ZMK_LISTENER(idle_listener, activity_state_changed_handler);
ZMK_SUBSCRIPTION(idle_listener, zmk_activity_state_changed);

static void pim447_process_movement(struct pimoroni_pim447_data *data, int delta_x,
                                    int delta_y, uint32_t time_between_interrupts,
                                    int max_speed, int max_time, float smoothing_factor)
{
    /*
     * Convert movement to an approximate number of sensor counts in the
     * configurable reference window (mouse_max_time, initially 20 ms).
     * Unlike the old '< 5 ms' test, this works with 20 ms I2C polling.
     */
    float interval_ms = (time_between_interrupts == 0)
                            ? (float)CONFIG_ZMK_PIMORONI_PIM447_POLL_INTERVAL_MS
                            : (float)time_between_interrupts;

#if !PIM447_HAS_IRQ
    /* Never infer a huge velocity from a spuriously short poll interval. */
    if (interval_ms < (float)CONFIG_ZMK_PIMORONI_PIM447_POLL_INTERVAL_MS) {
        interval_ms = (float)CONFIG_ZMK_PIMORONI_PIM447_POLL_INTERVAL_MS;
    }
#else
    if (interval_ms < 1.0f) {
        interval_ms = 1.0f;
    }
#endif

    const float reference_ms = (max_time > 0) ? (float)max_time : 20.0f;
    const float dx = (float)delta_x;
    const float dy = (float)delta_y;
    const float counts_per_window = sqrtf(dx * dx + dy * dy) * reference_ms / interval_ms;

    /*
     * Smoothstep acceleration: precision at low movement rates and a
     * progressively higher gain for faster rolls of the ball.
     * mouse_max_speed is the maximum gain; mouse_max_time controls when
     * acceleration begins by changing the measurement reference window.
     */
    float t = (counts_per_window - PIM447_ACCEL_START_COUNTS) /
              (PIM447_ACCEL_FULL_COUNTS - PIM447_ACCEL_START_COUNTS);
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }
    const float curve = t * t * (3.0f - 2.0f * t);
    const float maximum_gain = (max_speed > 1) ? (float)max_speed : 1.0f;
    const float target_gain = 1.0f + (maximum_gain - 1.0f) * curve;

    /*
     * Smooth only the gain, not the raw sensor counts.  Values of
     * smoothing_factor > 1 are now safe and cannot cause the overshoot
     * produced by the previous (1 - smoothing_factor) formula.
     * A higher smoothing factor means a more gradual gain transition.
     */
    if (smoothing_factor < 0.1f) {
        smoothing_factor = 0.1f;
    }
    const float response = 1.0f / (1.0f + smoothing_factor);

    k_mutex_lock(&data->data_lock, K_FOREVER);

    pim447_motion.gain += response * (target_gain - pim447_motion.gain);
    float gain = pim447_motion.gain;
    if (data->mode == PIM447_MODE_SCROLL) {
        gain *= 2.5f; /* Preserve the original scroll-mode speed factor. */
    }

    /* Keep subpixel fractions instead of truncating them on each poll. */
    const float movement_x = dx * gain + pim447_motion.remainder_x;
    const float movement_y = dy * gain + pim447_motion.remainder_y;
    const int report_x = (int)movement_x;
    const int report_y = (int)movement_y;
    pim447_motion.remainder_x = movement_x - (float)report_x;
    pim447_motion.remainder_y = movement_y - (float)report_y;

    data->smoothed_x = report_x;
    data->smoothed_y = report_y;
    data->previous_x = report_x;
    data->previous_y = report_y;
    k_mutex_unlock(&data->data_lock);
}

/*
 * All I2C traffic in the work handler is serialized under i2c_lock so it
 * cannot collide with sleep/wake writes from the activity listener thread
 * or with LED writes from other contexts.
 */
static void pimoroni_pim447_work_handler(struct k_work *work)
{
    struct pimoroni_pim447_data *data = CONTAINER_OF(work, struct pimoroni_pim447_data, irq_work);
    const struct pimoroni_pim447_config *config = data->dev->config;
    const struct device *dev = data->dev;
    uint8_t buf[5];
    int ret;

    LOG_DBG("PIM447 work handler triggered");

    k_mutex_lock(&data->i2c_lock, K_FOREVER);
#if !PIM447_HAS_IRQ
    if (!atomic_get(&pim447_polling_active)) {
        k_mutex_unlock(&data->i2c_lock);
        return;
    }
#endif

    ret = i2c_burst_read_dt(&config->i2c, REG_LEFT, buf, sizeof(buf));
    if (ret) {
        LOG_ERR("Failed to read movement data from PIM447: %d", ret);
        k_mutex_unlock(&data->i2c_lock);
        return;
    }

    /* Clear movement registers and the interrupt flag while we still hold the lock. */
    i2c_reg_write_byte_dt(&config->i2c, REG_LEFT, 0);
    i2c_reg_write_byte_dt(&config->i2c, REG_RIGHT, 0);
    i2c_reg_write_byte_dt(&config->i2c, REG_UP, 0);
    i2c_reg_write_byte_dt(&config->i2c, REG_DOWN, 0);

    uint8_t int_status;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_status);
    if (ret == 0 && (int_status & MSK_INT_TRIGGERED)) {
        int_status &= ~MSK_INT_TRIGGERED;
        i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_status);
    }

    k_mutex_unlock(&data->i2c_lock);

    uint32_t time_between_interrupts;
    enum pim447_mode mode;

#if !PIM447_HAS_IRQ
    /* Update motion timing only for nonzero motion (not every empty poll). */
    if (buf[0] || buf[1] || buf[2] || buf[3]) {
        uint32_t now = k_uptime_get_32();
        k_mutex_lock(&data->data_lock, K_FOREVER);
        data->previous_interrupt_time = data->last_interrupt_time;
        data->last_interrupt_time = now;
        k_mutex_unlock(&data->data_lock);
    }
#endif

    k_mutex_lock(&data->data_lock, K_FOREVER);
    time_between_interrupts = data->last_interrupt_time - data->previous_interrupt_time;
    mode = data->mode;
    k_mutex_unlock(&data->data_lock);

    int16_t delta_x = (int16_t)buf[1] - (int16_t)buf[0]; /* RIGHT - LEFT */
    int16_t delta_y = (int16_t)buf[3] - (int16_t)buf[2]; /* DOWN - UP */

    if (delta_x != 0 || delta_y != 0) {
        int max_speed, max_time;
        float smoothing_factor;

        k_mutex_lock(&pim447_settings_lock, K_FOREVER);
        if (mode == PIM447_MODE_MOUSE) {
            max_speed = pim447_settings.mouse_max_speed;
            max_time = pim447_settings.mouse_max_time;
            smoothing_factor = pim447_settings.mouse_smoothing_factor;
        } else {
            max_speed = pim447_settings.scroll_max_speed;
            max_time = pim447_settings.scroll_max_time;
            smoothing_factor = pim447_settings.scroll_smoothing_factor;
        }
        k_mutex_unlock(&pim447_settings_lock);

        pim447_process_movement(data, delta_x, delta_y, time_between_interrupts, max_speed,
                                max_time, smoothing_factor);

        /* Snapshot the smoothed deltas under the lock, then report them unlocked. */
        int report_x, report_y;
        k_mutex_lock(&data->data_lock, K_FOREVER);
        report_x = data->smoothed_x;
        report_y = data->smoothed_y;
        k_mutex_unlock(&data->data_lock);

        if (mode == PIM447_MODE_MOUSE) {
            if (report_x != 0) {
                ret = input_report_rel(dev, INPUT_REL_X, report_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_x: %d", ret);
                } else {
                    LOG_DBG("Reported delta_x: %d", report_x);
                }
            }
            if (report_y != 0) {
                ret = input_report_rel(dev, INPUT_REL_Y, report_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report delta_y: %d", ret);
                } else {
                    LOG_DBG("Reported delta_y: %d", report_y);
                }
            }
        } else {
            /* Vertical ball movement (Y) drives the wheel, horizontal (X) the hwheel. */
            if (report_y != 0) {
                ret = input_report_rel(dev, INPUT_REL_WHEEL, report_y, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report wheel: %d", ret);
                } else {
                    LOG_DBG("Reported wheel: %d", report_y);
                }
            }
            if (report_x != 0) {
                ret = input_report_rel(dev, INPUT_REL_HWHEEL, report_x, true, K_NO_WAIT);
                if (ret) {
                    LOG_ERR("Failed to report hwheel: %d", ret);
                } else {
                    LOG_DBG("Reported hwheel: %d", report_x);
                }
            }
        }
    } else {
        /* Start a new motion burst from a precise 1x gain.  Retain fractional
         * counts so small previous movements are not discarded. */
        k_mutex_lock(&data->data_lock, K_FOREVER);
        pim447_motion.gain = 1.0f;
        data->previous_x = 0;
        data->previous_y = 0;
        data->smoothed_x = 0;
        data->smoothed_y = 0;
        k_mutex_unlock(&data->data_lock);
    }

    data->sw_pressed = (buf[4] & MSK_SWITCH_STATE) != 0;

    if (data->sw_pressed != data->sw_pressed_prev) {
        ret = input_report_key(dev, INPUT_BTN_0, data->sw_pressed ? 1 : 0, true, K_NO_WAIT);
        if (ret) {
            LOG_ERR("Failed to report key: %d", ret);
        } else {
            LOG_DBG("Reported switch state: %d", data->sw_pressed);
        }
        data->sw_pressed_prev = data->sw_pressed;
    }

    float speed = 0.0f;
    if (delta_x != 0 || delta_y != 0) {
        speed = sqrtf((float)(delta_x * delta_x + delta_y * delta_y));
    }

    if (speed > 0.0f) {
        activate_automouse_layer();

        k_mutex_lock(&pim447_settings_lock, K_FOREVER);
        float hue_increment = pim447_settings.hue_increment_factor;
        k_mutex_unlock(&pim447_settings_lock);

        /* Only REACTIVE lighting follows movement. STATIC and RAINBOW do not
         * get overwritten by trackball motion. */
        pim447_rgb_on_motion(dev, speed, hue_increment);
    }
}

#if !PIM447_HAS_IRQ
/* Schedule the already-existing read/report worker; the polling callback never
 * touches I2C itself and does not duplicate the RGBW/button/mouse logic. */
static void pimoroni_pim447_poll_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!atomic_get(&pim447_polling_active)) {
        return;
    }

    const struct device *dev = pim447_get_device();
    if (dev != NULL) {
        struct pimoroni_pim447_data *data = dev->data;
        (void)k_work_submit(&data->irq_work);
    }

    if (atomic_get(&pim447_polling_active)) {
        (void)k_work_reschedule(&pim447_poll_work,
                                K_MSEC(CONFIG_ZMK_PIMORONI_PIM447_POLL_INTERVAL_MS));
    }
}
#endif

/*
 * GPIO ISR: runs in interrupt context. k_mutex_* is not ISR-safe, so the
 * timestamp pair is updated under irq_lock instead. The work handler reads
 * them back under data_lock in thread context; irq_lock provides the
 * necessary memory barrier / atomicity against that read.
 */
static void pimoroni_pim447_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                          gpio_port_pins_t pins)
{
    struct pimoroni_pim447_data *data = CONTAINER_OF(cb, struct pimoroni_pim447_data, int_gpio_cb);
    uint32_t current_time = k_uptime_get_32();

    unsigned int key = irq_lock();
    data->previous_interrupt_time = data->last_interrupt_time;
    data->last_interrupt_time = current_time;
    irq_unlock(key);

    k_work_submit(&data->irq_work);
}

static int pimoroni_pim447_enable_interrupt(const struct pimoroni_pim447_config *config,
                                            struct pimoroni_pim447_data *data, bool enable)
{
    uint8_t int_reg;
    int ret;

    k_mutex_lock(&data->i2c_lock, K_FOREVER);

    ret = i2c_reg_read_byte_dt(&config->i2c, REG_INT, &int_reg);
    if (ret) {
        LOG_ERR("Failed to read INT register");
        k_mutex_unlock(&data->i2c_lock);
        return ret;
    }

    LOG_DBG("INT register before changing: 0x%02X", int_reg);

    if (enable) {
        int_reg |= MSK_INT_OUT_EN;
    } else {
        int_reg &= ~MSK_INT_OUT_EN;
    }

    ret = i2c_reg_write_byte_dt(&config->i2c, REG_INT, int_reg);
    if (ret) {
        LOG_ERR("Failed to write INT register");
        k_mutex_unlock(&data->i2c_lock);
        return ret;
    }

    k_mutex_unlock(&data->i2c_lock);

    LOG_DBG("INT register after changing: 0x%02X", int_reg);

    return 0;
}

static int pimoroni_pim447_enable(const struct device *dev)
{
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    int ret;

    LOG_DBG("pimoroni_pim447_enable called");

#if PIM447_HAS_IRQ
    if (!device_is_ready(config->int_gpio.port)) {
        LOG_ERR("Interrupt GPIO device is not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT | GPIO_PULL_UP);
    if (ret) {
        LOG_ERR("Failed to configure interrupt GPIO");
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_FALLING);
    if (ret) {
        LOG_ERR("Failed to configure GPIO interrupt");
        return ret;
    }

    gpio_init_callback(&data->int_gpio_cb, pimoroni_pim447_gpio_callback,
                       BIT(config->int_gpio.pin));

    ret = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
    if (ret) {
        LOG_ERR("Failed to add GPIO callback");
        return ret;
    }

    ret = pimoroni_pim447_enable_interrupt(config, data, true);
    if (ret) {
        LOG_ERR("Failed to enable interrupt output");
        return ret;
    }

#else
    /* No INT wire: disable the external interrupt output and poll over I2C. */
    ret = pimoroni_pim447_enable_interrupt(config, data, false);
    if (ret) {
        LOG_ERR("Failed to disable PIM447 INT output: %d", ret);
        return ret;
    }
    atomic_set(&pim447_polling_active, 1);
    (void)k_work_reschedule(&pim447_poll_work,
                            K_MSEC(CONFIG_ZMK_PIMORONI_PIM447_POLL_INTERVAL_MS));
#endif

    LOG_DBG("pimoroni_pim447 enabled");
    return 0;
}

static int pimoroni_pim447_init(const struct device *dev)
{
    const struct pimoroni_pim447_config *config = dev->config;
    struct pimoroni_pim447_data *data = dev->data;
    int ret;

    LOG_INF("PIM447 driver initializing");

    data->dev = dev;
    data->sw_pressed_prev = false;
    data->mode = PIM447_MODE_MOUSE;

    k_mutex_init(&data->data_lock);
    k_mutex_init(&data->i2c_lock);

    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus device is not ready");
        return -ENODEV;
    }

    k_mutex_lock(&data->i2c_lock, K_FOREVER);

    uint8_t chip_id_l, chip_id_h;
    ret = i2c_reg_read_byte_dt(&config->i2c, REG_CHIP_ID_L, &chip_id_l);
    if (ret) {
        LOG_ERR("Failed to read chip ID low byte: %d", ret);
        k_mutex_unlock(&data->i2c_lock);
        return ret;
    }

    ret = i2c_reg_read_byte_dt(&config->i2c, REG_CHIP_ID_H, &chip_id_h);
    if (ret) {
        LOG_ERR("Failed to read chip ID high byte: %d", ret);
        k_mutex_unlock(&data->i2c_lock);
        return ret;
    }

    k_mutex_unlock(&data->i2c_lock);

    uint16_t chip_id = ((uint16_t)chip_id_h << 8) | chip_id_l;
    LOG_INF("PIM447 chip ID: 0x%04X", chip_id);
    if (chip_id != PIM447_CHIP_ID_EXPECTED) {
        LOG_WRN("Unexpected chip ID 0x%04X (expected 0x%04X) - continuing anyway; "
                "check wiring/I2C address if the trackball is unresponsive.",
                chip_id, PIM447_CHIP_ID_EXPECTED);
    }

    /* Both work items must be ready before interrupts or polling begin. */
    k_work_init(&data->irq_work, pimoroni_pim447_work_handler);
#if AUTOMOUSE_LAYER > 0
    k_work_init(&automouse_deactivate_work, automouse_deactivate_work_handler);
#endif

    ret = pimoroni_pim447_enable(dev);
    if (ret) {
        LOG_ERR("Failed to enable PIM447");
        return ret;
    }

    pim447_rgb_start(dev);

    LOG_INF("PIM447 driver initialized");
    return 0;
}

#if AUTOMOUSE_LAYER > 0
static void automouse_deactivate_work_handler(struct k_work *work)
{
    zmk_keymap_layer_deactivate(AUTOMOUSE_LAYER);
}

static void automouse_layer_timer_expiry(struct k_timer *timer)
{
    /*
     * Timer callbacks run in the system timer ISR context, where the
     * event-raising layer deactivate is not safe to call. Defer it to the
     * system workqueue instead.
     */
    k_work_submit(&automouse_deactivate_work);
}

K_TIMER_DEFINE(automouse_layer_timer, automouse_layer_timer_expiry, NULL);

static void activate_automouse_layer(void)
{
    zmk_keymap_layer_activate(AUTOMOUSE_LAYER);
    k_timer_start(&automouse_layer_timer, K_MSEC(CONFIG_ZMK_PIMORONI_PIM447_AUTOMOUSE_TIMEOUT_MS),
                  K_NO_WAIT);
}
#endif

static const struct pimoroni_pim447_config pimoroni_pim447_config = {
    .i2c = I2C_DT_SPEC_INST_GET(0),
    .int_gpio = GPIO_DT_SPEC_INST_GET_OR(0, int_gpios, {0}),
};

static struct pimoroni_pim447_data pimoroni_pim447_data;

DEVICE_DT_INST_DEFINE(0, pimoroni_pim447_init, NULL, &pimoroni_pim447_data,
                      &pimoroni_pim447_config, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);
