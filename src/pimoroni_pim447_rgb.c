/* SPDX-License-Identifier: MIT */
/* PIM447 RGBW lighting engine, independent of mouse movement processing. */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "pimoroni_pim447.h"
#include "pimoroni_pim447_led.h"
#include "pimoroni_pim447_rgb.h"

LOG_MODULE_REGISTER(pim447_rgb, CONFIG_ZMK_PIMORONI_PIM447_LOG_LEVEL);

#define RGB_FRAME_MS 80
#define RGB_SAVE_DELAY_MS 5000
#define RGB_SETTINGS_KEY "pim447_rgb/state"
#define RGB_SETTINGS_VERSION 1
#define RGB_CHANNEL_STEP 16
#define RGB_BRIGHT_STEP 16
#define RGB_HUE_STEP_DEG 10.0f

enum rgb_mode {
    RGB_MODE_STATIC = 0,
    RGB_MODE_RAINBOW,
    RGB_MODE_REACTIVE,
    RGB_MODE_OFF,
};

/* The one PIM447 instance owns one lighting state. */
static struct {
    enum rgb_mode mode;
    enum rgb_mode last_enabled_mode;
    uint8_t r, g, b, w;
    uint8_t brightness;     /* 0..255, applies to all channels */
    uint8_t speed;          /* degrees advanced per frame */
    float hue;             /* 0..360, used for rainbow/reactive/HSV hue adjustments */
    bool awake;
} rgb_state = {
    .mode = RGB_MODE_REACTIVE, /* Preserves original movement-triggered rainbow on first boot. */
    .last_enabled_mode = RGB_MODE_REACTIVE,
    .r = 255, .g = 32, .b = 0, .w = 0,
    .brightness = 80,
    .speed = 3,
    .hue = 0.0f,
    .awake = false,
};

K_MUTEX_DEFINE(rgb_lock);

struct rgb_saved_settings {
    uint8_t version;
    uint8_t mode;
    uint8_t last_enabled_mode;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
    uint8_t brightness;
    uint8_t speed;
    uint16_t hue_tenths;
} __packed;

static void rgb_save_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rgb_save_work, rgb_save_work_handler);

static int rgb_settings_set(const char *key, size_t len, settings_read_cb read_cb, void *cb_arg)
{
    if (strcmp(key, "state") != 0) {
        return 0;
    }

    if (len != sizeof(struct rgb_saved_settings)) {
        LOG_WRN("Ignoring RGBW settings with unexpected size: %u", (unsigned int)len);
        return 0;
    }

    struct rgb_saved_settings saved;
    ssize_t read_len = read_cb(cb_arg, &saved, sizeof(saved));
    if (read_len < 0) {
        return (int)read_len;
    }
    if (read_len != sizeof(saved)) {
        return -EINVAL;
    }

    if (saved.version != RGB_SETTINGS_VERSION ||
        saved.mode > RGB_MODE_OFF ||
        saved.last_enabled_mode >= RGB_MODE_OFF ||
        saved.speed < 1 || saved.speed > 16 ||
        saved.hue_tenths >= 3600) {
        LOG_WRN("Ignoring invalid or incompatible RGBW settings");
        return 0;
    }

    k_mutex_lock(&rgb_lock, K_FOREVER);
    rgb_state.mode = (enum rgb_mode)saved.mode;
    rgb_state.last_enabled_mode = (enum rgb_mode)saved.last_enabled_mode;
    rgb_state.r = saved.r;
    rgb_state.g = saved.g;
    rgb_state.b = saved.b;
    rgb_state.w = saved.w;
    rgb_state.brightness = saved.brightness;
    rgb_state.speed = saved.speed;
    rgb_state.hue = (float)saved.hue_tenths / 10.0f;
    k_mutex_unlock(&rgb_lock);

    LOG_INF("Restored PIM447 RGBW settings");
    return 0;
}

static int rgb_settings_commit(void);

SETTINGS_STATIC_HANDLER_DEFINE(pim447_rgb, "pim447_rgb", NULL,
                               rgb_settings_set, rgb_settings_commit, NULL);

static void rgb_schedule_save(void)
{
    (void)k_work_reschedule(&rgb_save_work, K_MSEC(RGB_SAVE_DELAY_MS));
}

static void rgb_save_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    struct rgb_saved_settings saved;
    k_mutex_lock(&rgb_lock, K_FOREVER);
    saved.version = RGB_SETTINGS_VERSION;
    saved.mode = (uint8_t)rgb_state.mode;
    saved.last_enabled_mode = (uint8_t)rgb_state.last_enabled_mode;
    saved.r = rgb_state.r;
    saved.g = rgb_state.g;
    saved.b = rgb_state.b;
    saved.w = rgb_state.w;
    saved.brightness = rgb_state.brightness;
    saved.speed = rgb_state.speed;
    saved.hue_tenths = (uint16_t)(rgb_state.hue * 10.0f + 0.5f);
    if (saved.hue_tenths >= 3600) {
        saved.hue_tenths = 0;
    }
    k_mutex_unlock(&rgb_lock);

    int ret = settings_save_one(RGB_SETTINGS_KEY, &saved, sizeof(saved));
    if (ret != 0) {
        LOG_ERR("Failed to save PIM447 RGBW settings: %d", ret);
    } else {
        LOG_DBG("Saved PIM447 RGBW settings");
    }
}

static void rgb_animation_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(rgb_animation_work, rgb_animation_work_handler);

static uint8_t saturate_add(uint8_t value, int step)
{
    int result = (int)value + step;
    if (result < 0) {
        result = 0;
    } else if (result > 255) {
        result = 255;
    }
    return (uint8_t)result;
}

static float wrap_hue(float hue)
{
    while (hue >= 360.0f) {
        hue -= 360.0f;
    }
    while (hue < 0.0f) {
        hue += 360.0f;
    }
    return hue;
}

/* rgb_lock must be held on entry. */
static void render_frame_locked(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t *w)
{
    uint8_t rr = 0, gg = 0, bb = 0, ww = 0;

    if (rgb_state.mode == RGB_MODE_STATIC) {
        rr = rgb_state.r;
        gg = rgb_state.g;
        bb = rgb_state.b;
        ww = rgb_state.w;
    } else if (rgb_state.mode != RGB_MODE_OFF) {
        hsv_to_rgbw(rgb_state.hue, 1.0f, 1.0f, &rr, &gg, &bb, &ww);
    }

    *r = (uint8_t)(((unsigned int)rr * rgb_state.brightness + 127U) / 255U);
    *g = (uint8_t)(((unsigned int)gg * rgb_state.brightness + 127U) / 255U);
    *b = (uint8_t)(((unsigned int)bb * rgb_state.brightness + 127U) / 255U);
    *w = (uint8_t)(((unsigned int)ww * rgb_state.brightness + 127U) / 255U);
}

/* Render without holding rgb_lock during I2C access. */
static void rgb_render(const struct device *dev)
{
    uint8_t r, g, b, w;
    bool awake;
    k_mutex_lock(&rgb_lock, K_FOREVER);
    awake = rgb_state.awake;
    render_frame_locked(&r, &g, &b, &w);
    k_mutex_unlock(&rgb_lock);

    if (!awake) {
        return;
    }

    int ret = pimoroni_pim447_set_leds(dev, r, g, b, w);
    if (ret) {
        LOG_WRN("RGBW LED write failed: %d", ret);
    }
}

static void rgb_animation_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    k_mutex_lock(&rgb_lock, K_FOREVER);
    bool animate = rgb_state.awake && rgb_state.mode == RGB_MODE_RAINBOW;
    if (animate) {
        rgb_state.hue = wrap_hue(rgb_state.hue + (float)rgb_state.speed);
    }
    k_mutex_unlock(&rgb_lock);

    if (!animate) {
        return;
    }

    const struct device *dev = pim447_get_device();
    if (dev != NULL) {
        rgb_render(dev);
    }

    k_mutex_lock(&rgb_lock, K_FOREVER);
    animate = rgb_state.awake && rgb_state.mode == RGB_MODE_RAINBOW;
    k_mutex_unlock(&rgb_lock);
    if (animate) {
        (void)k_work_reschedule(&rgb_animation_work, K_MSEC(RGB_FRAME_MS));
    }
}

static void rgb_reschedule_if_needed(void);

static int rgb_settings_commit(void)
{
    const struct device *dev = pim447_get_device();
    if (dev != NULL) {
        rgb_render(dev);
    }
    rgb_reschedule_if_needed();
    return 0;
}

static void rgb_reschedule_if_needed(void)
{
    bool animate;
    k_mutex_lock(&rgb_lock, K_FOREVER);
    animate = rgb_state.awake && rgb_state.mode == RGB_MODE_RAINBOW;
    k_mutex_unlock(&rgb_lock);

    if (animate) {
        (void)k_work_reschedule(&rgb_animation_work, K_MSEC(RGB_FRAME_MS));
    } else {
        (void)k_work_cancel_delayable(&rgb_animation_work);
    }
}

void pim447_rgb_start(const struct device *dev)
{
    k_mutex_lock(&rgb_lock, K_FOREVER);
    rgb_state.awake = true;
    k_mutex_unlock(&rgb_lock);
    rgb_render(dev);
    rgb_reschedule_if_needed();
}

void pim447_rgb_stop(void)
{
    k_mutex_lock(&rgb_lock, K_FOREVER);
    rgb_state.awake = false;
    k_mutex_unlock(&rgb_lock);
    (void)k_work_cancel_delayable(&rgb_animation_work);
    /* The hardware driver's sleep routine already blanks the LEDs. */
}

void pim447_rgb_on_motion(const struct device *dev, float speed, float hue_increment)
{
    bool reactive;
    k_mutex_lock(&rgb_lock, K_FOREVER);
    reactive = rgb_state.awake && rgb_state.mode == RGB_MODE_REACTIVE;
    if (reactive) {
        rgb_state.hue = wrap_hue(rgb_state.hue + speed * hue_increment);
    }
    k_mutex_unlock(&rgb_lock);

    if (reactive) {
        rgb_render(dev);
    }
}

int pim447_rgb_apply_action(const struct device *dev, uint32_t action)
{
    k_mutex_lock(&rgb_lock, K_FOREVER);

    switch (action) {
    case PIM447_RGB_STATIC:
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_RAINBOW:
        rgb_state.mode = RGB_MODE_RAINBOW;
        break;
    case PIM447_RGB_REACTIVE:
        rgb_state.mode = RGB_MODE_REACTIVE;
        break;
    case PIM447_RGB_OFF:
        rgb_state.mode = RGB_MODE_OFF;
        break;
    case PIM447_RGB_NEXT:
        rgb_state.mode = (enum rgb_mode)(((int)rgb_state.mode + 1) % 4);
        break;
    case PIM447_RGB_HUE_INC:
    case PIM447_RGB_HUE_DEC:
        rgb_state.hue = wrap_hue(rgb_state.hue +
                                 (action == PIM447_RGB_HUE_INC ? RGB_HUE_STEP_DEG : -RGB_HUE_STEP_DEG));
        hsv_to_rgbw(rgb_state.hue, 1.0f, 1.0f,
                    &rgb_state.r, &rgb_state.g, &rgb_state.b, &rgb_state.w);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_RED_INC:
        rgb_state.r = saturate_add(rgb_state.r, RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_RED_DEC:
        rgb_state.r = saturate_add(rgb_state.r, -RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_GREEN_INC:
        rgb_state.g = saturate_add(rgb_state.g, RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_GREEN_DEC:
        rgb_state.g = saturate_add(rgb_state.g, -RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_BLUE_INC:
        rgb_state.b = saturate_add(rgb_state.b, RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_BLUE_DEC:
        rgb_state.b = saturate_add(rgb_state.b, -RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_WHITE_INC:
        rgb_state.w = saturate_add(rgb_state.w, RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_WHITE_DEC:
        rgb_state.w = saturate_add(rgb_state.w, -RGB_CHANNEL_STEP);
        rgb_state.mode = RGB_MODE_STATIC;
        break;
    case PIM447_RGB_BRIGHT_INC:
        rgb_state.brightness = saturate_add(rgb_state.brightness, RGB_BRIGHT_STEP);
        break;
    case PIM447_RGB_BRIGHT_DEC:
        rgb_state.brightness = saturate_add(rgb_state.brightness, -RGB_BRIGHT_STEP);
        break;
    case PIM447_RGB_SPEED_INC:
        if (rgb_state.speed < 16) {
            rgb_state.speed++;
        }
        break;
    case PIM447_RGB_SPEED_DEC:
        if (rgb_state.speed > 1) {
            rgb_state.speed--;
        }
        break;
    case PIM447_RGB_TOGGLE:
        if (rgb_state.mode == RGB_MODE_OFF) {
            rgb_state.mode = rgb_state.last_enabled_mode;
        } else {
            rgb_state.last_enabled_mode = rgb_state.mode;
            rgb_state.mode = RGB_MODE_OFF;
        }
        break;
    default:
        k_mutex_unlock(&rgb_lock);
        return -EINVAL;
    }

    if (rgb_state.mode != RGB_MODE_OFF) {
        rgb_state.last_enabled_mode = rgb_state.mode;
    }
    k_mutex_unlock(&rgb_lock);

    rgb_render(dev);
    rgb_reschedule_if_needed();
    rgb_schedule_save();
    return 0;
}
