/* SPDX-License-Identifier: MIT */
#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <dt-bindings/pim447_rgb.h>

/* The RGBW engine runs only on the controller with the physical trackball. */
int pim447_rgb_apply_action(const struct device *dev, uint32_t action);
void pim447_rgb_start(const struct device *dev);
void pim447_rgb_stop(void);
void pim447_rgb_on_motion(const struct device *dev, float speed, float hue_increment);
