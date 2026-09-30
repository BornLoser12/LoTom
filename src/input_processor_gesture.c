/*
 * Cygnus trackball gesture input processor.
 *
 * Converts dominant X/Y relative movement into one of four behavior taps.
 * Binding order is right, left, down, up.
 */

#include <stdlib.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <drivers/behavior.h>
#include <drivers/input_processor.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/virtual_key_position.h>
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <zmk/events/position_state_changed.h>
#endif

#define DT_DRV_COMPAT cygnus_input_processor_gesture

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define CYGNUS_GESTURE_EVENT_QUEUE_SIZE 16

enum cygnus_gesture_direction {
    CYGNUS_GESTURE_RIGHT = 0,
    CYGNUS_GESTURE_LEFT = 1,
    CYGNUS_GESTURE_DOWN = 2,
    CYGNUS_GESTURE_UP = 3,
    CYGNUS_GESTURE_NONE = 255,
};

struct cygnus_gesture_config {
    uint8_t index;
    uint8_t layer;
    const struct zmk_behavior_binding *bindings;
    uint32_t tick;
    int32_t threshold;
    int32_t max_threshold;
    uint32_t tap_ms;
    uint32_t wait_ms;
    int32_t max_delta;
    bool track_remainders;
};

struct cygnus_gesture_input_event {
    int32_t value;
    atomic_val_t generation;
    uint16_t code;
    uint8_t input_device_index;
    bool sync;
};

struct cygnus_gesture_data {
    const struct device *dev;
    struct k_work process_work;
    struct k_msgq events;
    struct cygnus_gesture_input_event event_buffer[CYGNUS_GESTURE_EVENT_QUEUE_SIZE];
    // Only this generation and the message queue are accessed outside sysworkq.
    atomic_t generation;
    atomic_val_t processed_generation;
    struct k_work_delayable release_work;
    int32_t report_x;
    int32_t report_y;
    int32_t delta_x;
    int32_t delta_y;
    uint32_t position;
    int64_t last_triggered_at;
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    uint8_t source;
#endif
    uint8_t pressed_binding;
};

static uint32_t approx_hypot(uint32_t a, uint32_t b) {
    if (a < b) {
        uint32_t tmp = a;
        a = b;
        b = tmp;
    }

    return a + ((b * 3) >> 3);
}

static struct zmk_behavior_binding_event
gesture_binding_event(const struct cygnus_gesture_data *data) {
    return (struct zmk_behavior_binding_event) {
        .position = data->position,
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = data->source,
#endif
    };
}

static void release_pressed_binding(struct cygnus_gesture_data *data,
                                    const struct cygnus_gesture_config *cfg) {
    if (data->pressed_binding == CYGNUS_GESTURE_NONE) {
        return;
    }

    uint8_t binding = data->pressed_binding;
    data->pressed_binding = CYGNUS_GESTURE_NONE;
    struct zmk_behavior_binding_event event = gesture_binding_event(data);
    int ret = zmk_behavior_invoke_binding(&cfg->bindings[binding], event, false);
    if (ret < 0) {
        LOG_WRN("Gesture release failed: %d", ret);
    }
}

static void release_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct cygnus_gesture_data *data =
        CONTAINER_OF(dwork, struct cygnus_gesture_data, release_work);
    const struct cygnus_gesture_config *cfg = data->dev->config;

    release_pressed_binding(data, cfg);
}

// Called only on the system workqueue, like all press/release operations.
static void reset_gesture(struct cygnus_gesture_data *data,
                          const struct cygnus_gesture_config *cfg) {
    k_work_cancel_delayable(&data->release_work);
    release_pressed_binding(data, cfg);
    data->report_x = 0;
    data->report_y = 0;
    data->delta_x = 0;
    data->delta_y = 0;
    data->last_triggered_at = 0;
}

static void sync_generation(struct cygnus_gesture_data *data,
                            const struct cygnus_gesture_config *cfg) {
    atomic_val_t generation = atomic_get(&data->generation);
    if (generation != data->processed_generation) {
        reset_gesture(data, cfg);
        data->processed_generation = generation;
    }
}

static void add_dominant_movement(struct cygnus_gesture_data *data,
                                  const struct cygnus_gesture_config *cfg) {
    int32_t x = data->report_x;
    int32_t y = data->report_y;
    int32_t abs_x = abs(x);
    int32_t abs_y = abs(y);

    data->report_x = 0;
    data->report_y = 0;

    if (abs_x < cfg->threshold && abs_y < cfg->threshold) {
        return;
    }

    uint32_t movement = approx_hypot(abs_x, abs_y);

    if (abs_x >= abs_y) {
        data->delta_x =
            CLAMP(data->delta_x + (x < 0 ? -(int32_t)movement : (int32_t)movement),
                  -cfg->max_delta, cfg->max_delta);
    } else {
        data->delta_y =
            CLAMP(data->delta_y + (y < 0 ? -(int32_t)movement : (int32_t)movement),
                  -cfg->max_delta, cfg->max_delta);
    }
}

static enum cygnus_gesture_direction pick_direction(struct cygnus_gesture_data *data,
                                                    const struct cygnus_gesture_config *cfg) {
    int32_t abs_x = abs(data->delta_x);
    int32_t abs_y = abs(data->delta_y);

    if (abs_x < cfg->tick && abs_y < cfg->tick) {
        return CYGNUS_GESTURE_NONE;
    }

    if (abs_x >= abs_y) {
        if (data->delta_x > 0) {
            if (cfg->track_remainders) {
                data->delta_x -= cfg->tick;
            }
            data->delta_y = 0;
            return CYGNUS_GESTURE_RIGHT;
        }

        if (cfg->track_remainders) {
            data->delta_x += cfg->tick;
        }
        data->delta_y = 0;
        return CYGNUS_GESTURE_LEFT;
    }

    if (data->delta_y > 0) {
        if (cfg->track_remainders) {
            data->delta_y -= cfg->tick;
        }
        data->delta_x = 0;
        return CYGNUS_GESTURE_UP;
    }

    if (cfg->track_remainders) {
        data->delta_y += cfg->tick;
    }
    data->delta_x = 0;
    return CYGNUS_GESTURE_DOWN;
}

static int trigger_binding(struct cygnus_gesture_data *data,
                           const struct cygnus_gesture_config *cfg,
                           enum cygnus_gesture_direction direction) {
    int64_t now = k_uptime_get();

    if (cfg->wait_ms > 0 && data->last_triggered_at > 0 &&
        now - data->last_triggered_at < cfg->wait_ms) {
        return 0;
    }

    k_work_cancel_delayable(&data->release_work);
    release_pressed_binding(data, cfg);

    struct zmk_behavior_binding_event event = gesture_binding_event(data);
    data->pressed_binding = direction;
    int ret = zmk_behavior_invoke_binding(&cfg->bindings[direction], event, true);
    if (ret < 0) {
        // A behavior can have changed state before reporting an error.
        release_pressed_binding(data, cfg);
        return ret;
    }

    data->last_triggered_at = now;
    ret = k_work_reschedule(&data->release_work, K_MSEC(cfg->tap_ms));
    if (ret < 0) {
        release_pressed_binding(data, cfg);
        return ret;
    }

    if (!cfg->track_remainders) {
        data->delta_x = 0;
        data->delta_y = 0;
    }

    return 0;
}

static void process_movement(const struct device *dev,
                             const struct cygnus_gesture_input_event *event) {
    const struct cygnus_gesture_config *cfg = dev->config;
    struct cygnus_gesture_data *data = dev->data;
    int32_t value = event->value;

    data->position =
        ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(event->input_device_index, cfg->index);
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    data->source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    if (abs(value) > cfg->max_threshold) {
        data->report_x = 0;
        data->report_y = 0;
        return;
    }

    if (event->code == INPUT_REL_X) {
        data->report_x = value;
    } else {
        data->report_y = value;
    }

    if (!event->sync) {
        return;
    }

    add_dominant_movement(data, cfg);
    enum cygnus_gesture_direction direction = pick_direction(data, cfg);

    if (direction != CYGNUS_GESTURE_NONE) {
        int ret = trigger_binding(data, cfg, direction);
        if (ret < 0) {
            LOG_WRN("Gesture press failed: %d", ret);
        }
    }
}

static void process_work_cb(struct k_work *work) {
    struct cygnus_gesture_data *data =
        CONTAINER_OF(work, struct cygnus_gesture_data, process_work);
    const struct cygnus_gesture_config *cfg = data->dev->config;

    sync_generation(data, cfg);
    // Bound each invocation so a continuous spin cannot starve key releases.
    for (size_t i = 0; i < CYGNUS_GESTURE_EVENT_QUEUE_SIZE; i++) {
        struct cygnus_gesture_input_event event;
        if (k_msgq_get(&data->events, &event, K_NO_WAIT) != 0) {
            break;
        }
        sync_generation(data, cfg);
        if (event.generation != data->processed_generation ||
            !zmk_keymap_layer_active(cfg->layer)) {
            continue;
        }
        process_movement(data->dev, &event);
    }
    sync_generation(data, cfg);
    if (k_msgq_num_used_get(&data->events) > 0) {
        k_work_submit(&data->process_work);
    }
}

static void gesture_layer_changed(const struct device *dev, uint8_t layer) {
    const struct cygnus_gesture_config *cfg = dev->config;
    struct cygnus_gesture_data *data = dev->data;
    if (layer != cfg->layer) {
        return;
    }
    // Invalidate immediately, including leave/re-enter before queued work runs.
    atomic_inc(&data->generation);
    k_work_submit(&data->process_work);
}

static int cygnus_gesture_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    struct cygnus_gesture_data *data = dev->data;
    ARG_UNUSED(param1);
    ARG_UNUSED(param2);

    if (event->type != INPUT_EV_REL ||
        (event->code != INPUT_REL_X && event->code != INPUT_REL_Y)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    struct cygnus_gesture_input_event queued = {
        .value = event->value,
        .generation = atomic_get(&data->generation),
        .code = event->code,
        .input_device_index = state->input_device_index,
        .sync = event->sync,
    };
    if (k_msgq_put(&data->events, &queued, K_NO_WAIT) < 0) {
        // A missing X or Y sample must not combine with a later frame.
        atomic_inc(&data->generation);
        LOG_WRN("Gesture input queue full; dropping pending movement");
    }
    int ret = k_work_submit(&data->process_work);
    if (ret < 0) {
        atomic_inc(&data->generation);
        LOG_WRN("Gesture worker submission failed: %d", ret);
    }

    return ZMK_INPUT_PROC_STOP;
}

static struct zmk_input_processor_driver_api cygnus_gesture_driver_api = {
    .handle_event = cygnus_gesture_handle_event,
};

static int cygnus_gesture_init(const struct device *dev) {
    struct cygnus_gesture_data *data = dev->data;

    data->dev = dev;
    data->pressed_binding = CYGNUS_GESTURE_NONE;
    k_msgq_init(&data->events, (char *)data->event_buffer,
                sizeof(data->event_buffer[0]), ARRAY_SIZE(data->event_buffer));
    k_work_init(&data->process_work, process_work_cb);
    k_work_init_delayable(&data->release_work, release_work_cb);

    return 0;
}

#define GESTURE_BINDINGS(n)                                                                        \
    {LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}

#define CYGNUS_GESTURE_INST(n)                                                                     \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == 4, "Cygnus gesture requires 4 bindings");       \
    BUILD_ASSERT(DT_INST_PROP(n, gesture_layer) < 32, "Gesture layer must be below 32");           \
    static struct cygnus_gesture_data cygnus_gesture_data_##n = {                                  \
        .pressed_binding = CYGNUS_GESTURE_NONE,                                                    \
    };                                                                                             \
    static struct zmk_behavior_binding cygnus_gesture_bindings_##n[] = GESTURE_BINDINGS(n);        \
    static const struct cygnus_gesture_config cygnus_gesture_config_##n = {                        \
        .index = n,                                                                                \
        .layer = DT_INST_PROP(n, gesture_layer),                                                    \
        .bindings = cygnus_gesture_bindings_##n,                                                   \
        .tick = DT_INST_PROP_OR(n, tick, 80),                                                      \
        .threshold = DT_INST_PROP_OR(n, threshold, 2),                                             \
        .max_threshold = DT_INST_PROP_OR(n, max_threshold, 200),                                   \
        .tap_ms = DT_INST_PROP_OR(n, tap_ms, 35),                                                  \
        .wait_ms = DT_INST_PROP_OR(n, wait_ms, 120),                                               \
        .max_delta = DT_INST_PROP_OR(n, max_pending_activations, 2) *                              \
                     DT_INST_PROP_OR(n, tick, 80),                                                 \
        .track_remainders = DT_INST_PROP_OR(n, track_remainders, false),                           \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, &cygnus_gesture_init, NULL, &cygnus_gesture_data_##n,                 \
                          &cygnus_gesture_config_##n, POST_KERNEL,                                 \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &cygnus_gesture_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CYGNUS_GESTURE_INST)

#define GESTURE_DEVICE(n) DEVICE_DT_INST_GET(n),
static const struct device *const gesture_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(GESTURE_DEVICE)
};

static int gesture_layer_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *event = as_zmk_layer_state_changed(eh);
    if (event) {
        for (size_t i = 0; i < ARRAY_SIZE(gesture_devices); i++) {
            gesture_layer_changed(gesture_devices[i], event->layer);
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(cygnus_gesture_layer, gesture_layer_listener);
ZMK_SUBSCRIPTION(cygnus_gesture_layer, zmk_layer_state_changed);
