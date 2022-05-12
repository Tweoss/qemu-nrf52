
#ifndef QEMU_NRF52_Z_MODEL_H
#define QEMU_NRF52_Z_MODEL_H


#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/registerfields.h"


#define TYPE_ZMODEL "z-model"
OBJECT_DECLARE_SIMPLE_TYPE(z_model_state, ZMODEL)


struct z_model_state {
    SSIPeripheral parent_obj;

    bool running;
    int64_t timer_start_ns;
    int64_t update_counter_ns;
    uint32_t counter;

    uint8_t id;
};

typedef struct {
    int16_t acc_mg[3];
    int16_t gyr_mrad_s[3];
} z_model_acc;

typedef struct {
    uint32_t adc[3];
} z_model_adc;

void z_model__compute_acc(z_model_state *s, z_model_acc *p_acc);

void z_model__compute_adc(z_model_state *s, uint8_t id, z_model_adc *p_adc);

#endif //QEMU_NRF52_Z_MODEL_H
