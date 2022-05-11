
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "ui/console.h"
#include "hw/registerfields.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/arm/z_model.h"

void z_model__compute_acc(z_model_state *s, z_model_acc *p_acc)
{
    // TODO int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);


}

static int max11254_post_load(void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_ssi_max11254 = {
        .name = "z_model",
        .version_id = 1,
        .minimum_version_id = 1,
        .post_load = max11254_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_SSI_PERIPHERAL(parent_obj, z_model_state),
                VMSTATE_END_OF_LIST()
        },
};

static void ssi_max11254_reset(DeviceState *dev)
{
    z_model_state *s = ZMODEL(dev);
    (void)s;
}

static void max11254_realize(DeviceState *dev, Error **errp)
{
    //z_model_state *s = ZMODEL(dev);

}

static void max11254_instance_init(Object *obj)
{
    z_model_state *s = ZMODEL(obj);

    vmstate_register(NULL, 0, &vmstate_ssi_max11254, s);
}

static Property _properties[] = {
        DEFINE_PROP_UINT8("ID", z_model_state, id, 0),
        DEFINE_PROP_END_OF_LIST(),
};

static void max11254_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ssi_max11254;
    dc->reset = ssi_max11254_reset;
    dc->realize = max11254_realize;

    device_class_set_props(dc, _properties);
}

static const TypeInfo max11254_info = {
        .name          = TYPE_ZMODEL,
        .parent        = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(z_model_state),
        .instance_init = max11254_instance_init,
        .class_init    = max11254_class_init,
};

static void max11254_register_types(void)
{
    type_register_static(&max11254_info);
}

type_init(max11254_register_types)
