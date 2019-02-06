#include <linux/gpio.h>
#include <linux/version.h>


#define USB_DEVICE_VID          0x10C4
#define USB_VENDOR_PID          0xEA80
#define NUM_GPIOS               10
#define BREQ_SET_GPIO_MODE      0x66
#define GPIO_IN                 0
#define GPIO_OUT                2 // Push-Pull Output


struct cp2110_gpio_irq {
    struct  irq_domain  *irq_domain;
    struct  mutex       irq_lock;
            int         virq[NUM_GPIOS];
            u16         irq_mask;
};


struct cp2110_device {
    struct  usb_device          *usb_dev;
    struct  usb_interface       *interface;
    struct  mutex               usb_bus_lock;

    struct  cp2110_gpio_irq     irq_chip;

    struct  cp2110_gpio_chip    gpio_chip;
            u8                  gpio_states[2];
            u8                  *usb_xfer;
};


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
static inline int gpiochip_add_data(struct gpio_chip *chip, void *data) {
    return gpiochip_add(chip);
}
#endif


static int gpio_direction_input(struct gpio_chip *gpio, unsigned off) {
    return gpio_direction(*gpio, off, GPIO_IN);
}


static int gpio_direction_output(struct gpio_chip *gpio, unsigned off) {
    return gpio_direction(*gpio, off, GPIO_OUT);
}


static int gpio_direction(struct gpio_chip *gpio, unsigned off, unsigned direction) {
    struct cp2110_device *dev = gpiochip_get_data(gpio);
    int ret;
    unsigned int pipe;

    pipe = usb_sndctrlpipe(dev->usb_dev, 0);

    mutex_lock(&dev->usb_bus_lock);

    for (i = 0; i < NUM_GPIOS; i++) {
        if (i == off) {
            dev->usb_xfer[i] = direction;
        } else {
            dev->usb_xfer[i] = 4; // Value out of range
        }
    }

    ret = usb_control_msg(
        dev->usb_dev,
        pipe,
        BREQ_SET_GPIO_MODE,
        USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
        0,
        0,
        dev->usb_xfer,
        NUM_GPIOS,
        200
    );

    mutex_unlock(&dev->usb_bus_lock);

    return (ret == 3) ? 0 : -EIO;
}


static int cp2110_probe(struct usb_interface *interface, const struct usb_device_id *dev_id) {
    struct cp2110_device *dev;
    struct usb_device *usb_dev = usb_get_dev(interface_to_usbdev(interface));
    struct gpio_chip *gpio;
    int ret;

    ret = -ENOMEM;

    if (!udev) {
        return -ENODEV;
    }

    dev = kzalloc(sizeof(struct cp2110_device), GFP_KERNEL);
    if (!dev) {
        goto err_out;
    }

    dev->usb_dev = usb_dev;
    ev->interface = interface;

    gpio = &dev->gpio_chip;
    gpio->direction_input = gpio_direction_input;
    gpio->direction_output = gpio_direction_output;
    gpio->get = gpio_get_value;
    gpio->set = gpio_set_value;
    gpio->can_sleep = true;

    gpio->base = -1;
    gpio->ngpio = NUM_GPIOS;
    gpio->owner = THIS_MODULE;

    ret = gpiochip_add_data(&dev->gpio_chip, dev);
    if (ret) {
        dev_err(&usb_dev->dev, "failed to register gpio chip");
    }
}
