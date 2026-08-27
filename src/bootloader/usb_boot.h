/*
 * Polled USB-MIDI device for the SysEx bootloader. See usb_boot.c.
 */
#ifndef USB_BOOT_H
#define USB_BOOT_H

void usb_boot_init(void);   /* start the USB controller and attach */
void usb_boot_task(void);   /* poll: enumeration + MIDI-OUT endpoint drain */
void usb_boot_detach(void); /* drop off the bus (no-op if init never ran) */

/* Bring-up telemetry, read by the SUPEROS_USB_DIAG LED display in bootload.c.
 * bit0: host drove a bus reset (proves the host SAW us electrically)
 * bit1: SET_CONFIGURATION received (enumeration completed) */
extern volatile uint8_t usb_boot_flags;
#define USB_BOOT_SAW_RESET  0x01
#define USB_BOOT_CONFIGURED 0x02

#endif
