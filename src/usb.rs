pub(crate) fn usb_tx(cx: crate::app::usb_tx::Context<'_>) {
    use rtic::mutex_prelude::*;
    let mut usb_dev = cx.shared.usb_dev;
    let mut serial = cx.shared.serial;

    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
        usb_poll(usb_dev, serial);
    });
}

fn usb_poll<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::device::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
) {
    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial.write(&buf[0..count]).ok();
            // cortex_m::asm::delay(20_000_000); // for testing priority
        }
        _ => {}
    }
}