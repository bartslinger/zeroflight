pub(crate) fn serial_usb_interrupt_handler<B: usb_device::bus::UsbBus>(
    usb_dev: &mut usb_device::device::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
) {
    crate::drivers::usb::usb_poll(usb_dev, serial);
}
