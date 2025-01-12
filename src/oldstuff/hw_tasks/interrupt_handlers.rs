use rtic::mutex_prelude::TupleExt02;

pub(crate) fn usart1_irq(cx: crate::app::usart1_irq::Context<'_>) {
    super::crsf::crsf_interrupt_handler(cx.local.crsf_serial, cx.local.crsf_data_sender);
}

pub(crate) fn dma2_stream0_irq(cx: crate::app::dma2_stream0_irq::Context<'_>) {
    let dma_context = cx.local.icm42688p_dma_context;
    super::icm42688::icm42688_interrupt_handler(
        dma_context,
        cx.local.prev_fifo_count,
        cx.local.imu_data_sender,
    );
}

pub(crate) fn otg_fs_irq(cx: crate::app::otg_fs_irq::Context<'_>) {
    let mut usb_dev = cx.shared.usb_dev;
    let mut serial = cx.shared.serial;
    (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
        super::usb::serial_usb_interrupt_handler(usb_dev, serial);
    });
}
