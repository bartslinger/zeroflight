pub(crate) fn usart1_irq(cx: crate::app::usart1_irq::Context<'_>) {
    super::crsf::crsf_interrupt_handler(cx.local.crsf_serial, cx.local.crsf_data_sender);
}
