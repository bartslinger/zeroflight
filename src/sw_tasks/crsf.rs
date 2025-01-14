#![allow(dead_code)]
use crate::common::RcState;
use modular_bitfield::prelude::*;

#[bitfield]
pub(crate) struct Channels {
    pub(crate) channel_01: B11,
    pub(crate) channel_02: B11,
    pub(crate) channel_03: B11,
    pub(crate) channel_04: B11,
    #[allow(unused)]
    pub(crate) channel_05: B11,
    #[allow(unused)]
    pub(crate) channel_06: B11,
    #[allow(unused)]
    pub(crate) channel_07: B11,
    #[allow(unused)]
    pub(crate) channel_08: B11,
    #[allow(unused)]
    pub(crate) channel_09: B11,
    #[allow(unused)]
    pub(crate) channel_10: B11,
    #[allow(unused)]
    pub(crate) channel_11: B11,
    #[allow(unused)]
    pub(crate) channel_12: B11,
    #[allow(unused)]
    pub(crate) channel_13: B11,
    #[allow(unused)]
    pub(crate) channel_14: B11,
    #[allow(unused)]
    pub(crate) channel_15: B11,
    #[allow(unused)]
    pub(crate) channel_16: B11,
}

pub(crate) fn ticks_to_us(ticks: u16) -> u16 {
    ((ticks as i16 - 992) * 5 / 8 + 1500) as u16
}

pub(crate) async fn crsf_parser_task(
    _cx: crate::app::crsf_parser_task::Context<'_>,
    mut rx: rtic_sync::channel::Receiver<'static, u8, 64>,
    mut tx: rtic_sync::channel::Sender<'static, RcState, 1>,
) {
    defmt::info!("starting crsf parser");

    let crc8_dvb_s2 = crc::Crc::<u8>::new(&crc::CRC_8_DVB_S2);
    loop {
        // Check sync byte
        if rx.recv().await.unwrap() != 0xC8 {
            continue;
        }
        let length = rx.recv().await.unwrap();
        if length != 24 {
            continue;
        }
        // Check message type RC Channels Packed Payload
        if rx.recv().await.unwrap() != 0x16 {
            continue;
        }
        let mut channel_bytes = [0_u8; 22];
        for i in 0..22 {
            channel_bytes[i] = rx.recv().await.unwrap();
        }
        let crc_byte = rx.recv().await.unwrap();

        let mut digest = crc8_dvb_s2.digest();
        digest.update(&[0x16]);
        digest.update(&channel_bytes);
        let calculated_crc = digest.finalize();
        if calculated_crc != crc_byte {
            defmt::warn!("crc mismatch");
            continue;
        }

        let channels = Channels::from_bytes(channel_bytes);
        let rc_state = [
            ticks_to_us(channels.channel_01()),
            ticks_to_us(channels.channel_02()),
            ticks_to_us(channels.channel_03()),
            ticks_to_us(channels.channel_04()),
            ticks_to_us(channels.channel_05()),
            ticks_to_us(channels.channel_06()),
            ticks_to_us(channels.channel_07()),
            ticks_to_us(channels.channel_08()),
            ticks_to_us(channels.channel_09()),
            ticks_to_us(channels.channel_10()),
        ];
        // let roll_us = ticks_to_us(channels.channel_01());
        // let pitch_us = ticks_to_us(channels.channel_02());
        // let throttle_us = ticks_to_us(channels.channel_03());
        // let yaw_us = ticks_to_us(channels.channel_04());
        // let armed_channel_us = ticks_to_us(channels.channel_05());
        // let mode_us = ticks_to_us(channels.channel_06());
        // // ...
        // let reset_channel = ticks_to_us(channels.channel_09());
        // let pitch_offset = ticks_to_us(channels.channel_10());

        // if previous_armed_channel_state <= 1500 && armed_channel_us > 1500 && throttle_us <= 1000 {
        //     armed = true;
        // }
        // if armed_channel_us < 1500 {
        //     armed = false;
        // }
        // previous_armed_channel_state = armed_channel_us;
        //
        // cx.shared
        //     .flags
        //     .reset_ahrs
        //     .store(reset_channel > 1500, core::sync::atomic::Ordering::SeqCst);
        //
        // let roll = ((roll_us as i16 - 1500) as f32 / 500.0).max(-1.0).min(1.0);
        // let pitch = ((pitch_us as i16 - 1500) as f32 / 500.0).max(-1.0).min(1.0);
        // let throttle = ((throttle_us as i16 - 1000) as f32 / 1000.0)
        //     .max(0.0)
        //     .min(1.0);
        // let yaw = ((yaw_us as i16 - 1500) as f32 / 500.0).max(-1.0).min(1.0);
        // let mode = ((mode_us as i16 - 1000) as f32 / 1000.0).max(0.0).min(1.0);
        // let pitch_offset = ((pitch_offset as i16 - 1500) as f32 / 500.0)
        //     .max(-1.0)
        //     .min(1.0);
        //
        // let rc_in = RcState {
        //     armed,
        //     roll,
        //     pitch,
        //     throttle,
        //     yaw,
        //     mode,
        //     pitch_offset,
        // };
        if let Err(_) = tx.try_send(rc_state) {
            // defmt::error!("error publishing rc state");
        }
    }
}
