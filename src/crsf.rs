#![allow(dead_code)]
use modular_bitfield::prelude::*;

pub(crate) struct RcState {
    pub(crate) armed: bool,
    pub(crate) roll: u16,
    pub(crate) pitch: u16,
    pub(crate) throttle: u16,
    pub(crate) yaw: u16,
}

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
