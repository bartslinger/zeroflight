use crate::app::Mono;
use rtic_monotonics::fugit::Instant;

pub const PI: f32 = 3.14159265358979323846264338327950288_f32;

pub struct OutputCommand {
    pub armed: bool,
    pub roll: f32,
    pub pitch: f32,
    pub throttle: f32,
    #[allow(unused)]
    pub yaw: f32,
}

pub struct ActuatorCommands {
    pub s1: u16,
    pub s2: u16,
    pub s3: u16,
    pub s4: u16,
    pub s5: u16,
    pub s6: u16,
}

#[derive(Clone, Copy)]
pub struct RcState {
    pub armed: bool,
    pub roll: f32,
    pub pitch: f32,
    pub throttle: f32,
    pub yaw: f32,
    pub mode: f32,
    pub pitch_offset: f32,
}

pub struct ImuData {
    pub acceleration: (f32, f32, f32),
    pub rates: (f32, f32, f32),
}

pub struct AhrsState {
    pub angles: dcmimu::EulerAngles,
    pub rates: (f32, f32, f32),
    pub _acceleration: (f32, f32, f32),
}

pub struct TimestampedValue<T> {
    pub timestamp: Instant<u32, 1, 1000>,
    pub value: T,
}

impl<T> TimestampedValue<T> {
    pub fn new(inner: T) -> Self {
        use rtic_monotonics::Monotonic;
        TimestampedValue {
            timestamp: Mono::now(),
            value: inner,
        }
    }
}

pub enum Update<T> {
    Unchanged(TimestampedValue<T>),
    Updated(TimestampedValue<T>),
}

impl<T> Update<T> {
    pub fn new(initial_value: T) -> Self {
        Update::Unchanged(TimestampedValue::new(initial_value))
    }

    pub fn update(&mut self, value: T) {
        use rtic_monotonics::Monotonic;
        *self = Update::Updated(TimestampedValue {
            value: value,
            timestamp: Mono::now(),
        });
    }

    pub fn timestamped_value(&self) -> &TimestampedValue<T> {
        match self {
            Update::Unchanged(data) => data,
            Update::Updated(data) => data,
        }
    }

    // pub fn value(&self) -> &T {
    //     match self {
    //         Update::Unchanged(data) => &data.value,
    //         Update::Updated(data) => &data.value,
    //     }
    // }

    pub fn updated(&self) -> Option<&T> {
        match self {
            Update::Unchanged(_) => None,
            Update::Updated(data) => Some(&data.value),
        }
    }
}
