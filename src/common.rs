pub const PI: f32 = 3.14159265358979323846264338327950288_f32;

// pub struct OutputCommand {
//     pub armed: bool,
//     pub roll: f32,
//     pub pitch: f32,
//     pub throttle: f32,
//     #[allow(unused)]
//     pub yaw: f32,
// }
//
// pub struct ActuatorPwmCommands {
//     pub s1: u16,
//     pub s2: u16,
//     pub s3: u16,
//     pub s4: u16,
//     pub s5: u16,
//     pub s6: u16,
// }

// pub type RcState = [u16; 10];

#[derive(Clone, Copy)]
pub struct ImuData {
    pub acceleration: (f32, f32, f32),
    pub rates: (f32, f32, f32),
}

// #[derive(Copy, Clone, Default)]
// pub struct AhrsState {
//     pub angles: dcmimu::EulerAngles,
// }

// pub struct TimestampedValue<T> {
//     pub timestamp: Instant<u32, 1, 1_000>,
//     value: T,
// }
//
// impl<T> TimestampedValue<T> {
//     pub fn new(inner: T) -> Self {
//         TimestampedValue {
//             timestamp: Mono::now(),
//             value: inner,
//         }
//     }
//
//     pub fn set(&mut self, value: T, timestamp: Instant<u32, 1, 1_000>) {
//         self.value = value;
//         self.timestamp = timestamp;
//     }
//
//     pub fn update(&mut self, value: T) {
//         self.value = value;
//         self.timestamp = Mono::now();
//     }
//
//     pub fn value(&self) -> &T {
//         &self.value
//     }
// }
//
// pub struct MaybeUpdated<T> {
//     inner: TimestampedValue<T>,
//     updated: bool,
// }
//
// impl<T> MaybeUpdated<T> {
//     pub fn new(initial_value: T) -> Self {
//         Self {
//             inner: TimestampedValue::new(initial_value),
//             updated: false,
//         }
//     }
//
//     pub fn update(&mut self, value: T) {
//         self.inner.update(value);
//         self.updated = true;
//     }
//
//     // pub fn peek(&self) -> &T {
//     //     &self.inner.value
//     // }
//
//     pub fn read(&mut self) -> &TimestampedValue<T> {
//         self.updated = false;
//         &self.inner
//     }
//
//     pub fn updated(&mut self) -> Option<&TimestampedValue<T>> {
//         if self.updated {
//             self.updated = false;
//             Some(&self.inner)
//         } else {
//             None
//         }
//     }
// }
