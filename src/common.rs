pub const PI: f32 = 3.14159265358979323846264338327950288_f32;

pub struct OutputCommand {
    pub armed: bool,
    pub roll: u16,
    pub pitch: u16,
    pub throttle: u16,
    #[allow(unused)]
    pub yaw: u16,
}

#[derive(Clone, Copy)]
pub struct RcState {
    pub armed: bool,
    pub roll: u16,
    pub pitch: u16,
    pub throttle: u16,
    pub yaw: u16,
    pub mode: u16,
    pub pitch_offset: u16,
}

#[derive(Copy, Clone)]
pub struct AhrsState {
    pub angles: dcmimu::EulerAngles,
    pub rates: (f32, f32, f32),
    pub _acceleration: (f32, f32, f32),
}
