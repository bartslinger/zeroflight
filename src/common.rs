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

#[derive(Copy, Clone)]
pub struct AhrsState {
    pub angles: dcmimu::EulerAngles,
    pub rates: (f32, f32, f32),
    pub _acceleration: (f32, f32, f32),
}
