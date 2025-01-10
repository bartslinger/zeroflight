# Design Decisions

## Vehicle module

The idea behind the vehicle module is that it contains the "business logic" of the flight controller. The code in this
module is responsible for converting IMU and RC inputs to PWM outputs.

The `main_loop` function is called whenever a new IMU sample is available. This happens at 1000 Hz, because that is how
the IMU is configured now. Whenever a new RC update is available, it is not processed immediately. It will be processed
when the next IMU sample is ready. Therefore there might be up to 1 ms delay between an RC signal received and
processed. This is a trade-off to keep the `main_loop` simple.

The `ahrs_loop` that is responsible for calculation the attitude angles (roll and pitch). We don't need to calculate
these angles at 1000 Hz, because the angles don't change that fast. To ensure a fast control response, the angular rates
are controlled in the `main_loop`. Calculating the angles with the `dcmimu` crate takes more than 1 ms and thereby
exceeds the period of the 1000 Hz loop. Therefore, the `ahrs_loop` is only called with every 4th sample. The resulting
attitude angles are available as an input to the `main_loop`.
