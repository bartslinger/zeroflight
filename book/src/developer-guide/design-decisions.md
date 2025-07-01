# Design Decisions

## Task Priorities

### PWM Output

The PWM output task is the highest priority. This task has a timeout which can set failsafe values, to make sure the
motors are turned off. What could be done (but is not the case now) is to directly use radio commands in case the
vehicle task locks up and fails to publish PWM commands.

### IMU and Radio Receiver

Then there are high speed peripheral tasks, for the IMU and the radio receiver. These tasks share the same "fast_io"
executor, because there is not a huge benefit from giving one of them the possibility to interrupt the other one. These
tasks need to run fast because if the bytes are not taken quickly from the peripheral, it might loose data (I think).

I previously did the CRSF parsing (so not the UART receiving part) in a lower priority task. However, it should be fine
to do this here, because it is not very compute intensive. The next IMU sample might be delayed by fractions of
microseconds because of that.

### Vehicle logic

Next is fast vehicle logic. This is stuff like attitude/rate control, flight modes, etc. It runs at 1000 Hz and is
prioritized accordingly.

### AHRS

The attitude- and heading-reference system (AHRS) is compute intensive. A single calculation may take between 1.5 and 2
ms. That would exceed the period of the 1000Hz loop and therefore it runs at a lower priority.

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
