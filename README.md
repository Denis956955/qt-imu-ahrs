# qt-imu-ahrs

Real-time IMU attitude estimation and heading visualization using C++17, Qt 6, ESP32, ICM-20948 and a quaternion-based Error-State Kalman Filter (ESKF).

The application receives live accelerometer, gyroscope and magnetometer measurements from an ESP32 over a serial connection, estimates sensor orientation in real time and visualizes it using an artificial horizon and heading indicator.

## Demo

The demonstration shows the physical IMU board and the Qt application simultaneously.

A physical rotation of approximately 90° around the vertical axis produces an approximately 90° change in the displayed heading.

The recording also demonstrates magnetometer calibration and real-time attitude estimation.

**Demo video:**  
_Add video link here_

---

## Features

- Real-time IMU data acquisition
- Serial communication between ESP32 and Qt
- ICM-20948 9-DoF IMU
- Quaternion-based attitude representation
- 9-state Error-State Kalman Filter
- Gyroscope bias estimation
- Accelerometer bias estimation
- Six-position accelerometer calibration
- Hard-iron magnetometer calibration
- 3×3 soft-iron magnetometer correction
- Tilt-compensated magnetic heading
- Accelerometer-based attitude correction
- Magnetometer-based heading correction
- Adaptive accelerometer measurement noise
- Joseph-form covariance update
- Error-state injection and covariance reset
- Real-time artificial horizon
- Real-time heading indicator

---

## Hardware

Current test setup:

- ESP32
- ICM-20948 9-DoF IMU
  - 3-axis accelerometer
  - 3-axis gyroscope
  - 3-axis magnetometer
- USB serial connection to the desktop application

The ESP32 reads the IMU measurements and transmits them to the desktop application.

---

## Software

Main technologies:

- C++17
- Qt 6
- Qt Serial Port
- Eigen
- CMake
- Arduino / ESP32

---

## Architecture

```text
ICM-20948
    |
    v
ESP32
    |
    | USB / Serial
    v
Serial parser
    |
    v
Sensor calibration
    |
    v
Quaternion-based ESKF
    |
    v
Estimated orientation
    |
    +-------------------+
    |                   |
    v                   v
Artificial          Heading
horizon             indicator
```

---

# State Estimation

## Error-State Kalman Filter

The application uses a quaternion-based 9-state Error-State Kalman Filter.

The nominal orientation is represented by a quaternion.

The error state is

```text
δx = [δθ, δbg, δba]ᵀ
```

where:

- `δθ` — small attitude error
- `δbg` — gyroscope bias error
- `δba` — accelerometer bias error

The implementation follows the conventional ESKF processing sequence:

1. quaternion propagation using gyroscope measurements
2. linearized error-state propagation
3. covariance prediction
4. accelerometer and magnetometer measurement correction
5. Kalman gain calculation
6. Joseph-form covariance update
7. error-state injection into the nominal state
8. covariance reset after error injection

The covariance prediction follows the standard form:

```text
P⁻ = F P Fᵀ + Q
```

The measurement correction uses:

```text
y = z - h(x)

S = H P Hᵀ + R

K = P Hᵀ S⁻¹
```

where:

- `F` — linearized state transition matrix
- `H` — measurement Jacobian
- `P` — error-state covariance
- `Q` — process-noise covariance
- `R` — measurement-noise covariance
- `K` — Kalman gain
- `y` — innovation

The implementation is an engineering and educational prototype.

Its structure follows the standard ESKF formulation, but it has not been formally validated against a certified navigation system or a high-grade external attitude reference.

It should therefore not be considered suitable for safety-critical navigation without additional testing and validation.

---

# Sensor Processing

## Gyroscope

The gyroscope is used for short-term attitude propagation.

During initialization, stationary samples are collected and used to estimate the gyroscope bias.

Per-axis gyroscope variance is also calculated from stationary measurements and is used as part of the filter noise model.

---

## Accelerometer

The accelerometer is used primarily as an observation of the gravity direction.

A six-position calibration procedure is used to estimate accelerometer bias.

Under approximately static conditions, the gravity vector provides roll and pitch correction.

Because an accelerometer measures specific force rather than gravity directly, translational acceleration can disturb this correction.

---

## Magnetometer

The magnetometer provides heading information.

The raw magnetic field is corrected using

```text
m_cal = M (m_raw - b)
```

where:

- `m_raw` — raw magnetometer measurement
- `b` — hard-iron bias vector
- `M` — 3×3 soft-iron correction matrix

The calibrated magnetic vector is then used for tilt-compensated heading estimation and ESKF heading correction.

---

# Magnetometer Calibration

Magnetometer calibration is currently performed externally using MotionCal.

The calibration procedure estimates:

- hard-iron offset
- 3×3 soft-iron correction matrix

The resulting parameters are currently stored directly in the application.

Example:

```cpp
static const Vec3 magBias(
    -87.60,
     47.81,
     13.87
);

static const Mat3 magCorrection = (
    Mat3() <<
        0.947,  0.023,  0.134,
        0.023,  1.047, -0.022,
        0.134, -0.022,  1.028
).finished();
```

These values are specific to the sensor and its environment.

The magnetometer should be recalibrated if:

- the sensor is replaced
- the mounting configuration changes
- nearby ferromagnetic components are added or removed
- the electronics layout changes
- the magnetic environment changes significantly

Automatic magnetometer calibration is currently not implemented in the application.

---

# Adaptive Accelerometer Noise

The filter contains a basic mechanism for reducing accelerometer influence during dynamic motion.

Under approximately static conditions:

```text
σa = 0.05 g
```

If the acceleration magnitude differs from 1 g by more than approximately 0.10 g:

```text
σa = 0.30 g
```

If the difference exceeds approximately 0.25 g:

```text
σa = 1.00 g
```

The corresponding accelerometer measurement-noise terms in `R` are increased, reducing the Kalman gain for accelerometer correction.

This helps reject obvious non-gravitational acceleration, but it is not equivalent to full motion-state detection.

---

# Noise and Covariance Parameters

The filter currently uses a combination of:

- parameters estimated from real sensor measurements
- fixed engineering assumptions
- empirically selected tuning values

Not every element of the filter covariance matrices is statistically identified from sensor data.

## Parameters estimated from measurements

The following quantities are derived from calibration measurements:

- initial gyroscope bias
- per-axis gyroscope variance
- gyroscope standard deviation used for attitude process noise
- accelerometer bias
- accelerometer-bias variance

Gyroscope contribution to attitude process noise is calculated approximately as:

```text
qθx = σgx² Δt²
qθy = σgy² Δt²
qθz = σgz² Δt²
```

---

## Manually selected or bounded parameters

Some filter parameters are currently engineering tuning values.

Examples include:

### Initial attitude uncertainty

```text
σroll  = 0.05 rad
σpitch = 0.05 rad
σyaw   = 15°
```

### Minimum gyroscope-bias uncertainty

```text
σbg >= 0.001 rad/s
```

### Minimum accelerometer-bias uncertainty

```text
σba >= 0.02 g
```

### Gyroscope bias random walk

```text
1e-5
```

### Accelerometer bias random walk

```text
1e-5
```

### Normalized magnetometer measurement noise

```text
σm = 0.25
```

### Accelerometer measurement noise

Depending on measured acceleration magnitude:

```text
0.05 g
0.30 g
1.00 g
```

The current process and measurement noise matrices mainly assume independent axis noise, therefore their corresponding off-diagonal noise covariance terms are zero.

A more rigorous stochastic model could be obtained using longer datasets, Allan variance analysis, temperature characterization and cross-axis covariance measurements.

---

# Current Limitations

## Dynamic Acceleration

The main limitation of the current attitude estimator is the treatment of accelerometer measurements during translational motion.

The accelerometer correction assumes that measured specific force is predominantly caused by gravity.

The application increases accelerometer measurement noise when the magnitude of acceleration differs significantly from 1 g.

However, full motion-state recognition is not currently implemented.

As a result, linear acceleration may still be interpreted as a change in the gravity direction.

For example, significant horizontal acceleration may occur while the total acceleration magnitude remains relatively close to 1 g.

In this situation the filter may temporarily produce an incorrect roll or pitch estimate.

Therefore, during dynamic motion the artificial horizon may show temporary false attitude indications.

Possible future improvements include:

- translational acceleration estimation
- motion-state detection
- innovation gating
- adaptive measurement covariance
- GNSS aiding
- external velocity measurements
- additional motion constraints

---

## Magnetometer Disturbances

Magnetometer measurements are sensitive to:

- ferromagnetic materials
- electric motors
- high-current wiring
- permanent magnets
- nearby electronic equipment
- changes in mounting configuration

The current implementation does not perform online magnetic disturbance detection.

Incorrect magnetic measurements can therefore temporarily affect the heading estimate.

---

## Magnetometer Calibration

Magnetometer calibration is currently external and manual.

The Qt application does not automatically calculate the hard-iron bias or soft-iron correction matrix.

The current workflow uses MotionCal and then manually transfers the resulting calibration parameters into the application.

---

## Statistical Model

The current sensor noise model is intentionally simplified.

Some covariance parameters are derived from measured sensor statistics, while others are manually selected.

The current implementation does not model all possible IMU error sources, including:

- temperature-dependent bias
- complete axis correlation
- scale-factor instability
- long-term bias instability
- correlated random walk
- long-term sensor drift

---

# Why ESKF?

A quaternion belongs to a nonlinear rotation manifold and is constrained to unit norm.

Direct additive correction of quaternion components inside a conventional Kalman filter is therefore inconvenient.

The Error-State Kalman Filter separates the nonlinear nominal orientation from a small three-dimensional attitude error:

```text
δθ
```

The filter estimates this small rotation error, converts it into a quaternion correction and injects it into the nominal orientation.

The error state is then reset.

This provides a convenient separation between:

- nonlinear nominal-state propagation
- linearized error estimation
- covariance propagation

---

# Visualization

## Artificial Horizon

The artificial horizon visualizes:

- roll
- pitch

The display uses the current filtered attitude estimate.

---

## Heading Indicator

The heading indicator visualizes magnetic heading.

The heading is obtained from the calibrated magnetometer using tilt compensation and is also used in the estimator correction.

In the current calibrated setup, a physical yaw rotation of approximately 90° produces approximately the same angular change on the heading indicator.

---

# Build

## Dependencies

Required software:

- CMake
- C++17-compatible compiler
- Qt 6
- Qt6 SerialPort
- Eigen3

Example installation on Ubuntu / Linux Mint:

```bash
sudo apt update

sudo apt install \
    build-essential \
    cmake \
    qt6-base-dev \
    libqt6serialport6-dev \
    libeigen3-dev
```

Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/qt-imu-ahrs.git
cd qt-imu-ahrs
```

Configure:

```bash
cmake -S . -B build
```

Build:

```bash
cmake --build build -j$(nproc)
```

---

# Running

Connect the ESP32 with the ICM-20948.

Make sure the serial device is available, for example:

```text
/dev/ttyUSB0
```

Run the Qt application.

The processing chain is:

```text
IMU measurement
      ↓
Serial communication
      ↓
Sensor calibration
      ↓
Quaternion propagation
      ↓
ESKF prediction
      ↓
Accelerometer / magnetometer correction
      ↓
Attitude visualization
```

---

# Calibration Workflow

## 1. Gyroscope

Keep the sensor stationary during initialization.

Stationary measurements are used to estimate:

- gyroscope bias
- gyroscope variance

---

## 2. Accelerometer

Perform the six-position accelerometer calibration.

The sensor is placed statically in different orientations so that each accelerometer axis experiences approximately:

```text
+1 g
-1 g
```

The resulting measurements are used to estimate accelerometer bias.

---

## 3. Magnetometer

Upload firmware that outputs raw IMU measurements in MotionCal-compatible format.

Start MotionCal and rotate the sensor through the full 3D orientation space.

A good calibration should produce an approximately spherical and well-distributed magnetic point cloud.

Copy the resulting:

- magnetic offset
- 3×3 correction matrix

into the application calibration constants.

---

# Project Status

The project is a working engineering prototype.

Current functionality includes:

- live IMU acquisition
- sensor calibration
- quaternion attitude propagation
- Error-State Kalman Filter
- accelerometer correction
- magnetometer correction
- tilt-compensated heading
- artificial horizon
- heading visualization
- real-time Qt interface

The current goal is not to provide a production-grade AHRS, but to implement and experimentally test the complete processing chain from a physical IMU to a real-time attitude estimation and visualization application.

---

# Possible Future Improvements

- automatic magnetometer calibration
- online magnetic disturbance detection
- motion-state detection
- translational acceleration estimation
- innovation gating
- adaptive `R` based on innovation statistics
- Allan variance sensor characterization
- temperature compensation
- persistent calibration storage
- configurable calibration parameters
- automatic serial-port detection
- recorded-data replay mode
- automated ESKF tests
- comparison against an external attitude reference
- GNSS or external velocity aiding

---

# Disclaimer

This project is an experimental engineering implementation created for learning, testing and portfolio purposes.

Although the estimator follows the conventional Error-State Kalman Filter architecture, the implementation has not undergone the validation required for safety-critical navigation, aviation, automotive or other high-reliability applications.
