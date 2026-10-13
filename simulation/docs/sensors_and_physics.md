# Physics & Sensor Mathematical Modeling

## 1. Attitude Estimation (Complementary Filter)

### 1.1 Accelerometer Pitch & Roll Estimation
On embedded controllers, calculating intermediate squared acceleration values using signed 16-bit integers leads to arithmetic overflow, causing negative root arguments and generating `NaN` output. Values are cast to single-precision floating point to preserve stability:
$$\theta_{\text{acc}} = \text{atan2}\left(-a_x, \sqrt{a_y^2 + a_z^2}\right)$$
$$\phi_{\text{acc}} = \text{atan2}\left(a_y, a_z\right)$$

### 1.2 Rate Gyroscope Integration
Raw gyroscope angular rates $\vec{\omega} = [\omega_x, \omega_y, \omega_z]^T$ in $\text{rad/s}$ are integrated over sample interval $\Delta t$:
$$\Delta \theta_{\text{gyro}} = \omega_y \Delta t, \quad \Delta \phi_{\text{gyro}} = \omega_x \Delta t$$

### 1.3 State Update Formulation
Fusing gravity reference vectors with integrated angular rates using filtering factor $\alpha = 0.90$:
$$\theta_k = \alpha (\theta_{k-1} + \omega_y \Delta t) + (1 - \alpha) \theta_{\text{acc}} \quad (\text{Pitch state})$$
$$\phi_k = \alpha (\phi_{k-1} + \omega_x \Delta t) + (1 - \alpha) \phi_{\text{acc}} \quad (\text{Roll state})$$

---

## 2. Terrain Slope & Surface Normal Extraction

The three VL53L0X ToF rangefinders form an equilateral triangle with radial offset $r = 0.06\text{ m}$ on the ventral sensor plate:
* **Sensor 1 (Fore):** $(r, 0)$
* **Sensor 2 (Aft-Port):** $(-r/2, r\sqrt{3}/2)$
* **Sensor 3 (Aft-Starboard):** $(-r/2, -r\sqrt{3}/2)$

Using surface contact points $\mathbf{p}_i = [x_i, y_i, -d_{\text{ToF}, i}]^T$, planar span vectors and the surface normal unit vector $\hat{\mathbf{n}}$ are derived via vector cross-product:
$$\mathbf{u} = \mathbf{p}_2 - \mathbf{p}_1, \quad \mathbf{v} = \mathbf{p}_3 - \mathbf{p}_1$$
$$\hat{\mathbf{n}} = \frac{\mathbf{u} \times \mathbf{v}}{\Vert{}\mathbf{u} \times \mathbf{v}\Vert{}}$$

The local ground inclination angle $\beta$ relative to the vertical axis is:
$$\beta = \arccos\left(\frac{\vert{}\hat{n}_z\vert{}}{\Vert{}\hat{\mathbf{n}}\Vert{}}\right)$$

Terrain exhibiting $\beta > 0.2618\text{ rad}$ ($15^\circ$) exceeds dynamic rollover limits and triggers an abort.

---

## 3. Acoustic-Optical Texture Signature

* **VL53L0X ToF:** Narrow optical beam ($\approx 25^\circ$) that penetrates sparse foliage, returning reflections from inner branches or hard underlying surfaces.
* **HC-SR04 Sonar:** Wide acoustic conical wavefront ($30^\circ\text{--}40^\circ$) that reflects off outer foliage tips and canopy boundaries.

$$\bar{d}_{\text{ToF}} = \frac{1}{3} \sum_{i=1}^3 d_{\text{ToF}, i}, \quad \Delta d = \bar{d}_{\text{ToF}} - d_{\text{sonar}}$$

* **Solid Concrete / Rigid Runway:** $\Delta d \approx 0\text{ m}$ with temporal stability.
* **Vegetated Mound / Canopy:** $\Delta d < -0.05\text{ m}$ with elevated temporal dispersion $\sigma^2_{\Delta d}$.

---

## 4. 7-Dimensional Classification Feature Vector

The `sensing_domain::sensor_processor` maintains a rolling temporal window ($N = 10\text{ samples}$) and publishes the 7D feature vector $\vec{x} \in \mathbb{R}^7$ at 40 Hz:

$$\vec{x} = \begin{bmatrix} \bar{d}_{\text{ToF}} \\ \sigma^2_{\text{spatial}} \\ \Delta d \\ \sigma^2_{\Delta d} \\ \beta \\ \Vert{}\vec{\omega}\Vert{} \\ e_{\text{tilt}} \end{bmatrix} = \begin{bmatrix} \text{Mean optical distance across the 3 ToF sensors} \\ \text{Spatial sample variance across the 3 ToF ground returns} \\ \text{Acoustic-optical range differential } (\bar{d}_{\text{ToF}} - d_{\text{sonar}}) \\ \text{Temporal variance of range differential over 10 samples} \\ \text{Estimated local terrain slope angle derived from normal vector} \\ \text{IMU angular velocity magnitude } \sqrt{\omega_x^2 + \omega_y^2 + \omega_z^2} \\ \text{Tilt error between aircraft frame and local level vector} \end{bmatrix}$$

---

[**Back to main README**](../README.md)