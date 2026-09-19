# Physics & Sensor Mathematical Modeling

## 1. Attitude Estimation (Complementary Filter)

### 1.1 Accelerometer Formulation & Overflow Protection
On 16-bit embedded targets, calculating intermediate squared acceleration values using standard integers leads to arithmetic overflow, causing negative square-root arguments and producing `NaN` results. Casting values to single-precision floating point resolves the issue:
$$\theta_{\text{acc}} = \text{atan2}\left(-a_x, \sqrt{(\text{float})a_y^2 + (\text{float})a_z^2}\right) \cdot \frac{180^\circ}{\pi}$$
$$\phi_{\text{acc}} = \text{atan2}\left(a_y, a_z\right) \cdot \frac{180^\circ}{\pi}$$

### 1.2 Rate Gyroscope Bias Correction & Sensitivity
Raw gyroscope readings are bias-corrected using startup static offsets and scaled by the sensor sensitivity ($131.0\text{ LSB}/(^\circ/\text{s})$ for $\pm 250^\circ/\text{s}$ mode):
$$\omega_x = \frac{g_x - b_{g,x}}{131.0}, \quad \omega_y = \frac{g_y - b_{g,y}}{131.0}, \quad \omega_z = \frac{g_z - b_{g,z}}{131.0}$$

### 1.3 State Update Formulation
Fusing the gravitational vector with integrated rate gyroscopes using filter coefficient $\alpha = 0.90$:
$$\theta_k = \alpha (\theta_{k-1} + \omega_y \Delta t) + (1 - \alpha) \theta_{\text{acc}}$$
$$\phi_k = \alpha (\phi_{k-1} + \omega_x \Delta t) + (1 - \alpha) \phi_{\text{acc}}$$
$$\psi_k = \psi_{k-1} + \omega_z \Delta t \quad (\text{Dead-reckoning heading})$$

---

## 2. Terrain Slope & Surface Normal Extraction

The three VL53L0X ToF sensors form an equilateral triangle of radius $r = 0.06\text{ m}$ on the stabilized plate:

* **Sensor 1 (Fore):** $(r, 0)$
* **Sensor 2 (Aft-Left):** $(-r/2, r\sqrt{3}/2)$
* **Sensor 3 (Aft-Right):** $(-r/2, -r\sqrt{3}/2)$

Each sensor has a narrow Field of View ($\approx 25^\circ$) modeled via localized ray casting. Using contact points $\vec{P}_i = [x_i, y_i, d_{\text{ToF}, i}]^T$, the surface vectors and normal unit vector are computed as:

$$\vec{v}_{12} = \vec{P}_2 - \vec{P}_1, \quad \vec{v}_{13} = \vec{P}_3 - \vec{P}_1$$
$$\vec{N} = \vec{v}_{12} \times \vec{v}_{13}, \quad \hat{n} = \frac{\vec{N}}{\Vert{}\vec{N}\Vert{}}$$

The inclination angle $\beta$ relative to gravity $\vec{g} = [0, 0, -1]^T$ is:
$$\beta = \arccos(\hat{n} \cdot (-\vec{g})) = \arccos(n_z)$$
Terrain exhibiting $\beta > 15^\circ$ is classified as `UNSAFE`.

---

## 3. Acoustic-Optical Texture Signature

* **VL53L0X ToF:** Narrow optical cone ($25^\circ$) that penetrates sparse foliage to reflect from deeper structures.
* **HC-SR04 Sonar:** Wide acoustic cone ($30^\circ\text{--}40^\circ$) that specularly reflects off upper surface asperities or canopy tops.

$$\Delta d = d_{\text{sonar}} - \bar{d}_{\text{ToF}} = d_{\text{sonar}} - \frac{1}{3}\sum_{i=1}^3 d_{\text{ToF}, i}$$

* **Solid Flat Ground:** $\Delta d \approx 0$ with low temporal variance.
* **Porous Vegetation / Soft Surface:** $\Delta d < 0$ with elevated variance $\sigma^2(\Delta d)$.

---

## 4. 7D Classification Feature Vector

The Intelligence Domain evaluates a rolling buffer of 10 samples ($0.1\text{ s}$ window at $100\text{ Hz}$):
$$\vec{x} = \begin{bmatrix} \bar{d}_{\text{ToF}} \\ \sigma^2(d_{\text{ToF}}) \\ \Delta d \\ \sigma^2(\Delta d) \\ \beta \\ \Vert{}\vec{\omega}\Vert{} \\ \Vert{}\theta_k - \beta\Vert{} \end{bmatrix} = \begin{bmatrix} \text{Mean altitude from the 3 ToF sensors} \\ \text{Spatial dispersion across the 3 ToF range points} \\ \text{Differential acoustic-optical distance} \\ \text{Temporal variance of acoustic differential} \\ \text{Estimated terrain slope angle} \\ \text{Norm of angular velocity (turbulence metric)} \\ \text{Angular error between vehicle attitude and terrain slope} \end{bmatrix}$$

---

[**Back to main README**](../README.md)