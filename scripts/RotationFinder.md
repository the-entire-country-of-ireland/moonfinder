# Rotation Alignment from Dot Product Constraints

## Problem Statement

Given two sets of 3D unit vectors measured in different reference frames:
- **Magnetometer readings** $\{\mathbf{m}_i\}$ in frame A
- **Accelerometer readings** $\{\mathbf{a}_i\}$ in frame B

Find a rotation matrix $\mathbf{R}$ such that the dot product between the rotated magnetometer vector and the accelerometer vector is approximately equal to a known target value $t$ for all pairs:

$$\mathbf{a}_i^\top (\mathbf{R} \, \mathbf{m}_i) \approx t$$

### Physical Motivation

In many sensor fusion applications, the angle between Earth's magnetic field and gravity vector is known (approximately 90° at the equator, varying by location). If magnetometer and accelerometer are mounted in different orientations, we need to find the rotation between their reference frames. The target value $t = \cos(\theta)$ where $\theta$ is the known angle between the fields.

---

## Mathematical Derivation

### Step 1: Linearization

The constraint for each pair is:

$$\mathbf{a}_i^\top \mathbf{R} \, \mathbf{m}_i = t$$

This expression is **linear** in the 9 elements of $\mathbf{R}$.

### Step 2: Vectorization

Using the Kronecker product identity:

$$\mathbf{a}^\top \mathbf{R} \, \mathbf{m} = \text{vec}(\mathbf{a} \mathbf{m}^\top)^\top \text{vec}(\mathbf{R}) = (\mathbf{m} \otimes \mathbf{a})^\top \mathbf{r}$$

where:
- $\mathbf{r} = \text{vec}(\mathbf{R})$ is the 9-element vectorization of $\mathbf{R}$
- $\mathbf{m} \otimes \mathbf{a}$ is the Kronecker product (9-element vector)

### Step 3: Linear System

Each measurement pair gives one linear equation. Stacking $n$ pairs:

$$\mathbf{A} \mathbf{r} = \mathbf{b}$$

where:
- $\mathbf{A}$ is $n \times 9$, with row $i$ equal to $(\mathbf{m}_i \otimes \mathbf{a}_i)^\top$
- $\mathbf{b}$ is $n \times 1$, with all entries equal to $t$

### Step 4: Normal Equations

The least-squares solution minimizes $\|\mathbf{A}\mathbf{r} - \mathbf{b}\|^2$:

$$\mathbf{r} = (\mathbf{A}^\top \mathbf{A})^{-1} \mathbf{A}^\top \mathbf{b}$$

Since $\mathbf{b} = t \cdot \mathbf{1}$:

$$\mathbf{r} = (\mathbf{A}^\top \mathbf{A})^{-1} \cdot t \cdot (\mathbf{A}^\top \mathbf{1})$$

### Step 5: Fixed-Size Accumulation

Define:

$$\mathbf{G} = \mathbf{A}^\top \mathbf{A} = \sum_{i=1}^{n} (\mathbf{m}_i \otimes \mathbf{a}_i)(\mathbf{m}_i \otimes \mathbf{a}_i)^\top$$

$$\mathbf{h} = \mathbf{A}^\top \mathbf{1} = \sum_{i=1}^{n} (\mathbf{m}_i \otimes \mathbf{a}_i)$$

Both have **fixed size** regardless of $n$:
- $\mathbf{G}$: 9 × 9 symmetric matrix
- $\mathbf{h}$: 9 × 1 vector

The solution becomes:

$$\mathbf{r} = \mathbf{G}^{-1} (t \cdot \mathbf{h})$$

### Step 6: Projection to SO(3)

The least-squares solution $\mathbf{r}$ reshaped to a 3×3 matrix may not be a valid rotation. Project to the nearest rotation matrix using SVD:

$$\mathbf{R}_{\text{raw}} = \text{reshape}(\mathbf{r})$$

$$\mathbf{U}, \mathbf{\Sigma}, \mathbf{V}^\top = \text{SVD}(\mathbf{R}_{\text{raw}})$$

$$\mathbf{R} = \mathbf{U} \mathbf{V}^\top$$

If $\det(\mathbf{R}) = -1$, negate the last column of $\mathbf{U}$ to ensure a proper rotation.

---

## Algorithm Summary

```
INITIALIZE:
    G ← 9×9 zero matrix
    h ← 9×1 zero vector

FOR each measurement pair (m, a):
    m ← m / ‖m‖
    a ← a / ‖a‖
    k ← m ⊗ a                    # 9-element Kronecker product
    G ← G + k kᵀ                 # outer product accumulation
    h ← h + k

SOLVE:
    r ← G⁻¹ (t · h)              # 9-element solution
    R_raw ← reshape(r, 3×3)
    U, Σ, Vᵀ ← SVD(R_raw)
    R ← U Vᵀ                     # nearest rotation matrix
    if det(R) < 0:
        R ← U · diag(1,1,-1) · Vᵀ
```

## Implementation

### Python (NumPy)

```python
import numpy as np

class RotationFinder:
    def __init__(self, target=0.907):
        self.target = target
        self.G = np.zeros((9, 9))
        self.h = np.zeros(9)
        self.R = np.eye(3)
    
    def add_sample(self, mag, acc):
        m = mag / np.linalg.norm(mag)
        a = acc / np.linalg.norm(acc)
        k = np.kron(m, a)
        self.G += np.outer(k, k)
        self.h += k
    
    def solve(self):
        r = np.linalg.solve(self.G, self.target * self.h)
        R_raw = r.reshape(3, 3, order='F')
        U, _, Vt = np.linalg.svd(R_raw)
        self.R = U @ Vt
        if np.linalg.det(self.R) < 0:
            U[:, -1] *= -1
            self.R = U @ Vt
        return self.R
```

### C++ (Eigen)

```cpp
#include <Eigen/Dense>

class RotationFinder {
    Eigen::Matrix<double, 9, 9> G;
    Eigen::Matrix<double, 9, 1> h;
    double target;
    
public:
    Eigen::Matrix3d R;
    
    RotationFinder(double t = 0.907) : target(t) {
        G.setZero();
        h.setZero();
        R.setIdentity();
    }
    
    void addSample(const Eigen::Vector3d& mag, const Eigen::Vector3d& acc) {
        Eigen::Vector3d m = mag.normalized();
        Eigen::Vector3d a = acc.normalized();
        
        Eigen::Matrix<double, 9, 1> k;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                k(i * 3 + j) = m(i) * a(j);
        
        G += k * k.transpose();
        h += k;
    }
    
    void solve() {
        Eigen::Matrix<double, 9, 1> r = G.ldlt().solve(target * h);
        
        Eigen::Matrix3d R_raw;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R_raw(j, i) = r(i * 3 + j);
        
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_raw, Eigen::ComputeFullU | Eigen::ComputeFullV);
        R = svd.matrixU() * svd.matrixV().transpose();
        
        if (R.determinant() < 0) {
            Eigen::Matrix3d U = svd.matrixU();
            U.col(2) *= -1;
            R = U * svd.matrixV().transpose();
        }
    }
};
```

---

## Practical Considerations

### Sufficient Data

The system $\mathbf{G}$ must be full rank (rank 9) for a unique solution. This requires:
- At least 9 measurement pairs
- Measurements spanning diverse orientations
- Avoid collinear or coplanar configurations

### Noise Sensitivity

The SVD projection step provides robustness to noise by finding the nearest valid rotation matrix. For high-noise scenarios, consider:
- Collecting more samples
- Filtering outliers before accumulation
- Weighting samples by confidence

### Target Value

The target $t = \cos(\theta)$ where $\theta$ is the angle between Earth's magnetic field and gravity. Typical values:
- Equator: $t \approx 0$ (perpendicular)
- Mid-latitudes: $t \approx 0.5$ to $0.9$
- Polar regions: $t \approx 0.9$ or higher

Look up the magnetic inclination angle for your location to compute the appropriate target.
