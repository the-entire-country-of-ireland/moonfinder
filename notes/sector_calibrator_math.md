# Sector Calibrator: Quadratic Objectives

## 1. Model and notation

For a raw magnetometer sample $m_k\in\mathbb R^3$, append a homogeneous coordinate:

$$
\bar m_k = \begin{bmatrix}m_k\\1\end{bmatrix},
\qquad
y_k = W\bar m_k = A m_k+t,
$$

where $W=[A\;t]\in\mathbb R^{3\times4}$. The 12-vector $w=\operatorname{vec}(W)$ uses **column-major** order:

$$
w=(A_{00},A_{10},A_{20},A_{01},\ldots,A_{22},t_0,t_1,t_2)^T.
$$

For accepted accelerometer samples, $\hat a_k=a_k/\|a_k\|$.

## 2. Norm objective: streamed per sample

The calibrated field is constrained to unit magnitude:

$$
r_{n,k}=\|y_k\|^2-1
=\bar m_k^T(W^TW)\bar m_k-1.
$$

Let $Q=W^TW$ and let $q=\operatorname{vech}(Q)$ in the order

$$
(00,11,22,33,01,02,03,12,13,23).
$$

Define

$$
\phi_k=\bigl(m_x^2,m_y^2,m_z^2,1,2m_xm_y,2m_xm_z,2m_x,
2m_ym_z,2m_y,2m_z\bigr)^T,
$$

so that $r_{n,k}=\phi_k^Tq-1$. The loss is

$$
L_n(W)=\frac12\sum_k(\phi_k^Tq-1)^2.
$$

It is accumulated exactly by the per-sample statistics

$$
G_n=\sum_k\phi_k\phi_k^T,\qquad h_n=\sum_k\phi_k,\qquad N=\#\{k\},
$$

because

$$
L_n=\frac12\left(q^TG_nq-2h_n^Tq+N\right).
$$

**Streaming:** `G_norm`, `h_norm`, and `norm_count` update for every accepted sample.

## 3. Constant accelerometer-magnetometer dot objective: streamed per sample

The ideal dot product between the calibrated magnetic field and gravity direction is constant for a fixed local field:

$$
r_{d,k}=\hat a_k^Ty_k-c,$$

where $c$ is an unknown scalar. Define $d_k=\hat a_k^T W\bar m_k$ and

$$
\psi_k=\begin{bmatrix}
 m_x\hat a_k\\m_y\hat a_k\\m_z\hat a_k\\\hat a_k
\end{bmatrix}\in\mathbb R^{12},
\qquad d_k=\psi_k^Tw.
$$

With $x=[w;c]\in\mathbb R^{13}$, define $b_k=[\psi_k;-1]$. Then

$$
L_d(W,c)=\frac12\sum_k(b_k^Tx)^2
=\frac12x^TG_dx,
\qquad G_d=\sum_kb_kb_k^T.
$$

**Streaming:** `G_dot` updates per accepted sample. The unknown $c$ is solved jointly with $W$ and is initialized from the current mean dot product.

## 4. Sector finite-rotation objective: accumulated per sector

Within sector $s$, accelerometer directions lie approximately on a great circle. The sector finalizer estimates its plane normal $n_s$ as the smallest-eigenvalue eigenvector of

$$
S_s=\sum_{k\in s}\hat a_k\hat a_k^T.
$$

A phase basis $(e_{1,s},e_{2,s})$ is formed from the first projected accelerometer sample, and

$$
\theta_{s,k}=\operatorname{unwrap}\!\left(\operatorname{atan2}
(\hat a_k^Te_{2,s},\hat a_k^Te_{1,s})\right).
$$

Let $U_{s,k}=R(n_s,-\theta_{s,k})$. The magnetometer design matrix $M(m_k)\in\mathbb R^{3\times12}$ satisfies

$$
M(m_k)w=W\bar m_k.
$$

After inverse rotation, the predicted fixed field is

$$
z_{s,k}=U_{s,k}M(m_k)w=B_{s,k}w.
$$

The nuisance fixed field for this sector is eliminated by centering:

$$
\bar z_s=\frac1{K_s}\sum_{k\in s}z_{s,k},
\qquad
L_{p,s}(W)=\frac12\sum_{k\in s}\|z_{s,k}-\bar z_s\|^2.
$$

Define

$$
S_{B,s}=\sum_kB_{s,k},\qquad
S_{BB,s}=\sum_kB_{s,k}^TB_{s,k}.
$$

Then

$$
L_{p,s}(W)=\frac12w^T\left(S_{BB,s}-\frac1{K_s}S_{B,s}^TS_{B,s}\right)w.
$$

The C++ code symmetrizes this matrix and adds it to the global `G_pitch`:

$$
G_p=\sum_s\frac12(G_{p,s}+G_{p,s}^T),
\qquad L_p=\frac12w^TG_pw.
$$

**Per-sector:** unlike the norm and dot terms, $B_{s,k}$ depends on sector-specific $n_s$ and $\theta_{s,k}$. The current C++ implementation retains only the active sector temporarily and commits its 12x12 contribution at `finishSector()`.

## 5. Perpendicular magnetic-component objective: accumulated per sector

For each sample, remove the calibrated magnetic component parallel to gravity:

$$
h_{s,k}=\left(I-\hat a_{s,k}\hat a_{s,k}^T\right)y_{s,k}.
$$

To compare samples within a sector, express this horizontal component in the sector reference frame:

$$
u_{s,k}=U_{s,k}h_{s,k}=C_{s,k}w,
\qquad
C_{s,k}=U_{s,k}\left(I-\hat a_{s,k}\hat a_{s,k}^T\right)M(m_{s,k}).
$$

The physical statement is that $u_{s,k}$ should be constant within a sector. Eliminating its unknown sector mean gives

$$
L_{\perp,s}(W)=\frac12\sum_{k\in s}\|u_{s,k}-\bar u_s\|^2,
\qquad
\bar u_s=\frac1{K_s}\sum_{k\in s}u_{s,k}.
$$

With

$$
S_{C,s}=\sum_kC_{s,k},\qquad
S_{CC,s}=\sum_kC_{s,k}^TC_{s,k},
$$

the sector contribution is

$$
G_{\perp,s}=S_{CC,s}-\frac1{K_s}S_{C,s}^TS_{C,s},
\qquad
L_{\perp,s}=\frac12w^TG_{\perp,s}w.
$$

Thus this objective **is streamable within a sector**: only $S_C$, $S_{CC}$, and the temporary accelerometer samples needed to estimate $n_s$ and $\theta_{s,k}$ are required. At sector end, $G_{\perp,s}$ is symmetrized and added to global `G_perp`. It must not be centered across all sectors, because each sector has its own unknown horizontal mean.

## 6. Prior objective

The optional initializer anchor is

$$
L_r(W)=\frac12\|w-w_0\|^2,
$$

with $w_0=\operatorname{vec}(W_{prior})$. It is not data-streamed; it is applied during solving.

## 7. Total loss and LM/Gauss-Newton solve

The C++ solver minimizes

$$
L=\lambda_pL_p+\lambda_{\perp}L_{\perp}+\lambda_nL_n+\lambda_dL_d+\lambda_rL_r.
$$

At the current $W$, $q=\operatorname{vech}(W^TW)$ is nonlinear in $w$. The solver uses

$$
J_q=\frac{\partial q}{\partial w},
\qquad s_n=G_nq-h_n,
$$

and the Gauss-Newton contribution

$$
H_n=J_q^TG_nJ_q,\qquad g_n=J_q^Ts_n.
$$

The pitch, dot, and prior terms are quadratic and contribute exactly:

$$
H_p=G_p,\ g_p=G_pw;
\qquad H_d=G_d,\ g_d=G_dx;
\qquad H_r=I,\ g_r=w-w_0.
$$

The damped step solves

$$
\left(H+\mu\operatorname{diag}(|\operatorname{diag}(H)|+10^{-6})\right)\Delta=-g.
$$

A trial is accepted only when it lowers the true nonlinear loss. Accepted steps reduce $\mu$; rejected steps increase it. The scalar $c$ is clamped to $[-2,2]$, matching the C++ implementation.

## 8. Batch and streaming APIs

The Python reference module `scripts/sector_calibrator.py` supports both paths:

```python
cal = SectorCalibrator()
for mag, acc in zip(mag_sector, acc_sector):
    cal.ingest(mag, acc, start_sector=(first_sample),
               end_sector=(last_sample))
result = cal.solve()
```

or whole sectors:

```python
for mag_sector, acc_sector in sectors:
    cal.add_sector(mag_sector, acc_sector)
result = cal.solve()
```

The Python and C++ implementations now include this perpendicular term as `G_perp` / `G_perp_` and `perp_weight`. The global norm and dot statistics remain sample-streamed; pitch and perpendicular statistics are sector-streamed and committed at `finishSector()`.
