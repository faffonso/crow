# Terrasentia iLQR Tuning Handbook

This handbook covers the controller parameters in [crow/controllers/ilqr/config/ilqr_params.yaml](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/config/ilqr_params.yaml).

---

## 1. Time & Horizon

| Parameter | Intuition | Increase When… | Decrease When… |
|-----------|-----------|----------------|----------------|
| `dt` | Prediction sample time. Sets how far ahead each optimization knot looks. | Forward pass converges quickly but misses fast disturbances. | Solver takes too long; small `dt` causes numerical jitter in the backward pass. |
| `N` | Number of prediction steps. Combined with `dt`, defines the horizon (`N * dt`). | The robot needs smoother, anticipatory manoeuvres (e.g. long curves). | Backward pass becomes slow or regularization explodes. |

**Heuristic:** Start with `dt = 0.1` and `N = 15` (≈1.5 s horizon). Increase `N` for smoother paths (up to ~20–25). Decrease `dt` only if you observe coarse state predictions.

> [!NOTE]
> Longer horizons increase computational load quadratically because iLQR performs full forward and backward passes over `N` steps each iteration.

---

## 2. Reference & Limits

| Parameter | Intuition | Raising It… | Lowering It… |
|-----------|-----------|-------------|--------------|
| `v_ref` | Cruise speed used for rollout initialization and reference control `ur`. | Stretches the reference trajectory further; controller aims to move faster. | Requests slower motion; useful for precision manoeuvres or testing. |
| [v_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#132-136) | Hard cap on linear velocity used in the barrier function. | Permits higher wheel speeds (only if hardware allows). | Prevents solver from requesting unrealistic speeds; triggers earlier barrier penalty. |
| [omega_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#138-142) | Hard cap on angular velocity used in the barrier function. | Allows faster turns. | Tightens the angular constraint; safer for narrow rows. |

> [!WARNING]
> The barrier function goes to infinity as controls approach these limits. Setting them too tight causes the solver to struggle; setting them too loose defeats constraint enforcement.

---

## 3. Optimization Parameters

These control the iLQR iterative optimization loop.

| Parameter | Intuition | Increase When… | Decrease When… |
|-----------|-----------|----------------|----------------|
| `shoot_fac` | Exponential decay factor for single-shooting initial guess: `exp(shoot_fac * \|error\|)`. | Initial heading error is large but guesses are too timid. | Aggressive initial guesses cause divergence early. |
| `max_iter` | Maximum iterations per solve call. | Solver terminates before converging (cost still decreasing). | Computation time is too long; solution already good enough. |
| `tol` | Convergence tolerance on `delta_J`. Solver stops when improvement is below this. | You need faster average solve times and can tolerate small suboptimality. | Solution quality is poor; need tighter convergence. |

### Regularization (internal, not in YAML)

Within [fit()](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/ilqr.cpp#142-205), a Tikhonov regularization parameter `regu` is dynamically adjusted:
- **Starts at 1.0**
- **Multiplied by 0.7** when a feasible improvement is found.
- **Multiplied by 2.2** when no alpha step accepts.
- Clamped between `min_regu = 0.001` and `max_regu = 1000`.

You cannot tune `regu` directly, but if you observe the solver exiting early due to hitting `max_regu`, try:
- Reducing `N` or adjusting weights to improve conditioning.
- Increasing `max_iter` to allow more iterations to escape.

---

## 4. Terminal Cost Weights (Qf)

The **terminal cost matrix Qf** penalizes the final predicted state error. These weights drive the robot toward the goal at the end of the horizon.

| Parameter | Behaviour when Increased | Symptoms if Too High | Symptoms if Too Low |
|-----------|--------------------------|----------------------|---------------------|
| `Qf_x` | Strongly penalizes longitudinal error at horizon end (reaches goal x). | Large accelerations near end; may overshoot. | Robot stops short of goal x. |
| `Qf_y` | Strongly penalizes lateral error at horizon end. | Aggressive last-moment steering. | Robot finishes off-center from the row. |
| `Qf_theta` | Penalizes heading error at the final step. | Jerky yaw corrections near the end. | Robot arrives with wrong heading; needs correction on next cycle. |

> [!TIP]
> Terminal weights are typically **smaller** than running weights (contrary to some MPC setups) because iLQR's backward pass already propagates value through time. Start with `Qf_* ≈ 1–2` and increase only if you see steady-state offset.

---

## 5. Running Cost Weights (Q)

The **running cost matrix Q** penalizes state error at every intermediate prediction step. These weights determine how aggressively the controller tracks the path throughout the horizon.

| Parameter | Behaviour when Increased | Symptoms if Too High | Symptoms if Too Low |
|-----------|--------------------------|----------------------|---------------------|
| `Q_x` | Tightens longitudinal path tracking (keeps robot on schedule). | Large forward velocity swings; may hit [v_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#132-136). | Robot lags behind or overshoots waypoints. |
| `Q_y` | Aggressively reduces lateral row error. | Violent steering; angular commands saturate. | Robot drifts away from the furrow center. |
| `Q_theta` | Keeps the robot facing the trajectory direction. | Rapid heading corrections; oscillatory yaw. | Robot may reverse to match position instead of rotating. |

> [!IMPORTANT]
> In the **error-tracking dynamics model**, the state is **relative** to a reference frame aligned with the robot's heading. This means `Q_x` penalizes forward/backward error, `Q_y` penalizes left/right deviation, and `Q_theta` penalizes heading offset.

---

## 6. Control Effort Weights (R)

The **control cost matrix R** penalizes the magnitude of control inputs. Larger values smooth commands but reduce agility.

| Parameter | Behaviour when Increased | Use When… |
|-----------|--------------------------|-----------|
| `R_v` | Dampens linear velocity commands; discourages large speed changes. | Speed oscillates or barrier function is hit too often. |
| `R_omega` | Dampens angular velocity commands; reduces sharp pivots. | Robot wiggles left/right or yaw is jerky. |

**Key Insight:** Unlike some MPC formulations that penalize **change** in control (`Δu`), this iLQR penalizes the **absolute** control magnitude. This means `R` directly limits how large `v` and [omega](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#138-142) can grow, working in tandem with the barrier function.

---

## 7. Barrier Function Parameters

The barrier function enforces velocity limits **softly** within the cost rather than as hard constraints. It is defined as:

```
c = (1/t) * [ -log(1/eps + (v_max - v)/(v_max/2)) 
            - log(1/eps + v/(v_max/2)) 
            - log(1/eps + (omega_max - omega)/omega_max) 
            - log(1/eps + (omega + omega_max)/omega_max) ]
```

| Parameter | Intuition | Increase When… | Decrease When… |
|-----------|-----------|----------------|----------------|
| `eps` | Numerical offset inside log to prevent singularity at the boundary. | Solver hits NaN near limits (rare). | Barrier is too soft; controls exceed limits. |
| `t` | Sharpness of the barrier. Smaller `t` → steeper penalty near limits. | Constraints are violated; need tighter enforcement. | Solver struggles because barrier gradients are too large. |

**Typical Values:** `eps = 100`, `t = 100`. These are usually set once and left alone unless you see constraint violations or solver instability near boundaries.

---

## 8. Dynamics Model

```yaml
controller/dynamics:
  dt: *dt
  model: "error-tracking"
```

| Model | Description | When to Use |
|-------|-------------|-------------|
| `"standard"` | Classic unicycle kinematics: `x = [x, y, θ, v, ω]`. | General waypoint following without a reference trajectory. |
| `"error-tracking"` | State is the **error** relative to a reference trajectory: `e = [e_x, e_y, e_θ]`. | Path-following where you have a reference velocity `v_ref`. |

> [!NOTE]
> The **error-tracking** model incorporates a reference velocity `ur = [v_ref, omega_ref]` into the dynamics. This is ideal for row-following where you want the robot to track a virtual reference moving at `v_ref`.

---

## 9. Practical Tuning Workflow

1. **Choose a horizon** – Set `dt` and `N` to achieve a 1–2 s look-ahead. Verify the solver completes within your control loop period (check `/terrasentia/ilqr_time`).

2. **Set velocity targets** – Adjust `v_ref`, [v_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#132-136), and [omega_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#138-142) to match your desired cruise speed and hardware limits.

3. **Dial in running weights** – Start with:
   - `Q_x = Q_y = 10` (aggressive tracking)
   - `Q_theta = 2–3` (moderate heading correction)
   - Increase `Q_y` if lateral drift is too large; increase `Q_theta` if heading wanders.

4. **Smooth the commands** – Raise `R_v` and `R_omega` if outputs chatter or hit barrier limits. Lower them if the robot is sluggish.

5. **Refine terminal behavior** – If the robot consistently stops short or overshoots at the end of the horizon, adjust `Qf_*` weights.

6. **Check barrier parameters** – Only touch `eps` and `t` if you see constraint violations or NaN in the cost.

---

## 10. Common Symptoms & Fixes

| Symptom | Likely Cause | Adjustment |
|---------|--------------|-----------|
| Robot oscillates left/right | `Q_y` too high relative to `R_omega`. | ↓ `Q_y` or ↑ `R_omega`. |
| Robot lags behind reference | `Q_x` too low or `R_v` too high. | ↑ `Q_x` or ↓ `R_v`. |
| Robot spins in place | `Q_theta` too high; heading weight dominates. | ↓ `Q_theta` or ↑ `R_omega`. |
| Solver times out / hits `max_regu` | Horizon too long or cost matrices ill-conditioned. | ↓ `N`, ↓ extreme Q/Qf weights, ↑ `tol`. |
| Controls hit limits constantly | Barriers not active enough or limits too tight. | ↓ `t` for sharper barrier, or relax limits. |
| Robot reverses instead of rotating | `Q_theta` too low; positional weights dominate. | ↑ `Q_theta`. |
| Jerky commands at horizon end | `Qf_*` weights too large. | ↓ `Qf_x`, `Qf_y`, `Qf_theta`. |
| NaN in cost / solver crash | Barrier singularity or division by zero. | ↑ `eps`, check [v_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#132-136)/[omega_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#138-142) > 0. |

---

## 11. Debugging Tools

- **`/terrasentia/ilqr_time`** – Publishes solve time per cycle. Monitor for spikes or timeouts.
- **`/terrasentia/cmd_vel`** – Published `TwistStamped` with the first control action. Check for saturation.
- **`/terrasentia/path`** – Predicted trajectory for visualization in RViz.

Uncomment the `ROS_INFO_STREAM` lines in [ilqr.cpp](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/ilqr.cpp) ([run()](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/ilqr.cpp#65-141) and [fit()](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/ilqr.cpp#142-205)) to log:
- State error `_x0`
- Reference `_xref`
- Final cost `_J`
- Iteration count and regularization `regu`

---

## 12. Quick Reference: Parameter Summary

| Section | Parameter | Default | Effect |
|---------|-----------|---------|--------|
| **Horizon** | `N` | 15 | Look-ahead steps |
| | `dt` | 0.1 | Time step [s] |
| **Speed** | `v_ref` | 0.8 | Cruise reference [m/s] |
| | [v_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#132-136) | 0.9 | Linear velocity limit [m/s] |
| | [omega_max](file:///home/companion/catkin_ws/src/crow/controllers/ilqr/src/utils/cost.cpp#138-142) | 1.0 | Angular velocity limit [rad/s] |
| **Optimizer** | `shoot_fac` | 0.1 | Initial guess aggressiveness |
| | `max_iter` | 50 | Max iLQR iterations |
| | `tol` | 0.01 | Convergence tolerance |
| **Terminal (Qf)** | `Qf_x` | 2.0 | Final x weight |
| | `Qf_y` | 2.0 | Final y weight |
| | `Qf_theta` | 1.0 | Final heading weight |
| **Running (Q)** | `Q_x` | 10.0 | Path tracking x |
| | `Q_y` | 10.0 | Path tracking y |
| | `Q_theta` | 2.5 | Path tracking heading |
| **Control (R)** | `R_v` | 1.0 | Linear effort penalty |
| | `R_omega` | 0.5 | Angular effort penalty |
| **Barrier** | `eps` | 100 | Log offset |
| | `t` | 100 | Barrier sharpness |