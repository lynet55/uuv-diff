# Isaac Sim plant vs. MPPI prediction model

## The two models

| | Isaac Sim (MarineGym, `marinegym/robots/drone/underwaterVehicle.py`) | MPPI (`uuv-diff/models/fossen_diff.py`) |
|---|---|---|
| Frames | world z-up, body FLU | world NED, body FRD |
| State | PhysX rigid body: pose, quaternion, 6-DOF twist | `x = [ν (6, body) , p (3, NED), R (3×3)]` |
| Input used here | `[u, v, w, r] ∈ [-1, 1]` × `[0.45, 0.30, 0.25, 0.5]` | `τ = [Fx Fy Fz Tx Ty Tz]` (body) |
| Dynamics | PhysX integrates gravity, plus buoyancy, damping, added mass and Coriolis applied as external forces, `dt = 0.02`, 2 substeps | `(M_RB + M_A) ν̇ + C(ν)ν + D(ν)ν + g(η) = τ`, explicit Euler, `DT = 0.05` |
| Actuation | **`simple_block_4d: true`**: the commanded velocity is written into the body each step, thrusters are zeroed | forces/torques directly |
| Roll/pitch | angular velocity set to `[0, 0, r]` in world, so roll/pitch are held level | full 6-DOF with restoring moments |

So in this setup Isaac is a **kinematic** plant: whatever velocity arrives is applied. The MPPI's dynamic model only decides *which* velocity to send.

## From wrench to velocity (one control tick)

```
odom (z-up pose, world twist)
  │  ros_io.Inputs:  R = F R_zup F,  ν = F R_zupᵀ v_world,  p = F p_zup,   F = diag(1,-1,-1)
  ▼
x0 ──MPPI (K=1024 rollouts × T=30 × DT=0.05 s)──► τ0 (first wrench of the optimal plan)
  │  ν⁺ = ν + DT · (M_RB+M_A)⁻¹ (τ0 − C ν − D ν − g)        one Euler step of the same model
  ▼
[u, v, w, r]_FLU = [ν⁺₁, −ν⁺₂, −ν⁺₃, −ν⁺₆]   (FRD → FLU)
  │  ÷ VEL_MAX, clip to [-1, 1]  →  UDP :15000 {"action": [...]}
  ▼
Isaac: v_cmd_b = a ⊙ [u,v,w]_max,  v_world = R·v_cmd_b,  ω_world = [0, 0, a₄·r_max]
       → set_velocities(), then PhysX steps
```

The wrench never reaches the sim. It's converted to the velocity the fossen model *predicts* one step later, and that velocity is imposed.

## Mismatches and likely mistakes

1. **Rotational added mass is wrong in fossen.** `hydro[3:6] = -5.5` (K_ṗ, M_q̇, N_ṙ) vs. `0.12` in MarineGym and in the BlueROV2 literature. It looks copied from X_u̇. With inertia `0.16 kg m²` this makes the MPPI think the vehicle is ~35× harder to rotate, so its yaw plans are far too slow.
2. **Heave quadratic damping typo.** fossen `-39.99` vs. MarineGym `36.99`.
3. **Buoyancy differs and is unverified.** fossen uses `W/B = 0.98` (2 % positively buoyant). MarineGym uses `B = 997·9.8·0.011346 ≈ 110.9 N` against the USD mass (not checked; at 11.5 kg, `W ≈ 112.8 N` → slightly *negatively* buoyant).
4. **The sim's physics still acts in block mode.** The velocity is overwritten before each step, but PhysX then integrates gravity, buoyancy and hydro forces over that step. MarineGym's `calculate_acc` also finite-differences a velocity that was just teleported, giving spurious added-mass forces. Result: depth drifts (seen in a run: z −0.80 → −1.18 m) and the MPPI has to keep correcting heave.
5. **Time bases don't match.**
   - The MPPI predicts with `DT = 0.05 s` and ticks at 20 Hz wall time.
   - The sim steps `0.02 s × 2` per env step and runs slower than real time with the GUI (~4 Hz).
   - So several MPPI ticks land on one sim step, and the predicted `ν⁺` is held for a different duration than the model assumed.
   - Telemetry is also stamped at receive time, not sim time.
6. **One-step velocity is a small increment.** `ν⁺ − ν = DT·ν̇` is a few cm/s per tick. Progress comes from feedback: the sim adopts `ν⁺`, odom reports it, and the next tick adds another increment. It works, but the effective plant is an integrator the model doesn't know about.
7. **Yaw rate frame.** The MPPI sends body `r`, and the sim applies it as world z-rate. They're equal only while level, which block mode enforces, so this is fine as long as block mode is on.
8. **Roll/pitch/heave coupling is wasted.** The MPPI models restoring moments the sim never shows, and its rollouts can tilt while the sim stays level.

### Suggested fixes, in order
- Fix 1 and 2 in `fossen_diff.py` (one-line parameter changes).
- Either run the sim with thrusters (`simple_block_4d: false`, sending the wrench through the allocation) so both sides are dynamic, or make the MPPI use a kinematic/first-order velocity model so the model matches the plant.
- Stamp telemetry with sim time and run the MPPI on sim steps, not wall time.
