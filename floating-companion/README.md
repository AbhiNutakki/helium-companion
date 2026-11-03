# Helium Companion — Prototype Development

A Gazebo model and control environment for developing a physical floating
companion, inspired by Cuddle-Fish: **helium buoyancy, two flapping side wings,
a passive tail and a moving internal ballast**. Use this environment to iterate
on the mechanism and control behavior before testing hardware.

There are no propellers, vertical
thrusters, people, tracking markers, cameras or following modes in this revision.

## Run

Double-click `start.command`, or run:

```sh
./start.command --gui
```

Open http://127.0.0.1:8765. The native Gazebo window shows the actual articulated
robot. The dashboard shows the same body's position, measured wing angles and
internal weight position. It does not run a separate animation/physics model.
Use Control+C in the launcher Terminal to stop its processes.

To rebuild:

```sh
./scripts/setup.sh
```

This uses the existing Homebrew Gazebo Jetty installation and Python 3.14.
The dashboard binds only to loopback and uses an isolated Gazebo partition.
The launcher generates world files with mesh paths for the current checkout;
generated worlds, build products and the virtual environment are excluded from Git.

## What to try

- **Neutral float:** helium supports the mass with the wings stationary. There is
  no altitude controller or vertical motor secretly holding it up.
- **Swim forward:** the two side wings flap symmetrically.
- **Turn left / right:** change relative wing activity; turning also moves the robot.
- **Climb / descend:** shift internal ballast to pitch the body, then swim along
  the tilted body axis. It cannot climb straight upward while stationary.
- **Ballast only:** wings rest while the 6 g weight slides forward; observe the
  nose pitch down from the change in weight distribution.
- **Glide to a stop:** wings stop and drag slows it. It cannot reverse thrust or
  brake sharply like the old propeller model.
- **Direct wing controls:** independently choose frequency (rest, or 0.3–1.5 Hz)
  and half-amplitude (0–50 degrees). Half-amplitude 50 means 100 degrees of sweep.
- **Keyboard:** hold W to flap; A/D apply differential flapping; R/F pitch up/down
  while swimming. S coasts. Releasing controls stops the flapping after a brief
  smoothing interval. Manual commands expire after 0.4 simulation seconds.
- **Trim error:** positive grams represent a passive downward buoyancy mismatch;
  negative grams an upward mismatch. It is fixed environmental load, not an actuator.
- **Fin coefficient:** change the estimated membrane thrust. Set it to zero to
  verify that joint motion alone does not create hidden propulsion.
- **Waypoints:** click the room map for experimental forward-only travel. The robot
  turns and then glides near the point. This is a limited demonstration, not precise
  position hold or route planning. It will stop flapping for an obstructed route.

## What comes from Cuddle-Fish

Source: Xu et al., *Cuddle-Fish: Exploring a Soft Floating Robot with Flapping Wings
for Physical Interactions*, section 3.2:
https://kaikunze.de/papers/pdf/xu2025cuddle.pdf

The model uses the paper's 1 × 0.45 × 0.33 m envelope dimensions, two side hinges,
0.3–1.5 Hz flapping range and maximum 100-degree sweep. Differential wing activity
steers the body. A third servo shifts an internal weight for pitch; the tail is
passive. These are the mechanisms, not performance measurements.

The paper lists a 70 g robot. The dimensions interpreted as an ellipsoid imply
77.75 L. This simulation explicitly includes helium mass and trims its total mass
to displaced-air mass: **95.25 g total, 82.50 g dry**, using air density 1.225 kg/m³
and helium density 0.164 kg/m³. It does not silently treat the published 70 g as a
complete gas-inclusive neutral mass. This is a chosen trim condition, not an exact
reproduction of the paper's mass accounting or envelope geometry.

## Physical implementation

The Gazebo model has four dynamic links:

1. Main ellipsoidal body, with its centre of mass 18 mm below the buoyancy centre.
2. Left membrane wing on a revolute joint.
3. Right membrane wing on a mirrored revolute joint.
4. A 6 g ballast on a prismatic joint, travelling ±110 mm along the body.

Wing links each have a mass of 3 g as a simplified actuator/wing allocation.
Servo masses in the paper do not establish the actual combined wing-link mass;
this simulation allocation is an assumption. Wing half-span is 165 mm, giving an
approximately 780 mm span when horizontal, narrowing as the wings fold down.

The three joints use bounded force/torque servo controllers. They are not cosmetic
poses: the physics engine resolves their motion and reaction forces. The ballast
changes pitch through actual native gravitational and joint forces. There is no
commanded external pitch torque.

The forward aerodynamic force on each wing is a **phenomenological rectified
membrane model**:

```
F_forward = 0.5 × rho_air × S × C_T × (r_eff × measured_hinge_speed)^2
            × (0.75 + 0.25 × cos(2 × measured_hinge_angle))
```

`S = 0.026 m²`, `r_eff = 0.11 m`, default `C_T = 1.2`. The rectification assumes
that membrane deformation produces net forward force over both strokes. It does
not resolve that deformation. Each force is applied at its physical wing link;
its vertical component comes only from the pitched body axis. Setting `C_T` to
zero removes both forward forces. Stopping the measured hinges removes sustained
thrust. No direct heave force or reversible thrust is present.

Buoyancy is applied at the envelope centre and is `rho_air × V × g`; gravity is
native Gazebo gravity on every link. There is no runtime mass normalization hack.
A trim setting is a constant extra passive vertical load; it does not change
inertia and is not intended as a payload collision test. Drag and angular damping
lump together the body, wings and passive tail and are estimated coefficients.

## Testing

### Compare prototype settings

```sh
.venv/bin/python scripts/experiment.py --frequencies .6 1.2 --coefficients 0 1.2 --seconds 6 --repeats 2
```

This starts its own headless Gazebo session without changing a running dashboard.
Each case resets the robot and compares symmetric wing frequency and an assumed
thrust coefficient. Use `--amplitude` to choose wing half-amplitude, `--factor`
to change simulation speed and `--output` to choose a new results directory.
The runner refuses to overwrite an existing directory.

Each run saves a `summary.json` with travel, speed, altitude change, mean wing
force and obstacle-stop status, plus a CSV trajectory for every case. Failed or
interrupted runs retain completed cases with `complete: false`. Results use
simulation time, not elapsed wall time. Short runs include startup transients;
obstacle stops can limit travel, so these are comparisons rather than estimates
of steady-state speed. The checked-in `reports/experiments/baseline/` illustrates
four cases, including a zero-thrust control. These results do not calibrate the
physical wing or validate hardware performance.
`reports/experiments/wing-comparison/` adds two repeats of each of those four
settings, with source provenance and the individual CSV traces.

### Acceptance checks

```sh
.venv/bin/python scripts/test_all.py
```

This runs structural/API/HTTP unit tests first, then the actual Gazebo acceptance
suite in a separate partition on an automatically selected loopback port. It
shuts down its owned processes afterward and stops immediately if unit tests fail.
Unit output is saved to `reports/unit-tests.txt`. Acceptance and experiment
reports record a source fingerprint and Git revision to identify the tested code.
The suite checks
passive float, floor collision, trim mismatch, forward swimming, both turn
signs, physical ballast pitching, climb, descent, glide, joint limits, zero fin
coefficient, manual timeout and obstacle stopping. Results are written to
`reports/results.json` with revision `helium_flapping_v2`. Earlier propeller and
tracking test results do not count as tests of this model.

Against an already-running simulation, with no simultaneous user commands:

```sh
.venv/bin/python tests/integration.py --url http://127.0.0.1:8765
```

## Limits

This is a simulator for exploring the mechanism, **not validated Cuddle-Fish
performance or a hardware-ready controller**. The paper does not supply a
complete thrust-versus-stroke data set, exact ballast geometry, drag coefficients
or all mass properties. Those values need bench measurements.

The envelope and membranes are rigid geometries, not soft-body or fluid/structure
simulation. There is no CFD, propeller model, measured battery estimate, noise
prediction, helium leakage, vision, person recognition or autonomous following.
The obstacle guard is a conservative planar geometric check against the known
room; it is not a real onboard proximity sensor. It stops flapping but cannot
actively reverse thrust to brake. Real safety is not established by this model.

## Files

- `src/Companion.cc`: native joints, servo controllers, buoyancy, wing-force model,
  manual controls, experimental waypoints and telemetry.
- `scripts/generate_world.py`: ellipsoidal envelope, moving ballast, physical wings,
  passive tail and room. Generated meshes are in `assets/`.
- `bridge.py`, `app.py`: local transport and process lifecycle; no camera pipeline.
- `web/index.html`: joint-driven robot schematic, room view and mechanism controls.
- `tests/`: model/API and Gazebo acceptance checks.
- `reports/results.json`: current revision's integration results.

If the port is occupied, stop the old launcher with Control+C or use
`./start.command --port 8767 --partition flapping_second_run`. If the native
Gazebo GUI is unstable, run without `--gui`; the browser schematic and actual
physics still work, and no rendered camera is needed.
