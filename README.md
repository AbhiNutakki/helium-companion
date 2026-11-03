# Helium Companion

A floating companion robot project using helium buoyancy, two flapping wings,
a passive tail and sliding internal ballast.

This repository contains the Gazebo simulation and control software used to
develop the physical prototype. It provides a way to explore buoyancy, wing
motion, steering and ballast control before building and tuning the hardware.
It includes a browser control panel, native C++ physics plugin, model generator
and automated tests.

Inspired by [Cuddle-Fish](https://kaikunze.de/papers/pdf/xu2025cuddle.pdf).
Aerodynamic coefficients are currently estimated; simulation results have not
yet been validated against a physical prototype.

![Simulation dashboard](floating-companion/reports/dashboard.png)

## Get started

The current setup targets macOS with Homebrew Gazebo Jetty, Python 3.14, CMake
and a C++ compiler already installed.

```sh
cd floating-companion
./scripts/setup.sh
./start.command --gui
```

Open http://127.0.0.1:8765 for the controls. Omit `--gui` to run the Gazebo
physics server with only the browser dashboard.

See the [project documentation](floating-companion/README.md) for controls,
physics assumptions, limitations and testing instructions. Saved results include
14 passing Gazebo integration checks and 15 passing unit/HTTP tests.

## Prototype experiments

Compare wing settings in a separate simulation and save trajectories:

```sh
cd floating-companion
.venv/bin/python scripts/experiment.py --frequencies .6 1.2 --coefficients 0 1.2 --repeats 2
```

See the [eight-case comparison](floating-companion/reports/experiments/wing-comparison/summary.json)
for an example with repeated settings and zero-thrust controls. Coefficients
remain assumptions that need physical measurement.
