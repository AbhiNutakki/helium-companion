#!/usr/bin/env python3
"""Compare flapping settings in an owned Gazebo session; save metrics and traces."""
import argparse, csv, itertools, json, math, statistics, sys, time
from pathlib import Path
from session import ROOT, simulation
from provenance import source_info
sys.path.insert(0, str(ROOT/'tests'))
from integration import Lab


def run_case(lab, hz, amplitude, coefficient, seconds):
    lab.reset()
    lab.cmd('configure', thrust_coefficient=coefficient)
    lab.cmd('wings', left_hz=hz, right_hz=hz,
            left_amplitude=amplitude, right_amplitude=amplitude)
    rows = lab.sample(seconds)
    first, last = rows[0], rows[-1]
    elapsed = last['time']-first['time']
    return {
        'frequency_hz': hz, 'half_amplitude_deg': amplitude,
        'assumed_thrust_coefficient': coefficient, 'elapsed_sim_s': elapsed,
        'forward_travel_m': last['position'][0]-first['position'][0],
        'altitude_change_m': last['position'][2]-first['position'][2],
        'mean_speed_mps': statistics.mean(math.hypot(*r['velocity']) for r in rows),
        'mean_wing_force_N': statistics.mean(sum(r['wing_forces'][:2]) for r in rows),
        'obstacle_stop_observed': any(r['status']=='gliding_obstacle_stop' for r in rows),
    }, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frequencies', type=float, nargs='+', default=[.6, 1., 1.4])
    parser.add_argument('--coefficients', type=float, nargs='+', default=[.6, 1.2, 1.8])
    parser.add_argument('--amplitude', type=float, default=40)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--factor', type=float, default=6)
    parser.add_argument('--output', type=Path, default=ROOT/'reports'/'experiments'/time.strftime('%Y%m%d-%H%M%S'))
    args = parser.parse_args()
    if any(not math.isfinite(x) or not 0 <= x <= 1.5 for x in args.frequencies):
        parser.error('frequencies must be between 0 and 1.5 Hz')
    if any(0 < x < .3 for x in args.frequencies):
        parser.error('active frequencies must be at least 0.3 Hz')
    if any(not math.isfinite(x) or not 0 <= x <= 3 for x in args.coefficients):
        parser.error('coefficients must be between 0 and 3')
    if not math.isfinite(args.amplitude) or not 0 <= args.amplitude <= 50:
        parser.error('amplitude must be between 0 and 50 degrees')
    if not math.isfinite(args.seconds) or args.seconds <= 0 or args.repeats < 1:
        parser.error('seconds and repeats must be positive')
    if not math.isfinite(args.factor) or args.factor <= 0:
        parser.error('factor must be positive and finite')
    # Refuse an existing directory to preserve earlier measurements.
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'model_revision': 'helium_flapping_v2', 'complete': False, 'source': source_info(),
              'note': 'Simulation sensitivity study only; coefficients are assumptions, not hardware measurements.',
              'configuration': {k: v for k, v in vars(args).items() if k != 'output'}, 'cases': []}
    destination = args.output/'summary.json'
    try:
        with simulation(args.factor) as url:
            lab = Lab(url)
            for repeat, hz, coefficient in itertools.product(range(1, args.repeats+1), args.frequencies, args.coefficients):
                metrics, rows = run_case(lab, hz, args.amplitude, coefficient, args.seconds)
                number = len(report['cases'])+1
                trace = f'case-{number:03d}.csv'
                with (args.output/trace).open('w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(['sim_time_s','x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','left_force_N','right_force_N','status'])
                    for row in rows:
                        writer.writerow([row['time'], *row['position'], *row['velocity'], *row['wing_forces'][:2], row['status']])
                report['cases'].append(dict(metrics, repeat=repeat, trace=trace))
                destination.write_text(json.dumps(report, indent=2)+'\n')
                print(f"Case {number}: {hz:g} Hz, Ct={coefficient:g}, travel={metrics['forward_travel_m']:.3f} m", flush=True)
            lab.reset()
            report['complete'] = True
    finally:
        destination.write_text(json.dumps(report, indent=2)+'\n')
    print(destination)

if __name__ == '__main__':
    main()
