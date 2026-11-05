"""Z-fighting scan for the Mining pack (reuses the shared checker in ../check_zfighting.py).

    python3 3d-models/Mining/check_zfighting_mining.py            # every Mining model
    python3 3d-models/Mining/check_zfighting_mining.py dozer      # names containing "dozer"
"""
import importlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(1, os.path.dirname(HERE))
import mining_parts  # noqa: E402,F401  (installs the cylinder-hose builder)
from check_zfighting import report  # noqa: E402

BUILDERS = ['build_large_mining_dozer', 'build_large_wheel_loader', 'build_water_cart',
            'build_mining_motor_grader', 'build_mine_light_vehicle', 'build_mine_site_props',
            'build_underground_loader', 'build_underground_haul_truck',
            'build_development_jumbo', 'build_service_truck',
            'build_tyre_handler']


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    show_all = '--all' in sys.argv
    filt = args[0] if args else ''
    total = 0
    for mod in BUILDERS:
        if not os.path.exists(os.path.join(HERE, mod + '.py')):
            continue
        total += report(importlib.import_module(mod).build, filt, show_all)
    print('total:', total)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
