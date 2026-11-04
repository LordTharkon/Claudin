"""Run the shared z-fighting check (../check_zfighting.py) over the Farm models.

    python3 3d-models/Farm/check_zfighting_farm.py           # all farm models
    python3 3d-models/Farm/check_zfighting_farm.py baler     # names containing "baler"
"""
import importlib
import os
import sys

import farm_parts  # noqa: F401  (puts the shared helpers on sys.path)
from check_zfighting import report

BUILDERS = ['build_utility_tractor', 'build_row_crop_tractor', 'build_combine_harvester',
            'build_sprayer', 'build_round_baler', 'build_grain_trailer', 'build_farm_props',
            # expansion pack
            'build_reversible_plough',
            'build_disc_harrow',
            'build_seed_drill',
            'build_mower_conditioner',
            'build_telehandler',
            'build_stock_trailer',
            'build_atv',
            'build_livestock_props',
            ]


def main():
    filt = next((a for a in sys.argv[1:] if not a.startswith('--')), '')
    total = 0
    for mod in BUILDERS:
        if not os.path.exists(os.path.join(os.path.dirname(os.path.abspath(__file__)), mod + '.py')):
            continue
        total += report(importlib.import_module(mod).build, filt, '--all' in sys.argv)
    print('total:', total)
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
