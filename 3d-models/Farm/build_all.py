"""Regenerate every Farm model: python3 3d-models/Farm/build_all.py

Each build script writes Farm/<model>/<model>.glb and checks its triangle budget
(60k per machine, 20k for the props file; see README for the expansion budgets).
"""
import os
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))
for script in ('build_utility_tractor.py', 'build_row_crop_tractor.py', 'build_combine_harvester.py',
               'build_sprayer.py', 'build_round_baler.py', 'build_grain_trailer.py', 'build_farm_props.py',
               # expansion pack
               'build_reversible_plough.py',
               'build_disc_harrow.py',
               'build_seed_drill.py',
               ):
    subprocess.run([sys.executable, os.path.join(here, script)], check=True, cwd=here)
