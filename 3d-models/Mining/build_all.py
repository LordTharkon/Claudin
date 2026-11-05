"""Regenerate every Mining model: python3 3d-models/Mining/build_all.py"""
import os
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))
for script in ('build_large_mining_dozer.py', 'build_large_wheel_loader.py', 'build_water_cart.py',
               'build_mining_motor_grader.py', 'build_mine_light_vehicle.py', 'build_mine_site_props.py',
               'build_underground_loader.py', 'build_underground_haul_truck.py',
               'build_development_jumbo.py', 'build_service_truck.py'):
    subprocess.run([sys.executable, os.path.join(here, script)], check=True, cwd=here)
