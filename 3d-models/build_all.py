"""Regenerate every model in this folder: python3 build_all.py"""
import os
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))
for script in ('build_crawler_dozer.py', 'build_wheel_dozer.py', 'build_backhoe_loader.py',
               'build_mini_excavator.py', 'build_compact_track_loader.py',
               'build_wheel_tractor_scraper.py', 'build_articulated_dump_truck.py',
               'build_pickup_trucks.py', 'build_site_cabins.py', 'build_site_equipment.py',
               'build_site_materials.py', 'build_site_structures.py'):
    subprocess.run([sys.executable, os.path.join(here, script)], check=True, cwd=here)
