"""Regenerate every model in this folder: python3 build_all.py"""
import os
import subprocess
import sys

here = os.path.dirname(os.path.abspath(__file__))
for script in ('build_crawler_dozer.py', 'build_wheel_dozer.py'):
    subprocess.run([sys.executable, os.path.join(here, script)], check=True, cwd=here)
