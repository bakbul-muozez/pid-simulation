#!/usr/bin/env python3
"""
Setup script for PID Simulation Package
"""

from setuptools import setup, find_packages
import os
import re

# Read the contents of README file
def read_readme():
    with open("README.md", "r", encoding="utf-8") as fh:
        return fh.read()

# Read version from __init__.py
def get_version():
    init_file = os.path.join("src", "__init__.py")
    with open(init_file, "r", encoding="utf-8") as f:
        content = f.read()
        version_match = re.search(r"__version__ = ['\"]([^'\"]*)['\"]", content)
        if version_match:
            return version_match.group(1)
    return "1.0.0"

# Read requirements from requirements.txt
def get_requirements():
    requirements = []
    try:
        with open("requirements.txt", "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    # Remove comments and version constraints for basic dependencies
                    package = line.split(">=")[0].split("==")[0].split("#")[0].strip()
                    if package and not any(dev in package.lower() for dev in ['pytest', 'black', 'flake8', 'isort', 'mypy', 'sphinx', 'bandit', 'safety', 'build', 'twine']):
                        requirements.append(line)
    except FileNotFoundError:
        pass
    return requirements

setup(
    name="pid-simulation",
    version=get_version(),
    author="PID Simulation Team",
    author_email="contact@pid-simulation.dev",
    description="A comprehensive Python package for simulating and analyzing PID controllers",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    url="https://github.com/bakbul-muozez/pid-simulation",
    project_urls={
        "Bug Tracker": "https://github.com/bakbul-muozez/pid-simulation/issues",
        "Documentation": "https://pid-simulation.readthedocs.io/",
        "Source Code": "https://github.com/bakbul-muozez/pid-simulation",
        "Web Interface": "https://bakbul-muozez.github.io/pid-simulation/webapp/",
    },
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Physics",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.8",
    install_requires=get_requirements(),
    extras_require={
        "dev": [
            "pytest>=6.2.0",
            "pytest-cov>=2.12.0",
            "black>=21.0.0",
            "flake8>=3.9.0",
            "isort>=5.9.0",
            "mypy>=0.910",
        ],
        "docs": [
            "sphinx>=4.0.0",
            "sphinx-rtd-theme>=0.5.0",
        ],
        "web": [
            "plotly>=5.0.0",
            "dash>=2.0.0",
        ],
        "all": [
            "pytest>=6.2.0",
            "pytest-cov>=2.12.0",
            "black>=21.0.0",
            "flake8>=3.9.0",
            "isort>=5.9.0",
            "mypy>=0.910",
            "sphinx>=4.0.0",
            "sphinx-rtd-theme>=0.5.0",
            "plotly>=5.0.0",
            "dash>=2.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "pid-sim=main:main",
            "pid-simulation=main:main",
        ],
    },
    include_package_data=True,
    package_data={
        "": ["*.txt", "*.md", "*.yml", "*.yaml"],
        "public": ["*.html", "*.css", "*.js"],
    },
    zip_safe=False,
    keywords=[
        "pid", "control", "simulation", "engineering", "automation",
        "control-systems", "pid-controller", "tuning", "ziegler-nichols",
        "process-control", "feedback-control", "motor-control"
    ],
)
