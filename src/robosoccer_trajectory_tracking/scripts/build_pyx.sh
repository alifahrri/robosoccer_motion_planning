#!/usr/bin/env bash
python setup.py build_ext --inplace
echo '' >> scripts/__init__.py