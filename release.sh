#!/bin/bash
python setup.py bdist
python setup.py bdist_wheel
twine check dist/*
