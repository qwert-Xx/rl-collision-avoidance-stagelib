#!/bin/bash
c++ -O0 -Wall -shared -std=c++11 -fPIC $(python3 -m pybind11 --includes) -Iinclude stgCPPToPy.cpp -o stgCPPToPy$(python3-config --extension-suffix) -lstage
ln -s ../StageLibCpp/stgCPPToPy$(python3-config --extension-suffix) ../RLPy/