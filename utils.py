"""Shared methods.
"""
import numpy as np

MILLION = float(10**6)

def rad_to_deg(rad):
    return np.array(rad) *180./np.pi

def deg_to_rad(deg):
    return np.array(deg) *np.pi/180.


