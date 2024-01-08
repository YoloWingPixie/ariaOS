 # This file contains the PID gains for the launch vehicle. The gains are
# organized by axis and are stored in a dictionary. The keys for each axis
# are 'P', 'I', and 'D' for the proportional, integral, and derivative gains,
# respectively.

PID_SETTINGS = {
    'THROTTLE': {
        'P': 0.5,
        'I': 0.1,
        'D': 0.1
    },
    'PITCH': {
        'P': 2.31,
        'I': 2.60,
        'D': 0.26
    },
    'YAW': {
        'P': 5.15,
        'I': 0,
        'D': 0
    },
    'ROLL': {
        'P': 41.95,
        'I': 47.19,
        'D': 4.2
    },
    'AUTOTUNE_STATE': False
}