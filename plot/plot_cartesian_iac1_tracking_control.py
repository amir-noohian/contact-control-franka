#!/usr/bin/env python3
"""Plot IAC1E2 tracking, direct force feedback, and effective-angle estimation."""

from plot_cartesian_iac1_control import TRACKING_DATA_DIR, main


if __name__ == "__main__":
    main(default_directory=TRACKING_DATA_DIR, description=__doc__)
