#!/usr/bin/env python3
"""Plot a run of the IAC1 trajectory and force tracking controller."""

from plot_cartesian_iac1_control import TRACKING_DATA_DIR, main


if __name__ == "__main__":
    main(default_directory=TRACKING_DATA_DIR)
