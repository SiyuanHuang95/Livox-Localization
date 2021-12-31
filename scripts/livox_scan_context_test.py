#!/usr/bin/env python2

import numpy as np

from scan_context_manager import ScanContextManager
import os

if __name__ == "__main__":
    date_path = "/root/data_pc/route_1210/around_building/"
    Livox_SC = ScanContextManager(file_path=date_path)

    pcd_path = os.path.join(date_path, "odo_save")
    sc_path = os.path.join(date_path, "scancontext")
    rk_path = os.path.join(date_path, "ringkey")
    npy_path = os.path.join(date_path, "pcd_npy")

    make_sc = False
    if make_sc:
        # Test the ScanContext Maker
        Livox_SC.livox_load_pc_make_sc(pcd_path)

    else:
        # Test the ScanContext Load and Localization
        Livox_SC.livox_load_sc_rk(sc_path, rk_path)
        for i in range(0, 300, 50):
            file_name = "pc_" + str(i) + ".npy"
            print("load test pc: ", file_name)
            test_pc = os.path.join(npy_path, file_name)
            test_trans = Livox_SC.initialization(np.load(test_pc))
            print(test_trans)
