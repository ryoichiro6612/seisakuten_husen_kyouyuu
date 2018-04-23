#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

if __name__ == "__main__":
    src_point = np.float32([[0,0], [1, 0.], [0.5, 0], [0.5, 1]])
    dst_point = np.float32([[0,0], [2, 0], [1, 0], [1, 2]])

    M, mask = cv2.findHomography(np.array(src_point), np.array(dst_point), cv2.RANSAC, 3)

    src_point = np.float32([[0, 0], [1, 0], [1, 1], [0.5, 0], [0, 1]])

    src_point_3dim = []
    for src_point_each in src_point:
        src_point_3dim.append([src_point_each[0], src_point_each[1], 1])
    src_point_3dim = np.float32(src_point_3dim)

    for src_point_each in src_point_3dim:
        point_after = np.matrix(M) * np.matrix(src_point_each).T
        point_after /= point_after[2]
        print point_after[0:2]