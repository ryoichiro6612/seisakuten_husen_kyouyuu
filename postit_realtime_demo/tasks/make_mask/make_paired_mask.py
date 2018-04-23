#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

if __name__ == '__main__':
    file_out = open("mask_area_paired.csv", "w")
    original = [i for i in range(0, 23)] + [i for i in range(0, 23)]

    # for one block
    for i in range(1, 12):
        for j in range(23 - i + 1):
            out_array = original[j:j+i]
            for out in out_array:
                file_out.write(str(out) + ",")
            file_out.write("\n")

    # for two block
    for i in range(2, 12):
        for first in range(1, i / 2 + 1):
            second = i - first
            if first != second:
                for j in range(23 - first + 1):
                    for k in range(j + first + 1, j + 22 - second + 1):
                        out_array = original[j:j+first] + original[k:k+second]
                        for out in out_array:
                            file_out.write(str(out) + ",")
                        file_out.write("\n")
            else:
                for j in range(23 - first + 1):
                    for k in range(j + first + 1, min(j + 22 - second + 1, 23)):
                        out_array = original[j:j+first] + original[k:k+second]
                        for out in out_array:
                            file_out.write(str(out) + ",")
                        file_out.write("\n")