#!/usr/bin/env python
# -*- coding: utf-8 -*-

def one_line_check(mask_num, one_line_pattern, location_dots):
    alive_dots = location_dots[:]
    for num in mask_num:
        try:
            alive_dots.remove(num)
        except:
            continue
    for pattern in one_line_pattern:
        match_num = 0
        for num in alive_dots:
            if num in pattern:
                match_num += 1
        if match_num > 2:
            return True
    return False


if __name__ == '__main__':
    mask_area_file = open("../csv/mask_area_paired.csv", "r")
    mask_num_lines = mask_area_file.readlines()
    mask_num_list = [mask_num_lines[i].split(",")[:-1] for i in range(len(mask_num_lines))]
    mask_num_list.insert(0, [])  # for no masking
    # convert to int
    for i in range(len(mask_num_list)):
        for j in range(len(mask_num_list[i])):
            mask_num_list[i][j] = int(mask_num_list[i][j])
    # result file
    theory_result = open("./theory_result.csv", "w")

    location_dots = [0, 3, 6, 9, 12, 14, 17, 20]
    information_dots = [1, 2, 4, 5, 7, 8, 10, 11, 13, 15, 16, 18, 19, 21, 22]
    one_line_pattern = [[0, 3, 6], [12, 14, 17]]

    for mask_num in mask_num_list:
        location_hidden = 0
        information_hidden = 0
        for num in mask_num:
            if num in location_dots:
                location_hidden += 1
            elif num in information_dots:
                information_hidden += 1
        if location_hidden > 4:
            theory_result.write("0\n")
        elif information_hidden > 7:
            theory_result.write("0\n")
        elif location_hidden == 4 and one_line_check(mask_num, one_line_pattern, location_dots):
            theory_result.write("0\n")
        else:
            theory_result.write("1\n")
