#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from reedsolo import RSCodec
import sys
import os
import gc
import yaml
path = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(path)
from utils import csv_util, post_util


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        param.append([x, y])


def main():
    # read input args
    if len(sys.argv) != 3:
        print "usage: " + sys.argv[0] + " outer_size fps"
        return -1
    outer_size = int(sys.argv[1])
    fps = int(sys.argv[2])
    # delete old postit image
    old_postit_image = os.listdir('./datas/postit_saved/')
    for old_postit in old_postit_image:
        os.remove('./datas/postit_saved/' + old_postit)

    # memory save
    gc.enable()

    # set parameter
    analyse_config = yaml.load(open("./config/analyse_config.yaml", "r"))
    dist_thre = analyse_config["dist_thre"]
    angle_thre = analyse_config["angle_thre"]
    time_thre = analyse_config["time_thre"]
    # video_save = False
    skip_mode = True

    # video reader and writer
    cap = cv2.VideoCapture(0)
    cap.set(3, 2592)
    cap.set(4, 1944)

    # csv_file
    csv_every_second = open("./datas/csv/every_second.csv", "w")
    csv_final = open("./datas/csv/final.csv", "w")

    # reed solomon
    rs = RSCodec(14)


    #first select the desk area
    #while 1:
    #    ret, frame = cap.read()
    #    if ret:
    #        break

    cols = 860
    rows = 400
    projector_image = np.zeros((rows, cols, 3), np.int32)
    projector_image[:,:] = np.array([0, 255, 255], np.int32)

    cv2.imshow("projector", projector_image)

    ret, frame = cap.read()

    print frame.shape[0]
    print frame.shape[1]
    window_name = "SELECT DESK"
    frame_for_select = cv2.resize(frame, (int(frame.shape[1] * 0.2), int(frame.shape[0]*0.2)))
    print "Please select left-top and right-down"
    cv2.imshow(window_name, frame_for_select)
    desk_area = []
    while 1:
        ret, frame = cap.read()
        frame_for_select = cv2.resize(frame, (int(frame.shape[1] * 0.2), int(frame.shape[0]*0.2)))
        cv2.imshow(window_name, frame_for_select)
        cv2.setMouseCallback(window_name,mouse_callback, desk_area)
        cv2.waitKey(1)
        if len(desk_area) == 2:
            break

    cv2.destroyWindow(window_name)
    desk_area = np.array(desk_area) * 5

    print frame.shape[1]
    print frame.shape[0]
    # postit dictionary
    postit_saved = {}
    time = 0

    while (1):
        # skip or not skip
        if skip_mode and time % fps != 0:
            mimi = 0
            # grab frame
            #ret = cap.grab()
            #if ret == False:
            #    break
        else:
            # read frame
            print "progress: " + str(int(time / fps) / 60) + ":" + str((time / fps) % 60)
            ret, frame = cap.read()
            if ret == False:
                break
            # save frame for final
            frame_for_final = np.copy(frame)

        if time % fps == 0:
            # extract only desk area
            frame = frame[desk_area[0][1]:desk_area[1][1], desk_area[0][0]:desk_area[1][0]]

            # get postit
            getPostits = post_util.getPostits(frame, outer_size)
            # postit_image = getPostits.postit_image
            # postit_image_analyzing = getPostits.postit_image_analyzing
            postit_points = getPostits.postit_points

            #add buffer
            for i in range(0, len(postit_points)):
                for j in range(0, len(postit_points[i])):
                    postit_points[i][j] += desk_area[0]

            # read postit's id and save
            postit_ids = []
            bit_array_list = []
            for i in range(0, len(postit_points)):
                # postit_image_analyzing = cv2.imread("./postit_tmp_analyzing/" + str(i) + ".jpg")
                postit_image_analyzing = np.load("./tmp_datas/postit_tmp_analyzing/" + str(i) + ".npy")
                bit_array = post_util.readDots(postit_image_analyzing).bit_array
                bit_array_list.append(bit_array)
                result_num = -1
                try:
                    result_num = rs.decode(bit_array)[0]
                except:
                    result_num = -1
                postit_ids.append(result_num)
                # save postit image
                if result_num != -1:
                    # already exist
                    if postit_saved.has_key(result_num):
                        # judge move and rotate
                        # calc dist
                        dist = np.linalg.norm(
                            np.mean(postit_saved[result_num]["points_saved"], axis=0) - np.mean(postit_points[i],
                                                                                                axis=0))
                        # calc angle(degree)
                        angle_vec_before = np.array(postit_saved[result_num]["points_saved"][0]) - np.array(
                            postit_saved[result_num]["points_saved"][1])
                        angle_vec_after = np.array(postit_points[i][0]) - np.array(postit_points[i][1])
                        vec_cos = np.dot(angle_vec_before, angle_vec_after) / (
                        np.linalg.norm(angle_vec_before) * np.linalg.norm(angle_vec_after))
                        angle = np.arccos(vec_cos) * 180 / np.pi
                        # add information
                        if dist > dist_thre:
                            if len(postit_saved[result_num]["move"]) == 0:
                                postit_saved[result_num]["move"].append(time / fps)
                            elif (time / fps - postit_saved[result_num]["move"][-1]) > 5:
                                postit_saved[result_num]["move"].append(time / fps)
                        elif angle > angle_thre:
                            if len(postit_saved[result_num]["rotate"]) == 0:
                                postit_saved[result_num]["rotate"].append(time / fps)
                            elif (time / fps - postit_saved[result_num]["rotate"][-1]) > 5:
                                postit_saved[result_num]["rotate"].append(time / fps)
                        # renew
                        postit_saved[result_num]["points"] = postit_points[i]
                        postit_saved[result_num]["points_saved"] = postit_points[i]
                        postit_saved[result_num]["last_time"] = time / fps
                    # first appear
                    else:
                        postit_saved[result_num] = {"points": postit_points[i], "points_saved": postit_points[i],
                                                    "first_time": time / fps, "last_time": time / fps, "move": [],
                                                    "rotate": []}
                        postit_image_for_save = cv2.imread("./tmp_datas/postit_tmp/" + str(i) + ".jpg")
                        cv2.imwrite("./datas/postit_saved/" + str(result_num) + ".jpg", postit_image_for_save)


            # delete old postit(long time no see)
            for id, val in postit_saved.items():
                if (time / fps - val["last_time"]) > time_thre:
                    postit_saved[id]["points"] = [[-5, 0], [0, 0], [0, -5], [-5, -5]]

            # write csv
            csv_util.write_every_second(postit_saved, csv_every_second)
            # memory save
            del getPostits
            gc.collect()

            #renew viewing
            for key in postit_saved:
                cv2.drawContours(frame_for_final, [np.array(postit_saved[key]["points"]).astype(np.int)], 0,
                                 (0, 0, 255), 2)
                cv2.putText(frame_for_final, str(key), (
                np.mean(postit_saved[key]["points"], axis=0).astype(np.int)[0] - 40,
                np.mean(postit_saved[key]["points"], axis=0).astype(np.int)[1]), cv2.FONT_HERSHEY_PLAIN, 5.0,
                            (0, 240, 0), 5)
            show_img_final = cv2.resize(frame_for_final, (int(frame_for_final.shape[1] * 0.3), int(frame_for_final.shape[0]*0.3)))
            cv2.imshow("show2", show_img_final)

            #projector draw
            projector_image = np.zeros((rows, cols, 3), np.uint8)
            desk_width = desk_area[1][0] - desk_area[0][0]
            desk_height = desk_area[1][1] - desk_area[0][1]

            for key in postit_saved:
                postit_relative_points = []
                for i in range(0, len(postit_saved[key]["points"])):
                    scale = 0
                    relative_point = postit_saved[key]["points"][i] - desk_area[0]
                    relative_point[0] = int(cols * scale * relative_point[0] / desk_width)
                    relative_point[1] = int(rows * scale * relative_point[1]/ desk_height)
                    postit_relative_points.append(relative_point)
                for i in range(0, len(postit_relative_points)):
                    postit_relative_points[i][0] += int(0.05 * (postit_relative_points[i][0] - np.mean(postit_relative_points, axis=0)[0]))
                    postit_relative_points[i][1] += int(0.05 * (postit_relative_points[i][1] - np.mean(postit_relative_points, axis=0)[1]))
                pts = np.array(postit_relative_points, np.int32)
                cv2.polylines(projector_image,[pts],True,(0,255,255))
                cv2.putText(projector_image, str(key), (
                np.mean(postit_relative_points, axis=0).astype(np.int)[0] - 30,
                np.mean(postit_relative_points, axis=0).astype(np.int)[1] + 30), cv2.FONT_HERSHEY_PLAIN, 3.0,
                            (0, 240, 0), 5)
            cv2.imshow("projector", projector_image)




        # key waiting
        key = cv2.waitKey(1)
        if key == 27:
            break

        # add time
        time += 1

    # final save
    csv_util.write_final(postit_saved, csv_final)


if __name__ == "__main__":
    main()
