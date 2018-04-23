#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2
from reedsolo import RSCodec
import sys
import os
import gc
import yaml
import sqlite3
import datetime
import random
path = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(path)
from utils import csv_util, post_util


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        param.append([x, y])

def sakuhin_id(points):
    xy_array = [[[0,0],[460,1600]], [[400,1350],[1150,2000]], [[600,720],[1250,1200]],[[1150,1200],[1800,2000]], [[500, 0],[1150, 650]], [[1150, 0],[2500, 500]],
    [[1950, 900],[2500, 2000]], [[1150 ,650],[1950, 1000]]]
    for i in range(0, 8):
        #print (xy_array[i][0][0] ,points[0] ,xy_array[i][1][0] ,xy_array[i][0][1] , points[1] , xy_array[i][1][1])
        if xy_array[i][0][0] < points[0] < xy_array[i][1][0] and xy_array[i][0][1] < points[1] < xy_array[i][1][1]:
            return i
    #print("no sakuhin_id")
    return -1

def main():
    # read input args
    if len(sys.argv) != 3:
        print("usage: " + sys.argv[0] + " outer_size fps")
        return -1
    outer_size = int(sys.argv[1])
    #outer_size = 1000
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

    # db connection
    conn = sqlite3.connect('husen_kansou.db')

    # DBを開く。適合関数・変換関数を有効にする。
    conn = sqlite3.connect('husen_kansou.db',detect_types=sqlite3.PARSE_DECLTYPES|sqlite3.PARSE_COLNAMES)

    # "TIMESTAMP"コンバータ関数 をそのまま ”DATETIME” にも使う
    sqlite3.dbapi2.converters['DATETIME'] = sqlite3.dbapi2.converters['TIMESTAMP']

    c = conn.cursor()


    #first select the desk area
    while 1:
        ret, frame = cap.read()
        if ret:
            break

    print(frame.shape[0])
    print(frame.shape[1])
    window_name = "SELECT DESK"
    frame_for_select = cv2.resize(frame, (int(frame.shape[1] * 0.2), int(frame.shape[0]*0.2)))
    print("Please select left-top and right-down")
    cv2.imshow(window_name, frame_for_select)
    desk_area = [[0,0],[2592, 1944]]
    #cv2.setMouseCallback(window_name,mouse_callback, desk_area)
    #cv2.waitKey(0)
    cv2.destroyWindow(window_name)
    desk_area = np.array(desk_area) * 5

    print(frame.shape[1])
    print(frame.shape[0])
    # postit dictionary
    postit_saved = {}
    time = 0

    while (1):
        # skip or not skip
        if skip_mode and time % fps != 0:
            # grab frame
            ret, frame = cap.read()
            if ret == False:
                break
        else:
            # read frame
            print("progress: " + str(int(time / fps) / 60) + ":" + str((time / fps) % 60))
            
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
            #print len(postit_points)
            for i in range(0, len(postit_points)):
                # postit_image_analyzing = cv2.imread("./postit_tmp_analyzing/" + str(i) + ".jpg")
                postit_image_analyzing = np.load("./tmp_datas/postit_tmp_analyzing/" + str(i) + ".npy")
                #cv2.imshow("analyzing image", postit_image_analyzing)
                bit_array = post_util.readDots(postit_image_analyzing).bit_array
                bit_array_list.append(bit_array)
                print bit_array
                result_num = -1
                try:
                    result_num = rs.decode(bit_array)[0]
                except:
                    result_num = -1
                postit_ids.append(result_num)
                print result_num
                # save postit image
                if result_num != -1:
                    #rint(postit_points[i])
                    #postit_center = postit_points[i][0]
                    #for j in range(0 ,4):
                    #        postit_center += postit_points[i][j]
                    #postit_center /= 4
                    postit_center = np.mean(postit_points[i], axis = 0)
                    print postit_center
                    print sakuhin_id(postit_center)
                    # already exist
                    if postit_saved.has_key(result_num):
                        sid = sakuhin_id(postit_center)
                        if sid >= 0 and postit_saved[result_num]["ranged"] == False:
                            postit_image_for_save = cv2.imread("./tmp_datas/postit_tmp/" + str(i) + ".jpg")
                            c.execute('SELECT * FROM kansou WHERE husen_id=?', (result_num,))
                            rows = c.fetchall()
                            row_len = 0
                            tdate = datetime.datetime.now()
                            tstr = tdate.strftime('%Y_%m_%d_%H_%M_%S_')
                            cv2.imwrite("./datas/kansou_husen/" + str(sid) + "/" +tstr+ str(result_num) + "_" + str(len(rows)) + ".jpg", postit_image_for_save)
                            c.execute("INSERT INTO kansou VALUES (?, ?, ?)", (result_num,sid,datetime.datetime.now()))
                            print "dbinsert"
                            postit_saved[result_num]["ranged"] = True


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

                        if time/fps - postit_saved[result_num]["last_time"] > 600:
                        #if False:
                            print "mouikkai appear"
                            postit_saved[result_num] = {"points": postit_points[i], "points_saved": postit_points[i],
                                                        "first_time": time / fps, "last_time": time / fps, "move": [],
                                                        "rotate": [],"ranged":False}

                            # Insert a row of data
                            sid = sakuhin_id(postit_center)
                            if(sid >= 0 and postit_saved[result_num]["ranged"] == False):
                                postit_image_for_save = cv2.imread("./tmp_datas/postit_tmp/" + str(i) + ".jpg")
                                c.execute('SELECT * FROM kansou WHERE husen_id=?', (result_num,))
                                rows = c.fetchall()
                                row_len = 0
                                tdate = datetime.datetime.now()
                                tstr = tdate.strftime('%Y_%m_%d_%H_%M_%S_')
                                cv2.imwrite("./datas/kansou_husen/" + str(sid) + "/" +tstr+ str(result_num) + "_" + str(len(rows)) + ".jpg", postit_image_for_save)
                                c.execute("INSERT INTO kansou VALUES (?, ?, ?)", (result_num,sid,datetime.datetime.now()))
                                print "dbinsert"
                                postit_saved[result_num]["ranged"] = True
                            # Save (commit) the changes
                            conn.commit()

                        # renew
                        postit_saved[result_num]["points"] = postit_points[i]
                        postit_saved[result_num]["points_saved"] = postit_points[i]
                        postit_saved[result_num]["last_time"] = time / fps
                    # first appear
                    else:
                        print "first appear"
                        postit_saved[result_num] = {"points": postit_points[i], "points_saved": postit_points[i],
                                                    "first_time": time / fps, "last_time": time / fps, "move": [],
                                                    "rotate": [], "ranged":False}
                        postit_image_for_save = cv2.imread("./tmp_datas/postit_tmp/" + str(i) + ".jpg")

                        #for row in c.execute('SELECT * FROM kansou WHERE husen_id=?',(result_num)):
                        #    print(row)
                        #    row_len + 1
                        sid = sakuhin_id(postit_center)
                        if sid >= 0 and postit_saved[result_num]["ranged"] == False:
                            c.execute('SELECT * FROM kansou WHERE husen_id=?', (result_num,))
                            rows = c.fetchall()
                            row_len = 0
                            tdate = datetime.datetime.now()
                            tstr = tdate.strftime('%Y_%m_%d_%H_%M_%S_')
                            cv2.imwrite("./datas/kansou_husen/" + str(sid) + "/" +tstr+ str(result_num) + "_" + str(len(rows)) + ".jpg", postit_image_for_save)
                            print("./datas/kansou_husen/" + str(sid) + "/" +tstr+ str(result_num) + "_" + str(len(rows)) + ".jpg")
                        # Insert a row of data
                            c.execute("INSERT INTO kansou VALUES (?, ?, ?)", (result_num,sid,datetime.datetime.now()))
                            print "dbinsert:" + str(sid)
                            postit_saved[result_num]["ranged"] = True

                        # Save (commit) the changes
                        conn.commit()



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
            cv2.imwrite("last.jpg", frame_for_final)


            #projector draw
            cols = 1366
            rows = 768
            projector_image = np.zeros((rows, cols, 3), np.uint8)
            desk_width = desk_area[1][0] - desk_area[0][0]
            desk_height = desk_area[1][1] - desk_area[0][1]
            postit_relative_points = []
            #print postit_saved
            iii = 0
            for key in postit_saved:
                postit_relative_points.append([])
                '''
                for i in range(0, len(postit_saved[key]["points"])):
                    postit_relative_points[iii].append(postit_saved[key]["points"] - desk_area[0])
                    postit_relative_points[iii][i][0] = int(cols * 1.0 / desk_width * postit_relative_points[i][j][0])
                    postit_relative_points[iii][i][1] = int(rows * 1.0 / desk_height * postit_relative_points[i][j][1])

                #cv2.drawContours(projector_image,(postit_relative_points[i]),0,(0,0,255),2)
                cv2.drawContours(projector_image, [np.array(postit_saved[key]["points"]).astype(np.int)], 0,
                                 (0, 0, 255), 2)
                cv2.putText(projector_image, str(key), (
                np.mean(postit_saved[key]["points"], axis=0).astype(np.int)[0] - 40,
                np.mean(postit_saved[key]["points"], axis=0).astype(np.int)[1]), cv2.FONT_HERSHEY_PLAIN, 5.0,
                            (0, 240, 0), 5)
                #pts = np.array(postit_relative_points[i], np.int32)
                #cv2.polylines(projector_image,[pts],True,(0,255,255))
                #cv2.putText(projector_image,str(postit_ids[i]),(int(postit_relative_points[i][0][0]),int(postit_relative_points[i][0][1])), cv2.FONT_HERSHEY_SIMPLEX , 1,(255,255,255),2)
                iii+=1
                '''
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
