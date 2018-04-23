import gc
import os
import sys
import time as timer

import cv2
import numpy as np
from reedsolo import RSCodec
import yaml

path = os.path.join(os.path.dirname(__file__), '../')
sys.path.append(path)
from utils import csv_util, post_util

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        param.append([x,y])

def main():
    #read input args
    if len(sys.argv) != 3:
        print "usage: " + sys.argv[0] + " outer_size fps"
        return -1
    outer_size = int(sys.argv[1])
    fps = int(sys.argv[2])
    #delete old postit image
    old_postit_image = os.listdir('./datas/postit_saved/')
    for old_postit in old_postit_image:
        os.remove('./datas/postit_saved/' + old_postit)

    #memory save
    gc.enable()

    #set parameter
    analyse_config = yaml.load(open("./config/analyse_config.yaml", "r"))
    dist_thre = analyse_config["dist_thre"]
    angle_thre = analyse_config["angle_thre"]
    time_thre = analyse_config["time_thre"]
    video_save = False
    skip_mode = True

    #read movie file list
    movie_id = 0
    movie_files = os.listdir('./datas/movie/')
    if len(movie_files) == 0:
        print "no movie"
        return -1
    else:
        print "movie files:"
        for movie_file_each in movie_files:
            print movie_file_each

    #video reader and writer
    cap = cv2.VideoCapture('./datas/movie/' + movie_files[0])
    if video_save:
        writer = cv2.VideoWriter("./datas/movie/OUT.avi", cv2.cv.CV_FOURCC('M','J','P','G'), fps, (3840,2160))
    #csv_file
    csv_every_second = open("./datas/csv/every_second.csv", "w")
    csv_final = open("./datas/csv/final.csv", "w")

    #reed solomon
    rs = RSCodec(14)

    #first select the desk area
    ret, frame = cap.read()
    window_name = "SELECT DESK"
    frame_for_select = cv2.resize(frame, (int(frame.shape[1] * 0.2), int(frame.shape[0]*0.2)))
    print "Please select left-top and right-down"
    cv2.imshow(window_name, frame_for_select)
    desk_area = []
    cv2.setMouseCallback(window_name,mouse_callback, desk_area)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)
    desk_area = np.array(desk_area) * 5

    # set desk by myself
    # desk_area = np.array([[1615, 890], [2110, 1285]])

    # select the mask area
    window_name = "SELECT MASK"
    print "Please select left-top and right-down"
    cv2.imshow(window_name, frame[desk_area[0][1]:desk_area[1][1],desk_area[0][0]:desk_area[1][0]])
    mask_area = []
    cv2.setMouseCallback(window_name, mouse_callback, mask_area)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)

    #postit dictionary
    postit_saved = {}

    # set the mask area
    mask_area_file = open("./datas/csv/mask_area_paired.csv", "r")
    mask_num_lines = mask_area_file.readlines()
    mask_num_list = [mask_num_lines[i].split(",")[:-1] for i in range(len(mask_num_lines))]
    mask_num_list.insert(0, []) # for no masking
    # convert to int
    for i in range(len(mask_num_list)):
        for j in range(len(mask_num_list[i])):
            mask_num_list[i][j] = int(mask_num_list[i][j])
    #result file
    experiment_result = open("./datas/csv/experiment_result.csv", "w")
    # set resolution
    resolution = 0.00

    # mask_num_list = mask_num_list[121:123]
    previous_mask_len = 0

    for mask_num in mask_num_list:
        print "pattern:" + str(mask_num)
        before = timer.time()
        time = 0
        success_count = 0
        postit_saved = {}
        cap = cv2.VideoCapture('./datas/movie/' + movie_files[movie_id])
        while(1):
           #read frame
           # print "progress: " + str(time)
           ret, frame = cap.read()
           if ret == False:
               movie_id += 1
               if movie_id < len(movie_files):
                    cap = cv2.VideoCapture('./datas/movie/' + movie_files[movie_id])
                    ret, frame = cap.read()
               else:
                    break
           #save frame for final
           frame_for_final = np.copy(frame)

           #extract only desk area
           frame = frame[desk_area[0][1]:desk_area[1][1],desk_area[0][0]:desk_area[1][0]]

           #make mask
           for mask_num_each in mask_num:
               cv2.rectangle(frame, (mask_area[mask_num_each*2][0], mask_area[mask_num_each*2][1]), (mask_area[mask_num_each*2+1][0], mask_area[mask_num_each*2+1][1]), (200,200,200), -1)

           # change resolution
           frame = cv2.resize(frame,(int(frame.shape[1] * (1 - resolution)), int(frame.shape[0] * (1 - resolution))))
           outer_size_resized = outer_size * ((1 - resolution) ** 2)

           #get postit
           getPostits = post_util.getPostits(frame, outer_size_resized)
           #postit_image_analyzing = getPostits.postit_image_analyzing
           postit_points = getPostits.postit_points
           #add buffer
           for i in range(0, len(postit_points)):
               for j in range(0, len(postit_points[i])):
                   postit_points[i][j] += desk_area[0]

           #read postit's id and save
           postit_ids = []
           bit_array_list = []
           for i in range(0,len(postit_points)):
               #postit_image_analyzing = cv2.imread("./postit_tmp_analyzing/" + str(i) + ".jpg")
               postit_image_analyzing = np.load("./tmp_datas/postit_tmp_analyzing/" + str(i) + ".npy")
               bit_array = post_util.readDots(postit_image_analyzing).bit_array
               # print bit_array
               # bit_array_answer = []
               # for num in rs.encode([62]):
               #     bit_array_answer.append(num)
               # print bit_array_answer
               bit_array_list.append(bit_array)
               result_num = -1
               try:
                   result_num = rs.decode(bit_array)[0]
               except:
                   result_num = -1
               if result_num != -1 and result_num != 0:
                   # print "success:" + str(result_num)
                   success_count += 1
               postit_ids.append(result_num)
               #save postit image
               if result_num != -1:
                   #already exist
                   if postit_saved.has_key(result_num):
                        #judge move and rotate
                        #calc dist
                        dist = np.linalg.norm(np.mean(postit_saved[result_num]["points_saved"], axis = 0) - np.mean(postit_points[i], axis = 0))
                        #calc angle(degree)
                        angle_vec_before = np.array(postit_saved[result_num]["points_saved"][0]) - np.array(postit_saved[result_num]["points_saved"][1])
                        angle_vec_after = np.array(postit_points[i][0]) - np.array(postit_points[i][1])
                        vec_cos = np.dot(angle_vec_before, angle_vec_after) / (np.linalg.norm(angle_vec_before) * np.linalg.norm(angle_vec_after))
                        angle = np.arccos(vec_cos) * 180 / np.pi
                        #add information
                        if dist > dist_thre:
                            if len(postit_saved[result_num]["move"]) == 0:
                                postit_saved[result_num]["move"].append(time / fps)
                            elif (time/fps - postit_saved[result_num]["move"][-1]) > 5:
                                postit_saved[result_num]["move"].append(time / fps)
                        elif angle > angle_thre:
                            if len(postit_saved[result_num]["rotate"]) == 0:
                                postit_saved[result_num]["rotate"].append(time / fps)
                            elif (time/fps - postit_saved[result_num]["rotate"][-1]) > 5:
                                postit_saved[result_num]["rotate"].append(time / fps)
                        #renew
                        postit_saved[result_num]["points"] = postit_points[i]
                        postit_saved[result_num]["points_saved"] = postit_points[i]
                        postit_saved[result_num]["last_time"] = time / fps
                   #first appear
                   else:
                       postit_saved[result_num] = {"points": postit_points[i], "points_saved":postit_points[i], "first_time": time/fps, "last_time": time/fps, "move":[], "rotate":[]}
                       postit_image_for_save = cv2.imread("./tmp_datas/postit_tmp/" + str(i) + ".jpg")
                       cv2.imwrite("./datas/postit_saved/" + str(result_num) + ".jpg", postit_image_for_save)



           #delete old postit(long time no see)
           for id, val in postit_saved.items():
              if (time/fps - val["last_time"]) > time_thre:
                  postit_saved[id]["points"] = [[-5,0],[0, 0],[0,-5],[-5, -5]]

           #write csv
           csv_util.write_every_second(postit_saved, csv_every_second)
           #memory save
           del getPostits
           gc.collect()

           #key waiting
           key = cv2.waitKey(1)
           if key == 27:
               break


           #when drawing dict info
           for key in postit_saved:
                cv2.drawContours(frame_for_final,[np.array(postit_saved[key]["points"]).astype(np.int)], 0,(0,0,220),2)
                cv2.putText(frame_for_final, str(key), (np.mean(postit_saved[key]["points"], axis = 0).astype(np.int)[0] - 40, np.mean(postit_saved[key]["points"], axis = 0).astype(np.int)[1]), cv2.FONT_HERSHEY_PLAIN, 5.0, (0,140,0), 5)
           show_img_final = cv2.resize(frame_for_final, (int(frame_for_final.shape[1] * 0.3), int(frame_for_final.shape[0]*0.3)))
           cv2.imshow("show2", show_img_final)

           #add time
           time += 1

           if time > 23:
               break

        #final save
        csv_util.write_final(postit_saved, csv_final)

        after = timer.time()
        #print success rate
        if previous_mask_len != len(mask_num):
            experiment_result.write("\n")
        experiment_result.write(str(float(success_count) / time) + ",")
        previous_mask_len = len(mask_num)
        print "-----success result------"
        print "success count: " + str(success_count)
        print "time:" + str(time)
        print "success rate: " + str(float(success_count) / time)
        print "consumed time:" + str(after - before)
if __name__ == "__main__":
    main()