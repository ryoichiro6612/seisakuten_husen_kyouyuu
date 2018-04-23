import cv2
import numpy as np
import gc
import os
import yaml

#set parameters
#read yaml file
postit_config = yaml.load(open("./config/postit_config.yaml", "r"))
read_config = yaml.load(open("./config/read_config.yaml", "r"))

#expand ratio
expand_ratio = read_config["expand_ratio"]

#postit parameters
postit_width = postit_config["postit_width"] * expand_ratio
postit_height = postit_config["postit_height"] * expand_ratio

#common parameters for inner
space_x = postit_config["space_x"] * expand_ratio
rect_len = postit_config["rect_len"] * expand_ratio
x_buffer = postit_config["x_buffer"] * expand_ratio #from rectangle's edge
y_buffer = postit_config["y_buffer"] * expand_ratio #from rectangle's edge

#bit parameters for outer
bit_num = postit_config["bit_num"]
bit_width = x_buffer*2 + space_x * (bit_num - 1) + rect_len
bit_height = postit_config["bit_height"] * expand_ratio

#location parameters for outer
location_dot_num = postit_config["location_dot_num"]
horizon_x_buffer = postit_config["horizon_x_buffer"] * expand_ratio #from postit's edge
horizon_y_buffer = postit_config["horizon_y_buffer"] * expand_ratio #from postit's edge
location_width = x_buffer*2 + space_x * (location_dot_num - 1) + rect_len
location_height = postit_config["location_height"] * expand_ratio
horizon_space = postit_width/2 - horizon_x_buffer - location_width/2

#common parameters for outer
line_width = postit_config["line_width"] * expand_ratio
rect_rect_space_horizon = (horizon_space - location_width - bit_width*2) / 3
rect_rect_space_vertical = (postit_height/2 - horizon_y_buffer - location_height - location_width/2 - bit_width*2) / 3

#other parameters
error_thresh = read_config["error_thresh"]
dot_read_thre = read_config["dot_read_thre"]
dot_read_area = read_config["dot_read_area"] * expand_ratio
point_buffer = read_config["point_buffer"] * expand_ratio #used when searching in larger area and extract larger area of location point area
larger_buffer = read_config["larger_buffer"] * expand_ratio #used for setting larger area in whole postit area for analyzing
search_buffer = read_config["search_buffer"] * expand_ratio #area of searching information bit rectangle
outer_lower = read_config["outer_lower"]
outer_upper = read_config["outer_upper"]

class getPostits:
    postit_image = []
    postit_points = []
    postit_image_analyzing = []
    recognized_location_rectangle = 0
    def __init__(self, frame, outer_size):
        #initialize image
        self.postit_image = []
        self.postit_points = []
        self.postit_image_analyzing = []
        self.recognized_location_rectangle = 0

        #memory save
        gc.enable()

        #convert to gray to binary
        frame_original = np.copy(frame)
        grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        binImage = cv2.adaptiveThreshold(grayImage,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,25,5)
        contours, hierarchy = cv2.findContours(binImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        location_xy = [[] for i in range(0,8)]

        for count in range(0, len(contours)):
            #minAreaRect
            rect = cv2.minAreaRect(contours[count])
            box = cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            area = cv2.contourArea(box)


            #size check
            if area > outer_size * outer_lower and area < outer_size * outer_upper:
               #find child
               idx = hierarchy[0][count][2]
               if idx != -1:
                   #find child's child
                   #idx2 = hierarchy[0][idx][2]
                   if idx2  != -1:
                        cv2.drawContours(frame,[box],0,(0,0,255),2)
                        cv2.putText(frame, str(area), (box[0][0], box[0][1]), cv2.FONT_HERSHEY_PLAIN, 5.0, (0,128,0), 5)
                        #width-height ratio check
                        wh_ratio = np.linalg.norm(box[0] - box[1]) / np.linalg.norm(box[1] - box[2])
                        if wh_ratio < 1: wh_ratio = wh_ratio ** (-1)
                        if wh_ratio > 2.5: continue

                        #print area
                        #extract larger area
                        max_point = np.max(box, axis = 0)
                        min_point = np.min(box, axis = 0)
                        point_buffer_for_larger = int((point_buffer / expand_ratio) * ((float(outer_size) / 220) ** 0.5))
                        min_y = max(0, min_point[1] - point_buffer_for_larger)
                        max_y = min(frame_original.shape[0], max_point[1]+point_buffer_for_larger)
                        min_x = max(0,min_point[0]-point_buffer_for_larger)
                        max_x = min(frame_original.shape[1], max_point[0] + point_buffer_for_larger)
                        larger_area = frame_original[min_y:max_y, min_x:max_x]
                        larger_area = cv2.cvtColor(larger_area, cv2.COLOR_BGR2GRAY)
                        ret, larger_area = cv2.threshold(larger_area, 0.0, 255.0, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
                        #larger_area_original = np.copy(larger_area)

                        #find contours in larger area
                        flag_changed = False
                        contours_in_larger, hierarchy_in_larger = cv2.findContours(larger_area, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
                        for count_in_larger in range(0, len(contours_in_larger)):
                            rect_in_larger = cv2.minAreaRect(contours_in_larger[count_in_larger])
                            box_in_larger = cv2.cv.BoxPoints(rect_in_larger)
                            box_in_larger = np.int0(box_in_larger)
                            area_in_larger = cv2.contourArea(box_in_larger)
                            if area_in_larger > outer_size * outer_lower and area_in_larger < outer_size * outer_upper:
                                idx = hierarchy_in_larger[0][count_in_larger][2]
                                if idx != -1:
                                    #cv2.drawContours(larger_area_original,[box_in_larger],0, 255, 2)
                                    for i in range(0,4):
                                        box[i] =np.int0([min_point[0]-point_buffer_for_larger ,min_point[1] - point_buffer_for_larger])  + box_in_larger[i]
                                    flag_changed = True

                        #find candidate of left-top
                        candidate = []
                        for i in range(0,4):
                            if np.linalg.norm(box[i] - box[(i+1) % 4]) > np.linalg.norm(box[(i+1) % 4] - box[(i+2) % 4]):
                                candidate.append(i)
                        #read each location's id
                        for id in candidate:
                            #homography
                            before_point = np.float32([box[(i+id) % 4] for i in range(0, 4)])
                            after_point = np.float32([[0+point_buffer,0+point_buffer], [location_width+point_buffer, 0+point_buffer], [location_width+point_buffer, location_height+point_buffer], [0+point_buffer,location_height+point_buffer]])
                            M = cv2.getPerspectiveTransform(before_point, after_point)
                            dst = cv2.warpPerspective(frame_original, M, (location_width+point_buffer*2, location_height+ point_buffer*2))

                            #convert from color to binary
                            dst = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

                            ret, dst = cv2.threshold(dst, 0.0, 255.0, cv2.THRESH_BINARY | cv2.THRESH_OTSU)


                            if flag_changed == False:
                                #find contours again
                                dst_copy = np.copy(dst)
                                contours_a, hierarchy_a = cv2.findContours(dst_copy, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                                box_a_sorted = []
                                for count_a in range(0, len(contours_a)):
                                    #minAreaRect
                                    rect_a = cv2.minAreaRect(contours_a[count_a])
                                    box_a = cv2.cv.BoxPoints(rect_a)
                                    box_a = np.int0(box_a)
                                    area_a = cv2.contourArea(box_a)

                                    idx_a = hierarchy_a[0][count_a][2]
                                    location_area_size = location_width * location_height
                                    if idx_a != -1 and area_a > location_area_size * 0.7 and area_a < location_area_size*1.3:
                                        cv2.drawContours(dst, [box_a],0,0,2)
                                        #calc left-top
                                        box_argsort = np.argsort(box_a, axis=0)
                                        if box_argsort[0][1] == box_argsort[0][0] or box_argsort[1][1] == box_argsort[0][0]:
                                            box_left_top_idx = box_argsort[0][0]
                                        else:
                                            box_left_top_idx = box_argsort[1][0]

                                        for i in range(0,4):
                                            box_a_sorted.append(box_a[(box_left_top_idx + i) % 4])
                                        break
                            else:
                                box_a_sorted = after_point

                            if len(box_a_sorted) != 4:
                                break

                            #read each point
                            area_width = np.max(box_a_sorted, axis = 0)[0] - np.min(box_a_sorted, axis = 0)[0]
                            area_height = np.max(box_a_sorted, axis = 0)[1] - np.min(box_a_sorted, axis = 0)[1]
                            center_x = np.mean(box_a_sorted, axis = 0)[0]
                            location_ratio = (float(area_width + area_height) / (location_width + location_height + line_width*2))
                            space_x_mod = space_x * location_ratio
                            to_center_dst = (location_width/2 - (x_buffer + rect_len/2 + space_x * 2)) * location_ratio

                            dot_point = [0 for i in range(0,location_dot_num)]
                            for i in range(0, location_dot_num):
                                dst_area = dst[int(box_a_sorted[0][1] + area_height/2 - dot_read_area):int(box_a_sorted[0][1] + area_height/2 + dot_read_area), (int(center_x -  to_center_dst + space_x_mod * (i-2)) - dot_read_area):(int(center_x -  to_center_dst + space_x_mod * (i-2)) + dot_read_area)]
                                if np.mean(dst_area) < dot_read_thre:
                                    dot_point[i] = 1
                                # cv2.circle(dst, (int(center_x -  to_center_dst + space_x_mod * (i-2)), int(box_a_sorted[0][1] + area_height/2)), 5, 255, 3)
                                # cv2.circle(dst, (int(center_x -  to_center_dst + space_x_mod * (i-2)), int(box_a_sorted[0][1] + area_height/2 + 12)), 5, 0, 3)

                            if dot_point[0] ==1 and dot_point[location_dot_num - 1] == 0:
                                #calc dot id
                                dot_id = 0
                                for p in range(1,location_dot_num - 1):
                                    if dot_point[p] == 1:
                                        dot_id += 2**(p-1)

                                #homography_inv
                                after_point = np.float32([box[(i+id) % 4] for i in range(0, 4)])
                                before_point = np.float32([[0+point_buffer,0+point_buffer], [location_width+point_buffer, 0+point_buffer], [location_width+point_buffer, location_height+point_buffer], [0+point_buffer,location_height+point_buffer]])

                                M_inv = cv2.getPerspectiveTransform(before_point, after_point)
                                box_a_after = []
                                box_3d = [[box_a_sorted[i][0], box_a_sorted[i][1], 1] for i in range(0,4)]
                                for box_a_each in box_3d:
                                    box_a_after_each = np.matrix(M_inv) * np.matrix(box_a_each).T
                                    box_a_after_each /= box_a_after_each[2]
                                    box_a_after.append([float(box_a_after_each[0]),float(box_a_after_each[1])])
                                    cv2.circle(frame, (int(box_a_after_each[0]),int(box_a_after_each[1])), 5,(100,100,100), 5)

                                location_xy[dot_id].append(np.float32(box_a_after))
                                #write each id in this point
                                cv2.putText(frame, str(dot_id), (np.mean(box_a_after, axis = 0).astype(np.int)[0], np.mean(box_a_after, axis = 0).astype(np.int)[1]), cv2.FONT_HERSHEY_PLAIN, 5.0, (0,0,0), 5)

                        #memory save
                        del larger_area
                        gc.collect()

        #make from to vector
        vec_from_to = [[np.array([0,0]) for i in range(0,8)] for j in range(0,8)]
        # correction_x = int(-150 * float(220)/outer_size)
        # correction_y = int(-500 * float(220)/outer_size)
        correction_width = 5 * expand_ratio
        correction_height = 5 * expand_ratio
        right_basic_vec = np.array([float(postit_width/2 - horizon_x_buffer - location_width/2) / (location_width + correction_width), 0.0])
        under_basic_vec = np.array([0.0, float(postit_height/2 - horizon_y_buffer - location_height/2) / (location_height + correction_height)])

        vec_from_to[0][1] = right_basic_vec
        vec_from_to[0][2] = right_basic_vec * 2
        vec_from_to[0][3] = under_basic_vec * 2
        vec_from_to[0][4] = right_basic_vec + under_basic_vec * 2
        vec_from_to[0][5] = right_basic_vec * 2 + under_basic_vec * 2
        vec_from_to[0][6] = under_basic_vec
        vec_from_to[0][7] = right_basic_vec * 2 + under_basic_vec
        vec_from_to[1][2] = right_basic_vec
        vec_from_to[1][3] = -right_basic_vec + 2 * under_basic_vec
        vec_from_to[1][4] = under_basic_vec * 2
        vec_from_to[1][5] = right_basic_vec + under_basic_vec * 2
        vec_from_to[1][6] = -right_basic_vec + under_basic_vec
        vec_from_to[1][7] = right_basic_vec + under_basic_vec
        vec_from_to[2][3] = -right_basic_vec * 2 + under_basic_vec * 2
        vec_from_to[2][4] = -right_basic_vec + under_basic_vec * 2
        vec_from_to[2][5] = under_basic_vec * 2
        vec_from_to[2][6] = -right_basic_vec * 2 + under_basic_vec
        vec_from_to[2][7] = under_basic_vec
        vec_from_to[3][4] = right_basic_vec
        vec_from_to[3][5] = right_basic_vec * 2
        vec_from_to[3][6] = -under_basic_vec
        vec_from_to[3][7] = right_basic_vec*2 -under_basic_vec
        vec_from_to[4][5] = right_basic_vec
        vec_from_to[4][6] = -right_basic_vec - under_basic_vec
        vec_from_to[4][7] = right_basic_vec - under_basic_vec
        vec_from_to[5][6] = -right_basic_vec * 2 - under_basic_vec
        vec_from_to[5][7] = -under_basic_vec
        vec_from_to[6][7] = right_basic_vec * 2

        #other from to vectors
        for i in range(0,8):
            for j in range(0,8):
                #print vec_from_to[i][j]
                if (vec_from_to[i][j] == np.array([0,0]))[0] == True and (vec_from_to[i][j] == np.array([0,0]))[1] == True:
                    vec_from_to[i][j] = -vec_from_to[j][i]

        #search each point
        #for left_top
        location_points_all = []
        for m in range(0, len(location_xy)):
            #location_xy[0] includes boxes for id "0"
            for n in range(0, len(location_xy[m])):
                if location_xy[m][n][0][0] != -1:
                    right_vector = location_xy[m][n][1] - location_xy[m][n][0]
                    below_vector = location_xy[m][n][3] - location_xy[m][n][0]
                    #swap when m is 6 or 7(side place)
                    if m == 6 or m == 7:
                        below_vector_copy = np.copy(below_vector)
                        below_vector = np.array([right_vector[1], -right_vector[0]])
                        right_vector = np.array([below_vector_copy[1], below_vector_copy[0]])

                    sum_vector = right_vector + below_vector

                    #cv2.drawContours(frame,[np.array(postit_point).astype(np.int)], 0,(0,0,255),2)
                    location_points = [np.array([0,0]) for i in range(0,8)]
                    #location_info_bits = [1 for i in range(0,8)] #set 1 to avoid frequent result "0"
                    points_count = 0
                    points_to_delete = []
                    for i in range(0, 8):
                        if i != m:
                            dst_point = (np.mean(location_xy[m][n], axis = 0) + vec_from_to[m][i][0] * right_vector + vec_from_to[m][i][1] * below_vector).astype(np.int)
                            cv2.circle(frame, (dst_point[0], dst_point[1]), 10, (0,120,220), 5)
                            for j in range(0, len(location_xy[i])):
                                if location_xy[i][j][0][0] != -1 and np.linalg.norm(np.mean(location_xy[i][j], axis = 0) - dst_point) < error_thresh * (float(outer_size) / 220)**0.5:
                                    location_points[i] = np.mean(location_xy[i][j], axis = 0)
                                    #location_info_bits[i] = location_xy[i][j][1]
                                    #mark already used box
                                    points_to_delete.append([i, j])
                                    points_count += 1
                                    break
                    #cv2.imshow("dst points", cv2.resize(frame, (int(frame.shape[1] * 0.3), int(frame.shape[0] * 0.3))))
                    self.recognized_location_rectangle = points_count + 1
                    if points_count > 2:
                        #add self
                        location_points[m] = np.mean(location_xy[m][n], axis = 0)
                        #location_info_bits[m] = location_xy[m][n][1]
                        points_to_delete.append([m,n])
                        #delete already used box
                        for point_delete in points_to_delete:
                            location_xy[point_delete[0]][point_delete[1]][0][0] = -1

                        #append to all
                        location_points_all.append(location_points)


        #extract postit area
        #homography
        dst_points_original = [[horizon_x_buffer + location_width / 2, horizon_y_buffer+location_height / 2], [postit_width/2,horizon_y_buffer+location_height / 2], [postit_width-horizon_x_buffer - location_width / 2 ,horizon_y_buffer+location_height / 2],
                              [horizon_x_buffer + location_width / 2, postit_height-horizon_y_buffer-location_height / 2], [postit_width/2,postit_height-horizon_y_buffer-location_height / 2], [postit_width-horizon_x_buffer-location_width/2,postit_height-horizon_y_buffer-location_height / 2],
                              [horizon_x_buffer + location_height / 2, postit_height / 2],[postit_width-horizon_x_buffer-location_height/2, postit_height / 2]]

        #extract postit area for analyzing
        #homography
        dst_points_original_larger = []
        for each_dst_points in dst_points_original:
            dst_points_original_larger.append([each_dst_points[0] + larger_buffer, each_dst_points[1] + larger_buffer])

        #show frame and delete(for save memory)(information for read is drawn)
        show_img = cv2.resize(frame, (int(frame.shape[1] * 0.25), int(frame.shape[0]*0.25)))
        #cv2.imshow("show", show_img)
        del frame
        del show_img
        gc.collect()

        #delete old postit image
        old_postit_image = os.listdir('./tmp_datas/postit_tmp/')
        for old_postit in old_postit_image:
            os.remove('./tmp_datas/postit_tmp/' + old_postit)
        old_postit_image = os.listdir('./tmp_datas/postit_tmp_analyzing/')
        for old_postit in old_postit_image:
            os.remove('./tmp_datas/postit_tmp_analyzing/' + old_postit)
        #points include 8 points(1 postit's each point)
        count = 0
        for points in location_points_all:
            src_point = []
            dst_point = []
            dst_point_larger= []
            for i in range(0, len(points)):
                if points[i][0] != 0 and points[i][1] != 0:
                    src_point.append(points[i])
                    dst_point.append(np.float32(dst_points_original[i]))
                    dst_point_larger.append(np.float32(dst_points_original_larger[i]))
            M, mask = cv2.findHomography(np.array(src_point), np.array(dst_point_larger), cv2.RANSAC, 3)

            try:
                postit_result = cv2.warpPerspective(frame_original, M, (postit_width+larger_buffer * 2, postit_height+ larger_buffer * 2))
            except:
                continue
            #convert color to binary
            #postit_result_bin = np.copy(postit_result)
            postit_result_bin = cv2.cvtColor(postit_result, cv2.COLOR_BGR2GRAY)
            try:
                ret, postit_result_bin = cv2.threshold(postit_result_bin, 0.0, 255.0, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            except:
                continue
            #self.postit_image.append(postit_result[larger_buffer: (postit_result.shape[0] - larger_buffer), larger_buffer: (postit_result.shape[1] - larger_buffer)])
            postit_tmp = postit_result[larger_buffer: (postit_result.shape[0] - larger_buffer), larger_buffer: (postit_result.shape[1] - larger_buffer)]
            cv2.imwrite("./tmp_datas/postit_tmp/" + str(count) + ".jpg", postit_tmp)
            # cv2.imshow("postit_tmp", cv2.resize(postit_tmp,  (int(postit_tmp.shape[1] * 0.1), int(postit_tmp.shape[0]*0.1))))
            #cv2.imwrite("./postit_tmp_analyzing/" + str(count) + ".jpg", postit_result_bin)
            np.save("./tmp_datas/postit_tmp_analyzing/" + str(count) + ".npy", postit_result_bin)
            #self.postit_image_analyzing.append(postit_result_bin)

            M_inv, mask = cv2.findHomography(np.array(dst_point), np.array(src_point), cv2.RANSAC, 3)
            postit_area = []
            postit_area_before = np.array([(0,0,1), (postit_width, 0,1), (postit_width, postit_height,1),(0, postit_height,1)])
            postit_points_each = []
            for postit_area_before_each in postit_area_before:
                #print postit_area_before_each
                postit_area_after = np.matrix(M_inv) * np.matrix(postit_area_before_each).T
                postit_area_after /= postit_area_after[2]
                postit_points_each.append([float(postit_area_after[0]),float(postit_area_after[1])])
                #cv2.circle(frame, (postit_area_after[0], postit_area_after[1]), 5, (100,100,100), 2)
            self.postit_points.append(postit_points_each)
            count += 1

            #memory save
            del postit_result
            del postit_result_bin
            gc.collect()



class readDots():
    #for correct bit array
    bit_array = [0 for i in range(0,15)]
    outer_size = 1800 * (expand_ratio ** 2)
    def __init__(self, postit):
        #initialize
        self.bit_array = [0 for i in range(0,15)]


        #draw upper left and lower left
        start_left = horizon_x_buffer + location_width + rect_rect_space_horizon
        self.bit_array[0] = self.read_bit(postit, start_left, horizon_y_buffer, True)
        self.bit_array[1] = self.read_bit(postit, start_left, postit_height - horizon_y_buffer - bit_height, True)

        start_left += bit_width + rect_rect_space_horizon
        self.bit_array[2] = self.read_bit(postit, start_left, horizon_y_buffer, True)
        self.bit_array[3] = self.read_bit(postit, start_left, postit_height - horizon_y_buffer - bit_height, True)

        #draw upper right and lower right
        start_left = postit_width/2 + location_width/2 + rect_rect_space_horizon
        self.bit_array[4] =  self.read_bit(postit, start_left, horizon_y_buffer, True)
        self.bit_array[5] = self.read_bit(postit, start_left, postit_height - horizon_y_buffer - bit_height, True)

        start_left += bit_width + rect_rect_space_horizon
        self.bit_array[6] = self.read_bit(postit, start_left, horizon_y_buffer, True)
        #self.read_bit(bit_array_all[7],postit, start_left, postit_height - horizon_y_buffer - bit_height, True)

        #draw left upper and right upper
        start_upper = horizon_y_buffer + location_height + rect_rect_space_vertical
        self.bit_array[7] =  self.read_bit(postit, horizon_x_buffer, start_upper, False)
        self.bit_array[8] =  self.read_bit(postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)

        start_upper += bit_width + rect_rect_space_vertical
        self.bit_array[9] = self.read_bit(postit, horizon_x_buffer, start_upper, False)
        self.bit_array[10] = self.read_bit(postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)

        #draw left lower and right lower
        start_upper = postit_height/2 + location_width/2 + rect_rect_space_vertical
        self.bit_array[11] = self.read_bit(postit, horizon_x_buffer, start_upper, False)
        self.bit_array[12] = self.read_bit(postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)

        start_upper += bit_width + rect_rect_space_vertical
        self.bit_array[13] = self.read_bit(postit, horizon_x_buffer, start_upper, False)
        self.bit_array[14] = self.read_bit(postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)

    def read_bit(self, postit, first_x, first_y, horizon):
        #return val
        sum = 0

        #change parameter
        dot_read_thre = 100
        space_x_mod_ratio= 0.90

        #parameters for this purpose
        min_ratio = 0.75
        max_ratio = 1.1

        #add buffer
        first_x += larger_buffer
        first_y += larger_buffer
        if horizon == True:
            #extract search area
            min_y = max(0, first_y-search_buffer)
            max_y = min(postit.shape[0], first_y + bit_height + search_buffer)
            min_x = max(0, first_x-search_buffer)
            max_x = min(postit.shape[1], first_x + bit_width + search_buffer)

            search_area = postit[min_y:max_y, min_x:max_x]

            #find contours
            search_area_for_contours = np.copy(search_area)
            contours, hierarchy = cv2.findContours(search_area_for_contours, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            box_left_top_idx = -1
            # cv2.imshow("search_area", search_area)
            for count in range(0, len(contours)):
                #minAreaRect
                rect = cv2.minAreaRect(contours[count])
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                area = cv2.contourArea(box)
                # print area
                # cv2.waitKey(0)
                if area > self.outer_size * min_ratio and area < self.outer_size * max_ratio:
                    cv2.drawContours(search_area,[box],0,0,5)
                    #calc left-top
                    box_argsort = np.argsort(box, axis=0)
                    if box_argsort[0][1] == box_argsort[0][0] or box_argsort[1][1] == box_argsort[0][0]:
                        box_left_top_idx = box_argsort[0][0]
                    else:
                        box_left_top_idx = box_argsort[1][0]
                    break

            if box_left_top_idx == -1:
                return 100

            #make box_a_sorted
            box_a_sorted = []
            for i in range(0,4):
                box_a_sorted.append(box[(box_left_top_idx + i)%4])
            #set area parameters
            area_width = np.linalg.norm(box_a_sorted[1] - box_a_sorted[0])
            area_height = np.linalg.norm(box_a_sorted[2] - box_a_sorted[1])
            area_theta = np.pi/2
            if (box_a_sorted[1] - box_a_sorted[0])[0] != 0:
                area_tan = float((box_a_sorted[1] - box_a_sorted[0])[1]) / (box_a_sorted[1] - box_a_sorted[0])[0]
                area_theta = np.arctan(area_tan)

            center_x = np.mean(box_a_sorted, axis = 0)[0]
            center_y = np.mean(box_a_sorted, axis = 0)[1]
            bit_ratio = (float(area_width + area_height) / (bit_width + bit_height + line_width*2))
            space_x_mod = space_x * bit_ratio * space_x_mod_ratio
            space_x_mod_x = space_x_mod * np.cos(area_theta)
            space_x_mod_y = space_x_mod * np.sin(area_theta)
            to_center_dst = (bit_width/2 - (x_buffer + rect_len/2 + space_x * 4)) * bit_ratio
            to_center_dst_x = to_center_dst * np.cos(area_theta)
            to_center_dst_y = to_center_dst * np.sin(area_theta)



            dot_point = [0 for i in range(0,bit_num)]
            for i in range(0, bit_num):
                dst_area = search_area[int(center_y - to_center_dst_y + space_x_mod_y * (i-4) - dot_read_area):int(center_y - to_center_dst_y + space_x_mod_y * (i-4) + dot_read_area), (int(center_x -  to_center_dst_x + space_x_mod_x * (i-4) - dot_read_area)):(int(center_x -  to_center_dst_x + space_x_mod_x * (i-4) + dot_read_area))]
                if dst_area.shape[0] == 0: break
                if np.mean(dst_area) < dot_read_thre:
                    dot_point[i] = 1
                # cv2.circle(search_area, (int(center_x -  to_center_dst_x + space_x_mod_x * (i-4)), int(center_y - to_center_dst_y + space_x_mod_y * (i-4))), 5, 255, 3)
                # cv2.circle(search_area, (int(center_x -  to_center_dst_x + space_x_mod_x * (i-4)), int(center_y - to_center_dst_y + space_x_mod_y * (i-4) + 12)), 5, 0, 3)
            dot_point[0] = dot_point[bit_num - 1]

            for i in range(0, bit_num-1):
                if dot_point[i] == 1:
                    sum += 2 ** (i)
            return sum

        if horizon == False:
            #extract search area
            min_y = max(0, first_y-search_buffer)
            max_y = min(postit.shape[0], first_y + bit_width + search_buffer)
            min_x = max(0, first_x-search_buffer)
            max_x = min(postit.shape[1], first_x + bit_height + search_buffer)

            search_area = postit[min_y:max_y, min_x:max_x]

            #find contours
            search_area_for_contours = np.copy(search_area)
            contours, hierarchy = cv2.findContours(search_area_for_contours, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            box_left_top_idx = -1
            for count in range(0, len(contours)):
                #minAreaRect
                rect = cv2.minAreaRect(contours[count])
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                area = cv2.contourArea(box)
                if area > self.outer_size * min_ratio and area < self.outer_size * max_ratio:
                    #print area
                    cv2.drawContours(search_area,[box],0,0,5)
                     #calc left-top
                    box_argsort = np.argsort(box, axis=0)
                    if box_argsort[0][1] == box_argsort[0][0] or box_argsort[1][1] == box_argsort[0][0]:
                        box_left_top_idx = box_argsort[0][0]
                    else:
                        box_left_top_idx = box_argsort[1][0]
                    break

            if box_left_top_idx == -1:
                return 100

            #make box_a_sorted
            box_a_sorted = []
            for i in range(0,4):
                box_a_sorted.append(box[(box_left_top_idx + i)%4])
            #set area parameters
            area_width = np.linalg.norm(box_a_sorted[1] - box_a_sorted[0])
            area_height = np.linalg.norm(box_a_sorted[2] - box_a_sorted[1])
            area_theta = np.pi/2
            if (box_a_sorted[2] - box_a_sorted[1])[0] != 0:
                area_tan = float((box_a_sorted[2] - box_a_sorted[1])[1]) / (box_a_sorted[2] - box_a_sorted[1])[0]
                area_theta = np.arctan(area_tan)
                if area_theta < 0:
                    area_theta += np.pi

            center_x = np.mean(box_a_sorted, axis = 0)[0]
            center_y = np.mean(box_a_sorted, axis = 0)[1]
            bit_ratio = (float(area_width + area_height) / (bit_width + bit_height + line_width*2))
            space_x_mod = space_x * bit_ratio * space_x_mod_ratio
            space_x_mod_x = space_x_mod * np.cos(area_theta)
            space_x_mod_y = space_x_mod * np.sin(area_theta)
            to_center_dst = (bit_width/2 - (x_buffer + rect_len/2 + space_x * 4)) * bit_ratio
            to_center_dst_x = to_center_dst * np.cos(area_theta)
            to_center_dst_y = to_center_dst * np.sin(area_theta)

            dot_point = [0 for i in range(0,bit_num)]
            for i in range(0, bit_num):
                dst_area = search_area[(int(center_y -  to_center_dst_y + space_x_mod_y * (i-4) - dot_read_area)):(int(center_y -  to_center_dst_y + space_x_mod_y * (i-4) + dot_read_area)),int(center_x - to_center_dst_x + space_x_mod_x * (i-4) - dot_read_area):int(center_x - to_center_dst_x + space_x_mod_x * (i-4) + dot_read_area)]
                if dst_area.shape[0] == 0: break
                if np.mean(dst_area) < dot_read_thre:
                    dot_point[i] = 1
                # cv2.circle(search_area, (int(center_x - to_center_dst_x + space_x_mod_x * (i-4)),int(center_y -  to_center_dst_y + space_x_mod_y * (i-4)) ), 5, 255, 3)
                # cv2.circle(search_area, (int(center_x - to_center_dst_x + space_x_mod_x * (i-4) + 12),int(center_y -  to_center_dst_y + space_x_mod_y * (i-4)) ), 5, 0, 3)
            dot_point[0] = dot_point[bit_num - 1]
            for i in range(0, bit_num-1):
                if dot_point[i] == 1:
                    sum += 2 ** (i)
            return sum
