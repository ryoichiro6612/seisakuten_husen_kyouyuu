
import cv2
import numpy as np
from reedsolo import RSCodec
import yaml

#global parameters
postit_config = yaml.load(open("./config/postit_config.yaml", "r"))

#postit parameters
postit_width = postit_config["postit_width"]
postit_height = postit_config["postit_height"]

#common parameters for inner
space_x = postit_config["space_x"]
rect_len = postit_config["rect_len"]
x_buffer = postit_config["x_buffer"] #from rectangle's edge
y_buffer = postit_config["y_buffer"] #from rectangle's edge

#bit parameters for outer
bit_num = postit_config["bit_num"]
bit_width = x_buffer*2 + space_x * (bit_num - 1) + rect_len
bit_height = postit_config["bit_height"]

#location parameters for outer
location_dot_num = postit_config["location_dot_num"]
horizon_x_buffer = postit_config["horizon_x_buffer"] #from postit's edge
horizon_y_buffer = postit_config["horizon_y_buffer"] #from postit's edge
location_width = x_buffer*2 + space_x * (location_dot_num - 1) + rect_len
location_height = postit_config["location_height"]
horizon_space = postit_width/2 - horizon_x_buffer - location_width/2

#common parameters for outer
line_width = postit_config["line_width"]
rect_rect_space_horizon = (horizon_space - location_width - bit_width*2) / 3
rect_rect_space_vertical = (postit_height/2 - horizon_y_buffer - location_height - location_width/2 - bit_width*2) / 3

def draw_location_dot(postit, bit_array, first_x, first_y, horizon):

    if horizon == True:
        #draw outer rectangle
        cv2.rectangle(postit, (first_x, first_y), (first_x + location_width, first_y + location_height), 0, line_width)

        #draw inner rectangle
        for i in range(0, len(bit_array)):
            if bit_array[i] == 1:
                cv2.rectangle(postit, (first_x + i*space_x + x_buffer, first_y+y_buffer), (first_x + i*space_x + x_buffer+ rect_len, first_y + y_buffer+ rect_len),0,-1)

    if horizon == False:
        #draw outer rectangle
        cv2.rectangle(postit, (first_x, first_y), (first_x + location_height, first_y + location_width), 0, line_width)

        #draw inner rectangle
        for i in range(0, len(bit_array)):
            if bit_array[i] == 1:
                cv2.rectangle(postit, (first_x +y_buffer, first_y+ i*space_x + x_buffer), (first_x + y_buffer+ rect_len, first_y + i*space_x + x_buffer+ rect_len),0,-1)

def draw_bit(bit_array, postit, first_x, first_y, horizon):
    if horizon == True:
        #draw outer rectangle
        cv2.rectangle(postit, (first_x, first_y), (first_x + bit_width, first_y + bit_height), 0, line_width)
        #draw inner rectangle
        for i in range(0, len(bit_array)):
            if bit_array[i] == 1:
                cv2.rectangle(postit, (first_x + x_buffer + i*space_x, first_y + y_buffer), (first_x + x_buffer + i*space_x + rect_len, first_y + y_buffer + rect_len),0,-1)
    
    if horizon == False:
        #draw outer rectangle
        cv2.rectangle(postit, (first_x, first_y), (first_x + bit_height, first_y + bit_width), 0, line_width)
        #draw inner rectangle
        for i in range(0, len(bit_array)):
            if bit_array[i] == 1:
                cv2.rectangle(postit, (first_x + y_buffer , first_y+ x_buffer + i*space_x), (first_x + y_buffer + rect_len, first_y + x_buffer + i*space_x +  rect_len),0,-1)

def draw_all_bit(bit_array_all, postit):

        #draw upper left and lower left
        start_left = horizon_x_buffer + location_width + rect_rect_space_horizon
        draw_bit(bit_array_all[0],postit, start_left, horizon_y_buffer, True)        
        draw_bit(bit_array_all[1],postit, start_left, postit_height - horizon_y_buffer - bit_height, True)
        
        start_left += bit_width + rect_rect_space_horizon
        draw_bit(bit_array_all[2],postit, start_left, horizon_y_buffer, True)
        draw_bit(bit_array_all[3],postit, start_left, postit_height - horizon_y_buffer - bit_height, True)
                
        #draw upper right and lower right
        start_left = postit_width/2 + location_width/2 + rect_rect_space_horizon
        draw_bit(bit_array_all[4],postit, start_left, horizon_y_buffer, True)
        draw_bit(bit_array_all[5],postit, start_left, postit_height - horizon_y_buffer - bit_height, True)
        
        start_left += bit_width + rect_rect_space_horizon
        draw_bit(bit_array_all[6],postit, start_left, horizon_y_buffer, True)
        #draw_bit(bit_array_all[7],postit, start_left, postit_height - horizon_y_buffer - bit_height, True)

        #draw left upper and right upper
        start_upper = horizon_y_buffer + location_height + rect_rect_space_vertical
        draw_bit(bit_array_all[7],postit, horizon_x_buffer, start_upper, False)
        draw_bit(bit_array_all[8],postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)
        
        start_upper += bit_width + rect_rect_space_vertical
        draw_bit(bit_array_all[9],postit, horizon_x_buffer, start_upper, False)
        draw_bit(bit_array_all[10],postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)

        #draw left lower and right lower
        start_upper = postit_height/2 + location_width/2 + rect_rect_space_vertical
        draw_bit(bit_array_all[11],postit, horizon_x_buffer, start_upper, False)
        draw_bit(bit_array_all[12],postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)
        
        start_upper += bit_width + rect_rect_space_vertical
        draw_bit(bit_array_all[13],postit, horizon_x_buffer, start_upper, False)
        draw_bit(bit_array_all[14],postit, postit_width - bit_height - horizon_x_buffer, start_upper, False)

def main():

    for i in range(0,256):
        postit = np.tile(np.uint8([255]), (postit_height, postit_width,1))               

        #draw location dot
        for count in range(0,8):
            location_dot_array = [1] + [0 for j in range(0, location_dot_num-1)]
            for j in range(0, len(location_dot_array)-2):
                if count & 2**j != 0:
                    location_dot_array[j+1] = 1
            if count < 3:
             draw_location_dot(postit,location_dot_array, horizon_x_buffer + count * horizon_space , horizon_y_buffer, True)
            elif count < 6:
             draw_location_dot(postit,location_dot_array, horizon_x_buffer + (count-3) * horizon_space , postit_height - horizon_y_buffer - location_height, True)
            elif count == 6:
                draw_location_dot(postit,location_dot_array, horizon_y_buffer, postit_height/2 - location_width/2, False)                
            elif count == 7:
                draw_location_dot(postit,location_dot_array, postit_width - horizon_y_buffer - location_height, postit_height/2 - location_width/2, False)
       
        #need reed solomon
        rs = RSCodec(14)
        rs_result = rs.encode([i])

        #make bit array
        bit_array_all = []
        for number in rs_result:
            #print number
            bit_array = [0 for j in range(0,bit_num)]
            for j in range(0, bit_num - 1):
                if number & 2**j != 0:
                    bit_array[j] = 1
            bit_array[bit_num - 1] = bit_array[0]
            bit_array_all.append(bit_array)

        #draw bit array
        draw_all_bit(bit_array_all, postit)

        #outer rectangle
        cv2.rectangle(postit, (0,0), (postit_width-1,postit_height-1), 0, 1)
    
        #cv2.imshow("result", postit)
        #cv2.waitKey(0)
        filename = "./datas/postit/postit" + str(i) + ".jpg"
        cv2.imwrite(filename, postit)





if __name__ == '__main__':
    main()