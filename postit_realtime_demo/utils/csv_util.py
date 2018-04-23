def write_every_second(postit_saved, file):
    for key, val in postit_saved.items():
        file.write(str(key) + ",")
        file.write(str(val["points"][0][0]) + "," + str(val["points"][0][1]) + "," +str(val["points"][1][0]) + "," 
                   +str(val["points"][1][1]) + "," + str(val["points"][2][0]) + "," + str(val["points"][2][1]) + ","
                   +str(val["points"][3][0]) + "," +str(val["points"][3][1]) + ",")
    file.write("\n")

def write_final(postit_saved, file):
    for key, val in postit_saved.items():
        file.write(str(key) + ",")
        file.write(str(val["first_time"]) + ",")
        for time in val["move"]:
            file.write(str(time) + ",")
        file.write("r,")
        for time in val["rotate"]:
            file.write(str(time) + ",")
        file.write("\n")