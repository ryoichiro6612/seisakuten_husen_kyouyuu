import sys

def main():
    if len(sys.argv) != 6:
        print "usage:" + sys.argv[0] + " final_csv naruhodo_csv time_dif(movie_time - time_dif = naruhodo_time) before_naruhodo after_naruhodo"
        return -1
    #read argv
    final_csv = sys.argv[1]
    naruhodo_csv = sys.argv[2]
    time_dif = int(sys.argv[3]) #movie_time - time_dif = naruhodo_time

    #settings
    before_naruhodo = int(sys.argv[4]) #15?
    after_naruhodo = int(sys.argv[5]) #10?


    #read naruhodo
    file_naruhodo = open('./datas/csv/' + naruhodo_csv, "r")
    naruhodo_list = file_naruhodo.readlines()
    #remove "\n"
    for i in range(len(naruhodo_list)):
        naruhodo_list[i] = int(naruhodo_list[i][:-1])

    #read final
    file_final = open('./datas/csv/' + final_csv, "r")
    final_list = file_final.readlines()
    for i in range(len(final_list)):
        each_item = final_list[i].split(',')
        appear_time = int(each_item[1]) - time_dif
        naruhodo_count = sum(naruhodo_list[max(min(appear_time - before_naruhodo, len(naruhodo_list)-1), 0):min(max(appear_time + after_naruhodo,0), len(naruhodo_list))])
        final_list[i] = final_list[i][:-1]
        final_list[i] += 'n,' + str(naruhodo_count) + '\n'
    
    #write result file
    file_result = open('./datas/csv/final_naruhodo.csv',"w")
    for final_list_each in final_list:
        file_result.write(final_list_each)

if __name__ == "__main__":
    main()