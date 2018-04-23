import numpy as np

if __name__ == '__main__':

    file = open("mask_area.csv", "w")
    original = np.array([i for i in range(0, 23)])
    # for selecting one rectangle
    already_exist = []
    for j in range(0, 23):
        while ():
            a = np.random.permutation(original)
            out_array = a[0:1]
            out_array.sort()
            if out_array not in already_exist:
                already_exist.append(out_array)
                break
        for out in out_array:
            file.write(str(out))
            file.write(",")
        file.write("\n")

    # for selecting more than one rectangle
    for i in range(1, 11):
        already_exist = []
        for j in range(0, 100):
            while (1):
                exist_flag = False
                a = np.random.permutation(original)
                out_array = a[0:i + 1]
                out_array.sort()

                for already_exist_each in already_exist:
                    if all(out_array == already_exist_each):
                        exist_flag = True
                        break
                if not exist_flag:
                    already_exist.append(out_array)
                    break
            for out in out_array:
                file.write(str(out))
                file.write(",")
            file.write("\n")
