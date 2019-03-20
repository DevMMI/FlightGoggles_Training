import sys
import os
# 6864
# 7628
images_dir = "/home/mohamedisse/catkin_ws/src/data/images"
labels_dir = "/home/mohamedisse/catkin_ws/src/data/labels"

def main():
    if(len(sys.argv) < 3):
        print("Usage:\n$ python clear_values.py <int>beginning_of_bad_vals <int>end_of_bad_vals")
        exit(-1)

    begin = int(sys.argv[1])
    end = int(sys.argv[2])

    if(end < begin):
        print("bad values error")
        exit(-1)

    print("{} {}".format(begin, end))
    for i in range(begin, end+1):
        img_l = os.path.join(images_dir, "left{}.jpg".format(i))
        img_r = os.path.join(images_dir, "right{}.jpg".format(i))
        label_l = os.path.join(labels_dir, "label_l{}.txt".format(i))
        label_r = os.path.join(labels_dir, "label_r{}.txt".format(i))

        if os.path.isfile(img_l):
            os.remove(img_l)
        if os.path.isfile(img_r):
            os.remove(img_r)
        if os.path.isfile(label_l):
            os.remove(label_l)
        if os.path.isfile(label_r):
            os.remove(label_r)

        # print(img_l)
        # print(img_r)
        # print(label_l)
        # print(label_r)




main()
