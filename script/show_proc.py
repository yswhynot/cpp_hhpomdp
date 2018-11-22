import numpy as np
import cv2
import argparse

def plot_step(robot, target, img, res):
    # draw target
    x = target[1]
    y = target[0]
    cv2.rectangle(img, (int(x - res/2), int(y - res/2)),
            (int(x + res/2), int(y + res/2)), (100, 30, 30), -1)

    # draw pursuers
    for r in robot:
        x = r[1]
        y = r[0]
        cv2.rectangle(img, (int(x - res/2)+1, int(y - res/2)+1),
            (int(x + res/2)-1, int(y + res/2)-1), (30, 30, 100), -1)

    return img

def get_empty_map(map_mat, res):
    row, col = map_mat.shape
    height = row 
    width = col 
    img = np.full((height*res, width*res, 3), 255, np.uint8)
    for h in range(height):
        for w in range(width):
            if map_mat[h, w] == 0: continue
            hi = h * res 
            wi = w * res 
            cv2.rectangle(img, (wi, hi), (wi+res, hi+res), (0, 0, 0), -1)
    return img

def main(rnum, result, map_mat):
    step = rnum + 1
    row, col = result.shape
    res = 50
    rate = 10
    vel = 0.1

    prev_r = result[0:rnum, :]
    prev_t = result[rnum, :]
    for i in range(row / step):
        robot = result[(i*step):(i*step+rnum), :]
        target = result[(i*step+rnum), :]

        prev_r_pxl = prev_r*res + res/2
        prev_t_pxl = prev_t*res + res/2
        r_pxl = robot*res + res/2
        t_pxl = target*res + res/2
        c = int(np.max(np.sum(np.abs(r_pxl - prev_r_pxl), axis=0)) / rate / vel) + 1
        step_r = (r_pxl - prev_r_pxl) / c
        step_t = (t_pxl - prev_t_pxl) / c
        for ci in range(c):
            cr_pxl = prev_r_pxl + step_r*ci
            ct_pxl = prev_t_pxl + step_t*ci
            img = get_empty_map(map_mat, res)
            img = plot_step(cr_pxl, ct_pxl, img, res)
            cv2.imshow("Map", img)
            cv2.waitKey(rate)
        prev_r = robot.copy()
        prev_t = target.copy()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Display HHPOMDP experiment result')
    parser.add_argument('ep', type=int, help="Episode number")
    parser.add_argument('m', type=int, help="Map ID")
    parser.add_argument('r', type=int, help="Number of robots")
    args = parser.parse_args()
    
    result = np.loadtxt('../data/ep/ep%d_r%d_m%d.txt' % (args.ep, args.r, args.m))
    map_mat = np.loadtxt('../data/map/map%d.txt' % args.m)
    main(args.r, result, map_mat)

