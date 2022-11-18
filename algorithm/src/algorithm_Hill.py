#!/usr/bin/env python
# -*- coding: utf-8 -*-
global dist_list

num = 0
def Hill(front_dist):        #선행차와의 거리로 가속, 제동여부 결정/세부 수치는 필요시 변경
    print("Hill Function",front_dist)
    if num == 0:
        dist_list = []
    dist_list.append(front_dist)
    velocity = 0

    if front_dist <= 15.0:      #따라가고 서는 거리 일단 임의로 4m 지정
        velocity = 0            #선행차와 거리에 따른 차속 결정

    elif front_dist > 15.0 and front_dist <=24.0:
        velocity = 10

    elif front_dist > 24.0:
        velocity = 20
        
    if len(dist_list) ==2:
        if (dist_list[0] - dist_list[1]) > 19:
            front_dist = dist_list[1]
        elif (dist_list[1] - dist_list[0]) > 19:
            front_dist = dist_list[0]
        dist_list = []
    return velocity



if __name__ == '__main__':
    print("Algorithm_Hill.py")
