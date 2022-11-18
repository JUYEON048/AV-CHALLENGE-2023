#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

def Highway(front_dist, before_dist, before_velocity):    #선행차와의 거리로 가속, 제동여부 결정/세부 수치는 필요시 변경
    print("Highway Function",front_dist)
    print("before_dist = ", before_dist)
    velocity = 0

    if front_dist <= 35.0:
        if before_velocity == 40:
            velocity = 0
        else:
            velocity = 0

    #elif front_dist > 27.0 and front_dist <= 33.0:
    #    velocity = 15

    #elif front_dist > 33.0 and front_dist <= 50.0:
    #    velocity = 30

    elif front_dist > 35.0:
        if before_velocity == 0:
            velocity = 0
        else:
            velocity = 40

    return velocity

        

if __name__ == '__main__':
    print("Algorithm_Highway.py")
    
