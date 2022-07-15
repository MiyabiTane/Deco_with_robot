#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

# This is fake of Choregraph's function
# will be made by Aiko Ichikura
class Talk(object):
    def __init__(self):
        print("Start node of choregraph")
    
    def episode_11(self):
        for i in range(11):
            print("epospde 1-1: {}".format(i))
            time.sleep(1)

    def episode_12(self):
        for i in range(17):
            print("eposode 1-2: {}".format(i))
            time.sleep(1)

    def episode_13(self):
        for i in range(11):
            print("eposode 1-3: {}".format(i))
            time.sleep(1)
