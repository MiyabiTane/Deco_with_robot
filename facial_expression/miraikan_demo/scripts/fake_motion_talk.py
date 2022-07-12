#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

# This is fake of Choregraph's function
# will be made by Aiko Ichikura
class Talk(object):
    def __init__(self):
        print("Start node of choregraph")
    
    def test_func_0(self):
        for i in range(5):
            print("func0: {}".format(i))
            time.sleep(1)

    def test_func_1(self):
        for i in range(5):
            print("func1: {}".format(i))
            time.sleep(1)
