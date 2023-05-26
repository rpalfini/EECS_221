#!/usr/bin/env python2

def my_function(**kwargs):
    for key, value in kwargs.items():
        print(key, value)

my_dict = {'a': 1, 'b': 2, 'c': 3}
my_dict['chicken'] = 7
my_function(cat=7,dog=5,**my_dict)




