'''
Author: yangpeng yangpeng@meta-bounds.com
Date: 2022-11-13 02:49:19
LastEditTime: 2022-11-13 03:22:39
LastEditors: yangpeng yangpeng@meta-bounds.com
Description: 
FilePath: /src/script/graph_compare.py
'''
import sys
import os

def compare_file(file_path1, file_path2):
    print("diff factors in file: %s"%file_path1)
    file1 = open(file_path1, 'r')
    for line1 in file1:
        valid_str_line1 = line1.split(';')[0]
        valid_str_line2 = ''
        exist = False
        file2 = open(file_path2, 'r')
        for line2 in file2:
            valid_str_line2 = line2.split(';')[0]
            if valid_str_line1 == valid_str_line2:
                exist = True
                break
        if not exist:
            print (valid_str_line1)
        file2.close()
    file1.close()

if '__main__' == __name__:
    if len(sys.argv) != 3:
        print ('usage: %s file1, file2' % sys.argv[0])
        exit(-1)
    f1 = sys.argv[1]
    f2 = sys.argv[2]
    compare_file(f1, f2)
    compare_file(f2, f1)
    

