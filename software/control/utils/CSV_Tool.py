# -*- coding: utf-8 -*-
"""
Created on Wed May 16 14:24:17 2018

@author: Francois & Deepak
"""
import csv

'''       
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#                             CSV Communication
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''
class CSV_Register():
    
    def __init__(self,parent=None, header = None):
        self.file_directory=None
        self.header = header
        self.currFile = None
        self.writer = None
        
        
    def start_write(self):
        
        self.currFile = open(self.file_directory,'w')
        csv.register_dialect('myDialect', delimiter=',', quoting=csv.QUOTE_MINIMAL,lineterminator = '\n')
        self.writer = csv.writer(self.currFile , dialect='myDialect')
        self.writer.writerows(self.header)
        
    def write_line(self,data):
        self.writer.writerows(data)
        
    def close(self):
        if(self.currFile is not None):
            self.currFile.close()