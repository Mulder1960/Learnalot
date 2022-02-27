# -*- coding: utf-8 -*-
"""
Created on Fri Jan 24 08:03:33 2020

@author: Frans
"""


import struct
import matplotlib.pyplot as plt
import numpy as np
import datetime
import time
import calendar
Season_List=['Wi','Sp','Su','Au']
Season_ptr = 0

Day_of_Week = ['Ma','Di','Wo','Do','Vr','Za','Zo']
Month_of_Year = ['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec']
Day_time_Long = ["Early Morning","Morning","Noon","Early Afternoon","Afternoon","Evening","Night","Late Night"]
Day_time_Short = ["EM","MO","NO","EA","AF","EV","NI","LN"]

Analyse_Date = datetime.datetime(2019,12,22,3,30,15)
Analyse_Date_UTC = int(calendar.timegm(Analyse_Date.timetuple()))


print("Date to Analyse : " + 
      datetime.datetime.fromtimestamp(Analyse_Date_UTC).strftime('%Y-%m-%d %H:%M:%S')
      )
print("Year        : " , Analyse_Date.timetuple()[0])
print("Month       : " , Month_of_Year[Analyse_Date.timetuple()[1]-1])
print("Day         : " , Analyse_Date.timetuple()[2])
print("Week nr     : " , Analyse_Date.isocalendar()[1])
print("Weekday     : " , Day_of_Week[Analyse_Date.timetuple()[6]])
print("Day in year : " , Analyse_Date.timetuple()[7])
#find season
if (Analyse_Date.timetuple()[7]<81):
    Season_ptr=0
else:
    if (Analyse_Date.timetuple()[7]<173):
          Season_ptr=1
    else:
         if (Analyse_Date.timetuple()[7]<264):
            Season_ptr=2
         else:
             if (Analyse_Date.timetuple()[7]<356):
                Season_ptr=3
             else: Season_ptr=0

print("Season      : ", Season_List[Season_ptr])
