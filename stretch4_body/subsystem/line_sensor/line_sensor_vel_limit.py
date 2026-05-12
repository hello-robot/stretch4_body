#!/usr/bin/env python3
from stretch4_body.core.device import Device
import stretch4_body.core.hello_utils as hu
import numpy as np
import cv2
import math

class SensorEdgeModel:
    def __init__(self,name,dropoff_deg,normal):
        """
        dropoff_deg: degrees that adjacent sensors hazard bleed into the current sensor's model value, 0 is none, 30 is completely
        normal: orientation of edge normal (deg), in global coordinates

        """
        self.name=name
        a=(60-dropoff_deg*2) #invert the dropoff_deg to get the size of the middle segment, where the 60 deg is divided b--a--b
        a = min(60, max(0, a)) #deg
        #divide range into three segments x0--x1--x2--x3, across 0-60 deg range
        self.x0=0
        self.x1=(60-a)/2
        self.x2=self.x1+a
        self.x3=60.0

        self.normal=normal
        self.y0=0 #value of first end point
        self.y1=0 #value of second end point (ccw 60 deg from y0)
        self.is_hazard=False
        self.h_start=self.normal-30.0 #Start heading
        if self.h_start<0:
            self.h_start=360+self.h_start
        self.h_end=math.fmod(self.normal+30.0,360.0) #End heading (less than)

    def get_velocity_limit(self,heading):
        """
        Get vel scalar value (0-1) at heading. Return 1.0 if not in range
        """
        if self.is_heading_in_range(heading):
            if self.is_hazard:
                return 0.0
            pct=self._global_heading_to_edge_pct(heading)
            if pct is not None:
                ed=pct*60.0 #deg along the edge from 0 to 60

                if ed<self.x1:
                    return self.y0 + ed*(1.0-self.y0)/(self.x1-self.x0)
                if ed>=self.x1 and ed<=self.x2:
                    return 1.0
                if ed>self.x2:
                    return 1.0 + (ed-self.x2)*(self.y1-1.0)/(self.x3-self.x2)
        return 1.0


    def is_heading_in_range(self,h):
        return self._global_heading_to_edge_pct(h) is not None

    def _global_heading_to_edge_pct(self,h):
        """
        Convert a global heading (deg, 0-360) to a percentage of distance along the edge  (0 to 1 ) going CCW,
        where 0.5 is aligned with the sensor normal
        Return None if heading not in range (+/-30 deg of normal)
        """
        if h<0:
            h=h+360.0
        if h>360:
            h=h-360.0
        rp=math.fmod(self.normal+30.0,360.0)
        rn = self.normal - 30.0
        if rn<0: #Rollover
            if h<=rp:
                return 0.5+(h-self.normal)/60.0
            rn=360+self.normal-30.0
            if h>=rn:
                #print(h,'rn',rn)
                return -(rn-h)/60.0
        else:
            if rp==0:
                rp=360.0
            if h<=rp and h>=rn:
                return ((h-rn) / 60.0)
        return None


class LineSensorVelLimit(Device):
    """
    Given six line sensors around the base, modulate the maximum velocity of the base
    along a given heading using a simple model

    Each sensor faces one edge, with two vertices, of a hexagon, spanning a 60 deg range of headings

    Given a heading, h, return a scalar s (0-1) by which a target velocity along that heading is modulated

    To determine, s,
    - if the sensor along direction h detects a hazard, return 0
    - otherwise, return a value according to the SensorEdgeModel
    """
    
    
    def __init__(self,sensor_names):
        """
        Input list of six sensor names
        """
        Device.__init__(self,name='line_sensor_vel_limit')
        self.edges={}
        self.neighbor_0={}
        self.neighbor_1={}
        idx=0
        for sn in sensor_names:
            normal=math.fmod(self.params['sensor_normals'][sn]+self.params['phase_adj'],360.0)
            self.edges[sn]=SensorEdgeModel(sn,self.params['dropoff_deg'],normal) #Rotate system to match physical layout by phase_adj
            idx=idx+1

        self.neighbor_1[sensor_names[0]] = self.edges[sensor_names[5]]
        self.neighbor_0[sensor_names[5]] = self.edges[sensor_names[0]]
        for idx in range(6):
            if idx>0:
                self.neighbor_1[sensor_names[idx]] = self.edges[sensor_names[idx-1]]
            if idx<5:
                self.neighbor_0[sensor_names[idx]] = self.edges[sensor_names[idx+1]]
        self.is_valid=False
    
    def step(self,line_sensors_status):
        self.is_valid=True
        for sn in self.edges:
            self.edges[sn].is_hazard= (not(line_sensors_status[sn]['detection'] =='floor'))
        for sn in self.edges:
                self.edges[sn].y0=int(not self.neighbor_0[sn].is_hazard)
                self.edges[sn].y1=int(not self.neighbor_1[sn].is_hazard)
                    

    def get_sensor_on_heading(self,heading):
        """
        Get the sensor name on the heading
        """
        if self.is_valid:
            for sn in self.edges:
                if self.edges[sn].is_heading_in_range(heading):
                    return sn
        return None

    def get_velocity_limit(self,heading):
        sn=self.get_sensor_on_heading(heading)
        if sn is not None:
            return self.edges[sn].get_velocity_limit(heading)
        return 0.0 #Not yet valid, default to off

    def get_vel_limit_image(self):
        #Create image
        sz=1000
        im_width = im_height =1000
        r=100
        rr=int(sz/2-r)-50
        image = np.zeros((im_height, im_width,3), dtype=np.uint8)
        image=cv2.circle(image,center=(int(im_width/2),int(im_height/2)),radius=r, color=(255,0,255),thickness=2)
         
        for sn in self.edges:

            for i in range(60):
                h = math.fmod(self.edges[sn].h_start + i, 360)
                pct = self.edges[sn].get_velocity_limit(h)
                start_x = int(r * math.cos(hu.deg_to_rad(h)) + im_width / 2)
                start_y = int(r * math.sin(hu.deg_to_rad(h)) + im_height / 2)
                end_x = int(start_x + rr * math.cos(hu.deg_to_rad(h))*pct)
                end_y = int(start_y + rr * math.sin(hu.deg_to_rad(h)) * pct)
                if i == 30: #Center of sensor range
                    end_x = int(start_x + rr * math.cos(hu.deg_to_rad(h)))
                    end_y = int(start_y + rr * math.sin(hu.deg_to_rad(h)))
                    image = cv2.line(image, (start_x, start_y), (end_x, end_y), (255, 0, 0), 4) #Blue sensor center line
                    #Write sensor name
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_size = 0.5
                    cv2.putText(image, sn, (end_x,end_y), font, font_size, (255, 255, 255), 1, cv2.LINE_AA)

                elif pct>0:
                    color=(0,255,0)
                    if pct<.999:
                        color=(0,255,255)
                    image=cv2.line(image, (start_x, start_y), (end_x,end_y), color, 1) #Ray showing vel limit
        #Forward arrow
        image=cv2.line(image, (int(im_width/2),int(im_height/2)-r), (int(im_width/2),0), color=(0,0,255), thickness=6)#fwd
        image=cv2.line(image, (int(im_width/2),0), (int(im_width/2)+40,40), color=(0,0,255), thickness=6)#fwd
        image=cv2.line(image, (int(im_width/2),0), (int(im_width/2)-40,40), color=(0,0,255), thickness=6)#fwd
        return image

    def show(self):
        image=self.get_vel_limit_image()
        cv2.imshow('Line Sensor Velocity Limit',image)
        cv2.waitKey(1)



if __name__ == '__main__':
    import stretch4_body.subsystem.line_sensor.line_sensor_array as ll
    import time
    ls=ll.LineSensorArray()
    ls.startup()
    lvl=LineSensorVelLimit(sensor_names=ls.params['sensor_names'])

    while True:
        print('--------------')
        ts = time.time()
        ls.step_model()
        dt = time.time() - ts
        ls.pretty_print()
        lvl.step(ls.status)
        lvl.show()
        #print('#########################################')
        # print(l.status)
        # print('#########################################3')
        time.sleep(0.03)
    ls.stop()
