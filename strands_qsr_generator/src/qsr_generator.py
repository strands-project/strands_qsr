#!/usr/bin/env python
import roslib; roslib.load_manifest('strands_qsr_generator')

import rospy

from strands_qsr_msgs.srv import *
from strands_qsr_msgs.msg import *

from geometry_msgs.msg import Point32

import qsr
import random
import math
import json

from operator import itemgetter

class BBoxArray():
    """ Bounding box of an object with getter functions.
    """
    def __init__(self, bbox_t):

        bbox = list()
        for p in bbox_t.point:
            bbox.append([p.x,p.y,p.z])
            
        # Calc x_min and x_max for obj1
        x_sorted = sorted(bbox, key=itemgetter(0))
        self.x_min = x_sorted[0][0]
        self.x_max = x_sorted[7][0]

        # Calc y_min and y_max for obj
        y_sorted = sorted(bbox, key=itemgetter(1))
        self.y_min = y_sorted[0][1]
        self.y_max = y_sorted[7][1]

        # Calc z_min and z_max for obj
        z_sorted = sorted(bbox, key=itemgetter(2))
        self.z_min = z_sorted[0][2]
        self.z_max = z_sorted[7][2]
        
    def get_x_min(self):
        return self.x_min

    def get_x_max(self):
        return self.x_max

    def get_y_min(self):
        return self.y_min

    def get_y_max(self):
        return self.y_max

    
    def get_z_min(self):
        return self.z_min

    def get_z_max(self):
        return self.z_max


def gen_size_pred(obj, size):

    # fixed thresholds based on a k-means clustering with three clusters  
    if size >= 0.0372529395658:
        return [1.0, 'large', obj]
    elif size >= 0.00566770134577 and size < 0.0372529395658:
        return [1.0, 'medium', obj]
    else:
        return [1.0, 'small', obj]

def gen_pred(obj1, obj2, pred):

    return [1.0, pred, obj1, obj2]

class QSRGenerator():

    def __init__(self):
        rospy.init_node('qsr_generator')
        rospy.loginfo("Started QSR description service")

        self.service = rospy.Service('get_qsr_description', GetQSRDescription, self.handle_qsr_description)
        rospy.loginfo("Ready to generate scene descriptions")
        rospy.spin()
        rospy.loginfo("Stopped QSR description service")
    
    def handle_qsr_description(self,req):

        rospy.loginfo("Generate description")

        predicates = list()
        
        for obj1 in req.object_id:
            idx1 = req.object_id.index(obj1)

            # Calculate size of objects
            # Begin: calculating bbox volume
            bbox = BBoxArray(req.bbox[idx1])
            
            x_dim = bbox.get_x_max() - bbox.get_x_min()
            y_dim = bbox.get_y_max() - bbox.get_y_min()
            z_dim = bbox.get_z_max() - bbox.get_z_min()
            
            bbox_vol = x_dim * y_dim * z_dim
                
            predicates.append(gen_size_pred(obj1, bbox_vol))
            
            # End: calculating bbox volume
            
            for obj2 in req.object_id:
                if obj1 != obj2:
                    idx2 = req.object_id.index(obj2)
                    # print obj1, obj2
                    # Calculate spatial direction relations using the Ternary Point Calculus (TCP)
                    # Calculate distance relations
                    
                    cam_pos = [1.0,-2.0,1.698]

                    pos1 = [req.pose[idx1].position.x,  req.pose[idx1].position.y,  req.pose[idx1].position.z]
                    pos2 = [req.pose[idx2].position.x,  req.pose[idx1].position.y,  req.pose[idx1].position.z]
                    
                    qsrs = qsr.calc_QSR(cam_pos,pos1,pos2)           
                    for q in qsrs:
                        predicates.append(gen_pred(obj1,obj2,q))
                    
        res = GetQSRDescriptionResponse()
        res.predicates = json.dumps(predicates)
        rospy.loginfo("Waiting for next request...")

        return res

    

if __name__ == "__main__":
    qsr_gen = QSRGenerator()
    
