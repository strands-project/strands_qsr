from operator import itemgetter


### SIM
#object_types = ["Monitor",
                #"Cup",
                #"Keyboard",
                #"Pencil",
                #"Telephone",
                #"PC",
                #"Mouse",
                #"Lamp",
                #"Calculator",
                #"Headphone",
                #"Laptop",
                #"MobilePhone",
                #"Glass",
                #"Stapler",
                #"Keys",
                #"Book",
                #"Bottle"
    #]

##REAL
object_types = ["Monitor",
                "Cup",
                "Mug", #
                "Papers", #
                "PenStand", #
                "Jug", #
                "Headphones", #
                "Highlighter", #
                "Marker", #
                "Notebook", #
                "Mobile", #
                "Folder", #
                "Flask", #
                "Pen", #
                "Keyboard",
                "Pencil",
                "Telephone",
                "PC",
                "Mouse",
                "Lamp",
                "Calculator",
                "Headphone",
                "Laptop",
                "MobilePhone",
                "Glass",
                "Stapler",
                "Keys",
                "Book",
                "Bottle",
                "UNKNOWN"
    ]

#object_types = [u'Mobile', u'Monitor',  u'Flask', u'Laptop', u'Papers', u'Glass', u'Mug', u'Book', u'Bottle', u'Keyboard', u'Mouse']


class BBoxArray():
    """ Bounding box of an object with getter functions.
    """
    def __init__(self, bbox):
        self.points = bbox
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
    
    def get_size(self):
        return (self.get_x_max() - self.get_x_min(),
                self.get_y_max() - self.get_y_min(),
                self.get_z_max() - self.get_z_min() )
    
    def get_volume(self):
        return reduce(lambda x, y: x*y, self.get_size())
                



class Object(object):
    def __init__(self, name, obj_type, position, orientation, bbox):
        if obj_type not in object_types:
            raise Exception("Trying to create an object of non-existing type ({})".format(obj_type))
        self.name = name
        self.obj_type = obj_type
        self.position = position
        self.orientation =  orientation
        self.bbox = BBoxArray(bbox)
        
class GeometricState(object):
    def __init__(self, scene_id):
        self._objects =  {}
        self._scene_id = scene_id
        self._frame_id = "/table"
        self._stamp = None
        
    def add_object(self, name, obj_type, position, orientation, bbox):
        if name in self._objects.keys():
            raise Exception("Trying to add a duplicate object to a geometric state.")
        ob = Object(name, obj_type, position, orientation, bbox )
        self._objects[name] = ob
        
    def number_of_objects(self):
        return len(self._objects.keys())
    
    def get_object_names(self):
        return self._objects.keys()
    
    def get_object(self, obj):
        return self._objects[obj]
    
    def get_objects(self):
        return self._objects
    
    def get_object_type(self, obj):
        return self._objects[obj].obj_type
    
    @classmethod
    def from_scene_data(cls, scene_dict):
        gs = GeometricState(scene_dict["scene_id"])
        for i in scene_dict["objects"]:
            if scene_dict["type"][i] != "UNKNOWN":
                gs.add_object(i, scene_dict["type"][i],
                              scene_dict['position'][i],
                              scene_dict['orientation'][i],
                              scene_dict['bbox'][i])
        return gs

import rospy
import visualization_msgs.msg as vmsg
import geometry_msgs.msg as gmsg

class GeometricStateViz(object):
    def __init__(self, geometric_state):
        isinstance(geometric_state, GeometricState)
        self._geometric_state = geometric_state
        
        self._topic = rospy.Publisher("/geometric_state", vmsg.MarkerArray, latch=True)
        
    def publish_visualisation(self):
        ma = vmsg.MarkerArray()
        for n in range(50):
            m = vmsg.Marker()
            m.action = m.DELETE
            m.type == m.LINE_LIST
            m.id = n
            m.header.frame_id = "/table"
            m.ns = "geometric_state_bbox"
            ma.markers.append(m)
            m = vmsg.Marker()
            m.action = m.DELETE
            m.id = n
            m.ns = "geometric_state_labels"
            m.header.frame_id = "/table"
            ma.markers.append(m)
        self._topic.publish(ma)
        
        ma =  vmsg.MarkerArray()
        cols = [(1, 0, 0), (0, 1, 1), (1, 1, 0), (0, 1, 0), (1, 0, 1), (0, 0, 1)] 
        for n, obj in enumerate(self._geometric_state.get_objects().values()):
            marker =  vmsg.Marker()
            marker.header.frame_id = "/table"
            marker.ns = "geometric_state_bbox"
            marker.action = vmsg.Marker.ADD
            marker.id = n
            marker.type = vmsg.Marker.LINE_LIST
            marker.scale.x = 0.003 #.3cm width lines
            marker.color.a = 1
            marker.color.r,  marker.color.g, marker.color.b = cols[n%6]
            for pt in obj.bbox.points:
                for pt2 in obj.bbox.points:
                    if pt == pt2:
                        continue
                    point =  gmsg.Point()
                    point.x, point.y, point.z = pt
                    marker.points.append(point)
                    point =  gmsg.Point()
                    point.x, point.y, point.z = pt2
                    marker.points.append(point)
            text = vmsg.Marker()
            text.header.frame_id = "/table"
            text.type = text.TEXT_VIEW_FACING
            text.action = text.ADD
            text.scale.z = 0.1
            text.ns = "geometric_state_labels"
            text.id = n
            text.color.a = 1
            text.color.b =  text.color.r = text.color.g = 1
            text.pose.position.x = point.x
            text.pose.position.y = point.y
            text.pose.position.z = point.z
            text.points.append(point)
            text.text = self._geometric_state.get_object_type(obj.name)
            ma.markers.append(marker)
            ma.markers.append(text)
        
        self._topic.publish(ma)