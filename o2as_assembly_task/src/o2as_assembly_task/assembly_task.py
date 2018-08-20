import rospy
from util import *
from std_srvs.srv import *
from subtask import SubTask
from o2as_vision.srv import FindObject, FindObjectRequest
# from o2as_vision.srv import FindObjects, FindObjectsRequest

FIND_OBJECT_SERVICE = "find_object"
# FIND_OBJECTS_SERVICE = "find_objects"

class PartsInfo(object):
    def __init__(self, name, id, type, desc):
        self.name = name
        self.id   = id
        self.type = type
        self.desc = desc

class AssemblyTask(object):
    def __init__(self):
        self.init_subtasks()
        self.init_parts_dict()

        # vision
        self._find_object = ros_service_proxy(FIND_OBJECT_SERVICE, FindObject)
        # self._find_objects = ros_service_proxy(FIND_OBJECTS_SERVICE, FindObjects)

    #############
    ### parts ###
    #############

    def init_parts_dict(self):
        self._parts_name_dict = dict()
        self._parts_name_dict["dummy"] = "07_SBARB6200ZZ-30"

        belt_drive_unit_parts = rospy.get_param("~belt_drive_unit_parts")
        for item in belt_drive_unit_parts:
            name = item.keys()[0]
            info = PartsInfo(name=name, id=item[name]['id'], type=item[name]['type'], desc=item[name]['description'])
            self._parts_name_dict[name] = info
            rospy.logdebug("parts name=%s, id=%s, type=%s, desc=%s", name, info.id, info.type, info.desc)
        
        # TODO: this dictionary should initialized using yaml file

    def get_parts_info(self, name):
        return self._parts_name_dict[name]

    ################
    ### subtasks ###
    ################

    def init_subtasks(self):
        self._subtasks = []
        self._subtasks = [ SubTask("dummy_subtask", self.dummy_subtask, self) ]

    def dummy_subtask(self, **kwargs):
        rospy.logdebug("AssemblyTask.dummy_subtask() begin")
        try:
            # this dummy subtask just find object and add it to planning scene for the moment
            rospy.logdebug("find parts")
            #parts_info = self.get_parts_info("motor_pulley")    #  5
            #parts_info = self.get_parts_info("bearing_housing") #  7
            parts_info = self.get_parts_info("output_pulley")   # 11
            #parts_info = self.get_parts_info("drive_shaft")   # 8
            req = FindObjectRequest()
            # req = FindObjectsRequest()
            req.object_id = str(parts_info.id)
            req.camera = "b_bot_camera"
            rospy.logdebug("find_object()) object_id=%s, type=%s", parts_info.id, parts_info.type)
            self._find_object(req.expected_position, req.position_tolerance, req.object_id, req.camera)
            # self._find_objects(req.expected_position, req.position_tolerance, req.object_id, req.camera)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

    ############
    ### main ###
    ############

    def assemble_belt_unit(self):
        # execute all subtasks
        for subtask in self._subtasks:
            subtask.run()
            # TODO: monitor execution states of subtask
            # TODO: judge execution result of subtask

    def run(self, product_count = 1):
        # assemble product
        for i in range(product_count):
            self.assemble_belt_unit()

        # loop
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
