# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Define an actionlib server for ORK
"""
from __future__ import print_function
from object_recognition_core.pipelines.plasm import create_plasm
from object_recognition_msgs.msg import RecognizedObjectArray
from object_recognition_msgs.msg._ObjectRecognitionAction import ObjectRecognitionAction
from object_recognition_msgs.msg._ObjectRecognitionResult import ObjectRecognitionResult
from object_recognition_msgs.msg import TableArray
from object_recognition_msgs.msg import RecognizedObject
import actionlib
import rospy
import sys

DEFAULT_NODE_NAME = "object_recognition_server"

class RecognitionTabletopServer:
    """
    Main server that reads a config file, builds an actionlib server, reads an ecto plasm and run it when
    the actionlib server is queried.
    This server will also interpret tables from tabletop as recognized objects.
    Therefore it only works with tabletop segmentation (or anything that sends a table_array).
    """
    def __init__(self, ork_params):
        # create the plasm that will run the detection
        self.plasm = create_plasm(ork_params)
        self.plasm.configure_all()
        rospy.loginfo('ORK Tabletop server configured')

        # the results or the object recognition pipeline
        self.recognition_result = None
        self.table_result = None

        topics = ['recognized_object_array']

        for sink in ork_params.values():
            if 'recognized_object_array_topic' in sink:
                topics.append(sink['recognized_object_array_topic'])

        # subscribe to the output of the detection pipeline
        for topic in topics:
            rospy.Subscriber(topic, RecognizedObjectArray, self.callback_recognized_object_array)
            rospy.loginfo('Subscribed to the ' + topic + ' topic.')

        rospy.Subscriber("table_array", TableArray, self.callback_table_array)
        rospy.loginfo('Subscribed to the table_array topic.')

        # look for a cell that contains a cropper to select the ROI
        self.cropper = None
        for cell in self.plasm.cells():
            if 'crop_enabled' in cell.params:
                self.cropper = cell

        # actionlib stuff
        self.server = actionlib.SimpleActionServer('recognize_objects', ObjectRecognitionAction, self.execute, False)
        self.server.start()
        rospy.loginfo('ORK Tabletop server started')

    def callback_recognized_object_array(self, data):
        self.recognition_result = data

    def callback_table_array(self, data):
        self.table_result = data

    def compute_table_objects(self, tables):
        table_objects = []
        rospy.loginfo("Processing %d tables" % len(tables.tables))
        for table in tables.tables:
            table_obj = RecognizedObject()
            table_obj.header = table.header
            table_obj.type.key = "table"
            table_obj.type.db = "Tabletop"
            table_obj.confidence = 0.8  # does not exist
            table_obj.pose.header = table.header
            table_obj.pose.pose.pose = table.pose
            table_obj.pose.pose.covariance = [0.0] * 36     # undefined
            table_obj.bounding_contours = table.convex_hull
            table_objects.append(table_obj)
        return table_objects

    def execute_plasm_and_wait(self):
        self.recognition_result = None
        self.table_result = None
        self.plasm.execute(niter=1)
        # the pipeline should have published, wait for the results.
        max_tries = 5
        while self.recognition_result is None or self.table_result is None:
            not_there = " recognition_result" if self.recognition_result is None else ""
            not_there += " table_result" if self.table_result is None else ""
            rospy.loginfo('ORK results: waiting for' + not_there)
            rospy.sleep(0.1)
            max_tries -= 1
            if max_tries <= 0:
                break


    def execute(self, goal):
        if self.cropper is not None:
            self.cropper.params.crop_enabled = goal.use_roi
            if goal.use_roi:
                if len(goal.filter_limits) == 6:
                    self.cropper.params.x_min = goal.filter_limits[0]
                    self.cropper.params.x_max = goal.filter_limits[1]
                    self.cropper.params.y_min = goal.filter_limits[2]
                    self.cropper.params.y_max = goal.filter_limits[3]
                    self.cropper.params.z_min = goal.filter_limits[4]
                    self.cropper.params.z_max = goal.filter_limits[5]
                else:
                    print('WARNING: goal.use_roi is enabled but filter_limits doesn\'t have size 6 [x_min, x_max, y_min, y_max, z_min, z_max]. Roi disabled.', file=sys.stderr)
                    self.cropper.params.crop_enabled = False

        # Do lots of awesome groundbreaking robot stuff here
        result = ObjectRecognitionResult()
        self.execute_plasm_and_wait()

        if self.recognition_result is None or self.table_result is None:
            not_there = " recognition_result" if self.recognition_result is None else ""
            not_there += " table_result" if self.table_result is None else ""
            rospy.logwarn("Missing" + not_there + " - retrying once")
            self.execute_plasm_and_wait()

        if self.recognition_result is None or self.table_result is None:
            not_there = " recognition_result" if self.recognition_result is None else ""
            not_there += " table_result" if self.table_result is None else ""
            rospy.logerr("Still no" + not_there + " - failing")
            self.server.set_aborted()
        else:
            result.recognized_objects = self.recognition_result
            result.recognized_objects.objects.extend(self.compute_table_objects(self.table_result))
            # FIXME update cooccurrence - isn't currently set anyways

            rospy.loginfo('ORK results: received')

            # we have a result!
            self.server.set_succeeded(result=result)

        # reset our instance variable for the next round
        self.recognition_result = None
        self.table_result = None
