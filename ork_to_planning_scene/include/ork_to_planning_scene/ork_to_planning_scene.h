#ifndef ORK_TO_PLANNING_SCENE_H
#define ORK_TO_PLANNING_SCENE_H

#include <ros/ros.h>
#include <vector>
#include <map>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

#include <object_recognition_msgs/ObjectInformation.h>
#include <object_recognition_msgs/GetObjectInformation.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#include <moveit_msgs/GetPlanningScene.h>
#include "ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h"

namespace ork_to_planning_scene
{
    class OrkToPlanningScene
    {
        public:
            /// Class to sort CollisionObjects by their distance to a given pose.
            struct DistanceToPose
            {
                public:
                    DistanceToPose(const geometry_msgs::PoseStamped & pose, OrkToPlanningScene & otps);
                    bool operator()(const moveit_msgs::CollisionObject* lhs, const moveit_msgs::CollisionObject* rhs);
                protected:
                    geometry_msgs::PoseStamped pose_;
                    OrkToPlanningScene & otps_;
            };

        public:
            OrkToPlanningScene();

        protected:
            /// Main action callback that we provide.
            void orkToPlanningSceneCallback(
                    const ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoalConstPtr & goal);

            /// Removes all objects that are not expected to be detected again (e.g. because out of FOV).
            void removeNotExpectedToBeDetected(std::vector<moveit_msgs::CollisionObject> & objects,
                    const std::vector<std::string> & expected_objects);

            /// Verify that the current planning scene has these objects
            bool verifyPlanningScene(const std::vector<moveit_msgs::CollisionObject> & objects);

            /// Handle an ORK result and update planning scene accordingly.
            bool processObjectRecognition(const object_recognition_msgs::ObjectRecognitionResultConstPtr & objResult,
                    const std::vector<std::string> & expected_objects, bool verify,
                    bool add_tables, const std::string & table_prefix);

            /// Retrieves all CollisionObjects currently in the planning scene.
            std::vector<moveit_msgs::CollisionObject> getCollisionObjectsFromPlanningScene();

            /// Computes CollisionObjects from an ORK result. Names will not be unique, yet.
            std::vector<moveit_msgs::CollisionObject> getCollisionObjectsFromObjectRecognition(
                    const object_recognition_msgs::ObjectRecognitionResultConstPtr & objResult,
                    const std::string & table_prefix);

            /// Create a mesh from a list of points in a plane.
            shape_msgs::Mesh createMeshFromCountour(const std::vector<geometry_msgs::Point> & contours);

            /// Does this RecognizedObject fit our params for a table?
            bool isValidTable(const object_recognition_msgs::RecognizedObject & ro);

            /// Convert an ORK object to a CollisionObject as good as possible.
            bool collisionObjectFromRecognizedObject(const object_recognition_msgs::RecognizedObject & ro,
                    moveit_msgs::CollisionObject & co, const std::string & table_prefix);

            /// Determine which recognized objects match which in the planning scene
            /**
             * \param [in] planningSceneObjects the objects currently in the planning scene
             * \param [in] objectRecognitionObjects the recognized objects
             * \param [out] planningSceneUndetectedObjects objects in the planning scene not near any other recognized
             * \param [out] objectRecognitionNewObjects newly detected object that weren't there before
             * \param [out] planningSceneReplacedObjects old objects to be removed as they are replaced by a new one
             * \param [out] matchedObjects pairs of old/new objects that are now at a different pose
             * \param [in] handleTables if true only table objects will be processed, if false no table objects
             */
            void determineObjectMatches(const std::vector<moveit_msgs::CollisionObject> & planningSceneObjects,
                    const std::vector<moveit_msgs::CollisionObject> & objectRecognitionObjects,
                    std::vector<moveit_msgs::CollisionObject> & planningSceneUndetectedObjects,
                    std::vector<moveit_msgs::CollisionObject> & objectRecognitionNewObjects,
                    std::vector<moveit_msgs::CollisionObject> & planningSceneReplacedObjects,
                    std::vector<std::pair<moveit_msgs::CollisionObject, moveit_msgs::CollisionObject> > &
                    matchedObjects,
                    bool handleTables);

            /// Find objects in orObjects that are near psObject
            /**
             * \param [out] typeMatches near objects that have the same type
             * \param [out] otherTypeMatches near objects that have a different type
             */
            void findMatchingObjects(const moveit_msgs::CollisionObject & psObject,
                    const std::set<const moveit_msgs::CollisionObject*> & orObjects,
                    std::vector<const moveit_msgs::CollisionObject*> & typeMatches,
                    std::vector<const moveit_msgs::CollisionObject*> & otherTypeMatches,
                    double match_distance);

            /// Retrieve a pose from a CollisionObject
            static geometry_msgs::PoseStamped getPoseStamped(const moveit_msgs::CollisionObject & co);

            /// Compute the 2d distance between two poses
            double poseDistance(const geometry_msgs::PoseStamped & posePS,
                    const geometry_msgs::PoseStamped & poseOR);

            /// Determine the largest taken id for each known object's symbolic name, e.g.
            /// cup_1, cup_3, tray_2 -> {cup: 3, tray: 2}
            void updateMaxObjectId(const moveit_msgs::CollisionObject & co,
                    std::map<std::string, unsigned int> & maxObjectId);

        protected:
            // ROS Interface
            tf::TransformListener tf_;

            actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>  actionOrk_;
            ros::ServiceClient srvObjectInfo_;
            ros::ServiceClient srvPlanningScene_;
            ros::Publisher pubPlanningScene_;

            actionlib::SimpleActionServer<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction>
                actionOrkToPlanningScene_; 

            // Parameters
            double object_match_distance_;
            double table_match_distance_;
            double table_min_area_;
            double table_min_z_;
            double table_max_z_;
            double table_thickness_;
    };
}

#endif
