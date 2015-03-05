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
    /// Add recognized objects and tables from ORK to the Moveit planning scene.
    /**
     * Table meshes will be generated from table contours.
     * Ork must produce table objects for this.
     * Tables are assumed to be conves, i.e., corner tables or similar will
     * be represented by their convex hull, thus closing gaps!
     *
     * Convex hulls of matched tables are optionally merged, otherwise replaced.
     * Thus tables can only grow.
     * When re-visiting a known location after navigation it might thus be advisable to disable
     * merge_tables for the first observation to counter any inaccuracies due to localization.
     */
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

            /// Class to sort Tables by their distance to a given table.
            struct DistanceToTable
            {
                public:
                    DistanceToTable(const moveit_msgs::CollisionObject & table, OrkToPlanningScene & otps);
                    bool operator()(const moveit_msgs::CollisionObject* lhs, const moveit_msgs::CollisionObject* rhs);
                protected:
                    const moveit_msgs::CollisionObject & table_;
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
                    bool add_tables, const std::string & table_prefix, bool merge_tables,
                    ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult & result);

            void fillResult(ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult & result,
                    const std::vector<moveit_msgs::CollisionObject> & collision_objects);

            /// Retrieves all CollisionObjects currently in the planning scene.
            std::vector<moveit_msgs::CollisionObject> getCollisionObjectsFromPlanningScene(bool & ok);

            /// Computes CollisionObjects from an ORK result. Names will not be unique, yet.
            std::vector<moveit_msgs::CollisionObject> getCollisionObjectsFromObjectRecognition(
                    const object_recognition_msgs::ObjectRecognitionResultConstPtr & objResult,
                    const std::string & table_prefix);

            /// Create a mesh from a list of points in a plane.
            shape_msgs::Mesh createMeshFromCountour(const std::vector<geometry_msgs::Point> & contours);

            /// Does this RecognizedObject fit our params for a table?
            bool isValidTable(const object_recognition_msgs::RecognizedObject & ro);

            /// Compute the transform to move contour vertices in old_table to new_table.
            tf::Pose computeTableContourTransform(const moveit_msgs::CollisionObject & old_table,
                    const moveit_msgs::CollisionObject & new_table);

            /// Merge two table object contours.
            moveit_msgs::CollisionObject merge_table_objects(const moveit_msgs::CollisionObject & old_table,
                    const moveit_msgs::CollisionObject & new_table);

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
                    double match_distance, double z_match_distance);

            /// Find tables in orObjects that are near psObject
            /**
             * \param [out] typeMatches near objects that have the same type
             */
            void findMatchingTables(const moveit_msgs::CollisionObject & psObject,
                    const std::set<const moveit_msgs::CollisionObject*> & orObjects,
                    std::vector<const moveit_msgs::CollisionObject*> & typeMatches,
                    double match_distance, double z_match_distance);

            /// Retrieve a pose from a CollisionObject
            static geometry_msgs::PoseStamped getPoseStamped(const moveit_msgs::CollisionObject & co);

            /// Compute the 2d distance and z distance between two poses
            std::pair<double, double> poseDistance(const geometry_msgs::PoseStamped & posePS,
                    const geometry_msgs::PoseStamped & poseOR);

            /// Compute 2d distance between tables.
            /// Only match to points from or_table if they are close enough in z.
            /// Just performs point-poly distance for or_table's corners!
            double tableDistance(const moveit_msgs::CollisionObject & ps_table,
                    const moveit_msgs::CollisionObject & or_table, double z_match_distance);

            /// Determine the largest taken id for each known object's symbolic name, e.g.
            /// cup_1, cup_3, tray_2 -> {cup: 3, tray: 2}
            void updateMaxObjectId(const moveit_msgs::CollisionObject & co,
                    std::map<std::string, unsigned int> & maxObjectId);

            bool isTable(const object_recognition_msgs::ObjectType & type);

        protected:
            // ROS Interface
            tf::TransformListener tf_;

            actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>  actionOrk_;
            ros::ServiceClient srvObjectInfo_;
            ros::ServiceClient srvPlanningScene_;
            ros::Publisher pubPlanningScene_;
            ros::Publisher pubMarker_;

            actionlib::SimpleActionServer<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction>
                actionOrkToPlanningScene_; 

            // Parameters
            double object_match_distance_;
            double table_match_distance_;
            double object_z_match_distance_;
            double table_z_match_distance_;
            double table_min_area_;
            double table_min_z_;
            double table_max_z_;
            double table_thickness_;
    };
}

#endif

