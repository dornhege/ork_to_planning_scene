#include "ork_to_planning_scene/ork_to_planning_scene.h"
#include <moveit/move_group/capability_names.h>
#include "symbolic_name_tools/symbolic_name_tools.h"
#include <sstream>
#include <map>
#include <set>
#include <utility>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <opencv/cv.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

// Later TODO
// - Merge multiple tables into one when overlaps with N known tables?
// - isValidTable: Maybe at least one point should be in reach of robot? (maxDistOfTable or similar)

namespace ork_to_planning_scene
{

OrkToPlanningScene::OrkToPlanningScene() :
    actionOrk_("recognize_objects", true),
    actionOrkToPlanningScene_(ros::NodeHandle(), "ork_to_planning_scene",
            boost::bind(&OrkToPlanningScene::orkToPlanningSceneCallback, this, _1), false)
{
    ros::NodeHandle nh;
    tf_ = new tf::TransformListener(ros::Duration(60.0));

    srvObjectInfo_ = nh.serviceClient<object_recognition_msgs::GetObjectInformation>("get_object_info");
    srvPlanningScene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    pubPlanningScene_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    pubMarker_ = nh.advertise<visualization_msgs::MarkerArray>("ork_to_planning_scene_markers", 5);

    ROS_INFO("Waiting for recognize_objects action server to start.");
    actionOrk_.waitForServer();

    ROS_INFO("Waiting for get_object_info service.");
    srvObjectInfo_.waitForExistence();

    ROS_INFO("Waiting for %s service.", move_group::GET_PLANNING_SCENE_SERVICE_NAME.c_str());
    srvPlanningScene_.waitForExistence();

    ROS_INFO("Waiting for planning_scene publisher to have subscribers.");
    while(pubPlanningScene_.getNumSubscribers() < 1) {
        ros::Duration(0.5).sleep();
    }

    ros::NodeHandle nhPriv("~");
    nhPriv.param("object_match_distance", object_match_distance_, 0.15);
    nhPriv.param("table_match_distance", table_match_distance_, 0.1);
    nhPriv.param("object_z_match_distance", object_z_match_distance_, 0.15);
    nhPriv.param("table_z_match_distance", table_z_match_distance_, 0.1);
    nhPriv.param("table_min_area", table_min_area_, 0.16);
    nhPriv.param("table_min_z", table_min_z_, 0.3);
    nhPriv.param("table_max_z", table_max_z_, 1.1);
    nhPriv.param("table_thickness", table_thickness_, 0.075);

    ROS_INFO("Actions and Services available - spinning up ork_to_planning_scene action.");
    actionOrkToPlanningScene_.start();
    ROS_INFO("Ready!");
}


OrkToPlanningScene::DistanceToPose::DistanceToPose(
        const geometry_msgs::PoseStamped & pose, OrkToPlanningScene & otps) : pose_(pose), otps_(otps)
{
}

bool OrkToPlanningScene::DistanceToPose::operator()(
        const moveit_msgs::CollisionObject* lhs, const moveit_msgs::CollisionObject* rhs) 
{
    double dLhs = otps_.poseDistance(pose_, OrkToPlanningScene::getPoseStamped(*lhs)).first;
    double dRhs = otps_.poseDistance(pose_, OrkToPlanningScene::getPoseStamped(*rhs)).first;
    return dLhs < dRhs;
}

OrkToPlanningScene::DistanceToTable::DistanceToTable(
        const moveit_msgs::CollisionObject & table, OrkToPlanningScene & otps) : table_(table), otps_(otps)
{
}

bool OrkToPlanningScene::DistanceToTable::operator()(
        const moveit_msgs::CollisionObject* lhs, const moveit_msgs::CollisionObject* rhs) 
{
    double dLhs = otps_.tableDistance(table_, *lhs, HUGE_VAL);
    double dRhs = otps_.tableDistance(table_, *rhs, HUGE_VAL);
    return dLhs < dRhs;
}

void OrkToPlanningScene::orkToPlanningSceneCallback(
        const ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoalConstPtr & goal)
{
    ROS_INFO("Sending ObjectRecognition request.");
    object_recognition_msgs::ObjectRecognitionGoal orkGoal;
    orkGoal.use_roi = false;
    actionOrk_.sendGoal(orkGoal);

    bool finished_before_timeout = actionOrk_.waitForResult(ros::Duration(300.0));
    if(finished_before_timeout) {
        actionlib::SimpleClientGoalState state = actionOrk_.getState();
        ROS_INFO("ORK Action finished: %s", state.toString().c_str());
        if(state != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_ERROR("ORK action failed.");
            actionOrkToPlanningScene_.setAborted();
            return;
        }
        std::string table_prefix = goal->table_prefix;
        if(table_prefix.empty())
            table_prefix = "table";

        ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult result;
        bool updateOK = processObjectRecognition(actionOrk_.getResult(),
                goal->expected_objects, goal->verify_planning_scene_update,
                goal->add_tables, table_prefix, goal->merge_tables, result);
        if(!updateOK) {
            ROS_ERROR("processObjectRecognition failed - probably due to planning scene update not verified or moveit not running.");
            actionOrkToPlanningScene_.setAborted();
        } else {
            actionOrkToPlanningScene_.setSucceeded(result);
        }
    } else {
        ROS_ERROR("ObjectRecognition Action did not finish before the time out.");
        actionOrkToPlanningScene_.setAborted();
    }
}

class NotInSet {
    public:
        NotInSet(const std::set<std::string> & data) : data_(data) {}
        bool operator()(const moveit_msgs::CollisionObject & co) {
            return data_.find(co.id) == data_.end();
        }
    protected:
        const std::set<std::string> & data_;
};

void OrkToPlanningScene::removeNotExpectedToBeDetected(std::vector<moveit_msgs::CollisionObject> & objects,
        const std::vector<std::string> & expected_objects)
{
    // objects are undetected and would be removed
    // we only remove the expected_objects
    // So, here remove any object from objects that is not in expected_objects
    std::set<std::string> expected_objects_names(expected_objects.begin(), expected_objects.end());
    objects.erase(std::remove_if(objects.begin(), objects.end(), NotInSet(expected_objects_names)),
            objects.end());
}

bool OrkToPlanningScene::verifyPlanningScene(const std::vector<moveit_msgs::CollisionObject> & objects)
{
    // TODO implement - then: Need to decide how long we wait and how often we reget the planning scene...
    // Might be costly to verify in a loop
    ROS_INFO("Waiting 2s to make sure planning scene is updated");
    ROS_WARN("This is NOT an actual verification - that is not implemented, yet.");
    ros::Duration(2.0).sleep();

    return true;
}

bool OrkToPlanningScene::processObjectRecognition(
        const object_recognition_msgs::ObjectRecognitionResultConstPtr & objResult,
        const std::vector<std::string> & expected_objects, bool verify,
        bool add_tables, const std::string & table_prefix, bool merge_tables,
        ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult & result)
{
    bool ok;
    std::pair< std::vector<moveit_msgs::CollisionObject>, std::vector<moveit_msgs::CollisionObject> >
        psAllObjects = getCollisionObjectsFromPlanningScene(ok);
    const std::vector<moveit_msgs::CollisionObject> & psObjects = psAllObjects.first;
    // we care about the attached ones only to not duplicate their id for new ones
    const std::vector<moveit_msgs::CollisionObject> & psAttObjects = psAllObjects.second;
    if(!ok)
        return false;
    std::vector<moveit_msgs::CollisionObject> orObjects = getCollisionObjectsFromObjectRecognition(
            objResult, table_prefix);

    std::vector<moveit_msgs::CollisionObject> planningSceneUndetectedObjects;
    std::vector<moveit_msgs::CollisionObject> objectRecognitionNewObjects;
    std::vector<moveit_msgs::CollisionObject> planningSceneReplacedObjects;
    std::vector<std::pair<moveit_msgs::CollisionObject, moveit_msgs::CollisionObject> > matchedObjects;

    // process tables independent from other objects
    if(add_tables) {
        determineObjectMatches(psObjects, orObjects, planningSceneUndetectedObjects, objectRecognitionNewObjects,
                planningSceneReplacedObjects, matchedObjects, true);
    }
    determineObjectMatches(psObjects, orObjects, planningSceneUndetectedObjects, objectRecognitionNewObjects,
            planningSceneReplacedObjects, matchedObjects, false);

    // new ones need new unique names (either unique forever or reuse whats not in planning scene?)
    // forever is nicer, but not in PS is stateless - for now stateless
    std::map<std::string, unsigned int> maxObjectId;
    for(std::vector<std::pair<moveit_msgs::CollisionObject, moveit_msgs::CollisionObject> >::const_iterator it =
            matchedObjects.begin(); it != matchedObjects.end(); ++it) {
        updateMaxObjectId(it->first, maxObjectId);
    }
    // if we remove one done reassigning that name immediately
    forEach(const moveit_msgs::CollisionObject & co, planningSceneUndetectedObjects)
        updateMaxObjectId(co, maxObjectId);
    forEach(const moveit_msgs::CollisionObject & co, planningSceneReplacedObjects)
        updateMaxObjectId(co, maxObjectId);
    forEach(const moveit_msgs::CollisionObject & co, psAttObjects)
        updateMaxObjectId(co, maxObjectId);

    // assign new unique names
    forEach(moveit_msgs::CollisionObject & co, objectRecognitionNewObjects) {
        unsigned int & curId = maxObjectId[co.id];
        curId++;
        co.id = symbolic_name_tools::create_name(co.id, curId);
    }

    // finally make the setup PS for moveing matched ones, adding new, optionally removing undetected
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;  // this is fine, we don't leave anything unspecified that we don't want

    // set colors
    // for some reason a diff replaces colors, so re-set the colors for known objects somewhere else
    // Do these now before we remove the unexpected ones
    // The extra colors for then-removed objects shouldnt hurt
    forEach(const moveit_msgs::CollisionObject & co, planningSceneUndetectedObjects) {
        moveit_msgs::ObjectColor oc;
        oc.id = co.id;
        if(isTable(co.type)) {
            oc.color.r = 0.67;
            oc.color.g = 0.33;
            oc.color.b = 0.0;
            oc.color.a = 1.0;
        } else {
            oc.color.r = 0.0;
            oc.color.g = 1.0;
            oc.color.b = 0.0;
            oc.color.a = 1.0;
        }
        planning_scene.object_colors.push_back(oc);
    }

    removeNotExpectedToBeDetected(planningSceneUndetectedObjects, expected_objects);
    forEach(moveit_msgs::CollisionObject & co, planningSceneUndetectedObjects) {
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        planning_scene.world.collision_objects.push_back(co);
        ROS_INFO("Removing undetected object: %s", co.id.c_str());
    }
    forEach(moveit_msgs::CollisionObject & co, objectRecognitionNewObjects) {
        co.operation = moveit_msgs::CollisionObject::ADD;
        planning_scene.world.collision_objects.push_back(co);
        ROS_INFO("Adding new object: %s", co.id.c_str());
    }
    forEach(moveit_msgs::CollisionObject & co, planningSceneReplacedObjects) {
        co.operation = moveit_msgs::CollisionObject::REMOVE;
        planning_scene.world.collision_objects.push_back(co);
        ROS_INFO("Removing replaced object: %s", co.id.c_str());
    }
    for(std::vector<std::pair<moveit_msgs::CollisionObject, moveit_msgs::CollisionObject> >::const_iterator it =
            matchedObjects.begin(); it != matchedObjects.end(); ++it) {
        moveit_msgs::CollisionObject co = it->second;
        co.id = it->first.id;
        if(isTable(co.type)) {
            // matched tables should get replaced geoemtry
            co.operation = moveit_msgs::CollisionObject::MOVE;
            if(merge_tables) {  // merge new table in co with old table in it->ifrst
                co = merge_table_objects(it->first, co);
            }
        } else {
            co.operation = moveit_msgs::CollisionObject::MOVE;
            // delete geometry, we only want to move this.
            // These are type matches, i.e. the geometry is the same.
            co.primitives.clear();
            co.meshes.clear();
            co.planes.clear();
        }
        planning_scene.world.collision_objects.push_back(co);
        ROS_INFO("Updating matched object: %s", co.id.c_str());
    }
    // set colors for handled objects
    forEach(const moveit_msgs::CollisionObject & co, planning_scene.world.collision_objects) {
        if(co.operation != moveit_msgs::CollisionObject::REMOVE) {
            moveit_msgs::ObjectColor oc;
            oc.id = co.id;
            if(isTable(co.type)) {
                oc.color.r = 0.67;
                oc.color.g = 0.33;
                oc.color.b = 0.0;
                oc.color.a = 1.0;
            } else {
                oc.color.r = 0.0;
                oc.color.g = 1.0;
                oc.color.b = 0.0;
                oc.color.a = 1.0;
            }
            planning_scene.object_colors.push_back(oc);
        }
    }

    fillResult(result, planning_scene.world.collision_objects);

    // slight hack: Tables must be ADD for an update as geometry has changed
    // Was only set to MOVE before to differentiate that in fillResult
    forEach(moveit_msgs::CollisionObject & co, planning_scene.world.collision_objects) {
        if(isTable(co.type) && co.operation == moveit_msgs::CollisionObject::MOVE)
            co.operation = moveit_msgs::CollisionObject::ADD;
    }

    pubPlanningScene_.publish(planning_scene);
    if(verify) {
        return verifyPlanningScene(planning_scene.world.collision_objects);
    } else {
        return true;
    }
}

void OrkToPlanningScene::fillResult(ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkResult & result,
        const std::vector<moveit_msgs::CollisionObject> & collision_objects)
{
    forEach(const moveit_msgs::CollisionObject & co, collision_objects) {
        if(co.operation == moveit_msgs::CollisionObject::ADD) {
            if(isTable(co.type))
                result.added_tables.push_back(co.id);
            else
                result.added_objects.push_back(co.id);
        } else if(co.operation == moveit_msgs::CollisionObject::REMOVE) {
            if(isTable(co.type))
                result.removed_tables.push_back(co.id);
            else
                result.removed_objects.push_back(co.id);
        } else if(co.operation == moveit_msgs::CollisionObject::MOVE) {
            if(isTable(co.type))
                result.updated_tables.push_back(co.id);
            else
                result.moved_objects.push_back(co.id);
        }
    }
}

std::pair<std::vector<moveit_msgs::CollisionObject>, std::vector<moveit_msgs::CollisionObject> >
    OrkToPlanningScene::getCollisionObjectsFromPlanningScene(bool & ok)
{
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    if (!srvPlanningScene_.call(request, response)) {
        ok = false;
        ROS_ERROR("%s: planning scene request failed.", __func__);
        return std::make_pair(std::vector<moveit_msgs::CollisionObject>(),
                std::vector<moveit_msgs::CollisionObject>());
    }
    ok = true;
    std::vector<moveit_msgs::CollisionObject> attObjs;
    forEach(const moveit_msgs::AttachedCollisionObject & aco,
            response.scene.robot_state.attached_collision_objects) {
        attObjs.push_back(aco.object);
    }
    return std::make_pair(response.scene.world.collision_objects, attObjs);
}

std::vector<moveit_msgs::CollisionObject> OrkToPlanningScene::getCollisionObjectsFromObjectRecognition(
        const object_recognition_msgs::ObjectRecognitionResultConstPtr & objResult,
        const std::string & table_prefix)
{
    std::vector<moveit_msgs::CollisionObject> cos;
    forEach(const object_recognition_msgs::RecognizedObject & ro, objResult->recognized_objects.objects) {
        moveit_msgs::CollisionObject co;
        bool objOK = collisionObjectFromRecognizedObject(ro, co, table_prefix);
        if(!objOK) {
            if(isTable(ro.type)) {
                ROS_WARN("Filtered a table object as it is too small, not in the required z range, or not vertical.");
            } else {
                ROS_WARN_STREAM("Could not convert object with type: " << ro.type << " to CollisionObject.");
            }
            continue;
        }
        cos.push_back(co);
    }
    return cos;
}

shape_msgs::Mesh OrkToPlanningScene::createMeshFromCountour(const std::vector<geometry_msgs::Point> & contours,
        double z_offset)
{
    shape_msgs::Mesh mesh;
    forEach(const geometry_msgs::Point & pt, contours) {
        mesh.vertices.push_back(pt);
        mesh.vertices.back().z += z_offset;
    }
    for(int i = 2; i < mesh.vertices.size(); ++i) {
        // poor man's triangulation: Fan pattern.
        shape_msgs::MeshTriangle tri;
        tri.vertex_indices[0] = 0;
        tri.vertex_indices[1] = i;
        tri.vertex_indices[2] = i - 1;
        mesh.triangles.push_back(tri);
    }

    // Add the under side of the table
    for(int i = 0; i < contours.size(); ++i) {
        geometry_msgs::Point pt = contours[i];
        pt.z -= table_thickness_;
        pt.z += z_offset;
        mesh.vertices.push_back(pt);
    }

    // underside tris are the same as top, but with under side verts + inverted vertex order
    for(int i = contours.size() + 2; i < mesh.vertices.size(); ++i) {
        // poor man's triangulation: Star pattern.
        shape_msgs::MeshTriangle tri;
        tri.vertex_indices[0] = contours.size();
        tri.vertex_indices[1] = i - 1;
        tri.vertex_indices[2] = i;
        mesh.triangles.push_back(tri);
    }

    // create sides
    for(int i = 0; i < contours.size(); ++i) {
        int first_top = i;
        int next_top = (first_top + 1) % contours.size();
        int first_bottom = first_top + contours.size();
        int next_bottom = next_top + contours.size();
        // make a quad of these
        shape_msgs::MeshTriangle tri_top;
        tri_top.vertex_indices[0] = first_top;
        tri_top.vertex_indices[1] = next_top;
        tri_top.vertex_indices[2] = first_bottom;
        mesh.triangles.push_back(tri_top);
        shape_msgs::MeshTriangle tri_bottom;
        tri_bottom.vertex_indices[0] = first_bottom;
        tri_bottom.vertex_indices[1] = next_top;
        tri_bottom.vertex_indices[2] = next_bottom;
        mesh.triangles.push_back(tri_bottom);
    }

    return mesh;
}

tf::Pose OrkToPlanningScene::computeTableContourTransform(const moveit_msgs::CollisionObject & old_table,
        const moveit_msgs::CollisionObject & new_table)
{
    ROS_ASSERT(old_table.meshes.size() == old_table.mesh_poses.size());
    ROS_ASSERT(old_table.meshes.size() == 1);
    ROS_ASSERT(new_table.meshes.size() == new_table.mesh_poses.size());
    ROS_ASSERT(new_table.meshes.size() == 1);

    // need to transform old points into new tables pose
    tf::StampedTransform oldToNew;
    tf_->waitForTransform(new_table.header.frame_id, old_table.header.frame_id, new_table.header.stamp,
            ros::Duration(0.5));
    tf_->lookupTransform(new_table.header.frame_id, old_table.header.frame_id, new_table.header.stamp,
            oldToNew);

    tf::Pose oldPose;
    tf::Pose newPose;
    tf::poseMsgToTF(old_table.mesh_poses[0], oldPose);
    tf::poseMsgToTF(new_table.mesh_poses[0], newPose);
    // oldPose is in old_table frame_id
    // newPose is in new_table frame_id
    // we need a tf from a pt at oldPose in old_table frame to a pt at newPose in new_table frame
    tf::Pose oldToNewPt = newPose.inverseTimes(oldToNew) * oldPose;
    return oldToNewPt;
}

moveit_msgs::CollisionObject OrkToPlanningScene::merge_table_objects(const moveit_msgs::CollisionObject & old_table,
        const moveit_msgs::CollisionObject & new_table)
{
    moveit_msgs::CollisionObject ret = new_table;
    tf::Pose oldToNewPt;
    try {
        oldToNewPt = computeTableContourTransform(old_table, new_table);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return ret;
    }

    std::vector<cv::Point2f> points;
    // we should have created this from a contour, so the first half of the vertices
    // should be the original contour
    for(unsigned int i = 0; i < old_table.meshes[0].vertices.size()/2; ++i) {
        const geometry_msgs::Point & pt = old_table.meshes[0].vertices[i];
        tf::Vector3 tfPt;
        tf::pointMsgToTF(pt, tfPt);

        tf::Vector3 oldPtInNew = oldToNewPt * tfPt;

        if(fabs(oldPtInNew.z()) > 0.05) {
            ROS_ERROR("%s: Old Pt had non-zero z: (%f %f %f)", __func__,
                    oldPtInNew.x(), oldPtInNew.y(), oldPtInNew.z());
            continue;
        }
        points.push_back(cv::Point2f(oldPtInNew.x(), oldPtInNew.y()));
    }

    for(unsigned int i = 0; i < new_table.meshes[0].vertices.size()/2; ++i) {
        const geometry_msgs::Point & pt = new_table.meshes[0].vertices[i];
        if(fabs(pt.z) > 0.05) {
            ROS_ERROR("%s: new Pt had non-zero z: (%f %f %f)", __func__,
                    pt.x, pt.y, pt.z);
            continue;
        }
        points.push_back(cv::Point2f(pt.x, pt.y));
    }

    // do cv hull with the points of both
    std::vector<cv::Point2f> convex_hull;
    cv::convexHull(points, convex_hull, true);
    std::vector<geometry_msgs::Point> contours;
    forEach(const cv::Point2f & pt, convex_hull) {
        geometry_msgs::Point ptMsg;
        ptMsg.x = pt.x;
        ptMsg.y = pt.y;
        ptMsg.z = 0.0;
        contours.push_back(ptMsg);
    }
    // finally recreate mesh from merged contours
    // merged contours were created from already zero-corrected objects either from before
    // (because we put them in the planning scene) or because we just did that to the new table
    ret.meshes[0] = createMeshFromCountour(contours, 0.0);
    return ret;
}

double OrkToPlanningScene::tableDistance(const moveit_msgs::CollisionObject & ps_table,
        const moveit_msgs::CollisionObject & or_table, double z_match_distance)
{
    double minDist = HUGE_VAL;
    tf::Pose psToOr;
    // we want the tf to got to the or_table as that is the non-fixed frame and has
    // current timestamps for matching
    try {
        psToOr = computeTableContourTransform(ps_table, or_table);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return minDist;
    }
    tf::Pose orToPs = psToOr.inverse();

    std::vector<cv::Point2f> ps_contour;
    // contour is "upper" half of vertices
    for(unsigned int i = 0; i < ps_table.meshes[0].vertices.size()/2; ++i) {
        const geometry_msgs::Point & pt = ps_table.meshes[0].vertices[i];
        if(fabs(pt.z) > z_match_distance) {
            ROS_ERROR("%s: ps_table %s had large z val vertex (%f %f %f)", __func__, ps_table.id.c_str(),
                    pt.x, pt.y, pt.z);
            // still keep and error to not "deform" the contour
        }
        ps_contour.push_back(cv::Point2f(pt.x, pt.y));
    }

    // check each corner point or or_table to the contour of ps_table
    // This is not a fully correct check as corners arent necessarily the
    // closest points to a contour especially if they overlap.
    // This should only matter if multiple tables are close to each other.

    // we want to check how close or_table is to the ps_table contour
    for(unsigned int i = 0; i < or_table.meshes[0].vertices.size()/2; ++i) {
        const geometry_msgs::Point & pt = or_table.meshes[0].vertices[i];
        tf::Vector3 tfPt;
        tf::pointMsgToTF(pt, tfPt);
        tf::Vector3 orPtInPsFrame = orToPs * tfPt;
        if(fabs(orPtInPsFrame.z()) > z_match_distance)
            continue;
        cv::Point2f orPt(orPtInPsFrame.x(), orPtInPsFrame.y());
        // negate this as inside = pos, outside = neg
        double dist = - cv::pointPolygonTest(ps_contour, orPt, true);
        if(dist < minDist)      // no fabs! we use the signed distance for "more" penetration
            minDist = dist;
    }

    return minDist;
}

bool OrkToPlanningScene::isValidTable(const object_recognition_msgs::RecognizedObject & ro)
{
    // Assuming to ro has bounding contours with z about 0 and a pose.

    // z coord from tf pose in /base_footprint frame
    geometry_msgs::PoseStamped pose_transformed;
    geometry_msgs::PoseStamped pose;
    pose.header = ro.pose.header;
    pose.pose = ro.pose.pose.pose;
    try {
        tf_->waitForTransform("/base_footprint", ro.pose.header.frame_id, ro.pose.header.stamp,
                ros::Duration(0.5));
        tf_->transformPose("/base_footprint", pose, pose_transformed);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    if(pose_transformed.pose.position.z < table_min_z_ || pose_transformed.pose.position.z > table_max_z_) {
        return false;
    }

    tf::Pose poseTF;
    tf::poseMsgToTF(pose_transformed.pose, poseTF);
    // Rotate 0,0,1 by this pose and see if it still points up
    tf::Quaternion rotQuat = poseTF.getRotation();
    tf::Vector3 vZ(0.0, 0.0, 1.0);
    tf::Vector3 vZAtPose = tf::quatRotate(rotQuat, vZ);
    double dotDir = fabs(vZ.dot(vZAtPose));     // pointing straight up or down is OK
    if(dotDir < 0.9) {
        return false;
    }

    // area from contour
    // http://mathworld.wolfram.com/PolygonArea.html
    double det_sum = 0.0;
    for(int i = 0; i < ro.bounding_contours.size(); ++i) {
        int iNext = (i + 1) % ro.bounding_contours.size();
        det_sum += ro.bounding_contours[i].x * ro.bounding_contours[iNext].y;
        det_sum -= ro.bounding_contours[iNext].x * ro.bounding_contours[i].y;
    }
    det_sum *= 0.5;
    det_sum = fabs(det_sum);
    if(det_sum < table_min_area_)
        return false;
    return true;
}

double OrkToPlanningScene::estimateContourMeanZ(const std::vector<geometry_msgs::Point> & contours)
{
    if(contours.empty())
        return 0.0;
    double sumZ = 0.0;
    forEach(const geometry_msgs::Point & pt, contours) {
        sumZ += pt.z;
    }
    sumZ /= contours.size();
    return sumZ;
}

bool OrkToPlanningScene::collisionObjectFromRecognizedObject(const object_recognition_msgs::RecognizedObject & ro,
        moveit_msgs::CollisionObject & co, const std::string & table_prefix)
{
    co.header = ro.header;
    co.type = ro.type;

    double z_offset = 0.0;
    if(isTable(co.type)) {  // this is a table, not a RecognizedObject
        if(!isValidTable(ro)) {
            return false;
        }
        double contourMeanZ = estimateContourMeanZ(ro.bounding_contours);
        z_offset = contourMeanZ;    // we push the mesh against that, correct the pose by this
        co.meshes.push_back(createMeshFromCountour(ro.bounding_contours, -contourMeanZ));
        if(co.meshes.front().triangles.empty()) {
            ROS_WARN("Detected table had no triangles.");
            return false;
        }
        co.id = table_prefix;
    } else {    // Get a mesh or shape from somewhere
        // CollisionObjects contain primitives, meshes, or planes
        // RecognizedObjects provide PointClouds, bounding meshes, or contours
        // Unless we detect a primitive from data or db we can only use the mesh

        // First try to get a nice ground_truth_mesh from the DB
        object_recognition_msgs::GetObjectInformation::Request objectInfoReq;
        objectInfoReq.type = ro.type;
        object_recognition_msgs::GetObjectInformation::Response objectInfo;
        if(!srvObjectInfo_.call(objectInfoReq, objectInfo)) {
            // we need this to get the proper name, otherwise there is just the ugly co.type.key
            // This should be in the db
            ROS_ERROR_STREAM(__func__ << ": Could not get object info for: " << ro.type);
            return false;
        }

        if(!objectInfo.information.ground_truth_mesh.triangles.empty()) {
            co.meshes.push_back(objectInfo.information.ground_truth_mesh);
        } else if(!ro.bounding_mesh.triangles.empty()) {
            ROS_WARN_STREAM("Object " << objectInfo.information.name <<
                    " did not have a ground_truth_mesh, using bounding_mesh from recognized object instead. ObjectType: "
                    << ro.type);
            co.meshes.push_back(ro.bounding_mesh);
        } else {
            ROS_ERROR_STREAM("Object " << objectInfo.information.name <<
                    " did not have a ground_truth_mesh, neither was a bounding_mesh detected. This object has NO geometric information. ObjectType: " << ro.type);
            return false;
        }
        co.id = objectInfo.information.name;
        // write type information to param server, otherwise it is lost when converted to collision_detection::CollisionObject
        std::string type_information_namespace = "/object_type_information";
        ros::param::set(type_information_namespace+"/"+co.id+"/key", co.type.key);
        ros::param::set(type_information_namespace+"/"+co.id+"/db", co.type.db);
    }

    // we now have a mesh from somewhere
    // apply z_offset to pose
    tf::Pose mesh_pose;
    tf::poseMsgToTF(ro.pose.pose.pose, mesh_pose);
    tf::Pose z_offset_pose(tf::Quaternion(0,0,0,1), tf::Vector3(0.0, 0.0, z_offset));
    mesh_pose *= z_offset_pose;

    // TODO what with the merging
    geometry_msgs::Pose offset_mesh_pose;
    tf::poseTFToMsg(mesh_pose, offset_mesh_pose);
    co.mesh_poses.push_back(offset_mesh_pose);

    return true;
}


void OrkToPlanningScene::determineObjectMatches(
        const std::vector<moveit_msgs::CollisionObject> & planningSceneObjects,
        const std::vector<moveit_msgs::CollisionObject> & objectRecognitionObjects,
        std::vector<moveit_msgs::CollisionObject> & planningSceneUndetectedObjects,
        std::vector<moveit_msgs::CollisionObject> & objectRecognitionNewObjects,
        std::vector<moveit_msgs::CollisionObject> & planningSceneReplacedObjects,
        std::vector<std::pair<moveit_msgs::CollisionObject, moveit_msgs::CollisionObject> > & matchedObjects,
        bool handleTables)
{
    // This is not a complex best match assignment considering there might be N planning
    // scene objects near M new ones in general.
    // Basically we look that any planning scene object has a match of the same type
    // so that it is updated - or if the type doesnt match replaced
    // Unmatched ones are thus undetected.
    // Any recognized object that didnt take place either as a match or replacement is then new.

    std::set<const moveit_msgs::CollisionObject*> unhandledORObjects;
    forEach(const moveit_msgs::CollisionObject & co, objectRecognitionObjects) {
        if(handleTables && !isTable(co.type))
            continue;
        if(!handleTables && isTable(co.type))
            continue;

        if(co.mesh_poses.empty() && co.primitive_poses.empty()) {
            ROS_WARN_STREAM("Object Recognition object " << co.id << " did not have a valid pose. type: "
                    << co.type);
            continue;
        }
        unhandledORObjects.insert(&co);
    }

    // Try to match OR objects to the PS objects
    // Each PS object should have exactly one match
    forEach(const moveit_msgs::CollisionObject & psObject, planningSceneObjects) {
        if(handleTables && !isTable(psObject.type))
            continue;
        if(!handleTables && isTable(psObject.type))
            continue;

        std::vector<const moveit_msgs::CollisionObject*> typeMatches;
        std::vector<const moveit_msgs::CollisionObject*> otherTypeMatches;
        if(handleTables)
            findMatchingTables(psObject, unhandledORObjects, typeMatches,
                    table_match_distance_, table_z_match_distance_);
        else
            findMatchingObjects(psObject, unhandledORObjects, typeMatches, otherTypeMatches,
                    object_match_distance_, object_z_match_distance_);

        if(otherTypeMatches.empty()) {
            if(typeMatches.empty()) {
                planningSceneUndetectedObjects.push_back(psObject);
            } else {
                if(typeMatches.size() > 1) {
                    ROS_WARN("Found %zu objects near %s - using nearest.", typeMatches.size(), psObject.id.c_str());
                }
                matchedObjects.push_back(std::make_pair(psObject, *(typeMatches.front())));
                unhandledORObjects.erase(typeMatches.front());
            }
        } else {
            // matching objects of different type -> shouldnt happen
            ROS_WARN("Found objects near %s of different type.", psObject.id.c_str());
            forEach(const moveit_msgs::CollisionObject* co, otherTypeMatches)
                ROS_WARN_STREAM(co->id << " with type " << co->type);
            // if there is a good type match, use that to match this PS
            if(!typeMatches.empty()) {
                ROS_WARN("Also found object(s) with matching type -> using nearest of these.");
                forEach(const moveit_msgs::CollisionObject* co, otherTypeMatches)
                    ROS_WARN_STREAM(co->id << " with type " << co->type);
                // use nearest
                matchedObjects.push_back(std::make_pair(psObject, *(typeMatches.front())));
                unhandledORObjects.erase(typeMatches.front());
            } else {
                // otherwise, discard the PS and make this the new one.
                ROS_WARN("Did not find objects with matching type. Discarding %s and replacing by nearest detected one.", psObject.id.c_str());
                // replace old with nearest new
                objectRecognitionNewObjects.push_back(*(otherTypeMatches.front()));
                planningSceneReplacedObjects.push_back(psObject);
                unhandledORObjects.erase(otherTypeMatches.front());
            }
        }
    }

    // unmatched to any other obj are NEW objs
    forEach(const moveit_msgs::CollisionObject* co, unhandledORObjects) {
        objectRecognitionNewObjects.push_back(*co);
    }
}

void OrkToPlanningScene::findMatchingObjects(const moveit_msgs::CollisionObject & psObject,
        const std::set<const moveit_msgs::CollisionObject*> & orObjects,
        std::vector<const moveit_msgs::CollisionObject*> & typeMatches,
        std::vector<const moveit_msgs::CollisionObject*> & otherTypeMatches,
        double match_distance, double z_match_distance)
{
    geometry_msgs::PoseStamped psPose = getPoseStamped(psObject);
    forEach(const moveit_msgs::CollisionObject* co, orObjects) {
        std::pair<double, double> dist = poseDistance(psPose, getPoseStamped(*co));
        if(dist.first <= match_distance && dist.second <= z_match_distance) {
            if(psObject.type.key == co->type.key && psObject.type.db == co->type.db)
                typeMatches.push_back(co);
            else
                otherTypeMatches.push_back(co);
        }
    }
    sort(typeMatches.begin(), typeMatches.end(), DistanceToPose(psPose, *this));
    sort(otherTypeMatches.begin(), otherTypeMatches.end(), DistanceToPose(psPose, *this));
}

void OrkToPlanningScene::findMatchingTables(const moveit_msgs::CollisionObject & psObject,
        const std::set<const moveit_msgs::CollisionObject*> & orObjects,
        std::vector<const moveit_msgs::CollisionObject*> & typeMatches,
        double match_distance, double z_match_distance)
{
    forEach(const moveit_msgs::CollisionObject* co, orObjects) {
        double dist = tableDistance(psObject, *co, z_match_distance);
        if(dist <= match_distance) {
            if(!(psObject.type.key == co->type.key && psObject.type.db == co->type.db)) {
                ROS_ERROR("%s called with different key/db entries.", __func__);
            } else {
                typeMatches.push_back(co);
            }
        }
    }

    sort(typeMatches.begin(), typeMatches.end(), DistanceToTable(psObject, *this));
}

geometry_msgs::PoseStamped OrkToPlanningScene::getPoseStamped(const moveit_msgs::CollisionObject & co)
{
    ROS_ASSERT(!(co.mesh_poses.empty() && co.primitive_poses.empty()));

    std::vector<geometry_msgs::Pose> poses;
    if(!co.mesh_poses.empty() && !co.primitive_poses.empty()) {
        ROS_WARN("%s: CollisionObject %s had mesh_poses and primitive_poses -> using primitive_poses",
                __func__, co.id.c_str());
        poses = co.primitive_poses;
    } else if(!co.primitive_poses.empty()) {
        poses = co.primitive_poses;
    } else {
        ROS_ASSERT(!co.mesh_poses.empty());
        poses = co.mesh_poses;
    }
    geometry_msgs::PoseStamped ret;
    if(poses.size() > 1) {
        ROS_WARN("%s: CollisionObject %s had %zu poses -> using first.", __func__, co.id.c_str(), poses.size());
    }
    ret.pose = poses.front();
    ret.header = co.header;
    return ret;
}

std::pair<double, double> OrkToPlanningScene::poseDistance(const geometry_msgs::PoseStamped & posePS,
        const geometry_msgs::PoseStamped & poseOR)
{
    // OR poses might be in a sensor frame -> transform to PS frame first
    geometry_msgs::PoseStamped poseOR_transformed;
    try {
        tf_->waitForTransform(posePS.header.frame_id, poseOR.header.frame_id, poseOR.header.stamp,
                ros::Duration(0.5));
        tf_->transformPose(posePS.header.frame_id, poseOR, poseOR_transformed);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }

    tf::Pose tfPS;
    tf::Pose tfOR;
    tf::poseMsgToTF(posePS.pose, tfPS);
    tf::poseMsgToTF(poseOR_transformed.pose, tfOR);
    tf::Pose delta = tfPS.inverseTimes(tfOR);
    return std::make_pair(hypot(delta.getOrigin().x(), delta.getOrigin().y()),
            fabs(delta.getOrigin().z()));   // usually we're interested in the 2d distance independently
}


void OrkToPlanningScene::updateMaxObjectId(const moveit_msgs::CollisionObject & co,
        std::map<std::string, unsigned int> & maxObjectId)
{
    bool ok;
    std::pair<std::string, unsigned int> typeId = symbolic_name_tools::split_name(co.id, ok);
    if(!ok) {
        ROS_WARN("%s: Could not split name for %s", __func__, co.id.c_str());
        return;
    }
    unsigned int & curId = maxObjectId[typeId.first];
    if(typeId.second > curId) {
        curId = typeId.second;
    }
}

bool OrkToPlanningScene::isTable(const object_recognition_msgs::ObjectType & type)
{
    return type.db == "Tabletop";
}

}

