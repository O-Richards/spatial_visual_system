#include "spatial_visual_system/scene_writer.h"

#include <world_model_store_msgs/Insert.h>
#include <world_model_store_msgs/Details.h>

#include <ros/ros.h>

#include <spatial_visual_system/Detection.h>

namespace svs {
void SceneWriter::write(SofA &sofa) {
    nlohmann::json object_json;

    {
        std::lock_guard<std::mutex> sofa_lock{sofa.lock_};
        object_json = sofa.annotations_;
    }

    /*
     * Prepare the json string required
     * We need to flatten the labels and find the correct scene id
     */

    object_json[world_model_store_msgs::Details::SCENE_ID] = curr_scene_id_;

    // We flatten the labels into a json formatted string of a list of <label, label confidence>
    object_json["labels"] = object_json["labels"].dump();

#if USE_SERVICE
    world_model_store_msgs::Insert::Request req{};
    req.insert.json = object_json.dump();

    ROS_INFO_STREAM_COND(debug_, "SceneWriter::write: adding sofa: " << sofa.getId() << ": " << req.insert.json);

    world_model_store_msgs::Insert::Response resp{};
    new_scene_obj_serv_.call(req, resp);
    if (resp.status == resp.FAIL) {
        ROS_WARN_STREAM("SceneWriter::write: Failed to create new scene object." << 
                " service response: " << resp <<
                " Request: " << req);
    }
#endif
    
    // Publish detection to topic
    spatial_visual_system::Detection detection_msg{};
    detection_msg.json = object_json.dump();
    detection_msg.sofa_id = sofa.getId();
    detection_msg.scene_id = curr_scene_id_;
    detection_pub_.publish(detection_msg);
}

void SceneWriter::new_scene() {
#if USE_SERVICE
    // Insert a new scene into the db
    world_model_store_msgs::Insert::Request req{};
    world_model_store_msgs::Insert::Response resp{};
    nlohmann::json req_json{};

    // TODO: Do proper robot states
    int robot_state_id = new_robot_state();
    req_json[world_model_store_msgs::Details::ROBOT_STATE_ID] = robot_state_id;

    req.insert.json = req_json.dump();

    new_scene_serv_.call(req, resp);

    if (resp.status == resp.FAIL) {
        ROS_ERROR_STREAM("SceneWriter::new_scene: Failed to create new scene. service response: " << resp);
    }

    curr_scene_id_ = resp.inserted_id;
#else
    curr_scene_id_++;
#endif
}

// return -1 on error
int SceneWriter::new_robot_state() {
  world_model_store_msgs::Insert::Request req{};
  nlohmann::json new_data = {
                             {"frame_id", "/base_link"},
                             {"base_pose_x", 0},
                             {"base_pose_y", 0},
                             {"base_pose_z", 0},
                             {"base_pose_w", 0},
                             {"base_r", 0},
                             {"base_p", 0},
                             {"base_yw", 0},
                             {"head_pose_x", 0},
                             {"head_pose_y", 1},
                             {"head_pose_z", 2},
                             {"head_pose_w", 3},
                             {"head_r", 0.2},
                             {"head_p", 0.1},
                             {"head_yw", 0.1},
                             {"driveable_state", 1},
                             {"arm_state", 0},
                             {"shoulder_state", 0},
                             {"gripper_state", 0},
                             {"holding_object_id", nullptr}
  };

  req.insert.json = new_data.dump();  // TODO put data in here!
  world_model_store_msgs::Insert::Response resp{};
#if USE_SERVICE
  new_robot_state_serv_.call(req, resp);
#endif
  if (resp.status == resp.FAIL) {
    ROS_ERROR_STREAM(
        "SceneWriter::new_robot_state: Failed to create new robot state. "
        "service response: "
        << resp);
  }

  return resp.inserted_id;
}

} // namespace svs
