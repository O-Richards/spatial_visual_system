#ifndef SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_
#define SPATIAL_VISUAL_SYSTEM_INCLUDE_SCENE_H_

#include <vector>
#include <mutex>

#include "spatial_visual_system/sofa.h"
#include "spatial_visual_system/scene_writer.h"

#include <unsw_vision_msgs/DetectionList.h>

namespace svs {
class Scene {
public:
  SofA& addSofA() {sofa_.push_back(SofA{}); return sofa_.back();}
  void accept(SceneWriter& writer);
  void reset() {sofa_.clear();};
  void setPercept(const Percept& percept) {
      percept_ = percept;
  }
  const Percept& getPercept() const {return percept_;};
  Percept& getPercept() {return percept_;};

  const std::vector<SofA>& getSofA() const {return sofa_;};
  std::vector<SofA>& getSofA() {return sofa_;};

  std::mutex lock_;

  void setDetections(const unsw_vision_msgs::DetectionList& detections) {
      detections_ = detections;
  }

  unsw_vision_msgs::DetectionList copyDetectionsWLock() {
      std::lock_guard<std::mutex> lock{lock_};

      return detections_;
  }

  void saveSofA(SofA& sofa, const std::string& dir);
  void save(const std::string& dir);

private:
  std::vector<SofA> sofa_;
  Percept percept_;
  unsw_vision_msgs::DetectionList detections_;
};
} // namespace svs

#endif
