#ifndef SVS_NODELET_H_
#define SVS_NODELET_H_

#include <nodelet/nodelet.h>

namespace svs {
class SvsNodelet : public nodelet::Nodelet {
  public:
    virtual void onInit();
};
} // namespace svs
#endif
