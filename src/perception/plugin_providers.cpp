#include "functional/detection_api.h"
#include "functional/guidance_api.h"
#include "functional/tracking_api.h"

namespace plane::algorithm {

PLUMA_PROVIDER_SOURCE(DetectionPlugin, 1, 1);
PLUMA_PROVIDER_SOURCE(TrackingPlugin, 1, 1);
PLUMA_PROVIDER_SOURCE(GuidancePlugin, 1, 1);

}  // namespace plane::algorithm
