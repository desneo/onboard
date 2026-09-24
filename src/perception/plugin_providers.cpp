#include "functional/det_mot_api.h"
#include "functional/image_input_api.h"
#include "functional/videostream_output_api.h"

namespace plane::algorithm {

PLUMA_PROVIDER_SOURCE(ImageInputPlugin, 1, 1);
PLUMA_PROVIDER_SOURCE(DetMotPlugin, 2, 2);
PLUMA_PROVIDER_SOURCE(VideoStreamOutputPlugin, 1, 1);

}  // namespace plane::algorithm
