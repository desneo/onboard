// 插件加载契约测试：加载 plugins_dir 并断言三类 provider 各至少 1 个。
// 用法: plugin_load_check [plugins_dir]
#include <Pluma/Pluma.hpp>
#include <spdlog/spdlog.h>

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "functional/det_mot_api.h"
#include "functional/image_input_api.h"
#include "functional/videostream_output_api.h"

namespace plane::algorithm {

PLUMA_PROVIDER_SOURCE(ImageInputPlugin, 1, 1);
PLUMA_PROVIDER_SOURCE(DetMotPlugin, 2, 2);
PLUMA_PROVIDER_SOURCE(VideoStreamOutputPlugin, 1, 1);

}  // namespace plane::algorithm

namespace {

template <typename ProviderType>
std::size_t count_providers(pluma::Pluma& pluma) {
    std::vector<ProviderType*> providers;
    pluma.getProviders(providers);
    return providers.size();
}

}  // namespace

int main(int argc, char** argv) {
    const std::string plugins_dir = argc > 1 ? argv[1] : "plugins";

    pluma::Pluma pluma;
    pluma.acceptProviderType<plane::algorithm::ImageInputPluginProvider>();
    pluma.acceptProviderType<plane::algorithm::DetMotPluginProvider>();
    pluma.acceptProviderType<plane::algorithm::VideoStreamOutputPluginProvider>();
    const int loaded = pluma.loadFromFolder(plugins_dir);

    const std::size_t image_input =
        count_providers<plane::algorithm::ImageInputPluginProvider>(pluma);
    const std::size_t det_mot = count_providers<plane::algorithm::DetMotPluginProvider>(pluma);
    const std::size_t video_output =
        count_providers<plane::algorithm::VideoStreamOutputPluginProvider>(pluma);

    std::printf("plugins_dir=%s loaded=%d image_input=%zu det_mot=%zu video_output=%zu\n",
                plugins_dir.c_str(), loaded, image_input, det_mot, video_output);

    // 三插件均须可用（取图/检测/推流）。
    if (image_input < 1 || det_mot < 1 || video_output < 1) {
        std::fprintf(stderr, "插件加载契约失败: 三类 provider 均需 >= 1\n");
        return 1;
    }
    return 0;
}
