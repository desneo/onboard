#include "video/ffmpeg_h264_encoder.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cstdint>

namespace plane::video {

FfmpegH264Encoder::FfmpegH264Encoder() = default;

FfmpegH264Encoder::~FfmpegH264Encoder() { close(); }

bool FfmpegH264Encoder::init(int width, int height, int fps, int bitrate_kbps, std::string& error) {
    if (codec_ != nullptr && width == width_ && height == height_ && fps == fps_) {
        return true;
    }
    close();
    return open_codec(width, height, fps, bitrate_kbps, error);
}

bool FfmpegH264Encoder::open_codec(int width, int height, int fps, int bitrate_kbps, std::string& error) {
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (codec == nullptr) {
        error = "未找到 H264 编码器";
        return false;
    }
    codec_ = avcodec_alloc_context3(codec);
    if (codec_ == nullptr) {
        error = "编码器上下文分配失败";
        return false;
    }
    codec_->width = width;
    codec_->height = height;
    codec_->time_base = AVRational{1, fps};
    codec_->framerate = AVRational{fps, 1};
    codec_->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_->bit_rate = static_cast<std::int64_t>(bitrate_kbps) * 1000;
    codec_->gop_size = fps * 2;
    codec_->max_b_frames = 0;

    AVDictionary* options = nullptr;
    av_dict_set(&options, "preset", "veryfast", 0);
    av_dict_set(&options, "tune", "zerolatency", 0);
    const int open_result = avcodec_open2(codec_, codec, &options);
    av_dict_free(&options);
    if (open_result < 0) {
        error = "H264 编码器打开失败";
        close();
        return false;
    }

    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (frame_ == nullptr || packet_ == nullptr) {
        error = "编码帧/包分配失败";
        close();
        return false;
    }
    frame_->format = codec_->pix_fmt;
    frame_->width = width;
    frame_->height = height;
    if (av_frame_get_buffer(frame_, 32) < 0) {
        error = "编码帧缓冲分配失败";
        close();
        return false;
    }

    sws_ = sws_getContext(width, height, AV_PIX_FMT_BGR24, width, height, AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr,
                          nullptr, nullptr);
    if (sws_ == nullptr) {
        error = "颜色转换上下文创建失败";
        close();
        return false;
    }

    width_ = width;
    height_ = height;
    fps_ = fps;
    pts_ = 0;
    return true;
}

bool FfmpegH264Encoder::encode(const cv::Mat& bgr_frame, std::vector<std::uint8_t>& out, bool& keyframe,
                               std::string& error) {
    out.clear();
    keyframe = false;
    if (codec_ == nullptr) {
        error = "编码器未初始化";
        return false;
    }
    if (bgr_frame.empty() || bgr_frame.cols != width_ || bgr_frame.rows != height_) {
        error = "帧尺寸与编码器不一致";
        return false;
    }
    const std::uint8_t* src_data[1] = {bgr_frame.data};
    const int src_linesize[1] = {static_cast<int>(bgr_frame.step)};
    sws_scale(sws_, src_data, src_linesize, 0, height_, frame_->data, frame_->linesize);

    frame_->pts = pts_++;
    int result = avcodec_send_frame(codec_, frame_);
    if (result < 0) {
        error = "发送编码帧失败";
        return false;
    }
    while (true) {
        result = avcodec_receive_packet(codec_, packet_);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
            break;
        }
        if (result < 0) {
            error = "接收编码包失败";
            return false;
        }
        out.insert(out.end(), packet_->data, packet_->data + packet_->size);
        if ((packet_->flags & AV_PKT_FLAG_KEY) != 0) {
            keyframe = true;
        }
        av_packet_unref(packet_);
    }
    return true;
}

void FfmpegH264Encoder::close() {
    if (sws_ != nullptr) {
        sws_freeContext(sws_);
        sws_ = nullptr;
    }
    if (frame_ != nullptr) {
        av_frame_free(&frame_);
    }
    if (packet_ != nullptr) {
        av_packet_free(&packet_);
    }
    if (codec_ != nullptr) {
        avcodec_free_context(&codec_);
    }
    width_ = 0;
    height_ = 0;
    fps_ = 0;
    pts_ = 0;
}

}  // namespace plane::video
