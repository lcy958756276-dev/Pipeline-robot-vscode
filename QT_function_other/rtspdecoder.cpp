/**
 * @file rtspdecoder.cpp
 * @brief RTSP 视频流解码类，基于 FFmpeg，实现 QThread 异步解码
 *
 * 功能：
 * - 连接 RTSP 视频流
 * - 使用 FFmpeg 解码 H.264/H.265 视频
 * - 转换为 QImage 并通过信号发送到 UI
 * - 自动重连功能
 */

#include "rtspdecoder.h"
#include <QDebug>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

/**
 * @brief 构造函数，初始化线程对象
 * @param parent 父对象指针
 */
RtspDecoder(QObject *parent) : QThread(parent) {}

/**
 * @brief 析构函数，安全停止线程并释放资源
 */
~RtspDecoder() {
    requestInterruption();
    wait();
    cleanup();
}

/**
 * @brief 设置 RTSP URL
 * @param url RTSP 流地址
 *
 * 线程安全，使用 QMutexLocker 锁保护成员变量
 */
void setUrl(const QString &url) {
    QMutexLocker locker(&mutex_);
    rtsp_url_ = url;
}

/**
 * @brief QThread 线程入口函数
 *
 * 持续解码 RTSP 流，转换为 QImage 并发送 frameDecoded 信号。
 * 断开连接时触发 connectionLost 信号，可实现自动重连。
 */
void run() {
    while(!isInterruptionRequested()) {
        if(!initFFmpeg()) {
            emit errorOccurred("初始化失败");
            break;
        }

        AVFrame *frame = av_frame_alloc();
        bool connection_ok = true;

        while(connection_ok && !isInterruptionRequested()) {
            AVPacket pkt;
            int ret = av_read_frame(fmt_ctx_, &pkt);
            if(ret < 0) {
                if(ret == AVERROR_EOF || avio_feof(fmt_ctx_->pb)) {
                    qDebug() << "RTSP连接中断";
                    emit connectionLost();
                    connection_ok = false;
                }
                break;
            }

            if(pkt.stream_index == video_stream_index_) {
                if(avcodec_send_packet(codec_ctx_, &pkt) < 0) {
                    av_packet_unref(&pkt);
                    continue;
                }

                AVFrame *rgb_frame = av_frame_alloc();
                uint8_t *buffer = nullptr;
                int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_RGB32,
                                                         codec_ctx_->width,
                                                         codec_ctx_->height, 1);
                buffer = (uint8_t *)av_malloc(num_bytes * sizeof(uint8_t));
                av_image_fill_arrays(rgb_frame->data, rgb_frame->linesize, buffer,
                                     AV_PIX_FMT_RGB32, codec_ctx_->width, codec_ctx_->height, 1);

                while(avcodec_receive_frame(codec_ctx_, frame) == 0) {
                    sws_scale(sws_ctx_,
                              (uint8_t const * const *)frame->data,
                              frame->linesize, 0, codec_ctx_->height,
                              rgb_frame->data, rgb_frame->linesize);

                    QImage img(rgb_frame->data[0],
                               codec_ctx_->width,
                               codec_ctx_->height,
                               rgb_frame->linesize[0],
                               QImage::Format_RGB32);

                    emit frameDecoded(img.copy());
                }

                av_free(buffer);
                av_frame_free(&rgb_frame);
            }

            av_packet_unref(&pkt);
        }

        av_frame_free(&frame);
        cleanup();

        if(need_reconnect_ && reconnect_attempts_++ < max_reconnect_attempts_) {
            QThread::msleep(2000);
        } else {
            break;
        }
    }
}

/**
 * @brief 初始化 FFmpeg，打开 RTSP 流并准备解码器
 * @return 成功返回 true，失败返回 false
 */
bool initFFmpeg() {
    cleanup();
    avformat_network_init();

    AVDictionary *opts = nullptr;
    av_dict_set(&opts, "stimeout", "5000000", 0);
    av_dict_set(&opts, "max_delay", "500000", 0);
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);
    av_dict_set(&opts, "fflags", "nobuffer", 0);
    av_dict_set(&opts, "flags", "low_delay", 0);
    av_dict_set(&opts, "analyzeduration", "450000", 0);
    av_dict_set(&opts, "probesize", "96", 0);
    av_dict_set(&opts, "infbuf", "1", 0);

    if(avformat_open_input(&fmt_ctx_, rtsp_url_.toUtf8().constData(), nullptr, &opts) != 0) {
        av_dict_free(&opts);
        return false;
    }
    av_dict_free(&opts);

    if(avformat_find_stream_info(fmt_ctx_, nullptr) < 0) return false;

    video_stream_index_ = -1;
    for(unsigned int i = 0; i < fmt_ctx_->nb_streams; i++) {
        if(fmt_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index_ = i;
            break;
        }
    }
    if(video_stream_index_ == -1) return false;

    AVCodecParameters *codecpar = fmt_ctx_->streams[video_stream_index_]->codecpar;
    AVCodec *codec = avcodec_find_decoder(codecpar->codec_id);
    if(!codec) return false;

    codec_ctx_ = avcodec_alloc_context3(codec);
    if(avcodec_parameters_to_context(codec_ctx_, codecpar) < 0) return false;

    if(avcodec_open2(codec_ctx_, codec, nullptr) < 0) return false;

    sws_ctx_ = sws_getContext(codec_ctx_->width, codec_ctx_->height,
                              codec_ctx_->pix_fmt,
                              codec_ctx_->width, codec_ctx_->height,
                              AV_PIX_FMT_RGB32,
                              SWS_BILINEAR, nullptr, nullptr, nullptr);
    return sws_ctx_ != nullptr;
}

/**
 * @brief 释放 FFmpeg 相关资源
 *
 * 包括 SWS 转换器、解码器上下文和输入流。
 */
void cleanup() {
    if(sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }

    if(codec_ctx_) {
        avcodec_close(codec_ctx_);
        avcodec_free_context(&codec_ctx_);
        codec_ctx_ = nullptr;
    }

    if(fmt_ctx_) {
        avformat_close_input(&fmt_ctx_);
        fmt_ctx_ = nullptr;
    }

    avformat_network_deinit();
}
