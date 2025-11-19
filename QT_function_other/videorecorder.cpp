/**
 * @file videorecorder.cpp
 * @brief 使用 FFmpeg 将 QImage 视频帧录制为 H.264 视频文件
 *
 * 功能：
 * - 支持 RGB32 图像转 YUV420P
 * - 多线程录制，使用队列缓存帧
 * - 自动处理编码器缓冲区，保证零延迟
 * - 支持 start/stop 控制
 */

#include "videorecorder.h"
#include <QDebug>
#include <QDateTime>

VideoRecorder(QObject *parent)
    : QObject(parent),
      m_formatContext(nullptr),
      m_codecContext(nullptr),
      m_videoStream(nullptr),
      m_swsContext(nullptr),
      m_recording(false),
      m_stopRequested(false),
      m_width(0),
      m_height(0),
      m_fps(25),
      m_frameCount(0)
{
}

~VideoRecorder()
{
    stopRecording();
}

/**
 * @brief 启动视频录制
 * @param filename 输出文件名
 * @param width 视频宽度
 * @param height 视频高度
 * @param fps 帧率
 * @return 是否成功启动
 */
bool startRecording(const QString &filename, int width, int height, int fps)
{
    if (m_recording) {
        emit errorOccurred("Already recording");
        return false;
    }

    m_frameCount = 0; 
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_filename = filename;

    // 清空队列
    {
        QMutexLocker locker(&m_mutex);
        m_frameQueue.clear();
    }

    // 初始化输出格式
    avformat_alloc_output_context2(&m_formatContext, nullptr, nullptr, filename.toUtf8().constData());
    if (!m_formatContext) {
        emit errorOccurred("Could not create output context");
        return false;
    }

    AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        emit errorOccurred("H.264 codec not found");
        return false;
    }

    m_videoStream = avformat_new_stream(m_formatContext, codec);
    if (!m_videoStream) {
        emit errorOccurred("Could not create video stream");
        return false;
    }

    m_codecContext = avcodec_alloc_context3(codec);
    if (!m_codecContext) {
        emit errorOccurred("Could not alloc codec context");
        return false;
    }

    // 设置编码器参数
    m_codecContext->width = width;
    m_codecContext->height = height;
    m_codecContext->time_base = {1, fps};
    m_codecContext->framerate = {fps, 1};
    m_codecContext->pix_fmt = AV_PIX_FMT_YUV420P;
    m_codecContext->bit_rate = 4000000;

    av_opt_set(m_codecContext->priv_data, "preset", "fast", 0);
    av_opt_set(m_codecContext->priv_data, "tune", "zerolatency", 0);

    if (avcodec_open2(m_codecContext, codec, nullptr) < 0) {
        emit errorOccurred("Could not open codec");
        avcodec_free_context(&m_codecContext);
        return false;
    }

    if (avcodec_parameters_from_context(m_videoStream->codecpar, m_codecContext) < 0) {
        emit errorOccurred("Could not copy codec parameters");
        avcodec_free_context(&m_codecContext);
        return false;
    }

    if (!(m_formatContext->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&m_formatContext->pb, filename.toUtf8().constData(), AVIO_FLAG_WRITE) < 0) {
            emit errorOccurred("Could not open output file");
            avcodec_free_context(&m_codecContext);
            avformat_free_context(m_formatContext);
            return false;
        }
    }

    if (avformat_write_header(m_formatContext, nullptr) < 0) {
        emit errorOccurred("Could not write header");
        if (m_formatContext && !(m_formatContext->oformat->flags & AVFMT_NOFILE))
            avio_closep(&m_formatContext->pb);
        avcodec_free_context(&m_codecContext);
        avformat_free_context(m_formatContext);
        return false;
    }

    // 初始化 RGB->YUV 转换
    m_swsContext = sws_getContext(width, height, AV_PIX_FMT_RGB32,
                                  width, height, AV_PIX_FMT_YUV420P,
                                  SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!m_swsContext) {
        emit errorOccurred("Could not create SWS context");
        if (m_formatContext && !(m_formatContext->oformat->flags & AVFMT_NOFILE))
            avio_closep(&m_formatContext->pb);
        avcodec_free_context(&m_codecContext);
        avformat_free_context(m_formatContext);
        return false;
    }

    m_stopRequested = false;
    m_recording = true;
    m_thread = std::thread(&VideoRecorder::recordingThread, this);

    emit recordingStarted();
    return true;
}

/**
 * @brief 停止视频录制
 */
void stopRecording()
{
    if (!m_recording) return;

    m_stopRequested = true;
    m_condition.wakeAll();

    if (m_thread.joinable()) {
        m_thread.join();
    }

    m_recording = false;
    emit recordingFinished(m_filename);
}

/**
 * @brief 添加帧到录制队列
 * @param frame QImage 图像
 */
void addFrame(const QImage &frame)
{
    if (!m_recording) return;

    QImage local = frame;
    if (local.format() != QImage::Format_RGB32) {
        local = local.convertToFormat(QImage::Format_RGB32);
    }

    QMutexLocker locker(&m_mutex);
    if (m_frameQueue.size() >= MAX_QUEUE_SIZE) {
        m_frameQueue.takeFirst(); // 队列满则丢弃最早帧
    }
    m_frameQueue.append(local.copy());
    m_condition.wakeOne();
}

/**
 * @brief 是否正在录制
 * @return true/false
 */
bool isRecording() const
{
    return m_recording;
}

/**
 * @brief 录制线程函数
 *
 * 功能：
 * - 从队列取帧
 * - RGB->YUV420P 转换
 * - 发送给 H.264 编码器
 * - 写入输出文件
 * - 结束时 flush 编码器并写 trailer
 */
void recordingThread()
{
    AVFrame *yuvFrame = av_frame_alloc();
    yuvFrame->format = AV_PIX_FMT_YUV420P;
    yuvFrame->width = m_width;
    yuvFrame->height = m_height;
    if (av_frame_get_buffer(yuvFrame, 32) < 0) {
        qWarning() << "Failed to alloc yuv buffer";
        av_frame_free(&yuvFrame);
        return;
    }

    AVPacket *packet = av_packet_alloc();
    if (!packet) {
        av_frame_free(&yuvFrame);
        return;
    }

    while (!m_stopRequested) {
        QImage frame;

        {
            QMutexLocker locker(&m_mutex);
            if (m_frameQueue.isEmpty()) {
                m_condition.wait(&m_mutex, 200); // 队列为空则等待
                continue;
            }
            frame = m_frameQueue.takeFirst();
        }

        if (frame.isNull()) continue;

        uint8_t *srcData[4] = { const_cast<uint8_t*>(frame.bits()), nullptr, nullptr, nullptr };
        int srcLinesize[4] = { frame.bytesPerLine(), 0, 0, 0 };

        sws_scale(m_swsContext, srcData, srcLinesize, 0, m_height,
                  yuvFrame->data, yuvFrame->linesize);

        yuvFrame->pts = m_frameCount++;

        if (avcodec_send_frame(m_codecContext, yuvFrame) < 0) {
            qDebug() << "Error sending frame to encoder";
            continue;
        }

        while (avcodec_receive_packet(m_codecContext, packet) == 0) {
            av_packet_rescale_ts(packet, m_codecContext->time_base, m_videoStream->time_base);
            packet->stream_index = m_videoStream->index;
            av_interleaved_write_frame(m_formatContext, packet);
            av_packet_unref(packet);
        }
    }

    // flush encoder
    avcodec_send_frame(m_codecContext, nullptr);
    while (avcodec_receive_packet(m_codecContext, packet) == 0) {
        av_packet_rescale_ts(packet, m_codecContext->time_base, m_videoStream->time_base);
        packet->stream_index = m_videoStream->index;
        av_interleaved_write_frame(m_formatContext, packet);
        av_packet_unref(packet);
    }

    av_write_trailer(m_formatContext);

    av_frame_free(&yuvFrame);
    av_packet_free(&packet);

    if (m_swsContext) {
        sws_freeContext(m_swsContext);
        m_swsContext = nullptr;
    }

    if (m_codecContext) {
        avcodec_close(m_codecContext);
        avcodec_free_context(&m_codecContext);
        m_codecContext = nullptr;
    }

    if (m_formatContext && !(m_formatContext->oformat->flags & AVFMT_NOFILE)) {
        avio_closep(&m_formatContext->pb);
    }

    if (m_formatContext) {
        avformat_free_context(m_formatContext);
        m_formatContext = nullptr;
    }

    // 清空帧队列
    {
        QMutexLocker locker(&m_mutex);
        m_frameQueue.clear();
    }
}
