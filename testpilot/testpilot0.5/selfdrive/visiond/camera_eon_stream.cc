#include "camera_eon_stream.h"

#include <string>
#include <unistd.h>
#include <vector>
#include <string.h>

#include <czmq.h>
#include <libyuv.h>
#include <capnp/serialize.h>
#include "cereal/gen/cpp/log.capnp.h"

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "buffering.h"
#include "/usr/include/opencv2/opencv.hpp"


extern "C" {
#include <libavcodec/avcodec.h>
}

extern volatile int do_exit;

#define FRAME_WIDTH 1164
#define FRAME_HEIGHT 874

namespace {
void camera_open(CameraState *s, cl_mem *yuv_cls, bool rear, cl_device_id device_id, cl_context context, cl_command_queue q) {
  assert(yuv_cls);
  s->yuv_cls = yuv_cls;
  s->device_id = device_id;
  s->context = context;
  s->q = q;
}

void camera_close(CameraState *s) {
  tbuffer_stop(&s->camera_tb);
}

void camera_release_buffer(void *cookie, int buf_idx) {
  CameraState *s = static_cast<CameraState *>(cookie);
}

void camera_init(CameraState *s, int camera_id, unsigned int fps) {
  assert(camera_id < ARRAYSIZE(cameras_supported));
  s->ci = cameras_supported[camera_id];
  assert(s->ci.frame_width != 0);

  s->frame_size = s->ci.frame_height * s->ci.frame_stride;
  s->fps = fps;

  tbuffer_init2(&s->camera_tb, FRAME_BUF_COUNT, "frame", camera_release_buffer,
                s);
}

void run_eon_stream(DualCameraState *s) {
  int err;
  uint8_t stream_start[74] =
{0x00,0x00,0x00,0x01,0x40,0x01,0x0c,0x01,0xff,0xff,0x01,0x60,0x00,0x00,0x03,0x00,0xb0,0x00,0x00,0x03,0x00,0x00,0x03,0x00,0x5d,0xac,0x59,0x00,0x00,0x00,0x01,0x42,0x01,0x01,0x01,0x60,0x00,0x00,0x03,0x00,0xb0,0x00,0x00,0x03,0x00,0x00,0x03,0x00,0x5d,0xa0,0x02,0x50,0x80,0x38,0x1c,0x5c,0x66,0x5a,0xee,0x4c,0x92,0xec,0x80,0x00,0x00,0x00,0x01,0x44,0x01,0xc0,0xf1,0x80,0x04,0x20};

  AVFrame *frame = av_frame_alloc();
  assert(frame);

  // @ for bind, > for connect
  //zsock_t *frame_sock = zsock_new_pull(">tcp://172.17.0.1:5559");
  zsock_t *frame_sock = zsock_new_pair("@tcp://*:5559");
  assert(frame_sock);
  void *frame_sock_raw = zsock_resolve(frame_sock);
  zstr_send(frame_sock, ""); // tell runner vision is ready to go

  CameraState *const rear_camera = &s->rear;
  auto *tb = &rear_camera->camera_tb;

  AVPacket avpkt;
  av_init_packet(&avpkt);

  avpkt.size = sizeof(stream_start);
  avpkt.data = &stream_start[0];
  int frame_id = 0;

  while (!do_exit) {

    zmsg_t *msg= zmsg_recv(frame_sock_raw);
    assert (msg);
    char *payload = zmsg_popstr (msg);
    cv::Mat rawData( 1, 3052062, CV_8UC1, (void *) payload);
    cv::Mat image_rgb(cv::imdecode(rawData,1));
    cv::Mat image;
    cv::cvtColor(image_rgb,image_rgb, cv::COLOR_RGB2YUV);
    image_rgb.copyTo(image);
    cv::Mat image_half;
    cv::resize(image, image_half, cv::Size(), 0.5, 0.5);
    cv::vector<cv::Mat> yuv_channels;
    cv::vector<cv::Mat> yuv_half;
    split(image, yuv_channels);
    split(image_half, yuv_half);


    av_init_packet(&avpkt);
    //avpkt.data = image.ptr();

    int got_frame = 0;
    int len = FRAME_WIDTH * FRAME_HEIGHT;
    got_frame = 1;
    frame->width = FRAME_WIDTH;
    frame->height = FRAME_HEIGHT;

    assert(frame->width == FRAME_WIDTH);
    assert(frame->height == FRAME_HEIGHT);
    const int buf_idx = tbuffer_select(tb);
    rear_camera->camera_bufs_metadata[buf_idx] = {
      .frame_id = static_cast<uint32_t>(frame_id),
      .timestamp_eof = nanos_since_boot(),
      .frame_length = 0,
      .integ_lines = 0,
      .global_gain = 0,
    };
    cl_mem yuv_cl = rear_camera->yuv_cls[buf_idx];
    cl_event map_event;
    void *yuv_buf = (void *)clEnqueueMapBuffer(rear_camera->q, yuv_cl, CL_TRUE,
                                                CL_MAP_WRITE, 0, frame->width * frame->height * 3 / 2,
                                                0, NULL, &map_event, &err);
    assert(err == 0);
    clWaitForEvents(1, &map_event);
    clReleaseEvent(map_event);
    uint8_t *write_ptr = (uint8_t *)yuv_buf;


    for(int line_idx = 0; line_idx < frame->height; line_idx++) {
      memcpy(write_ptr, yuv_channels[0].ptr(line_idx), frame->width);
      write_ptr += frame->width;
    }

    for(int line_idx = 0; line_idx < frame->height/2; line_idx++) {
      memcpy(write_ptr, yuv_half[1].ptr(line_idx), frame->width / 2);
      write_ptr += frame->width/2;
    }
    for(int line_idx = 0; line_idx < frame->height/2;line_idx++) {
      memcpy(write_ptr, yuv_half[2].ptr(line_idx), frame->width / 2);
      write_ptr += frame->width/2;
    }
    clEnqueueUnmapMemObject(rear_camera->q, yuv_cl, yuv_buf, 0, NULL, &map_event);
    clWaitForEvents(1, &map_event);
    clReleaseEvent(map_event);
    tbuffer_dispatch(tb, buf_idx);
    free(payload);
    image.release();
    image_half.release();
    yuv_channels[0].release();
    yuv_channels[1].release();
    yuv_channels[2].release();
    yuv_half[0].release();
    yuv_half[1].release();
    yuv_half[2].release();

}
  av_frame_free(&frame);
}

}  // namespace

CameraInfo cameras_supported[CAMERA_ID_MAX] = {
  [CAMERA_ID_IMX298] = {
      .frame_width = FRAME_WIDTH,
      .frame_height = FRAME_HEIGHT,
      .frame_stride = FRAME_WIDTH*3,
      .bayer = false,
      .bayer_flip = false,
  },
};

void cameras_init(DualCameraState *s) {
  memset(s, 0, sizeof(*s));

  camera_init(&s->rear, CAMERA_ID_IMX298, 20);
  s->rear.transform = (mat3){{
    1.0,  0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0,  0.0, 1.0,
  }};
}

void camera_autoexposure(CameraState *s, float grey_frac) {}

void cameras_open(DualCameraState *s, cl_mem *yuv_cls_rear, cl_device_id device_id, cl_context context, cl_command_queue q) {
  assert(yuv_cls_rear);
  int err;

  camera_open(&s->rear, yuv_cls_rear, true, device_id, context, q);
}

void cameras_close(DualCameraState *s) {
  camera_close(&s->rear);
}

void cameras_run(DualCameraState *s) {
  set_thread_name("Eon streaming");
  run_eon_stream(s);
  cameras_close(s);
}
