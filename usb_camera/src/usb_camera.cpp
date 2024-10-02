/*
 * Copyright 2024 Myeong Jin Lee
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../include/usb_camera/usb_camera.hpp"

usb_cam::usb_cam() : streaming(false), m_fd(-1)
{
}

usb_cam::~usb_cam()
{
}

int usb_cam::xioctl(int fd, unsigned long request, void * arg)
{
  int r;
  do {
    r = ioctl(fd, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

m_deviceInfo usb_cam::get_device_info(const std::string & devicePath)
{
  m_deviceInfo devInfo;
  int fd = open(devicePath.c_str(), O_RDWR);
  if (fd == -1) {
    CERR_ENDL("Failed to open device: " << devicePath);
    return devInfo;
  }

  struct v4l2_capability cap;
  if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
    CERR_ENDL("Failed to get device capabilities. Error: " << strerror(errno));
    close(fd);
    return devInfo;
  }

  devInfo.device_name = (char *)cap.card;
  devInfo.driver = (char *)cap.driver;
  devInfo.bus_info = (char *)cap.bus_info;

  struct v4l2_fmtdesc fmt;
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.index = 0;

  while (xioctl(fd, VIDIOC_ENUM_FMT, &fmt) != -1) {
    std::string fmt_desc = (char *)fmt.description;
    devInfo.formats.push_back(fmt_desc);

    struct v4l2_frmsizeenum frmsize;
    frmsize.pixel_format = fmt.pixelformat;
    frmsize.index = 0;

    while (xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != -1) {
      ResolutionInfo resInfo;
      if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
        resInfo.resolution = std::make_pair(frmsize.discrete.width, frmsize.discrete.height);

        struct v4l2_frmivalenum frmival;
        frmival.pixel_format = fmt.pixelformat;
        frmival.width = frmsize.discrete.width;
        frmival.height = frmsize.discrete.height;
        frmival.index = 0;

        while (xioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) != -1) {
          if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
            float fps =
              static_cast<float>(frmival.discrete.denominator) / frmival.discrete.numerator;
            resInfo.fps.push_back(fps);
          }
          frmival.index++;
        }

        if (!resInfo.fps.empty()) {
          devInfo.resolution_info.push_back(resInfo);
        }
      }
      frmsize.index++;
    }
    fmt.index++;
  }

  close(fd);
  return devInfo;
}

v4l2_stream_err usb_cam::start_stream(const m_deviceConfig & config)
{
  m_fd = open(config.path.c_str(), O_RDWR);
  if (m_fd == -1) {
    CERR_ENDL("Failed to open device: " << config.path);
    return STREAM_ERR_NO_DEVICE;
  }

  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config.resolution.first;
  fmt.fmt.pix.height = config.resolution.second;

  std::string normalized_format = normalize_format(config.format);

  if (normalized_format == "mjpeg") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  } else if (normalized_format == "yuyv") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  } else if (normalized_format == "h264") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
  } else if (normalized_format == "nv12") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
  } else if (normalized_format == "rgb24") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
  } else if (normalized_format == "grey") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
  } else if (normalized_format == "uyvy") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
  } else if (normalized_format == "yvyu") {
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YVYU;
  } else {
    CERR_ENDL("Unsupported format: " << config.format);
    close(m_fd);
    return STREAM_ERR_FORMAT;
  }

  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (xioctl(m_fd, VIDIOC_S_FMT, &fmt) == -1) {
    CERR_ENDL("Failed to set format");
    close(m_fd);
    return STREAM_ERR_FORMAT;
  }

  struct v4l2_streamparm streamparm;
  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  streamparm.parm.capture.timeperframe.numerator = 1;
  streamparm.parm.capture.timeperframe.denominator = static_cast<int>(config.fps);

  if (xioctl(m_fd, VIDIOC_S_PARM, &streamparm) == -1) {
    CERR_ENDL("Failed to set frame rate");
    close(m_fd);
    return STREAM_ERR_FPS;
  }

  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (xioctl(m_fd, VIDIOC_REQBUFS, &req) == -1) {
    CERR_ENDL("Failed to request buffers");
    close(m_fd);
    return STREAM_ERR_REQUEST_BUFFER;
  }

  buffers.resize(req.count);
  buffer_lengths.resize(req.count);

  for (unsigned int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (xioctl(m_fd, VIDIOC_QUERYBUF, &buf) == -1) {
      CERR_ENDL("Failed to query buffer");
      close(m_fd);
      return STREAM_ERR_QUERY_BUFFER;
    }

    buffers[i] = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, buf.m.offset);
    buffer_lengths[i] = buf.length;

    if (buffers[i] == MAP_FAILED) {
      CERR_ENDL("Failed to map buffer");
      close(m_fd);
      return STREAM_ERR_MAP_BUFFER;
    }

    if (xioctl(m_fd, VIDIOC_QBUF, &buf) == -1) {
      CERR_ENDL("Failed to queue buffer");
      close(m_fd);
      return STREAM_ERR_QUEUE_BUFFER;
    }
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(m_fd, VIDIOC_STREAMON, &type) == -1) {
    CERR_ENDL("Failed to start streaming");
    close(m_fd);
    return STREAM_ERR_STREAM_FAIL;
  }

  streaming = true;

  // Start the streaming thread
  stream_thread = std::thread([this, config]() {
    while (streaming) {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (xioctl(m_fd, VIDIOC_DQBUF, &buf) == -1) {
        CERR_ENDL("Failed to dequeue buffer");
        break;
      }

      cv::Mat img(cv::Size(config.resolution.first, config.resolution.second), CV_8UC3);
      img = cv::imdecode(cv::Mat(1, buf.bytesused, CV_8UC1, buffers[buf.index]), cv::IMREAD_COLOR);
      m_image = img.clone();

      if (xioctl(m_fd, VIDIOC_QBUF, &buf) == -1) {
        CERR_ENDL("Failed to queue buffer");
        break;
      }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(m_fd, VIDIOC_STREAMOFF, &type);
    close(m_fd);
  });

  return STREAM_OK;
}

void usb_cam::stop_stream()
{
  if (!streaming) {
    return;
  }

  streaming = false;

  if (stream_thread.joinable()) {
    stream_thread.join();
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(m_fd, VIDIOC_STREAMOFF, &type) == -1) {
    CERR_ENDL("Failed to stop streaming");
  }

  for (size_t i = 0; i < buffers.size(); ++i) {
    if (buffers[i] != MAP_FAILED) {
      munmap(buffers[i], buffer_lengths[i]);
    }
  }

  buffers.clear();
  buffer_lengths.clear();
  m_image.release();

  if (m_fd != -1) {
    close(m_fd);
    m_fd = -1;
  }
}

bool usb_cam::set_control(int control_id, int value)
{
  struct v4l2_control control;
  control.id = control_id;
  control.value = value;

  std::string control_name = get_control_name(control_id);

  if (xioctl(m_fd, VIDIOC_S_CTRL, &control) == -1) {
    CERR_ENDL(
      "Failed to set control (" << control_name << ", ID: " << control_id
                                << "): " << strerror(errno));
    return false;
  }

  return true;
}

bool usb_cam::set_control(std::string control_name, int value)
{
  return set_control(get_control_by_name(control_name), value);
}

int usb_cam::get_control(int control_id)
{
  struct v4l2_control control;
  control.id = control_id;

  std::string control_name = get_control_name(control_id);

  if (xioctl(m_fd, VIDIOC_G_CTRL, &control) == -1) {
    CERR_ENDL(
      "Failed to get control (" << control_name << ", ID: " << control_id
                                << "): " << strerror(errno));
    return -1;
  }

  return control.value;
}

int usb_cam::get_control(std::string control_name)
{
  return get_control(get_control_by_name(control_name));
}

bool usb_cam::query_control(int control_id, v4l2_queryctrl & queryctrl)
{
  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = control_id;

  std::string control_name = get_control_name(control_id);

  if (ioctl(m_fd, VIDIOC_QUERYCTRL, &queryctrl) == -1) {
    if (errno != EINVAL) {
      CERR_ENDL(
        "Failed to query control (" << control_name << ", ID: " << control_id
                                    << "): " << strerror(errno));
    } else {
      CERR_ENDL("Control (" << control_name << ", ID: " << control_id << ") is not supported.");
    }
    queryctrl.flags |= V4L2_CTRL_FLAG_DISABLED;
    return false;
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    CERR_ENDL("Control (" << control_name << ", ID: " << control_id << ") is disabled.");
    return false;
  }
  return true;
}

bool usb_cam::query_control(std::string control_name, v4l2_queryctrl & queryctrl)
{
  return query_control(get_control_by_name(control_name), queryctrl);
}

void usb_cam::reset_controls_to_default()
{
  struct v4l2_queryctrl queryctrl;
  memset(&queryctrl, 0, sizeof(queryctrl));

  for (queryctrl.id = V4L2_CID_BASE; queryctrl.id < V4L2_CID_LASTP1; queryctrl.id++) {
    if (ioctl(m_fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
      if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
        set_control(queryctrl.id, queryctrl.default_value);
      }
    }
  }

  for (queryctrl.id = V4L2_CID_PRIVATE_BASE; queryctrl.id < V4L2_CID_PRIVATE_BASE + 20;
       queryctrl.id++) {
    if (ioctl(m_fd, VIDIOC_QUERYCTRL, &queryctrl) == 0) {
      if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
        set_control(queryctrl.id, queryctrl.default_value);
      }
    }
  }
}

std::string usb_cam::get_control_name(int control_id)
{
  switch (control_id) {
    case V4L2_CID_BRIGHTNESS:
      return "Brightness";
    case V4L2_CID_CONTRAST:
      return "Contrast";
    case V4L2_CID_SATURATION:
      return "Saturation";
    case V4L2_CID_HUE:
      return "Hue";
    case V4L2_CID_GAMMA:
      return "Gamma";
    case V4L2_CID_SHARPNESS:
      return "Sharpness";
    case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
      return "White Balance Temperature";
    case V4L2_CID_AUTO_WHITE_BALANCE:
      return "Auto White Balance";
    case V4L2_CID_EXPOSURE_ABSOLUTE:
      return "Exposure (Absolute)";
    case V4L2_CID_EXPOSURE_AUTO:
      return "Auto Exposure";
    case V4L2_CID_EXPOSURE_AUTO_PRIORITY:
      return "Auto Exposure Priority";
    case V4L2_CID_POWER_LINE_FREQUENCY:
      return "Power Line Frequency";
    case V4L2_CID_BACKLIGHT_COMPENSATION:
      return "Backlight Compensation";
    case V4L2_CID_FOCUS_ABSOLUTE:
      return "Focus (Absolute)";
    case V4L2_CID_FOCUS_AUTO:
      return "Auto Focus";
    case V4L2_CID_ZOOM_ABSOLUTE:
      return "Zoom (Absolute)";
    case V4L2_CID_PAN_ABSOLUTE:
      return "Pan (Absolute)";
    case V4L2_CID_TILT_ABSOLUTE:
      return "Tilt (Absolute)";
    case V4L2_CID_PRIVACY:
      return "Privacy";
    case V4L2_CID_ROTATE:
      return "Rotate";
    case V4L2_CID_HFLIP:
      return "Horizontal Flip";
    case V4L2_CID_VFLIP:
      return "Vertical Flip";
    case V4L2_CID_COLOR_KILLER:
      return "Color Killer";
    case V4L2_CID_COLORFX:
      return "Color Effects";
    case V4L2_CID_AUTOGAIN:
      return "Auto Gain";
    case V4L2_CID_GAIN:
      return "Gain";
    case V4L2_CID_HUE_AUTO:
      return "Auto Hue";
    case V4L2_CID_RED_BALANCE:
      return "Red Balance";
    case V4L2_CID_BLUE_BALANCE:
      return "Blue Balance";
    case V4L2_CID_DO_WHITE_BALANCE:
      return "Do White Balance";
    case V4L2_CID_AUTOBRIGHTNESS:
      return "Auto Brightness";
    case V4L2_CID_BAND_STOP_FILTER:
      return "Band Stop Filter";
    case V4L2_CID_ILLUMINATORS_1:
      return "Illuminators 1";
    case V4L2_CID_ILLUMINATORS_2:
      return "Illuminators 2";
    case V4L2_CID_ISO_SENSITIVITY:
      return "ISO Sensitivity";
    case V4L2_CID_ISO_SENSITIVITY_AUTO:
      return "Auto ISO Sensitivity";
    case V4L2_CID_EXPOSURE_METERING:
      return "Exposure Metering";
    case V4L2_CID_SCENE_MODE:
      return "Scene Mode";
    case V4L2_CID_3A_LOCK:
      return "3A Lock (Auto Exposure, White Balance, and Focus Lock)";
    case V4L2_CID_AUTO_FOCUS_START:
      return "Start Auto Focus";
    case V4L2_CID_AUTO_FOCUS_STOP:
      return "Stop Auto Focus";
    case V4L2_CID_AUTO_FOCUS_RANGE:
      return "Auto Focus Range";
    default:
      return "Unknown Control";
  }
}

int usb_cam::get_control_by_name(std::string name)
{
  if (name == "brightness") {
    return V4L2_CID_BRIGHTNESS;
  } else if (name == "contrast") {
    return V4L2_CID_CONTRAST;
  } else if (name == "saturation") {
    return V4L2_CID_SATURATION;
  } else if (name == "hue") {
    return V4L2_CID_HUE;
  } else if (name == "gamma") {
    return V4L2_CID_GAMMA;
  } else if (name == "sharpness") {
    return V4L2_CID_SHARPNESS;
  } else if (name == "whitebalance") {
    return V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  } else if (name == "auto_whitebalance") {
    return V4L2_CID_AUTO_WHITE_BALANCE;
  } else if (name == "exposure") {
    return V4L2_CID_EXPOSURE_ABSOLUTE;
  } else if (name == "auto_exposure") {
    return V4L2_CID_EXPOSURE_AUTO;
  } else if (name == "auto_exposure_priority") {
    return V4L2_CID_EXPOSURE_AUTO_PRIORITY;
  } else if (name == "power_line_frequency") {
    return V4L2_CID_POWER_LINE_FREQUENCY;
  } else if (name == "backlight_compensation") {
    return V4L2_CID_BACKLIGHT_COMPENSATION;
  } else if (name == "focus") {
    return V4L2_CID_FOCUS_ABSOLUTE;
  } else if (name == "auto_focus") {
    return V4L2_CID_FOCUS_AUTO;
  } else if (name == "zoom") {
    return V4L2_CID_ZOOM_ABSOLUTE;
  } else if (name == "pan") {
    return V4L2_CID_PAN_ABSOLUTE;
  } else if (name == "tilt") {
    return V4L2_CID_TILT_ABSOLUTE;
  } else if (name == "privacy") {
    return V4L2_CID_PRIVACY;
  } else if (name == "rotate") {
    return V4L2_CID_ROTATE;
  } else if (name == "horizontal_flip") {
    return V4L2_CID_HFLIP;
  } else if (name == "vertical_flip") {
    return V4L2_CID_VFLIP;
  } else if (name == "color_killer") {
    return V4L2_CID_COLOR_KILLER;
  } else if (name == "colorfx") {
    return V4L2_CID_COLORFX;
  } else if (name == "autogain") {
    return V4L2_CID_AUTOGAIN;
  } else if (name == "gain") {
    return V4L2_CID_GAIN;
  } else if (name == "auto_hue") {
    return V4L2_CID_HUE_AUTO;
  } else if (name == "red_balance") {
    return V4L2_CID_RED_BALANCE;
  } else if (name == "blue_balance") {
    return V4L2_CID_BLUE_BALANCE;
  } else if (name == "do_white_balance") {
    return V4L2_CID_DO_WHITE_BALANCE;
  } else if (name == "auto_brightness") {
    return V4L2_CID_AUTOBRIGHTNESS;
  } else if (name == "band_stop_filter") {
    return V4L2_CID_BAND_STOP_FILTER;
  } else if (name == "illuminators_1") {
    return V4L2_CID_ILLUMINATORS_1;
  } else if (name == "illuminators_2") {
    return V4L2_CID_ILLUMINATORS_2;
  } else if (name == "iso_sensitivity") {
    return V4L2_CID_ISO_SENSITIVITY;
  } else if (name == "iso_sensitivity_auto") {
    return V4L2_CID_ISO_SENSITIVITY_AUTO;
  } else if (name == "exposure_metering") {
    return V4L2_CID_EXPOSURE_METERING;
  } else if (name == "scene_mode") {
    return V4L2_CID_SCENE_MODE;
  } else if (name == "3a_lock") {
    return V4L2_CID_3A_LOCK;
  } else if (name == "auto_focus_start") {
    return V4L2_CID_AUTO_FOCUS_START;
  } else if (name == "auto_focus_stop") {
    return V4L2_CID_AUTO_FOCUS_STOP;
  } else if (name == "auto_focus_range") {
    return V4L2_CID_AUTO_FOCUS_RANGE;
  }
  return -1;
}

std::string usb_cam::normalize_format(const std::string & format)
{
  static std::unordered_map<std::string, std::string> format_map = {
    {"mjpeg", "mjpeg"}, {"motion-jpeg", "mjpeg"}, {"yuyv", "yuyv"}, {"yuyv 4:2:2", "yuyv"},
    {"h264", "h264"},   {"h.264", "h264"},        {"nv12", "nv12"}, {"rgb24", "rgb24"},
    {"grey", "grey"},   {"uyvy", "uyvy"},         {"yvyu", "yvyu"}};

  std::string format_lower = format;
  std::transform(format_lower.begin(), format_lower.end(), format_lower.begin(), ::tolower);

  auto it = format_map.find(format_lower);
  if (it != format_map.end()) {
    return it->second;
  }
  return format_lower;
}

bool usb_cam::is_format_supported(
  const std::string & supported_format, const std::string & requested_format)
{
  return normalize_format(supported_format) == normalize_format(requested_format);
}
