# OpenCV 4.8 or higher Installation Guide (YOLO Support and ROS Dependency Reconfiguration)

This guide explains the steps to install **OpenCV 4.8 or higher** from source to enable Yolov8 ONNX model functionalities in the `image_recognition` ROS package. It covers the reinstallation of OpenCV and ROS dependencies to ensure smooth integration. **Existing ROS and OpenCV packages will be removed**, so reconfiguration may be required.

---

## ⚠️ Important Notice

- **ROS2 does not support changing OpenCV version.** Installing OpenCV manully might break ROS2 environment. **Few ROS packages might not work properly**. 
- **OpenCV 4.8.0 or higher is recommended** for YOLO and DNN compatibility.
---

## Step-by-Step Installation (OpenCV 4.10.0)
### 1. Download OpenCV 4.10.0 Source Code

Make sure you use **version 4.8 or higher** to ensure YOLO and DNN compatibility.

```bash
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.10.0.zip
unzip opencv.zip
cd opencv-4.10.0
```

### 2. Create a Build Directory

```bash
mkdir build && cd build
```

### 3. Configure OpenCV Build Options

Configure the build with YOLO support, video capture libraries, and ROS-friendly options. For CUDA support, refer to the [official OpenCV CUDA guide](https://opencv.org/).

```bash
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_TBB=OFF \
      -D WITH_IPP=OFF \
      -D WITH_1394=OFF \
      -D BUILD_WITH_DEBUG_INFO=OFF \
      -D BUILD_DOCS=OFF \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_PACKAGE=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D WITH_QT=OFF \
      -D WITH_GTK=ON \
      -D WITH_OPENGL=ON \
      -D BUILD_opencv_python3=ON \
      -D WITH_V4L=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_XINE=ON \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D BUILD_NEW_PYTHON_SUPPORT=ON \
      -D OPENCV_SKIP_PYTHON_LOADER=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON ../
```

### 4. Compile OpenCV

Compiling OpenCV can take some time depending on your hardware. Use all available CPU cores for faster compilation:

```bash
time make -j$(nproc)
```

### 5. Install OpenCV

Once the build completes, install it:

```bash
sudo make install
sudo ldconfig
```

---

## Post-Installation Configuration

### 1. Set `PKG_CONFIG_PATH` for ROS Compatibility

Ensure ROS detects the new OpenCV installation by updating the `PKG_CONFIG_PATH`:

```bash
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
```

To make this change permanent, add the above line to your `~/.bashrc`:

```bash
echo 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Change CMake Configuration for `cv_bridge`
Make sure the ROS `cv_bridge` package links to the newly installed OpenCV libraries:

```bash
sudo nano /opt/ros/humble/share/cv_bridge/cmake/cv_bridge-extras.cmake 
```

Change the file content to:
```cmake
set(OpenCV_VERSION 4.10.0)
set(OpenCV_VERSION_MAJOR 4)
set(OpenCV_VERSION_MINOR 10)
set(OpenCV_VERSION_PATCH 0)
set(OpenCV_SHARED ON)
set(OpenCV_CONFIG_PATH /usr/local/lib/cmake/opencv4)
set(OpenCV_INSTALL_PATH /usr)
set(OpenCV_LIB_COMPONENTS opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_barcode;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_datasets;opencv_dnn_objdetect;opencv_dnn_superres;opencv_dpm;opencv_face;opencv_freetype;opencv_fuzzy;opencv_hdf;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_mcc;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_text;opencv_tracking;opencv_videostab;opencv_viz;opencv_wechat_qrcode;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto)
set(OpenCV_USE_MANGLED_PATHS FALSE)
set(OpenCV_MODULES_SUFFIX )
```

## Final Verification

1. **Test OpenCV Installation:**

   Launch Python and verify OpenCV version:

   ```bash
   python3 -c "import cv2; print(cv2.__version__)"
   ```

   You should see `4.10.0` as the output.

2. **Verify YOLO Model Compatibility:**

   Test loading a YOLO model with OpenCV's DNN module:

   ```python
   import cv2
   net = cv2.dnn.readNet('yolov8n.onnx')  # Use an appropriate YOLO model
   print("YOLO model loaded successfully")
   ```

3. **Rebuild ROS Packages:**

   If needed, rebuild your ROS workspace to ensure everything works with the new OpenCV installation:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```