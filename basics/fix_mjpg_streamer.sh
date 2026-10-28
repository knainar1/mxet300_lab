#!/bin/bash
# MXET 300 Lab 8 - rebuild mjpg-streamer with the OpenCV input plugin
#
# The version of mjpg-streamer on the SCUTTLE image was built without
# input_opencv.so and cvfilter_py.so, which Lab 8 needs. Upstream hard-requires
# OpenCV 3 and this image ships OpenCV 4, so the source needs three small
# patches before it will build.
#
# Run once per robot:   bash fix_mjpg_streamer.sh

set -e

echo "=== Step 1 of 4: checking build dependencies ==="
sudo apt-get update || true
sudo apt-get install -y cmake build-essential libjpeg-dev libopencv-dev \
    python3-dev python3-numpy

echo
echo "=== Step 2 of 4: downloading mjpg-streamer source ==="
SRC="$HOME/mjpg-streamer-fix"
rm -rf "$SRC"
git clone --depth 1 https://github.com/jacksonliam/mjpg-streamer.git "$SRC"
cd "$SRC/mjpg-streamer-experimental"

echo
echo "=== Step 3 of 4: patching for OpenCV 4 ==="

# The OpenCV input plugin is gated on OpenCV major version 3. Drop the gate.
sed -i 's/ONLYIF OpenCV_FOUND ${OpenCV_VERSION_MAJOR} EQUAL 3/ONLYIF OpenCV_FOUND/' \
    plugins/input_opencv/CMakeLists.txt

# CV_IMWRITE_JPEG_QUALITY was removed in OpenCV 4.
sed -i 's/CV_IMWRITE_JPEG_QUALITY/cv::IMWRITE_JPEG_QUALITY/' \
    plugins/input_opencv/input_opencv.cpp

# The Python filter has the same kind of broken version gate.
sed -i 's/ONLYIF PYTHONLIBS_FOUND NUMPY_FOUND ${PYTHON_VERSION_MAJOR} EQUAL 3/ONLYIF PYTHONLIBS_FOUND NUMPY_FOUND/' \
    plugins/input_opencv/filters/cvfilter_py/CMakeLists.txt

# OpenCV 4 changed MatAllocator::allocate from int flags to cv::AccessFlag,
# so NumpyAllocator no longer overrides it and the class becomes abstract.
CONV="plugins/input_opencv/filters/cvfilter_py/conversion.cpp"
sed -i 's/size_t\* step, int flags, UMatUsageFlags usageFlags/size_t* step, AccessFlag flags, UMatUsageFlags usageFlags/' "$CONV"
sed -i 's/bool allocate(UMatData\* u, int accessFlags, UMatUsageFlags usageFlags)/bool allocate(UMatData* u, AccessFlag accessFlags, UMatUsageFlags usageFlags)/' "$CONV"

echo "Patches applied."

echo
echo "=== Step 4 of 4: building and installing (this takes a few minutes) ==="
rm -rf _build
cmake -B _build -DCMAKE_BUILD_TYPE=Release .
make -C _build -j2
sudo make -C _build install

echo
if [ -f /usr/local/lib/mjpg-streamer/input_opencv.so ] && \
   [ -f /usr/local/lib/mjpg-streamer/cvfilter_py.so ]; then
    echo "SUCCESS. Both plugins are installed:"
    ls -1 /usr/local/lib/mjpg-streamer/input_opencv.so
    ls -1 /usr/local/lib/mjpg-streamer/cvfilter_py.so
    echo
    echo "You can now continue with the lab."
else
    echo "FAILED. One or both plugins are missing from"
    echo "/usr/local/lib/mjpg-streamer/. Show this output to your TA."
    exit 1
fi