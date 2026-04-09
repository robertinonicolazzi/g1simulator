# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# The work in this file is based on cv_bridge:
# https://github.com/ros-perception/vision_opencv/tree/kinetic/cv_bridge
# Parts of the code were taken and modified. The original license
# is reproduced below.
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# Copyright (c) 2016, Tal Regev.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import sys

import cv2
import numpy

CV_CN_SHIFT = 3
CV_CN_MAX = 512

# CV types
CV_8U = 0
CV_8S = 1
CV_16U = 2
CV_16S = 3
CV_32S = 4
CV_32F = 5
CV_64F = 6


class CvBridgeCustom:
    def __init__(self):
        self.cvdepth_to_numpy_depth = {
            cv2.CV_8U: "uint8",
            cv2.CV_8S: "int8",
            cv2.CV_16U: "uint16",
            cv2.CV_16S: "int16",
            cv2.CV_32S: "int32",
            cv2.CV_32F: "float32",
            cv2.CV_64F: "float64",
        }

        self._encoding_flags = {
            "mono8": "GRAY",
            "mono16": "GRAY",
            "bgr8": "BGR",
            "bgr16": "BGR",
            "rgb8": "RGB",
            "rgb16": "RGB",
            "bgra8": "BGRA",
            "bgra16": "BGRA",
            "yuv422": "YUV422",
            "bayer_rggb8": "BAYER_RGGB",
            "bayer_rggb16": "BAYER_RGGB",
            "bayer_bggr8": "BAYER_BGGR",
            "bayer_bggr16": "BAYER_BGGR",
            "bayer_gbrg8": "BAYER_GBRG",
            "bayer_gbrg16": "BAYER_GBRG",
            "bayer_grbg8": "BAYER_GRBG",
            "bayer_grbg16": "BAYER_GRBG",
        }

        self._color_conversion_codes = {
            "GRAY2RGB": cv2.COLOR_GRAY2RGB,
            "GRAY2BGR": cv2.COLOR_GRAY2BGR,
            "GRAY2RGBA": cv2.COLOR_GRAY2RGBA,
            "GRAY2BGRA": cv2.COLOR_GRAY2BGRA,
            "RGB2GRAY": cv2.COLOR_RGB2GRAY,
            "RGB2BGR": cv2.COLOR_RGB2BGR,
            "RGB2RGBA": cv2.COLOR_RGB2RGBA,
            "RGB2BGRA": cv2.COLOR_RGB2BGRA,
            "BGR2GRAY": cv2.COLOR_BGR2GRAY,
            "BGR2RGB": cv2.COLOR_BGR2RGB,
            "BGR2RGBA": cv2.COLOR_BGR2RGBA,
            "BGR2BGRA": cv2.COLOR_BGR2BGRA,
            "RGBA2GRAY": cv2.COLOR_RGBA2GRAY,
            "RGBA2RGB": cv2.COLOR_RGBA2RGB,
            "RGBA2BGR": cv2.COLOR_RGBA2BGR,
            "RGBA2BGRA": cv2.COLOR_RGBA2BGRA,
            "BGRA2GRAY": cv2.COLOR_BGRA2GRAY,
            "BGRA2RGB": cv2.COLOR_BGRA2RGB,
            "BGRA2BGR": cv2.COLOR_BGRA2BGR,
            "BGRA2RGBA": cv2.COLOR_BGRA2RGBA,
            "YUV2GRAY_UYVY": cv2.COLOR_YUV2GRAY_UYVY,
            "YUV2RGB_UYVY": cv2.COLOR_YUV2RGB_UYVY,
            "YUV2BGR_UYVY": cv2.COLOR_YUV2BGR_UYVY,
            "YUV2RGBA_UYVY": cv2.COLOR_YUV2RGBA_UYVY,
            "YUV2BGRA_UYVY": cv2.COLOR_YUV2BGRA_UYVY,
        }

    def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
        """
        Converts a ros image message to an openCV format.
        If desired_encoding is "passthrough", then the returned image has the same
        format as img_msg. Otherwise desired_encoding must be one of the standard
        image encodings.
        """

        dtype, n_channels = self._encoding_to_dtype_with_channels(img_msg.encoding)
        dtype = numpy.dtype(dtype)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        # Full row length in bytes
        step = img_msg.step

        if n_channels == 1:
            strides = (step, dtype.itemsize)
            im = numpy.ndarray(
                shape=(img_msg.height, img_msg.width),
                dtype=dtype,
                buffer=img_msg.data,
                strides=strides,
            )
        else:
            strides = (step, n_channels, dtype.itemsize)
            im = numpy.ndarray(
                shape=(img_msg.height, img_msg.width, n_channels),
                dtype=dtype,
                buffer=img_msg.data,
                strides=strides,
            )
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        # Add hack to avoid degrading colors when working with opencv
        if desired_encoding == "rgb8":
            desired_encoding = "bgr8"

        if desired_encoding == "passthrough" or img_msg.encoding == desired_encoding:
            return im

        try:
            img_encoding = img_msg.encoding

            # GRAYSCALE DEPTH IMAGES
            # Return these as they come from source, without attempting
            # to do any conversion to rgb/bgr encodings.

            # 8UC1 encoding can be considered equal to mono8 as stated here
            # https://github.com/ros-perception/vision_opencv/blob/melodic/cv_bridge/src/cv_bridge.cpp#L671
            if img_encoding == "8UC1":
                return im

            # Do scaling between CV_16U/mono16 [0,65535] and mono8 [0,255] images.
            if img_encoding == "16UC1":
                return numpy.uint8(im / 257)

            # If the data type is single precision float, convert it to
            # uint8 (single channel).
            if img_encoding == "32FC1":
                return numpy.uint8(im)

            conversion_flag = self._get_conversion_flag(img_encoding, desired_encoding)

            if conversion_flag == "SAME_FORMAT":
                image_converted = im
            else:
                image_converted = cv2.cvtColor(im, conversion_flag)
        except Exception as e:
            raise Exception(f"Could not encode {img_encoding} image to {desired_encoding}")

        return image_converted

    def compressed_imgmsg_to_cv2(self, cmprs_img_msg, desired_encoding="passthrough"):
        """
        Converts a ros compressed image message to an openCV format.
        If desired_encoding is "passthrough", then the returned image has the same
        format as img_msg. Otherwise desired_encoding must be one of the standard
        image encodings.
        """

        str_msg = cmprs_img_msg.data
        buf = numpy.ndarray(shape=(1, len(str_msg)), dtype=numpy.uint8, buffer=cmprs_img_msg.data)
        im = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)

        # Assume the data encoding is rgb8, since we don't have information
        # on it.
        img_encoding = "rgb8"

        if desired_encoding == "passthrough" or img_encoding == desired_encoding:
            return im

        try:
            conversion_flag = self._get_conversion_flag(img_encoding, desired_encoding)

            if conversion_flag == "SAME_FORMAT":
                image_converted = im
            else:
                image_converted = cv2.cvtColor(im, conversion_flag)
        except Exception as e:
            raise Exception(f"Could not encode compressed image to {desired_encoding}")

        return image_converted

    def _encoding_to_dtype_with_channels(self, encoding):
        return self._cvtype2_to_dtype_with_channels(self._encoding_to_cvtype2(encoding))

    def _cvtype2_to_dtype_with_channels(self, cvtype):
        return self.cvdepth_to_numpy_depth[self._cv_mat_depth(cvtype)], self._cv_mat_cn(cvtype)

    def _encoding_to_cvtype2(self, encoding):
        encoding_to_type_params = {
            "bgr8": (CV_8U, 3),
            "mono8": (CV_8U, 1),
            "rgb8": (CV_8U, 3),
            "mono16": (CV_16U, 1),
            "bgr16": (CV_16U, 3),
            "rgb16": (CV_16U, 3),
            "bgra8": (CV_8U, 4),
            "rgba8": (CV_8U, 4),
            "bgra16": (CV_16U, 4),
            "rgba16": (CV_16U, 4),
            "bayer_rggb8": (CV_8U, 1),
            "bayer_bggr8": (CV_8U, 1),
            "bayer_gbrg8": (CV_8U, 1),
            "bayer_grbg8": (CV_8U, 1),
            "bayer_rggb16": (CV_16U, 1),
            "bayer_bggr16": (CV_16U, 1),
            "bayer_gbrg16": (CV_16U, 1),
            "bayer_grbg16": (CV_16U, 1),
            "yuv422": (CV_8U, 2),
            "8UC1": (CV_8U, 1),
            "8UC2": (CV_8U, 2),
            "8UC3": (CV_8U, 3),
            "8UC4": (CV_8U, 4),
            "8SC1": (CV_8S, 1),
            "8SC2": (CV_8S, 2),
            "8SC3": (CV_8S, 3),
            "8SC4": (CV_8S, 4),
            "16UC1": (CV_16U, 1),
            "16UC2": (CV_16U, 2),
            "16UC3": (CV_16U, 3),
            "16UC4": (CV_16U, 4),
            "16SC1": (CV_16S, 1),
            "16SC2": (CV_16S, 2),
            "16SC3": (CV_16S, 3),
            "16SC4": (CV_16S, 4),
            "32FC1": (CV_32F, 1),
            "32FC2": (CV_32F, 2),
            "32FC3": (CV_32F, 3),
            "32FC4": (CV_32F, 4),
            "32SC1": (CV_32S, 1),
            "32SC2": (CV_32S, 2),
            "32SC3": (CV_32S, 3),
            "32SC4": (CV_32S, 4),
            "64FC1": (CV_64F, 1),
            "64FC2": (CV_64F, 2),
            "64FC3": (CV_64F, 3),
            "64FC4": (CV_64F, 4),
        }

        params = encoding_to_type_params.get(encoding)
        if params is not None:
            return self._cv_make_type(*params)
        else:
            raise Exception("Unrecognized image encoding [" + encoding + "]")

    def _cv_make_type(self, depth, cn):
        return self._cv_mat_depth(depth) + (((cn) - 1) << CV_CN_SHIFT)

    def _cv_mat_depth(self, flags):
        return (flags) & ((1 << CV_CN_SHIFT) - 1)

    def _cv_mat_cn(self, flags):
        return (((flags) & ((CV_CN_MAX - 1) << CV_CN_SHIFT)) >> CV_CN_SHIFT) + 1

    def _get_conversion_flag(self, source_encoding, dest_encoding):
        """
        Returns a flag used to convert an image from one cv2 encoding format to
        another. If the source or destiny encodings are not supported it returns
        None.
        """

        source_fmt = self._encoding_flags.get(source_encoding)
        dest_fmt = self._encoding_flags.get(dest_encoding)

        if source_fmt is None or dest_fmt is None:
            return None

        if source_fmt == dest_fmt:
            return "SAME_FORMAT"

        common_fmts = ["GRAY", "RGB", "BGR"]
        common_fmts_with_opacity = ["RGBA", "BGRA"]

        if (
            source_fmt in common_fmts
            or source_fmt in common_fmts_with_opacity
            and dest_fmt in common_fmts
            or dest_fmt in common_fmts_with_opacity
        ):
            return self._color_conversion_codes[f"{source_fmt}2{dest_fmt}"]

        elif (
            source_fmt == "YUV422"
            and dest_fmt in common_fmts
            or dest_fmt in common_fmts_with_opacity
        ):
            return self._color_conversion_codes[f"YUV2{dest_fmt}_UYVY"]

        elif "BAYER" in source_fmt:
            try:
                special_conversion_codes = {
                    "BayerBG2GRAY": cv2.COLOR_BayerBG2GRAY,
                    "BayerBG2RGB": cv2.COLOR_BayerBG2RGB,
                    "BayerBG2BGR": cv2.COLOR_BayerBG2BGR,
                    "BayerRG2GRAY": cv2.COLOR_BayerRG2GRAY,
                    "BayerRG2RGB": cv2.COLOR_BayerRG2RGB,
                    "BayerRG2BGR": cv2.COLOR_BayerRG2BGR,
                    "BayerGR2GRAY": cv2.COLOR_BayerGR2GRAY,
                    "BayerGR2RGB": cv2.COLOR_BayerGR2RGB,
                    "BayerGR2BGR": cv2.COLOR_BayerGR2BGR,
                    "BayerGB2GRAY": cv2.COLOR_BayerGB2GRAY,
                    "BayerGB2RGB": cv2.COLOR_BayerGB2RGB,
                    "BayerGB2BGR": cv2.COLOR_BayerGB2BGR,
                }
            except Exception as e:
                raise Exception("Current OpenCV distribution does not support " "Bayer format.")

            if source_fmt == "BAYER_RGGB" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerBG2{dest_fmt}"]

            elif source_fmt == "BAYER_BGGR" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerRG2{dest_fmt}"]

            elif source_fmt == "BAYER_GBRG" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerGR2{dest_fmt}"]

            elif source_fmt == "BAYER_GRBG" and dest_fmt in common_fmts:
                return special_conversion_codes[f"BayerGB2{dest_fmt}"]
        else:
            return None
