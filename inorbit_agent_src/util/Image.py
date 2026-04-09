# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Image processing wrapper.
# Uses cv2 if available, if not uses Pillow (PIL fork).
import imghdr

import numpy
from util.overrides import overrides

# Package for image processing used (cv2 or PIL)
PACKAGE = None

# Checking OpenCv availability
try:
    import cv2
except Exception as e:
    pass
else:
    from util.CvBridgeCustom import CvBridgeCustom

    PACKAGE = "cv2"

# If not available, use PIL
if not PACKAGE:
    try:
        from PIL import Image as Im
        from PIL import ImageDraw
        from PIL import ImageEnhance
    except Exception as e:
        pass
    else:
        import io
        import sys

        PACKAGE = "PIL"

supported_img_encodings = ["jpg", "jpeg", "png"]

"""
This is the class to be used to process images. The interface is defined in
ImageBase class.
If OpenCv is available, cv2ImageWrapper is used as Image class. If not,
PILImageWrapper is used as image class.
If none of them is available, an ImageError exception is raised.
"""
Image = None


class ImageError(Exception):
    """
    This is the error raised by :class:`cv_bridge.CvBridge` methods when
    failing.
    """

    pass


class ImageBase:
    """
    Pure virtual class used to define the Image class interface
    """

    # Indicates the package being used
    _PACKAGE = None

    # Default parameters
    DEFAULT_CONTRAST = None
    DEFAULT_BRIGHTNESS = None

    def __init__(self):
        # Object containing the opened image.
        self._im = None
        # Memory buffer with the image encoded to jpeg.
        self._buf = None
        # Source (original) encoding of the image.
        self._source_format = None

    def open(self, image_path):
        """
        Opens a image from image_path, and saves the object in _im.
        Raises an IOError if the image can't be opened.
        """

        raise NotImplementedError()

    def resize(self, target_size):
        """
        Resizes the image, keeping its aspect ratio.
        target_size: Targeted (width, height).

        TODO(ivanpauno): Actually when using cv2, we are expanding images if they
                        are smaller than the specified size, and when using PIL
                        we aren't.
        """

        raise NotImplementedError()

    def encode(self, skip_conversion, quality):
        """
        Saves an internal buffer with the image encoded to jpg by default (if
        skip_conversion flag is false) or to the source format (skip_conversion
        is True), with a given compression quality.
        quality:
            - JPG/JPEG: compression quality of the image (0 --> 100 -highest-).
            - PNG: compression level (0 --> 9 -not compressed-)
        """

        raise NotImplementedError()

    def toBytes(self):
        """
        Returns saved memory buffer _buf as bytes.
        """

        raise NotImplementedError()

    def fromImgMsg(self, img_msg, output_encoding):
        """
        Opens an image from a ROS message. Also loaded in _im.
        img_msg: ROS message of type sensor_msgs/Image
        output_encoding: the encoding to convert the image to.
        """

        raise NotImplementedError()

    def fromCompressedImgMsg(self, img_msg, output_encoding):
        """
        Opens an image from a ROS message (COMPRESSED). Also loaded in _im.
        img_msg: ROS message of type sensor_msgs/CompressedImage
        output_encoding: the encoding to convert the image to.
        TODO (Flor_Grosso): implement this method for PILImageWrapper.
        """

        raise NotImplementedError()

    def fillPolygons(self, base_img, contours, default_color):
        """
        It receives an image to use as background, a list of objects (contours)
        with 'data' (polygon's vertices) and 'cost' values to fill the polygons
        with, and a default color to be used in case no fill color is
        provided.
        With this input, it draws and fills the polygons over the base image and
        returns the resulting one.
        """

        raise NotImplementedError()

    def isFormatSupported(self, fmt):
        """
        Checks if an image format is supported by the Image module.
        """

        return fmt in supported_img_encodings

    def enhance(self, contrast, brightness):
        """
        Adjusts contrast and brightness of the image.
        """

        raise NotImplementedError()


class PILImageWrapper(ImageBase):
    """
    This class allows simple image processing with PIL.
    """

    _PACKAGE = "PIL"

    DEFAULT_CONTRAST = 1.0
    DEFAULT_BRIGHTNESS = 1.0

    @overrides(ImageBase)
    def open(self, image_path):
        self._im = Im.open(image_path)
        self._source_format = self._im.format

        return self._source_format.lower()

    @overrides(ImageBase)
    def resize(self, target_size):
        # NOTE (Flor_Grosso): this PIL's method already takes the aspect ratio
        # preservation into account, so there's no need to handle that
        # externally.
        # For reference, check:
        # http://effbot.org/imagingbook/image.htm#tag-Image.Image.thumbnail
        # TODO (FlorGrosso): check whether the image needs to be resized
        # or if it has already the expected size, avoiding unnecessary
        # processing.
        self._im.thumbnail(target_size)

    @overrides(ImageBase)
    def encode(self, skip_conversion, quality):
        if self._im is None:
            return

        # If it is not a JPEG supported encoding, convert the encoding to RGB.
        # PIL doesn't convert it automaticaly when saving to JPEG.
        if self._im.mode not in ("L", "RGB", "CMYK"):
            self._im = self._im.convert("RGB")

        # If the no conversion flag is active, preserve source encoding.
        # If not, convert to jpg by default.
        if skip_conversion and self._source_format:
            out_format = self._source_format
        else:
            out_format = "JPEG"

        # Check if the output encoding is supported by this module
        if not out_format or out_format.lower() not in supported_img_encodings:
            return None

        with io.BytesIO() as output:
            if out_format == "JPEG" or out_format == "JPG":
                self._im.save(output, format=out_format, quality=quality)
            elif out_format == "PNG":
                self._im.save(output, format=out_format, compress_level=quality)

            self._buf = output.getvalue()

    @overrides(ImageBase)
    def toBytes(self):
        if self._buf is None:
            return None
        return self._buf

    @overrides(ImageBase)
    def fromImgMsg(self, img_msg, output_encoding):
        """
        This code was adapted from CvBridgeCustom.imgmsg_to_cv2 method
        (check ./CvBridgeCustom.py) in order to work with PIL.
        """

        # Map encoding from sensor_msgs/Image to
        # (numpy.dtype, channels, PILmode).
        # TODO(ivanpauno): Add support to other encoding types.
        encoding_to_dtype_Nchannels_PILmode = {
            "bgr8": ("uint8", 3, "RGB"),
            "mono8": ("uint8", 1, "L"),
            "rgb8": ("uint8", 3, "RGB"),
            "mono16": ("uint16", 1, "L"),
            "bgr16": ("uint16", 3, "RGB"),
            "rgb16": ("uint16", 3, "RGB"),
            "bgra8": None,
            "rgba8": ("uint8", 4, "RGBA"),
            "bgra16": None,
            "rgba16": None,
            "bayer_rggb8": None,
            "bayer_bggr8": None,
            "bayer_gbrg8": None,
            "bayer_grbg8": None,
            "bayer_rggb16": None,
            "bayer_bggr16": None,
            "bayer_gbrg16": None,
            "bayer_grbg16": None,
            "yuv422": None,
            "8UC1": ("uint8", 1, "L"),
            "8UC2": ("uint8", 2, "L"),
            "8UC3": ("uint8", 3, "L"),
            "8UC4": ("uint8", 4, "L"),
            "8SC1": ("int8", 1, "IS;8"),
            "8SC2": ("int8", 2, "IS;8"),
            "8SC3": ("int8", 3, "IS;8"),
            "8SC4": ("int8", 4, "IS;8"),
            "16UC1": ("uint16", 1, "I;16"),
            "16UC2": ("uint16", 2, "I;16"),
            "16UC3": ("uint16", 3, "I;16"),
            "16UC4": ("uint16", 4, "I;16"),
            "16SC1": ("int16", 1, "IS;16"),
            "16SC2": ("int16", 2, "IS;16"),
            "16SC3": ("int16", 3, "IS;16"),
            "16SC4": ("int16", 4, "IS;16"),
            "32FC1": ("float32", 1, "F"),
            "32FC2": ("float32", 2, "F"),
            "32FC3": ("float32", 3, "F"),
            "32FC4": ("float32", 4, "F"),
            "32SC1": ("int32", 1, "I"),
            "32SC2": ("int32", 2, "I"),
            "32SC3": ("int32", 3, "I"),
            "32SC4": ("int32", 4, "I"),
            "64FC1": ("float64", 1, "F"),
            "64FC2": ("float64", 2, "F"),
            "64FC3": ("float64", 3, "F"),
            "64FC4": ("float64", 4, "F"),
        }

        # Get the encoding of the image
        rv = encoding_to_dtype_Nchannels_PILmode[img_msg.encoding]

        # Raise NotImplementedError if encoding wasn't supported.
        if rv is None:
            raise ImageError(f"Encoding {img_msg.encoding}: not supported.")

        dtype, n_channels, mode = rv

        # Get appropiated data type for numpy
        dtype = numpy.dtype(dtype)
        dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")

        # Full row length in bytes
        step = img_msg.step

        # Create array depending on number of channels (mono=1, rgb=3, rgba=4)
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
        # Swap bytes if endian type is different
        if img_msg.is_bigendian == (sys.byteorder == "little"):
            im = im.byteswap().newbyteorder()

        try:
            # Do scaling between CV_16U [0,65535] and mono8 [0,255] images.
            # TODO (Flor_Grosso): this is a special case when converting
            # depth images to mono8. Consider extending this to other formats.
            if img_msg.encoding == "16UC1":
                im = im / 257

            # Create PIL Image object from the array data.
            im = Im.fromarray(im, mode)

            # Convert to output_encoding if necessary.
            if img_msg.encoding != output_encoding:
                if output_encoding == "mono8":
                    im = im.convert("L")
                elif output_encoding == "rgb8":
                    im = im.convert("RGB")
        except Exception as e:
            raise ImageError(f"Could not encode {img_msg.encoding} image to {output_encoding}.")

        self._im = im

    @overrides(ImageBase)
    def fillPolygons(self, base_img, contours, default_cost):
        # Create PIL Image object
        base = Im.fromarray(base_img.astype("uint8"))

        # PIL supports drawing a polygon at a time.
        for polygon in contours:
            # Convert list of list into list of tupples, to match required
            # input format.
            coords = list(map(tuple, polygon.get("data", [])))
            color = polygon.get("color", default_cost)
            ImageDraw.Draw(base).polygon(coords, outline=color, fill=color)

        return numpy.array(base)

    @overrides(ImageBase)
    def enhance(self, brightness, contrast):
        """
        Adjust brightness and contrast of the image
        """
        if contrast != self.DEFAULT_CONTRAST:
            """
            A factor of less than 1.0 makes the image darker (and a value of
            0.0 makes the image completely black). A factor of greater than
            1.0 makes the image brighter. A factor of exactly 1.0 leaves the
            original image unchanged.
            """
            self._im = ImageEnhance.Contrast(self._im).enhance(contrast)
        if brightness != self.DEFAULT_BRIGHTNESS:
            """
            Factors greater than 1.0 make the image brighter, less than 1.0
            makes the image darker. A factor of 0.0 results in a completely
            black image:
            """
            self._im = ImageEnhance.Contrast(self._im).enhance(brightness)


class cv2ImageWrapper(ImageBase):
    """
    This class allows simple image processing with cv2 (OpenCV).
    """

    _PACKAGE = "cv2"

    DEFAULT_CONTRAST = 1.0
    DEFAULT_BRIGHTNESS = 0

    def __init__(self):
        ImageBase.__init__(self)
        # HACK(herchu): Initialize the cv2 encodings only if an object of this class
        # is ever constructed. Otherwise, it will fail if `cv` is not available
        self._cv2_encodings_to_compression_quality = {
            "jpg": int(cv2.IMWRITE_JPEG_QUALITY),  # 0(lowest) to 100(highest)
            "jpeg": int(cv2.IMWRITE_JPEG_QUALITY),
            "png": int(
                cv2.IMWRITE_PNG_COMPRESSION
            ),  # 0 (no compression) to 9 (highest compression)
        }

    @overrides(ImageBase)
    def open(self, image_path):
        self._im = cv2.imread(image_path)
        self._source_format = imghdr.what(image_path)

        return self._source_format

    @overrides(ImageBase)
    def resize(self, target_size):
        if self._im is None:
            return

        if self._should_resize_to(target_size):
            width, height = self._compute_output_image_size(target_size)
            # Check if resizing should take place based on the image's
            # current dimensions and the expected size.
            self._im = cv2.resize(self._im, (width, height), interpolation=cv2.INTER_LINEAR)

    @overrides(ImageBase)
    def encode(self, skip_conversion, quality):
        if self._im is None:
            return

        # If the no conversion flag is active, preserve source encoding.
        # If not, convert to jpg by default.
        if skip_conversion and self._source_format:
            out_format = self._source_format
        else:
            out_format = "jpg"

        if out_format not in supported_img_encodings:
            return

        self._buf = cv2.imencode(
            "." + out_format,
            self._im,
            [self._cv2_encodings_to_compression_quality[out_format], quality],
        )[1]

    @overrides(ImageBase)
    def toBytes(self):
        if self._buf is None:
            return None
        return self._buf.tobytes()

    @overrides(ImageBase)
    def fromImgMsg(self, img_msg, output_encoding="passthrough"):
        bridge = CvBridgeCustom()
        self._im = bridge.imgmsg_to_cv2(img_msg, output_encoding)

    @overrides(ImageBase)
    def fromCompressedImgMsg(self, img_msg, output_encoding="passthrough"):
        bridge = CvBridgeCustom()
        self._im = bridge.compressed_imgmsg_to_cv2(img_msg, output_encoding)

    """
    Computes the size of the output image, keeping the aspect ratio.
    """

    def _compute_output_image_size(self, target_size):
        output_width = target_size[0]
        output_height = target_size[1]

        src_height, src_width = self._im.shape[:2]
        # Preserve the aspect ratio of the image by setting its larger
        # dimension to the default values and the other proportional to
        # this scaling.
        if src_width > src_height:
            if src_width > 0:
                output_height = max(1, int(src_height * float(output_width) / src_width))
        else:
            if src_height > 0:
                output_width = max(1, int(src_width * float(output_height) / src_height))

        # TODO (Flor_Grosso): Update custom image config with output width and
        # height to report it. Pay attention to how this config affects
        # images differently according to its mode (landscape or portrait),
        # since it is a general setting for now.

        return output_width, output_height

    def _should_resize_to(self, target_size):
        """
        Checks if the image should be resized to the output width and height,
        depending on the dimensions of the source.
        If the source height and width is equal to the output's, no resizing
        is required.
        """

        src_height, src_width = self._im.shape[:2]
        output_width = target_size[0]
        output_height = target_size[1]

        return not (
            src_width == output_width
            and src_height == output_height
            or output_width > src_width
            or output_height > src_height
        )

    @overrides(ImageBase)
    def fillPolygons(self, base_img, contours, default_cost):
        # Make sure the base image is received as a matrix that can be
        # processed by fillPoly
        image = numpy.ascontiguousarray(base_img, dtype=numpy.uint8)

        # Fill one polygon at a time, since they might have different number of
        # vertices and that breaks the nested array data source.
        for polygon in contours:
            # Round coordinates to integers that match matrix indexes.
            # Note that we use floor here since matrix indexes go from
            # 0...map_width - 1 and 0...map_height - 1
            coords = numpy.array(
                [
                    [(numpy.floor(float(i))).astype(int) for i in vertices]
                    for vertices in polygon.get("data", [])
                ]
            )
            if coords.size == 0:
                continue
            cv2.fillPoly(image, [coords], polygon.get("color", default_cost))

        return image

    @overrides(ImageBase)
    def enhance(self, brightness, contrast):
        """
        Adjust contrast and brightness of the source image by
        scaling and adding an offset to it, respectively:

        out_img = alpha * in_img + beta

        alpha (0-100) controls contrast and beta (1.0-3.0) brightness
        """
        if contrast or brightness != self.DEFAULT_BRIGHTNESS:
            self._im = cv2.convertScaleAbs(self._im, alpha=contrast, beta=brightness)


# Let "Image" class being an alias of one of the Wrappers
if PACKAGE == "PIL":
    Image = PILImageWrapper
elif PACKAGE == "cv2":
    Image = cv2ImageWrapper
else:
    raise ImageError("Neither OpenCv nor PIL found in the system.")
