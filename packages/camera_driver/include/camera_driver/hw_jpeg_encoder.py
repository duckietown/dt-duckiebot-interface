"""Minimal V4L2 M2M driver for the Pi's hardware JPEG encoder (/dev/video31)."""
import ctypes, fcntl, mmap, os, select

import numpy as np

VIDEO_MAX_PLANES = 8
BUF_TYPE_CAPTURE_MPLANE = 9
BUF_TYPE_OUTPUT_MPLANE = 10
MEMORY_MMAP = 1

def fourcc(s):
    return ord(s[0]) | (ord(s[1]) << 8) | (ord(s[2]) << 16) | (ord(s[3]) << 24)

PIX_BGR24 = fourcc("BGR3")   # byte order B,G,R == OpenCV layout
PIX_JPEG = fourcc("JPEG")
CID_JPEG_QUALITY = 0x009d0903  # V4L2_CID_JPEG_CLASS_BASE(0x009d0900) + 3


class PlanePixFormat(ctypes.Structure):
    _fields_ = [("sizeimage", ctypes.c_uint32), ("bytesperline", ctypes.c_uint32),
                ("reserved", ctypes.c_uint16 * 6)]

class PixFormatMplane(ctypes.Structure):
    _fields_ = [("width", ctypes.c_uint32), ("height", ctypes.c_uint32),
                ("pixelformat", ctypes.c_uint32), ("field", ctypes.c_uint32),
                ("colorspace", ctypes.c_uint32),
                ("plane_fmt", PlanePixFormat * VIDEO_MAX_PLANES),
                ("num_planes", ctypes.c_uint8), ("flags", ctypes.c_uint8),
                ("ycbcr_enc", ctypes.c_uint8), ("quantization", ctypes.c_uint8),
                ("xfer_func", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 7)]

class FormatUnion(ctypes.Union):
    _fields_ = [("pix_mp", PixFormatMplane), ("raw_data", ctypes.c_uint8 * 200)]

class Format(ctypes.Structure):
    # the kernel union contains pointer members (v4l2_window), so on 64-bit it is
    # 8-byte aligned: 4 bytes of padding sit between `type` and `fmt`, making the
    # struct 208 bytes rather than the 204 a naive layout gives.
    _fields_ = [("type", ctypes.c_uint32), ("_pad", ctypes.c_uint32),
                ("fmt", FormatUnion)]

class RequestBuffers(ctypes.Structure):
    _fields_ = [("count", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("memory", ctypes.c_uint32), ("capabilities", ctypes.c_uint32),
                ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]

class PlaneUnion(ctypes.Union):
    _fields_ = [("mem_offset", ctypes.c_uint32), ("userptr", ctypes.c_ulong),
                ("fd", ctypes.c_int32)]

class Plane(ctypes.Structure):
    _fields_ = [("bytesused", ctypes.c_uint32), ("length", ctypes.c_uint32),
                ("m", PlaneUnion), ("data_offset", ctypes.c_uint32),
                ("reserved", ctypes.c_uint32 * 11)]

class Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

class Timecode(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("flags", ctypes.c_uint32),
                ("frames", ctypes.c_uint8), ("seconds", ctypes.c_uint8),
                ("minutes", ctypes.c_uint8), ("hours", ctypes.c_uint8),
                ("userbits", ctypes.c_uint8 * 4)]

class BufferUnion(ctypes.Union):
    _fields_ = [("offset", ctypes.c_uint32), ("userptr", ctypes.c_ulong),
                ("planes", ctypes.POINTER(Plane)), ("fd", ctypes.c_int32)]

class Buffer(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("bytesused", ctypes.c_uint32), ("flags", ctypes.c_uint32),
                ("field", ctypes.c_uint32), ("timestamp", Timeval),
                ("timecode", Timecode), ("sequence", ctypes.c_uint32),
                ("memory", ctypes.c_uint32), ("m", BufferUnion),
                ("length", ctypes.c_uint32), ("reserved2", ctypes.c_uint32),
                ("request_fd", ctypes.c_int32)]

class Control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]

def _iowr(nr, size):
    return (3 << 30) | (size << 16) | (ord("V") << 8) | nr

def _iow(nr, size):
    return (1 << 30) | (size << 16) | (ord("V") << 8) | nr

VIDIOC_S_FMT = _iowr(5, ctypes.sizeof(Format))
VIDIOC_REQBUFS = _iowr(8, ctypes.sizeof(RequestBuffers))
VIDIOC_QUERYBUF = _iowr(9, ctypes.sizeof(Buffer))
VIDIOC_QBUF = _iowr(15, ctypes.sizeof(Buffer))
VIDIOC_DQBUF = _iowr(17, ctypes.sizeof(Buffer))
VIDIOC_STREAMON = _iow(18, 4)
VIDIOC_STREAMOFF = _iow(19, 4)
VIDIOC_S_CTRL = _iowr(28, ctypes.sizeof(Control))


class InvalidJpegOutputError(RuntimeError):
    """The V4L2 encoder completed a frame without a usable JPEG payload."""


class HardwareJpegEncoder:
    def __init__(self, width, height, quality=90, device="/dev/video31", out_size=None):
        if out_size is None:
            # a q90 frame measures well under a byte per pixel, but a noisy one can
            # run much larger, so the buffer scales with the resolution rather than
            # sitting at what one sensor needed: 480x640 keeps the 768 KiB that was
            # measured, 1296x972 (watchtower, traffic light) gets 1.8 MiB.
            out_size = max(768 * 1024, width * height * 3 // 2)
        self.w, self.h, self.out_size = width, height, out_size
        self.device = device
        self.quality_applied = True
        self.in_map = None
        self.out_map = None
        self.fd = os.open(device, os.O_RDWR)
        try:
            f = Format(type=BUF_TYPE_OUTPUT_MPLANE)
            f.fmt.pix_mp.width, f.fmt.pix_mp.height = width, height
            f.fmt.pix_mp.pixelformat = PIX_BGR24
            f.fmt.pix_mp.num_planes = 1
            f.fmt.pix_mp.plane_fmt[0].bytesperline = width * 3
            f.fmt.pix_mp.plane_fmt[0].sizeimage = width * height * 3
            fcntl.ioctl(self.fd, VIDIOC_S_FMT, f)
            self.in_size = f.fmt.pix_mp.plane_fmt[0].sizeimage
            # the driver may pad rows for alignment. we feed it a tightly packed
            # numpy buffer, so anything else would be read with the wrong stride
            # and come out skewed. bail out and let the caller use software.
            if (f.fmt.pix_mp.plane_fmt[0].bytesperline != width * 3
                    or self.in_size != width * height * 3):
                raise RuntimeError(
                    f"encoder wants padded input (bytesperline "
                    f"{f.fmt.pix_mp.plane_fmt[0].bytesperline}, sizeimage {self.in_size}) "
                    f"for {width}x{height}, expected {width * 3} and {width * height * 3}"
                )

            f = Format(type=BUF_TYPE_CAPTURE_MPLANE)
            f.fmt.pix_mp.width, f.fmt.pix_mp.height = width, height
            f.fmt.pix_mp.pixelformat = PIX_JPEG
            f.fmt.pix_mp.num_planes = 1
            f.fmt.pix_mp.plane_fmt[0].sizeimage = out_size
            fcntl.ioctl(self.fd, VIDIOC_S_FMT, f)
            self.out_size = f.fmt.pix_mp.plane_fmt[0].sizeimage

            try:
                fcntl.ioctl(self.fd, VIDIOC_S_CTRL, Control(id=CID_JPEG_QUALITY, value=quality))
            except OSError:
                # some kernels expose quality only via ext-ctrls; the block then falls
                # back to its own default of 80, so the caller is told about it
                self.quality_applied = False

            self.in_map = self._setup(BUF_TYPE_OUTPUT_MPLANE)
            self.out_map = self._setup(BUF_TYPE_CAPTURE_MPLANE)
            self._prealloc()

            for t in (BUF_TYPE_OUTPUT_MPLANE, BUF_TYPE_CAPTURE_MPLANE):
                fcntl.ioctl(self.fd, VIDIOC_STREAMON, ctypes.c_int(t))
        except BaseException:
            try:
                for mapping in (self.in_map, self.out_map):
                    if mapping is not None:
                        try:
                            mapping.close()
                        except Exception:
                            pass
            finally:
                try:
                    os.close(self.fd)
                except OSError:
                    pass
            raise

    def _setup(self, btype):
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS,
                    RequestBuffers(count=1, type=btype, memory=MEMORY_MMAP))
        planes = (Plane * VIDEO_MAX_PLANES)()
        b = Buffer(index=0, type=btype, memory=MEMORY_MMAP, length=1)
        b.m.planes = planes
        fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
        return mmap.mmap(self.fd, planes[0].length, mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE, offset=planes[0].m.mem_offset)

    def _prealloc(self):
        """Per-frame ioctl structs, built once: rebuilding them showed up in the profile."""
        self._planes, self._qb, self._dqb = {}, {}, {}
        for btype, size in ((BUF_TYPE_OUTPUT_MPLANE, self.in_size),
                            (BUF_TYPE_CAPTURE_MPLANE, self.out_size)):
            qplanes = (Plane * VIDEO_MAX_PLANES)()
            qplanes[0].length = size
            qbuf = Buffer(index=0, type=btype, memory=MEMORY_MMAP, length=1)
            qbuf.m.planes = qplanes
            dplanes = (Plane * VIDEO_MAX_PLANES)()
            dbuf = Buffer(type=btype, memory=MEMORY_MMAP, length=1)
            dbuf.m.planes = dplanes
            self._planes[btype] = (qplanes, dplanes)
            self._qb[btype], self._dqb[btype] = qbuf, dbuf

    def _qbuf(self, btype, bytesused):
        self._planes[btype][0][0].bytesused = bytesused
        fcntl.ioctl(self.fd, VIDIOC_QBUF, self._qb[btype])

    def _dqbuf(self, btype):
        fcntl.ioctl(self.fd, VIDIOC_DQBUF, self._dqb[btype])
        return self._planes[btype][1][0].bytesused

    def encode(self, image) -> bytes:
        """image: HxWx3 uint8 BGR array. Copied into the input buffer, then encoded.

        Writing into the mmap and letting the block read from there is the fast path:
        rotating straight into this buffer is 4x slower because it is uncached DMA memory.
        """
        view = memoryview(image)
        if not view.c_contiguous:
            view = memoryview(np.ascontiguousarray(image))
        flat = view.cast("B")
        self.in_map.seek(0)
        self.in_map.write(flat)
        self._qbuf(BUF_TYPE_CAPTURE_MPLANE, 0)
        self._qbuf(BUF_TYPE_OUTPUT_MPLANE, flat.nbytes)
        ready, _, _ = select.select([self.fd], [], [], 2.0)
        if not ready:
            # the fd is blocking, so dequeuing now would hang the caller's event loop
            raise TimeoutError("hardware JPEG encoder did not complete within 2s")
        self._dqbuf(BUF_TYPE_OUTPUT_MPLANE)
        n = self._dqbuf(BUF_TYPE_CAPTURE_MPLANE)
        if n == 0 or n > self.out_size:
            raise InvalidJpegOutputError(
                f"hardware JPEG encoder returned invalid output size: {n}"
            )
        return self.out_map[:n]

    def close(self):
        for t in (BUF_TYPE_OUTPUT_MPLANE, BUF_TYPE_CAPTURE_MPLANE):
            try:
                fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, ctypes.c_int(t))
            except OSError:
                pass  # best effort: STREAMOFF fails if the stream was never started
        for m in (self.in_map, self.out_map):
            m.close()
        os.close(self.fd)
