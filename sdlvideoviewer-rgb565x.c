/**
 * Copyright (C) 2012 by Tomasz Moń <desowin@gmail.com>
 *
 * compile with:
 *   gcc -o sdlvideoviewer-rgb565x sdlvideoviewer-rgb565x.c -lSDL
 *
 * Based on V4L2 video capture example and process-vmalloc.c
 * Capture+output (process) V4L2 device tester.
 *
 * Pawel Osciak, p.osciak <at> samsung.com
 * 2009, Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version
 */

#include <SDL/SDL.h>
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

#include <endian.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define __USE_BSD

#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define max(a, b) (a > b ? a : b)
#define min(a, b) (a > b ? b : a)

typedef enum
{
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
} io_method;

struct buffer
{
    void *start;
    size_t length;
};

static char *dev_name = NULL;
static char *mem2mem_dev_name = NULL;
static io_method io = IO_METHOD_MMAP;
static int fd = -1;
struct buffer *buffers = NULL;
static unsigned int n_buffers = 0;

static int hflip = 0;
static int vflip = 0;

static size_t WIDTH = 640;
static size_t HEIGHT = 240;
/* Spacing between input and output display */
#define SEPARATOR 10

/* 
 * data stored in V4L2_PIX_FMT_RGB565X-like format
 *
 * Difference between original format is that higher three bits of green
 * are swapped with lower three bits of green.
 */
static uint16_t *buffer_sdl;
static uint16_t *buffer_m2m_sdl;

SDL_Surface *data_sf;
SDL_Surface *data_m2m_sf;

#define V4L2_CID_TRANS_TIME_MSEC        (V4L2_CID_PRIVATE_BASE)
#define V4L2_CID_TRANS_NUM_BUFS         (V4L2_CID_PRIVATE_BASE + 1)

#define NUM_BUFS	4

#define perror_exit(cond, func)\
	if (cond) {\
		fprintf(stderr, "%s:%d: ", __func__, __LINE__);\
		perror(func);\
		exit(EXIT_FAILURE);\
	}

#define error_exit(cond, func)\
	if (cond) {\
		fprintf(stderr, "%s:%d: failed\n", func, __LINE__);\
		exit(EXIT_FAILURE);\
	}

#define perror_ret(cond, func)\
	if (cond) {\
		fprintf(stderr, "%s:%d: ", __func__, __LINE__);\
		perror(func);\
		return ret;\
	}

#define memzero(x)\
	memset(&(x), 0, sizeof (x));

#define PROCESS_DEBUG 1
#ifdef PROCESS_DEBUG
#define debug(msg, ...)\
	fprintf(stderr, "%s: " msg, __func__, ##__VA_ARGS__);
#else
#define debug(msg, ...)
#endif

static int mem2mem_fd;
static char *p_src_buf[NUM_BUFS], *p_dst_buf[NUM_BUFS];
static size_t src_buf_size[NUM_BUFS], dst_buf_size[NUM_BUFS];
static uint32_t num_src_bufs = 0, num_dst_bufs = 0;

/* transize = WIDTH*HEIGHT/translen*2 */
static int transsize;

/* Command-line params */
int translen = 1;
/* For displaying multi-buffer transaction simulations, indicates current
   buffer in an ongoing transaction */
int curr_buf = 0;
int transtime = 1;
int num_frames = 1000;


static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));

    exit(EXIT_FAILURE);
}

static int xioctl(int fd, int request, void *arg)
{
    int r;

    do
    {
        r = ioctl(fd, request, arg);
    }
    while (-1 == r && EINTR == errno);

    return r;
}


static void render(SDL_Surface * pre, SDL_Surface * post)
{
    SDL_Rect rect_pre = {
        .x = 0,.y = 0,
        .w = WIDTH,.h = HEIGHT
    };

    SDL_Rect rect_post = {
        .x = 0,.y = HEIGHT + SEPARATOR,
        .w = WIDTH,.h = HEIGHT
    };

    SDL_Surface *screen = SDL_GetVideoSurface();

    SDL_BlitSurface(pre, NULL, screen, &rect_pre);
    SDL_BlitSurface(post, NULL, screen, &rect_post);

    SDL_UpdateRect(screen, 0, 0, 0, 0);
}



void YCbCrToRGB(int y, int cb, int cr, uint8_t * r, uint8_t * g, uint8_t * b)
{
    double Y = (double)y;
    double Cb = (double)cb;
    double Cr = (double)cr;

    int R = (int)(Y + 1.40200 * (Cr - 0x80));
    int G = (int)(Y - 0.34414 * (Cb - 0x80) - 0.71414 * (Cr - 0x80));
    int B = (int)(Y + 1.77200 * (Cb - 0x80));

    *r = max(0, min(255, R));
    *g = max(0, min(255, G));
    *b = max(0, min(255, B));
}

/* 
 * YCbCr to RGB lookup table
 *
 * Indexes are [Y][Cb][Cr]
 * Y, Cb, Cr range is 0-255
 *
 * Stored value bits:
 *   24-16 Red
 *   15-8  Green
 *   7-0   Blue
 */
uint32_t YCbCr_to_RGB[256][256][256];

static void generate_YCbCr_to_RGB_lookup()
{
    int y;
    int cb;
    int cr;

    for (y = 0; y < 256; y++)
    {
        for (cb = 0; cb < 256; cb++)
        {
            for (cr = 0; cr < 256; cr++)
            {
                double Y = (double)y;
                double Cb = (double)cb;
                double Cr = (double)cr;

                int R = (int)(Y + 1.40200 * (Cr - 0x80));
                int G =
                    (int)(Y - 0.34414 * (Cb - 0x80) - 0.71414 * (Cr - 0x80));
                int B = (int)(Y + 1.77200 * (Cb - 0x80));

                R = max(0, min(255, R));
                G = max(0, min(255, G));
                B = max(0, min(255, B));

                YCbCr_to_RGB[y][cb][cr] = R << 16 | G << 8 | B;
            }
        }
    }
}

#define COLOR_GET_RED(color)   ((color >> 16) & 0xFF)
#define COLOR_GET_GREEN(color) ((color >> 8) & 0xFF)
#define COLOR_GET_BLUE(color)  (color & 0xFF)

/**
 * Converts RGB888 color to RGB565
 */
static uint16_t inline RGB888_to_RGB565(uint32_t rgb)
{
    uint16_t tmp;

    tmp = ((rgb >> 3) & 0x1F) << 11 |   /* Blue */
        ((rgb >> 10) & 0x3F) << 5 | /* Green */
        ((rgb >> 19) & 0x1F);   /* Red */

#if __BYTE_ORDER == __LITTLE_ENDIAN
    /* In LE lower byte is stored under lower address */
    tmp = ((tmp >> 8) & 0xFF) | ((tmp & 0xFF) << 8);
#elif __BYTE_ORDER == __BIG_ENDIAN
    /* Nothing to do */
#else
#error "Unknown Endianess"
#endif

    return tmp;
}

/**
 *  Converts YUV422 to RGB565
 *  Before first use call generate_YCbCr_to_RGB_lookup();
 *
 *  input is pointer to YUV422 encoded data in following order: Y0, Cb, Y1, Cr.
 *  output is pointer to 16 bit RGB565X buffer.
 */
static void inline YUV422_to_RGB565(uint16_t * output, const uint8_t * input)
{
    uint8_t y0 = input[0];
    uint8_t cb = input[1];
    uint8_t y1 = input[2];
    uint8_t cr = input[3];

    uint32_t rgb = YCbCr_to_RGB[y0][cb][cr];
    output[0] = RGB888_to_RGB565(rgb);

    rgb = YCbCr_to_RGB[y1][cb][cr];
    output[1] = RGB888_to_RGB565(rgb);
}

static void process_image(const void *p)
{
    const uint8_t *buffer_yuv = p;

    size_t x;
    size_t y;

    for (y = 0; y < HEIGHT; y++)
        for (x = 0; x < WIDTH; x += 2)
            YUV422_to_RGB565(&buffer_sdl[y * WIDTH + x],
                             buffer_yuv + (y * WIDTH + x) * 2);
}

static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io)
    {
    case IO_METHOD_READ:
        if (-1 == read(fd, buffers[0].start, buffers[0].length))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit("read");
            }
        }

        process_image(buffers[0].start);

        break;

    case IO_METHOD_MMAP:
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit("VIDIOC_DQBUF");
            }
        }

        assert(buf.index < n_buffers);

        process_image(buffers[buf.index].start);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

        break;

    case IO_METHOD_USERPTR:
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit("VIDIOC_DQBUF");
            }
        }

        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long)buffers[i].start
                && buf.length == buffers[i].length)
                break;

        assert(i < n_buffers);

        process_image((void *)buf.m.userptr);

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

        break;
    }

    return 1;
}

static void read_input_frame(void)
{
    for (;;)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO(&fds);
        FD_SET(fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select(fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;

            errno_exit("select");
        }

        if (0 == r)
        {
            fprintf(stderr, "select timeout\n");
            exit(EXIT_FAILURE);
        }

        if (read_frame())
            break;

        /* EAGAIN - continue select loop. */
    }
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    switch (io)
    {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF");

        break;
    }
}

static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    switch (io)
    {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
        {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");

        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
        {
            struct v4l2_buffer buf;

            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.index = i;
            buf.m.userptr = (unsigned long)buffers[i].start;
            buf.length = buffers[i].length;

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");

        break;
    }
}

static void uninit_device(void)
{
    unsigned int i;

    switch (io)
    {
    case IO_METHOD_READ:
        free(buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap(buffers[i].start, buffers[i].length))
                errno_exit("munmap");
        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free(buffers[i].start);
        break;
    }

    free(buffers);
}

static void init_read(unsigned int buffer_size)
{
    buffers = calloc(1, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc(buffer_size);

    if (!buffers[0].start)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }
}

static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                    "memory mapping\n", dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    if (req.count < 2)
    {
        fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
        exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
            errno_exit("VIDIOC_QUERYBUF");

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(NULL /* start anywhere */ ,
                                        buf.length, PROT_READ | PROT_WRITE  /* required 
                                                                             */ ,
                                        MAP_SHARED /* recommended */ ,
                                        fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            errno_exit("mmap");
    }
}

static void init_userp(unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;
    unsigned int page_size;

    page_size = getpagesize();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s does not support "
                    "user pointer i/o\n", dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_REQBUFS");
        }
    }

    buffers = calloc(4, sizeof(*buffers));

    if (!buffers)
    {
        fprintf(stderr, "Out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers)
    {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = memalign( /* boundary */ page_size,
                                            buffer_size);

        if (!buffers[n_buffers].start)
        {
            fprintf(stderr, "Out of memory\n");
            exit(EXIT_FAILURE);
        }
    }
}

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf(stderr, "%s is no V4L2 device\n", dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
            errno_exit("VIDIOC_QUERYCAP");
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    switch (io)
    {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE))
        {
            fprintf(stderr, "%s does not support read i/o\n", dev_name);
            exit(EXIT_FAILURE);
        }

        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
            exit(EXIT_FAILURE);
        }

        break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;   /* reset to default */

        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
        errno_exit("VIDIOC_S_FMT");

    /* Note VIDIOC_S_FMT may change width and height. */

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    if (fmt.fmt.pix.width != WIDTH)
        WIDTH = fmt.fmt.pix.width;

    if (fmt.fmt.pix.height != HEIGHT)
        HEIGHT = fmt.fmt.pix.height;

    switch (io)
    {
    case IO_METHOD_READ:
        init_read(fmt.fmt.pix.sizeimage);
        break;

    case IO_METHOD_MMAP:
        init_mmap();
        break;

    case IO_METHOD_USERPTR:
        init_userp(fmt.fmt.pix.sizeimage);
        break;
    }
}

static void close_device(void)
{
    if (-1 == close(fd))
        errno_exit("close");

    fd = -1;
}

static void open_device(void)
{
    struct stat st;

    if (-1 == stat(dev_name, &st))
    {
        fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode))
    {
        fprintf(stderr, "%s is no device\n", dev_name);
        exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */  | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        fprintf(stderr, "Cannot open '%s': %d, %s\n",
                dev_name, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }
}

static void init_mem2mem_dev()
{
    int ret;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_control ctrl;

    mem2mem_fd = open(mem2mem_dev_name, O_RDWR | O_NONBLOCK, 0);
    perror_exit(mem2mem_fd < 0, "open");

    if (hflip != 0)
    {
        ctrl.id = V4L2_CID_HFLIP;
        ctrl.value = 1;
        ret = ioctl(mem2mem_fd, VIDIOC_S_CTRL, &ctrl);
        if (ret != 0)
            fprintf(stderr, "%s:%d: Set HFLIP failed\n",
                    __func__, __LINE__);
    }

    if (vflip != 0)
    {
        ctrl.id = V4L2_CID_VFLIP;
        ctrl.value = 1;
        ret = ioctl(mem2mem_fd, VIDIOC_S_CTRL, &ctrl);
        if (ret != 0)
            fprintf(stderr, "%s:%d: Set VFLIP failed\n",
                    __func__, __LINE__);
    }

    ctrl.id = V4L2_CID_TRANS_TIME_MSEC;
    ctrl.value = transtime;
    ret = ioctl(mem2mem_fd, VIDIOC_S_CTRL, &ctrl);
    perror_exit(ret != 0, "ioctl");

    ctrl.id = V4L2_CID_TRANS_NUM_BUFS;
    ctrl.value = translen;
    ret = ioctl(mem2mem_fd, VIDIOC_S_CTRL, &ctrl);
    perror_exit(ret != 0, "ioctl");

    ret = ioctl(mem2mem_fd, VIDIOC_QUERYCAP, &cap);
    perror_exit(ret != 0, "ioctl");

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "Device does not support capture\n");
        exit(EXIT_FAILURE);
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_OUTPUT))
    {
        fprintf(stderr, "Device does not support output\n");
        exit(EXIT_FAILURE);
    }

    /* Set format for capture */
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565X;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    ret = ioctl(mem2mem_fd, VIDIOC_S_FMT, &fmt);
    perror_exit(ret != 0, "ioctl");

    /* The same format for output */
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = WIDTH;
    fmt.fmt.pix.height = HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565X;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    ret = ioctl(mem2mem_fd, VIDIOC_S_FMT, &fmt);
    perror_exit(ret != 0, "ioctl");
}

static void gen_buf(uint8_t * dst, uint8_t * src, size_t size)
{
    size_t i;
    uint16_t tmp;
    uint16_t *dst_buf = (uint16_t *) dst;
    uint16_t *src_buf = (uint16_t *) src;

    size /= 2;

    for (i = 0; i < size; i++)
    {
        tmp = src_buf[i];

        /* 
         * V4L2_PIX_FMT_RGB565X stores most significant Green bits in
         * byte 0. Since SDL displays otherwise, swap those here.
         */
        dst_buf[i] = ((tmp & 0xE000) >> 13) | ((tmp & 0x0007) << 13) |
            (tmp & 0x1FF8);
    }
}


static int read_mem2mem_frame(int last)
{
    struct v4l2_buffer buf;
    int ret;

    memzero(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    buf.memory = V4L2_MEMORY_MMAP;

    ret = ioctl(mem2mem_fd, VIDIOC_DQBUF, &buf);
    debug("Dequeued source buffer, index: %d\n", buf.index);
    if (ret)
    {
        switch (errno)
        {
        case EAGAIN:
            debug("Got EAGAIN\n");
            return 0;

        case EIO:
            debug("Got EIO\n");
            return 0;

        default:
            perror("ioctl");
            return 0;
        }
    }

    /* Verify we've got a correct buffer */
    assert(buf.index < num_src_bufs);

    /* Enqueue back the buffer (note that the index is preserved) */
    if (!last)
    {
        uint8_t *p_buf = (uint8_t *) buffer_sdl;
        p_buf += curr_buf * transsize;

        gen_buf((uint8_t *) p_src_buf[buf.index], p_buf, transsize);


        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        ret = ioctl(mem2mem_fd, VIDIOC_QBUF, &buf);
        perror_ret(ret != 0, "ioctl");
    }


    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    debug("Dequeuing destination buffer\n");
    ret = ioctl(mem2mem_fd, VIDIOC_DQBUF, &buf);
    if (ret)
    {
        switch (errno)
        {
        case EAGAIN:
            debug("Got EAGAIN\n");
            return 0;

        case EIO:
            debug("Got EIO\n");
            return 0;

        default:
            perror("ioctl");
            return 1;
        }
    }
    debug("Dequeued dst buffer, index: %d\n", buf.index);
    /* Verify we've got a correct buffer */
    assert(buf.index < num_dst_bufs);

    debug("Current buffer in the transaction: %d\n", curr_buf);

    uint8_t *p_post = (uint8_t *) buffer_m2m_sdl;
    p_post += curr_buf * transsize;
    ++curr_buf;
    if (curr_buf >= translen)
    {
        curr_buf = 0;
        read_input_frame();
    }

    /* Display results */
    gen_buf(p_post, (uint8_t *) p_dst_buf[buf.index], transsize);

    render(data_sf, data_m2m_sf);

    /* Enqueue back the buffer */
    if (!last)
    {
        // gen_dst_buf(p_dst_buf[buf.index], dst_buf_size[buf.index]);
        ret = ioctl(mem2mem_fd, VIDIOC_QBUF, &buf);
        perror_ret(ret != 0, "ioctl");
        debug("Enqueued back dst buffer\n");
    }

    return 0;
}

static void start_mem2mem()
{
    int ret;
    int i;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers reqbuf;
    enum v4l2_buf_type type;
    int last = 0;
    SDL_Event event;

    init_mem2mem_dev();

    memzero(reqbuf);
    reqbuf.count = NUM_BUFS;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(mem2mem_fd, VIDIOC_REQBUFS, &reqbuf);
    perror_exit(ret != 0, "ioctl");
    num_src_bufs = reqbuf.count;
    debug("Got %d src buffers\n", num_src_bufs);

    reqbuf.count = NUM_BUFS;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(mem2mem_fd, VIDIOC_REQBUFS, &reqbuf);
    perror_exit(ret != 0, "ioctl");
    num_dst_bufs = reqbuf.count;
    debug("Got %d dst buffers\n", num_dst_bufs);

    transsize = WIDTH * HEIGHT / translen * 2;

    for (i = 0; i < num_src_bufs; ++i)
    {
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = ioctl(mem2mem_fd, VIDIOC_QUERYBUF, &buf);
        perror_exit(ret != 0, "ioctl");
        debug("QUERYBUF returned offset: %x\n", buf.m.offset);

        src_buf_size[i] = buf.length;
        p_src_buf[i] = mmap(NULL, buf.length,
                            PROT_READ | PROT_WRITE, MAP_SHARED,
                            mem2mem_fd, buf.m.offset);
        perror_exit(MAP_FAILED == p_src_buf[i], "mmap");
    }

    for (i = 0; i < num_dst_bufs; ++i)
    {
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = ioctl(mem2mem_fd, VIDIOC_QUERYBUF, &buf);
        perror_exit(ret != 0, "ioctl");
        debug("QUERYBUF returned offset: %x\n", buf.m.offset);

        dst_buf_size[i] = buf.length;
        p_dst_buf[i] = mmap(NULL, buf.length,
                            PROT_READ | PROT_WRITE, MAP_SHARED,
                            mem2mem_fd, buf.m.offset);
        perror_exit(MAP_FAILED == p_dst_buf[i], "mmap");
    }

    read_input_frame();
    for (i = 0; i < num_src_bufs; ++i)
    {
        uint8_t *p_buf = (uint8_t *) buffer_sdl;
        p_buf += (i % translen) * transsize;

        gen_buf((uint8_t *) p_src_buf[i], p_buf, transsize);


        memzero(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = ioctl(mem2mem_fd, VIDIOC_QBUF, &buf);
        perror_exit(ret != 0, "ioctl");
    }

    for (i = 0; i < num_dst_bufs; ++i)
    {
        memzero(buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        ret = ioctl(mem2mem_fd, VIDIOC_QBUF, &buf);
        perror_exit(ret != 0, "ioctl");
    }

    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    ret = ioctl(mem2mem_fd, VIDIOC_STREAMON, &type);
    debug("STREAMON (%ld): %d\n", VIDIOC_STREAMON, ret);
    perror_exit(ret != 0, "ioctl");

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(mem2mem_fd, VIDIOC_STREAMON, &type);
    debug("STREAMON (%ld): %d\n", VIDIOC_STREAMON, ret);
    perror_exit(ret != 0, "ioctl");

    while (num_frames)
    {
        fd_set read_fds;
        int r;


        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT)
                return;


        FD_ZERO(&read_fds);
        FD_SET(mem2mem_fd, &read_fds);

        debug("Before select");
        r = select(mem2mem_fd + 1, &read_fds, NULL, NULL, 0);
        perror_exit(r < 0, "select");
        debug("After select");

        if (num_frames == 1)
            last = 1;
        if (read_mem2mem_frame(last))
        {
            fprintf(stderr, "Read frame failed\n");
            break;
        }
        --num_frames;
        printf("FRAMES LEFT: %d\n", num_frames);
    }

    close(mem2mem_fd);

    for (i = 0; i < num_src_bufs; ++i)
        munmap(p_src_buf[i], src_buf_size[i]);

    for (i = 0; i < num_dst_bufs; ++i)
        munmap(p_dst_buf[i], dst_buf_size[i]);
}

static void usage(FILE * fp, int argc, char **argv)
{
    fprintf(fp,
            "Usage: %s [options]\n\n"
            "Options:\n"
            "-i | --input-device name   Video device name [/dev/video0]\n"
            "-o | --m2m-device name     mem2mem device name [/dev/video1]\n"
            "-h | --help                Print this message\n"
            "-m | --mmap                Use memory mapped buffers\n"
            "-r | --read                Use read() calls\n"
            "-u | --userp               Use application allocated buffers\n"
            "-x | --width               Video width\n"
            "-y | --height              Video height\n"
            "-t | --translen            Transaction length [1]\n"
            "-T | --time                Transaction time in ms [1]\n"
            "-n | --num-frames          Number of frames to process [1000]\n"
            "-f | --hflip               Horizontal Mirror\n"
            "-v | --flip                Vertical Mirror\n"
            "", argv[0]);
}

static const char short_options[] = "d:o:hmrux:y:t:T:n:fv";

static const struct option long_options[] = {
    {"input-device", required_argument, NULL, 'd'},
    {"m2m-device", required_argument, NULL, 'o'},
    {"help", no_argument, NULL, 'h'},
    {"mmap", no_argument, NULL, 'm'},
    {"read", no_argument, NULL, 'r'},
    {"userp", no_argument, NULL, 'u'},
    {"width", required_argument, NULL, 'x'},
    {"height", required_argument, NULL, 'y'},
    {"translen", required_argument, NULL, 't'},
    {"time", required_argument, NULL, 'T'},
    {"num-frames", required_argument, NULL, 'n'},
    {"hflip", no_argument, NULL, 'f'},
    {"vflip", no_argument, NULL, 'v'},
    {0, 0, 0, 0}
};

static int sdl_filter(const SDL_Event * event)
{
    return event->type == SDL_QUIT;
}

int main(int argc, char **argv)
{
    dev_name = "/dev/video0";
    mem2mem_dev_name = "/dev/video1";

    for (;;)
    {
        int index;
        int c;

        c = getopt_long(argc, argv, short_options, long_options, &index);

        if (-1 == c)
            break;

        switch (c)
        {
        case 0:                /* getopt_long() flag */
            break;

        case 'd':
            dev_name = optarg;
            break;

        case 'o':
            mem2mem_dev_name = optarg;
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

        case 'm':
            io = IO_METHOD_MMAP;
            break;

        case 'r':
            io = IO_METHOD_READ;
            break;

        case 'u':
            io = IO_METHOD_USERPTR;
            break;

        case 'x':
            WIDTH = atoi(optarg);
            break;

        case 'y':
            HEIGHT = atoi(optarg);
            break;

        case 't':
            translen = atoi(optarg);
            break;

        case 'T':
            transtime = atoi(optarg);
            break;

        case 'n':
            num_frames = atoi(optarg);
            break;

        case 'f':
            hflip = 1;
            break;

        case 'v':
            vflip = 1;
            break;

        default:
            usage(stderr, argc, argv);
            exit(EXIT_FAILURE);
        }
    }

    generate_YCbCr_to_RGB_lookup();

    open_device();
    init_device();

    atexit(SDL_Quit);
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        return 1;

    SDL_WM_SetCaption("SDL mem2mem tester", NULL);

    buffer_sdl = (uint16_t *) malloc(WIDTH * HEIGHT * 2);
    buffer_m2m_sdl = (uint16_t *) malloc(WIDTH * HEIGHT * 2);

    SDL_SetVideoMode(WIDTH, HEIGHT * 2 + SEPARATOR, 16, SDL_HWSURFACE);

    data_sf = SDL_CreateRGBSurfaceFrom(buffer_sdl, WIDTH, HEIGHT,
                                       16, WIDTH * 2,
                                       0x1F00, 0xE007, 0x00F8, 0);

    data_m2m_sf = SDL_CreateRGBSurfaceFrom(buffer_m2m_sdl, WIDTH, HEIGHT,
                                           16, WIDTH * 2,
                                           0x1F00, 0xE007, 0x00F8, 0);

    SDL_SetEventFilter(sdl_filter);

    start_capturing();
    start_mem2mem();
    stop_capturing();

    uninit_device();
    close_device();

    SDL_FreeSurface(data_sf);
    SDL_FreeSurface(data_m2m_sf);
    free(buffer_sdl);
    free(buffer_m2m_sdl);

    exit(EXIT_SUCCESS);

    return 0;
}
