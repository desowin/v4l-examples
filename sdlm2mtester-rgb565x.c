/**
 * Copyright (C) 2012 by Tomasz Moń <desowin@gmail.com>
 *
 * compile with:
 *   gcc -o sdlm2mtester-rgb565x sdlm2mtester-rgb565x.c -lSDL
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

static char *mem2mem_dev_name = NULL;

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

static uint8_t *data;

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

static void init_input_data(uint8_t * data)
{
    size_t i;
    for(i = WIDTH * HEIGHT * 3; i--; )
        data[i] = (i % 3 == 0) ? (i / 3) % WIDTH :
            (i % 3 == 1) ? (i / 3) / WIDTH : 0;
}

static void next_input_frame()
{
    size_t i;
    size_t x;
    size_t y;

    for(i = 0; i < WIDTH * HEIGHT * 3; i += 1 + rand() % 3)
        data[i] -= rand() % 8;

    uint32_t * data_rgb;

    for (y = 0; y < HEIGHT; y++)
    {
        for (x = 0; x < WIDTH; x++)
        {
            data_rgb = (uint32_t*)&data[(y*WIDTH+x)*3];
            buffer_sdl[y*WIDTH+x] = RGB888_to_RGB565(*data_rgb);
        }
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
        next_input_frame();
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

    next_input_frame();
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
            "-o | --m2m-device name     mem2mem device name [/dev/video1]\n"
            "-h | --help                Print this message\n"
            "-x | --width               Video width\n"
            "-y | --height              Video height\n"
            "-t | --translen            Transaction length [1]\n"
            "-T | --time                Transaction time in ms [1]\n"
            "-n | --num-frames          Number of frames to process [1000]\n"
            "", argv[0]);
}

static const char short_options[] = "o:hx:y:t:T:n:";

static const struct option long_options[] = {
    {"m2m-device", required_argument, NULL, 'o'},
    {"help", no_argument, NULL, 'h'},
    {"width", required_argument, NULL, 'x'},
    {"height", required_argument, NULL, 'y'},
    {"translen", required_argument, NULL, 't'},
    {"time", required_argument, NULL, 'T'},
    {"num-frames", required_argument, NULL, 'n'},
    {0, 0, 0, 0}
};

static int sdl_filter(const SDL_Event * event)
{
    return event->type == SDL_QUIT;
}

int main(int argc, char **argv)
{
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

        case 'o':
            mem2mem_dev_name = optarg;
            break;

        case 'h':
            usage(stdout, argc, argv);
            exit(EXIT_SUCCESS);

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

        default:
            usage(stderr, argc, argv);
            exit(EXIT_FAILURE);
        }
    }

    atexit(SDL_Quit);
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        return 1;

    SDL_WM_SetCaption("SDL mem2mem tester", NULL);

    data = (uint8_t*)malloc(WIDTH*HEIGHT*3);
    init_input_data(data);

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

    start_mem2mem();

    free(data);

    SDL_FreeSurface(data_sf);
    SDL_FreeSurface(data_m2m_sf);
    free(buffer_sdl);
    free(buffer_m2m_sdl);

    exit(EXIT_SUCCESS);

    return 0;
}
