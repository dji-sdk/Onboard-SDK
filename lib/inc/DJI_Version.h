#ifndef DJI_VERSION_H
#define DJI_VERSION_H

#define MAKE_VERSION(a, b, c, d)                                               \
    (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) |                     \
     ((c << 8) & 0x0000ff00) | (d & 0x000000ff))

#ifdef SDK_VERSION_2_3
#define SDK_VERSION (MAKE_VERSION(2, 3, 10, 0))
#endif

#ifdef SDK_VERSION_3_0
#define SDK_VERSION (MAKE_VERSION(3, 0, 10, 0))
#endif

#ifdef SDK_VERSION_3_1
#define SDK_VERSION (MAKE_VERSION(3, 1, 10, 0))
#endif

#endif // DJI_VERSION_H

