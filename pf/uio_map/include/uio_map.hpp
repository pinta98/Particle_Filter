#pragma once

#include <stdint.h>

#define MAX_UIO_PATH_SIZE       256
#define MAX_UIO_NAME_SIZE       64
#define MAX_UIO_MAPS            5
#define UIO_INVALID_ADDR        0

typedef struct {
  uint64_t addr;
  uint64_t phy;
  uint64_t size;
} UioMap;

enum uio_devices : unsigned int;

int uio_initialize(UioMap *device, unsigned int uio);
int uio_release(UioMap *device);
