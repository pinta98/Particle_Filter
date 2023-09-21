#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdio.h>
#include <cassert>
#include<iostream>

#include "uio_map.hpp"

int uio_initialize(UioMap *device, unsigned int uio) {
    char devuio[ MAX_UIO_PATH_SIZE ];
    char uiosize[ MAX_UIO_PATH_SIZE ];
    int fd, ret;
    FILE* Fp;

    sprintf(devuio, "/dev/uio%u",uio);
    if((fd = open(devuio, O_RDWR)) < 0)
        return -1;

    sprintf(uiosize, "/sys/class/uio/uio%u/maps/map0/size",uio);
    Fp = fopen(uiosize, "r");
    if (!Fp)
        return -2;

    ret = fscanf(Fp, "0x%lx", &device->size);
    fclose(Fp);
    if (ret < 0)
        return -3;

    device->phy = UIO_INVALID_ADDR;
    sprintf(uiosize, "/sys/class/uio/uio%u/maps/map0/addr",uio);
    Fp = fopen(uiosize, "r");
    if (!Fp)
        return -4;

    ret = fscanf(Fp, "0x%lx", &device->phy);
    if (ret < 0)
        return -5;

    fclose(Fp);

    device->addr = (uint64_t)mmap(NULL, device->size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);

    if(device->addr == (uint64_t) MAP_FAILED)
        return -5;

    assert(device->addr);

    return 0;
}

int uio_release(UioMap *device) {

  if(device == nullptr) return 1;
  munmap((void*)(device->addr), device->size);
  return 0;
}
