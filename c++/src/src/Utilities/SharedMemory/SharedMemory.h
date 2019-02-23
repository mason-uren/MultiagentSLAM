//
// Created by Mason U'Ren on 2019-02-14.
//

#ifndef C_SHAREDMEMORY_H
#define C_SHAREDMEMORY_H

#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <zconf.h>
#include <sys/mman.h>
#include <SharedMemoryStructs.h>
#include <cstring>

class SharedMemory {
public:
    SharedMemory() {
        !shm_unlink("/slam_config_in") ?
            printf("%s\n", "Status: unlinked shared-memory.") :
            printf("%s\n", "Status: no shared-memory link.");
        slam_md = shm_open("/slam_config_in", O_CREAT | O_RDWR | O_EXCL, 0600);
        pg_size = sysconf(_SC_PAGE_SIZE);


        /* Set size */
        if (ftruncate(slam_md, pg_size) == -1) {
            perror("ftruncate failure");
            exit(1);
        }

        /* Map memory */
        virt_addr = mmap(nullptr, (size_t) pg_size, PROT_WRITE | PROT_READ, MAP_SHARED, slam_md, 0);
        if (mlock(virt_addr, (size_t) pg_size) != 0) {
            perror("mlock failure.");
            exit(1);
        }
    }
    ~SharedMemory() {
        munmap(virt_addr, (size_t) pg_size);
        close(slam_md);
        shm_unlink("/slam_config_in");
    }

    bool writeMemoryIn(SYS_CONFIG_IN *configIn) {
        memcpy(virt_addr, configIn, sizeof(SYS_CONFIG_IN));
        return true;
    }

    bool readMemoryIn(SYS_CONFIG_IN *configIn) {
        SYS_CONFIG_IN temp;
        memcpy(&temp, virt_addr, sizeof(SYS_CONFIG_IN));
        if (temp.block_id != configIn->block_id) {
            memcpy(configIn, &temp, sizeof(SYS_CONFIG_IN));
            return true;
        }
        return false;
    }

private:
    int slam_md;
    long pg_size;
    void *virt_addr;

};


#endif //C_SHAREDMEMORY_H
