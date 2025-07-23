/*
 * $QNXLicenseC:

 * Copyright 2016, QNX Software Systems. All Rights Reserved.
 *
 * You must obtain a written license from and pay applicable license fees to QNX
 * Software Systems before you may reproduce, modify or distribute this software,
 * or any work that includes all or part of this software.   Free development
 * licenses are available for evaluation and non-commercial purposes.  For more
 * information visit http://licensing.qnx.com or email licensing@qnx.com.
 *
 * This file may contain contributions from others.  Please review this entire
 * file for other proprietary rights or license notices, as well as the QNX
 * Development Suite License Guide at http://licensing.qnx.com/license-guide/
 * for other information.
 * $
 */

#ifndef BOOT_DATA_API_H
#define BOOT_DATA_API_H

#include <stdbool.h>
#include <stdint.h>

#define BOOT_DATA_VERSION       2
#define BOOT_DATA_MIN_VERSION   1

/*
 * Boot data error flags
 */
#define BOOT_ERROR_IFS_LOADING_SHIFT  (0)
#define BOOT_ERROR_IFS_LOADING_BIT    (1 << BOOT_ERROR_IFS_LOADING_SHIFT)
#define BOOT_ERROR_IFS_SCAN_SHIFT     (1)
#define BOOT_ERROR_IFS_SCAN_BIT       (1 << BOOT_ERROR_IFS_SCAN_SHIFT)
#define BOOT_ERROR_IFS_UPDATE_SHIFT   (2)
#define BOOT_ERROR_IFS_UPDATE_BIT     (1 << BOOT_ERROR_IFS_UPDATE_SHIFT)
#define BOOT_ERROR_OS_MOUNT_SHIFT     (3)
#define BOOT_ERROR_OS_MOUNT_BIT       (1 << BOOT_ERROR_OS_MOUNT_SHIFT)

/* Tracks the state of the boot progress */
typedef enum {
    PROGRESS_IPL_START,
    PROGRESS_IFS_START,
    PROGRESS_OS_START,
    PROGRESS_OS_BOOTED,
    PROGRESS_OS_RESTARTING,
    PROGRESS_OS_RESTART,
    PROGRESS_OS_POWERINGOFF,
    PROGRESS_OS_POWEROFF,
    PROGRESS_MAX
} progress_t;

typedef enum {
    MODE_NONE,      /* No mode set */
    MODE_MIRROR,    /* The component is a mirror copy of the other configuration */
    MODE_MAX
} component_mode_t;

typedef struct {
    char version[32];               /* String version of the component */
    uint32_t boot_data_version;     /* Component's current boot data structure version */
    uint32_t boot_data_min_version; /* Component's minimum boot data structure version */
    uint32_t index_version;         /* Component's current configuration index version */
    uint32_t index_min_version;     /* Component's minimum configuration index version */
    uint32_t crc32;                 /* Checksum of component */
    uint32_t crc32_data_size;       /* Size of the data the checksum applies to */
    component_mode_t mode;          /* Mode of the component */
    uint32_t reserved[8];
} component_t;

typedef struct {
    uint32_t ifs_index;             /* Current ifs index. Used to index the proper ifs_component */
    uint32_t os_index;              /* Current os index. Used to index the proper os_component */
    uint32_t ifs_index_version;     /* Current ifs index version */
    uint32_t ifs_index_min_version; /* Minimum ifs index version */
    uint32_t os_index_version;      /* Current os index version */
    uint32_t os_index_min_version;  /* Minimum os index version */
    uint32_t hard_boot_count;       /* Hard reset count for this configuration */
    uint32_t soft_boot_count;       /* Software reset count for this configuration */
    uint32_t failure_boot_count;    /* Tracks the amount of consecutive boots that have been unsuccessful */
    uint32_t reserved[8];
} configuration_t;

typedef struct {
    uint32_t magic;                     /* Magic number identifying structure */
    uint32_t crc32;                     /* CRC for the structure starting from size and covers data_size */
    uint32_t size;                      /* Size of the structure */
    uint32_t data_size;                 /* Total size of structure plus any custom data found after that needs to be tracked */
    uint32_t reserved[8];               /* Reserved for future IPL use */
    component_t ipl_component;          /* ipl component information */
    component_t ifs_component[2];       /* ifs component information */
    component_t os_component[2];        /* os component information */
    uint32_t version;                   /* Structure version */
    uint32_t min_version;               /* Minimum compatible structure version */
    uint32_t updating;                  /* Specified in an update cycle */
    uint32_t boot_failsafe;             /* Specified when the boot failsafe option should be taken */
    progress_t progress;                /* Boot progress to track the location of any failures */
    uint32_t error_flags;               /* Flags to track errors */
    uint32_t hard_boot_count;           /* Tracks total hard reset counts */
    uint32_t soft_boot_count;           /* Tracks total software reset count */
    uint32_t sequence;                  /* Detect which boot data file is current */
    configuration_t configuration[2];   /* Current and previous configuration */
} boot_data_t;

/**
 * \brief Read boot data structure from device
 * \param[in]  file      boot file to read from
 * \param[out] boot_data destination for boot_data structure
 * \retval int See @boot_data_error_t
 */
int boot_data_read(boot_data_t *boot_data);

/**
 * \brief Write boot data to the device and updates CRC data in structure
 * \param[in] boot_data boot data to write
 * \retval On success, it returns the number of bytes written. If an error occurs -1 is returned.
 */
int boot_data_write(boot_data_t *boot_data);

typedef enum {
    BOOT_DATA_NONE_ITEM,        /* No item selected */
    BOOT_DATA_CURRENT_ITEM,     /* Current configured item */
    BOOT_DATA_PREVIOUS_ITEM,    /* Last configured item */
    BOOT_DATA_NEW_ITEM          /* Configuration that is ongoing but isn't committed yet */
} boot_data_item_t;

bool boot_data_is_updating();
void boot_data_set_updating(bool updating);

void boot_data_update_ifs();
void boot_data_update_os();
void boot_data_revert();

const char * boot_data_get_ifs_version(boot_data_item_t item);
const char * boot_data_get_os_version(boot_data_item_t item);
void boot_data_set_ifs_version(boot_data_item_t item, const char *version);
void boot_data_set_os_version(boot_data_item_t item, const char *version);

uint32_t boot_data_get_error_flags();
void boot_data_set_error_flags(uint32_t error_flags);

progress_t boot_data_get_progress();
const char * boot_data_get_ipl_version();
const char * boot_data_get_ifs_location(boot_data_item_t item);
const char * boot_data_get_os_location(boot_data_item_t item);

int boot_data_commit();


#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL$ $Rev$")
#endif

