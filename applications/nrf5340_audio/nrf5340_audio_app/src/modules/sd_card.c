/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "sd_card.h"

#include <zephyr.h>
#include <device.h>
#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <string.h>
#include <drivers/gpio.h>
#include <devicetree.h>

#include "macros_common.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(sd_card, CONFIG_LOG_SD_CARD_LEVEL);

#define SD_ROOT_PATH "/SD:/"
#define PATH_MAX_LEN 260 /* Maximum length for path support by Windows file system*/

static const char *sd_root_path = "/SD:";
static FATFS fat_fs;
static bool sd_init_success;

static struct fs_mount_t mnt_pt = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

int sd_card_list_files(char *path)
{
	int ret;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;
	char abs_path_name[PATH_MAX_LEN + 1] = SD_ROOT_PATH;

	if (!sd_init_success) {
		return -ENODEV;
	}

	if (path == NULL) {
		ret = fs_opendir(&dirp, sd_root_path);
		RET_IF_ERR_MSG(ret, "Open SD card root dir failed");
	} else {
		if (strlen(path) > CONFIG_FS_FATFS_MAX_LFN) {
			RET_IF_ERR_MSG(-FR_INVALID_NAME, "Path is too long");
		} else {
			strcat(abs_path_name, path);
			ret = fs_opendir(&dirp, abs_path_name);
			RET_IF_ERR_MSG(ret, "Open assigned path failed");
		}
	}
	while (true) {
		ret = fs_readdir(&dirp, &entry);
		RET_IF_ERR(ret);

		if (entry.name[0] == 0) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			LOG_INF("[DIR ] %s", entry.name);
		} else {
			LOG_INF("[FILE] %s", entry.name);
		}
	}

	ret = fs_closedir(&dirp);
	RET_IF_ERR_MSG(ret, "Close SD card root dir failed");

	return 0;
}

int sd_card_write(char const *const filename, char const *const data, size_t *size)
{
	struct fs_file_t f_entry;
	char abs_path_name[PATH_MAX_LEN + 1] = SD_ROOT_PATH;
	int ret;

	if (!sd_init_success) {
		return -ENODEV;
	}

	if (strlen(filename) > CONFIG_FS_FATFS_MAX_LFN) {
		RET_IF_ERR_MSG(-FR_INVALID_NAME, "Filename is too long");
	} else {
		strcat(abs_path_name, filename);
	}

	ret = fs_open(&f_entry, abs_path_name, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
	RET_IF_ERR_MSG(ret, "Create file failed");

	/* If the file exists, moves the file position pointer to the end of the file */
	ret = fs_seek(&f_entry, 0, FS_SEEK_END);
	RET_IF_ERR_MSG(ret, "Seek file pointer failed");

	ret = fs_write(&f_entry, data, *size);
	if (ret < 0) {
		RET_IF_ERR_MSG(ret, "Write file failed");
	} else {
		*size = ret;
	}

	ret = fs_close(&f_entry);
	RET_IF_ERR_MSG(ret, "Close file failed");

	return 0;
}

int sd_card_read(char const *const filename, char *const data, size_t *size)
{
	struct fs_file_t f_entry;
	char abs_path_name[PATH_MAX_LEN + 1] = SD_ROOT_PATH;
	int ret;

	if (!sd_init_success) {
		return -ENODEV;
	}

	if (strlen(filename) > CONFIG_FS_FATFS_MAX_LFN) {
		RET_IF_ERR_MSG(-FR_INVALID_NAME, "Filename is too long");
	} else {
		strcat(abs_path_name, filename);
	}

	ret = fs_open(&f_entry, abs_path_name, FS_O_READ);
	RET_IF_ERR_MSG(ret, "Open file failed");

	ret = fs_read(&f_entry, data, *size);
	if (ret < 0) {
		RET_IF_ERR_MSG(ret, "Read file failed");
	} else {
		*size = ret;
		if (*size == 0) {
			LOG_WRN("File is empty");
		}
	}

	ret = fs_close(&f_entry);
	RET_IF_ERR_MSG(ret, "Close file failed");

	return 0;
}

int sd_card_init(void)
{
	int ret;
	static const char *sd_dev = "SD";
	uint64_t sd_card_size_bytes;
	uint32_t sector_count;
	size_t sector_size;

	ret = disk_access_init(sd_dev);
	if (ret) {
		LOG_DBG("SD card init failed, please check if SD card inserted");
		return -ENODEV;
	}

	ret = disk_access_ioctl(sd_dev, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
	RET_IF_ERR_MSG(ret, "Unable to get sector count");
	LOG_DBG("Sector count: %d", sector_count);

	ret = disk_access_ioctl(sd_dev, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
	RET_IF_ERR_MSG(ret, "Unable to get sector size");
	LOG_DBG("Sector size: %d bytes", sector_size);

	sd_card_size_bytes = (uint64_t)sector_count * sector_size;
	LOG_INF("SD card volume size: %d MB", (uint32_t)(sd_card_size_bytes >> 20));

	mnt_pt.mnt_point = sd_root_path;
	ret = fs_mount(&mnt_pt);
	RET_IF_ERR_MSG(ret, "Mnt. disk failed, could be format issue. should be FAT/exFAT");

	sd_init_success = true;

	return 0;
}
