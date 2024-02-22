/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/types.h>
#include <zephyr/drivers/disk.h>
#include <errno.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ramdisk, CONFIG_RAMDISK_LOG_LEVEL);

#define RAMDISK_SECTOR_SIZE 512
#define RAMDISK_VOLUME_SIZE (CONFIG_DISK_RAM_VOLUME_SIZE * 1024)
#define RAMDISK_SECTOR_COUNT (RAMDISK_VOLUME_SIZE / RAMDISK_SECTOR_SIZE)

#if IS_ENABLED(CONFIG_DISK_RAM_DECOMPRESS)
#include <lz4frame.h>
#define LZ4_MAGIC 0x184D2204
static uint8_t unpacked_buf[RAMDISK_VOLUME_SIZE];
#endif

#if IS_ENABLED(CONFIG_DISK_RAM_EXTERNAL)
static uint8_t *ramdisk_buf;
#else
static uint8_t ramdisk_buf[RAMDISK_VOLUME_SIZE];
#endif

#if IS_ENABLED(CONFIG_DISK_RAM_EXTERNAL)
static uint8_t *disk_ram_external_map(void)
{
#if defined(CONFIG_MMU)
	uint8_t *virt_start;

	z_phys_map(&virt_start, CONFIG_DISK_RAM_START,
				RAMDISK_VOLUME_SIZE, K_MEM_CACHE_WB | K_MEM_PERM_RW);
	return virt_start;
#else
	return (uint8_t *)CONFIG_DISK_RAM_START;
#endif
}
#endif

#if IS_ENABLED(CONFIG_DISK_RAM_DECOMPRESS)
static void disk_ram_external_unmap(char *virt_start)
{
#if defined(CONFIG_MMU)
	z_phys_unmap(virt_start, RAMDISK_VOLUME_SIZE);
#endif
}

static int disk_ram_is_compressed(char *virt_start)
{
	uint32_t magic = *(uint32_t *)virt_start;

	return magic == LZ4_MAGIC;
}

static int disk_ram_decompress(char *compressed_buf, size_t compressed_size,
				char *decompressed_buf, size_t decompressed_size)
{
	size_t src_size = compressed_size;
	size_t dst_size = decompressed_size;
	LZ4F_decompressionContext_t lz4_ctx;
	int ret;

	ret = LZ4F_createDecompressionContext(&lz4_ctx, LZ4F_VERSION);
	if (ret < 0) {
		LOG_ERR("Can't create decompression context, error: %s\n",
			LZ4F_getErrorName(ret));
		return ret;
	}

	ret = LZ4F_decompress(lz4_ctx, decompressed_buf, &dst_size,
			compressed_buf, &src_size, NULL);
	if (ret < 0) {
		LOG_ERR("Decompression failed: %s\n", LZ4F_getErrorName(ret));
		return ret;
	}
	LZ4F_freeDecompressionContext(lz4_ctx);

	return ret;
}
#endif

static void *lba_to_address(uint32_t lba)
{
	return &ramdisk_buf[lba * RAMDISK_SECTOR_SIZE];
}

static int disk_ram_access_status(struct disk_info *disk)
{
	return DISK_STATUS_OK;
}

static int disk_ram_access_init(struct disk_info *disk)
{
	return 0;
}

static int disk_ram_access_read(struct disk_info *disk, uint8_t *buff,
				uint32_t sector, uint32_t count)
{
	uint32_t last_sector = sector + count;

	if (last_sector < sector || last_sector > RAMDISK_SECTOR_COUNT) {
		LOG_ERR("Sector %" PRIu32 " is outside the range %u",
			last_sector, RAMDISK_SECTOR_COUNT);
		return -EIO;
	}

	memcpy(buff, lba_to_address(sector), count * RAMDISK_SECTOR_SIZE);

	return 0;
}

static int disk_ram_access_write(struct disk_info *disk, const uint8_t *buff,
				 uint32_t sector, uint32_t count)
{
	uint32_t last_sector = sector + count;

	if (last_sector < sector || last_sector > RAMDISK_SECTOR_COUNT) {
		LOG_ERR("Sector %" PRIu32 " is outside the range %u",
			last_sector, RAMDISK_SECTOR_COUNT);
		return -EIO;
	}

	memcpy(lba_to_address(sector), buff, count * RAMDISK_SECTOR_SIZE);

	return 0;
}

static int disk_ram_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	switch (cmd) {
	case DISK_IOCTL_CTRL_SYNC:
		break;
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buff = RAMDISK_SECTOR_COUNT;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buff = RAMDISK_SECTOR_SIZE;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buff  = 1U;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct disk_operations ram_disk_ops = {
	.init = disk_ram_access_init,
	.status = disk_ram_access_status,
	.read = disk_ram_access_read,
	.write = disk_ram_access_write,
	.ioctl = disk_ram_access_ioctl,
};

static struct disk_info ram_disk = {
	.name = CONFIG_DISK_RAM_VOLUME_NAME,
	.ops = &ram_disk_ops,
};

static int disk_ram_init(const struct device *dev)
{
	ARG_UNUSED(dev);
#if IS_ENABLED(CONFIG_DISK_RAM_EXTERNAL)
	ramdisk_buf = disk_ram_external_map();
#if IS_ENABLED(CONFIG_DISK_RAM_DECOMPRESS)
	if (disk_ram_is_compressed((char *)ramdisk_buf)) {
		if (disk_ram_decompress((char *)ramdisk_buf, RAMDISK_VOLUME_SIZE,
					(char *)unpacked_buf, RAMDISK_VOLUME_SIZE) < 0) {
			return -EINVAL;
		}

		LOG_INF("Freeing compressed initrd");
		disk_ram_external_unmap(ramdisk_buf);
		ramdisk_buf = unpacked_buf;
	}
#endif
#endif
	return disk_access_register(&ram_disk);
}

SYS_INIT(disk_ram_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
