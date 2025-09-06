
#include "fs.hpp"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"
#include <errno.h>

esp_vfs_fat_mount_config_t mount_config;
wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
bool mount_state = false;

bool get_mount_state() { return mount_state; }
void dump_heap_info(const char *tag) {
  // 全体のフリー
  size_t free_heap = esp_get_free_heap_size();
  size_t min_free_heap = esp_get_minimum_free_heap_size();

  // 8bitアクセス可能領域
  size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t min_free_8bit = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

  // 32bitアクセス可能領域
  size_t free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
  size_t min_free_32bit = heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT);

  // PSRAM (外付けRAMがあれば)
  size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t min_free_psram = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

  printf("---- Heap Info [%s] ----\n", tag);
  printf("Total free heap: %u (min: %u)\n", (unsigned)free_heap,
         (unsigned)min_free_heap);
  printf("  8-bit capable : %u (min: %u)\n", (unsigned)free_8bit,
         (unsigned)min_free_8bit);
  printf(" 32-bit capable : %u (min: %u)\n", (unsigned)free_32bit,
         (unsigned)min_free_32bit);
  printf("   SPIRAM (ext) : %u (min: %u)\n", (unsigned)free_psram,
         (unsigned)min_free_psram);

  // より詳細に見たい場合はこれで全ヒープ領域の状態を出力
  heap_caps_print_heap_info(MALLOC_CAP_8BIT);
  heap_caps_print_heap_info(MALLOC_CAP_32BIT);
  heap_caps_print_heap_info(MALLOC_CAP_SPIRAM);
  heap_caps_print_heap_info(MALLOC_CAP_8BIT);
}
void mount() {
  if (mount_state) {
    return;
  }
  mount_config.max_files = 8;
  mount_config.format_if_mount_failed = true;
  mount_config.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;
  const char *base_path = "/spiflash";

  // printf("storage0: try mount\n");
  esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage0",
                                                   &mount_config, &s_wl_handle);
  if (err != ESP_OK) {
    printf("storage0: Failed to mount FATFS (%s)\n", esp_err_to_name(err));
    dump_heap_info("mount fail");
    return;
  } else {
    // printf("storage0: mount OK\n");
    mount_state = true;
  }
}

void umount() {
  if (!mount_state) {
    return;
  }
  const char *base_path = "/spiflash";
  esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle);
  mount_state = false;
}