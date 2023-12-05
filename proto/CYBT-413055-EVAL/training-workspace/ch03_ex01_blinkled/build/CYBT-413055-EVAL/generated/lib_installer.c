// ../mtb_shared/wiced_btsdk/dev-kit/baselib/20719B2/release-v4.4.2/COMPONENT_20719B2/make/scripts/wiced-gen-lib-installer.pl
// args: 
// output: C:/Users/alexl/projects/Headphones-V2/proto/CYBT-413055-EVAL/training-workspace/ch03_ex01_blinkled/build/CYBT-413055-EVAL/generated/lib_installer.c
#include <stdint.h>
typedef struct tag_PATCH_TABLE_ENTRY_t {
	uint32_t breakout;
	uint32_t replacement;
} PATCH_TABLE_ENTRY_t;
void patch_autoInstall(uint32_t old_address, uint32_t new_address);
void patch_autoReplace(uint32_t breakout_address, uint32_t replacement);
void patch_autoReplaceData(uint32_t breakout_address, uint32_t replacement);
void install_libs(void);

void install_libs(void)
{
}
#define PATCH_ENTRIES_WITH_LIBRARIES (0 + CY_PATCH_ENTRIES_BASE)
#if PATCH_ENTRIES_WITH_LIBRARIES > NUM_PATCH_ENTRIES
#error Too many patch entries for device, after adding libraries
#endif
