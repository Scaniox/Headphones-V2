#
# Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
# an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
#
# This software, including source code, documentation and related
# materials ("Software") is owned by Cypress Semiconductor Corporation
# or one of its affiliates ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products.  Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#
ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif
#
# Basic Configuration
#
APPNAME=HSPK
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920721M2EVB-03

SUPPORTED_TARGETS = \
  CYW9M2BASE-43012BT \
  CYW920721M2EVK-01 \
  CYW920721M2EVK-02 \
  CYW943012BTEVK-01 \
  CYW955572BTEVK-01 \
  CYW920721M2EVB-03

#
# Advanced Configuration
#
SOURCES=
INCLUDES=
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# App features/defaults
#
OTA_FW_UPGRADE?=1
BT_DEVICE_ADDRESS?=default
UART?=AUTO
XIP?=xip
TRANSPORT?=UART
FASTPAIR_ENABLE :=1
AMA_SUPPORT := 0
AAC_SUPPORT ?= 1
# SPEAKER option is not applicable for CYW9M2BASE-43012BT kit
SPEAKER ?= 0
AUTO_ELNA_SWITCH ?= 0
AUTO_EPA_SWITCH ?= 0
ENABLE_DEBUG?=0
AUDIO_SHIELD_20721M2EVB_03_INCLUDED?=0

-include internal.mk

ifeq ($(SPEAKER),1)
CY_APP_DEFINES+=-DSPEAKER
endif

# wait for SWD attach
ifeq ($(ENABLE_DEBUG),1)
CY_APP_DEFINES+=-DENABLE_DEBUG=1
endif

# standard baselib prebuilt libs
ifeq ($(AMA_SUPPORT), 1)
CY_APP_PATCH_LIBS += i2s_aud_record_lib.a
endif

# Chip-specific patch libs
CY_43012C0_APP_PATCH_LIBS += wiced_audio_sink_lib.a
CY_43012C0_APP_PATCH_LIBS += wiced_sco_lib.a
CY_20721B2_APP_PATCH_LIBS += wiced_audio_sink_lib.a
CY_20721B2_APP_PATCH_LIBS += sco_aud_hook_lib.a

CY_APP_DEFINES += -DAPPNAME=\"$(APPNAME)\"
ifeq ($(AMA_SUPPORT), 1)
CY_APP_DEFINES += -DAMA_ENABLED
CY_APP_DEFINES += -DAMA_SPEECH_AUDIO_FORMAT_OPUS_BITRATE=0
CY_APP_DEFINES += -DAMA_VOICE_BUFFER_LENGTH_IN_MS=500
CY_APP_DEFINES += -DAMA_AUDIO_SNK_INCLUDED
CY_APP_DEFINES += -DAMA_HANDSFREE_INCLUDED
endif
CY_APP_DEFINES += -DAPP_TRANSPORT_DETECT_ON
CY_APP_DEFINES += -DAVRC_ADV_CTRL_INCLUDED
CY_APP_DEFINES += -DAVRC_METADATA_INCLUDED
CY_APP_DEFINES += -DDSP_BOOT_RAMDOWNLOAD
CY_APP_DEFINES += -DFASTPAIR_ACCOUNT_KEY_NUM=5
CY_APP_DEFINES += -DNREC_ENABLE
CY_APP_DEFINES += -DWICED_APP_LE_INCLUDED=TRUE
CY_APP_DEFINES += -DBT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS=1
CY_APP_DEFINES += -DWICED_BT_TRACE_ENABLE

# Options for Power Consumption Measurement
ifeq ($(CYPRESS_LOWPOWER_MODE), 1)
FASTPAIR_ENABLE = 0
CY_APP_DEFINES += -DLOW_POWER_MEASURE_MODE
endif

ifeq ($(FASTPAIR_LOWPOWER_MODE), 1)
CY_APP_DEFINES += -DLOW_POWER_MEASURE_MODE
endif

#
# Components (middleware libraries)
#
COMPONENTS += bsp_design_modus
COMPONENTS += a2dp_sink_profile
COMPONENTS += audio_insert_lib
COMPONENTS += audiomanager
COMPONENTS += avrc_controller
COMPONENTS += bt_hs_spk_lib
COMPONENTS += button_manager
COMPONENTS += handsfree_profile
ifeq ($(AMA_SUPPORT), 1)
COMPONENTS += ama
COMPONENTS += audio_record_lib
COMPONENTS += nanopbuf
endif

ifeq ($(TARGET),CYW920721M2EVK-01)
CY_APP_DEFINES += -DBT_HS_SPK_CONTROL_LINK_KEY_COUNT=8
CY_APP_DEFINES += -DAUDIO_INSERT_ENABLED
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DCT_HANDLE_PASSTHROUGH_COMMANDS
CY_APP_DEFINES += -DPLATFORM_LED_DISABLED
CY_APP_DEFINES += -DAPP_BUTTON_MAX=3
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib
CY_APP_PATCH_LIBS += wiced_mem_lib.a
AUTO_ELNA_SWITCH = 1
endif

ifneq ($(filter $(TARGET),CYW920721M2EVK-02 CYW920721M2EVB-03),)
CY_APP_DEFINES += -DBT_HS_SPK_CONTROL_LINK_KEY_COUNT=8
CY_APP_DEFINES += -DAUDIO_INSERT_ENABLED
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DCT_HANDLE_PASSTHROUGH_COMMANDS
CY_APP_DEFINES += -DPLATFORM_LED_DISABLED
CY_APP_DEFINES += -DAPP_BUTTON_MAX=3
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib
CY_APP_PATCH_LIBS += wiced_mem_lib.a
AUTO_ELNA_SWITCH = 1
endif # TARGET

ifeq ($(TARGET),CYW920721M2EVB-03)
AUDIO_SHIELD_20721M2EVB_03_INCLUDED=1
endif

ifeq ($(AUDIO_SHIELD_20721M2EVB_03_INCLUDED),1)
DISABLE_COMPONENTS += bsp_design_modus
COMPONENTS += bsp_design_modus_shield
endif

ifeq ($(TARGET),CYW9M2BASE-43012BT)
CY_APP_DEFINES += -DBT_HS_SPK_CONTROL_LINK_KEY_COUNT=2*BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS
CY_APP_DEFINES += -DAK_4679_CODEC_ENABLE
CY_APP_DEFINES += -DPLATFORM_LED_DISABLED
CY_APP_DEFINES += -DAPP_BUTTON_MAX=1
COMPONENTS += cyw9bt_audio
COMPONENTS += codec_ak4679_lib
OTA_FW_UPGRADE = 0
AUTO_ELNA_SWITCH = 0
AAC_SUPPORT := 0
endif # TARGET

ifeq ($(TARGET),CYW943012BTEVK-01)
CY_APP_DEFINES += -DBT_HS_SPK_CONTROL_LINK_KEY_COUNT=2*BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DPLATFORM_LED_DISABLED
CY_APP_DEFINES += -DAPP_BUTTON_MAX=3
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib
OTA_FW_UPGRADE = 0
AUTO_ELNA_SWITCH = 0
AAC_SUPPORT := 0
endif # TARGET

ifeq ($(TARGET),CYW955572BTEVK-01)
# Application Configuration
OTA_FW_UPGRADE := 0
CY_APP_DEFINES += -DAPP_CFG_ENABLE_BR_AUDIO=1
CY_APP_DEFINES += -DWICED_BT_HFP_HF_WBS_INCLUDED=TRUE
CY_APP_DEFINES += -DBT_HS_SPK_CONTROL_LINK_KEY_COUNT=2*BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS
CY_APP_DEFINES += -DCS47L35_CODEC_ENABLE
CY_APP_DEFINES += -DCT_HANDLE_PASSTHROUGH_COMMANDS
CY_APP_DEFINES += -DPLATFORM_LED_DISABLED
CY_APP_DEFINES += -DAPP_BUTTON_MAX=3
# TODO: audio insert
#CY_APP_DEFINES += -DAUDIO_INSERT_ENABLED
COMPONENTS += cyw9bt_audio2
COMPONENTS += codec_cs47l35_lib

# Apply new Audio Profiles
DISABLE_COMPONENTS += a2dp_sink_profile
DISABLE_COMPONENTS += avrc_controller
DISABLE_COMPONENTS += handsfree_profile
COMPONENTS += a2dp_sink_profile_btstack
COMPONENTS += audio_sink_route_config_lib
COMPONENTS += avrc_controller_btstack
COMPONENTS += handsfree_profile_btstack
endif # TARGET

ifeq ($(FASTPAIR_ENABLE),1)
CY_APP_DEFINES += -DFASTPAIR_ENABLE
COMPONENTS += gfps_provider
endif

ifeq ($(OTA_FW_UPGRADE),1)
CY_APP_DEFINES += -DOTA_FW_UPGRADE=1
COMPONENTS += fw_upgrade_lib
COMPONENTS += spp_lib
OTA_SEC_FW_UPGRADE ?= 0
ifeq ($(OTA_SEC_FW_UPGRADE), 1)
CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
endif
endif

ifeq ($(AUTO_ELNA_SWITCH),1)
CY_APP_DEFINES += -DAUTO_ELNA_SWITCH
endif

ifeq ($(AUTO_EPA_SWITCH),1)
CY_APP_DEFINES += -DAUTO_EPA_SWITCH
endif

ifeq ($(AAC_SUPPORT), 1)
ifneq ($(TARGET),CYW955572BTEVK-01)
CY_APP_PATCH_LIBS += ia_aaclc_lib.a
endif
CY_APP_DEFINES += -DWICED_BT_A2DP_SINK_MAX_NUM_CODECS=2
CY_APP_DEFINES += -DA2DP_SINK_AAC_ENABLED
CY_APP_DEFINES += -DWICED_A2DP_EXT_CODEC=1
else
CY_APP_DEFINES += -DWICED_BT_A2DP_SINK_MAX_NUM_CODECS=1
endif

CY_APP_DEFINES += -DWICED_BT_HFP_HF_MAX_CONN=BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS
CY_APP_DEFINES += -DWICED_BT_A2DP_SINK_MAX_NUM_CONN=BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS
CY_APP_DEFINES += -DMAX_CONNECTED_RCC_DEVICES=BT_HS_SPK_CONTROL_BR_EDR_MAX_CONNECTIONS

# Add led manager component
ifeq ($(filter $(CY_APP_DEFINES),-DPLATFORM_LED_DISABLED),)
COMPONENTS += led_manager
endif

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI><COMMIT><LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=BTSpy ClientControl

ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
$(error TARGET $(TARGET) not supported for this application. Edit SUPPORTED_TARGETS in the code example makefile to add new BSPs)
endif
include $(CY_TOOLS_DIR)/make/start.mk
