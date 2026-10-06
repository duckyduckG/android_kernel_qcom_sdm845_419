# SPDX-License-Identifier: GPL-2.0
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
	dtbo-$(CONFIG_ARCH_SDM845) += \
		sdm845-mtp-overlay.dtbo \
		sdm845-v2-mtp-overlay.dtbo \
		sda845-mtp-overlay.dtbo \
		sda845-v2-hdk-overlay.dtbo \
		sda845-v2-mtp-overlay.dtbo \
		sdm845-v2.1-mtp-overlay.dtbo \
		sda845-v2.1-mtp-overlay.dtbo \
		sdm845-v2-qvr-dvt-overlay.dtbo \
		sdm845-v2-qvr-evt-overlay.dtbo \
		sdm845-v2.1-oem-overlay.dtbo \
		sda845-v2.1-oem-overlay.dtbo

sdm845-mtp-overlay.dtbo-base := sdm845.dtb
sdm845-v2-mtp-overlay.dtbo-base := sdm845-v2.dtb
sda845-mtp-overlay.dtbo-base := sda845.dtb
sda845-v2-hdk-overlay.dtbo-base := sda845-v2.dtb
sda845-v2-mtp-overlay.dtbo-base := sda845-v2.dtb
sda845-v2.1-mtp-overlay.dtbo-base := sda845-v2.1.dtb
sdm845-v2.1-mtp-overlay.dtbo-base := sdm845-v2.1.dtb
sdm845-v2-qvr-dvt-overlay.dtbo-base := sdm845-v2.dtb
sdm845-v2-qvr-evt-overlay.dtbo-base := sdm845-v2.dtb
sdm845-v2.1-oem-overlay.dtbo-base := sdm845-v2.1.dtb
sda845-v2.1-oem-overlay.dtbo-base := sda845-v2.1.dtb
else
dtb-$(CONFIG_ARCH_SDM845) += \
	sdm845-mtp.dtb \
	sdm845-v2-mtp.dtb \
	sdm845-v2-qvr-dvt.dtb \
	sdm845-v2-qvr-evt.dtb \
	sdm845-v2.1-oem-overlay.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files            := *.dtb *.dtbo
