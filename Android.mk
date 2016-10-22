#Android makefile to build kernel as a part of Android Build

#ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/arm/boot/zImage

KERNEL_ARCH_ARM_CONFIGS := kernel/arch/arm/configs
KERNEL_GEN_CONFIG_FILE := huawei_k3v2oem1_$(HW_PRODUCT)_defconfig
KERNEL_GEN_CONFIG_PATH := $(KERNEL_ARCH_ARM_CONFIGS)/$(KERNEL_GEN_CONFIG_FILE)

KERNEL_COMMON_DEFCONFIG := $(KERNEL_ARCH_ARM_CONFIGS)/$(KERNEL_DEFCONFIG)
KERNEL_PRODUCT_CONFIGS  := device/hisi/k3v2oem1/product_spec/kernel_config/$(HW_PRODUCT)

$(shell cd device/hisi/customize/hsad;./xml2complete.sh > /dev/null)
$(shell cd kernel/drivers/huawei/hsad;./xml2code.sh)

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)
$(KERNEL_GEN_CONFIG_PATH): FORCE
	$(shell device/hisi/k3v2oem1/kernel-config.sh -f $(KERNEL_COMMON_DEFCONFIG) -d $(KERNEL_PRODUCT_CONFIGS) -o $(KERNEL_GEN_CONFIG_PATH))

$(KERNEL_CONFIG): $(KERNEL_OUT) $(KERNEL_GEN_CONFIG_PATH)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-linux-androideabi- $(KERNEL_GEN_CONFIG_FILE)
	@rm -frv $(KERNEL_GEN_CONFIG_PATH)


$(TARGET_PREBUILT_KERNEL): $(KERNEL_CONFIG)
	$(hide) $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-linux-androideabi- -j 18
	$(hide) $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-linux-androideabi- -j 18 zImage

kernelconfig: $(KERNEL_OUT) $(KERNEL_GEN_CONFIG_PATH)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-linux-androideabi- $(KERNEL_GEN_CONFIG_FILE) menuconfig
	@rm -frv $(KERNEL_GEN_CONFIG_PATH)
