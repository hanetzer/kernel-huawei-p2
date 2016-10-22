PRODUCT_NAME=U9700L
PRODUCT_BRAND=Huawei
PRODUCT_CHARACTERISTICS := nosdcard
#add macro to enable command type lcd
export USE_LCD_JDI_OTM1282B := true
#add macro to enable command type TP&TK
export USE_TP_TK_U9700L := true
#NOTICE! Do not config PRODUCT_DEVICE
export USE_WIFIDISPLAY_NO := true

export U9700_MODEM := true

PRODUCT_LCD_DISPLAY=HD

FELICA_SUPPORT=true
export USE_NFC_DEVICE := true

# disable tomoyo on 9700 general product
#--------------------------------------------------
# # for tomoyo mode: learning, debug, enforce
# export USE_TOMOYO_MODE=debug
# 
# PRODUCT_PACKAGES += \
# 		    ccs-init \
# 		    ccs-auditd
#-------------------------------------------------- 

PRODUCT_PACKAGES += TemperatureMonitor2
PRODUCT_PACKAGES += PartnerBookmarksProvider
#HANDLE_EMO_REQUEST := true
#CONFIG_FOR_DM_REQ := false

PRODUCT_PROPERTY_OVERRIDES += \
	com.huawei.HwBeam=true
	
PRODUCT_PROPERTY_OVERRIDES += \
	com.huawei.HwBeam.SoftAP=true	

DEVICE_PACKAGE_OVERLAYS := device/hisi/k3v2oem1/product_spec/overlay/$(PRODUCT_NAME) $(DEVICE_PACKAGE_OVERLAYS)
