TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

### NOTE: there should only be one build.mak for a given IOC family and this should be located in the ###-IOC-01 directory

#=============================
# Build the IOC application TEKMSO4104B-IOC-01
# We actually use $(APPNAME) below so this file can be included by multiple IOCs

PROD_IOC = $(APPNAME)
# TEKMSO4104B-IOC-01.dbd will be created and installed
DBD += $(APPNAME).dbd

# TEKMSO4104B-IOC-01.dbd will be made up from these files:
$(APPNAME)_DBD += base.dbd
## ISIS standard dbd ##
#$(APPNAME)_DBD += icpconfig.dbd
#$(APPNAME)_DBD += pvdump.dbd
#$(APPNAME)_DBD += asSupport.dbd
#$(APPNAME)_DBD += devIocStats.dbd
#$(APPNAME)_DBD += caPutLog.dbd
#$(APPNAME)_DBD += utilities.dbd
## add other dbd here ##
$(APPNAME)_DBD += calc.dbd 
$(APPNAME)_DBD += asyn.dbd
$(APPNAME)_DBD += drvAsynSerialPort.dbd
$(APPNAME)_DBD += drvAsynIPPort.dbd
$(APPNAME)_DBD += stream.dbd
$(APPNAME)_DBD += drvVxi11.dbd

$(APPNAME)_DBD += ../../Db/incrementor.dbd
#$(APPNAME)_DBD += asubFunctions.dbd

# Add all the support libraries needed by this IOC
## ISIS standard libraries ##
#ALE# $(APPNAME)_LIBS += asubFunctions
#ALE# $(APPNAME)_LIBS += devIocStats 
#ALE# $(APPNAME)_LIBS += pvdump $(MYSQLLIB) easySQLite sqlite 
#ALE# $(APPNAME)_LIBS += caPutLog
#ALE# $(APPNAME)_LIBS += icpconfig pugixml
$(APPNAME)_LIBS += autosave
#ALE# $(APPNAME)_LIBS += utilities
## Add other libraries here ##
$(APPNAME)_LIBS += stream 
#$(APPNAME)_LIBS += pcrecpp pcre 
$(APPNAME)_LIBS += asyn 
$(APPNAME)_LIBS += calc sscan
$(APPNAME)_LIBS += seq pv
$(APPNAME)_LIBS_WIN32 += oncrpc
$(APPNAME)_SRCS += incrementor.c
# TEKMSO4104B-IOC-01_registerRecordDeviceDriver.cpp derives from TEKMSO4104B-IOC-01.dbd
$(APPNAME)_SRCS += $(APPNAME)_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
$(APPNAME)_SRCS_DEFAULT += $(APPNAME)Main.cpp
$(APPNAME)_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#$(APPNAME)_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
## area detector already includes PVA, so avoid including it twice
ifeq ($(AREA_DETECTOR),)
#include $(CONFIG)/CONFIG_PVA_ISIS
endif

$(APPNAME)_LIBS += $(EPICS_BASE_IOC_LIBS)
$(APPNAME)_SYS_LIBS_Linux += tirpc

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

