#!/app/IOC-DEVEL/Tektronix_MSO58LP/bin/linux-x86_64/TEKMSO58LP
#
# st-all-asyn.cmd
# Startup script for Tektronix MSO58LP using ASYN driver
# 
# High-performance ASYN driver for waveform + measurement acquisition
# Configuration: SPARC:DIAG:TEK-TEST
#
# Channels (waveform acquisition):
#   CH1 - BEAMBPM01
#   CH2 - EOSPHD01
#   CH3 - PLADIS01
#   CH4 - PLALAS01
#
# Measurements:
#   MEAS1 - BEAMBPM01 (CH1)
#   MEAS2 - EOSPHD01 (CH2)
#   MEAS3 - PLADIS01 (CH3)
#   MEAS4 - PLALAS01 (CH4)
#

# Increase error log buffer for debugging
errlogInit2(65536, 256)

< envPaths

# Set EPICS environment variables
epicsEnvSet "EPICS_CA_MAX_ARRAY_BYTES" "3000000"

# PV prefix
epicsEnvSet "P" "SPARC:DIAG:TEK-TEST"

# Network configuration
epicsEnvSet "TEK_IP"   "ddsparctekmso001.lnf.infn.it"
epicsEnvSet "TEK_PORT" "4004"

# Acquisition settings
# POLL_TIME in seconds: 0.01 = 100Hz, 0.02 = 50Hz, 0.1 = 10Hz
epicsEnvSet "POLL_TIME" "0.1"
epicsEnvSet "NELM"      "10000"
epicsEnvSet "NUM_CH"    "8"
epicsEnvSet "NUM_MEAS"  "8"

cd ${TOP}

# Register all support components
dbLoadDatabase "dbd/TEKMSO58LP.dbd"
TEKMSO58LP_registerRecordDeviceDriver pdbbase

#============================================================
# Configure the ASYN driver for Tektronix MSO58LP
#============================================================
# drvTekMSO58LPConfigure(portName, ipAddress, ipPort, 
#                        numChannels, numMeasurements, 
#                        maxPoints, pollTime)
#
# Arguments:
#   portName        - ASYN port name (used in DB records)
#   ipAddress       - Oscilloscope IP address/hostname
#   ipPort          - TCP port (4004 for this device)
#   numChannels     - Number of channels to support (1-8)
#   numMeasurements - Number of measurements to support (1-10)
#   maxPoints       - Maximum waveform points per channel
#   pollTime        - Polling interval in seconds

drvTekMSO58LPConfigure("TEK1", "$(TEK_IP)", $(TEK_PORT), $(NUM_CH), $(NUM_MEAS), $(NELM), $(POLL_TIME))

# Enable ASYN trace for debugging (comment out for production)
# asynSetTraceMask("TEK1", 0, 0x09)
# asynSetTraceIOMask("TEK1", 0, 0x02)

#============================================================
# Load Database Records
#============================================================

# Global device records
dbLoadRecords("db/device_asyn.template", "P=$(P),PORT=TEK1,NELM=$(NELM),POLL_TIME=$(POLL_TIME)")

#============================================================
# Channel Waveform Acquisition
#============================================================

# CH1 - BEAMBPM01
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=1,NELM=$(NELM),CHANAME=BEAMBPM01,COEFF=1.0,CHARGE_EGU=pC")

# CH2 - EOSPHD01
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=2,NELM=$(NELM),CHANAME=EOSPHD01,COEFF=1.0,CHARGE_EGU=pC")

# CH3 - PLADIS01
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=3,NELM=$(NELM),CHANAME=PLADIS01,COEFF=1.0,CHARGE_EGU=pC")

# CH4 - PLALAS01
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=4,NELM=$(NELM),CHANAME=PLALAS01,COEFF=1.0,CHARGE_EGU=pC")



# CH7 - TEST
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=7,NELM=$(NELM),CHANAME=TEST,COEFF=1.0,CHARGE_EGU=pC")


# CH7 - TEST
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=8,NELM=$(NELM),CHANAME=TEST2,COEFF=1.0,CHARGE_EGU=pC")

#============================================================
# Measurement Devices
#============================================================

# # MEAS1 - BEAMBPM01 on Channel 1
# dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=1,CHANNEL=1,TYPE=MEAN,ENABLE=1,NAME=BEAMBPM01")

# # MEAS2 - EOSPHD01 on Channel 2
# dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=2,CHANNEL=2,TYPE=MEAN,ENABLE=1,NAME=EOSPHD01")

# # MEAS3 - PLADIS01 on Channel 3
# dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=3,CHANNEL=3,TYPE=MEAN,ENABLE=1,NAME=PLADIS01")

# # MEAS4 - PLALAS01 on Channel 4
# dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=4,CHANNEL=4,TYPE=MEAN,ENABLE=1,NAME=PLALAS01")

#============================================================
# Initialize IOC
#============================================================

cd ${TOP}/iocBoot/${IOC}
iocInit

#============================================================
# Post-initialization commands
#============================================================

# Enable measurements for acquisition (do after iocInit)
dbpf "$(P):MEAS1:Enable" "1"
dbpf "$(P):MEAS2:Enable" "1"
dbpf "$(P):MEAS3:Enable" "1"
dbpf "$(P):MEAS4:Enable" "1"

# Enable channels for waveform acquisition
dbpf "$(P):CH1:Enable" "1"
dbpf "$(P):CH2:Enable" "1"
dbpf "$(P):CH3:Enable" "1"
dbpf "$(P):CH4:Enable" "1"

# Set polling rate (can also be changed at runtime)
# dbpf "$(P):PollTime" "0.1"

# Print driver report
# drvTekMSO58LPReport TEK1 1
