#!/app/IOC-DEVEL/Tektronix_MSO58LP/bin/linux-x86_64/TEKMSO58LP
#
# st-bcm-asyn.cmd
# Startup script for Tektronix MSO58LP using ASYN driver
# 
# This startup script uses the high-performance ASYN driver
# instead of StreamDevice for acquisition rates >10Hz
#
# Example: 100Hz polling for BCM charge monitoring
#

# Increase error log buffer for debugging
errlogInit2(65536, 256)

< envPaths

# Set EPICS environment variables
epicsEnvSet "EPICS_CA_MAX_ARRAY_BYTES" "3000000"

# PV prefix
epicsEnvSet "P" "SPARC:DIAG:TEK-BCM"

# Network configuration
epicsEnvSet "TEK_IP"   "192.168.197.195"
epicsEnvSet "TEK_PORT" "4000"

# Acquisition settings
# POLL_TIME in seconds: 0.01 = 100Hz, 0.02 = 50Hz, 0.1 = 10Hz
epicsEnvSet "POLL_TIME" "0.1"
epicsEnvSet "NELM"      "10000"
epicsEnvSet "NUM_CH"    "8"
epicsEnvSet "NUM_MEAS"  "10"

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
#   ipAddress       - Oscilloscope IP address
#   ipPort          - TCP port (typically 4000 for raw socket)
#   numChannels     - Number of channels to enable (1-8)
#   numMeasurements - Number of measurements to enable (1-10)
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

# Channel 1 - AC1BCM01 Beam Current Monitor
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=1,NELM=$(NELM),CHANAME=AC1BCM01,COEFF=2.1e11,CHARGE_EGU=pC")

# Channel 5 - Diagnostic
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=5,NELM=$(NELM),CHANAME=CH5,COEFF=2.1e11,CHARGE_EGU=pC")

# Channel 6 - DGL02 Diagnostic
dbLoadRecords("db/channel_asyn.template", "P=$(P),PORT=TEK1,CHANNEL=6,NELM=$(NELM),CHANAME=DGL02,COEFF=2,CHARGE_EGU=pC")

# Measurements for charge monitoring
dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=1,CHANNEL=1,TYPE=AREA,ENABLE=1,COEFF=2.1e11,CHARGE_EGU=pC")
dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=5,CHANNEL=5,TYPE=AREA,ENABLE=1,COEFF=2.1e11,CHARGE_EGU=pC")
dbLoadRecords("db/measurement_asyn.template", "P=$(P),PORT=TEK1,MEAS=6,CHANNEL=6,TYPE=AREA,ENABLE=1,COEFF=2.1e11,CHARGE_EGU=pC")

#============================================================
# Initialize IOC
#============================================================

cd ${TOP}/iocBoot/${IOC}
iocInit

#============================================================
# Post-initialization commands
#============================================================

# Enable channels for acquisition (do after iocInit)
dbpf "$(P):CH1:Enable" "1"
dbpf "$(P):CH5:Enable" "1" 
dbpf "$(P):CH6:Enable" "1"

# Set polling rate (can also be changed at runtime)
# dbpf "$(P):PollTime" "0.01"

# Print driver report
# drvTekMSO58LPReport TEK1 1

# Display acquisition status
echo "=========================================="
echo "Tektronix MSO58LP ASYN Driver Started"
echo "  IP: $(TEK_IP):$(TEK_PORT)"
echo "  Poll Rate: $(POLL_TIME)s = $((1.0/$(POLL_TIME))) Hz"
echo "  Max Points: $(NELM)"
echo "  Channels: $(NUM_CH), Measurements: $(NUM_MEAS)"
echo "=========================================="
