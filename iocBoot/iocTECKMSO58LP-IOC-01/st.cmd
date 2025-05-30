#!../../bin/linux-x86_64/TEKMSO58LP-IOC-01

## You may have to change TEKMSO4104B-IOC-01 to something else
## everywhere it appears in this file

# Increase this if you get <<TRUNCATED>> or discarded messages warnings in your errlog output
errlogInit2(65536, 256)

< envPaths

epicsEnvSet "STREAM_PROTOCOL_PATH" "$(TOP)/Tektronix_MSO58LPSup"
epicsEnvSet "EPICS_CA_MAX_ARRAY_BYTES" "100000"

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/TEKMSO58LP-IOC-01.dbd"
TEKMSO58LP_IOC_01_registerRecordDeviceDriver pdbbase

##ISIS## Run IOC initialisation 
#ALE# < $(IOCSTARTUP)/init.cmd

#ALE##  Se si vuole dialogare vxi11 decommentare la prossima riga e commentare quella seguente.
#vxi11Configure("IP", "192.168.197.139", 0, 0.0,"inst0", 0, 0)
drvAsynIPPortConfigure("inst0", "192.168.197.139:4000")
## Load record instances

##ISIS## Load common DB records 
#ALE#< $(IOCSTARTUP)/dbload.cmd

## Load our record instances
#ALE## Se si usa vxi11 sostituire inst0 con IP
dbLoadRecords("db/devTektronix_MSO58LP.db","P=SPARC:TEST:$(IOC), PORT=inst0")

##ISIS## Stuff that needs to be done after all records are loaded but before iocInit is called 
#ALE#< $(IOCSTARTUP)/preiocinit.cmd

cd ${TOP}/iocBoot/${IOC}
iocInit

## Start any sequence programs
#seq sncxxx,"user=mjc23Host"

##ISIS## Stuff that needs to be done after iocInit is called e.g. sequence programs 
#ALE#< $(IOCSTARTUP)/postiocinit.cmd
