#!../../bin/linux-x86_64/TEKMSO58LP
## You may have to change TEKMSO4104B-IOC-01 to something else
## everywhere it appears in this file

# Increase this if you get <<TRUNCATED>> or discarded messages warnings in your errlog output
errlogInit2(65536, 256)

< envPaths

epicsEnvSet "STREAM_PROTOCOL_PATH" "$(TOP)/Tektronix_MSO58LPSup"
epicsEnvSet "EPICS_CA_MAX_ARRAY_BYTES" "100000"

epicsEnvSet "P" "SPARC:DIAG:TEK-TEST"

cd ${TOP}

## Register all support components
dbLoadDatabase "dbd/TEKMSO58LP.dbd"
TEKMSO58LP_registerRecordDeviceDriver pdbbase


#ALE##  Se si vuole dialogare vxi11 decommentare la prossima riga e commentare quella seguente.
#vxi11Configure("IP", "192.168.197.139", 0, 0.0,"inst0", 0, 0)
drvAsynIPPortConfigure("inst0", "192.168.197.139:4004")
## Load record instances


## Load our record instances
#ALE## Se si usa vxi11 sostituire inst0 con IP
dbLoadRecords("db/devmso58lp.db","P=$(P), PORT=inst0")

dbLoadRecords("db/channel.template","P=$(P),SCANTIME=1 second,CHANAME=Trigger,CHANNEL=7,SENSIBILITY=5.0,PORT=inst0,NELM=10000")
dbLoadRecords("db/channel.template","P=$(P),SCANTIME=1 second,CHANAME=BCM,CHANNEL=8,SENSIBILITY=2.0,PORT=inst0,NELM=10000")


##ISIS## Stuff that needs to be done after all records are loaded but before iocInit is called 
#ALE#< $(IOCSTARTUP)/preiocinit.cmd

cd ${TOP}/iocBoot/${IOC}
iocInit

## Start any sequence programs
#seq sncxxx,"user=mjc23Host"

##ISIS## Stuff that needs to be done after iocInit is called e.g. sequence programs 
#ALE#< $(IOCSTARTUP)/postiocinit.cmd
