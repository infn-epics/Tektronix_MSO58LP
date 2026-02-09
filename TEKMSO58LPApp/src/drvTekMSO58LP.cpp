/**
 * @file drvTekMSO58LP.cpp
 * @brief ASYN Port Driver Implementation for Tektronix MSO58LP Oscilloscope
 * 
 * High-performance driver supporting >10Hz acquisition rates through:
 * - Direct socket communication (no StreamDevice overhead)
 * - Dedicated polling thread with configurable rate
 * - Pre-allocated waveform buffers
 * - Binary data transfer for faster waveform acquisition
 * - Thread-safe parameter access
 * 
 * @author EPICS Team
 * @date 2026
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <algorithm>
#include <stdarg.h>

#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <asynOctetSyncIO.h>
#include <drvAsynIPPort.h>

#include "drvTekMSO58LP.h"

/* Timeout definitions */
#define TEK_WRITE_TIMEOUT   1.0
#define TEK_READ_TIMEOUT    2.0
#define TEK_WAVEFORM_TIMEOUT 5.0

/* Communication buffer sizes */
#define TEK_CMD_BUFFER_SIZE     256
#define TEK_RESPONSE_BUFFER_SIZE 65536
#define TEK_WAVEFORM_BUFFER_SIZE (TEK_MAX_WAVEFORM_PTS * 12)

static const char *driverName = "drvTekMSO58LP";

/* Static C function to be called by epicsThread */
static void pollerThreadC(void *pPvt)
{
    drvTekMSO58LP *pDriver = (drvTekMSO58LP *)pPvt;
    pDriver->pollerThread();
}

/**
 * Constructor - creates and initializes the driver
 */
drvTekMSO58LP::drvTekMSO58LP(const char *portName, const char *ipAddress, 
                             int ipPort, int numChannels, int numMeasurements,
                             int maxPoints, double pollTime)
    : asynPortDriver(portName,
                     std::max(numChannels, numMeasurements), /* maxAddr */
                     asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | 
                     asynInt32ArrayMask | asynOctetMask | asynDrvUserMask,
                     asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask |
                     asynOctetMask, /* Interrupt mask */
                     ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                     1, /* autoConnect */
                     0, 0), /* priority, stackSize - use defaults */
      ipPort_(ipPort),
      numChannels_(numChannels),
      numMeasurements_(numMeasurements),
      maxPoints_(maxPoints),
      pollTime_(pollTime),
      pasynUserOctet_(NULL),
      pasynOctet_(NULL),
      octetPvt_(NULL),
      connected_(0),
      pollerThreadId_(NULL),
      pollerEvent_(NULL),
      pollerRunning_(0),
      pollerExitRequest_(0),
      pollCount_(0),
      errorCount_(0),
      debugLevel_(TEK_DEBUG_ERROR)  /* Default: only errors */
{
    static const char *functionName = "drvTekMSO58LP";
    asynStatus status;
    char asynPortName[64];
    int i;
    char paramName[64];

    /* Save IP address */
    strncpy(ipAddress_, ipAddress, TEK_MAX_STRING_SIZE - 1);
    ipAddress_[TEK_MAX_STRING_SIZE - 1] = '\0';

    /* Validate parameters */
    if (numChannels_ > TEK_MAX_CHANNELS) numChannels_ = TEK_MAX_CHANNELS;
    if (numMeasurements_ > TEK_MAX_MEASUREMENTS) numMeasurements_ = TEK_MAX_MEASUREMENTS;
    if (maxPoints_ > TEK_MAX_WAVEFORM_PTS) maxPoints_ = TEK_MAX_WAVEFORM_PTS;
    if (pollTime_ < 0.001) pollTime_ = 0.001;  /* Minimum 1ms */

    /* Create data lock mutex */
    dataLock_ = epicsMutexCreate();

    /* Create global parameters */
    createParam(TEK_IDN_STRING,           asynParamOctet,   &P_Idn);
    createParam(TEK_VENDOR_STRING,        asynParamOctet,   &P_Vendor);
    createParam(TEK_MODEL_STRING,         asynParamOctet,   &P_Model);
    createParam(TEK_SERIAL_STRING,        asynParamOctet,   &P_Serial);
    createParam(TEK_FIRMWARE_STRING,      asynParamOctet,   &P_Firmware);
    createParam(TEK_CONNECTED_STRING,     asynParamInt32,   &P_Connected);
    createParam(TEK_POLL_TIME_STRING,     asynParamFloat64, &P_PollTime);
    createParam(TEK_RECORD_LENGTH_STRING, asynParamInt32,   &P_RecordLength);
    createParam(TEK_ACQUIRE_STRING,       asynParamInt32,   &P_Acquire);
    createParam(TEK_RUN_STOP_STRING,      asynParamInt32,   &P_RunStop);
    
    /* Diagnostic parameters */
    createParam(TEK_POLL_COUNT_STRING,    asynParamInt32,   &P_PollCount);
    createParam(TEK_ERROR_COUNT_STRING,   asynParamInt32,   &P_ErrorCount);
    createParam(TEK_LAST_ERROR_STRING,    asynParamOctet,   &P_LastError);
    createParam(TEK_COMM_TIME_STRING,     asynParamFloat64, &P_CommTime);
    createParam(TEK_DEBUG_LEVEL_STRING,   asynParamInt32,   &P_DebugLevel);
    
    /* Set initial debug level */
    setIntegerParam(P_DebugLevel, debugLevel_);

    /* Create per-channel parameters */
    for (i = 0; i < numChannels_; i++) {
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_ENABLE_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChEnable[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_LABEL_STRING, i+1);
        createParam(paramName, asynParamOctet, &P_ChLabel[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_XINC_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChXinc[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_YMULT_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChYmult[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_YOFF_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChYoff[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_YZERO_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChYzero[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_XUNIT_STRING, i+1);
        createParam(paramName, asynParamOctet, &P_ChXunit[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_YUNIT_STRING, i+1);
        createParam(paramName, asynParamOctet, &P_ChYunit[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_NR_PT_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChNrPt[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_WAVEFORM_STRING, i+1);
        createParam(paramName, asynParamInt32Array, &P_ChWaveform[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_WAVEFORM_SCALED_STRING, i+1);
        createParam(paramName, asynParamFloat64Array, &P_ChWaveformScaled[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_WAVEFORM_TIME_STRING, i+1);
        createParam(paramName, asynParamFloat64Array, &P_ChWaveformTime[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_COUPLING_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChCoupling[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_SCALE_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChScale[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_OFFSET_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChOffset[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_POSITION_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChPosition[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_BANDWIDTH_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChBandwidth[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_DATA_READY_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChDataReady[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_DATA_START_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChDataStart[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_DATA_STOP_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChDataStop[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_REFRESH_RATE_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChRefreshRate[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_RESET_STATS_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChResetStats[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_DATA_WIDTH_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChDataWidth[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_MARKER_START_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChMarkerStart[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_MARKER_END_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChMarkerEnd[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_MARKER_START_SAMPLE_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChMarkerStartSample[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_MARKER_END_SAMPLE_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_ChMarkerEndSample[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_MEAN_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsMean[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_MIN_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsMin[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_MAX_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsMax[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_RMS_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsRMS[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_INTEGRAL_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsIntegral[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_PP_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsPP[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_CH_STATS_STDDEV_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_ChStatsStdDev[i]);
        
        /* Allocate waveform buffers */
        rawWaveform_[i] = (epicsInt32 *)calloc(maxPoints_, sizeof(epicsInt32));
        scaledWaveform_[i] = (epicsFloat64 *)calloc(maxPoints_, sizeof(epicsFloat64));
        timeArray_[i] = (epicsFloat64 *)calloc(maxPoints_, sizeof(epicsFloat64));
        waveformLength_[i] = 0;
        xinc_[i] = 1.0;
        ymult_[i] = 1.0;
        yoff_[i] = 0.0;
        yzero_[i] = 0.0;
        firstAcq_[i] = 1;
        configRead_[i] = 0;
        lastDataWidth_[i] = 0;  /* 0 = not yet sent */
        memset(&lastAcqTime_[i], 0, sizeof(epicsTimeStamp));
        
        /* Set default enable state */
        setIntegerParam(P_ChEnable[i], 0);
        setDoubleParam(P_ChRefreshRate[i], 0.0);
        setIntegerParam(P_ChResetStats[i], 0);
        setIntegerParam(P_ChDataWidth[i], 2);
        setDoubleParam(P_ChMarkerStart[i], 0.0);
        setDoubleParam(P_ChMarkerEnd[i], 0.0);
        setIntegerParam(P_ChMarkerStartSample[i], 0);
        setIntegerParam(P_ChMarkerEndSample[i], 0);
        setDoubleParam(P_ChStatsMean[i], 0.0);
        setDoubleParam(P_ChStatsMin[i], 0.0);
        setDoubleParam(P_ChStatsMax[i], 0.0);
        setDoubleParam(P_ChStatsRMS[i], 0.0);
        setDoubleParam(P_ChStatsIntegral[i], 0.0);
        setDoubleParam(P_ChStatsPP[i], 0.0);
        setDoubleParam(P_ChStatsStdDev[i], 0.0);
        
        /* Set default data window: 1 to maxPoints */
        setIntegerParam(P_ChDataStart[i], 1);
        setIntegerParam(P_ChDataStop[i], maxPoints_);
    }

    /* Initialize DATa tracking */
    lastDataSource_ = -1;

    /* Create per-measurement parameters */
    for (i = 0; i < numMeasurements_; i++) {
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_ENABLE_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_MeasEnable[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_SOURCE_STRING, i+1);
        createParam(paramName, asynParamInt32, &P_MeasSource[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_TYPE_STRING, i+1);
        createParam(paramName, asynParamOctet, &P_MeasType[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_VALUE_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_MeasValue[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_MEAN_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_MeasMean[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_MIN_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_MeasMin[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_MAX_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_MeasMax[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_STDDEV_STRING, i+1);
        createParam(paramName, asynParamFloat64, &P_MeasStddev[i]);
        
        snprintf(paramName, sizeof(paramName), "%s_%d", TEK_MEAS_UNITS_STRING, i+1);
        createParam(paramName, asynParamOctet, &P_MeasUnits[i]);
        
        setIntegerParam(P_MeasEnable[i], 0);
    }

    /* Set initial values */
    setDoubleParam(P_PollTime, pollTime_);
    setIntegerParam(P_RecordLength, maxPoints_);
    setIntegerParam(P_Connected, 0);
    setIntegerParam(P_PollCount, 0);
    setIntegerParam(P_ErrorCount, 0);
    setIntegerParam(P_Acquire, 1);
    setIntegerParam(P_RunStop, 1);

    /* Create the underlying asyn octet port for TCP communication */
    snprintf(asynPortName, sizeof(asynPortName), "%s_TCP", portName);
    
    char ipPortStr[64];
    snprintf(ipPortStr, sizeof(ipPortStr), "%s:%d", ipAddress_, ipPort_);
    
    /* Configure IP port - use drvAsynIPPortConfigure */
    status = (asynStatus)drvAsynIPPortConfigure(asynPortName, ipPortStr, 0, 0, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Error calling drvAsynIPPortConfigure for %s: %d\n",
            driverName, functionName, ipPortStr, status);
    }

    /* Connect to the asyn octet interface */
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserOctet_, NULL);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Error connecting to octet port %s: %d\n",
            driverName, functionName, asynPortName, status);
        pasynUserOctet_ = NULL;
    }

    /* Set input/output terminators (only if connected) */
    if (pasynUserOctet_) {
        pasynOctetSyncIO->setInputEos(pasynUserOctet_, "\n", 1);
        pasynOctetSyncIO->setOutputEos(pasynUserOctet_, "\n", 1);
    }

    /* Create event for signaling the poller */
    pollerEvent_ = epicsEventCreate(epicsEventEmpty);

    /* Create the polling thread */
    pollerRunning_ = 1;
    pollerThreadId_ = epicsThreadCreate("TekMSO58LP_Poller",
                                        epicsThreadPriorityMedium,
                                        epicsThreadGetStackSize(epicsThreadStackMedium),
                                        pollerThreadC, this);

    if (pollerThreadId_ == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: Error creating poller thread\n",
            driverName, functionName);
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Driver initialized for %s:%d, %d channels, %d measurements, poll=%.3fs\n",
        driverName, functionName, ipAddress_, ipPort_, 
        numChannels_, numMeasurements_, pollTime_);
}

/**
 * Destructor
 */
drvTekMSO58LP::~drvTekMSO58LP()
{
    int i;

    /* Signal poller thread to exit */
    pollerExitRequest_ = 1;
    epicsEventSignal(pollerEvent_);
    
    /* Wait for thread to exit (with timeout) */
    epicsThreadSleep(0.5);

    /* Free waveform buffers */
    for (i = 0; i < numChannels_; i++) {
        if (rawWaveform_[i]) free(rawWaveform_[i]);
        if (scaledWaveform_[i]) free(scaledWaveform_[i]);
        if (timeArray_[i]) free(timeArray_[i]);
    }

    /* Disconnect from scope */
    if (pasynUserOctet_) {
        pasynOctetSyncIO->disconnect(pasynUserOctet_);
    }

    /* Destroy mutex and event */
    if (dataLock_) epicsMutexDestroy(dataLock_);
    if (pollerEvent_) epicsEventDestroy(pollerEvent_);
}

/**
 * Debug print function with level filtering
 * Only prints if level <= debugLevel_
 */
void drvTekMSO58LP::debugPrint(int level, const char *format, ...)
{
    va_list args;
    
    if (level > debugLevel_) return;
    
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

/**
 * Polling thread - main acquisition loop
 */
void drvTekMSO58LP::pollerThread()
{
    static const char *functionName = "pollerThread";
    int acquire;
    int connected;
    epicsTimeStamp startTime, endTime;
    double elapsedTime;
    int i;
    int enabledCh;
    int enabledMeas;

    debugPrint(TEK_DEBUG_INFO, "DEBUG: %s:%s: Poller thread started\n", driverName, functionName);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Poller thread started\n", driverName, functionName);

    while (!pollerExitRequest_) {
        /* Get current settings */
        lock();
        getIntegerParam(P_Acquire, &acquire);
        getIntegerParam(P_Connected, &connected);
        getDoubleParam(P_PollTime, &pollTime_);
        unlock();

        debugPrint(TEK_DEBUG_TRACE, "DEBUG: pollerThread loop: acquire=%d, connected=%d, pollTime=%.3f\n", 
               acquire, connected, pollTime_);

        if (acquire && connected) {
            epicsTimeGetCurrent(&startTime);

            /* Read enabled channels */
            for (i = 0; i < numChannels_; i++) {
                lock();
                getIntegerParam(P_ChEnable[i], &enabledCh);
                unlock();

                if (enabledCh) {
                    debugPrint(TEK_DEBUG_TRACE, "DEBUG: Reading channel %d (enabled)\n", i+1);
                    /* Read channel configuration on first enable or periodically */
                    if (!configRead_[i] || (pollCount_ % 100) == 0) {
                        debugPrint(TEK_DEBUG_INFO, "DEBUG: Reading channel %d config (first=%d)\n", i+1, !configRead_[i]);
                        readChannelConfig(i);
                        configRead_[i] = 1;
                    }
                    /* Read waveform data */
                    debugPrint(TEK_DEBUG_TRACE, "DEBUG: Reading channel %d waveform\n", i+1);
                    readWaveformBinary(i);
                    debugPrint(TEK_DEBUG_TRACE, "DEBUG: Channel %d waveform done\n", i+1);
                }
            }

            /* Read enabled measurements */
            for (i = 0; i < numMeasurements_; i++) {
                lock();
                getIntegerParam(P_MeasEnable[i], &enabledMeas);
                unlock();

                if (enabledMeas) {
                    readMeasurement(i);
                }
            }

            /* Update statistics */
            lock();
            pollCount_++;
            setIntegerParam(P_PollCount, (int)pollCount_);

            epicsTimeGetCurrent(&endTime);
            elapsedTime = epicsTimeDiffInSeconds(&endTime, &startTime);
            setDoubleParam(P_CommTime, elapsedTime);

            /* Post updates to all clients */
            callParamCallbacks();
            unlock();

        } else if (!connected) {
            /* Try to connect - but only if pasynUserOctet_ is valid */
            debugPrint(TEK_DEBUG_INFO, "DEBUG: Not connected, pasynUserOctet_=%p\n", (void*)pasynUserOctet_);
            if (pasynUserOctet_) {
                debugPrint(TEK_DEBUG_INFO, "DEBUG: Attempting to connect...\n");
                asynStatus status = connectToScope();
                if (status == asynSuccess) {
                    lock();
                    setIntegerParam(P_Connected, 1);
                    connected_ = 1;
                    callParamCallbacks();
                    unlock();

                    /* Read identification after connect */
                    readIdentification();
                }
            }
        }

        /* Sleep for poll interval */
        epicsEventWaitWithTimeout(pollerEvent_, pollTime_);
    }

    pollerRunning_ = 0;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Poller thread exiting\n", driverName, functionName);
}

/**
 * Connect to the oscilloscope
 */
asynStatus drvTekMSO58LP::connectToScope()
{
    static const char *functionName = "connectToScope";
    asynStatus status;
    char response[256];
    size_t nWrite, nRead;
    int eomReason;

    printf("DEBUG: connectToScope() called\n");
    fflush(stdout);

    /* Check if octet interface is available */
    if (!pasynUserOctet_) {
        debugPrint(TEK_DEBUG_ERROR, "DEBUG: connectToScope - pasynUserOctet_ is NULL!\n");
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: pasynUserOctet_ is NULL\n", driverName, functionName);
        return asynError;
    }

    debugPrint(TEK_DEBUG_INFO, "DEBUG: connectToScope - calling writeRead for *IDN?\n");

    /* Try a simple query to verify connection */
    status = pasynOctetSyncIO->writeRead(pasynUserOctet_, "*IDN?", 5,
                                         response, sizeof(response)-1,
                                         TEK_DEFAULT_TIMEOUT, &nWrite, &nRead, &eomReason);
    
    debugPrint(TEK_DEBUG_INFO, "DEBUG: connectToScope - writeRead returned status=%d, nRead=%zu\n", status, nRead);
    
    if (status == asynSuccess && nRead > 0) {
        response[nRead] = '\0';
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: Connected to %s\n", driverName, functionName, response);
        
        /* Configure scope for optimized communication */
        pasynOctetSyncIO->write(pasynUserOctet_, "HEADer OFF", 10,
                                TEK_WRITE_TIMEOUT, &nWrite);
        pasynOctetSyncIO->write(pasynUserOctet_, "VERBose OFF", 11,
                                TEK_WRITE_TIMEOUT, &nWrite);
        /* Use SRIbinary: signed integer, little-endian (native x86 byte order) */
        pasynOctetSyncIO->write(pasynUserOctet_, ":DATa:ENCdg SRIbinary", 21,
                                TEK_WRITE_TIMEOUT, &nWrite);
        
        /* Reset tracking so DATa:SOUrce/WIDth are re-sent on first use */
        lastDataSource_ = -1;
        for (int j = 0; j < numChannels_; j++) {
            lastDataWidth_[j] = 0;
            configRead_[j] = 0;
        }
        
        return asynSuccess;
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: Failed to connect\n", driverName, functionName);
    return asynError;
}

/**
 * Disconnect from the oscilloscope
 */
asynStatus drvTekMSO58LP::disconnectFromScope()
{
    connected_ = 0;
    setIntegerParam(P_Connected, 0);
    return asynSuccess;
}

/**
 * Low-level write/read with error handling
 */
asynStatus drvTekMSO58LP::writeRead(const char *writeBuffer, char *readBuffer,
                                    size_t readBufferSize, size_t *nRead, double timeout)
{
    static const char *functionName = "writeRead";
    asynStatus status;
    size_t nWrite;
    int eomReason;

    if (!pasynUserOctet_) {
        return asynError;
    }

    status = pasynOctetSyncIO->writeRead(pasynUserOctet_,
                                         writeBuffer, strlen(writeBuffer),
                                         readBuffer, readBufferSize - 1,
                                         timeout, &nWrite, nRead, &eomReason);

    if (status != asynSuccess) {
        errorCount_++;
        setIntegerParam(P_ErrorCount, (int)errorCount_);
        setStringParam(P_LastError, writeBuffer);
        
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: writeRead failed for '%s': %d\n",
            driverName, functionName, writeBuffer, status);
        
        /* Check if disconnected */
        if (status == asynTimeout || status == asynDisconnected) {
            disconnectFromScope();
        }
    } else {
        readBuffer[*nRead] = '\0';
    }

    return status;
}

/**
 * Write a command without reading response
 */
asynStatus drvTekMSO58LP::writeCmd(const char *cmd)
{
    asynStatus status;
    size_t nWrite;

    if (!pasynUserOctet_) {
        return asynError;
    }

    status = pasynOctetSyncIO->write(pasynUserOctet_, cmd, strlen(cmd),
                                     TEK_WRITE_TIMEOUT, &nWrite);
    return status;
}

/**
 * Query (write and read response)
 */
asynStatus drvTekMSO58LP::query(const char *cmd, char *response, size_t maxLen)
{
    size_t nRead;
    return writeRead(cmd, response, maxLen, &nRead, TEK_READ_TIMEOUT);
}

/**
 * Read device identification
 */
asynStatus drvTekMSO58LP::readIdentification()
{
    static const char *functionName = "readIdentification";
    char response[256];
    asynStatus status;
    char vendor[64], model[64], serial[64], firmware[64];

    status = query("*IDN?", response, sizeof(response));
    if (status != asynSuccess) return status;

    /* Parse: TEKTRONIX,MSO58LP,SERIALNUM,CF:91.1 FV:1.2.3 */
    if (sscanf(response, "%[^,],%[^,],%[^,],%s",
               vendor, model, serial, firmware) >= 3) {
        lock();
        setStringParam(P_Idn, response);
        setStringParam(P_Vendor, vendor);
        setStringParam(P_Model, model);
        setStringParam(P_Serial, serial);
        setStringParam(P_Firmware, firmware);
        callParamCallbacks();
        unlock();

        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: ID: %s %s %s\n", driverName, functionName, vendor, model, serial);
    }

    return asynSuccess;
}

/**
 * Read channel configuration (scale factors, etc.)
 * Uses individual WFMOutpre queries for reliability.
 * Must be called AFTER DATa:SOUrce and DATa:WIDth are set for this channel.
 */
asynStatus drvTekMSO58LP::readChannelConfig(int channel)
{
    char cmd[64], response[256];
    asynStatus status;
    int ch = channel + 1;  /* 1-indexed */
    int dataWidth;

    /* Ensure DATa:SOUrce is set to this channel */
    if (lastDataSource_ != channel) {
        snprintf(cmd, sizeof(cmd), ":DATa:SOUrce CH%d", ch);
        writeCmd(cmd);
        lastDataSource_ = channel;
    }

    /* Set DATa:WIDth for this channel (affects YMUlt scaling) */
    getIntegerParam(P_ChDataWidth[channel], &dataWidth);
    if (dataWidth != 1 && dataWidth != 2) dataWidth = 2;
    if (lastDataWidth_[channel] != dataWidth) {
        snprintf(cmd, sizeof(cmd), ":DATa:WIDth %d", dataWidth);
        writeCmd(cmd);
        lastDataWidth_[channel] = dataWidth;
    }

    /* Query each parameter individually for reliable parsing.
     * With HEADER OFF, each response is just the value (e.g. "80.0000E-9").
     * With HEADER ON, response has prefix (e.g. ":WFMOUTPRE:XINCR 80.0000E-9"). */

    double xinc = 0, ymult = 0, yoff = 0, yzero = 0;
    int nrPt = 0;
    char xunit[32] = "", yunit[32] = "";

    /* Helper: extract numeric value from response (handles both HEADER ON/OFF) */
    /* With HEADER OFF: "80.0E-9"  With HEADER ON: ":WFMOUTPRE:XINCR 80.0E-9" */

    status = query("WFMOutpre:NR_Pt?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        nrPt = atoi(val ? val + 1 : response);
        debugPrint(TEK_DEBUG_INFO, "DEBUG: readChannelConfig ch%d NR_Pt raw='%s' parsed=%d\n", ch, response, nrPt);
    } else {
        debugPrint(TEK_DEBUG_ERROR, "DEBUG: readChannelConfig ch%d NR_Pt query FAILED\n", ch);
    }

    status = query("WFMOutpre:XINcr?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        xinc = atof(val ? val + 1 : response);
        debugPrint(TEK_DEBUG_INFO, "DEBUG: readChannelConfig ch%d XINcr raw='%s' parsed=%e\n", ch, response, xinc);
    } else {
        debugPrint(TEK_DEBUG_ERROR, "DEBUG: readChannelConfig ch%d XINcr query FAILED\n", ch);
    }

    status = query("WFMOutpre:XUNit?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        char *p = val ? val + 1 : response;
        if (*p == '"') p++;
        char *end = strchr(p, '"');
        if (end) *end = '\0';
        strncpy(xunit, p, sizeof(xunit) - 1);
    }

    status = query("WFMOutpre:YMUlt?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        ymult = atof(val ? val + 1 : response);
        debugPrint(TEK_DEBUG_INFO, "DEBUG: readChannelConfig ch%d YMUlt raw='%s' parsed=%e\n", ch, response, ymult);
    } else {
        debugPrint(TEK_DEBUG_ERROR, "DEBUG: readChannelConfig ch%d YMUlt query FAILED\n", ch);
    }

    status = query("WFMOutpre:YOFf?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        yoff = atof(val ? val + 1 : response);
    }

    status = query("WFMOutpre:YZEro?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        yzero = atof(val ? val + 1 : response);
    }
    
    /* Warn if critical values are zero */
    if (xinc == 0.0) {
        debugPrint(TEK_DEBUG_ERROR, "WARNING: readChannelConfig ch%d xinc=0! Time array will be all zeros\n", ch);
    }
    if (ymult == 0.0) {
        debugPrint(TEK_DEBUG_ERROR, "WARNING: readChannelConfig ch%d ymult=0! Waveform will be all zeros\n", ch);
    }

    status = query("WFMOutpre:YUNit?", response, sizeof(response));
    if (status == asynSuccess) {
        char *val = strrchr(response, ' ');
        char *p = val ? val + 1 : response;
        if (*p == '"') p++;
        char *end = strchr(p, '"');
        if (end) *end = '\0';
        strncpy(yunit, p, sizeof(yunit) - 1);
    }

    /* Update cache and parameters */
    xinc_[channel] = xinc;
    ymult_[channel] = ymult;
    yoff_[channel] = yoff;
    yzero_[channel] = yzero;

    lock();
    setDoubleParam(P_ChXinc[channel], xinc);
    setDoubleParam(P_ChYmult[channel], ymult);
    setDoubleParam(P_ChYoff[channel], yoff);
    setDoubleParam(P_ChYzero[channel], yzero);
    setIntegerParam(P_ChNrPt[channel], nrPt);
    if (xunit[0]) setStringParam(P_ChXunit[channel], xunit);
    if (yunit[0]) setStringParam(P_ChYunit[channel], yunit);
    unlock();

    debugPrint(TEK_DEBUG_INFO, "DEBUG: readChannelConfig: ch%d width=%d xinc=%e ymult=%e yoff=%e yzero=%e nrPt=%d\n",
               ch, dataWidth, xinc, ymult, yoff, yzero, nrPt);

    return asynSuccess;
}

/**
 * Read waveform using binary transfer (faster than ASCII)
 * 
 * Optimization: DATa:SOUrce, DATa:ENCdg, DATa:WIDth are only sent when changed.
 * DATa:ENCdg is set once at connect (SRIbinary = little-endian signed integer).
 * Y scaling params (YMUlt, YOFf, YZEro) are read in readChannelConfig().
 */
asynStatus drvTekMSO58LP::readWaveformBinary(int channel)
{
    char cmd[128];
    char *response;
    asynStatus status;
    size_t nWrite, nRead;
    int eomReason;
    int ch = channel + 1;
    int numPoints;
    int i;
    int dataStart, dataStop;
    int dataWidth;

    /* Check if connected */
    if (!pasynUserOctet_) {
        return asynError;
    }

    /* Get per-channel data window */
    getIntegerParam(P_ChDataStart[channel], &dataStart);
    getIntegerParam(P_ChDataStop[channel], &dataStop);
    getIntegerParam(P_ChDataWidth[channel], &dataWidth);
    
    /* Validate */
    if (dataWidth != 1 && dataWidth != 2) dataWidth = 2;
    if (dataStart < 1) dataStart = 1;
    if (dataStop > maxPoints_) dataStop = maxPoints_;
    if (dataStop < dataStart) dataStop = dataStart;
    
    int windowSize = dataStop - dataStart + 1;

    debugPrint(TEK_DEBUG_TRACE, "DEBUG: readWaveformBinary: channel=%d, ch=%d, start=%d, stop=%d, size=%d\n", 
               channel, ch, dataStart, dataStop, windowSize);

    /* Allocate large buffer for waveform data */
    response = (char *)malloc(TEK_WAVEFORM_BUFFER_SIZE);
    if (!response) {
        debugPrint(TEK_DEBUG_ERROR, "DEBUG: readWaveformBinary: malloc failed!\n");
        return asynError;
    }

    /* Only send DATa:SOUrce when channel changes */
    if (lastDataSource_ != channel) {
        snprintf(cmd, sizeof(cmd), ":DATa:SOUrce CH%d", ch);
        writeCmd(cmd);
        lastDataSource_ = channel;
    }
    
    /* Only send DATa:WIDth when width changes for this channel */
    if (lastDataWidth_[channel] != dataWidth) {
        snprintf(cmd, sizeof(cmd), ":DATa:WIDth %d", dataWidth);
        writeCmd(cmd);
        lastDataWidth_[channel] = dataWidth;
        /* Width changed: must re-read config for updated YMUlt */
        configRead_[channel] = 0;
    }
    
    /* DATa:STARt and DATa:STOP - always set (cheap, values may change) */
    snprintf(cmd, sizeof(cmd), ":DATa:STARt %d", dataStart);
    writeCmd(cmd);
    snprintf(cmd, sizeof(cmd), ":DATa:STOP %d", dataStop);
    writeCmd(cmd);

    /* Query the curve data */
    debugPrint(TEK_DEBUG_INFO, "DEBUG: readWaveformBinary: querying CURVe?, window=%d-%d (%d points)\n", dataStart, dataStop, windowSize);
    status = pasynOctetSyncIO->writeRead(pasynUserOctet_,
                                         ":CURVe?", 7,
                                         response, TEK_WAVEFORM_BUFFER_SIZE - 1,
                                         TEK_WAVEFORM_TIMEOUT, &nWrite, &nRead, &eomReason);
    debugPrint(TEK_DEBUG_INFO, "DEBUG: readWaveformBinary: CURVe returned status=%d, nRead=%zu\n", status, nRead);

    if (status != asynSuccess || nRead == 0) {
        debugPrint(TEK_DEBUG_WARNING, "DEBUG: readWaveformBinary: CURVe failed, freeing buffer\n");
        free(response);
        return status;
    }

    debugPrint(TEK_DEBUG_TRACE, "DEBUG: readWaveformBinary: parsing binary data, nRead=%zu\n", nRead);

    /* Parse IEEE 488.2 definite length binary block
     * Format: #<n><length><binary_data>
     * <n> = number of digits in length field
     * <length> = byte count of binary data
     * SRIbinary encoding: signed integer, little-endian (native x86 byte order)
     */
    epicsMutexLock(dataLock_);
    
    numPoints = 0;
    char *dataPtr = response;
    
    /* Check for # header */
    if (*dataPtr == '#') {
        dataPtr++;
        int numDigits = *dataPtr - '0';  /* Number of digits in length field */
        dataPtr++;
        
        if (numDigits > 0 && numDigits < 10) {
            /* Parse the length field */
            char lenStr[16];
            strncpy(lenStr, dataPtr, numDigits);
            lenStr[numDigits] = '\0';
            int byteCount = atoi(lenStr);
            dataPtr += numDigits;
            
            debugPrint(TEK_DEBUG_INFO, "DEBUG: readWaveformBinary: binary header: #%d, byteCount=%d, width=%d\n", numDigits, byteCount, dataWidth);
            
            /* Each sample is dataWidth bytes */
            int expectedPoints = byteCount / dataWidth;
            numPoints = (expectedPoints < windowSize) ? expectedPoints : windowSize;
            if (numPoints > maxPoints_) numPoints = maxPoints_;
            
            /* Parse little-endian signed integers (SRIbinary = native x86) */
            unsigned char *binData = (unsigned char *)dataPtr;
            if (dataWidth == 2) {
                /* 16-bit signed little-endian: can memcpy directly on x86 */
                int16_t *srcData = (int16_t *)binData;
                for (i = 0; i < numPoints; i++) {
                    rawWaveform_[channel][i] = srcData[i];
                }
            } else {
                /* 8-bit signed */
                for (i = 0; i < numPoints; i++) {
                    int8_t value = (int8_t)binData[i];
                    rawWaveform_[channel][i] = value;
                }
            }
        } else {
            debugPrint(TEK_DEBUG_WARNING, "DEBUG: readWaveformBinary: invalid binary header numDigits=%d\n", numDigits);
        }
    } else {
        debugPrint(TEK_DEBUG_WARNING, "DEBUG: readWaveformBinary: expected # header, got 0x%02X\n", (unsigned char)*dataPtr);
    }
    
    waveformLength_[channel] = numPoints;
    debugPrint(TEK_DEBUG_INFO, "DEBUG: readWaveformBinary: parsed %d points\n", numPoints);

    /* Scale the waveform: voltage = (raw - YOFf) * YMUlt + YZEro */
    double xinc = xinc_[channel];
    double ymult = ymult_[channel];
    double yoff = yoff_[channel];
    double yzero = yzero_[channel];
    debugPrint(TEK_DEBUG_TRACE, "DEBUG: readWaveformBinary: scaling with xinc=%e, ymult=%e, yoff=%e, yzero=%e\n", xinc, ymult, yoff, yzero);

    /* Time array starts from dataStart offset */
    for (i = 0; i < numPoints; i++) {
        scaledWaveform_[channel][i] = (rawWaveform_[channel][i] - yoff) * ymult + yzero;
        timeArray_[channel][i] = (dataStart - 1 + i) * xinc;  /* Account for start offset */
    }

    epicsMutexUnlock(dataLock_);
    debugPrint(TEK_DEBUG_TRACE, "DEBUG: readWaveformBinary: dataLock_ released, numPoints=%d\n", numPoints);

    /* Update NrPt parameter and signal DataReady to trigger waveform reads */
    if (numPoints > 0) {
        static int dataReadyCount[TEK_MAX_CHANNELS] = {0};
        dataReadyCount[channel]++;
        
        /* Calculate refresh rate from timestamps */
        epicsTimeStamp now;
        epicsTimeGetCurrent(&now);
        double refreshRate = 0.0;
        
        if (!firstAcq_[channel]) {
            double elapsed = epicsTimeDiffInSeconds(&now, &lastAcqTime_[channel]);
            if (elapsed > 0.0) {
                refreshRate = 1.0 / elapsed;
            }
        }
        firstAcq_[channel] = 0;
        lastAcqTime_[channel] = now;
        
        lock();

        /* Compute statistics over marker window before signaling DataReady */
        computeStats(channel, numPoints);

        setIntegerParam(P_ChNrPt[channel], numPoints);
        setDoubleParam(P_ChRefreshRate[channel], refreshRate);
        
        /* Signal DataReady (I/O Intr supported on scalar Int32) to trigger waveform record processing */
        setIntegerParam(P_ChDataReady[channel], dataReadyCount[channel]);
        
        callParamCallbacks();
        unlock();
        
        debugPrint(TEK_DEBUG_INFO, "DEBUG: readWaveformBinary: DataReady[%d]=%d, %d points, rate=%.1f Hz\n", channel+1, dataReadyCount[channel], numPoints, refreshRate);
    }

    free(response);
    return asynSuccess;
}

/**
 * Compute statistics over the marker window for a given channel.
 * Must be called with dataLock_ NOT held (we lock internally).
 * Caller must hold the asyn port lock (lock()/unlock()) when calling.
 */
void drvTekMSO58LP::computeStats(int channel, int numPoints)
{
    double markerStart = 0.0, markerEnd = 0.0;
    getDoubleParam(P_ChMarkerStart[channel], &markerStart);
    getDoubleParam(P_ChMarkerEnd[channel], &markerEnd);

    double xinc = xinc_[channel];

    /* Convert marker times to sample indices */
    int startSample = 0;
    int endSample = numPoints;

    if (xinc > 0.0) {
        if (markerStart > 0.0) {
            startSample = (int)(markerStart / xinc + 0.5);
        }
        if (markerEnd > 0.0) {
            endSample = (int)(markerEnd / xinc + 0.5);
        }
    }

    /* Clamp to valid range */
    if (startSample < 0) startSample = 0;
    if (startSample > numPoints) startSample = numPoints;
    if (endSample <= 0 || endSample > numPoints) endSample = numPoints;
    if (endSample <= startSample) endSample = numPoints;

    /* Publish computed sample indices */
    setIntegerParam(P_ChMarkerStartSample[channel], startSample);
    setIntegerParam(P_ChMarkerEndSample[channel], endSample);

    int count = endSample - startSample;
    if (count <= 0) {
        setDoubleParam(P_ChStatsMean[channel], 0.0);
        setDoubleParam(P_ChStatsMin[channel], 0.0);
        setDoubleParam(P_ChStatsMax[channel], 0.0);
        setDoubleParam(P_ChStatsRMS[channel], 0.0);
        setDoubleParam(P_ChStatsIntegral[channel], 0.0);
        setDoubleParam(P_ChStatsPP[channel], 0.0);
        setDoubleParam(P_ChStatsStdDev[channel], 0.0);
        return;
    }

    /* Compute stats over [startSample, endSample) */
    epicsMutexLock(dataLock_);

    double sum = 0.0, sumSq = 0.0;
    double vmin = scaledWaveform_[channel][startSample];
    double vmax = vmin;

    for (int i = startSample; i < endSample; i++) {
        double v = scaledWaveform_[channel][i];
        sum += v;
        sumSq += v * v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }

    epicsMutexUnlock(dataLock_);

    double mean = sum / count;
    double rms = sqrt(sumSq / count);
    double integral = sum * xinc;
    double pp = vmax - vmin;
    double variance = (sumSq / count) - (mean * mean);
    double stddev = (variance > 0.0) ? sqrt(variance) : 0.0;

    setDoubleParam(P_ChStatsMean[channel], mean);
    setDoubleParam(P_ChStatsMin[channel], vmin);
    setDoubleParam(P_ChStatsMax[channel], vmax);
    setDoubleParam(P_ChStatsRMS[channel], rms);
    setDoubleParam(P_ChStatsIntegral[channel], integral);
    setDoubleParam(P_ChStatsPP[channel], pp);
    setDoubleParam(P_ChStatsStdDev[channel], stddev);

    debugPrint(TEK_DEBUG_INFO, "DEBUG: computeStats CH%d: samples=%d-%d mean=%.6e pp=%.6e rms=%.6e\n",
               channel + 1, startSample, endSample, mean, pp, rms);
}

/**
 * Read measurement value
 */
asynStatus drvTekMSO58LP::readMeasurement(int measurement)
{
    char cmd[64], response[128];
    asynStatus status;
    int meas = measurement + 1;
    double value;

    /* Get current measurement value */
    snprintf(cmd, sizeof(cmd), "MEASUrement:MEAS%d:RESUlts:CURRentacq:MEAN?", meas);
    status = query(cmd, response, sizeof(response));
    if (status == asynSuccess) {
        value = atof(response);
        lock();
        setDoubleParam(P_MeasValue[measurement], value);
        unlock();
    }

    /* Get measurement mean (all acquisitions) */
    snprintf(cmd, sizeof(cmd), "MEASUrement:MEAS%d:RESUlts:ALLAcqs:MEAN?", meas);
    status = query(cmd, response, sizeof(response));
    if (status == asynSuccess) {
        value = atof(response);
        lock();
        setDoubleParam(P_MeasMean[measurement], value);
        unlock();
    }

    return asynSuccess;
}

/**
 * Set record length
 */
asynStatus drvTekMSO58LP::setRecordLength(int points)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "HORizontal:RECOrdlength %d", points);
    return writeCmd(cmd);
}

/**
 * Set channel label
 */
asynStatus drvTekMSO58LP::setChannelLabel(int channel, const char *label)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "CH%d:LABEL:NAME \"%s\"", channel + 1, label);
    return writeCmd(cmd);
}

/**
 * Set measurement source channel
 */
asynStatus drvTekMSO58LP::setMeasurementSource(int meas, int channel)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "MEASUrement:MEAS%d:SOUrce CH%d", meas + 1, channel);
    return writeCmd(cmd);
}

/**
 * Set measurement type
 */
asynStatus drvTekMSO58LP::setMeasurementType(int meas, const char *type)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "MEASUrement:MEAS%d:TYPe %s", meas + 1, type);
    return writeCmd(cmd);
}

/**
 * Set measurement state (enable/disable)
 */
asynStatus drvTekMSO58LP::setMeasurementState(int meas, int state)
{
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "MEASUrement:MEAS%d:STATE %s", meas + 1, state ? "ON" : "OFF");
    return writeCmd(cmd);
}

/**
 * Write Int32 - handle control parameters
 */
asynStatus drvTekMSO58LP::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int i;

    /* Debug level parameter */
    if (function == P_DebugLevel) {
        if (value >= TEK_DEBUG_NONE && value <= TEK_DEBUG_TRACE) {
            debugLevel_ = value;
            debugPrint(TEK_DEBUG_INFO, "DEBUG: Debug level set to %d\n", debugLevel_);
        }
    }

    /* Global parameters */
    if (function == P_RecordLength) {
        status = setRecordLength(value);
    }
    else if (function == P_Acquire) {
        /* Start/stop acquisition polling */
        epicsEventSignal(pollerEvent_);
    }
    else if (function == P_RunStop) {
        /* Run/stop oscilloscope acquisition */
        if (value) {
            writeCmd("ACQuire:STATE RUN");
        } else {
            writeCmd("ACQuire:STATE STOP");
        }
    }

    /* Channel enable parameters */
    for (i = 0; i < numChannels_; i++) {
        if (function == P_ChEnable[i]) {
            /* Reset config read flag so config is re-read on next enable */
            if (value == 0) {
                configRead_[i] = 0;
                firstAcq_[i] = 1;
                setDoubleParam(P_ChRefreshRate[i], 0.0);
            }
            break;
        }
        if (function == P_ChDataWidth[i]) {
            /* Width changed - force config re-read to get updated YMUlt */
            debugPrint(TEK_DEBUG_INFO, "DEBUG: DataWidth changed for channel %d to %d\n", i+1, value);
            configRead_[i] = 0;
            break;
        }
        if (function == P_ChResetStats[i]) {
            /* Reset waveform data, stats, and refresh rate for this channel */
            debugPrint(TEK_DEBUG_INFO, "DEBUG: Resetting stats for channel %d\n", i+1);
            epicsMutexLock(dataLock_);
            memset(scaledWaveform_[i], 0, maxPoints_ * sizeof(epicsFloat64));
            memset(rawWaveform_[i], 0, maxPoints_ * sizeof(epicsInt32));
            memset(timeArray_[i], 0, maxPoints_ * sizeof(epicsFloat64));
            waveformLength_[i] = 0;
            epicsMutexUnlock(dataLock_);
            firstAcq_[i] = 1;
            setDoubleParam(P_ChRefreshRate[i], 0.0);
            /* Zero all stats directly */
            setDoubleParam(P_ChStatsMean[i], 0.0);
            setDoubleParam(P_ChStatsMin[i], 0.0);
            setDoubleParam(P_ChStatsMax[i], 0.0);
            setDoubleParam(P_ChStatsRMS[i], 0.0);
            setDoubleParam(P_ChStatsIntegral[i], 0.0);
            setDoubleParam(P_ChStatsPP[i], 0.0);
            setDoubleParam(P_ChStatsStdDev[i], 0.0);
            setIntegerParam(P_ChMarkerStartSample[i], 0);
            setIntegerParam(P_ChMarkerEndSample[i], 0);
            /* Trigger DataReady so FLNK chain processes the zeroed arrays */
            {
                static int resetCount[TEK_MAX_CHANNELS] = {0};
                resetCount[i]++;
                setIntegerParam(P_ChNrPt[i], 0);
                setIntegerParam(P_ChDataReady[i], resetCount[i]);
            }
            break;
        }
    }

    /* Measurement parameters */
    for (i = 0; i < numMeasurements_; i++) {
        if (function == P_MeasEnable[i]) {
            status = setMeasurementState(i, value);
            break;
        }
        if (function == P_MeasSource[i]) {
            status = setMeasurementSource(i, value);
            break;
        }
    }

    /* Set the parameter value */
    status = setIntegerParam(function, value);
    callParamCallbacks();

    return status;
}

/**
 * Write Float64 - handle control parameters
 */
asynStatus drvTekMSO58LP::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int i;

    if (function == P_PollTime) {
        if (value >= 0.001) {
            pollTime_ = value;
        }
    }

    /* Marker changes: store value and recompute stats immediately */
    for (i = 0; i < numChannels_; i++) {
        if (function == P_ChMarkerStart[i] || function == P_ChMarkerEnd[i]) {
            setDoubleParam(function, value);
            int nrpt = 0;
            getIntegerParam(P_ChNrPt[i], &nrpt);
            if (nrpt > 0) {
                computeStats(i, nrpt);
            }
            callParamCallbacks();
            return asynSuccess;
        }
    }

    status = setDoubleParam(function, value);
    callParamCallbacks();
    return status;
}

/**
 * Write Octet - handle string parameters
 */
asynStatus drvTekMSO58LP::writeOctet(asynUser *pasynUser, const char *value,
                                     size_t maxChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    int i;

    /* Channel label */
    for (i = 0; i < numChannels_; i++) {
        if (function == P_ChLabel[i]) {
            status = setChannelLabel(i, value);
            break;
        }
    }

    /* Measurement type */
    for (i = 0; i < numMeasurements_; i++) {
        if (function == P_MeasType[i]) {
            status = setMeasurementType(i, value);
            break;
        }
    }

    status = setStringParam(function, value);
    callParamCallbacks();
    *nActual = strlen(value);
    return status;
}

/**
 * Read Float64 array - return waveform data
 */
asynStatus drvTekMSO58LP::readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                           size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    int i;

    epicsMutexLock(dataLock_);

    for (i = 0; i < numChannels_; i++) {
        if (function == P_ChWaveformScaled[i]) {
            size_t n = std::min(nElements, (size_t)waveformLength_[i]);
            memcpy(value, scaledWaveform_[i], n * sizeof(epicsFloat64));
            *nIn = n;
            epicsMutexUnlock(dataLock_);
            return asynSuccess;
        }
        if (function == P_ChWaveformTime[i]) {
            size_t n = std::min(nElements, (size_t)waveformLength_[i]);
            memcpy(value, timeArray_[i], n * sizeof(epicsFloat64));
            *nIn = n;
            epicsMutexUnlock(dataLock_);
            return asynSuccess;
        }
    }

    epicsMutexUnlock(dataLock_);
    return asynError;
}

/**
 * Read Int32 array - return raw waveform data
 */
asynStatus drvTekMSO58LP::readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                         size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    int i;

    epicsMutexLock(dataLock_);

    for (i = 0; i < numChannels_; i++) {
        if (function == P_ChWaveform[i]) {
            size_t n = std::min(nElements, (size_t)waveformLength_[i]);
            memcpy(value, rawWaveform_[i], n * sizeof(epicsInt32));
            *nIn = n;
            epicsMutexUnlock(dataLock_);
            return asynSuccess;
        }
    }

    epicsMutexUnlock(dataLock_);
    return asynError;
}

/**
 * Report function for debugging
 */
void drvTekMSO58LP::report(FILE *fp, int details)
{
    fprintf(fp, "Tektronix MSO58LP driver: %s\n", portName);
    fprintf(fp, "  IP Address: %s:%d\n", ipAddress_, ipPort_);
    fprintf(fp, "  Connected: %d\n", connected_);
    fprintf(fp, "  Channels: %d, Measurements: %d\n", numChannels_, numMeasurements_);
    fprintf(fp, "  Max points: %d\n", maxPoints_);
    fprintf(fp, "  Poll time: %.3f s (%.1f Hz)\n", pollTime_, 1.0/pollTime_);
    fprintf(fp, "  Poll count: %lu\n", pollCount_);
    fprintf(fp, "  Error count: %lu\n", errorCount_);
    
    if (details > 0) {
        for (int i = 0; i < numChannels_; i++) {
            int enabled;
            getIntegerParam(P_ChEnable[i], &enabled);
            fprintf(fp, "  CH%d: enabled=%d, points=%d, xinc=%.3e, ymult=%.3e, yoff=%.1f, yzero=%.3e\n",
                    i+1, enabled, waveformLength_[i], xinc_[i], ymult_[i], yoff_[i], yzero_[i]);
        }
    }

    asynPortDriver::report(fp, details);
}

/* Configuration function - called from iocsh */
extern "C" int drvTekMSO58LPConfigure(const char *portName, const char *ipAddress,
                                       int ipPort, int numChannels, int numMeasurements,
                                       int maxPoints, double pollTime)
{
    new drvTekMSO58LP(portName, ipAddress, ipPort, numChannels, 
                      numMeasurements, maxPoints, pollTime);
    return 0;
}

/* iocsh registration */
static const iocshArg configArg0 = {"portName", iocshArgString};
static const iocshArg configArg1 = {"ipAddress", iocshArgString};
static const iocshArg configArg2 = {"ipPort", iocshArgInt};
static const iocshArg configArg3 = {"numChannels", iocshArgInt};
static const iocshArg configArg4 = {"numMeasurements", iocshArgInt};
static const iocshArg configArg5 = {"maxPoints", iocshArgInt};
static const iocshArg configArg6 = {"pollTime", iocshArgDouble};

static const iocshArg * const configArgs[] = {
    &configArg0, &configArg1, &configArg2, &configArg3,
    &configArg4, &configArg5, &configArg6
};

static const iocshFuncDef configFuncDef = {"drvTekMSO58LPConfigure", 7, configArgs};

static void configCallFunc(const iocshArgBuf *args)
{
    drvTekMSO58LPConfigure(args[0].sval, args[1].sval, args[2].ival,
                           args[3].ival, args[4].ival, args[5].ival,
                           args[6].dval);
}

static void drvTekMSO58LPRegister(void)
{
    iocshRegister(&configFuncDef, configCallFunc);
}

extern "C" {
    epicsExportRegistrar(drvTekMSO58LPRegister);
}
