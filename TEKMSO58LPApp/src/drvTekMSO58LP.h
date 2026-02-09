/**
 * @file drvTekMSO58LP.h
 * @brief ASYN Port Driver for Tektronix MSO58LP Oscilloscope
 * 
 * High-performance ASYN driver for >10Hz acquisition rates.
 * Uses dedicated polling thread for fast data acquisition.
 * 
 * @author EPICS Team
 * @date 2026
 */

#ifndef DRV_TEK_MSO58LP_H
#define DRV_TEK_MSO58LP_H

#include <asynPortDriver.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsEvent.h>

#ifdef __cplusplus

/* Maximum supported values */
#define TEK_MAX_CHANNELS        8
#define TEK_MAX_MEASUREMENTS    10
#define TEK_MAX_WAVEFORM_PTS    250000
#define TEK_MAX_STRING_SIZE     256
#define TEK_DEFAULT_TIMEOUT     1.0

/* Parameter indices - will be created dynamically */
/* Global device parameters */
#define TEK_IDN_STRING              "TEK_IDN"
#define TEK_VENDOR_STRING           "TEK_VENDOR"
#define TEK_MODEL_STRING            "TEK_MODEL"
#define TEK_SERIAL_STRING           "TEK_SERIAL"
#define TEK_FIRMWARE_STRING         "TEK_FIRMWARE"
#define TEK_CONNECTED_STRING        "TEK_CONNECTED"
#define TEK_POLL_TIME_STRING        "TEK_POLL_TIME"
#define TEK_RECORD_LENGTH_STRING    "TEK_RECORD_LENGTH"
#define TEK_ACQUIRE_STRING          "TEK_ACQUIRE"
#define TEK_RUN_STOP_STRING         "TEK_RUN_STOP"

/* Per-channel parameters (suffix with channel number) */
#define TEK_CH_ENABLE_STRING        "TEK_CH_ENABLE"
#define TEK_CH_LABEL_STRING         "TEK_CH_LABEL"
#define TEK_CH_XINC_STRING          "TEK_CH_XINC"
#define TEK_CH_YMULT_STRING         "TEK_CH_YMULT"
#define TEK_CH_YOFF_STRING          "TEK_CH_YOFF"
#define TEK_CH_YZERO_STRING         "TEK_CH_YZERO"
#define TEK_CH_XUNIT_STRING         "TEK_CH_XUNIT"
#define TEK_CH_YUNIT_STRING         "TEK_CH_YUNIT"
#define TEK_CH_NR_PT_STRING         "TEK_CH_NR_PT"
#define TEK_CH_WAVEFORM_STRING      "TEK_CH_WAVEFORM"
#define TEK_CH_WAVEFORM_SCALED_STRING "TEK_CH_WAVEFORM_SCALED"
#define TEK_CH_WAVEFORM_TIME_STRING "TEK_CH_WAVEFORM_TIME"
#define TEK_CH_COUPLING_STRING      "TEK_CH_COUPLING"
#define TEK_CH_SCALE_STRING         "TEK_CH_SCALE"
#define TEK_CH_OFFSET_STRING        "TEK_CH_OFFSET"
#define TEK_CH_POSITION_STRING      "TEK_CH_POSITION"
#define TEK_CH_BANDWIDTH_STRING     "TEK_CH_BANDWIDTH"
#define TEK_CH_DATA_READY_STRING    "TEK_CH_DATA_READY"
#define TEK_CH_DATA_START_STRING    "TEK_CH_DATA_START"
#define TEK_CH_DATA_STOP_STRING     "TEK_CH_DATA_STOP"
#define TEK_CH_REFRESH_RATE_STRING  "TEK_CH_REFRESH_RATE"
#define TEK_CH_RESET_STATS_STRING   "TEK_CH_RESET_STATS"
#define TEK_CH_DATA_WIDTH_STRING    "TEK_CH_DATA_WIDTH"
#define TEK_CH_MARKER_START_STRING  "TEK_CH_MARKER_START"
#define TEK_CH_MARKER_END_STRING    "TEK_CH_MARKER_END"
#define TEK_CH_MARKER_START_SAMPLE_STRING "TEK_CH_MARKER_START_SAMPLE"
#define TEK_CH_MARKER_END_SAMPLE_STRING   "TEK_CH_MARKER_END_SAMPLE"
#define TEK_CH_STATS_MEAN_STRING    "TEK_CH_STATS_MEAN"
#define TEK_CH_STATS_MIN_STRING     "TEK_CH_STATS_MIN"
#define TEK_CH_STATS_MAX_STRING     "TEK_CH_STATS_MAX"
#define TEK_CH_STATS_RMS_STRING     "TEK_CH_STATS_RMS"
#define TEK_CH_STATS_INTEGRAL_STRING "TEK_CH_STATS_INTEGRAL"
#define TEK_CH_STATS_PP_STRING      "TEK_CH_STATS_PP"
#define TEK_CH_STATS_STDDEV_STRING  "TEK_CH_STATS_STDDEV"

/* Per-measurement parameters */
#define TEK_MEAS_ENABLE_STRING      "TEK_MEAS_ENABLE"
#define TEK_MEAS_SOURCE_STRING      "TEK_MEAS_SOURCE"
#define TEK_MEAS_TYPE_STRING        "TEK_MEAS_TYPE"
#define TEK_MEAS_VALUE_STRING       "TEK_MEAS_VALUE"
#define TEK_MEAS_MEAN_STRING        "TEK_MEAS_MEAN"
#define TEK_MEAS_MIN_STRING         "TEK_MEAS_MIN"
#define TEK_MEAS_MAX_STRING         "TEK_MEAS_MAX"
#define TEK_MEAS_STDDEV_STRING      "TEK_MEAS_STDDEV"
#define TEK_MEAS_UNITS_STRING       "TEK_MEAS_UNITS"

/* Statistics/diagnostic parameters */
#define TEK_POLL_COUNT_STRING       "TEK_POLL_COUNT"
#define TEK_ERROR_COUNT_STRING      "TEK_ERROR_COUNT"
#define TEK_LAST_ERROR_STRING       "TEK_LAST_ERROR"
#define TEK_COMM_TIME_STRING        "TEK_COMM_TIME"
#define TEK_DEBUG_LEVEL_STRING      "TEK_DEBUG_LEVEL"

/* Debug levels */
#define TEK_DEBUG_NONE      0   /* No debug output */
#define TEK_DEBUG_ERROR     1   /* Errors only */
#define TEK_DEBUG_WARNING   2   /* Warnings and errors */
#define TEK_DEBUG_INFO      3   /* Info, warnings, errors */
#define TEK_DEBUG_FLOW      4   /* Detailed flow tracing */
#define TEK_DEBUG_TRACE     5   /* Everything including data */

/**
 * @brief ASYN Port Driver for Tektronix MSO58LP
 * 
 * This driver provides high-speed acquisition from Tektronix MSO58LP
 * oscilloscopes using direct socket communication and a dedicated
 * polling thread.
 */
class drvTekMSO58LP : public asynPortDriver {
public:
    /**
     * @brief Constructor
     * @param portName The name of this ASYN port
     * @param ipAddress IP address of the oscilloscope
     * @param ipPort TCP port number (typically 4000 for raw socket)
     * @param numChannels Number of channels to support (1-8)
     * @param numMeasurements Number of measurements to support (1-10)
     * @param maxPoints Maximum waveform points per channel
     * @param pollTime Polling interval in seconds (e.g., 0.01 for 100Hz)
     */
    drvTekMSO58LP(const char *portName, const char *ipAddress, int ipPort,
                  int numChannels, int numMeasurements, int maxPoints, 
                  double pollTime);
    
    virtual ~drvTekMSO58LP();
    
    /* Override asynPortDriver methods */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value,
                                  size_t maxChars, size_t *nActual);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                        size_t nElements, size_t *nIn);
    virtual asynStatus readInt32Array(asynUser *pasynUser, epicsInt32 *value,
                                      size_t nElements, size_t *nIn);
    
    /* These are called by the polling thread */
    void pollerThread();
    
    /* Report function for debugging */
    virtual void report(FILE *fp, int details);

protected:
    /* Global device parameters */
    int P_Idn;
    int P_Vendor;
    int P_Model;
    int P_Serial;
    int P_Firmware;
    int P_Connected;
    int P_PollTime;
    int P_RecordLength;
    int P_Acquire;
    int P_RunStop;
    
    /* Per-channel parameters - arrays indexed by channel */
    int P_ChEnable[TEK_MAX_CHANNELS];
    int P_ChLabel[TEK_MAX_CHANNELS];
    int P_ChXinc[TEK_MAX_CHANNELS];
    int P_ChYmult[TEK_MAX_CHANNELS];
    int P_ChYoff[TEK_MAX_CHANNELS];
    int P_ChYzero[TEK_MAX_CHANNELS];
    int P_ChXunit[TEK_MAX_CHANNELS];
    int P_ChYunit[TEK_MAX_CHANNELS];
    int P_ChNrPt[TEK_MAX_CHANNELS];
    int P_ChWaveform[TEK_MAX_CHANNELS];
    int P_ChWaveformScaled[TEK_MAX_CHANNELS];
    int P_ChWaveformTime[TEK_MAX_CHANNELS];
    int P_ChCoupling[TEK_MAX_CHANNELS];
    int P_ChScale[TEK_MAX_CHANNELS];
    int P_ChOffset[TEK_MAX_CHANNELS];
    int P_ChPosition[TEK_MAX_CHANNELS];
    int P_ChBandwidth[TEK_MAX_CHANNELS];
    int P_ChDataReady[TEK_MAX_CHANNELS];
    int P_ChDataStart[TEK_MAX_CHANNELS];
    int P_ChDataStop[TEK_MAX_CHANNELS];
    int P_ChRefreshRate[TEK_MAX_CHANNELS];
    int P_ChResetStats[TEK_MAX_CHANNELS];
    int P_ChDataWidth[TEK_MAX_CHANNELS];
    int P_ChMarkerStart[TEK_MAX_CHANNELS];
    int P_ChMarkerEnd[TEK_MAX_CHANNELS];
    int P_ChMarkerStartSample[TEK_MAX_CHANNELS];
    int P_ChMarkerEndSample[TEK_MAX_CHANNELS];
    int P_ChStatsMean[TEK_MAX_CHANNELS];
    int P_ChStatsMin[TEK_MAX_CHANNELS];
    int P_ChStatsMax[TEK_MAX_CHANNELS];
    int P_ChStatsRMS[TEK_MAX_CHANNELS];
    int P_ChStatsIntegral[TEK_MAX_CHANNELS];
    int P_ChStatsPP[TEK_MAX_CHANNELS];
    int P_ChStatsStdDev[TEK_MAX_CHANNELS];
    
    /* Per-measurement parameters */
    int P_MeasEnable[TEK_MAX_MEASUREMENTS];
    int P_MeasSource[TEK_MAX_MEASUREMENTS];
    int P_MeasType[TEK_MAX_MEASUREMENTS];
    int P_MeasValue[TEK_MAX_MEASUREMENTS];
    int P_MeasMean[TEK_MAX_MEASUREMENTS];
    int P_MeasMin[TEK_MAX_MEASUREMENTS];
    int P_MeasMax[TEK_MAX_MEASUREMENTS];
    int P_MeasStddev[TEK_MAX_MEASUREMENTS];
    int P_MeasUnits[TEK_MAX_MEASUREMENTS];
    
    /* Diagnostic parameters */
    int P_PollCount;
    int P_ErrorCount;
    int P_LastError;
    int P_CommTime;
    int P_DebugLevel;

private:
    /* Debug output helper */
    void debugPrint(int level, const char *fmt, ...);
    
    /* Communication methods */
    asynStatus connectToScope();
    asynStatus disconnectFromScope();
    asynStatus writeRead(const char *writeBuffer, char *readBuffer, 
                         size_t readBufferSize, size_t *nRead, double timeout);
    asynStatus writeCmd(const char *cmd);
    asynStatus query(const char *cmd, char *response, size_t maxLen);
    
    /* Acquisition methods */
    asynStatus readIdentification();
    asynStatus readChannelConfig(int channel);
    asynStatus readWaveform(int channel);
    asynStatus readWaveformBinary(int channel);
    asynStatus readMeasurement(int measurement);
    asynStatus setRecordLength(int points);
    asynStatus setChannelLabel(int channel, const char *label);
    asynStatus setMeasurementSource(int meas, int channel);
    asynStatus setMeasurementType(int meas, const char *type);
    asynStatus setMeasurementState(int meas, int state);
    
    /* Helper methods */
    void parseWaveformData(const char *data, int channel);
    void scaleWaveform(int channel);
    void generateTimeArray(int channel);
    void computeStats(int channel, int numPoints);
    
    /* Configuration */
    char ipAddress_[TEK_MAX_STRING_SIZE];
    int ipPort_;
    int numChannels_;
    int numMeasurements_;
    int maxPoints_;
    double pollTime_;
    
    /* Communication state */
    asynUser *pasynUserOctet_;
    asynOctet *pasynOctet_;
    void *octetPvt_;
    int connected_;
    
    /* Polling thread */
    epicsThreadId pollerThreadId_;
    epicsEventId pollerEvent_;
    int pollerRunning_;
    int pollerExitRequest_;
    
    /* Waveform buffers - pre-allocated for speed */
    epicsInt32 *rawWaveform_[TEK_MAX_CHANNELS];
    epicsFloat64 *scaledWaveform_[TEK_MAX_CHANNELS];
    epicsFloat64 *timeArray_[TEK_MAX_CHANNELS];
    int waveformLength_[TEK_MAX_CHANNELS];
    
    /* Channel configuration cache */
    double xinc_[TEK_MAX_CHANNELS];
    double ymult_[TEK_MAX_CHANNELS];
    double yoff_[TEK_MAX_CHANNELS];
    double yzero_[TEK_MAX_CHANNELS];
    
    /* Track last-sent DATa settings to avoid redundant SCPI commands */
    int lastDataSource_;   /* last channel set via DATa:SOUrce (-1 = none) */
    int lastDataWidth_[TEK_MAX_CHANNELS];  /* last DATa:WIDth sent per channel */
    
    /* Per-channel refresh rate tracking */
    epicsTimeStamp lastAcqTime_[TEK_MAX_CHANNELS];
    int firstAcq_[TEK_MAX_CHANNELS];  /* 1 = first acquisition pending */
    int configRead_[TEK_MAX_CHANNELS]; /* 0 = config not yet read */
    
    /* Statistics */
    unsigned long pollCount_;
    unsigned long errorCount_;
    int debugLevel_;
    
    /* Mutex for thread safety */
    epicsMutexId dataLock_;
};

/* Configuration function for iocsh */
extern "C" {
    int drvTekMSO58LPConfigure(const char *portName, const char *ipAddress, 
                               int ipPort, int numChannels, int numMeasurements,
                               int maxPoints, double pollTime);
}

#endif /* __cplusplus */

#endif /* DRV_TEK_MSO58LP_H */
