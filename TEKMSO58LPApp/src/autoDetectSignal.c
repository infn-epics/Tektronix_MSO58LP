#include <stdio.h>
#include <string.h>
#include <math.h>
#include <registryFunction.h>
#include <aSubRecord.h>
#include <epicsExport.h>

/**
 * Auto-detect signal edges in waveform data
 * 
 * Inputs:
 *   A (DOUBLE): Threshold value
 *   B (DOUBLE): Time increment (XINC)
 *   C (DOUBLE array): Waveform data
 *   D (LONG): Number of valid samples
 *   E (LONG): Current DataStart (1-based absolute sample index in scope record)
 * 
 * Outputs:
 *   VALA (DOUBLE): Start time of detected signal (relative to DataStart)
 *   VALB (DOUBLE): End time of detected signal (relative to DataStart)
 *   VALC (LONG):   Found flag (1 if signal detected, 0 otherwise)
 *   VALD (LONG):   Suggested new DataStart (1-based, = DataStart + start_idx - margin)
 *   VALE (LONG):   Suggested new WindowSize (end_idx - start_idx + 2*margin)
 */
#define AUTO_DETECT_MARGIN 100  /* extra samples of margin around detected signal */

static long autoDetectSignal(aSubRecord *prec)
{
    double threshold = *(double *)prec->a;
    double xinc = *(double *)prec->b;
    double *waveform = (double *)prec->c;
    long nsamples = *(long *)prec->d;
    long dataStart = *(long *)prec->e;  /* 1-based absolute sample index */
    
    double *start_time  = (double *)prec->vala;
    double *end_time    = (double *)prec->valb;
    long *found         = (long *)prec->valc;
    long *new_datastart = (long *)prec->vald;
    long *new_winsize   = (long *)prec->vale;
    
    long i;
    long start_idx = -1;
    long end_idx = -1;
    int in_signal = 0;
    
    /* Initialize outputs */
    *start_time  = 0.0;
    *end_time    = 0.0;
    *found       = 0;
    *new_datastart = dataStart;   /* default: keep current DataStart */
    *new_winsize   = nsamples;    /* default: keep current window size */
    
    /* Validate inputs */
    if (nsamples <= 0 || nsamples > (long)prec->nec) {
        printf("autoDetectSignal: Invalid sample count %ld (max %ld)\n", nsamples, (long)prec->nec);
        return 0;
    }
    
    if (xinc <= 0.0) {
        printf("autoDetectSignal: Invalid XINC %f\n", xinc);
        return 0;
    }
    
    /* Find signal edges using threshold crossing detection */
    for (i = 0; i < nsamples; i++) {
        double val = fabs(waveform[i]);  /* Use absolute value to catch negative signals */
        
        if (!in_signal && val > threshold) {
            /* Rising edge detected */
            start_idx = i;
            in_signal = 1;
            
            /* Look back a few samples for better edge detection */
            if (i > 5) {
                for (long j = i - 1; j >= i - 5 && j >= 0; j--) {
                    if (fabs(waveform[j]) < threshold * 0.5) {
                        start_idx = j + 1;
                        break;
                    }
                }
            }
        }
        else if (in_signal && val < threshold * 0.5) {
            /* Falling edge detected (use hysteresis) */
            end_idx = i;
            
            /* Look forward a few samples to confirm end */
            int confirmed = 1;
            for (long j = i + 1; j < i + 10 && j < nsamples; j++) {
                if (fabs(waveform[j]) > threshold) {
                    confirmed = 0;
                    break;
                }
            }
            
            if (confirmed) {
                in_signal = 0;
                break;  /* Found complete pulse */
            }
        }
    }
    
    /* If signal started but didn't end, use end of buffer */
    if (start_idx >= 0 && end_idx < 0) {
        end_idx = nsamples - 1;
    }
    
    /* Calculate times and set outputs */
    if (start_idx >= 0 && end_idx >= 0) {
        *start_time = start_idx * xinc;
        *end_time   = end_idx * xinc;
        *found = 1;

        /* Suggest a new DataStart and WindowSize that frame the detected signal
         * with AUTO_DETECT_MARGIN samples of margin on each side.
         * DataStart is 1-based in the scope record: offset start_idx within the
         * current window maps to absolute sample (dataStart + start_idx). */
        long abs_start = dataStart + start_idx;
        long abs_end   = dataStart + end_idx;
        long new_start = abs_start - AUTO_DETECT_MARGIN;
        if (new_start < 1) new_start = 1;
        long win = (abs_end - new_start + 1) + AUTO_DETECT_MARGIN;
        if (win < 1) win = 1;
        *new_datastart = new_start;
        *new_winsize   = win;

        printf("autoDetectSignal: Signal found from sample %ld to %ld (%.6e s to %.6e s)\n",
               start_idx, end_idx, *start_time, *end_time);
        printf("autoDetectSignal: Suggested DataStart=%ld, WindowSize=%ld\n",
               new_start, win);
    } else {
        printf("autoDetectSignal: No signal found above threshold %.6e V\n", threshold);
    }
    
    return 0;  /* Success */
}

/* Register the function */
epicsRegisterFunction(autoDetectSignal);
