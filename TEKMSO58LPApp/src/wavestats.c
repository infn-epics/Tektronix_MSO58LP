/**
 * wavestats.c - Waveform statistics aSub function
 * 
 * Calculates statistics for oscilloscope waveform data:
 *   - Integral (area under curve)
 *   - Mean (average value)
 *   - Min/Max
 *   - RMS (root mean square)
 * 
 * Inputs:
 *   A - xinc (time increment between samples)
 *   B - norm (normalization coefficient)
 *   C - waveform array (double)
 *   D - number of points
 *   E - start sample index (for windowed calculation)
 *   F - end sample index (for windowed calculation)
 * 
 * Outputs:
 *   VALA - Integral
 *   VALB - Mean
 *   VALC - Min
 *   VALD - Max
 *   VALE - RMS
 */
#include <registryFunction.h>
#include <epicsExport.h>
#include "aSubRecord.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/**
 * incrementor - Simple incrementor function for testing
 */
static long incrementor(aSubRecord *prec) {
    long i;
    double *a;
    
    prec->pact = 1;
    
    a = (double *)prec->a;
    
    for(i=0; i < prec->nova; ++i) {
        ((double *)prec->vala)[i] = (double)i * (*a);
    }
    
    prec->pact = 0;
    
    return 0;
}

/**
 * wavestats - Calculate waveform statistics
 */
static long wavestats(aSubRecord *prec) {
    double xinc = *((double *)prec->a);
    double norm = *((double *)prec->b);
    double *y = (double *)prec->c;
    
    long n = *(long*)prec->d;
    long start = *(long*)prec->e;
    long end = *(long*)prec->f;
    
    /* Get actual number of elements in input C */
    long nec = prec->nec;  /* Max elements for input C */
    long noc = prec->noc;  /* Actual elements in input C array */

    double sum = 0.0;
    double sumsq = 0.0;
    double max, min;
    long count;
    
    /* Debug: print input info */
    static int debugCount = 0;
    if (debugCount < 10 || debugCount % 100 == 0) {
        printf("wavestats[%s]: n=%ld, start=%ld, end=%ld, nec=%ld, noc=%ld, y=%p\n", 
               prec->name, n, start, end, nec, noc, (void*)y);
        if (y && nec > 0) {
            printf("wavestats[%s]: y[0]=%f, y[1]=%f, y[2]=%f (nec=%ld elements available)\n", 
                   prec->name, y[0], y[1], y[2], nec);
        }
    }
    debugCount++;
    
    /* Initialize outputs to invalid */
    *((double *)prec->vala) = 0.0;  /* Integral */
    *((double *)prec->valb) = 0.0;  /* Mean */
    *((double *)prec->valc) = 0.0;  /* Min */
    *((double *)prec->vald) = 0.0;  /* Max */
    *((double *)prec->vale) = 0.0;  /* RMS */
    
    /* Validate inputs */
    if (n <= 0 || y == NULL) {
        return 0;
    }
    
    /* Clamp start/end to valid range */
    if (start < 0) start = 0;
    if (start >= n) start = 0;
    if (end <= start) end = n;
    if (end > n) end = n;
    
    count = end - start;
    if (count <= 0) {
        return 0;
    }
    
    /* Initialize min/max from first sample in window */
    max = min = y[start];
    
    /* Calculate statistics over the window */
    for (long i = start; i < end; ++i) {
        double val = y[i];
        
        if (val > max) max = val;
        if (val < min) min = val;
        
        sum += val;
        sumsq += val * val;
    }
    
    /* Calculate results */
    double integral = sum * xinc * norm;
    double mean = sum / count;
    double rms = sqrt(sumsq / count);
    
    /* Output results */
    *((double *)prec->vala) = integral;
    *((double *)prec->valb) = mean;
    *((double *)prec->valc) = min;
    *((double *)prec->vald) = max;
    *((double *)prec->vale) = rms;
    
    return 0;
}

epicsRegisterFunction(incrementor);
epicsRegisterFunction(wavestats);
