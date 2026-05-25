/*
 * findMarkerEdge.c
 *
 * Two aSub functions to snap the stats markers onto the first sample
 * crossing AutoThreshold:
 *
 *   findMarkerStart - scan from the LEFT  (index 0 .. n-1)
 *   findMarkerEnd   - scan from the RIGHT (index n-1 .. 0)
 *
 * Inputs:
 *   A (DOUBLE)        : threshold (absolute value)
 *   C (DOUBLE array)  : waveform (NEC = NELM)
 *   D (LONG)          : nsamples (NrPt, valid sample count)
 *
 * Outputs:
 *   VALA (LONG) : window-relative sample index (0-based)
 *                 - findMarkerStart: falls back to 0      if no crossing
 *                 - findMarkerEnd  : falls back to n-1    if no crossing
 *   VALB (LONG) : 1 if a crossing was found, 0 otherwise (optional)
 */
#include <stdio.h>
#include <math.h>
#include <epicsTypes.h>
#include <registryFunction.h>
#include <aSubRecord.h>
#include <epicsExport.h>

static long scan_edge(aSubRecord *prec, int fromRight)
{
    double threshold = *(double *)prec->a;
    double *wf       = (double *)prec->c;
    epicsInt32 n     = *(epicsInt32 *)prec->d;
    epicsInt32 *out  = (epicsInt32 *)prec->vala;
    epicsInt32 *found= (prec->novb > 0 && prec->valb) ? (epicsInt32 *)prec->valb : NULL;

    if (out)   *out = 0;
    if (found) *found = 0;

    if (n <= 0 || (long)n > (long)prec->nec) {
        return 0;
    }

    if (!fromRight) {
        for (epicsInt32 i = 0; i < n; i++) {
            if (fabs(wf[i]) > threshold) {
                if (out)   *out = i;
                if (found) *found = 1;
                return 0;
            }
        }
        /* nothing found: leave at 0 */
    } else {
        for (epicsInt32 i = n - 1; i >= 0; i--) {
            if (fabs(wf[i]) > threshold) {
                if (out)   *out = i;
                if (found) *found = 1;
                return 0;
            }
        }
        if (out) *out = n - 1;
    }
    return 0;
}

static long findMarkerStart(aSubRecord *prec) { return scan_edge(prec, 0); }
static long findMarkerEnd  (aSubRecord *prec) { return scan_edge(prec, 1); }

epicsRegisterFunction(findMarkerStart);
epicsRegisterFunction(findMarkerEnd);
