/* tekStoreWaveform.c
 *
 * Per-channel waveform file storage for the Tektronix MSO58LP IOC.
 *
 * Implemented as an EPICS aSub function.  One aSub record per channel
 * keeps a small state structure (FILE *, counter, NumAcq, filename) in
 * prec->dpvt across processings.  A control word (INPA) selects the
 * phase:
 *     1  INIT   open file, write metadata header, reset counter
 *     2  APPEND append one CSV row of the current waveform
 *     3  CLOSE  flush and close the file
 *
 * All other inputs supply waveform metadata and the sample array.  The
 * aSub outputs status, current filename and acquisition counter so the
 * database can react (e.g. clear the per-channel Active flag when
 * counter reaches NumAcq).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <aSubRecord.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsTime.h>
#include <errlog.h>
#include <registryFunction.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <alarm.h>

typedef struct {
    FILE   *fp;
    long    counter;
    long    numAcq;
    char    filename[1024];
} tekStoreState;

/* Copy a CHAR waveform field into a NUL-terminated C string buffer. */
static void copyCharArray(char *dst, size_t dstsz, const char *src, size_t srcsz)
{
    size_t n = srcsz < dstsz - 1 ? srcsz : dstsz - 1;
    size_t i = 0;
    if (!src) { dst[0] = '\0'; return; }
    for (; i < n && src[i]; i++) dst[i] = src[i];
    dst[i] = '\0';
    /* Trim trailing whitespace */
    while (i > 0 && (dst[i-1] == ' ' || dst[i-1] == '\t' ||
                     dst[i-1] == '\r' || dst[i-1] == '\n')) {
        dst[--i] = '\0';
    }
}

static long tekStoreWaveform_init(aSubRecord *prec)
{
    prec->dpvt = calloc(1, sizeof(tekStoreState));
    if (!prec->dpvt) {
        errlogPrintf("tekStoreWaveform_init: out of memory for %s\n", prec->name);
        return -1;
    }
    return 0;
}

static long tekStoreWaveform(aSubRecord *prec)
{
    tekStoreState *st = (tekStoreState *) prec->dpvt;
    if (!st) return -1;

    long   ctrl   = *(long   *) prec->a;
    const char   *pathArr = (const char   *) prec->b;
    const char   *baseArr = (const char   *) prec->c;
    const char   *nameArr = (const char   *) prec->d;
    long   numAcq = *(long   *) prec->e;
    double xinc   = *(double *) prec->f;
    double ymult  = *(double *) prec->g;
    double yzero  = *(double *) prec->h;
    double yoff   = *(double *) prec->i;
    double *wf    =  (double *) prec->j;
    long   npt    = *(long   *) prec->k;
    long   dstart = *(long   *) prec->l;

    long  *status = (long *) prec->vala;
    char  *fnOut  = (char *) prec->valb;
    long  *cnt    = (long *) prec->valc;

    char path[512], base[128], chname[128];
    copyCharArray(path,   sizeof(path),   pathArr, prec->nob);
    copyCharArray(base,   sizeof(base),   baseArr, prec->noc);
    /* INPD is a STRING field (40-byte buffer), NOD == 1. */
    copyCharArray(chname, sizeof(chname), nameArr, MAX_STRING_SIZE);

    *status = 0;

    if (ctrl == 1) { /* INIT */
        if (st->fp) { fclose(st->fp); st->fp = NULL; }
        st->counter = 0;
        st->numAcq  = numAcq > 0 ? numAcq : 1;

        if (path[0] == '\0') strcpy(path, ".");
        if (base[0] == '\0') strcpy(base, "tek");
        if (chname[0] == '\0') snprintf(chname, sizeof(chname), "CH");

        /* Build timestamp YYYYMMDDTHHMMSS (local time). */
        time_t now = time(NULL);
        struct tm tm;
        localtime_r(&now, &tm);
        char ts[32];
        strftime(ts, sizeof(ts), "%Y%m%dT%H%M%S", &tm);

        /* Ensure directory exists (best-effort, single level). */
        struct stat sb;
        if (stat(path, &sb) != 0) {
            if (mkdir(path, 0775) != 0 && errno != EEXIST) {
                errlogPrintf("tekStoreWaveform[%s]: cannot create dir %s: %s\n",
                             prec->name, path, strerror(errno));
            }
        }

        snprintf(st->filename, sizeof(st->filename),
                 "%s/%s-%s-%s.dat", path, base, chname, ts);

        st->fp = fopen(st->filename, "w");
        if (!st->fp) {
            errlogPrintf("tekStoreWaveform[%s]: cannot open %s: %s\n",
                         prec->name, st->filename, strerror(errno));
            strncpy(fnOut, "OPEN-FAILED", MAX_STRING_SIZE - 1);
            fnOut[MAX_STRING_SIZE - 1] = '\0';
            *cnt = 0;
            return 0;
        }

        double first_sample_time = (dstart > 0 ? (dstart - 1) : 0) * xinc;

        fprintf(st->fp,
                "# channel=%s xinc=%.12g ymult=%.12g yzero=%.12g "
                "yoff=%.12g npts=%ld data_start=%ld first_sample_time_s=%.12g "
                "num_acq=%ld timestamp=%s\n",
                chname, xinc, ymult, yzero, yoff, npt, dstart,
                first_sample_time, st->numAcq, ts);
        fflush(st->fp);

        /* Filename truncated to MAX_STRING_SIZE for the stringout PV. */
        strncpy(fnOut, st->filename, MAX_STRING_SIZE - 1);
        fnOut[MAX_STRING_SIZE - 1] = '\0';
        *cnt = 0;
        *status = 1;
        return 0;
    }
    else if (ctrl == 2) { /* APPEND */
        if (!st->fp) {
            *status = 0;
            return 0;
        }
        long n = npt;
        if (n > (long) prec->noj) n = (long) prec->noj;
        if (n <= 0) {
            *status = 1;
            *cnt = st->counter;
            return 0;
        }
        for (long i = 0; i < n; i++) {
            if (i) fputc(',', st->fp);
            fprintf(st->fp, "%.9g", wf[i]);
        }
        fputc('\n', st->fp);
        fflush(st->fp);
        st->counter++;
        *cnt = st->counter;

        if (st->counter >= st->numAcq) {
            fflush(st->fp);
            fclose(st->fp);
            st->fp = NULL;
            *status = 2;        /* done */
        } else {
            *status = 1;        /* still active */
        }
        return 0;
    }
    else if (ctrl == 3) { /* CLOSE */
        if (st->fp) {
            fflush(st->fp);
            fclose(st->fp);
            st->fp = NULL;
        }
        *cnt = st->counter;
        *status = 3;            /* closed by request */
        return 0;
    }

    /* ctrl == 0 or unknown: report current state without action */
    *cnt = st->counter;
    *status = st->fp ? 1 : 0;
    return 0;
}

epicsRegisterFunction(tekStoreWaveform_init);
epicsRegisterFunction(tekStoreWaveform);
