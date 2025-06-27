#include <registryFunction.h>
#include <epicsExport.h>
#include "aSubRecord.h"
#include "stdlib.h"
   
static long incrementor(aSubRecord *prec) {
    long i;
    double *a;
    
    prec->pact = 1;
    
    a = (double *)prec->a;
    
    for(i=0; i < prec->nova; ++i)
    {
        //printf("index= %d", i);
        ((double *)prec->vala)[i] = (double)i * (*a);
    }
    
    prec->pact = 0;

    //Debug message - prints to IOC
    //printf("my_asub_routine called");
    
    return 0;
}

static long integrator(aSubRecord *prec) {
    double *y = (double *)prec->a;
    double xinc = *((double *)prec->b);
    double xmin = *((double *)prec->c);
    double xmax = *((double *)prec->d);
    double norm = *((double *)prec->e);
    int n = prec->nea;

    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        double x = i * xinc;
        if (x >= xmin && x <= xmax)
            sum += y[i];
    }

    *((double *)prec->vala) = (sum * xinc) / norm;
    return 0;
}

epicsRegisterFunction(incrementor);
epicsRegisterFunction(integrator);
