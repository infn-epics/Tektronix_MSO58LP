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
epicsRegisterFunction(incrementor);

