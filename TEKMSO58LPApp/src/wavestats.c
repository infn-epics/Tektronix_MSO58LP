#include <registryFunction.h>
#include <epicsExport.h>
#include "aSubRecord.h"
#include "stdlib.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
   
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

static long wavestats(aSubRecord *prec) {
    double xinc = *((double *)prec->a);
    double norm = *((double *)prec->b);
    double *y = (double *)prec->c;
    
    long n = *(long*)prec->d;
    long start = *(long*)prec->e;
    long end = *(long*)prec->f;


    double sum = 0.0, max = 0, min = 0, sumsq = 0.0;
    *((double *)prec->vala) = -1;
    *((double *)prec->valb) = -1;
    *((double *)prec->valc) = -1;
    *((double *)prec->vald) = -1;
    *((double *)prec->vale) = -1;
    if (n <= 0) {
        printf("%s]# wavestats empty array\n",prec->name);
       
        return 0;
    }
    if (start >= n){
        printf("%s]# wavestats start %ld > len %ld\n",prec->name,start,n);
        return 0;

    }
    if (start >= end){
        printf("%s]# wavestats start %ld > end %ld\n",prec->name,start,end);
        return 0;

    }
    if (end > n){
        printf("%s] wavestats end %ld > len %ld\n",prec->name,end,n);
        end=n;

    }
    //printf("processing %d array\n",n);

    max = min = y[0];

   
    for (int i = start; i < end; ++i) {
        if (y[i] > max) {
            max = y[i];
         /*   if(!strcmp(prec->name,"SPARC:DIAG:TEK:wavestats_7_")){
                printf("found new max %E at %d\n",max,i);

            }*/
        }
        if (y[i] < min) {
            min = y[i];
        }
        double val = y[i]* xinc;
        sum += y[i];
        sumsq += val * val;
       if ( (y[i] > 1) && (y[i]<10)){
            printf("%s]-%d integral  len=%ld xinc=%f norm=%f val:%f\n",prec->name,i,n, xinc,norm,y[i]);
    }
    }
    double integral=sum * xinc * norm;
    double avg=sum / n;
    double RMS=sqrt(sumsq* norm / n);
    *((double *)prec->vala) = integral;
    *((double *)prec->valb) = avg ;
    *((double *)prec->valc) = min ;
    *((double *)prec->vald) = max;
    *((double *)prec->vale) = RMS;
    if(isnan(*((double *)prec->vala))){
        printf("%s] integral NaN len=%ld xinc=%f norm=%f first:%f\n",prec->name,n, xinc,norm,y[0]);

    } else {
        // printf("%s] integral buffer=%ld start=%ld end=%ld end-size=%ld xinc=%f norm=%f integral:%f min:%f max:%f avg:%f rms:%f first:%f\n",prec->name,n, start,end,end-start,xinc,norm,integral,min,max,avg,RMS,y[0]);
    }
    


    return 0;
}

epicsRegisterFunction(incrementor);
epicsRegisterFunction(wavestats);
