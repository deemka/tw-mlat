#include <stdio.h>
#include <time.h>
#include "math_float.h"

int main1()
{
    
    srand(time(NULL));
    unsigned ts_in[20];
    int delta[20];
    const int delta_real = 168000;
    const int err_max = 100;
    int delta_mean = 0;
    
    for (int i=0; i<sizeof(ts_in)/sizeof(unsigned); i++) {
	delta[i] =  rand()%(2*err_max) - err_max;
	ts_in[i] = i*delta_real + delta[i];
	printf("%d %d\n", ts_in[i], delta[i]);
    }
    
    printf("%d %d\n", mean(&delta[0], 20), median(&delta[0], 20));
    return 0;   
}


int main()
{
    return main1();
}
