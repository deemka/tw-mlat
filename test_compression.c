#include "pack.h"
#include <stdio.h>

int main()
{
    packed_data_t pd;
    error_t err = pd_init(&pd, 4);
    uint16_t a[4];

    a[0] = 0b000001111111110;
    a[1] = 0b000011111111111;
    a[2] = 0b000001111111100;
    a[3] = 0b000000011111000;
    printf ("%d \n",a[0]);
    printf ("%d \n",a[1]);
    printf ("%d \n",a[2]);
    printf ("%d \n",a[3]);
    
    err |= pd_put_sample(&pd, a[0]);
    err |= pd_put_sample(&pd, a[1]);
    err |= pd_put_sample(&pd, a[2]);
    err |= pd_put_sample(&pd, a[3]);
    printf ("%02x %02x %02x %02x %02x %02x\n", pd.data[0], pd.data[1], pd.data[2], pd.data[3], pd.data[4], pd.data[5]);
    

    uint16_t** buf;
    *buf = malloc(4*2);

    err |= pd_pack_buffer(&pd, &a[0], 4);
    err |= pd_unpack_buffer(&pd, buf);

    for (a[0]=0; a[0]<4; a[0]++) {
	printf ("%d \n", (*buf)[a[0]]);
    }
    return 0;
}
