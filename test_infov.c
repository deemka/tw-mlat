#include "planimetrics.h"
#include <stdio.h>

int main()
{
    position2d_t v, o;

    v.x = 0;  v.y = 10;  v.orientation = M_PI*50/180.;
    o.x = 1200; o.y = 1200; o.orientation = M_PI*270/180.;
			       
    
    printf("%s\n", ((pln_is_in_fov(&v, &o, M_PI*85/180.)) ?  "in FOV" : "out of FOV") );
			      

    return 0;
}
