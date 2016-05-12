#include "stage.h"
#include <libconfig.h>

error_t stg_init_from_config(stage_t* stage, const char* cfile)
{
    error_t err = ERR_NOERROR;
    config_t cfg;
    config_setting_t *cfg_size;

    stage->beaconlist = NULL;
    
    /* Read stage config */
    config_init(&cfg);
    if(! config_read_file(&cfg, cfile)) {
	return ERR_CONFIG;
    }
    cfg_size = config_lookup(&cfg, "stage.size");
    config_setting_lookup_float(cfg_size, "width", &stage->width);
    config_setting_lookup_float(cfg_size, "depth", &stage->depth);

    /* Read beacons config */
    err |= bcn_read_config(cfile, &stage->beaconlist);
    if (err != ERR_NOERROR) {
	return err;
    }

    return err;
}

int stg_in_mutual_fov(sensor_t const * const s, beacon_t const * const b)
{
    position2d_t ps;
    ps.x = s->position_wcs.x;
    ps.y = s->position_wcs.y;
    ps.orientation = s->position_wcs.orientation_h;
    
    position2d_t pb;
    pb.x = b->position.x;
    pb.y = b->position.y;
    pb.orientation = b->position.orientation_h;

    return ( pln_is_in_fov(&ps, &pb, SNS_BEAM_ANGLE) &&
	     pln_is_in_fov(&pb, &ps, BCN_BEAM_ANGLE) );
    
}
