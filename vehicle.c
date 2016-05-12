/* Last modified: <07-Dec-2015 14:04:50 CET by Dmitry Ebel> */

#include "vehicle.h"
#include <math.h>
#include <stdio.h>
static int compar_cm(const void* v1, const void* v2)
{
    /* reverse order ! */
    if ( *((cm_t*)v1) < *((cm_t*)v2) ) {
	return 1;
    } else {
	return -1;
    }
    return 0;
}


uint8_t veh_is_moving(const vehicle_t* const v)
{
    return (v->flags & VEH_MOVING);
}

void veh_set_moving(vehicle_t* const v, const uint8_t m)
{
    v->flags |= VEH_MOVING;
}

error_t veh_init(vehicle_t * const v, const uint8_t id, sensorlist_t sl) {
    v->id = id;
    v->timestamp_us = 0;
    v->prev_timestamp_us = 0;
    v->sensorlist = sl;
    kf_init(&(v->kf_pos));
    kf_init(&(v->kf_rot));
    
    return ERR_NOERROR;
}

void veh_set_position(vehicle_t * const v, const position2d_t * const p)
{
    v->position.x = p->x;
    v->position.y = p->y;
    v->position.orientation = p->orientation;
}

position2d_t veh_get_position(const vehicle_t* const v)
{
    return v->position;
}

double veh_get_rotation(const vehicle_t * const v)
{
    return v->position.orientation;
}

position2d_t veh_get_filtered_position(const vehicle_t* const v)
{
    position2d_t res;
    res.x = mtr_get(v->kf_pos.x, 0, 0);
    res.y = mtr_get(v->kf_pos.x, 1, 0);
    return res;
}

error_t veh_calc_rotation(vehicle_t * const vh)
{
    error_t err = ERR_NOTENOUGHDATA;
    sensornode_t* sn;
    cm_t u_vcs[2], v_vcs[2], u_wcs[2], v_wcs[2];
    cm_t d_vcs[2], d_wcs[2];
    double phi = 0;
    size_t cnt = 0;
    double sns_q[16];
    
    sn = vh->sensorlist;
    while (sn != NULL) {
	if (sn->sensor->flags & SNS_MLAT_OK ) {
	    sns_q[sn->sensor->id] = sn->sensor->mlat_score;
	}
	sn = sn->next;
    }
    qsort((void*)(&sns_q[0]), 16, sizeof(cm_t), &compar_cm);

    double sns_q_crit = sns_q[2];

    sn = vh->sensorlist;
    while (sn != NULL) {
	if (sn->sensor->flags & SNS_MLAT_OK
	    && sn->sensor->mlat_score >= sns_q_crit) {
	    u_vcs[0] = sn->sensor->position_mnt.x;
	    u_vcs[1] = sn->sensor->position_mnt.y;
	    u_wcs[0] = sn->sensor->position_wcs.x;
	    u_wcs[1] = sn->sensor->position_wcs.y;
	    sensornode_t* sn2 = sn->next;
	    while(sn2 != NULL) {
		if (sn->sensor->flags & SNS_MLAT_OK
		    && sn->sensor->mlat_score >= sns_q_crit) {
		    v_vcs[0] = sn2->sensor->position_mnt.x;
		    v_vcs[1] = sn2->sensor->position_mnt.y;
		    v_wcs[0] = sn2->sensor->position_wcs.x;
		    v_wcs[1] = sn2->sensor->position_wcs.y;
		
		    d_vcs[0] = v_vcs[0] - u_vcs[0];
		    d_vcs[1] = v_vcs[1] - u_vcs[1];
		    d_wcs[0] = v_wcs[0] - u_wcs[0];
		    d_wcs[1] = v_wcs[1] - u_wcs[1];
		
		    phi += pln_get_angle(&d_vcs[0], &d_wcs[0], 2);
		    cnt++;
		    err = ERR_NOERROR;
		}
		sn2 = sn2->next;
	    }
	}
	sn = sn->next;
    }

    if (err == ERR_NOERROR) {
	phi /= cnt;
	vh->position.orientation = phi;
    }
    
    return err;
}

error_t veh_calc_position(vehicle_t * const vh)
{
    error_t err = veh_calc_rotation(vh);
    sensornode_t* sn = vh->sensorlist;

    if (err != ERR_NOERROR) {
	return err;
    }

    double sns_q[16];
    sn = vh->sensorlist;
    while (sn != NULL) {
	if (sn->sensor->flags & SNS_MLAT_OK ) {
	    sns_q[sn->sensor->id] = sn->sensor->mlat_score;
	}
	sn = sn->next;
    }
    qsort((void*)(&sns_q[0]), 16, sizeof(cm_t), &compar_cm);
    double sns_q_crit = sns_q[2];

    position2d_t res;
    res.x = 0;
    res.y = 0;
    
    uint8_t count = 0;

    /* averaging sensor positions */
    matrix_t* rot_matrix = mtr_alloc(2,2);
    position2d_t trans_vcs, trans_wcs;

    err |= mtr_make_rot2d(rot_matrix, vh->position.orientation);
    if (err != ERR_NOERROR) {
	return err;
    }
    
    FILE* of;
    char cfn[256];
    sprintf(cfn, "/tmp/mlat.txt");
    of = fopen(cfn, "w");

    sn = vh->sensorlist;
    while (sn != NULL) {
	if (sn->sensor->flags & SNS_MLAT_OK
	    && sn->sensor->mlat_score >= sns_q_crit) {
	    for (uint8_t i=0; i<sn->sensor->id +1; i++) {
		fprintf(of, "%.1f %.1f %.1f\n", sn->sensor->position_wcs.x, sn->sensor->position_wcs.y, 2.*(i+1));
	    }

	    /* rotate the sensor vector in VCS */
	    trans_vcs.x = sn->sensor->position_mnt.x;
	    trans_vcs.y = sn->sensor->position_mnt.y;
	    err |= mtr_apply_to_pos2d(rot_matrix, &trans_vcs, &trans_wcs);

	    res.x += sn->sensor->position_wcs.x - trans_wcs.x;
	    res.y += sn->sensor->position_wcs.y - trans_wcs.y;

	    count++;
	}
	sn = sn->next;
    }
	
    res.x /= count;
    res.y /= count;

    vh->position.x = res.x;
    vh->position.y = res.y;

    fprintf(of, "%.1f %.1f 5\n", vh->position.x, vh->position.y);

 fclose(of);

    return err;
}

error_t veh_update_kfs(vehicle_t * const vh)
{
    kf_state_t current_kf_state;
    error_t err = ERR_NOERROR;
    current_kf_state.x = vh->position.x;
    current_kf_state.y = vh->position.y;
    current_kf_state.vx = vh->vx;
    current_kf_state.vy = vh->vy;
    err |= kf_update(&(vh->kf_pos), &current_kf_state);
    
    current_kf_state.x = vh->position.orientation;
    current_kf_state.vx = vh->rot_speed;
    err |= kf_update(&(vh->kf_rot), &current_kf_state);
    return err;
}

error_t veh_check_new_position(vehicle_t* vh)
{
    cm_t pos_change = sqrt ( (vh->position.x - vh->prev_position.x)*(vh->position.x - vh->prev_position.x) + 
			     (vh->position.y - vh->prev_position.y)*(vh->position.y - vh->prev_position.y) );
    if (pos_change > 2*VEH_MAX_SPEED_CMPS*1e-6*(vh->timestamp_us - vh->prev_timestamp_us) &&
	vh->prev_position.x > 0 &&
	vh->prev_position.y > 0) {
	/* outlier -> dismiss calculated position, keep previous position */
	vh->position.x = vh->prev_position.x;
	vh->position.y = vh->prev_position.y;
	return ERR_NOERROR +1;
    } else {
	/* position is reasonable (no outlier) */
	vh->prev_position.x = vh->position.x;
	vh->prev_position.y = vh->position.y;
	return ERR_NOERROR;
    }
}

void veh_update_timestamp(vehicle_t* vh, const uint64_t ts)
{
    vh->prev_timestamp_us = vh->timestamp_us;
    vh->timestamp_us = ts;
}
