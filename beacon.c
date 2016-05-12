//Last modified: <07-Dec-2015 13:09:26 CET by Dmitry Ebel>
#include "beacon.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <libconfig.h>

beacon_t bcn_init(const uint8_t id, const cm_t x, const cm_t y, const cm_t z)
{
    beacon_t res;
    res.id   = id;
    res.flags = 0x0 | BCN_POSKNOWN | BCN_IN_FOV; /* init as visible since position and rotation unknown */
    res.position.x    = x;
    res.position.y    = y;
    res.position.z    = z;
        
    return res;
}

beacon_t bcn_new(const uint8_t id)
{
    beacon_t res;
    res.flags = 0x0 | BCN_IN_FOV; /* init as visible since position and rotation unknown */;
    res.id   = id;
    res.position.x    = 0;
    res.position.y    = 0;
    res.position.z    = 0;
        
    return res;
}

error_t bcn_add_to_list(beaconnode_t** beaconlist, beacon_t* b)
{
    beaconnode_t* newnode = (beaconnode_t*)malloc(sizeof(beaconnode_t));
    if (newnode == NULL) {
	return ERR_MALLOC;
    }
    newnode->beacon = (beacon_t*)malloc(sizeof(beacon_t));
    newnode->beacon->id = b->id;
    newnode->beacon->flags = b->flags;
    newnode->beacon->position.x = b->position.x;
    newnode->beacon->position.y = b->position.y;
    newnode->beacon->position.z = b->position.z;


    if (*beaconlist == NULL) {
	*beaconlist = newnode;
	(*beaconlist)->next = NULL;
    }
    else {
	newnode->next = *beaconlist;
	*beaconlist = newnode;
    }
    return ERR_NOERROR;
}

error_t bcn_append(beaconnode_t** beaconlist, beacon_t* beacon)
{
    beaconnode_t* newnode = NULL;
    newnode = (beaconnode_t*)malloc(sizeof(beaconnode_t));
    if (newnode == NULL) {
	return ERR_MALLOC;
    }

    newnode->beacon = beacon;
    newnode->next = NULL;

    beaconnode_t *bn = *beaconlist;
    if (*beaconlist == NULL) {
	*beaconlist = (beaconnode_t*)malloc(sizeof(beaconnode_t));
	if (*beaconlist == NULL) return ERR_MALLOC;
	*beaconlist = newnode;
    } else {
	while (bn->next != NULL) {
	    bn = bn->next;
	}
	bn->next = newnode;
    }
    return ERR_NOERROR;
}

beaconnode_t* bcn_get_byID(beaconnode_t** beaconlist, const uint8_t id)
{
    beaconnode_t* bn = *beaconlist;

    while (bn != NULL) {
	if (bn->beacon->id == id) {
	    return bn;
	}
	bn = bn->next;
    }
    return NULL;
}

error_t bcn_remove_byID(beaconnode_t** beaconlist, const uint8_t id)
{
    beaconnode_t* bn = *beaconlist;
    beaconnode_t* prevnode = bn;
    
    while (bn != NULL) {
	if (bn->beacon->id == id) {
	    prevnode->next = bn->next;
	    if (bn == *beaconlist) { /* first node to be removed */
		*beaconlist = bn->next;
	    }
	    free(bn);
	    bn = NULL;

	    return ERR_NOERROR;
	}
	prevnode = bn;
	bn = bn->next;
    }
    
    return ERR_NODENOTFOUND;
}

uint8_t bcn_count(const beaconlist_t const *bl, const uint8_t mask)
{
    uint8_t res = 0;
    beaconnode_t* bn = *bl;
    
    while (bn != NULL) {
	if (mask == BCN_ANY || ( (bn->beacon->flags & mask) == mask) ) {
	    res++;
	}
	bn = bn->next;
    }
    return res;
}

int8_t bcn_is_visible(const beacon_t* const beacon)
{
    return (beacon->flags & BCN_SIGNAL_OK);
}

int8_t bcn_is_ignored(const beacon_t const *beacon)
{
    return (beacon->flags & BCN_IGNORED);
}

uint8_t beacon_count_visible(const beaconlist_t const *beaconlist)
{
    const uint8_t mask = BCN_IN_FOV | BCN_SIGNAL_OK | (~BCN_IGNORED);
    return bcn_count(beaconlist, mask);
}

error_t bcn_ignore_byID(beaconnode_t** const beaconlist, const int8_t id)
{    
    beaconnode_t* bn = *beaconlist;
    
    while (bn != NULL) {
	if (bn->beacon->id == id) {
	    bn->beacon->flags |= BCN_IGNORED;
	    return ERR_NOERROR;
	}
	bn = bn->next;
    }
    return ERR_NODENOTFOUND;
}

void bcn_ignore(beacon_t* const beacon)
{
    beacon->flags |= BCN_IGNORED;
}

void bcn_print(const beacon_t* const beacon)
{
    printf("BID=%2d x=%-6.1f y=%-6.1f z=%-6.1f\n", beacon->id,
	   (float)beacon->position.x, (float)beacon->position.y, (float)beacon->position.z);
}

void bcn_list_beacons(beaconnode_t** beaconlist)
{
    beaconnode_t* bn = *beaconlist;
    
    printf("Beacons:\n");
    while (bn != NULL) {
	bcn_print(bn->beacon);
	bn = bn->next;
    }
    printf("\n");
}

error_t bcn_read_config(const char* fn, beaconlist_t* bl)
{
    config_t cfg;
    config_setting_t *cfg_beacon, *cfg_beacons, *cfg_mntpos;
    int bid, num_beacons, bcn_cnt, used;
    double sx, sy, sz;
    beacon_t b;
    
    config_init(&cfg);
    if(! config_read_file(&cfg, fn)) {
	fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg),
		config_error_line(&cfg), config_error_text(&cfg));
	config_destroy(&cfg);
	return(1);
    }
    
    cfg_beacons = config_lookup(&cfg, "stage.beacons");

    num_beacons  = config_setting_length(cfg_beacons);
    for(bcn_cnt = 0; bcn_cnt < num_beacons; bcn_cnt++) {
	cfg_beacon = config_setting_get_elem(cfg_beacons, bcn_cnt);
	config_setting_lookup_int(cfg_beacon, "id", &bid);
	config_setting_lookup_int(cfg_beacon, "used", &used);
	if (used == 1) {
#if LIBCONFIG_VER_MAJOR >=1 && LIBCONFIG_VER_MINOR>=5
	    cfg_mntpos = config_setting_lookup(cfg_beacon, "mount");
#else
	    cfg_mntpos = config_lookup_from(cfg_beacon, "mount"); 
#endif
	    config_setting_lookup_float(cfg_mntpos, "x", &sx);
	    config_setting_lookup_float(cfg_mntpos, "y", &sy);
	    config_setting_lookup_float(cfg_mntpos, "z", &sz);
	    b = bcn_init(bid, sx, sy, sz);
	    bcn_add_to_list(bl, &b);
	}
    }
    return ERR_NOERROR;
}
