// make_test_track.c — flat oval TRK1 blob (CONTRACTS §8) for tests.
//
// Geometry: ellipse rx=60 rz=40 in the x/z plane at y=0, 256 centerline
// samples, drivable width 10 m, terrain strip half-width 7 m (2 vertices per
// sample, 2 triangles per segment, closed loop). Checkpoints at samples 64,
// 128, 192. Spawn at sample 0 facing the direction of travel.

#include "make_test_track.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TT_SAMPLES 256
#define TT_CHECKPOINTS 3
#define TT_RX 60.0f
#define TT_RZ 40.0f
#define TT_WIDTH 10.0f
#define TT_TERRAIN_HALF 7.0f

typedef struct Writer
{
	uint8_t* p;
	size_t off;
} Writer;

static void wr( Writer* w, const void* data, size_t len )
{
	memcpy( w->p + w->off, data, len );
	w->off += len;
}

static void wr_u16( Writer* w, uint16_t v )
{
	wr( w, &v, 2 );
}

static void wr_u32( Writer* w, uint32_t v )
{
	wr( w, &v, 4 );
}

static void wr_u64( Writer* w, uint64_t v )
{
	wr( w, &v, 8 );
}

static void wr_f32( Writer* w, float v )
{
	wr( w, &v, 4 );
}

uint8_t* make_test_track( size_t* out_len )
{
	const uint32_t S = TT_SAMPLES;
	const uint32_t C = TT_CHECKPOINTS;
	const uint32_t V = 2 * S;
	const uint32_t T = 2 * S;

	size_t len = 28 + (size_t)40 * S + (size_t)4 * C + (size_t)12 * V + (size_t)12 * T + 12;
	uint8_t* blob = (uint8_t*)malloc( len );
	if ( blob == NULL )
	{
		return NULL;
	}

	Writer w = { blob, 0 };

	// Header
	wr_u32( &w, 0x4B525453u ); // "STRK"
	wr_u16( &w, 1 );		   // version
	wr_u16( &w, (uint16_t)C );
	wr_u64( &w, 0x7E57DA7Aull ); // seed (provenance only)
	wr_u32( &w, S );
	wr_u32( &w, V );
	wr_u32( &w, T );

	// Centerline samples + cached tangents/sides for the terrain strip
	static float px[TT_SAMPLES], pz[TT_SAMPLES], sx[TT_SAMPLES], sz[TT_SAMPLES];
	static float tx0, tz0;

	for ( uint32_t i = 0; i < S; ++i )
	{
		double theta = ( 2.0 * 3.14159265358979323846 * (double)i ) / (double)S;
		float pos_x = (float)( TT_RX * sin( theta ) );
		float pos_z = (float)( TT_RZ * cos( theta ) );

		// Tangent (direction of travel, increasing i)
		double dx = TT_RX * cos( theta );
		double dz = -TT_RZ * sin( theta );
		double tl = sqrt( dx * dx + dz * dz );
		float tan_x = (float)( dx / tl );
		float tan_z = (float)( dz / tl );

		// side = cross(up, tangent) with up = +Y: (tz, 0, -tx)
		float side_x = tan_z;
		float side_z = -tan_x;

		px[i] = pos_x;
		pz[i] = pos_z;
		sx[i] = side_x;
		sz[i] = side_z;
		if ( i == 0 )
		{
			tx0 = tan_x;
			tz0 = tan_z;
		}

		wr_f32( &w, pos_x );
		wr_f32( &w, 0.0f );
		wr_f32( &w, pos_z );
		wr_f32( &w, 0.0f ); // up
		wr_f32( &w, 1.0f );
		wr_f32( &w, 0.0f );
		wr_f32( &w, tan_x ); // tangent
		wr_f32( &w, 0.0f );
		wr_f32( &w, tan_z );
		wr_f32( &w, TT_WIDTH );
	}

	// Checkpoint sample indices (ascending)
	wr_u32( &w, S / 4 );
	wr_u32( &w, S / 2 );
	wr_u32( &w, ( 3 * S ) / 4 );

	// Terrain vertices: [2i] = left edge, [2i+1] = right edge
	for ( uint32_t i = 0; i < S; ++i )
	{
		wr_f32( &w, px[i] - TT_TERRAIN_HALF * sx[i] );
		wr_f32( &w, 0.0f );
		wr_f32( &w, pz[i] - TT_TERRAIN_HALF * sz[i] );
		wr_f32( &w, px[i] + TT_TERRAIN_HALF * sx[i] );
		wr_f32( &w, 0.0f );
		wr_f32( &w, pz[i] + TT_TERRAIN_HALF * sz[i] );
	}

	// Terrain triangles: quad per segment (wraps), wound so normals face +Y.
	for ( uint32_t i = 0; i < S; ++i )
	{
		uint32_t j = ( i + 1 ) % S;
		uint32_t l0 = 2 * i, r0 = 2 * i + 1;
		uint32_t l1 = 2 * j, r1 = 2 * j + 1;

		// Winding checked analytically: with side pointing left of travel and
		// up = +Y, (l0, l1, r0) and (r0, l1, r1) give upward-facing normals
		// when the loop runs clockwise viewed from +Y; flip check below keeps
		// it robust for either travel direction.
		float ax = px[i] - TT_TERRAIN_HALF * sx[i];
		float az = pz[i] - TT_TERRAIN_HALF * sz[i];
		float bx = px[j] - TT_TERRAIN_HALF * sx[j];
		float bz = pz[j] - TT_TERRAIN_HALF * sz[j];
		float cx = px[i] + TT_TERRAIN_HALF * sx[i];
		float cz = pz[i] + TT_TERRAIN_HALF * sz[i];
		// normal.y of cross(b-a, c-a) for y-flat triangles: (bz-az)(cx-ax) - (bx-ax)(cz-az)
		float ny = ( bz - az ) * ( cx - ax ) - ( bx - ax ) * ( cz - az );

		if ( ny > 0.0f )
		{
			wr_u32( &w, l0 );
			wr_u32( &w, l1 );
			wr_u32( &w, r0 );
			wr_u32( &w, r0 );
			wr_u32( &w, l1 );
			wr_u32( &w, r1 );
		}
		else
		{
			wr_u32( &w, l0 );
			wr_u32( &w, r0 );
			wr_u32( &w, l1 );
			wr_u32( &w, l1 );
			wr_u32( &w, r0 );
			wr_u32( &w, r1 );
		}
	}

	// Spawn pose: sample 0, yaw from its tangent (forward = +Z at yaw 0,
	// rotating about +Y: fwd = (sin yaw, 0, cos yaw) → yaw = atan2(tx, tz)).
	wr_u32( &w, 0 );
	wr_f32( &w, (float)atan2( (double)tx0, (double)tz0 ) );
	wr_f32( &w, 0.0f ); // reserved

	*out_len = w.off;
	return blob;
}
