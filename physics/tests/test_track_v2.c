// test_track_v2.c — TRK1 v2 static props + off-corridor grass fallback
// (CONTRACTS §8 v2, docs/ROAD-SURFACE.md §6).
//
//   1. FORMAT       — v1 still parses; v2 with 0/2 props parses; malformed v2
//                     (missing/truncated prop records, count > 4096, bad
//                     half extents / NaN pose, unknown version) rejected with
//                     the exact error codes.
//   2. PROP WORLD   — driving the chassis straight into a prop placed on the
//                     spawn straight produces a hit event → crash damage
//                     accrues (front-region), at a chassis X consistent with
//                     the prop pose, and the prop physically blocks the car.
//   3. GRASS QUERY  — road_query with grass_fallback ON: off-corridor points
//                     return the flat plane at ground_y (+Y normal, kind
//                     GRASS); ON-corridor answers are BIT-IDENTICAL to a
//                     pre-change capture (hex-float compare). Legacy mode
//                     (grass_fallback OFF) keeps the old off-corridor
//                     extrapolation with on_road == 0.
//   4. GRASS DRIVE  — on a v2 track the car can drive off the corridor onto
//                     grass: OFF_TRACK fires, the car does NOT fall into the
//                     void, and it keeps rolling (drivable, just slow).

#include "make_test_track.h"
#include "road.h"	 // src/, not the public include/ surface
#include "sim/sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_failures = 0;

#define CHECK( cond, msg )                                                                                             \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( !( cond ) )                                                                                               \
		{                                                                                                              \
			fprintf( stderr, "FAIL: %s (line %d)\n", msg, __LINE__ );                                                  \
			g_failures++;                                                                                              \
		}                                                                                                              \
		else                                                                                                           \
		{                                                                                                              \
			printf( "ok: %s\n", msg );                                                                                 \
		}                                                                                                              \
	} while ( 0 )

// Bit-exact float compare (the pre-change capture is exact hex-float bits).
static int f32_bits_equal( float a, float b )
{
	return memcmp( &a, &b, 4 ) == 0;
}

// ---------------------------------------------------------------------------
// v2 blob builder: take the v1 flat-oval blob, set version = 2, append
// prop_count + 36-byte prop records (CONTRACTS §8 v2 layout).
// ---------------------------------------------------------------------------

typedef struct PropRec
{
	uint16_t type;
	float pos[3];
	float yaw;
	float half[3];
} PropRec;

static void put_u16( uint8_t* p, uint16_t v )
{
	memcpy( p, &v, 2 );
}

static void put_u32( uint8_t* p, uint32_t v )
{
	memcpy( p, &v, 4 );
}

static void put_f32( uint8_t* p, float v )
{
	memcpy( p, &v, 4 );
}

// declared_count is written into the blob; records_written records actually
// follow (differing values build deliberately truncated blobs).
static uint8_t* make_v2_track( size_t* out_len, uint32_t declared_count, const PropRec* props,
							   uint32_t records_written )
{
	size_t v1_len = 0;
	uint8_t* v1 = make_test_track( &v1_len );
	if ( v1 == NULL )
	{
		return NULL;
	}

	size_t len = v1_len + 4 + (size_t)36 * records_written;
	uint8_t* blob = (uint8_t*)malloc( len );
	if ( blob == NULL )
	{
		free( v1 );
		return NULL;
	}
	memcpy( blob, v1, v1_len );
	free( v1 );

	put_u16( blob + 4, 2 ); // version = 2
	put_u32( blob + v1_len, declared_count );
	for ( uint32_t i = 0; i < records_written; ++i )
	{
		uint8_t* r = blob + v1_len + 4 + (size_t)36 * i;
		put_u16( r + 0, props[i].type );
		put_u16( r + 2, 0 ); // _pad
		put_f32( r + 4, props[i].pos[0] );
		put_f32( r + 8, props[i].pos[1] );
		put_f32( r + 12, props[i].pos[2] );
		put_f32( r + 16, props[i].yaw );
		put_f32( r + 20, props[i].half[0] );
		put_f32( r + 24, props[i].half[1] );
		put_f32( r + 28, props[i].half[2] );
		put_u32( r + 32, 0 ); // reserved (pads the record to 36 bytes)
	}

	*out_len = len;
	return blob;
}

// ---------------------------------------------------------------------------
// 1. FORMAT — parse/reject matrix
// ---------------------------------------------------------------------------

static void test_format( void )
{
	// v1 still parses (explicit assertion, not just via other suites).
	{
		size_t len = 0;
		uint8_t* blob = make_test_track( &len );
		CHECK( blob != NULL && sim_load_track( (sim_ptr_t)blob, (uint32_t)len ) == 0, "v1 blob parses OK" );
		free( blob );
	}

	static const PropRec two_props[2] = {
		{ 0, { 14.0f, 0.6f, 40.0f }, 0.3f, { 0.5f, 0.6f, 0.5f } },	 // tire stack on the spawn straight
		{ 1, { -20.0f, 0.6f, 40.0f }, 0.0f, { 0.5f, 0.5f, 0.5f } }, // armco behind the spawn (never touched)
	};

	// v2 with 0 props parses.
	{
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 0, NULL, 0 );
		CHECK( blob != NULL && sim_load_track( (sim_ptr_t)blob, (uint32_t)len ) == 0, "v2 blob with 0 props parses OK" );
		free( blob );
	}

	// v2 with 2 props parses.
	{
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 2, two_props, 2 );
		CHECK( blob != NULL && sim_load_track( (sim_ptr_t)blob, (uint32_t)len ) == 0, "v2 blob with 2 props parses OK" );
		free( blob );
	}

	// v2 truncated to the v1 length (prop_count missing) → TRUNCATED.
	{
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 0, NULL, 0 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)( len - 4 ) );
		CHECK( rc == SIM_ERR_TRUNCATED, "v2 without prop_count → SIM_ERR_TRUNCATED" );
		free( blob );
	}

	// v2 declaring 2 props but carrying only 1 record → TRUNCATED.
	{
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 2, two_props, 1 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)len );
		CHECK( rc == SIM_ERR_TRUNCATED, "v2 with truncated prop records → SIM_ERR_TRUNCATED" );
		free( blob );
	}

	// v2 with one record truncated mid-record → TRUNCATED.
	{
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 1, two_props, 1 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)( len - 10 ) );
		CHECK( rc == SIM_ERR_TRUNCATED, "v2 with mid-record truncation → SIM_ERR_TRUNCATED" );
		free( blob );
	}

	// prop_count > 4096 → BAD_COUNTS (checked before any allocation).
	{
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 4097, NULL, 0 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)len );
		CHECK( rc == SIM_ERR_BAD_COUNTS, "prop_count 4097 → SIM_ERR_BAD_COUNTS" );
		free( blob );
	}

	// Zero / NaN half extents and NaN pose → BAD_GEOMETRY.
	{
		PropRec bad = { 0, { 5.0f, 0.5f, 40.0f }, 0.0f, { 0.0f, 0.5f, 0.5f } };
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 1, &bad, 1 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)len );
		CHECK( rc == SIM_ERR_BAD_GEOMETRY, "prop with zero half extent → SIM_ERR_BAD_GEOMETRY" );
		free( blob );
	}
	{
		PropRec bad = { 0, { 5.0f, NAN, 40.0f }, 0.0f, { 0.5f, 0.5f, 0.5f } };
		size_t len = 0;
		uint8_t* blob = make_v2_track( &len, 1, &bad, 1 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)len );
		CHECK( rc == SIM_ERR_BAD_GEOMETRY, "prop with NaN position → SIM_ERR_BAD_GEOMETRY" );
		free( blob );
	}

	// Unknown version → BAD_VERSION.
	{
		size_t len = 0;
		uint8_t* blob = make_test_track( &len );
		put_u16( blob + 4, 3 );
		int32_t rc = sim_load_track( (sim_ptr_t)blob, (uint32_t)len );
		CHECK( rc == SIM_ERR_BAD_VERSION, "version 3 → SIM_ERR_BAD_VERSION" );
		free( blob );
	}
}

// ---------------------------------------------------------------------------
// 2. PROP WORLD — drive the chassis into a prop, damage accrues.
// ---------------------------------------------------------------------------

// The impact must be above the 6 m/s damage threshold for the damage gate to
// be meaningful (the launch reaches ~9-10 m/s over 14 m).
#define SIM_DAMAGE_THRESHOLD_TEST_MIN 6.0f

static void test_prop_collision( void )
{
	// Spawn (sample 0) is at (0, 0, 40) facing +X on the flat oval. Prop A
	// sits 14 m dead ahead on the spawn straight; prop B is behind the car.
	static const PropRec props[2] = {
		{ 0, { 14.0f, 0.6f, 40.0f }, 0.3f, { 0.5f, 0.6f, 0.5f } },
		{ 1, { -20.0f, 0.6f, 40.0f }, 0.0f, { 0.5f, 0.5f, 0.5f } },
	};
	size_t len = 0;
	uint8_t* blob = make_v2_track( &len, 2, props, 2 );
	if ( blob == NULL || sim_load_track( (sim_ptr_t)blob, (uint32_t)len ) != 0 )
	{
		fprintf( stderr, "FAIL: could not load 2-prop v2 track\n" );
		g_failures++;
		free( blob );
		return;
	}
	const SimStateV1* s = sim_state();
	sim_reset();

	int error_seen = 0;
	int impact_tick = -1;
	float impact_x = 0.0f;
	float speed_before_impact = 0.0f; // last tick BEFORE the hit (the solver
									  // absorbs the impact within the hit tick)
	float prev_speed = 0.0f;
	float damage_before_zone = 0.0f;
	float max_x = -1000.0f;

	for ( int tick = 0; tick < 2400; ++tick ) // 6 s, plenty to cover 14 m
	{
		uint32_t status = sim_step( 0, 65535, 0, 0 ); // full throttle, straight
		if ( status & SIM_STATUS_ERROR )
		{
			error_seen = 1;
			break;
		}
		if ( s->pos[0] > max_x )
		{
			max_x = s->pos[0];
		}
		float d = sim_damage( 0 );
		if ( s->pos[0] < 9.0f && d > damage_before_zone )
		{
			damage_before_zone = d; // any damage before the prop zone is a bug
		}
		if ( impact_tick < 0 && d > 0.0f )
		{
			impact_tick = tick;
			impact_x = s->pos[0];
			speed_before_impact = prev_speed;
		}
		prev_speed = s->speed;
	}

	printf( "prop crash: impact_tick=%d impact_x=%.2f speed_before=%.2f max_x=%.2f dmg=(%.3f, %.3f, %.4f, %.4f)\n",
			impact_tick, (double)impact_x, (double)speed_before_impact, (double)max_x, (double)sim_damage( 0 ),
			(double)sim_damage( 1 ), (double)sim_damage( 2 ), (double)sim_damage( 3 ) );

	CHECK( !error_seen, "no ERROR while driving into the prop" );
	CHECK( damage_before_zone == 0.0f, "no damage before reaching the prop" );
	CHECK( impact_tick > 0, "chassis-vs-prop hit event accrues damage" );
	// Impact chassis X consistent with the prop pose: nose (+2.2 m) meets the
	// yawed 0.5 m box at x ≈ 14 − ~0.6 − 2.2 ≈ 11.2.
	CHECK( impact_x > 9.5f && impact_x < 13.5f, "impact position matches the prop pose" );
	CHECK( speed_before_impact > SIM_DAMAGE_THRESHOLD_TEST_MIN, "approach speed above the damage threshold" );
	CHECK( sim_damage( 0 ) > 0.0f, "overall damage > 0 after the crash" );
	CHECK( sim_damage( 1 ) > 0.0f, "front impact accrues steering damage" );
	// The prop is static and solid: the car cannot end up meaningfully past it.
	CHECK( max_x < 15.5f, "prop physically blocks the car" );

	free( blob );
}

// ---------------------------------------------------------------------------
// 3. GRASS QUERY — flat-oval road_query, grass fallback vs legacy, and the
// bit-identical pre-change capture for on-corridor points.
// ---------------------------------------------------------------------------

// Same oval builder as tests/test_road.c (ellipse rx=60 rz=40, width 10).
#define OVAL_S 256
#define OVAL_RX 60.0
#define OVAL_RZ 40.0
#define OVAL_WIDTH 10.0f

static void build_oval( RoadSample* s )
{
	for ( int i = 0; i < OVAL_S; ++i )
	{
		double th = ( 2.0 * 3.14159265358979323846 * (double)i ) / (double)OVAL_S;
		double dx = OVAL_RX * cos( th );
		double dz = -OVAL_RZ * sin( th );
		double tl = sqrt( dx * dx + dz * dz );
		s[i].pos = ( b3Vec3 ){ (float)( OVAL_RX * sin( th ) ), 0.0f, (float)( OVAL_RZ * cos( th ) ) };
		s[i].tangent = ( b3Vec3 ){ (float)( dx / tl ), 0.0f, (float)( dz / tl ) };
		s[i].up = ( b3Vec3 ){ 0.0f, 1.0f, 0.0f };
		s[i].width = OVAL_WIDTH;
	}
}

static b3Vec3 side_of( const RoadSample* s )
{
	return b3Cross( s->up, s->tangent );
}

// One pre-change capture entry: query input reproduced by expression, output
// compared bit-exactly (recorded from the pre-change kernel with %a).
typedef struct Capture
{
	const char* label;
	uint32_t hint;
	float px, py, pz;						  // expected surface point
	float nx, ny, nz;						  // expected normal
	float lateral;							  // expected signed lateral
	uint32_t seg;							  // expected segment
} Capture;

static void check_capture( const Road* road, b3Vec3 p, const Capture* c )
{
	RoadQuery q;
	road_query( road, p, c->hint, &q );
	char msg[128];
	snprintf( msg, sizeof( msg ), "%s: on-corridor query bit-identical to pre-change capture", c->label );
	CHECK( f32_bits_equal( q.point.x, c->px ) && f32_bits_equal( q.point.y, c->py ) &&
			   f32_bits_equal( q.point.z, c->pz ) && f32_bits_equal( q.normal.x, c->nx ) &&
			   f32_bits_equal( q.normal.y, c->ny ) && f32_bits_equal( q.normal.z, c->nz ) &&
			   f32_bits_equal( q.lateral, c->lateral ) && q.seg == c->seg && q.on_road == 1 &&
			   q.kind == ROAD_KIND_ASPHALT,
		   msg );
}

static void test_grass_query( void )
{
	static RoadSample flat[OVAL_S];
	build_oval( flat );

	Road grass_road, legacy_road;
	if ( road_load( &grass_road, flat, OVAL_S, 1 ) != 0 || road_load( &legacy_road, flat, OVAL_S, 0 ) != 0 )
	{
		fprintf( stderr, "FAIL: road_load failed\n" );
		g_failures++;
		return;
	}

	// ground_y: flat oval samples all at y = 0 → 0 − 1.35 exactly.
	CHECK( f32_bits_equal( grass_road.ground_y, -1.35f ), "ground_y = min sample y - 1.35 (exact)" );

	// Off-corridor (|l| = width/2 + 9 > 13): grass plane.
	{
		b3Vec3 p = b3MulAdd( flat[0].pos, 0.5f * OVAL_WIDTH + 9.0f, side_of( &flat[0] ) );
		RoadQuery q;
		road_query( &grass_road, p, 0, &q );
		CHECK( q.kind == ROAD_KIND_GRASS && !q.on_road, "off-corridor query reports kind = GRASS, on_road = 0" );
		CHECK( f32_bits_equal( q.point.y, -1.35f ), "grass surface height is exactly ground_y" );
		CHECK( f32_bits_equal( q.point.x, p.x ) && f32_bits_equal( q.point.z, p.z ),
			   "grass surface point is directly below the query point" );
		CHECK( f32_bits_equal( q.normal.x, 0.0f ) && f32_bits_equal( q.normal.y, 1.0f ) &&
				   f32_bits_equal( q.normal.z, 0.0f ),
			   "grass normal is exactly +Y" );
	}

	// Far off in the infield: still the grass plane (query is total).
	{
		b3Vec3 p = { 10.0f, 3.0f, 5.0f }; // deep inside the oval
		RoadQuery q;
		road_query( &grass_road, p, 128, &q );
		CHECK( q.kind == ROAD_KIND_GRASS && f32_bits_equal( q.point.y, -1.35f ),
			   "deep-infield query returns the grass plane" );
	}

	// Legacy mode (v1 tracks): off-corridor keeps the pre-change behavior —
	// extrapolated shoulder ramp, on_road = 0, kind = ASPHALT.
	{
		b3Vec3 p = b3MulAdd( flat[0].pos, 0.5f * OVAL_WIDTH + 11.0f, side_of( &flat[0] ) );
		RoadQuery q;
		road_query( &legacy_road, p, 0, &q );
		float expect = -1.2f * ( 11.0f / 8.0f ); // −1.65 m, ramp extrapolation
		CHECK( !q.on_road && q.kind == ROAD_KIND_ASPHALT, "legacy off-corridor stays on_road = 0, kind = ASPHALT" );
		CHECK( fabsf( q.point.y - expect ) < 0.005f, "legacy off-corridor keeps the extrapolated shoulder ramp" );
	}

	// On-corridor bit-identity: captured from the PRE-change kernel (%a) on
	// this exact oval; the same queries must reproduce every bit, in both
	// grass and legacy modes.
	static const Capture caps[] = {
		{ "centerline s10", 10, 0x1.d2859ep+3f, 0x0p+0f, 0x1.3668f6p+5f, 0x0p+0f, 0x1p+0f, 0x0p+0f, 0x0p+0f, 10 },
		{ "crown s10 l=W/4", 10, 0x1.c55858p+3f, -0x1.999992p-8f, 0x1.22aee4p+5f, -0x1.afc5fcp-11f, 0x1.fffe5cp-1f,
		  -0x1.433352p-8f, 0x1.3ffffep+1f, 10 },
		{ "kerb s64", 64, 0x1.b3999ap+5f, 0x1.47ae14p-7f, 0x1.178430p-49f, 0x0p+0f, 0x1p+0f, 0x0p+0f, 0x1.633330p+2f,
		  64 },
		{ "shoulder s0", 0, 0x1.b3116ap-18f, -0x1.51eb68p-4f, 0x1.13999ap+5f, -0x1.87596ap-27f, 0x1.fa55dep-1f,
		  -0x1.2fcd20p-3f, 0x1.633330p+2f, 0 },
		{ "midseg s5 l=2", 5, 0x1.05ad1ap+3f, -0x1.04a2e0p-8f, 0x1.2cfddep+5f, -0x1.86c404p-12f, 0x1.fffef8p-1f,
		  -0x1.043e56p-8f, 0x1.fe8682p+0f, 5 },
		{ "outer shoulder s100", 100, 0x1.fdf344p+4f, -0x1.2f5c2cp+0f, -0x1.39a7e8p+4f, -0x1.23a2aap-4f,
		  0x1.fa55dep-1f, 0x1.0a84e2p-3f, 0x1.9ccccep+3f, 99 },
	};

	b3Vec3 pts[6];
	pts[0] = flat[10].pos;
	pts[1] = b3MulAdd( flat[10].pos, 0.25f * OVAL_WIDTH, side_of( &flat[10] ) );
	pts[2] = b3MulAdd( flat[64].pos, 0.5f * OVAL_WIDTH + 0.55f, side_of( &flat[64] ) );
	pts[3] = b3MulAdd( flat[0].pos, 0.5f * OVAL_WIDTH + 0.55f, side_of( &flat[0] ) );
	pts[4] = b3MulAdd( b3MulAdd( flat[5].pos, 2.0f, side_of( &flat[5] ) ), 1.0f, flat[5].tangent );
	pts[5] = b3MulAdd( flat[100].pos, 0.5f * OVAL_WIDTH + 7.9f, side_of( &flat[100] ) );

	for ( int k = 0; k < 6; ++k )
	{
		check_capture( &grass_road, pts[k], &caps[k] );
		check_capture( &legacy_road, pts[k], &caps[k] );
	}

	road_free( &grass_road );
	road_free( &legacy_road );
}

// ---------------------------------------------------------------------------
// 4. GRASS DRIVE — v2 track: straight off the corridor onto solid grass.
// ---------------------------------------------------------------------------

static void test_grass_drive( void )
{
	size_t len = 0;
	uint8_t* blob = make_v2_track( &len, 0, NULL, 0 ); // v2, no props
	if ( blob == NULL || sim_load_track( (sim_ptr_t)blob, (uint32_t)len ) != 0 )
	{
		fprintf( stderr, "FAIL: could not load v2 track for the grass drive\n" );
		g_failures++;
		free( blob );
		return;
	}
	const SimStateV1* s = sim_state();
	sim_reset();

	// Full throttle, dead straight: the oval curves away, the car crosses the
	// shoulder and the corridor skirt (~13 m lateral, around x ≈ 45) and drops
	// onto the grass plane. Pre-change this ended in a void fall.
	//
	// Input update for the drivetrain wave: once on the grass, the driver
	// lifts to 40% throttle. The geared 1st (9.8:1) at full throttle just
	// lights the rears up to the limiter on the 0.55-mu surface (kinetic
	// bristle friction) and the car bogs into wheelspin — the "drivable,
	// recoverable" gate is about sane recovery inputs, not a pinned pedal.
	int error_seen = 0;
	int off_track_on_grass = 0;
	float min_y = 1000.0f;
	for ( int tick = 0; tick < 4000; ++tick ) // 10 s
	{
		uint32_t throttle = s->pos[0] > 50.0f ? 26214u : 65535u; // lift on grass
		uint32_t status = sim_step( 0, throttle, 0, 0 );
		if ( status & SIM_STATUS_ERROR )
		{
			error_seen = 1;
			break;
		}
		if ( s->pos[1] < min_y )
		{
			min_y = s->pos[1];
		}
		// Chassis center comfortably past the corridor edge → wheels on grass.
		if ( s->pos[0] > 50.0f && ( status & SIM_STATUS_OFF_TRACK ) )
		{
			off_track_on_grass = 1;
		}
	}

	printf( "grass drive: final pos=(%.1f, %.2f, %.1f) speed=%.2f min_y=%.2f off_track_on_grass=%d\n",
			(double)s->pos[0], (double)s->pos[1], (double)s->pos[2], (double)s->speed, (double)min_y,
			off_track_on_grass );

	CHECK( !error_seen, "no ERROR during the grass excursion" );
	CHECK( s->pos[0] > 55.0f, "car actually left the corridor (x past the skirt)" );
	CHECK( off_track_on_grass, "OFF_TRACK fires while driving on grass (grass laps stay invalidatable)" );
	// Solid landscape: the car sits on the grass plane at ground_y = −1.35,
	// chassis center ≈ −0.55 — it must NOT have fallen into the void.
	CHECK( min_y > -1.35f, "car never fell below the grass plane (no void fall)" );
	CHECK( s->pos[1] > -1.0f && s->pos[1] < 0.0f, "car settled ON the grass plane" );
	CHECK( s->speed > 1.0f, "car still rolling on grass (drivable, recoverable)" );

	free( blob );
}

int main( void )
{
	test_format();
	test_prop_collision();
	test_grass_query();
	test_grass_drive();

	if ( g_failures != 0 )
	{
		fprintf( stderr, "test_track_v2: %d failures\n", g_failures );
		return 1;
	}
	printf( "test_track_v2 PASS\n" );
	return 0;
}
