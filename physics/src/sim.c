// sim.c — deterministic kernel: world lifecycle, TRK1 parsing, lap logic,
// state export, FNV-1a-64 state hash. See docs/CONTRACTS.md §1, §8 (binding).

#include "sim/sim.h"
#include "vehicle.h"

#include "box3d/box3d.h"

#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Track data (parsed TRK1, CONTRACTS §8)
// ---------------------------------------------------------------------------

typedef struct CenterlineSample
{
	b3Vec3 pos;
	b3Vec3 up;
	b3Vec3 tangent;
	float width;
} CenterlineSample;

typedef struct Track
{
	CenterlineSample* samples;
	uint32_t sample_count;
	uint32_t* checkpoints; // ascending centerline indices
	uint32_t checkpoint_count;

	// Geometry retained for the lifetime of the mesh shape (Box3D keeps a
	// reference to b3MeshData; the def arrays below are cloned by b3CreateMesh).
	b3MeshData* mesh;

	uint32_t spawn_index;
	float spawn_yaw;
} Track;

typedef struct Sim
{
	b3WorldId world;
	int world_valid;
	Track track;
	int track_valid;
	Vehicle vehicle;

	SimStateV1 state;

	// Lap tracking
	uint32_t nearest_idx;	// centerline sample nearest to the car
	uint32_t lap_start_tick;
	uint32_t lap_invalid;	// sticky until next start-line crossing
	uint32_t lap_time_ticks;
	uint32_t last_status;
} Sim;

static Sim g_sim; // zero-initialized: world_valid = 0, track_valid = 0

// Incremental nearest-centerline search window (samples each direction).
#define SIM_SEARCH_WINDOW 12

// ---------------------------------------------------------------------------
// FNV-1a-64 (CONTRACTS §1.4)
// ---------------------------------------------------------------------------

#define FNV_OFFSET 0xcbf29ce484222325ull
#define FNV_PRIME 0x00000100000001b3ull

static uint64_t sFnv1a64( uint64_t h, const void* data, size_t len )
{
	const unsigned char* p = (const unsigned char*)data;
	for ( size_t i = 0; i < len; ++i )
	{
		h ^= (uint64_t)p[i];
		h *= FNV_PRIME;
	}
	return h;
}

// Hash exactly: tick, chassis pos/quat/vel/angvel, all wheel state, checkpoint
// mask. In SimStateV1 the first group is contiguous: bytes [0, 184).
static uint64_t sStateHash( const SimStateV1* s )
{
	uint64_t h = FNV_OFFSET;
	h = sFnv1a64( h, s, 184 );
	h = sFnv1a64( h, &s->checkpoint_mask, 4 );
	return h;
}

// ---------------------------------------------------------------------------
// TRK1 parsing — IEEE bit-pattern reads via memcpy, no text parsing.
// ---------------------------------------------------------------------------

typedef struct Reader
{
	const unsigned char* p;
	size_t len;
	size_t off;
	int fail;
} Reader;

static uint16_t sReadU16( Reader* r )
{
	uint16_t v = 0;
	if ( r->off + 2 > r->len )
	{
		r->fail = 1;
		return 0;
	}
	memcpy( &v, r->p + r->off, 2 );
	r->off += 2;
	return v;
}

static uint32_t sReadU32( Reader* r )
{
	uint32_t v = 0;
	if ( r->off + 4 > r->len )
	{
		r->fail = 1;
		return 0;
	}
	memcpy( &v, r->p + r->off, 4 );
	r->off += 4;
	return v;
}

static uint64_t sReadU64( Reader* r )
{
	uint64_t v = 0;
	if ( r->off + 8 > r->len )
	{
		r->fail = 1;
		return 0;
	}
	memcpy( &v, r->p + r->off, 8 );
	r->off += 8;
	return v;
}

static float sReadF32( Reader* r )
{
	uint32_t bits = sReadU32( r );
	float f;
	memcpy( &f, &bits, 4 );
	return f;
}

static void sFreeTrack( Track* t )
{
	free( t->samples );
	free( t->checkpoints );
	if ( t->mesh != NULL )
	{
		b3DestroyMesh( t->mesh );
	}
	memset( t, 0, sizeof( *t ) );
}

static void sTeardownWorld( void )
{
	if ( g_sim.world_valid )
	{
		vehicle_destroy( &g_sim.vehicle );
		b3DestroyWorld( g_sim.world );
		g_sim.world_valid = 0;
	}
	if ( g_sim.track_valid )
	{
		sFreeTrack( &g_sim.track );
		g_sim.track_valid = 0;
	}
}

int32_t sim_load_track( uint32_t ptr, uint32_t len )
{
	sTeardownWorld();
	memset( &g_sim.state, 0, sizeof( g_sim.state ) );
	g_sim.last_status = SIM_STATUS_ERROR;

	Reader r = { .p = (const unsigned char*)(uintptr_t)ptr, .len = len, .off = 0, .fail = 0 };

	// Header
	if ( len < 28 )
	{
		return SIM_ERR_TRUNCATED;
	}
	uint32_t magic = sReadU32( &r );
	if ( magic != 0x4B525453u ) // "STRK" little-endian: S,T,R,K
	{
		return SIM_ERR_BAD_MAGIC;
	}
	uint16_t version = sReadU16( &r );
	if ( version != 1 )
	{
		return SIM_ERR_BAD_VERSION;
	}
	uint16_t checkpoint_count = sReadU16( &r );
	(void)sReadU64( &r ); // seed (provenance only)
	uint32_t sample_count = sReadU32( &r );
	uint32_t vertex_count = sReadU32( &r );
	uint32_t triangle_count = sReadU32( &r );

	if ( sample_count < 8 || sample_count > 1000000 || vertex_count < 3 || vertex_count > 4000000 ||
		 triangle_count < 1 || triangle_count > 8000000 || checkpoint_count < 1 || checkpoint_count > 32 )
	{
		return SIM_ERR_BAD_COUNTS;
	}

	// Total size check before allocating
	size_t need = 28 + (size_t)40 * sample_count + (size_t)4 * checkpoint_count + (size_t)12 * vertex_count +
				  (size_t)12 * triangle_count + 12;
	if ( (size_t)len < need )
	{
		return SIM_ERR_TRUNCATED;
	}

	Track* t = &g_sim.track;
	t->samples = (CenterlineSample*)malloc( sizeof( CenterlineSample ) * sample_count );
	t->checkpoints = (uint32_t*)malloc( sizeof( uint32_t ) * checkpoint_count );
	b3Vec3* verts = (b3Vec3*)malloc( sizeof( b3Vec3 ) * vertex_count );
	int32_t* indices = (int32_t*)malloc( sizeof( int32_t ) * 3 * (size_t)triangle_count );
	if ( t->samples == NULL || t->checkpoints == NULL || verts == NULL || indices == NULL )
	{
		free( verts );
		free( indices );
		sFreeTrack( t );
		return SIM_ERR_ALLOC;
	}
	t->sample_count = sample_count;
	t->checkpoint_count = checkpoint_count;

	for ( uint32_t i = 0; i < sample_count; ++i )
	{
		CenterlineSample* s = &t->samples[i];
		s->pos.x = sReadF32( &r );
		s->pos.y = sReadF32( &r );
		s->pos.z = sReadF32( &r );
		s->up.x = sReadF32( &r );
		s->up.y = sReadF32( &r );
		s->up.z = sReadF32( &r );
		s->tangent.x = sReadF32( &r );
		s->tangent.y = sReadF32( &r );
		s->tangent.z = sReadF32( &r );
		s->width = sReadF32( &r );
	}

	for ( uint32_t i = 0; i < checkpoint_count; ++i )
	{
		t->checkpoints[i] = sReadU32( &r );
		if ( t->checkpoints[i] >= sample_count || ( i > 0 && t->checkpoints[i] <= t->checkpoints[i - 1] ) )
		{
			free( verts );
			free( indices );
			sFreeTrack( t );
			return SIM_ERR_BAD_COUNTS;
		}
	}

	for ( uint32_t i = 0; i < vertex_count; ++i )
	{
		verts[i].x = sReadF32( &r );
		verts[i].y = sReadF32( &r );
		verts[i].z = sReadF32( &r );
	}

	for ( uint32_t i = 0; i < 3 * triangle_count; ++i )
	{
		uint32_t idx = sReadU32( &r );
		if ( idx >= vertex_count )
		{
			free( verts );
			free( indices );
			sFreeTrack( t );
			return SIM_ERR_BAD_GEOMETRY;
		}
		indices[i] = (int32_t)idx;
	}

	t->spawn_index = sReadU32( &r );
	t->spawn_yaw = sReadF32( &r );
	(void)sReadF32( &r ); // reserved

	if ( r.fail || t->spawn_index >= sample_count )
	{
		free( verts );
		free( indices );
		sFreeTrack( t );
		return SIM_ERR_TRUNCATED;
	}

	// --- Build the collision mesh (static triangle soup, mapping decision in NOTES.md) ---
	b3MeshDef meshDef = { 0 };
	meshDef.vertices = verts;
	meshDef.indices = indices;
	meshDef.vertexCount = (int)vertex_count;
	meshDef.triangleCount = (int)triangle_count;
	meshDef.weldVertices = false;	// trackgen emits indexed triangles; welding would be redundant
	meshDef.useMedianSplit = false; // SAH build, deterministic for fixed input
	meshDef.identifyEdges = true;	// adjacency info avoids internal-edge catching

	t->mesh = b3CreateMesh( &meshDef, NULL, 0 );

	// b3CreateMesh clones vertex/index data into the b3MeshData block.
	free( verts );
	free( indices );

	if ( t->mesh == NULL )
	{
		sFreeTrack( t );
		return SIM_ERR_BAD_GEOMETRY;
	}
	g_sim.track_valid = 1;

	// --- World ---
	b3WorldDef worldDef = b3DefaultWorldDef();
	worldDef.gravity = ( b3Vec3 ){ 0.0f, -9.81f, 0.0f };
	worldDef.enableSleep = false; // the car must always accept forces
	// workerCount = 0 and no task callbacks → Box3D runs strictly
	// single-threaded on the calling thread (no internal scheduler).
	worldDef.workerCount = 0;
	worldDef.enqueueTask = NULL;
	worldDef.finishTask = NULL;

	g_sim.world = b3CreateWorld( &worldDef );
	g_sim.world_valid = 1;

	b3BodyDef groundDef = b3DefaultBodyDef();
	groundDef.type = b3_staticBody;
	groundDef.name = "terrain";
	b3BodyId ground = b3CreateBody( g_sim.world, &groundDef );

	b3ShapeDef groundShape = b3DefaultShapeDef();
	groundShape.baseMaterial.friction = 1.0f;
	groundShape.baseMaterial.restitution = 0.0f;
	groundShape.filter.categoryBits = SIM_CAT_TERRAIN;
	groundShape.filter.maskBits = ~0ull;
	b3CreateMeshShape( ground, &groundShape, t->mesh, b3Vec3_one );

	// --- Vehicle at spawn ---
	CenterlineSample* spawn = &t->samples[t->spawn_index];
	b3Vec3 spawn_pos = b3MulAdd( spawn->pos, 0.9f, spawn->up );
	vehicle_create( g_sim.world, &g_sim.vehicle, spawn_pos, t->spawn_yaw );

	sim_reset();
	return 0;
}

// ---------------------------------------------------------------------------
// Lap logic — incremental nearest-centerline tracking, deterministic.
// ---------------------------------------------------------------------------

static float sDistSqToSample( b3Vec3 p, const CenterlineSample* s )
{
	b3Vec3 d = b3Sub( p, s->pos );
	return b3Dot( d, d );
}

// Search a fixed window around the previous nearest index; fixed iteration
// order and strict '<' make the result deterministic.
static uint32_t sFindNearest( const Track* t, b3Vec3 p, uint32_t last )
{
	uint32_t n = t->sample_count;
	uint32_t best = last;
	float best_d = sDistSqToSample( p, &t->samples[last] );
	for ( int k = -SIM_SEARCH_WINDOW; k <= SIM_SEARCH_WINDOW; ++k )
	{
		if ( k == 0 )
		{
			continue;
		}
		uint32_t idx = (uint32_t)( ( (int)last + k + (int)n ) % (int)n );
		float d = sDistSqToSample( p, &t->samples[idx] );
		if ( d < best_d )
		{
			best_d = d;
			best = idx;
		}
	}
	return best;
}

static void sUpdateLap( uint32_t* status )
{
	Sim* sim = &g_sim;
	const Track* t = &sim->track;
	uint32_t n = t->sample_count;

	b3Pos body_pos = b3Body_GetPosition( sim->vehicle.chassis );
	b3Vec3 p = { body_pos.x, body_pos.y, body_pos.z };

	uint32_t prev_idx = sim->nearest_idx;
	uint32_t idx = sFindNearest( t, p, prev_idx );
	sim->nearest_idx = idx;

	const CenterlineSample* s = &t->samples[idx];

	// Off-track: lateral distance from the centerline exceeds half width.
	b3Vec3 rel = b3Sub( p, s->pos );
	b3Vec3 side = b3Cross( s->up, s->tangent );
	float lateral = b3Dot( rel, side );
	int off_track = b3AbsFloat( lateral ) > 0.5f * s->width;
	if ( off_track )
	{
		*status |= SIM_STATUS_OFF_TRACK;
	}

	// Continuous lap progress: sample index plus projection onto the tangent.
	float along = b3Dot( rel, s->tangent );
	uint32_t next = ( idx + 1 == n ) ? 0 : idx + 1;
	float seg = b3Distance( t->samples[next].pos, s->pos );
	float frac = seg > 0.0001f ? b3ClampFloat( along / seg, 0.0f, 0.999f ) : 0.0f;
	sim->state.lap_progress = ( (float)idx + frac ) / (float)n;

	// Checkpoints: award checkpoint k when the nearest index reaches its
	// sample (within the search window step) while on track, in order.
	// Reaching a later checkpoint first = corner cut → LAP_INVALID.
	uint32_t mask = sim->state.checkpoint_mask;
	for ( uint32_t k = 0; k < t->checkpoint_count; ++k )
	{
		if ( mask & ( 1u << k ) )
		{
			continue;
		}
		uint32_t cp = t->checkpoints[k];
		// Did we advance across cp this tick (forward direction)?
		uint32_t from = prev_idx;
		uint32_t to = idx;
		int crossed;
		if ( from <= to )
		{
			crossed = ( from < cp || from == cp ) && cp <= to;
		}
		else
		{
			// wrapped past the start line this tick
			crossed = cp >= from || cp <= to;
		}
		if ( crossed )
		{
			if ( off_track )
			{
				// Passed the checkpoint region while off the drivable surface.
				sim->lap_invalid = 1;
				*status |= SIM_STATUS_LAP_INVALID;
			}
			else
			{
				// Order check: all lower checkpoints must already be set.
				uint32_t lower = ( 1u << k ) - 1u;
				if ( ( mask & lower ) != lower )
				{
					sim->lap_invalid = 1;
					*status |= SIM_STATUS_LAP_INVALID;
				}
				mask |= ( 1u << k );
			}
		}
	}
	sim->state.checkpoint_mask = mask;

	// Start-line crossing: nearest index wraps from the last quarter to the
	// first quarter in the forward direction.
	int wrapped_fwd = prev_idx > ( 3 * n ) / 4 && idx < n / 4;
	int wrapped_back = idx > ( 3 * n ) / 4 && prev_idx < n / 4;

	if ( wrapped_fwd )
	{
		uint32_t all = ( 1u << t->checkpoint_count ) - 1u;
		if ( mask == all && !sim->lap_invalid )
		{
			sim->lap_time_ticks = sim->state.tick - sim->lap_start_tick;
			sim->state.lap_time_ticks = sim->lap_time_ticks;
			*status |= SIM_STATUS_LAP_COMPLETE;
		}
		else if ( mask != 0 || sim->lap_invalid )
		{
			*status |= SIM_STATUS_LAP_INVALID;
		}
		// New lap begins either way.
		sim->state.checkpoint_mask = 0;
		sim->lap_invalid = 0;
		sim->lap_start_tick = sim->state.tick;
	}
	else if ( wrapped_back )
	{
		// Reversed across the start line: restart lap tracking.
		sim->state.checkpoint_mask = 0;
		sim->lap_invalid = 0;
		sim->lap_start_tick = sim->state.tick;
	}

	if ( sim->lap_invalid )
	{
		*status |= SIM_STATUS_LAP_INVALID;
	}
}

// ---------------------------------------------------------------------------
// State export
// ---------------------------------------------------------------------------

static void sExportState( void )
{
	Sim* sim = &g_sim;
	SimStateV1* st = &sim->state;

	b3WorldTransform xf = b3Body_GetTransform( sim->vehicle.chassis );
	b3Vec3 lv = b3Body_GetLinearVelocity( sim->vehicle.chassis );
	b3Vec3 av = b3Body_GetAngularVelocity( sim->vehicle.chassis );

	st->pos[0] = xf.p.x;
	st->pos[1] = xf.p.y;
	st->pos[2] = xf.p.z;
	st->quat[0] = xf.q.v.x;
	st->quat[1] = xf.q.v.y;
	st->quat[2] = xf.q.v.z;
	st->quat[3] = xf.q.s;
	st->lin_vel[0] = lv.x;
	st->lin_vel[1] = lv.y;
	st->lin_vel[2] = lv.z;
	st->ang_vel[0] = av.x;
	st->ang_vel[1] = av.y;
	st->ang_vel[2] = av.z;
	st->speed = b3Length( lv );

	vehicle_export( &sim->vehicle, st );
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void sim_reset( void )
{
	Sim* sim = &g_sim;
	if ( !sim->world_valid || !sim->track_valid )
	{
		return;
	}

	const Track* t = &sim->track;
	const CenterlineSample* spawn = &t->samples[t->spawn_index];
	b3Vec3 spawn_pos = b3MulAdd( spawn->pos, 0.9f, spawn->up );
	vehicle_reset( sim->world, &sim->vehicle, spawn_pos, t->spawn_yaw );

	memset( &sim->state, 0, sizeof( sim->state ) );
	sim->nearest_idx = t->spawn_index;
	sim->lap_start_tick = 0;
	sim->lap_invalid = 0;
	sim->lap_time_ticks = 0;
	sim->last_status = SIM_STATUS_RUNNING;

	sExportState();
}

uint32_t sim_step( int32_t steer, uint32_t throttle, uint32_t brake, uint32_t flags )
{
	Sim* sim = &g_sim;
	if ( !sim->world_valid || !sim->track_valid )
	{
		return SIM_STATUS_ERROR;
	}

	// Quantized inputs → floats (CONTRACTS §2). The integers are canonical.
	int32_t s = steer;
	if ( s < -32767 )
	{
		s = -32767;
	}
	if ( s > 32767 )
	{
		s = 32767;
	}
	uint32_t th = throttle > 65535u ? 65535u : throttle;
	uint32_t br = brake > 65535u ? 65535u : brake;

	float steer_f = (float)s / 32767.0f;
	float throttle_f = (float)th / 65535.0f;
	float brake_f = (float)br / 65535.0f;

	vehicle_update( sim->world, &sim->vehicle, steer_f, throttle_f, brake_f, flags );
	b3World_Step( sim->world, SIM_DT, 4 );

	sim->state.tick += 1;

	uint32_t status = SIM_STATUS_RUNNING;
	sUpdateLap( &status );
	sExportState();

	sim->last_status = status;
	return status;
}

uint32_t sim_state_ptr( void )
{
	return (uint32_t)(uintptr_t)&g_sim.state;
}

uint32_t sim_state_size( void )
{
	return (uint32_t)sizeof( SimStateV1 );
}

uint32_t sim_state_hash_lo( void )
{
	return (uint32_t)( sStateHash( &g_sim.state ) & 0xFFFFFFFFull );
}

uint32_t sim_state_hash_hi( void )
{
	return (uint32_t)( sStateHash( &g_sim.state ) >> 32 );
}

uint32_t sim_lap_time_ticks( void )
{
	return g_sim.lap_time_ticks;
}

const SimStateV1* sim_state( void )
{
	return &g_sim.state;
}
