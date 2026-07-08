// vehicle.h — internal vehicle model interface.
// Raycast-suspension + discretized brush-model tires (docs/TIRE-MODEL.md)
// on a single Box3D rigid body.

#ifndef SIM_VEHICLE_H
#define SIM_VEHICLE_H

#include "box3d/box3d.h"
#include "road.h"
#include "sim/sim.h"

// Collision filter categories
#define SIM_CAT_TERRAIN 0x0001ull
#define SIM_CAT_CHASSIS 0x0002ull

typedef struct WheelRuntime
{
	float steer_angle;		// rad, + = left per steering convention below
	float spin_angle;		// rad, wrapped to [-pi, pi]
	float omega;			// wheel angular speed (rad/s), + = rolling forward
	float compression;		// suspension compression (m), 0 = fully extended
	float prev_compression; // previous tick, for damper velocity
	float slip_ratio;
	float slip_angle;		// rad
	int in_contact;			// raycast hit this tick
	b3Vec3 contact_point;	// world
	b3Vec3 wheel_center;	// world
	float load;				// Fz (N) this tick, 0 if airborne
	// Tire thermal state (docs/TIRE-MODEL.md §2). Internal only — never
	// exported into SimStateV1 and never hashed directly (it feeds grip,
	// which feeds the hashed dynamics). Reset to ambient (25 C) with the car.
	float t_surf; // tread surface temperature (deg C)
	float t_core; // carcass core temperature (deg C)
	// Incremental analytic-road hint (docs/ROAD-SURFACE.md §1): last segment
	// index for the ±6 window search. Bootstrapped by a full deterministic
	// scan on the first query after create/reset (road_hint_valid == 0).
	uint32_t road_hint;
	int road_hint_valid;
} WheelRuntime;

typedef struct Vehicle
{
	b3BodyId chassis;
	WheelRuntime wheels[SIM_WHEEL_COUNT]; // 0=FL 1=FR 2=RL 3=RR
	// Steering rack torque (Nm at the rim) for force feedback. Pure output —
	// never feeds back into the simulation, so it has no replay/hash impact.
	float rack_torque;
	int valid;
} Vehicle;

// Create the chassis body and initialize wheel runtime. pos/yaw = spawn pose.
void vehicle_create( b3WorldId world, Vehicle* v, b3Vec3 pos, float yaw );

// Destroy the chassis body (world teardown destroys shapes with it).
void vehicle_destroy( Vehicle* v );

// Teleport back to spawn pose and zero all dynamic state.
void vehicle_reset( b3WorldId world, Vehicle* v, b3Vec3 pos, float yaw );

// Per-tick force computation, before b3World_Step. Inputs are unquantized
// floats: steer [-1,1], throttle [0,1], brake [0,1], flags bitfield.
// `road` is the analytic surface the wheels contact (docs/ROAD-SURFACE.md);
// wheels fall back to the mesh raycast when outside its domain.
void vehicle_update( b3WorldId world, Vehicle* v, const Road* road, float steer, float throttle, float brake,
					 uint32_t flags );

// Write per-wheel state into the packed SimStateV1 block.
void vehicle_export( const Vehicle* v, SimStateV1* state );

// Brush contact-patch evaluation (docs/TIRE-MODEL.md §1) — the per-tire force
// model used inside vehicle_update, exposed for tests/test_tire.c sweeps.
// Internal linkage only: NOT in the wasm EXPORTED_FUNCTIONS list, so the
// CONTRACTS §1.1 export surface is unchanged.
//   sigma_x = slip ratio, sigma_y = tan(slip angle), fz = load (N),
//   t_surf = tread surface temperature (deg C).
// Outputs: fx (N, wheel forward), fy (N, wheel side/left), trail (m, emergent
// pneumatic trail: lateral-force centroid distance behind the patch center).
void vehicle_brush_patch( float sigma_x, float sigma_y, float fz, float t_surf, float* out_fx, float* out_fy,
						  float* out_trail );

// Two-node tire thermal step (docs/TIRE-MODEL.md §2 + ROAD-SURFACE.md §2):
// friction power into the surface, speed-scaled convection, surface↔core
// exchange, plus track conduction while in contact. Advances w->t_surf and
// w->t_core by dt. Exposed (like vehicle_brush_patch) for tests/test_road.c's
// lockup flash-heat gate; NOT a wasm export.
void vehicle_tire_thermal( WheelRuntime* w, float power, float speed, int in_contact, float dt );

#endif // SIM_VEHICLE_H
