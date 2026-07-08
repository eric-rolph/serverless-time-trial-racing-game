// vehicle.h — internal vehicle model interface.
// Quarter-car unsprung corners (docs/SUSPENSION.md) + discretized brush-model
// tires (docs/TIRE-MODEL.md) on a single Box3D rigid body.

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
	float steer_angle;		// rad, + = left per steering convention below (includes toe)
	float spin_angle;		// rad, wrapped to [-pi, pi]
	float omega;			// wheel angular speed (rad/s), + = rolling forward
	float compression;		// suspension SPRING compression (m), 0 = fully extended
	// Unsprung 1-DOF state (docs/SUSPENSION.md §1): wheel-center offset below
	// the hardpoint measured along the strut axis (-chassis up). travel =
	// rest_length at full droop, rest_length - max_travel at the bump stop.
	float travel;	  // m, + = down along the strut
	float travel_vel; // m/s, d(travel)/dt, + = extending toward droop
	float slip_ratio;
	float slip_angle;		// rad
	int in_contact;			// road within suspension reach this tick
	b3Vec3 contact_point;	// world
	b3Vec3 wheel_center;	// world (unsprung wheel center)
	float load;				// tire-spring Fz (N) fed to the brush model, 0 if airborne
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

// One 1600 Hz sub-step of the per-corner quarter-car (docs/SUSPENSION.md §1):
// tire vertical spring vs suspension spring/damper acting on the 22 kg
// unsprung mass, semi-implicit Euler on (travel, travel_vel), travel clamps.
//   hit_dist     = road contact distance below the hardpoint along the strut
//                  (pass something > rest_length + wheel_radius when airborne)
//   hit_dist_dot = its rate of change (chassis approaching road < 0)
//   g_along      = gravity component along strut-down (9.81 * up.y upright)
// Outputs the tire spring force (>= 0, the brush model's Fz) and the
// suspension strut force (+ = pushing chassis up / wheel down) BEFORE the
// integration, i.e. the forces acting across this sub-step. Exposed for
// tests/test_suspension.c; NOT a wasm export.
void vehicle_suspension_step( WheelRuntime* w, float hit_dist, float hit_dist_dot, float g_along, float dt,
							  float* out_tire_force, float* out_susp_force );

// Kinematic camber/toe from suspension spring compression (docs/SUSPENSION.md
// §2). wheel = 0..3 (FL FR RL RR). Outputs conventional signed camber (rad,
// negative = top of the wheel leaning toward the car centerline — same value
// both sides of an axle) and the toe contribution to THIS wheel's steer angle
// (rad, + = left; toe-in steers left wheels right and right wheels left).
// Exposed for tests/test_suspension.c; NOT a wasm export.
void vehicle_wheel_kinematics( int wheel, float compression, float* out_camber, float* out_steer_add );

// Camber thrust as equivalent lateral slip (docs/SUSPENSION.md §3):
// sigma_y contribution = k_camber_thrust * sin(gamma), gamma = wheel camber
// relative to the road contact normal, + = wheel top leaning LEFT (+side).
// Exposed for tests/test_suspension.c sweeps; NOT a wasm export.
float vehicle_camber_thrust_sigma( float gamma );

// Composite mass properties from the docs/SUSPENSION.md §4 mass layout:
// mass = SPRUNG mass (total minus 4 unsprung corners — vertical loads reach
// the ground through the tire springs), center = composite CoM, inertia =
// full-layout principal tensor about that CoM (the wheels yaw/pitch/roll with
// the chassis; only their vertical DOF is separate). Exposed for
// tests/test_suspension.c; NOT a wasm export.
b3MassData vehicle_mass_data( void );

#endif // SIM_VEHICLE_H
