#ifndef world_h
#define world_h

#include <queue>

#include "extension.h"
#include "body.h"
#include "joint.h"
#include "extra_utils.h"

struct CollisionEvent {
	Body *body;
	Body *other_body;
	int lua_listener;
	int lua_script_instance;
};

class World : public b2ContactListener {
private:
	static World *get_userdata(lua_State *L, int index);
	std::queue<CollisionEvent> collision_event_queue;
	static int index(lua_State *L);
	static int newindex(lua_State *L);
	static int new_body(lua_State *L);
	static int new_joint(lua_State *L);
	static int step(lua_State *L);
	static int destroy(lua_State *L);
	void add_collision_event(CollisionEvent event);
public:
	b2World *world;
	World(b2Vec2 gravity);
	~World();
	void push(lua_State *L);
	void dispatch_collision_events(lua_State *L);

	void on_enter_collision(b2Contact *contact, bool is_first);
	void on_exit_collision(b2Contact *contact, bool is_first);
	void on_before_collision(b2Contact *contact, bool is_first);
	void on_after_collision(b2Contact *contact, bool is_first);
	// b2ContactListener
	void BeginContact(b2Contact *contact);
	void EndContact(b2Contact *contact);
	void PreSolve(b2Contact *contact, const b2Manifold *old_manifold);
	void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse);
};

#endif