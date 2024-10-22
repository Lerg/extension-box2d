#ifndef body_h
#define body_h

#include "extension.h"

enum PhysicsBody {
	PhysicsBodyDynamic,
	PhysicsBodyStatic,
	PhysicsBodyKinematic
};

class Body {
private:
	static Body *get_userdata(lua_State *L);
	b2World *world;
	static int index(lua_State *L);
	static int newindex(lua_State *L);
	static int new_body(lua_State *L);
	static int set_transform(lua_State *L);
	static int apply_force(lua_State *L);
	static int destroy(lua_State *L);
public:
	Body(b2World *world, b2Body *body);
	static Body *get_table_userdata(lua_State *L, const char *key, int index);
	double position_z;
	bool is_active;
	int lua_instance, lua_script_instance;
	int lua_on_enter_collision, lua_on_exit_collision, lua_on_before_collision, lua_on_after_collision;
	b2Body *body;
	void delete_lua_references(lua_State *L);
	void push(lua_State *L);
};

#endif