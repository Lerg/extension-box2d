#ifndef joint_h
#define joint_h

#include "extension.h"

enum PhysicsJoint {
	PhysicsJointDistance,
	PhysicsJointFriction,
	PhysicsJointGear,
	PhysicsJointMotor,
	PhysicsJointMouse,
	PhysicsJointPrismatic,
	PhysicsJointPulley,
	PhysicsJointRevolute,
	PhysicsJointRope,
	PhysicsJointWeld,
	PhysicsJointWheel
};

class Joint {
private:
	b2World *world;
	static int index(lua_State *L);
	static int newindex(lua_State *L);
	static int destroy(lua_State *L);
public:
	Joint(b2World *world, b2Joint *joint);
	static Joint *get_table_userdata(lua_State *L, const char *key, int index);
	b2Joint *joint;
	int lua_instance;
	void delete_lua_references(lua_State *L);
	void push(lua_State *L);
};

#endif