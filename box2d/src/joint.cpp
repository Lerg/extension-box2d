#include "joint.h"

Joint::Joint(b2World *world, b2Joint *joint) :
	world(NULL),
	joint(NULL) {
	this->world = world;
	this->joint = joint;
	joint->SetUserData(this);
}

Joint *Joint::get_userdata(lua_State *L) {
	Joint *joint = NULL;
	lua_getfield(L, 1, "__userdata");
	if (lua_islightuserdata(L, -1)) {
		joint = (Joint *)lua_touserdata(L, -1);
	}
	lua_pop(L, 1);
	return joint;
}

Joint *Joint::get_table_userdata(lua_State *L, const char *key, int index) {
	Joint *joint = NULL;
	lua_getfield(L, index, key);
	if (lua_istable(L, -1)) {
		lua_getfield(L, -1, "__userdata");
		if (lua_islightuserdata(L, -1)) {
			joint = (Joint *)lua_touserdata(L, -1);
		}
		lua_pop(L, 1);
	}
	lua_pop(L, 1);
	return joint;
}

int Joint::index(lua_State *L) {
	Joint *lua_joint = get_userdata(L);
	if (lua_joint == NULL) {
		return 0;
	}

	const char *key = lua_tostring(L, 2);
	switch (hash_string(key)) {
		default:
			lua_pushnil(L);
	}
	return 1;
}

int Joint::newindex(lua_State *L) {
	Joint *lua_joint = get_userdata(L);
	if (lua_joint == NULL) {
		return 0;
	}

	const char *key = lua_tostring(L, 2);
	const int value_index = 3;
	switch (hash_string(key)) {
		default:
			lua_rawset(L, 1);
	}
	return 0;
}

int Joint::destroy(lua_State *L) {
	utils::check_arg_count(L, 1);

	Joint *lua_joint = get_userdata(L);
	if (lua_joint == NULL) {
		return 0;
	}

	lua_joint->delete_lua_references(L);
	lua_joint->world->DestroyJoint(lua_joint->joint);
	delete lua_joint;

	return 0;
}

void Joint::delete_lua_references(lua_State *L) {
	utils::unref(L, lua_instance);
	lua_instance = LUA_REFNIL;
}

void Joint::push(lua_State *L) {
	// joint
	lua_createtable(L, 0, 1);

	lua_pushvalue(L, -1);
	lua_instance = luaL_ref(L, LUA_REGISTRYINDEX);

	// joint.__userdata
	lua_pushlightuserdata(L, this);
	lua_setfield(L, -2, "__userdata");

	// joint:destroy()
	lua_pushcfunction(L, destroy);
	lua_setfield(L, -2, "destroy");

	// joint's metatable
	lua_createtable(L, 0, 2);
	lua_pushcfunction(L, index);
	lua_setfield(L, -2, "__index");
	lua_pushcfunction(L, newindex);
	lua_setfield(L, -2, "__newindex");
	lua_setmetatable(L, -2);
}