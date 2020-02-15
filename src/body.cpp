#include "body.h"

Body::Body(b2World *world, b2Body *body) :
	world(NULL),
	body(NULL),
	position_z(0),
	is_active(false),
	lua_script_instance(LUA_REFNIL),
	lua_on_enter_collision(LUA_REFNIL),
	lua_on_exit_collision(LUA_REFNIL),
	lua_on_before_collision(LUA_REFNIL),
	lua_on_after_collision(LUA_REFNIL) {
	this->world = world;
	this->body = body;
	body->SetUserData(this);
}

int Body::index(lua_State *L) {
	lua_getfield(L, 1, "__userdata");
	Body *lua_body = (Body *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	const char *key = lua_tostring(L, 2);
	switch (hash_string(key)) {
		case HASH_active:
			lua_pushboolean(L, lua_body->is_active);
			break;
		case HASH_angular_velocity:
			lua_pushnumber(L, lua_body->body->GetAngularVelocity());
			break;
		case HASH_linear_velocity: {
			b2Vec2 velocity = lua_body->body->GetLinearVelocity();
			utils::push_vector(L, velocity.x, velocity.y, 0);
			break;
		}
		case HASH_position: {
			b2Vec2 position = lua_body->body->GetPosition();
			utils::push_vector(L, position.x, position.y, lua_body->position_z);
			break;
		}
		case HASH_type:
			switch (lua_body->body->GetType()) {
				case b2_dynamicBody:
					lua_pushnumber(L, PhysicsBodyDynamic);
					break;
				case b2_staticBody:
					lua_pushnumber(L, PhysicsBodyStatic);
					break;
				case b2_kinematicBody:
					lua_pushnumber(L, PhysicsBodyKinematic);
					break;
				default:
					lua_pushnil(L);
			}
			break;
		case HASH_angle:
			lua_pushnumber(L, lua_body->body->GetAngle());
			break;
		default:
			lua_pushnil(L);
	}
	return 1;
}

int Body::newindex(lua_State *L) {
	lua_getfield(L, 1, "__userdata");
	Body *lua_body = (Body *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	const char *key = lua_tostring(L, 2);
	const int value_index = 3;
	switch (hash_string(key)) {
		case HASH_active:
			if (lua_isboolean(L, value_index)) {
				lua_body->is_active = lua_toboolean(L, value_index);
				lua_body->body->SetActive(lua_body->is_active);
			}			
			break;
		case HASH_angular_velocity:
			if (lua_isnumber(L, value_index)) {
				lua_body->body->SetAngularVelocity(lua_tonumber(L, value_index));
			}			
			break;
		case HASH_linear_velocity:
			if (lua_isuserdata(L, value_index)) {
				b2Vec2 linear_velocity;
				lua_getfield(L, value_index, "x");
				linear_velocity.x = lua_tonumber(L, -1);
				lua_pop(L, 1);

				lua_getfield(L, value_index, "y");
				linear_velocity.y = lua_tonumber(L, -1);
				lua_pop(L, 1);
				lua_body->body->SetLinearVelocity(linear_velocity);
			}			
			break;
		case HASH_position:
			if (lua_isuserdata(L, value_index)) {
				b2Vec2 position;
				lua_getfield(L, value_index, "x");
				position.x = lua_tonumber(L, -1);
				lua_pop(L, 1);

				lua_getfield(L, value_index, "y");
				position.y = lua_tonumber(L, -1);
				lua_pop(L, 1);

				lua_getfield(L, value_index, "z");
				lua_body->position_z = lua_tonumber(L, -1);
				lua_pop(L, 1);

				lua_body->body->SetTransform(position, lua_body->body->GetAngle());
			}
			break;
		case HASH_type:
			if (lua_isnumber(L, value_index)) {
				double body_type = lua_tonumber(L, -1);
				if (body_type == 1.0) {
					lua_body->body->SetType(b2_dynamicBody);
				} else if (body_type == 2.0) {
					lua_body->body->SetType(b2_staticBody);
				} else if (body_type == 3.0) {
					lua_body->body->SetType(b2_kinematicBody);
				}
			}
			break;
		default:
			lua_rawset(L, 1);
	}
	return 0;
}
// apply_force(force, [position])
int Body::apply_force(lua_State *L) {
	utils::check_arg_count(L, 2, 3);

	lua_getfield(L, 1, "__userdata");
	if (lua_type(L, -1) != LUA_TLIGHTUSERDATA) {
		return 0;
	}
	Body *lua_body = (Body *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	b2Vec2 force;
	if (lua_isuserdata(L, 2)) {
		lua_getfield(L, 2, "x");
		force.x = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_getfield(L, 2, "y");
		force.y = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}

	if (lua_isuserdata(L, 3)) {
		b2Vec2 position;
		lua_getfield(L, 3, "x");
		position.x = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_getfield(L, 3, "y");
		position.y = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_body->body->ApplyForce(force, position, true);
	} else {
		lua_body->body->ApplyForceToCenter(force, true);
	}

	return 0;
}

int Body::destroy(lua_State *L) {
	utils::check_arg_count(L, 1);

	lua_getfield(L, 1, "__userdata");
	if (lua_type(L, -1) != LUA_TLIGHTUSERDATA) {
		return 0;
	}
	Body *lua_body = (Body *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	lua_body->delete_lua_references(L);
	lua_body->world->DestroyBody(lua_body->body);
	delete lua_body;

	return 0;
}

void Body::delete_lua_references(lua_State *L) {
	utils::unref(L, lua_instance);
	utils::unref(L, lua_script_instance);
	utils::unref(L, lua_on_enter_collision);
	utils::unref(L, lua_on_exit_collision);
	utils::unref(L, lua_on_before_collision);
	utils::unref(L, lua_on_after_collision);
	lua_instance = LUA_REFNIL;
	lua_script_instance = LUA_REFNIL;
	lua_on_enter_collision = LUA_REFNIL;
	lua_on_exit_collision = LUA_REFNIL;
	lua_on_before_collision = LUA_REFNIL;
	lua_on_after_collision = LUA_REFNIL;
}

void Body::push(lua_State *L) {
	// body
	lua_createtable(L, 0, 1);

	lua_pushvalue(L, -1);
	lua_instance = luaL_ref(L, LUA_REGISTRYINDEX);

	// body.__userdata
	lua_pushlightuserdata(L, this);
	lua_setfield(L, -2, "__userdata");

	// body:apply_force()
	lua_pushcfunction(L, apply_force);
	lua_setfield(L, -2, "apply_force");

	// body:destroy()
	lua_pushcfunction(L, destroy);
	lua_setfield(L, -2, "destroy");

	// body's metatable
	lua_createtable(L, 0, 2);
	lua_pushcfunction(L, index);
	lua_setfield(L, -2, "__index");
	lua_pushcfunction(L, newindex);
	lua_setfield(L, -2, "__newindex");
	lua_setmetatable(L, -2);
}