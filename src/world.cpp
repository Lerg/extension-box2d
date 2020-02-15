#include "world.h"

World::World(b2Vec2 gravity) :
	world(NULL) {
	world = new b2World(gravity);
	world->SetContactListener(this);
}

World::~World() {
	if (world != NULL) {
		delete world;
		world = NULL;
	}
}

int World::index(lua_State *L) {
	lua_getfield(L, 1, "__userdata");
	World *lua_world = (World *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	const char *key = lua_tostring(L, 2);
	switch (hash_string(key)) {
		/*case HASH_x:
			lua_pushnumber(L, luaSource->x);
			break;
		*/
		default:
			lua_pushnil(L);
	}
	return 1;
}

int World::newindex(lua_State *L) {
	lua_getfield(L, 1, "__userdata");
	World *world = (World *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	const char *key = lua_tostring(L, 2);
	const int value_index = 3;
	switch (hash_string(key)) {
		/*case HASH_:
			break;
		*/
	}
	return 0;
}

int World::new_body(lua_State *L) {
	utils::check_arg_count(L, 2);

	lua_getfield(L, 1, "__userdata");
	if (!lua_islightuserdata(L, -1)) {
		return 0;
	}
	World *lua_world = (World *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	b2BodyDef body_definition;
	body_definition.type = b2_dynamicBody;

	b2FixtureDef fixture_definition;
	
	utils::get_table(L, 2);

	lua_getfield(L, -1, "type");
	if (lua_isnumber(L, -1)) {
		double body_type = lua_tonumber(L, -1);
		if (body_type == PhysicsBodyStatic) {
			body_definition.type = b2_staticBody;
		} else if (body_type == PhysicsBodyKinematic) {
			body_definition.type = b2_kinematicBody;
		}
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "is_fixed_rotation");
	if (lua_isboolean(L, -1)) {
		body_definition.fixedRotation = lua_toboolean(L, -1);
	}
	lua_pop(L, 1);

	double position_z = 0;

	lua_getfield(L, -1, "position");
	if (lua_isuserdata(L, -1)) {
		lua_getfield(L, -1, "x");
		body_definition.position.x = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_getfield(L, -1, "y");
		body_definition.position.y = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_getfield(L, -1, "z");
		position_z = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "is_sensor");
	if (lua_isboolean(L, -1)) {
		fixture_definition.isSensor = lua_toboolean(L, -1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "density");
	if (lua_isnumber(L, -1)) {
		fixture_definition.density = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "friction");
	if (lua_isnumber(L, -1)) {
		fixture_definition.friction = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "restitution");
	if (lua_isnumber(L, -1)) {
		fixture_definition.restitution = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	double width = 0;
	lua_getfield(L, -1, "width");
	if (lua_isnumber(L, -1)) {
		width = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	double height = 0;
	lua_getfield(L, -1, "height");
	if (lua_isnumber(L, -1)) {
		height = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	double radius = 0;
	lua_getfield(L, -1, "radius");
	if (lua_isnumber(L, -1)) {
		radius = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "category_bits");
	if (lua_isnumber(L, -1)) {
		fixture_definition.filter.categoryBits = (uint16)lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "mask_bits");
	if (lua_isnumber(L, -1)) {
		fixture_definition.filter.maskBits = (uint16)lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	lua_getfield(L, -1, "group_index");
	if (lua_isnumber(L, -1)) {
		fixture_definition.filter.groupIndex = (int16)lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	b2PolygonShape polygon_shape;
	b2CircleShape circle_shape;
	if (width > 0.0 && height > 0.0) {
		polygon_shape.SetAsBox(width / 2.0, height / 2.0);
		fixture_definition.shape = &polygon_shape;
	} else if (radius > 0.0) {
		circle_shape.m_radius = radius;
		fixture_definition.shape = &circle_shape;
	} else {
		dmLogError("Body must have a shape.");
	}

	b2Body *body = lua_world->world->CreateBody(&body_definition);
	body->CreateFixture(&fixture_definition);

	Body *lua_body = new Body(lua_world->world, body);
	lua_body->position_z = position_z;

	dmScript::GetInstance(L);
	lua_body->lua_script_instance = dmScript::Ref(L, LUA_REGISTRYINDEX);
	lua_pop(L, 1);

	utils::table_get_function(L, "on_enter_collision", &lua_body->lua_on_enter_collision);
	utils::table_get_function(L, "on_exit_collision", &lua_body->lua_on_exit_collision);
	utils::table_get_function(L, "on_before_collision", &lua_body->lua_on_before_collision);
	utils::table_get_function(L, "on_after_collision", &lua_body->lua_on_after_collision);

	lua_pop(L, 1); // params.

	lua_body->push(L);
	return 1;
}

int World::new_joint(lua_State *L) {
	utils::check_arg_count(L, 2);

	lua_getfield(L, 1, "__userdata");
	if (!lua_islightuserdata(L, -1)) {
		return 0;
	}
	World *lua_world = (World *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	utils::get_table(L, 2);

	int joint_type = PhysicsJointWeld;
	lua_getfield(L, -1, "type");
	if (lua_isnumber(L, -1)) {
		joint_type = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	Body *body = NULL;
	lua_getfield(L, -1, "body");
	if (lua_istable(L, -1)) {
		lua_getfield(L, -1, "__userdata");
		if (lua_islightuserdata(L, -1)) {
			body = (Body *)lua_touserdata(L, -1);
		}
		lua_pop(L, 1);
	}
	lua_pop(L, 1);

	Body *other_body = NULL;;
	lua_getfield(L, -1, "other_body");
	if (lua_istable(L, -1)) {
		lua_getfield(L, -1, "__userdata");
		if (lua_islightuserdata(L, -1)) {
			other_body = (Body *)lua_touserdata(L, -1);
		}
		lua_pop(L, 1);
	}
	lua_pop(L, 1);

	bool collide_connected = false;
	lua_getfield(L, -1, "collide_connected");
	if (lua_isboolean(L, -1)) {
		collide_connected = lua_toboolean(L, -1);
	}
	lua_pop(L, 1);

	b2Vec2 anchor = b2Vec2_zero;
	lua_getfield(L, -1, "anchor");
	if (lua_isuserdata(L, -1)) {
		lua_getfield(L, -1, "x");
		anchor.x = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_getfield(L, -1, "y");
		anchor.y = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
	lua_pop(L, 1);
 
	b2Vec2 other_anchor = b2Vec2_zero;
	lua_getfield(L, -1, "other_anchor");
	if (lua_isuserdata(L, -1)) {
		lua_getfield(L, -1, "x");
		other_anchor.x = lua_tonumber(L, -1);
		lua_pop(L, 1);

		lua_getfield(L, -1, "y");
		other_anchor.y = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
	lua_pop(L, 1);

	double length = 0;
	lua_getfield(L, -1, "length");
	if (lua_isnumber(L, -1)) {
		length = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	double frequency = 0;
	lua_getfield(L, -1, "frequency");
	if (lua_isnumber(L, -1)) {
		frequency = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	double damping = 0;
	lua_getfield(L, -1, "damping");
	if (lua_isnumber(L, -1)) {
		damping = lua_tonumber(L, -1);
	}
	lua_pop(L, 1);

	lua_pop(L, 1); // params.

	b2Joint *joint = NULL;

	if (body == NULL) {
		dmLogError("body should not be nil.");
		return 0;
	} else if (joint_type != PhysicsJointMouse && other_body == NULL) {
		dmLogError("other_body should not be nil.");
		return 0;
	}

	switch (joint_type) {
		case PhysicsJointDistance: {
				b2DistanceJointDef joint_definition;
				joint_definition.Initialize(body->body, other_body->body, anchor, other_anchor);
				if (length > 0) {
					joint_definition.length = length;
				} else {
					joint_definition.length = b2Distance(body->body->GetPosition(), other_body->body->GetPosition());
				}
				dmLogInfo("sdgf %f", joint_definition.length);
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		default:
			break;
	}
	
	if (joint != NULL) {
		Joint *lua_joint = new Joint(lua_world->world, joint);
		lua_joint->push(L);
		return 1;
	} else {
		return 0;
	}
}

int World::step(lua_State *L) {
	utils::check_arg_count(L, 4);

	lua_getfield(L, 1, "__userdata");
	if (lua_type(L, -1) != LUA_TLIGHTUSERDATA) {
		return 0;
	}
	World *lua_world = (World *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	double time_step = lua_tonumber(L, 2);
	double velocity_iterations = lua_tonumber(L, 3);
	double position_iterations = lua_tonumber(L, 4);

	lua_world->world->Step((float)time_step, (int)velocity_iterations, (int)position_iterations);
	lua_world->dispatch_collision_events(L);

	return 0;
}

int World::destroy(lua_State *L) {
	utils::check_arg_count(L, 1);

	lua_getfield(L, 1, "__userdata");
	if (lua_type(L, -1) != LUA_TLIGHTUSERDATA) {
		return 0;
	}
	World *lua_world = (World *)lua_touserdata(L, -1);
	lua_pop(L, 1);

	for (b2Body *body = lua_world->world->GetBodyList(); body; body = body->GetNext()) {
		Body *lua_body = (Body *)body->GetUserData();
		lua_body->delete_lua_references(L);
		delete lua_body;
	}

	delete lua_world;

	return 0;
}

void World::push(lua_State *L) {
	// world
	lua_createtable(L, 0, 1);
	// world.__userdata
	lua_pushlightuserdata(L, this);
	lua_setfield(L, -2, "__userdata");

	// world:new_body()
	lua_pushcfunction(L, new_body);
	lua_setfield(L, -2, "new_body");

	// world:new_joint()
	lua_pushcfunction(L, new_joint);
	lua_setfield(L, -2, "new_joint");

	// world:step()
	lua_pushcfunction(L, step);
	lua_setfield(L, -2, "step");

	// world:destroy()
	lua_pushcfunction(L, destroy);
	lua_setfield(L, -2, "destroy");

	// world's metatable
	lua_createtable(L, 0, 2);
	lua_pushcfunction(L, index);
	lua_setfield(L, -2, "__index");
	lua_pushcfunction(L, newindex);
	lua_setfield(L, -2, "__newindex");
	lua_setmetatable(L, -2);
}

void World::add_collision_event(CollisionEvent event) {
	collision_event_queue.push(event);
}

void World::dispatch_collision_events(lua_State *L) {
	while (!collision_event_queue.empty()) {
		CollisionEvent collision_event = collision_event_queue.front();
		if (collision_event.lua_listener == LUA_REFNIL) {
			return;
		}
		lua_rawgeti(L, LUA_REGISTRYINDEX, collision_event.lua_listener);
		lua_rawgeti(L, LUA_REGISTRYINDEX, collision_event.lua_script_instance);
		dmScript::SetInstance(L);

		lua_newtable(L);
		lua_rawgeti(L, LUA_REGISTRYINDEX, collision_event.body->lua_instance);
		lua_setfield(L, -2, "self");
		lua_rawgeti(L, LUA_REGISTRYINDEX, collision_event.other_body->lua_instance);
		lua_setfield(L, -2, "other");
		lua_call(L, 1, 0);

		collision_event_queue.pop();
	}
}

void World::on_enter_collision(b2Contact *contact, bool is_first) {
	Body *body;
	Body *other_body;
	if (is_first) {
		body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
	} else {
		body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
	}
	if (body->lua_on_enter_collision != LUA_REFNIL) {
		CollisionEvent event = {
			.body = body,
			.other_body = other_body,
			.lua_listener = body->lua_on_enter_collision,
			.lua_script_instance = body->lua_script_instance
		};
		add_collision_event(event);
	}
}

void World::on_exit_collision(b2Contact *contact, bool is_first) {
	Body *body;
	Body *other_body;
	if (is_first) {
		body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
	} else {
		body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
	}
	if (body->lua_on_exit_collision != LUA_REFNIL) {
		CollisionEvent event = {
			.body = body,
			.other_body = other_body,
			.lua_listener = body->lua_on_exit_collision,
			.lua_script_instance = body->lua_script_instance
		};
		add_collision_event(event);
	}
}

void World::on_before_collision(b2Contact *contact, bool is_first) {
	Body *body;
	Body *other_body;
	if (is_first) {
		body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
	} else {
		body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
	}
	if (body->lua_on_before_collision != LUA_REFNIL) {
		CollisionEvent event = {
			.body = body,
			.other_body = other_body,
			.lua_listener = body->lua_on_before_collision,
			.lua_script_instance = body->lua_script_instance
		};
		add_collision_event(event);
	}
}

void World::on_after_collision(b2Contact *contact, bool is_first) {
	Body *body;
	Body *other_body;
	if (is_first) {
		body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
	} else {
		body = (Body *)contact->GetFixtureB()->GetBody()->GetUserData();
		other_body = (Body *)contact->GetFixtureA()->GetBody()->GetUserData();
	}
	if (body->lua_on_after_collision != LUA_REFNIL) {
		CollisionEvent event = {
			.body = body,
			.other_body = other_body,
			.lua_listener = body->lua_on_after_collision,
			.lua_script_instance = body->lua_script_instance
		};
		add_collision_event(event);
	}
}

// b2ContactListener
void World::BeginContact(b2Contact *contact) {
	on_enter_collision(contact, true);
	on_enter_collision(contact, false);
}

void World::EndContact(b2Contact *contact) {
	on_exit_collision(contact, true);
	on_exit_collision(contact, false);
}

void World::PreSolve(b2Contact *contact, const b2Manifold *old_manifold) {
	on_before_collision(contact, true);
	on_before_collision(contact, false);
}

void World::PostSolve(b2Contact* contact, const b2ContactImpulse *impulse) {
	on_after_collision(contact, true);
	on_after_collision(contact, false);
}