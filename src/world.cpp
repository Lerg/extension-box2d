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

World *World::get_world_userdata(lua_State *L, int index) {
	World *lua_world = NULL;
	lua_getfield(L, index, "__userdata");
	if (lua_islightuserdata(L, -1)) {
		lua_world = (World *)lua_touserdata(L, -1);
	}
	lua_pop(L, 1);
	return lua_world;
}

int World::index(lua_State *L) {
	World *lua_world = get_world_userdata(L, 1);
	if (lua_world == NULL) {
		return 0;
	}

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
	World *lua_world = get_world_userdata(L, 1);
	if (lua_world == NULL) {
		return 0;
	}

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

	World *lua_world = get_world_userdata(L, 1);
	if (lua_world == NULL) {
		return 0;
	}

	b2BodyDef body_definition;
	body_definition.type = b2_dynamicBody;

	b2FixtureDef fixture_definition;
	
	utils::get_table(L, 2);

	int body_type = utils::table_get_integer(L, "type", -1);
	if (body_type != -1) {
		if (body_type == PhysicsBodyStatic) {
			body_definition.type = b2_staticBody;
		} else if (body_type == PhysicsBodyKinematic) {
			body_definition.type = b2_kinematicBody;
		}
	}

	body_definition.fixedRotation = utils::table_get_boolean(L, "is_fixed_rotation", false);

	double position_z = 0;
	Vectormath::Aos::Vector3 *position = utils::table_get_vector3(L, "position", NULL);
	if (position != NULL) {
		body_definition.position.x = position->getX();
		body_definition.position.y = position->getY();
		position_z = position->getZ();
	}

	fixture_definition.isSensor = utils::table_get_boolean(L, "is_sensor", false);
	fixture_definition.density = utils::table_get_double(L, "density", 1);
	fixture_definition.friction = utils::table_get_double(L, "friction", 0);
	fixture_definition.restitution = utils::table_get_double(L, "restitution", 0);

	double width = utils::table_get_double(L, "width", 0);
	double height = utils::table_get_double(L, "height", 0);
	double radius = utils::table_get_double(L, "radius", 0);
	if (utils::table_is_number(L, "category_bits")) {
		fixture_definition.filter.categoryBits = (uint16)utils::table_get_double(L, "category_bits", 0);
	}
	if (utils::table_is_number(L, "mask_bits")) {
		fixture_definition.filter.maskBits = (uint16)utils::table_get_double(L, "mask_bits", 0);
	}
	if (utils::table_is_number(L, "group_index")) {
		fixture_definition.filter.groupIndex = (int16)utils::table_get_double(L, "group_index", 0);
	}

	b2PolygonShape polygon_shape;
	b2CircleShape circle_shape;
	if (width > 0.0 && height > 0.0) {
		polygon_shape.SetAsBox(width / 2.0, height / 2.0);
		fixture_definition.shape = &polygon_shape;
	} else if (radius > 0.0) {
		circle_shape.m_radius = radius;
		fixture_definition.shape = &circle_shape;
	} else {
		luaL_error(L, "Body must have a shape.");
	}

	b2Body *body = lua_world->world->CreateBody(&body_definition);
	body->CreateFixture(&fixture_definition);

	Body *lua_body = new Body(lua_world->world, body);
	lua_body->position_z = position_z;

	dmScript::GetInstance(L);
	lua_body->lua_script_instance = dmScript::Ref(L, LUA_REGISTRYINDEX);
	lua_pop(L, 1);

	utils::table_get_functionp(L, "on_enter_collision", &lua_body->lua_on_enter_collision);
	utils::table_get_functionp(L, "on_exit_collision", &lua_body->lua_on_exit_collision);
	utils::table_get_functionp(L, "on_before_collision", &lua_body->lua_on_before_collision);
	utils::table_get_functionp(L, "on_after_collision", &lua_body->lua_on_after_collision);

	lua_pop(L, 1); // params.

	lua_body->push(L);
	return 1;
}

int World::new_joint(lua_State *L) {
	utils::check_arg_count(L, 2);

	World *lua_world = get_world_userdata(L, 1);
	if (lua_world == NULL) {
		return 0;
	}

	utils::get_table(L, 2);

	int joint_type = utils::table_get_integer(L, "type", -1);
	bool collide_connected = utils::table_get_boolean(L, "collide_connected", false);
	double length = utils::table_get_double(L, "length", 0);
	double frequency = utils::table_get_double(L, "frequency", 0);
	double damping = utils::table_get_double(L, "damping", 0);
	double max_force = utils::table_get_double(L, "max_force", 0);
	double max_torque = utils::table_get_double(L, "max_torque", 0);
	double angular_offset = utils::table_get_double(L, "angular_offset", 0);
	double ratio = utils::table_get_double(L, "ratio", 0);
	double correction_factor = utils::table_get_double(L, "correction_factor", 0);

	Body *body = Body::get_table_userdata(L, "body", -1);
	Body *other_body = Body::get_table_userdata(L, "other_body", -1);
	Joint *joint_param = Joint::get_table_userdata(L, "joint", -1);
	Joint *other_joint_param = Joint::get_table_userdata(L, "other_joint", -1);

	b2Vec2 anchor = b2Vec2_zero;
	Vectormath::Aos::Vector3 *v = utils::table_get_vector3(L, "anchor", NULL);
	if (v != NULL) {
		anchor.x = v->getX();
		anchor.y = v->getY();
	}

	b2Vec2 other_anchor = b2Vec2_zero;
	v = utils::table_get_vector3(L, "other_anchor", NULL);
	if (v != NULL) {
		other_anchor.x = v->getX();
		other_anchor.y = v->getY();
	}

	b2Vec2 linear_offset = b2Vec2_zero;
	v = utils::table_get_vector3(L, "linear_offset", NULL);
	if (v != NULL) {
		linear_offset.x = v->getX();
		linear_offset.y = v->getY();
	}

	b2Vec2 target = b2Vec2_zero;
	v = utils::table_get_vector3(L, "target", NULL);
	if (v != NULL) {
		target.x = v->getX();
		target.y = v->getY();
	}

	b2Vec2 axis = b2Vec2_zero;
	v = utils::table_get_vector3(L, "axis", NULL);
	if (v != NULL) {
		axis.x = v->getX();
		axis.y = v->getY();
	}

	lua_pop(L, 1); // params.

	switch (joint_type) {
		case PhysicsJointDistance:
		case PhysicsJointFriction:
		case PhysicsJointMotor:
		case PhysicsJointMouse:
		case PhysicsJointPrismatic:
		case PhysicsJointPulley:
		case PhysicsJointRevolute:
		case PhysicsJointRope:
		case PhysicsJointWeld:
		case PhysicsJointWheel:
			if (body == NULL) {
				luaL_error(L, "Incorrect `body`.");
				return 0;
			}
	}

	switch (joint_type) {
		case PhysicsJointDistance:
		case PhysicsJointFriction:
		case PhysicsJointMotor:
		case PhysicsJointPrismatic:
		case PhysicsJointPulley:
		case PhysicsJointRevolute:
		case PhysicsJointRope:
		case PhysicsJointWeld:
		case PhysicsJointWheel:
			if (other_body == NULL) {
				luaL_error(L, "Incorrect `other_body`.");
				return 0;
			}
	}

	if (joint_type == PhysicsJointGear) {
		if (joint_param == NULL) {
			luaL_error(L, "Incorrect `joint`.");
			return 0;
		}
		if (other_joint_param == NULL) {
			luaL_error(L, "Incorrect `other_joint`.");
			return 0;
		}
	}

	b2Joint *joint = NULL;

	switch (joint_type) {
		case PhysicsJointDistance: {
				b2DistanceJointDef joint_definition;
				joint_definition.Initialize(body->body, other_body->body, anchor, other_anchor);
				if (length > 0) {
					joint_definition.length = length;
				} else {
					joint_definition.length = b2Distance(body->body->GetPosition(), other_body->body->GetPosition());
				}
				joint_definition.frequencyHz = frequency;
				joint_definition.dampingRatio = damping;
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		case PhysicsJointFriction: {
				b2FrictionJointDef joint_definition;
				joint_definition.Initialize(body->body, other_body->body, anchor);
				joint_definition.maxForce = max_force;
				joint_definition.maxTorque = max_torque;
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		case PhysicsJointGear: {
				b2GearJointDef joint_definition;
				joint_definition.joint1 = joint_param->joint;
				joint_definition.joint2 = other_joint_param->joint;
				joint_definition.ratio = ratio;
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		case PhysicsJointMotor: {
				b2MotorJointDef joint_definition;
				joint_definition.Initialize(body->body, other_body->body);
				joint_definition.linearOffset = linear_offset;
				joint_definition.angularOffset = angular_offset;
				joint_definition.maxForce = max_force;
				joint_definition.maxTorque = max_torque;
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		case PhysicsJointMouse: {
				b2MouseJointDef joint_definition;
				joint_definition.target = target;
				joint_definition.maxForce = max_force;
				joint_definition.frequencyHz = frequency;
				joint_definition.dampingRatio = damping;
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		case PhysicsJointPrismatic: {
				b2PrismaticJointDef joint_definition;
				joint_definition.Initialize(body->body, other_body->body, anchor, axis);
				/*joint_definition.enableLimit = enable_limit;
				joint_definition.lowerTranslation = lower_translation;
				joint_definition.upperTranslation = upper_translation;
				joint_definition.enableMotor = enable_motor;
				joint_definition.maxMotorForce = max_force;
				joint_definition.motorSpeed = speed;*/
				joint = lua_world->world->CreateJoint(&joint_definition);
			}
			break;
		default:
			luaL_error(L, "Incorrect `type`.");
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

	World *lua_world = get_world_userdata(L, 1);
	if (lua_world == NULL) {
		return 0;
	}

	double time_step = lua_tonumber(L, 2);
	double velocity_iterations = lua_tonumber(L, 3);
	double position_iterations = lua_tonumber(L, 4);

	lua_world->world->Step((float)time_step, (int)velocity_iterations, (int)position_iterations);
	lua_world->dispatch_collision_events(L);

	return 0;
}

int World::destroy(lua_State *L) {
	utils::check_arg_count(L, 1);

	World *lua_world = get_world_userdata(L, 1);
	if (lua_world == NULL) {
		return 0;
	}

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