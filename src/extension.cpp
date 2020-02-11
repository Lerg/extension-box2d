#include "extension.h"
#include "world.h"

static int extension_init(lua_State *L) {
	utils::check_arg_count(L, 0);
	dmLogInfo("Box2D version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	return 0;
}

static int extension_new_world(lua_State *L) {
	utils::check_arg_count(L, 0, 1);
	b2Vec2 gravity(0.0f, -10.0f);

	if (lua_istable(L, 1)) {
		utils::get_table(L, 1);
		lua_getfield(L, -1, "gravity");
		if (lua_isuserdata(L, -1)) {
			lua_getfield(L, -1, "x");
			gravity.x = lua_tonumber(L, -1);
			lua_pop(L, 1);

			lua_getfield(L, -1, "y");
			gravity.y = lua_tonumber(L, -1);
			lua_pop(L, 1);
		}
		lua_pop(L, 1);
	}

	World *world = new World(gravity);
	world->push(L);
	return 1;
}

static const luaL_reg lua_functions[] = {
	{"init", extension_init},
	{"new_world", extension_new_world},
	{0, 0}
};

dmExtension::Result APP_INITIALIZE(dmExtension::AppParams *params) {
	return dmExtension::RESULT_OK;
}

dmExtension::Result APP_FINALIZE(dmExtension::AppParams *params) {
	return dmExtension::RESULT_OK;
}

dmExtension::Result INITIALIZE(dmExtension::Params *params) {
	lua_State *L = params->m_L;
	luaL_register(L, EXTENSION_NAME_STRING, lua_functions);

	lua_pushnumber(L, 1);
	lua_setfield(L, -2, "dynamic_body");

	lua_pushnumber(L, 2);
	lua_setfield(L, -2, "static_body");

	lua_pushnumber(L, 3);
	lua_setfield(L, -2, "kinematic_body");

	lua_pop(params->m_L, 1);

	return dmExtension::RESULT_OK;
}

dmExtension::Result UPDATE(dmExtension::Params *params) {
	return dmExtension::RESULT_OK;
}

void EXTENSION_ON_EVENT(dmExtension::Params *params, const dmExtension::Event *event) {
	switch (event->m_Event) {
		case dmExtension::EVENT_ID_ACTIVATEAPP:
			break;
		case dmExtension::EVENT_ID_DEACTIVATEAPP:
			break;
	}
}

dmExtension::Result FINALIZE(dmExtension::Params *params) {
	return dmExtension::RESULT_OK;
}

DM_DECLARE_EXTENSION(EXTENSION_NAME, EXTENSION_NAME_STRING, APP_INITIALIZE, APP_FINALIZE, INITIALIZE, UPDATE, EXTENSION_ON_EVENT, FINALIZE)