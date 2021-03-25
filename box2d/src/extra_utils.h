#ifndef extra_utils_h
#define extra_utils_h

#include "extension.h"

namespace extra_utils {
	void table_get_b2vec(lua_State *L, const char *key, b2Vec2 *vector);
}

#endif