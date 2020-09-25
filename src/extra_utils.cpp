#include "extra_utils.h"

namespace extra_utils {
	void table_get_b2vec(lua_State *L, const char *key, b2Vec2 *vector) {
		Vectormath::Aos::Vector3 *v = utils::table_get_vector3(L, key, NULL);
		if (v != NULL) {
			vector->x = v->getX();
			vector->y = v->getY();
		}
	}
}