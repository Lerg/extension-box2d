# Box2D Extension for Defold

Defold already has Box2D as a physics engine, however it's very limited. Therefore to unleash its full potential this extension was made. It doesn't cover the full Box2D API yet, but it's easy to add particular features you are missing. PRs are welcome.

In this extension Box2D is not tied to the Defold world so to speak, it can run fully independent. This for instance allows you to run a quick simulation of a flying projectile when a player is aiming and show the computed projectile path. Yes, like in Angry Birds.

You can have as many physics worlds as you want and it's up to you how you would like to tie physics objects and graphical objects on screen together. Normally you would just set your game object position and rotation in the update method to its physics body's position and rotation.

Keep in mind that Box2D works in meters, not in pixels. Either scale each physics value to match your game coordinate system, or scale down your game world to match the physics coordinate system. Some games are better coded with the first option, others with another.


# API Overview

## Functions

- [`box2d.init()`](#function_init)
- [`box2d.new_world()`](#function_new_world)

## Properties

- [`box2d.dynamic_body`](#property_dynamic_body)
- [`box2d.static_body`](#property_static_body)
- [`box2d.kinematic_body`](#property_kinematic_body)
- [`box2d.distance_joint`](#property_distance_joint)
- [`box2d.friction_joint`](#property_friction_joint)
- [`box2d.gear_joint`](#property_gear_joint)
- [`box2d.motor_joint`](#property_motor_joint)
- [`box2d.mouse_joint`](#property_mouse_joint)
- [`box2d.prismatic_joint`](#property_prismatic_joint)
- [`box2d.pulley_joint`](#property_pulley_joint)
- [`box2d.revolute_joint`](#property_revolute_joint)
- [`box2d.rope_joint`](#property_rope_joint)
- [`box2d.weld_joint`](#property_weld_joint)
- [`box2d.wheel_joint`](#property_wheel_joint)

## Events

- [`collision`](#event_collision)

# Project Settings

Open `game.project` and a new entry to the `dependencies` property:

- `https://github.com/Lerg/extension-box2d/archive/master.zip`

Then select `Project -> Fetch Libraries` to download the extension in your project.

After that you have to disable Defold's physics engine otherwise the inner libraries would clash and your game won't build. To do so you can open `game.project` and set `App Manifest` to `/box2d/physics_null.appmanifest`.

# Functions

---

## <a name="function_init">`box2d.init()`</a>

Initializes the extension and displays Box2D version.

---

## <a name="function_new_world">`box2d.new_world(params)`</a>

Crates and returns a new physics world.

The `params` table

### gravity <sub>optional</sub>
Vector3. Sets the gravity vector for the world. Default is (0, -10).

## Example

```lua
box2d.init()

local world = box2d.new_world{
	gravity = vmath.vector3(0, -20, 0)
}
```

---

# Box2D world API.

## Functions

- [`world.new_body()`](#function_world_new_body)
- [`world.new_joint()`](#function_world_new_joint)
- [`world.step()`](#function_world_step)
- [`world.destroy()`](#function_world_destroy)

## <a name="function_world_new_body">`world.new_body(params)`</a>

Creates a new physics body. Please refer to the official Box2D documentation for body descriptions and their parameters.

The `params` table

### type <sub>optional</sub>
Body type. `box2d.static_body`, `box2d.dynamic_body` (default) or `box2d.kinematic_body`. 

### is_fixed_rotation <sub>optional</sub>
Boolean.

### position <sub>optional</sub>
Vector3.

### is_sensor <sub>optional</sub>
Boolean.

### density <sub>optional</sub>
Float.

### friction <sub>optional</sub>
Float.

### restitution <sub>optional</sub>
Float. Bouncyness.

### width <sub>optional</sub>
Float. If width and height are set a box is created.

### height <sub>optional</sub>
Float.

### radius <sub>optional</sub>
Float. If radius is set and width/height are missing a circle is created.

### on_enter_collision <sub>optional</sub>
Function. Receives collision event.

### on_exit_collision <sub>optional</sub>
Function. Receives collision event.

### on_before_collision <sub>optional</sub>
Function. Receives collision event.

### on_after_collision <sub>optional</sub>
Function. Receives collision event.

## Example

```lua
self.world = box2d.new_world{
	gravity = self.gravity
}

local ground_body = self.world:new_body{
	type = box2d.static_body,
	position = vmath.vector(0, -500),
	width = 1000,
	height = 10,
	friction = 1,
	on_enter_collision = function(event)
		print('event.other.position', event.other.position)
	end
}
```

---

## <a name="function_world_new_joint">`world.new_joint(params)`</a>

Creates a new joint between physics bodies. Please refer to the official Box2D documentation for joint descriptions and their parameters.

The `params` table

### type <sub>optional</sub>
Joint type.

### collide_connected <sub>optional</sub>
Boolean.

### length <sub>optional</sub>
Float.

### frequency <sub>optional</sub>
Float.

### damping <sub>optional</sub>
Float.

### max_force <sub>optional</sub>
Float.

### max_torque <sub>optional</sub>
Float.

### angular_offset <sub>optional</sub>
Float.

### ratio <sub>optional</sub>
Float.

### correction_factor <sub>optional</sub>
Float.

### body <sub>optional</sub>
Physics body.

### other_body <sub>optional</sub>
Physics body.

### joint <sub>optional</sub>
Physics joint.

### other_joint <sub>optional</sub>
Physics joint.

### anchor <sub>optional</sub>
Vector3.

### other_anchor <sub>optional</sub>
Vector3.

### linear_offset <sub>optional</sub>
Vector3.

### target <sub>optional</sub>
Vector3.

### axis <sub>optional</sub>
Vector3.

## Example

```lua
local joint = self.world:new_joint{
	type = box2d.weld_joint,
	body = player_body,
	other_body = wall_body,
	anchor = player_body.position
}
```

---

## <a name="function_world_step">`world.step(time_step, velocity_iterations, position_iterations)`</a>

Progresses the simulation.

### time_step
Float. 

### velocity_iterations
Integer.

### position_iterations
Integer.

## Example

```lua
function update(dt)
	self.world:step(dt, 6, 4)
end
```

---

## <a name="function_world_destroy">`world.destroy()`</a>

Destroys physics world and all physics bodies in it.

---

# Events

## <a name="event_collision">`collision`</a>

Occurs when physics bodies are colliding.

### <a name="event_collision_self">`event.self`</a>

Physics body.

### <a name="event_collision_self">`event.other`</a>

Physics body.

---

# Box2D Body API.

## Functions

- [`body.apply_force()`](#function_body_apply_force)
- [`body.destroy()`](#function_body_destroy)

## Properties

- [`body.active`](#property_body_active)
- [`body.angular_velocity`](#property_body_angular_velocity)
- [`body.linear_velocity`](#property_body_linear_velocity)
- [`body.position`](#property_body_position)
- [`body.type`](#property_body_type)
- [`body.angle`](#property_body_angle)

## <a name="function_body_apply_force">`body.apply_force(force, position)`</a>

Applies force to the body.

### force
Vector3.

### position <sub>optional</sub>
Vector3.

---

## <a name="function_body_destroy">`body.destroy()`</a>

Destroys the body.

---

## <a name="property_body_active">`body.active`</a>

Boolean. Read/Write.

---

## <a name="property_body_angular_velocity">`body.angular_velocity`</a>

Float. Read/Write.

---

## <a name="property_body_position">`body.position`</a>

Vector3. Read/Write.

---

## <a name="property_body_type">`body.type`</a>

Body type. Read/Write.

---

## <a name="property_body_angle">`body.angle`</a>

Float. Read.

---

# Usage

Create a world using `box2d.new_world()`, add bodies calling `world:new_body()` with the newly created world. Run physics simulation calling `world:step()`, usually in your `update()` method. Play with physics bodies by applying force to them or changing their velocities, update visual representation of physics bodies with position and angle of the bodies.

---

# Patreon

If you like this extension please consider supporting me on Patreon https://patreon.com/Lerg
