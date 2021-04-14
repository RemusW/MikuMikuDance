R"zzz(#version 330 core
uniform mat4 bone_transform;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
flat out vec4 color;
in vec4 vertex_position;
void main() {
	mat4 mvp = projection * view * model;
	vec4 position = vertex_position;
	mat4 M = bone_transform;
	M[0][3] = 0;
	gl_Position = mvp * M * position;
	if (position.x == 0 && position.y == 0) {
		color = vec4(1, 0, 0, 1);
	} else {
		color = vec4(0, 0, 1, 1);
	}
})zzz"
