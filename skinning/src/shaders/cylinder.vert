R"zzz(#version 330 core
uniform mat4 bone_transform; // transform the cylinder to the correct configuration
const float kPi = 3.1415926535897932384626433832795;
uniform mat4 projection;
uniform mat4 model;
uniform mat4 view;
in vec4 vertex_position;

// FIXME: Implement your vertex shader for cylinders
// Note: you need call sin/cos to transform the input mesh to a cylinder
void main() {
	mat4 mvp = projection * view * model;
	vec4 position;
	// Reorient along t
	position.x = vertex_position.y;
	position.y = vertex_position.z;
	position.z = vertex_position.x;
	position.w = 1;

	// resize along l
	mat4 M = bone_transform;
	float l = M[0][3];
	M[0][3] = 0;
	if(position.x == 0.5)
		position.x = l/2;
	else if(position.x == 1)
		position.x = l;

	if(position.y >0)
		position.y = 3;
	// make into cylinder
    position.y = sin(position.z * 2*kPi);
    position.z = cos(position.z * 2*kPi);
	position.y *= 0.25;
	position.z *= 0.25;
	gl_Position = mvp *M*position;
})zzz"
