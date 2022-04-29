#version 420

in vec3 vPosition;
in vec3 vNormal;
in vec3 vColor;

out vec3 fPosition;
out vec3 fNormal;
out vec3 fColor;

uniform mat4 MVP;
uniform mat4 world;
uniform mat4 worldIT;

void main(void) {
   gl_Position = MVP * vec4(vPosition, 1.0f);

   fPosition = (world * vec4(vPosition, 1)).xyz;
   fNormal = (worldIT * vec4(vNormal, 0)).xyz;
   fColor = vColor;
}

