#version 420

in vec3 fPosition;
in vec3 fNormal;
in vec3 fColor;

uniform vec3 uColor;
uniform bool uUniformColor;
uniform bool uUseDiffuseShading;

out vec4 fragColor;

const vec3 lightPos = vec3(-2.0f, 5.0, 5.0f);
const float lightIntensity = 1.5;

void main(void) {
    vec3 color;
    if(uUniformColor){
        color = uColor;
    } else {
        color = fColor;
    }

   if(uUseDiffuseShading)
   {
       const vec3 toLightNormal = normalize(lightPos - fPosition);
       const float k_diff = 0.5f + dot(toLightNormal, fNormal) * 0.45;
       color = color * k_diff * lightIntensity;
   }

   fragColor = vec4(color, 1.0f);
}
