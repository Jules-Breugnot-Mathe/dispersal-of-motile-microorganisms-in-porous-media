#ifndef SHADER_HPP
#define SHADER_HPP

#include <string>
#include <glad/glad.h>

class Shader {
public:
    unsigned int ID;

    Shader(const std::string& vertexPath, const std::string& fragmentPath);
    void use() const;

    void setVec3(const std::string& name, float x, float y, float z) const;
};

#endif
