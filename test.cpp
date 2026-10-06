#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

struct Vec3 {
    float x, y, z;
    Vec3 operator+(const Vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vec3 operator-(const Vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float value) const { return {x * value, y * value, z * value}; }
};

static float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static Vec3 normalize(const Vec3& value) {
    const float length = std::sqrt(dot(value, value));
    return length > 0.0001f ? value * (1.0f / length) : Vec3{0.0f, 0.0f, 0.0f};
}

struct Mat4 {
    float value[16]{};
    static Mat4 identity() {
        Mat4 result;
        result.value[0] = result.value[5] = result.value[10] = result.value[15] = 1.0f;
        return result;
    }
};

static Mat4 perspective(float fieldOfView, float aspect, float nearPlane, float farPlane) {
    Mat4 result{};
    const float tangent = std::tan(fieldOfView * 0.5f);
    result.value[0] = 1.0f / (aspect * tangent);
    result.value[5] = 1.0f / tangent;
    result.value[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    result.value[11] = -1.0f;
    result.value[14] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
    return result;
}

static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& worldUp) {
    const Vec3 forward = normalize(target - eye);
    const Vec3 right = normalize(cross(forward, worldUp));
    const Vec3 up = cross(right, forward);
    Mat4 result = Mat4::identity();
    result.value[0] = right.x; result.value[4] = right.y; result.value[8] = right.z;
    result.value[1] = up.x; result.value[5] = up.y; result.value[9] = up.z;
    result.value[2] = -forward.x; result.value[6] = -forward.y; result.value[10] = -forward.z;
    result.value[12] = -dot(right, eye); result.value[13] = -dot(up, eye); result.value[14] = dot(forward, eye);
    return result;
}

static Mat4 modelMatrix(const Vec3& position, const Vec3& scale) {
    Mat4 result = Mat4::identity();
    result.value[0] = scale.x; result.value[5] = scale.y; result.value[10] = scale.z;
    result.value[12] = position.x; result.value[13] = position.y; result.value[14] = position.z;
    return result;
}

struct CameraState {
    bool automatic = true;
    bool rotating = false;
    bool panning = false;
    bool firstMouse = true;
    double lastX = 0.0;
    double lastY = 0.0;
    float yaw = -2.28f;
    float pitch = -0.22f;
    Vec3 position{7.0f, 3.5f, 6.0f};
};

struct CameraKeyframe {
    float time;
    Vec3 position;
    Vec3 target;
};

static Vec3 lerp(const Vec3& a, const Vec3& b, float amount) {
    return a * (1.0f - amount) + b * amount;
}

static void cameraFromKeyframes(float time, const CameraKeyframe* keyframes, int count, Vec3& position, Vec3& target) {
    const float duration = keyframes[count - 1].time;
    const float loopTime = std::fmod(time, duration);
    for (int index = 0; index < count - 1; ++index) {
        const CameraKeyframe& current = keyframes[index];
        const CameraKeyframe& next = keyframes[index + 1];
        if (loopTime <= next.time) {
            const float amount = (loopTime - current.time) / (next.time - current.time);
            position = lerp(current.position, next.position, amount);
            target = lerp(current.target, next.target, amount);
            return;
        }
    }
    position = keyframes[0].position;
    target = keyframes[0].target;
}

static Vec3 cameraForward(const CameraState& camera) {
    const float horizontal = std::cos(camera.pitch);
    return normalize({std::sin(camera.yaw) * horizontal, std::sin(camera.pitch), std::cos(camera.yaw) * horizontal});
}

struct CollisionBox {
    float minX;
    float maxX;
    float minZ;
    float maxZ;
};

static bool cameraCanOccupy(const Vec3& position) {
    constexpr float radius = 0.38f;
    if (position.x < -12.4f || position.x > 12.4f || position.z < -10.4f || position.z > 10.4f) return false;
    const CollisionBox obstacles[] = {
        {-4.78f, 4.78f, -3.53f, -0.27f},
        {-1.93f, 1.93f, 0.27f, 2.43f},
        {-9.58f, -5.22f, -7.83f, -5.97f},
        {5.67f, 7.73f, -6.88f, -4.82f},
        {4.57f, 6.03f, 1.05f, 2.55f}
    };
    for (const CollisionBox& obstacle : obstacles) {
        if (position.x > obstacle.minX - radius && position.x < obstacle.maxX + radius &&
            position.z > obstacle.minZ - radius && position.z < obstacle.maxZ + radius) return false;
    }
    return true;
}

static void moveCamera(CameraState& camera, const Vec3& delta) {
    Vec3 candidate = camera.position;
    candidate.x += delta.x;
    if (cameraCanOccupy(candidate)) camera.position.x = candidate.x;
    candidate = camera.position;
    candidate.z += delta.z;
    if (cameraCanOccupy(candidate)) camera.position.z = candidate.z;
    candidate = camera.position;
    candidate.y = std::max(0.7f, std::min(6.0f, candidate.y + delta.y));
    camera.position.y = candidate.y;
}

static GLuint compileShader(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[1024]{};
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::cerr << "Erreur shader: " << log << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return shader;
}

static GLuint createProgram(bool unlit) {
    const char* vertexSource = R"glsl(
        #version 330 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec3 normal;
        out vec3 worldPosition;
        out vec3 worldNormal;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        void main() {
            vec4 world = model * vec4(position, 1.0);
            worldPosition = world.xyz;
            worldNormal = mat3(transpose(inverse(model))) * normal;
            gl_Position = projection * view * world;
        }
    )glsl";
    const char* fragmentSource = unlit ? R"glsl(
        #version 330 core
        in vec3 worldPosition;
        in vec3 worldNormal;
        out vec4 fragmentColor;
        uniform vec3 objectColor;
        void main() {
            fragmentColor = vec4(objectColor, 1.0);
        }
    )glsl" : R"glsl(
        #version 330 core
        in vec3 worldPosition;
        in vec3 worldNormal;
        out vec4 fragmentColor;
        uniform vec3 objectColor;
        uniform vec3 cameraPosition;
        uniform vec3 lightPosition[5];
        uniform vec3 lightColor[5];
        uniform vec3 lightDirection[5];
        uniform float lightType[5];
        uniform float lightIntensity[5];
        uniform float time;
        vec3 lightContribution(int index) {
            vec3 normal = normalize(worldNormal);
            vec3 direction = lightType[index] < 0.5
                ? normalize(-lightDirection[index])
                : normalize(lightPosition[index] - worldPosition);
            float distanceToLight = length(lightPosition[index] - worldPosition);
            float attenuation = lightType[index] < 0.5
                ? 1.0
                : 1.0 / (1.0 + 0.08 * distanceToLight * distanceToLight);
            float diffuse = max(dot(normal, direction), 0.0);
            vec3 viewDirection = normalize(cameraPosition - worldPosition);
            vec3 halfway = normalize(direction + viewDirection);
            float specular = pow(max(dot(normal, halfway), 0.0), 32.0);
            return lightColor[index] * (diffuse + specular * 0.35) * lightIntensity[index] * attenuation;
        }
        void main() {
            vec3 ambient = objectColor * (0.12 + 0.025 * sin(time));
            vec3 result = ambient;
            for (int index = 0; index < 5; ++index) result += objectColor * lightContribution(index);
            fragmentColor = vec4(pow(result, vec3(1.0 / 2.2)), 1.0);
        }
    )glsl";
    const GLuint vertex = compileShader(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragment = compileShader(GL_FRAGMENT_SHADER, fragmentSource);
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    GLint success = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char log[1024]{};
        glGetProgramInfoLog(program, sizeof(log), nullptr, log);
        std::cerr << "Erreur programme GLSL: " << log << std::endl;
        std::exit(EXIT_FAILURE);
    }
    glDeleteShader(vertex);
    glDeleteShader(fragment);
    return program;
}

static std::vector<float> makeCube() {
    const Vec3 corners[] = {{-1,-1,1}, {1,-1,1}, {1,1,1}, {-1,1,1}, {-1,-1,-1}, {1,-1,-1}, {1,1,-1}, {-1,1,-1}};
    const int faces[][4] = {{0,1,2,3}, {5,4,7,6}, {3,2,6,7}, {4,5,1,0}, {1,5,6,2}, {4,0,3,7}};
    const Vec3 normals[] = {{0,0,1}, {0,0,-1}, {0,1,0}, {0,-1,0}, {1,0,0}, {-1,0,0}};
    const int triangles[] = {0,1,2, 0,2,3};
    std::vector<float> result;
    for (int face = 0; face < 6; ++face)
        for (int vertex : triangles) {
            const Vec3& point = corners[faces[face][vertex]];
            const Vec3& normal = normals[face];
            result.insert(result.end(), {point.x, point.y, point.z, normal.x, normal.y, normal.z});
        }
    return result;
}

static void drawCube(GLuint vao, GLuint program, const Vec3& position, const Vec3& scale, const Vec3& color) {
    glUniformMatrix4fv(glGetUniformLocation(program, "model"), 1, GL_FALSE, modelMatrix(position, scale).value);
    glUniform3f(glGetUniformLocation(program, "objectColor"), color.x, color.y, color.z);
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

static void drawTelevisionImage(GLuint vao, GLuint program) {
    const char* text[] = {
        "11111 10000 10000 11110 00111 00111 10001 10001 01110",
        "10001 11000 10000 10001 01000 01000 10001 10001 10001",
        "10001 10100 10000 10001 01000 01000 10001 10001 10001",
        "11111 10010 10000 11110 01000 01000 10101 10001 01110",
        "10001 10001 10000 10001 01000 01000 11011 10001 00100",
        "10001 10001 10000 10001 01000 01000 10001 10001 00100",
        "10001 10001 11111 11110 00111 00111 10001 10001 00100"
    };
    const float cellWidth = 0.075f;
    const float cellHeight = 0.10f;
    const float startX = -1.70f;
    const float startY = 3.78f;
    for (int row = 0; row < 7; ++row) {
        int column = 0;
        for (const char* cursor = text[row]; *cursor != '\0'; ++cursor) {
            if (*cursor == ' ') {
                ++column;
                continue;
            }
            if (*cursor == '1') {
                const float x = startX + column * cellWidth;
                const float y = startY - row * cellHeight;
                drawCube(vao, program, {x, y, -2.19f}, {cellWidth * 0.42f, cellHeight * 0.42f, 0.02f}, {0.95f, 0.85f, 0.18f});
            }
            ++column;
        }
    }
}

static void drawWelcomeCharacter(GLuint vao, GLuint program, float time) {
    const float wave = std::sin(time * 4.0f) * 0.35f;
    const float x = 5.3f;
    const float z = 1.8f;
    drawCube(vao, program, {x, 1.7f, z}, {0.75f, 1.0f, 0.45f}, {0.08f, 0.22f, 0.48f});
    drawCube(vao, program, {x, 3.15f, z}, {0.52f, 0.52f, 0.52f}, {0.55f, 0.28f, 0.16f});
    drawCube(vao, program, {x, 3.55f, z}, {0.56f, 0.16f, 0.56f}, {0.05f, 0.04f, 0.03f});
    drawCube(vao, program, {x - 0.18f, 3.2f, z - 0.48f}, {0.07f, 0.07f, 0.04f}, {0.02f, 0.02f, 0.02f});
    drawCube(vao, program, {x + 0.18f, 3.2f, z - 0.48f}, {0.07f, 0.07f, 0.04f}, {0.02f, 0.02f, 0.02f});
    drawCube(vao, program, {x, 3.05f, z - 0.50f}, {0.07f, 0.10f, 0.05f}, {0.42f, 0.18f, 0.10f});
    drawCube(vao, program, {x, 2.82f, z - 0.50f}, {0.20f, 0.045f, 0.04f}, {0.08f, 0.02f, 0.02f});
    drawCube(vao, program, {x - 1.0f, 1.95f + wave, z}, {0.22f, 0.75f, 0.22f}, {0.08f, 0.22f, 0.48f});
    drawCube(vao, program, {x + 1.0f, 1.95f, z}, {0.22f, 0.75f, 0.22f}, {0.08f, 0.22f, 0.48f});
    drawCube(vao, program, {x - 0.38f, 0.45f, z}, {0.22f, 0.75f, 0.22f}, {0.04f, 0.06f, 0.12f});
    drawCube(vao, program, {x + 0.38f, 0.45f, z}, {0.22f, 0.75f, 0.22f}, {0.04f, 0.06f, 0.12f});
    drawCube(vao, program, {x - 1.0f, 2.85f + wave, z}, {0.25f, 0.25f, 0.25f}, {0.55f, 0.28f, 0.16f});
}

static void keyCallback(GLFWwindow* window, int key, int, int action, int) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        CameraState* camera = static_cast<CameraState*>(glfwGetWindowUserPointer(window));
        camera->automatic = !camera->automatic;
        std::cout << (camera->automatic ? "Camera automatique: ON" : "Camera manuelle: ON") << std::endl;
    }
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int) {
    CameraState* camera = static_cast<CameraState*>(glfwGetWindowUserPointer(window));
    if (button == GLFW_MOUSE_BUTTON_LEFT) camera->rotating = action == GLFW_PRESS;
    if (button == GLFW_MOUSE_BUTTON_RIGHT) camera->panning = action == GLFW_PRESS;
    if (action == GLFW_PRESS) camera->firstMouse = true;
}

static void cursorPositionCallback(GLFWwindow* window, double x, double y) {
    CameraState* camera = static_cast<CameraState*>(glfwGetWindowUserPointer(window));
    if (camera->firstMouse) {
        camera->lastX = x;
        camera->lastY = y;
        camera->firstMouse = false;
        return;
    }
    const float deltaX = static_cast<float>(x - camera->lastX);
    const float deltaY = static_cast<float>(camera->lastY - y);
    camera->lastX = x;
    camera->lastY = y;
    if (camera->automatic) return;
    if (camera->rotating) {
        camera->yaw += deltaX * 0.006f;
        camera->pitch = std::max(-1.35f, std::min(1.35f, camera->pitch + deltaY * 0.006f));
    } else if (camera->panning) {
        const Vec3 right = normalize(cross(cameraForward(*camera), {0.0f, 1.0f, 0.0f}));
        moveCamera(*camera, -right * (deltaX * 0.025f) + Vec3{0.0f, deltaY * 0.025f, 0.0f});
    }
}

static void scrollCallback(GLFWwindow* window, double, double yOffset) {
    CameraState* camera = static_cast<CameraState*>(glfwGetWindowUserPointer(window));
    if (!camera->automatic) {
        moveCamera(*camera, cameraForward(*camera) * static_cast<float>(yOffset) * 0.8f);
    }
}

int main() {
    if (!glfwInit()) { std::cerr << "Impossible d'initialiser GLFW.\n"; return EXIT_FAILURE; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1100, 700, "Modern OpenGL - Virtual Room", nullptr, nullptr);
    if (!window) { glfwTerminate(); return EXIT_FAILURE; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { glfwDestroyWindow(window); glfwTerminate(); return EXIT_FAILURE; }
    glEnable(GL_DEPTH_TEST);

    CameraState cameraState;
    glfwSetWindowUserPointer(window, &cameraState);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPositionCallback);
    glfwSetScrollCallback(window, scrollCallback);
    const GLuint program = createProgram(false);
    const GLuint unlitProgram = createProgram(true);
    const CameraKeyframe keyframes[] = {
        {0.0f, {7.5f, 3.2f, 5.8f}, {0.0f, 1.5f, 0.0f}},
        {4.0f, {4.0f, 4.8f, 7.0f}, {0.0f, 2.0f, -1.0f}},
        {8.0f, {-6.5f, 3.8f, 5.0f}, {-1.0f, 2.0f, -1.5f}},
        {12.0f, {-7.0f, 2.8f, -5.5f}, {0.0f, 2.0f, -2.0f}},
        {16.0f, {6.0f, 3.5f, -6.5f}, {1.0f, 2.5f, -2.0f}},
        {20.0f, {7.5f, 3.2f, 5.8f}, {0.0f, 1.5f, 0.0f}}
    };
    const std::vector<float> cube = makeCube();
    GLuint vao = 0, vbo = 0;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, cube.size() * sizeof(float), cube.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glBindVertexArray(0);

    std::cout << "Virtual Room chargee. ESPACE: mode manuel/automatique, fleches: deplacement, clic gauche: regarder, clic droit: deplacer, molette: avancer/reculer, ESC: quitter.\n";
    double previousTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        const double currentTime = glfwGetTime();
        const float deltaTime = static_cast<float>(currentTime - previousTime);
        previousTime = currentTime;
        glfwPollEvents();
        if (!cameraState.automatic) {
            const float movementSpeed = 4.0f * deltaTime;
            const Vec3 forward = cameraForward(cameraState);
            const Vec3 horizontalForward = normalize({forward.x, 0.0f, forward.z});
            const Vec3 right = normalize(cross(horizontalForward, {0.0f, 1.0f, 0.0f}));
            Vec3 movement{};
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) movement = movement + horizontalForward * movementSpeed;
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) movement = movement - horizontalForward * movementSpeed;
            if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) movement = movement - right * movementSpeed;
            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) movement = movement + right * movementSpeed;
            moveCamera(cameraState, movement);
        }
        Vec3 automaticCamera{};
        Vec3 automaticTarget{};
        cameraFromKeyframes(static_cast<float>(currentTime) * 0.8f, keyframes, 6, automaticCamera, automaticTarget);
        const Vec3 camera = cameraState.automatic ? automaticCamera : cameraState.position;
        const Vec3 target = cameraState.automatic ? automaticTarget : camera + cameraForward(cameraState);
        int width = 1, height = 1;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.035f, 0.025f, 0.02f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUseProgram(program);
        glUniformMatrix4fv(glGetUniformLocation(program, "view"), 1, GL_FALSE, lookAt(camera, target, {0.0f, 1.0f, 0.0f}).value);
        glUniformMatrix4fv(glGetUniformLocation(program, "projection"), 1, GL_FALSE, perspective(1.05f, static_cast<float>(width) / height, 0.1f, 100.0f).value);
        glUniform3f(glGetUniformLocation(program, "cameraPosition"), camera.x, camera.y, camera.z);
        const Vec3 lightPositions[] = {{-3.0f, 5.1f, -2.0f}, {0.0f, 6.5f, 1.0f}, {6.0f, 3.5f, -4.0f}, {-6.0f, 3.0f, 2.0f}, {0.0f, 5.0f, 0.0f}};
        const Vec3 lightColors[] = {{1.0f, 0.72f, 0.25f}, {1.0f, 0.82f, 0.48f}, {0.25f, 0.55f, 1.0f}, {0.35f, 1.0f, 0.55f}, {0.75f, 0.80f, 1.0f}};
        const Vec3 lightDirections[] = {{0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {-0.4f, -1.0f, -0.25f}};
        const float lightTypes[] = {1.0f, 1.0f, 1.0f, 1.0f, 0.0f};
        const float lightIntensities[] = {5.0f, 4.0f, 3.0f, 2.5f, 0.75f};
        for (int index = 0; index < 5; ++index) {
            glUniform3f(glGetUniformLocation(program, ("lightPosition[" + std::to_string(index) + "]").c_str()), lightPositions[index].x, lightPositions[index].y, lightPositions[index].z);
            glUniform3f(glGetUniformLocation(program, ("lightColor[" + std::to_string(index) + "]").c_str()), lightColors[index].x, lightColors[index].y, lightColors[index].z);
            glUniform3f(glGetUniformLocation(program, ("lightDirection[" + std::to_string(index) + "]").c_str()), lightDirections[index].x, lightDirections[index].y, lightDirections[index].z);
            glUniform1f(glGetUniformLocation(program, ("lightType[" + std::to_string(index) + "]").c_str()), lightTypes[index]);
            glUniform1f(glGetUniformLocation(program, ("lightIntensity[" + std::to_string(index) + "]").c_str()), lightIntensities[index]);
        }
        glUniform1f(glGetUniformLocation(program, "time"), static_cast<float>(currentTime));
        glUseProgram(unlitProgram);
        glUniformMatrix4fv(glGetUniformLocation(unlitProgram, "view"), 1, GL_FALSE, lookAt(camera, target, {0.0f, 1.0f, 0.0f}).value);
        glUniformMatrix4fv(glGetUniformLocation(unlitProgram, "projection"), 1, GL_FALSE, perspective(1.05f, static_cast<float>(width) / height, 0.1f, 100.0f).value);
        glUseProgram(program);

        // Sol, plafond et murs d'une piece agrandie avec une ouverture au fond.
        drawCube(vao, program, {0.0f, -0.45f, 0.0f}, {13.0f, 0.45f, 11.0f}, {0.22f, 0.13f, 0.08f});
        drawCube(vao, program, {0.0f, 7.0f, 0.0f}, {13.0f, 0.25f, 11.0f}, {0.28f, 0.25f, 0.22f});
        drawCube(vao, program, {-7.5f, 3.3f, -11.0f}, {5.25f, 3.8f, 0.25f}, {0.16f, 0.22f, 0.28f});
        drawCube(vao, program, {7.5f, 3.3f, -11.0f}, {5.25f, 3.8f, 0.25f}, {0.16f, 0.22f, 0.28f});
        drawCube(vao, program, {0.0f, 5.75f, -11.0f}, {2.25f, 1.25f, 0.25f}, {0.16f, 0.22f, 0.28f});
        drawCube(vao, program, {-13.0f, 3.3f, 0.0f}, {0.25f, 3.8f, 11.0f}, {0.18f, 0.24f, 0.27f});
        drawCube(vao, program, {13.0f, 3.3f, 0.0f}, {0.25f, 3.8f, 11.0f}, {0.18f, 0.24f, 0.27f});

        // Porte d'accueil dans l'ouverture du mur du fond.
        drawCube(vao, program, {0.0f, 2.25f, -10.68f}, {2.0f, 2.25f, 0.12f}, {0.30f, 0.12f, 0.05f});
        drawCube(vao, program, {-2.15f, 2.25f, -10.65f}, {0.15f, 2.4f, 0.22f}, {0.42f, 0.22f, 0.08f});
        drawCube(vao, program, {2.15f, 2.25f, -10.65f}, {0.15f, 2.4f, 0.22f}, {0.42f, 0.22f, 0.08f});
        drawCube(vao, program, {0.0f, 4.65f, -10.65f}, {2.3f, 0.15f, 0.22f}, {0.42f, 0.22f, 0.08f});
        drawCube(vao, program, {1.55f, 2.25f, -10.42f}, {0.12f, 0.12f, 0.08f}, {0.95f, 0.72f, 0.18f});

        // Bureau principal, plateau, pieds et ecran.
        drawCube(vao, program, {0.0f, 2.0f, -1.9f}, {4.4f, 0.22f, 1.25f}, {0.38f, 0.18f, 0.08f});
        for (float x : {-3.7f, 3.7f}) drawCube(vao, program, {x, 0.8f, -1.9f}, {0.22f, 1.2f, 0.22f}, {0.20f, 0.10f, 0.05f});
        glUseProgram(unlitProgram);
        drawCube(vao, unlitProgram, {0.0f, 3.15f, -2.35f}, {2.25f, 1.05f, 0.12f}, {0.04f, 0.08f, 0.10f});
        glUseProgram(program);
        drawCube(vao, program, {0.0f, 2.15f, -2.35f}, {0.12f, 0.95f, 0.12f}, {0.10f, 0.10f, 0.10f});
        drawCube(vao, program, {0.0f, 2.02f, -2.35f}, {0.75f, 0.08f, 0.5f}, {0.10f, 0.10f, 0.10f});
        drawCube(vao, program, {0.0f, 2.23f, -1.2f}, {1.0f, 0.06f, 0.65f}, {0.22f, 0.22f, 0.18f});

        // Chaise devant le bureau.
        drawCube(vao, program, {0.0f, 1.25f, 0.7f}, {1.55f, 0.18f, 1.35f}, {0.10f, 0.22f, 0.28f});
        drawCube(vao, program, {0.0f, 2.55f, 1.75f}, {1.55f, 1.3f, 0.18f}, {0.10f, 0.22f, 0.28f});
        drawCube(vao, program, {0.0f, 0.45f, 0.7f}, {0.15f, 0.8f, 0.15f}, {0.08f, 0.08f, 0.08f});

        // Bibliotheque a gauche et livres colores.
        drawCube(vao, program, {-7.4f, 3.0f, -6.9f}, {1.8f, 3.0f, 0.55f}, {0.28f, 0.13f, 0.07f});
        for (int shelf = 0; shelf < 4; ++shelf) {
            drawCube(vao, program, {-7.4f, 0.7f + shelf * 1.45f, -6.25f}, {1.45f, 0.08f, 0.16f}, {0.08f, 0.05f, 0.03f});
            drawCube(vao, program, {-8.25f, 1.15f + shelf * 1.45f, -6.15f}, {0.28f, 0.42f, 0.16f}, {0.55f, 0.12f + shelf * 0.08f, 0.08f});
            drawCube(vao, program, {-7.5f, 1.15f + shelf * 1.45f, -6.15f}, {0.18f, 0.42f, 0.16f}, {0.10f, 0.35f, 0.55f});
        }

        // Lampe suspendue au-dessus du bureau, plante et tableau mural.
        drawCube(vao, program, {-3.0f, 6.35f, -2.0f}, {0.06f, 0.65f, 0.06f}, {0.10f, 0.10f, 0.08f});
        drawCube(vao, program, {-3.0f, 5.75f, -2.0f}, {0.65f, 0.18f, 0.65f}, {0.95f, 0.55f, 0.12f});
        glUseProgram(unlitProgram);
        drawCube(vao, unlitProgram, {-3.0f, 5.48f, -2.12f}, {0.22f, 0.10f, 0.22f}, {1.0f, 0.85f, 0.25f});
        glUseProgram(program);
        drawCube(vao, program, {6.7f, 0.8f, -5.8f}, {0.65f, 0.7f, 0.65f}, {0.28f, 0.16f, 0.08f});
        drawCube(vao, program, {6.7f, 2.1f, -5.8f}, {0.22f, 1.0f, 0.22f}, {0.10f, 0.38f, 0.16f});
        drawCube(vao, program, {4.2f, 4.0f, -7.65f}, {2.1f, 1.5f, 0.08f}, {0.55f, 0.25f, 0.12f});
        drawCube(vao, program, {4.2f, 4.0f, -7.52f}, {1.8f, 1.2f, 0.05f}, {0.08f, 0.25f, 0.32f});
        glUseProgram(unlitProgram);
        drawTelevisionImage(vao, unlitProgram);
        glUseProgram(program);
        drawWelcomeCharacter(vao, program, static_cast<float>(currentTime));
        glfwSwapBuffers(window);
    }
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);
    glDeleteProgram(unlitProgram);
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
