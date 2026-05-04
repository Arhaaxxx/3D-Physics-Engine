#define _USE_MATH_DEFINES
#include <cmath>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <fstream>
#include <vector>

#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>

#pragma comment(lib, "ws2_32.lib")

#include "json.hpp"
using json = nlohmann::json;

// =========================
// CHANGE THIS PATH
// =========================
const char* FILE_PATH = "C:/Users/arhaa/OneDrive/Desktop/AA/state.json";

// =========================
// NEW BODY STRUCT (ADDED)
// =========================
struct BodyData {
    glm::vec3 position;
    float radius;
};

struct Vec3 {
    float x, y, z;
};

// =========================
// CAPSULE STRUCT (ADDED)
// =========================
struct CapsuleData {
    glm::vec3 p1;
    glm::vec3 p2;
    float radius;
    float height;
};

std::vector<BodyData> bodies;
std::vector<CapsuleData> capsules;

SOCKET sock;
char buffer[65536];
std::string incoming;

// =========================
// SAFE JSON LOAD
// =========================
// std::vector<BodyData> loadBodies() {
//     static std::vector<BodyData> lastValid;

//     std::ifstream f(FILE_PATH);
//     if (!f.is_open()) return lastValid;

//     try {
//         json j; 
//         f >> j;

//         std::vector<BodyData> bodies;

//         for (auto& b : j["bodies"]) {
//             auto p = b["position"];
//             auto r = b["rotation"];

//             bodies.push_back({
//                 (float)p[0], (float)p[1], (float)p[2],
//                 (float)r[0], (float)r[1], (float)r[2],
//                 (float)b["radius"]
//             });
//         }

//         lastValid = bodies;
//         return bodies;

//     } catch (...) {
//         return lastValid;
//     }
// }

// =========================
// LOAD CAPSULES (ADDED)
// =========================
// std::vector<CapsuleData> loadCapsules() {
//     static std::vector<CapsuleData> lastValid;

//     std::ifstream f(FILE_PATH);
//     if (!f.is_open()) return lastValid;

//     try {
//         json j;
//         f >> j;

//         std::vector<CapsuleData> caps;

//         if (j.contains("capsules")) {
//             for (auto& c : j["capsules"]) {
//                 auto a = c["p1"];
//                 auto b = c["p2"];

//                 caps.push_back({
//                     { (float)a[0], (float)a[1], (float)a[2] },
//                     { (float)b[0], (float)b[1], (float)b[2] },
//                     (float)c["radius"],
//                     (float)c["height"]
//                 });
//             }
//         }

//         lastValid = caps;
//         return caps;

//     } catch (...) {
//         return lastValid;
//     }
// }

// =========================
// SHADER UTILS
// =========================
GLuint compile(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    return s;
}

GLuint createProgram(const char* vs, const char* fs) {
    GLuint v = compile(GL_VERTEX_SHADER, vs);
    GLuint f = compile(GL_FRAGMENT_SHADER, fs);

    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);

    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

// =========================
// ROTATION FROM DIRECTION (ADDED)
// =========================
glm::mat4 rotationFromDirection(glm::vec3 dir) {
    glm::vec3 up(0,0,1);

    if (glm::length(dir) < 1e-4f)
        return glm::mat4(1.0f);

    dir = glm::normalize(dir);
    float c = glm::dot(up, dir);

    if (c > 0.999f) return glm::mat4(1.0f);
    if (c < -0.999f)
        return glm::rotate(glm::mat4(1.0f),
                           glm::radians(180.0f),
                           glm::vec3(1,0,0));

    glm::vec3 axis = glm::normalize(glm::cross(up, dir));
    float angle = acos(c);

    return glm::rotate(glm::mat4(1.0f), angle, axis);
}

void initSocket() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2,2), &wsaData);

    sock = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(65432);
    server.sin_addr.s_addr = inet_addr("127.0.0.1");

    if (connect(sock, (sockaddr*)&server, sizeof(server)) < 0) {
        std::cout << "Connection failed\n";
    } else {
        std::cout << "Connected to Python\n";
    }
}

void receiveData() {

    // ---------- 1. READ SIZE (4 bytes) ----------
    int size = 0;
    int received = 0;

    while (received < 4) {
        int r = recv(sock, ((char*)&size) + received, 4 - received, 0);
        if (r <= 0) return;  // disconnected
        received += r;
    }

    // ---------- 2. READ FULL JSON ----------
    std::vector<char> buffer(size);
    int total = 0;

    while (total < size) {
        int r = recv(sock, buffer.data() + total, size - total, 0);
        if (r <= 0) return;  // disconnected
        total += r;
    }

    // ---------- 3. PARSE ----------
    try {
        std::string data(buffer.begin(), buffer.end());
        json j = json::parse(data);

        // ---------- BODIES ----------
        bodies.clear();

        if (j.contains("bodies")) {
            for (auto& b : j["bodies"]) {

                BodyData bdata;

                bdata.position = glm::vec3(
                    (float)b["position"][0],
                    (float)b["position"][1],
                    (float)b["position"][2]
                );

                bdata.radius = (float)b["radius"];

                bodies.push_back(bdata);
            }
        }

        // ---------- CAPSULES ----------
        capsules.clear();

        if (j.contains("capsules")) {
            for (auto& c : j["capsules"]) {

                CapsuleData cap;

                cap.p1 = glm::vec3(
                    (float)c["p1"][0],
                    (float)c["p1"][1],
                    (float)c["p1"][2]
                );

                cap.p2 = glm::vec3(
                    (float)c["p2"][0],
                    (float)c["p2"][1],
                    (float)c["p2"][2]
                );

                cap.radius = (float)c["radius"];
                cap.height = (float)c["height"];

                capsules.push_back(cap);
            }
        }

    } catch (...) {
        // ignore malformed frame
    }
}

glm::vec3 camPos(0.0f, -10.0f, 6.0f);
float yaw = 90.0f;
float pitch = -20.0f;

float lastX = 400, lastY = 300;
bool firstMouse = true;

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float sensitivity = 0.1f;
    float dx = lastX - xpos;
    float dy = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    dx *= sensitivity;
    dy *= sensitivity;

    yaw += dx;
    pitch += dy;

    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;
}

void createSphere(std::vector<float>& vertices,
                  std::vector<unsigned int>& indices,
                  int stacks = 16, int slices = 16)
{
    for (int i = 0; i <= stacks; ++i) {
        float phi = 3.14159265358979323846f * i / stacks;

        for (int j = 0; j <= slices; ++j) {
            float theta = 2 * M_PI * j / slices;

            float x = sin(phi) * cos(theta);
            float y = sin(phi) * sin(theta);
            float z = cos(phi);

            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = i * (slices + 1) + j;
            int second = first + slices + 1;

            indices.push_back(first);
            indices.push_back(second);
            indices.push_back(first + 1);

            indices.push_back(second);
            indices.push_back(second + 1);
            indices.push_back(first + 1);
        }
    }
}

void createCylinder(std::vector<float>& vertices,
                    std::vector<unsigned int>& indices,
                    int segments = 20)
{
    const float PI = 3.14159265358979323846f;

    // side vertices
    for (int i = 0; i <= segments; i++) {
        float theta = 2.0f * PI * i / segments;
        float x = cos(theta);
        float y = sin(theta);

        // bottom
        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(-0.5f);

        // top
        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(0.5f);
    }

    // side indices
    for (int i = 0; i < segments; i++) {
        int start = i * 2;

        indices.push_back(start);
        indices.push_back(start + 1);
        indices.push_back(start + 2);

        indices.push_back(start + 1);
        indices.push_back(start + 3);
        indices.push_back(start + 2);
    }
}

// =========================
// MAIN
// =========================
int main() {
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);

    GLFWwindow* window = glfwCreateWindow(
        mode->width,
        mode->height,
        "Simulation",
        NULL,
        NULL
    );

    glfwSetWindowPos(window, 0, 0);
    glfwMakeContextCurrent(window);

    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "GLAD failed\n";
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    // transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // =========================
    // CUBE
    // =========================
    float cubeVerts[] = {
        -0.5f,-0.5f,-0.5f,  0.5f,-0.5f,-0.5f,  0.5f, 0.5f,-0.5f, -0.5f, 0.5f,-0.5f,
        -0.5f,-0.5f, 0.5f,  0.5f,-0.5f, 0.5f,  0.5f, 0.5f, 0.5f, -0.5f, 0.5f, 0.5f
    };

    unsigned int cubeIdx[] = {
        0,1,2,2,3,0, 4,5,6,6,7,4,
        0,4,7,7,3,0, 1,5,6,6,2,1,
        3,2,6,6,7,3, 0,1,5,5,4,0
    };

    GLuint VAO,VBO,EBO;
    glGenVertexArrays(1,&VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1,&VBO);
    glBindBuffer(GL_ARRAY_BUFFER,VBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(cubeVerts),cubeVerts,GL_STATIC_DRAW);

    glGenBuffers(1,&EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof(cubeIdx),cubeIdx,GL_STATIC_DRAW);

    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);

    std::vector<float> sphereVerts;
    std::vector<unsigned int> sphereInd;

    createSphere(sphereVerts, sphereInd);

    GLuint sVAO, sVBO, sEBO;

    glGenVertexArrays(1, &sVAO);
    glBindVertexArray(sVAO);

    glGenBuffers(1, &sVBO);
    glBindBuffer(GL_ARRAY_BUFFER, sVBO);
    glBufferData(GL_ARRAY_BUFFER, sphereVerts.size()*sizeof(float), sphereVerts.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &sEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereInd.size()*sizeof(unsigned int), sphereInd.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    std::vector<float> cylVerts;
    std::vector<unsigned int> cylInd;

    createCylinder(cylVerts, cylInd);

    GLuint cVAO, cVBO, cEBO;

    glGenVertexArrays(1, &cVAO);
    glBindVertexArray(cVAO);

    glGenBuffers(1, &cVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cVBO);
    glBufferData(GL_ARRAY_BUFFER, cylVerts.size()*sizeof(float), cylVerts.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &cEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, cylInd.size()*sizeof(unsigned int), cylInd.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // =========================
    // GRID PLANE (ADD THIS)
    // =========================
    float plane[] = {
        -100,-100,0,
        100,-100,0,
        100, 100,0,
        -100, 100,0
    };

    unsigned int planeIdx[] = {0,1,2, 2,3,0};

    GLuint pVAO, pVBO, pEBO;
    glGenVertexArrays(1,&pVAO);
    glBindVertexArray(pVAO);

    glGenBuffers(1,&pVBO);
    glBindBuffer(GL_ARRAY_BUFFER,pVBO);
    glBufferData(GL_ARRAY_BUFFER,sizeof(plane),plane,GL_STATIC_DRAW);

    glGenBuffers(1,&pEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,pEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof(planeIdx),planeIdx,GL_STATIC_DRAW);

    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);

    const char* gridVS = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    out vec3 worldPos;
    uniform mat4 MVP;
    uniform mat4 model;
    void main(){
        worldPos = vec3(model * vec4(aPos,1.0));
        gl_Position = MVP * vec4(aPos,1.0);
    })";

    const char* gridFS = R"(
    #version 330 core
    in vec3 worldPos;
    out vec4 FragColor;

    float gridLine(float coord){
        float line = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
        return min(line,1.0);
    }

    void main(){
        float x = gridLine(worldPos.x);
        float y = gridLine(worldPos.y);

        float grid = min(x,y);
        float intensity = 1.0 - grid;

        vec3 color = vec3(0.4);
        float alpha = intensity * 0.4;

        float dist = length(worldPos.xy);
        alpha *= 1.0 / (1.0 + dist * 0.1);

        FragColor = vec4(color, alpha);
    })";

    GLuint gridShader = createProgram(gridVS, gridFS);

    // =========================
    // SHADER
    // =========================
    const char* vs = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    uniform mat4 MVP;
    void main(){
        gl_Position = MVP * vec4(aPos,1.0);
    })";

    const char* fs = R"(
    #version 330 core
    out vec4 FragColor;
    void main(){
        FragColor = vec4(0.3,0.9,0.9,1.0);
    })";

    GLuint cubeShader = createProgram(vs, fs);

    initSocket();

    // =========================
    // LOOP
    // =========================
    static glm::vec3 camPos(10,10,8);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        float speed = 0.1f;

        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.z = sin(glm::radians(pitch));
        front = glm::normalize(front);

        glm::vec3 right = glm::normalize(glm::cross(front, glm::vec3(0,0,1)));

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camPos += front * speed;

        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camPos -= front * speed;

        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camPos -= right * speed;

        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camPos += right * speed;

        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            camPos.z += speed;

        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            camPos.z -= speed;

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

        glClearColor(0.1f,0.1f,0.15f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        receiveData();

        static std::vector<CapsuleData> lastCapsules;

        if (!capsules.empty()) {
            lastCapsules = capsules;
        } else {
            capsules = lastCapsules;
        }

        // =========================
        // CAMERA CENTER
        // =========================
        static glm::vec3 smoothCenter(0,0,0);

        glm::vec3 center(0,0,0);
        int count = 0;

        // include bodies
        for (auto& b : bodies) {
            center += b.position / 5.0f;
            count++;
        }

        // include capsules (midpoint)
        for (auto& c : capsules) {
            glm::vec3 a(c.p1.x/5.0f, c.p1.y/5.0f, c.p1.z/5.0f);
            glm::vec3 b(c.p2.x/5.0f, c.p2.y/5.0f, c.p2.z/5.0f);

            center += (a + b) * 0.5f;
            count++;
        }

        // fallback
        if (count > 0)
            center /= (float)count;

        glm::vec3 targetCam = center + glm::vec3(8,8,6);
        // camPos = camPos * 0.92f + targetCam * 0.08f;

        glm::mat4 view = glm::lookAt(camPos, camPos + front, glm::vec3(0,0,1));
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        glm::mat4 projection = glm::perspective(
            glm::radians(45.0f),
            (float)width / (float)height,
            0.1f,
            200.0f
        );

        // =========================
        // DRAW ALL CUBES
        // =========================
        glUseProgram(cubeShader);
        glBindVertexArray(sVAO);

        for (int i = 0; i < bodies.size(); i++) {
            // if (i == 0) continue;
            BodyData b = bodies[i];

            glm::vec3 position = b.position / 5.0f;

            glm::mat4 model = glm::translate(glm::mat4(1.0f), position);

            // // 🔥 ROTATION (THIS FIXES YOUR ISSUE)
            // model = glm::rotate(model, rotation.x, glm::vec3(1,0,0));
            // model = glm::rotate(model, rotation.y, glm::vec3(0,1,0));
            // model = glm::rotate(model, rotation.z, glm::vec3(0,0,1));

            // 🔥 SHAPE VISUALIZATION
            // (we infer shape by size for now since JSON doesn’t include shape)
            
            // uniform scale for all bodies
            float r = b.radius / 5.0f;
            model = glm::scale(model, glm::vec3(r));

            glm::mat4 MVP = projection * view * model;

            glUniformMatrix4fv(
                glGetUniformLocation(cubeShader,"MVP"),
                1, GL_FALSE,
                glm::value_ptr(MVP)
            );

            glDrawElements(GL_TRIANGLES, sphereInd.size(), GL_UNSIGNED_INT, 0);
        }

        // =========================
        // DRAW CAPSULES (ADDED)
        // =========================
        for (auto& c : capsules) {

            glm::vec3 a(c.p1.x/5.0f, c.p1.y/5.0f, c.p1.z/5.0f);
            glm::vec3 b(c.p2.x/5.0f, c.p2.y/5.0f, c.p2.z/5.0f);

            float radius = c.radius / 5.0f;

            glm::vec3 diff = b - a;
            float fullLength = glm::length(diff);

            if (fullLength < 0.0001f) continue;

            glm::vec3 dir = diff / fullLength;

            // 🔥 Correct endpoints (surface → surface)
            glm::vec3 start = a + dir * radius;
            glm::vec3 end   = b - dir * radius;

            float length = glm::length(end - start) + 2.0f * radius;
            if (length < 0.001f) length = 0.001f;

            glm::vec3 mid = (start + end) * 0.5f;

            // -------- CORRECT ROTATION --------
            glm::vec3 up = glm::vec3(0, 0, 1);  // cylinder default axis

            float dot = glm::dot(up, dir);

            glm::mat4 rotation;

            if (dot > 0.9999f) {
                rotation = glm::mat4(1.0f);
            }
            else if (dot < -0.9999f) {
                rotation = glm::rotate(glm::mat4(1.0f), glm::pi<float>(), glm::vec3(1,0,0));
            }
            else {
                glm::vec3 axis = glm::normalize(glm::cross(up, dir));
                float angle = acos(dot);
                rotation = glm::rotate(glm::mat4(1.0f), angle, axis);
            }

            // -------- CYLINDER --------
            glBindVertexArray(cVAO);

            glm::mat4 model =
                glm::translate(glm::mat4(1.0f), mid) *
                rotation *
                glm::scale(glm::mat4(1.0f), glm::vec3(radius, radius, length)) *
                glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));

            glm::mat4 MVP = projection * view * model;

            glUniformMatrix4fv(
                glGetUniformLocation(cubeShader,"MVP"),
                1, GL_FALSE,
                glm::value_ptr(MVP)
            );

            glDrawElements(GL_TRIANGLES, cylInd.size(), GL_UNSIGNED_INT, 0);


            // -------- SPHERE CAPS --------
            glBindVertexArray(sVAO);

            // top sphere
            glm::mat4 m1 = glm::translate(glm::mat4(1.0f), a);
            m1 = glm::scale(m1, glm::vec3(radius));

            glm::mat4 MVP1 = projection * view * m1;

            glUniformMatrix4fv(
                glGetUniformLocation(cubeShader,"MVP"),
                1, GL_FALSE,
                glm::value_ptr(MVP1)
            );

            glDrawElements(GL_TRIANGLES, sphereInd.size(), GL_UNSIGNED_INT, 0);


            // bottom sphere
            glm::mat4 m2 = glm::translate(glm::mat4(1.0f), b);
            m2 = glm::scale(m2, glm::vec3(radius));

            glm::mat4 MVP2 = projection * view * m2;

            glUniformMatrix4fv(
                glGetUniformLocation(cubeShader,"MVP"),
                1, GL_FALSE,
                glm::value_ptr(MVP2)
            );

            glDrawElements(GL_TRIANGLES, sphereInd.size(), GL_UNSIGNED_INT, 0);
        }
        // =========================
        // DRAW GRID (INFINITE)
        // =========================
        glUseProgram(gridShader);

        // snap grid under camera (same as your working version)
        glm::vec3 gridPos = glm::floor(camPos);
        gridPos.z = 0;

        glm::mat4 gridModel = glm::translate(glm::mat4(1.0f), gridPos);
        glm::mat4 gridMVP = projection * view * gridModel;

        glBindVertexArray(pVAO);

        glUniformMatrix4fv(
            glGetUniformLocation(gridShader,"MVP"),
            1, GL_FALSE,
            glm::value_ptr(gridMVP)
        );

        glUniformMatrix4fv(
            glGetUniformLocation(gridShader,"model"),
            1, GL_FALSE,
            glm::value_ptr(gridModel)
        );

        glDrawElements(GL_TRIANGLES,6,GL_UNSIGNED_INT,0);

        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}