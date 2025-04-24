#pragma once

#include "Vector.hpp"

// TODO MISSION
class Matrix4f {
private:
    float m[4][4];
public:
    Matrix4f() {
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i][j] = 0;
    }

    static Matrix4f Identity() {
        Matrix4f mat;
        for (int i = 0; i < 4; i++)
            mat.m[i][i] = 1.0f;
        return mat;
    }

    // 矩阵乘法
    Matrix4f operator*(const Matrix4f &other) const {
        Matrix4f result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    // 矩阵乘以向量(将向量视为4D向量，w=1)
    Vector3f operator*(const Vector3f &vec) const {
        float x = vec.x * m[0][0] + vec.y * m[0][1] + vec.z * m[0][2] + m[0][3];
        float y = vec.x * m[1][0] + vec.y * m[1][1] + vec.z * m[1][2] + m[1][3];
        float z = vec.x * m[2][0] + vec.y * m[2][1] + vec.z * m[2][2] + m[2][3];
        float w = vec.x * m[3][0] + vec.y * m[3][1] + vec.z * m[3][2] + m[3][3];

        // 透视除法
        if (w != 0) {
            x /= w;
            y /= w;
            z /= w;
        }

        return Vector3f(x, y, z);
    }

    // 创建平移矩阵
    static Matrix4f Translate(float tx, float ty, float tz) {
        Matrix4f mat = Identity();
        mat.m[0][3] = tx;
        mat.m[1][3] = ty;
        mat.m[2][3] = tz;
        return mat;
    }

    // 创建缩放矩阵
    static Matrix4f Scale(float sx, float sy, float sz) {
        Matrix4f mat = Identity();
        mat.m[0][0] = sx;
        mat.m[1][1] = sy;
        mat.m[2][2] = sz;
        return mat;
    }

    // 创建绕X轴旋转矩阵(角度制)
    static Matrix4f RotateX(float angle) {
        float rad = angle * M_PI / 180.0f;
        Matrix4f mat = Identity();
        mat.m[1][1] = cos(rad);
        mat.m[1][2] = -sin(rad);
        mat.m[2][1] = sin(rad);
        mat.m[2][2] = cos(rad);
        return mat;
    }

    // 创建绕Y轴旋转矩阵(角度制)
    static Matrix4f RotateY(float angle) {
        float rad = angle * M_PI / 180.0f;
        Matrix4f mat = Identity();
        mat.m[0][0] = cos(rad);
        mat.m[0][2] = sin(rad);
        mat.m[2][0] = -sin(rad);
        mat.m[2][2] = cos(rad);
        return mat;
    }

    // 创建绕Z轴旋转矩阵(角度制)
    static Matrix4f RotateZ(float angle) {
        float rad = angle * M_PI / 180.0f;
        Matrix4f mat = Identity();
        mat.m[0][0] = cos(rad);
        mat.m[0][1] = -sin(rad);
        mat.m[1][0] = sin(rad);
        mat.m[1][1] = cos(rad);
        return mat;
    }
};