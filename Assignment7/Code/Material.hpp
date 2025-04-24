//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE , MICROFACET };

class Material{
private:
    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    // TODO MISSION
    Vector3f getMiddleVector(const Vector3f &a, const Vector3f &b){
        Vector3f c = a + b;
        return c.normalized();
    } 
    float DisneyDiffuse(float NdotL, float NdotV, float HdotV,float roughness) {
        float Fd90 = 0.5f + 2.0f * roughness * HdotV * HdotV;
        float FdV = 1 + (Fd90 - 1) * std::pow( 1 - NdotV,5 );
	    float FdL = 1 + (Fd90 - 1) * std::pow( 1 - NdotL,5 );
	    return (1 / M_PI) * FdV * FdL;
    }
    float D_GGX(float roughness, float NdotH) {
        float alpha2 = roughness * roughness;
        float cosTheta2 = NdotH * NdotH;
        float denom = (cosTheta2 * (alpha2 - 1.0f) + 1.0f);
        return alpha2 / (M_PI * denom * denom);
    }
    float G_Smith(float roughness, float NdotV, float NdotL) {
        float G1 = G_Smith_GGX(NdotV, roughness);
        float G2 = G_Smith_GGX(NdotL, roughness);
        return G1 * G2;
    }
    float G_Smith_GGX(float NdotV, float a)
    {
        float r = (a + 1.0);
        float k = (r*r) / 8.0;
        
        float nom   = NdotV;
        float denom = NdotV * (1.0 - k) + k;
        
        return nom / denom;
    }
    float Fresnel(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float kr;
        if(abs(ior-1)<0.1f){
            float cosi = clamp(-1, 1, dotProduct(I, N));
            float etai = 1, etat = ior;
            if (cosi > 0) {  std::swap(etai, etat); }

            float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));

            if (sint >= 1) {
                kr = 1;
            }
            else {
                float cost = sqrtf(std::max(0.f, 1 - sint * sint));
                cosi = fabsf(cosi);
                float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
                float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
                kr = (Rs * Rs + Rp * Rp) / 2;
            }
        }
        else
        {
            float F0 = (ior - 1) * (ior - 1) / ((ior + 1) * (ior + 1));
            kr = F0 + (1 - F0) * pow(1 - dotProduct(I, N), 5);
        }
        
        return kr;
    }

public:
    MaterialType m_type;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float roughness;

    inline Material(MaterialType t = DIFFUSE, Vector3f e = Vector3f(0.0f));
    inline Material(MaterialType t, Vector3f e,Vector3f Kd);
    inline Material(MaterialType t, Vector3f e,float ior, Vector3f Kd, Vector3f Ks, float roughness);
    inline MaterialType getType();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    m_emission = e;
}
Material::Material(MaterialType t, Vector3f e,Vector3f Kd){
    m_type = t;
    m_emission = e;
    this->Kd = Kd;
}
Material::Material(MaterialType t, Vector3f e, float ior, Vector3f Kd, Vector3f Ks, float roughness){
    m_type = t;
    m_emission = e;
    this->ior = ior;
    this->Kd = Kd;
    this->Ks = Ks;
    this->roughness = roughness;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}

Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        case MICROFACET:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        case MICROFACET:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}

// TODO MISSION
Vector3f Material::eval(const Vector3f &V, const Vector3f &L, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse model
            float cosalpha = dotProduct(N, L);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            float NdotL = dotProduct(N, L);
            if(NdotL > 0.0f)
            {
                Vector3f H = getMiddleVector(L, V);
            
                float NdotV = dotProduct(N, V);
                float NdotH = dotProduct(N, H);
                float HdotV = dotProduct(H, V);

                // Vector3f diffuse = Kd * DisneyDiffuse(NdotL, NdotV, HdotV, roughness);
                float D = D_GGX(roughness, NdotH);
                float G = G_Smith(roughness, NdotV, NdotL);
                float F = Fresnel(V, H, ior);
                Vector3f diffuse = Kd * (Vector3f(clamp(0.0,1.0,1.0-F))/M_PI);
                Vector3f specular = Ks * (D * G * F) / (4 * NdotL * NdotV);
                return diffuse + specular;
            }
            else
                return Vector3f(0.0f);

            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
