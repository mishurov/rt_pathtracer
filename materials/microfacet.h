#ifndef RTPT_INTEGRATORS_MICROFACET_H_
#define RTPT_INTEGRATORS_MICROFACET_H_

// PBRT https://github.com/mmp/pbrt-v3

#include "integrator/utils.h"

namespace rt_pathtracer {

FW_CUDA_FUNC float Erf(float x) {
    float a1 = 0.254829592f;
    float a2 = -0.284496736f;
    float a3 = 1.421413741f;
    float a4 = -1.453152027f;
    float a5 = 1.061405429f;
    float p = 0.3275911f;

    int sign = 1;
    if (x < 0) sign = -1;
    x = std::abs(x);

    float t = 1 / (1 + p * x);
    float y =
        1 -
        (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);

    return sign * y;
}

FW_CUDA_FUNC float ErfInv(float x) {
    float w, p;
    x = clampf(x, -.99999f, .99999f);
    w = -std::log((1 - x) * (1 + x));
    if (w < 5) {
        w = w - 2.5f;
        p = 2.81022636e-08f;
        p = 3.43273939e-07f + p * w;
        p = -3.5233877e-06f + p * w;
        p = -4.39150654e-06f + p * w;
        p = 0.00021858087f + p * w;
        p = -0.00125372503f + p * w;
        p = -0.00417768164f + p * w;
        p = 0.246640727f + p * w;
        p = 1.50140941f + p * w;
    } else {
        w = sqrt(w) - 3;
        p = -0.000200214257f;
        p = 0.000100950558f + p * w;
        p = 0.00134934322f + p * w;
        p = -0.00367342844f + p * w;
        p = 0.00573950773f + p * w;
        p = -0.0076224613f + p * w;
        p = 0.00943887047f + p * w;
        p = 1.00167406f + p * w;
        p = 2.83297682f + p * w;
    }
    return p * x;
}


FW_CUDA_FUNC float CosTheta(const FW::Vec3f &w) { return w.z; }

FW_CUDA_FUNC float Cos2Theta(const FW::Vec3f &w) { return w.z * w.z; }

FW_CUDA_FUNC float Sin2Theta(const FW::Vec3f &w) {
    return fmaxf((float)0, (float)1 - Cos2Theta(w));
}

FW_CUDA_FUNC float SinTheta(const FW::Vec3f &w) { return std::sqrt(Sin2Theta(w)); }

FW_CUDA_FUNC float TanTheta(const FW::Vec3f &w) { return SinTheta(w) / CosTheta(w); }

FW_CUDA_FUNC float Tan2Theta(const FW::Vec3f &w) {
    return Sin2Theta(w) / Cos2Theta(w);
}

FW_CUDA_FUNC float CosPhi(const FW::Vec3f &w) {
    float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 1 : clampf(w.x / sinTheta, -1, 1);
}

FW_CUDA_FUNC float SinPhi(const FW::Vec3f &w) {
    float sinTheta = SinTheta(w);
    return (sinTheta == 0) ? 0 : clampf(w.y / sinTheta, -1, 1);
}

FW_CUDA_FUNC float Cos2Phi(const FW::Vec3f &w) { return CosPhi(w) * CosPhi(w); }

FW_CUDA_FUNC float Sin2Phi(const FW::Vec3f &w) { return SinPhi(w) * SinPhi(w); }


FW_CUDA_FUNC FW::Vec3f Reflect(const FW::Vec3f &wo, const FW::Vec3f &n) {
    return -wo + 2 * FW::dot(wo, n) * n;
}

FW_CUDA_FUNC bool Refract(const FW::Vec3f &wi, const FW::Vec3f &n, float eta,
                    FW::Vec3f *wt) {
    float cosThetaI = FW::dot(n, wi);
    float sin2ThetaI = fmaxf(float(0), float(1 - cosThetaI * cosThetaI));
    float sin2ThetaT = eta * eta * sin2ThetaI;

    if (sin2ThetaT >= 1) return false;
    float cosThetaT = std::sqrt(1 - sin2ThetaT);
    *wt = eta * -wi + (eta * cosThetaI - cosThetaT) * n;
    return true;
}

FW_CUDA_FUNC FW::Vec3f FresnelConductor(float cosThetaI, const FW::Vec3f &etai,
                     const FW::Vec3f &etat, const FW::Vec3f &k) {
    cosThetaI = clampf(cosThetaI, -1, 1);
    FW::Vec3f eta = etat / etai;
    FW::Vec3f etak = k / etai;

    float cosThetaI2 = cosThetaI * cosThetaI;
    float sinThetaI2 = 1. - cosThetaI2;
    FW::Vec3f eta2 = eta * eta;
    FW::Vec3f etak2 = etak * etak;

    FW::Vec3f t0 = eta2 - etak2 - sinThetaI2;
    FW::Vec3f a2plusb2 = t0 * t0 + eta2 * etak2 * 4;
    a2plusb2 = FW::Vec3f(sqrt(a2plusb2.x), sqrt(a2plusb2.y), sqrt(a2plusb2.z));
    FW::Vec3f t1 = a2plusb2 + cosThetaI2;
    FW::Vec3f a = 0.5f * (a2plusb2 + t0);
    a = FW::Vec3f(sqrt(a.x), sqrt(a.y), sqrt(a.z));
    FW::Vec3f t2 = (float)2 * cosThetaI * a;
    FW::Vec3f Rs = (t1 - t2) / (t1 + t2);

    FW::Vec3f t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
    FW::Vec3f t4 = t2 * sinThetaI2;
    FW::Vec3f Rp = Rs * (t3 - t4) / (t3 + t4);

    return (Rp + Rs) * 0.5;
}

FW_CUDA_FUNC float FresnelDielectric(float cosThetaI, float etaI, float etaT) {
    cosThetaI = clampf(cosThetaI, -1, 1);
    bool entering = cosThetaI > 0.f;
    if (!entering) {
        float temp;
        temp = etaI;
        etaI = etaT;
        etaT = temp;
        cosThetaI = abs(cosThetaI);
    }

    float sinThetaI = sqrt(fmax((float)0, 1 - cosThetaI * cosThetaI));
    float sinThetaT = etaI / etaT * sinThetaI;

    if (sinThetaT >= 1) return 1;
    float cosThetaT = sqrt(fmax((float)0, 1 - sinThetaT * sinThetaT));
    float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
                  ((etaT * cosThetaI) + (etaI * cosThetaT));
    float Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
                  ((etaI * cosThetaI) + (etaT * cosThetaT));
    return (Rparl * Rparl + Rperp * Rperp) / 2;
}

FW_CUDA_FUNC float BeckmannLambda(const FW::Vec3f &w,
                                 float alphax, float alphay) {
    float absTanTheta = abs(TanTheta(w));
    if (isinf(absTanTheta)) return 0.;
    float alpha =
        sqrt(Cos2Phi(w) * alphax * alphax + Sin2Phi(w) * alphay * alphay);
    float a = 1 / (alpha * absTanTheta);
    if (a >= 1.6f) return 0;
    return (1 - 1.259f * a + 0.396f * a * a) / (3.535f * a + 2.181f * a * a);
}

FW_CUDA_FUNC float BeckmannD(const FW::Vec3f &wh,
                             float alphax, float alphay) {
    float tan2Theta = Tan2Theta(wh);
    if (isinf(tan2Theta)) return 0.;
    float cos4Theta = Cos2Theta(wh) * Cos2Theta(wh);

    return exp(-tan2Theta * (Cos2Phi(wh) / (alphax * alphax) +
                                  Sin2Phi(wh) / (alphay * alphay))) /
           (PB_PI * alphax * alphay * cos4Theta);
}

FW_CUDA_FUNC float BeckmannG(const FW::Vec3f &wo, const FW::Vec3f &wi,
                             float alphax, float alphay) {
    return 1 /
           (1 + BeckmannLambda(wo, alphax, alphay) +
           BeckmannLambda(wi, alphax, alphay));
}

FW_CUDA_FUNC float BeckmannG1(const FW::Vec3f &w,
                             float alphax, float alphay) {
    return 1 / (1 + BeckmannLambda(w, alphax, alphay));
}

FW_CUDA_FUNC void BeckmannSample11(float cosThetaI, float U1, float U2,
                             float *slope_x, float *slope_y) {
    if (cosThetaI > .9999) {
        float r = sqrt(-log(1.0f - U1));
        float sinPhi = sin(2 * PB_PI * U2);
        float cosPhi = cos(2 * PB_PI * U2);
        *slope_x = r * cosPhi;
        *slope_y = r * sinPhi;
        return;
    }

    float sinThetaI =
        sqrt(fmaxf((float)0, (float)1 - cosThetaI * cosThetaI));
    float tanThetaI = sinThetaI / cosThetaI;
    float cotThetaI = 1 / tanThetaI;

    float a = -1, c = Erf(cotThetaI);
    float sample_x = fmaxf(U1, (float)1e-6f);

    float thetaI = acos(cosThetaI);
    float fit = 1 + thetaI * (-0.876f + thetaI * (0.4265f - 0.0594f * thetaI));
    float b = c - (1 + c) * powf(1 - sample_x, fit);

    float SQRT_PI_INV = 1.f / sqrt(PB_PI);
    float normalization =
        1 /
        (1 + c + SQRT_PI_INV * tanThetaI * std::exp(-cotThetaI * cotThetaI));

    int it = 0;
    while (++it < 10) {
        if (!(b >= a && b <= c)) b = 0.5f * (a + c);

        float invErf = ErfInv(b);
        float value =
            normalization *
                (1 + b + SQRT_PI_INV * tanThetaI * exp(-invErf * invErf)) -
            sample_x;
        float derivative = normalization * (1 - invErf * tanThetaI);

        if (abs(value) < 1e-5f) break;

        if (value > 0)
            c = b;
        else
            a = b;

        b -= value / derivative;
    }

    *slope_x = ErfInv(b);

    *slope_y = ErfInv(2.0f * fmaxf(U2, (float)1e-6f) - 1.0f);

    //CHECK(!std::isinf(*slope_x));
    //CHECK(!std::isnan(*slope_x));
    //CHECK(!std::isinf(*slope_y));
    //CHECK(!std::isnan(*slope_y));
}

FW_CUDA_FUNC FW::Vec3f BeckmannSample(const FW::Vec3f &wi, float alpha_x, float alpha_y,
                               float U1, float U2) {
    FW::Vec3f wiStretched =
        FW::Vec3f(alpha_x * wi.x, alpha_y * wi.y, wi.z).normalized();

    float slope_x, slope_y;
    BeckmannSample11(CosTheta(wiStretched), U1, U2, &slope_x, &slope_y);

    float tmp = CosPhi(wiStretched) * slope_x - SinPhi(wiStretched) * slope_y;
    slope_y = SinPhi(wiStretched) * slope_x + CosPhi(wiStretched) * slope_y;
    slope_x = tmp;

    slope_x = alpha_x * slope_x;
    slope_y = alpha_y * slope_y;

    return FW::Vec3f(-slope_x, -slope_y, 1.f).normalized();
}

FW_CUDA_FUNC FW::Vec3f BeckmannSample_wh(const FW::Vec3f &wo,
                                 const FW::Vec2f &u,
                                 float alphax, float alphay) {
    FW::Vec3f wh;
    bool flip = wo.z < 0;
    wh = BeckmannSample(flip ? -wo : wo, alphax, alphay, u[0], u[1]);
    if (flip) wh = -wh;
    return wh;
}

FW_CUDA_FUNC float BeckmannPdf(const FW::Vec3f &wo, const FW::Vec3f &wh,
                                 float alphax, float alphay) {
    //if (sampleVisibleArea)
        return BeckmannD(wh, alphax, alphay) *
                  BeckmannG1(wo, alphax, alphay) * AbsDot(wo, wh) / AbsCosTheta(wo);
    //else
    //    return BeckmannD(wh, alphax, alphay) * AbsCosTheta(wh);
}

FW_CUDA_FUNC float MicrofacetReflectionPdf(const FW::Vec3f &wo,
                                           const FW::Vec3f &wi,
                                           float alphax, float alphay) {
    if (!SameHemisphere(wo, wi)) return 0;
    FW::Vec3f wh = (wo + wi).normalized();
    return BeckmannPdf(wo, wh, alphax, alphay) / (4 * FW::dot(wo, wh));
}

FW_CUDA_FUNC float MicrofacetTransmissionPdf(
                                 const FW::Vec3f &wo, const FW::Vec3f &wi,
                                 float etaA, float etaB,
                                 float alphax, float alphay) {
    if (SameHemisphere(wo, wi)) return 0;
    float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    FW::Vec3f wh = (wo + wi * eta).normalized();

    float sqrtDenom = FW::dot(wo, wh) + eta * FW::dot(wi, wh);
    float dwh_dwi =
        abs((eta * eta * FW::dot(wi, wh)) / (sqrtDenom * sqrtDenom));
    return BeckmannPdf(wo, wh, alphax, alphay) * dwh_dwi;
}


FW_CUDA_FUNC FW::Vec3f MicrofacetTransmissionF(
                               const FW::Vec3f &wo, const FW::Vec3f &wi,
                               float etaA, float etaB,
                               FW::Vec3f T,
                               float alphax, float alphay) {
    float cosThetaO = CosTheta(wo);
    float cosThetaI = CosTheta(wi);

    if (cosThetaI == 0 || cosThetaO == 0) return FW::Vec3f(0);

    float eta = CosTheta(wo) > 0 ? (etaB / etaA) : (etaA / etaB);
    FW::Vec3f wh = (wo + wi * eta).normalized();
    if (wh.z < 0) wh = -wh;

    FW::Vec3f F = FresnelDielectric(FW::dot(wo, wh), etaA, etaB);

    float sqrtDenom = FW::dot(wo, wh) + eta * FW::dot(wi, wh);
    //float factor = (mode == TransportMode::Radiance) ? (1 / eta) : 1;
    float factor = 1 / eta;

    return (FW::Vec3f(1) - F) * T *
           abs(BeckmannD(wh, alphax, alphay) *
                    BeckmannG(wo, wi, alphax, alphay) * eta * eta *
                    AbsDot(wi, wh) * AbsDot(wo, wh) * factor * factor /
                    (cosThetaI * cosThetaO * sqrtDenom * sqrtDenom));
}


FW_CUDA_FUNC FW::Vec3f MicrofacetReflectionF(
                             const FW::Vec3f &wo, const FW::Vec3f &wi,
                             FW::Vec3f etaA, FW::Vec3f etaB,
                             FW::Vec3f R, FW::Vec3f k,
                             float alphax, float alphay) {
    float cosThetaO = AbsCosTheta(wo), cosThetaI = AbsCosTheta(wi);
    FW::Vec3f wh = wi + wo;
    if (cosThetaI == 0 || cosThetaO == 0) return FW::Vec3f(0.);
    if (wh.x == 0 && wh.y == 0 && wh.z == 0) return FW::Vec3f(0.);
    wh.normalize();
    FW::Vec3f F = FresnelConductor(FW::dot(wi, wh), etaA, etaB, k);
    return R * BeckmannD(wh, alphax, alphay) *
                      BeckmannG(wo, wi, alphax, alphay) * F /
                           (4 * cosThetaI * cosThetaO);
}



} // namespace rt_pathtracer

#endif // RTPT_INTEGRATORS_MICROFACET_H_
