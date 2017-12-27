#include "geometry/quadrics.h"

namespace rt_pathtracer {

// https://github.com/daliborgaric/pixie/blob/master/src/ri/quadrics.cpp

template <class T> FW_CUDA_FUNC	int SolveQuadric(T a,T b,T c,T *r)
{
	if (a != 0) {
		const double delta = b * b - 4 * a *c;

		if (delta < 0) return 0;
		else if (delta == 0) {
			r[0] = (T) (- b) / (2*a);
			return 1;
		} else {
			double sqrtDelta = sqrt(delta);

			r[0] = (T) (-sqrtDelta - b) / (2*a);
			r[1] = (T) (sqrtDelta - b) / (2*a);

			return 2;
		}
	} else if (b != 0) {
		r[0] = -c/b;
		return 1;
	} else {
		return 0;
	}
}


__device__ bool IntersectHyperboloid(const FW::Ray& r, const Quadric& q,
						Intersection& isect)
{
	float x1 = q.params[0];
	float y1 = q.params[1];
	float z1 = q.params[2];
	float x2 = q.params[3];
	float y2 = q.params[4];
	float z2 = q.params[5];

	FW::Vec3f p1(x1, y1, z1);
	FW::Vec3f p2(x2, y2, z2);

	float phimax = q.params[6];

	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float rtmin = ro.tmin;
	float rtmax = ro.tmax;


	unsigned int ns,i;
	float ts[2];
	FW::Vec3f P;
	float a, b, c;
	FW::Vec3f Nt;

	float dx = p2.x - p1.x;
	float dy = p2.y - p1.y;
	float dz = p2.z - p1.z;

	if (p1.z == p2.z) {
		ns = 1;
		ts[0] = (p1.z - o.z) / d.z;
	} else {
		float x;
		float y;
		float zmin, dq, dmin;

		if ((dx * dx + dy * dy) < TMIN) {
			x = p1.x;
			y = p1.y;
			zmin = p1.z;
		} else {

			float tmin = (-dx * p1.x - dy * p1.y)
						/ (dx * dx + dy * dy);
			x = p1.x + dx * tmin;
			y = p1.y + dy * tmin;
			zmin = p1.z + dz * tmin;
		}

		dmin = sqrt(x * x + y * y);

		if (fabs(p2.z - zmin) > fabs(p1.z - zmin)) {
			dq = sqrt((p2.x * p2.x + p2.y * p2.y) - dmin * dmin)
								/ (p2.z - zmin);
		} else {
			dq = sqrt((p1.x * p1.x + p1.y * p1.y) - dmin * dmin)
								/ (p1.z - zmin);
		}

		a = d.x * d.x + d.y * d.y - d.z * d.z * dq * dq;
		b = 2 * d.x * o.x + 2 * d.y * o.y
				- 2 * d.z * (o.z - zmin) * dq * dq;
		c = o.x * o.x + o.y * o.y - dmin * dmin
				- (o.z - zmin) * (o.z - zmin) * dq * dq;

		if (a == 0) {
			if (b == 0) return false;

			ns = 1;
			ts[0] = -c / b;
		} else {
			ns = SolveQuadric<float>(a, b, c, ts);
			if (ns == 0)
				return false;
		}
	}

	for (i = 0; i < ns; i++) {
		float t = ts[i];
		float x, y;
		float ustart;
		float u, v;

		if (t >= rtmax) return false;
		if (t <= rtmin) continue;

		P = o + d * t;

		if (p1.z < p2.z) {
			if (P.z < p1.z) continue;
			if (P.z > p2.z) continue;
			v = (P.z - p1.z) / (p2.z - p1.z);
		} else if (p1.z > p2.z) {
			if (P.z < p2.z) continue;
			if (P.z > p1.z) continue;
			v = (P.z - p1.z) / (p2.z - p1.z);
		} else {
			float r1 = sqrt(p1.x * p1.x + p1.y * p1.y);
			float r2 = sqrt(p2.x * p2.x + p2.y * p2.y);
			float r = sqrt(P.x * P.x + P.y * P.y);

			v = (r - r1) / (r2 - r1);
			if (v < 0) continue;

			if (v > 1) continue;
		}
		
		x = (float) (p1.x * (1.0 - v) + p2.x * (float)v);
		y = (float) (p1.y * (1.0 - v) + p2.y * (float)v);

		u = atan2(P.y,P.x);
		ustart = atan2(y,x);

		if (u < 0) u = 2 * M_PI + u;
		if (ustart < 0) ustart = 2 * M_PI + ustart;

		u -= ustart;
		if (u < 0) u = 2 * M_PI + u;
		if (phimax < 0) {
			u = u - 2 * M_PI;
			if (u < phimax) continue;
		} else {
			if (u > phimax) continue;
		}

		{
			Nt.x = (float) ((p2.z - p1.z) * phimax * P.x);
			Nt.y = (float) ((p2.z - p1.z) * phimax * P.y);
			Nt.z = (float) (-phimax * (( -1 + v) * p1.x * p1.x +
					p1.x * (p2.x - 2 * v * p2.x) +
					v * (p2.x * p2.x + (p2.y - p1.y) *
					(p2.y - p1.y)) + p1.y*(p2.y - p1.y)));
			//if (FW::dot(d, Nt) > 0) continue;
		}

		isect.uv = FW::Vec2f(u / phimax, v);
		isect.n = q.basis.transformV(Nt).normalized();
		isect.p = q.basis.transformP(P);
		isect.t = (isect.p - r.origin).length();
		isect.material_id = q.material_id;

		return true;
	}

	return false;
}


// https://bitbucket.org/luxrender/lux/src/shapes/torus.cpp


FW_CUDA_FUNC int SignR(double Z)
{
	if (Z > 0.0) return 1;
	if (Z < 0.0) return -1;
	
	return 0;
}

FW_CUDA_FUNC double CBRT(double Z)
{
	double ret;
	const double THIRD = 1./3.;
	ret = pow(fabs(Z),THIRD) * SignR(Z);
	return ret;
}

FW_CUDA_FUNC int Cubic(double A[4], double X[3])
{
	const double PI = 3.1415926535897932;
	const double THIRD = 1./3.;
	double W, P, Q, DIS, PHI;
	int L;

	if (A[3] != 0.0) {
		W = A[2] / A[3] * THIRD;
		double W2 = W * W;
		double P1 = -(A[1] / A[3] * THIRD - W2);
		P = P1 * P1 * P1;
		Q = -.5 * (2.0 * W2 * W-(A[1] * W-A[0]) / A[3]);
		DIS = Q * Q - P;

		if (DIS < 0.0) {
			PHI = acos(fmin(1.0, fmax(-1.0, Q / sqrt(P))));
			P = 2.0 * sqrt(P1);
			for (int i = 0; i < 3; i++)
				X[i] = P *
					cos((PHI + 2 * ((double)i) * PI)
						* THIRD) - W;
			L = 3;
		}
		else {
			DIS = sqrt(DIS);
			X[0] = CBRT(Q+DIS)+CBRT(Q-DIS)-W;
			L = 1;
		}
	}
	else if (A[2] != 0.0) {
		P = 0.5 * A[1] / A[2];
		DIS = P * P - A[0] / A[2];
		if (DIS > 0.0) {
			DIS = sqrt(DIS);
			X[0] = -P - DIS;
			X[1] = -P + DIS;
			L = 2;
		}
		else {
			return 0;
		}
	}
	else if (A[1] != 0.0) {
		X[0] = A[0] / A[1];
		L = 1;
	}
	else {
		return 0;
	}
 
	for (int i = 0; i < L; i++) {
		X[i] -= (A[0] + X[i] * (A[1] + X[i] * (A[2] + X[i] * A[3])))
			/ (A[1] + X[i] * (2.0 * A[2] + X[i] * 3.0 * A[3]));
	}

	return L;
}

FW_CUDA_FUNC int Quartic(double dd[5], double sol[4])
{
	double AA[4], z[3];
	double a, b, c, d, f, p, q, r, zsol, xK2, xL, xK, sqp, sqm;
	int ncube;
	int Nsol = 0;

	if (dd[4] == 0.0)
	{
		return 0;
	}

	a = dd[4];
	b = dd[3];
	c = dd[2];
	d = dd[1];
	f = dd[0];

	double aa = a * a;
	double bb = b * b;

	p = (-3.0 * b * b + 8.0 * a * c) / (8.0 * aa);
	q = (bb * b - 4.0 * a * b * c + 8.0 * d * aa) / (8.0 * aa * a);
	r = (-3.0 * bb * bb + 16.0 * a * bb * c -
		64.0 * aa * b * d + 256.0 * aa * a * f) / (256.0*aa*aa);
	
	AA[3] = 8.0;
	AA[2] = -4.0 * p;
	AA[1] = -8.0 * r;
	AA[0] = 4.0 * p * r - q * q;

	ncube = Cubic(AA, z);
	
	zsol = -1.e99;
	for (int i = 0; i < ncube; i++)
		zsol = fmax(zsol, z[i]);

	xK2 = 2.0 * zsol - p;
	xK = sqrt(xK2);
	xL = q / (2.0 * xK);
	sqp = xK2 - 4.0 * (zsol + xL);
	sqm = xK2 - 4.0 * (zsol - xL);

	if (sqp >= 0.0) {
		sqp = sqrt(sqp);

		if (sqm >= 0.0) {
			sqm = sqrt(sqm);
			sol[3] = 0.5 * (xK + sqp);
			sol[2] = 0.5 * (xK - sqp);
			sol[1] = 0.5 * (-xK + sqm);
			sol[0] = 0.5 * (-xK - sqm);
			Nsol = 4;
		}
		else {
			sol[1] = 0.5 * (xK + sqp);
			sol[0] = 0.5 * (xK - sqp);
			Nsol = 2;
		}
	}
	else {
		if (sqm < 0.0)
			return 0;

		sqm = sqrt(sqm);

		sol[1] = 0.5 * (-xK + sqm);
		sol[0] = 0.5 * (-xK - sqm);
		Nsol = 2;
	}
	
	for (int i = 0; i < Nsol; i++)
		sol[i] -= b/(4.0*a);

	return Nsol;
}


__device__ bool IntersectTorus(const FW::Ray& r, const Quadric& q,
						Intersection& isect) {
 
	float rmajor = q.params[0];
	float rminor = q.params[1];
	float thetamin = q.params[2];
	float thetamax = q.params[3];
	float phimax = q.params[4];

	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float rtmin = ro.tmin;
	float rtmax = ro.tmax;


	double r2 = rminor * rminor;
	double R2 = rmajor * rmajor;

	double dd = FW::dot(d, d);
	double pd = FW::dot(o, d);
	double pp = FW::dot(o, o);
	double prR = pp - r2 - R2;

	double coef[5];

	coef[4] = dd * dd;
	coef[3] = 4.0 * dd * pd;
	coef[2] = 4.0 * pd * pd + 2.0 * dd * prR + 4.0 * R2 * d.z * d.z;
	coef[1] = 4.0 * pd * prR + 8.0 * R2 * o.z * d.z;
	coef[0] = prR * prR + 4.0 * R2 * (o.z * o.z - r2);

	double t[4];
	int Nsol;

	Nsol = Quartic(coef, t);
	if (Nsol < 1)
		return false;

	float tmax = t[Nsol-1];
	if (tmax < rtmin)
		return false;

	float thit = t[0];
	int ti = 0;
	while (thit < rtmin) {
		ti++;
		if (ti >= Nsol)
			return false;
		thit = t[ti];
	}
	if (thit > rtmax)
		return false;

	FW::Vec3f phit;
	float phi, theta;

	while (true) {
		phit = o + d * thit;
		phi = atan2f(phit.y, phit.x);
		if (phi < 0.f) phi += 2.f * M_PI;
		
		float sintheta = FW::clamp(phit.z / rminor, -1.f, 1.f);
		theta = asinf(sintheta);

		if (phit.x * phit.x + phit.y * phit.y < R2)
			theta = M_PI - theta;

		//if (theta < 0.f)
		//if (thetamin >= 0 && theta < 0.f)
		if (theta < thetamin)
			theta += 2.f * M_PI;

		if (!(theta < thetamin || theta > thetamax || phi > phimax))
			break;

		ti++;
		if (ti >= Nsol)
			return false;
		thit = t[ti];
		if (thit > rtmax) 
			return false;
	}

	float costheta = cosf(theta);
	phit.x = cosf(phi) * (rmajor + rminor * costheta);
	phit.y = sinf(phi) * (rmajor + rminor * costheta);
	phit.z = rminor * sinf(theta);

	isect.p = q.basis.transformP(phit);
	isect.t = (isect.p - r.origin).length();
	isect.material_id = q.material_id;

	float u = phi / phimax;
	float v = (theta - thetamin) / (thetamax - thetamin);
	isect.uv = FW::Vec2f(u, v);

	float zradius = sqrtf(phit.x * phit.x + phit.y * phit.y);

	FW::Vec3f dpdv, dpdu;
	float cosphi, sinphi;
	if (zradius == 0)
	{
		cosphi = 0;
		sinphi = 1;
		dpdv = (thetamax - thetamin) *
			FW::Vec3f(-phit.z * cosphi, -phit.z * sinphi,
				rminor * costheta);
		FW::Vec3f norm = phit;
		dpdu = FW::cross(dpdv, norm);
	}
	else
	{
		float invzradius = 1.f / zradius;
		cosphi = phit.x * invzradius;
		sinphi = phit.y * invzradius;
		dpdu = FW::Vec3f(-phimax * phit.y, phimax * phit.x, 0);
		dpdv = (thetamax - thetamin) *
			FW::Vec3f(-phit.z * cosphi, -phit.z * sinphi,
				rminor * costheta);
	}

	FW::Vec3f du = q.basis.transformV(dpdu);
	FW::Vec3f dv = q.basis.transformV(dpdv);
	FW::Vec3f n = FW::cross(du, dv);
	isect.n = n.normalized();
	isect.du = du.normalized();

	return true;
}

} // namespace rt_pathtracer
