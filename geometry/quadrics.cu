#include "geometry/quadrics.h"

namespace rt_pathtracer {

FW_CUDA_FUNC FW::Vec2f RayBox(const FW::Vec3f& mn,
			const FW::Vec3f& mx, const FW::Ray& ray)
{
    const FW::Vec3f& orig = ray.origin;
    const FW::Vec3f& dir  = ray.direction;

    FW::Vec3f t0 = (mn - orig) / dir;
    FW::Vec3f t1 = (mx - orig) / dir;

    float tmin = FW::min(t0,t1).max();
    float tmax = FW::max(t0,t1).min();

    return FW::Vec2f(tmin, tmax);
}


FW_CUDA_FUNC bool IntersectQuadric(const Quadric& q, const FW::Ray& r,
							Intersection& isect)
{
	switch (q.type) {
	case kqSphere:
		return IntersectSphere(r, q, isect);
	case kqCone:
		return IntersectCone(r, q, isect);
	case kqCylinder:
		return IntersectCylinder(r, q, isect);
	case kqHyperboloid:
		return IntersectHyperboloid(r, q, isect);
	case kqParaboloid:
		return IntersectParaboloid(r, q, isect);
	case kqDisk:
		return IntersectDisk(r, q, isect);
	case kqTorus:
		return IntersectTorus(r, q, isect);
	}
	return false;
}


struct Tval {
	int i;
	float t;
};


FW_CUDA_FUNC void InsertionSort(Tval arr[], int length) {
	int i, j;
	Tval tmp;
	for (i = 1; i < length; i++) {
		j = i;
		while (j > 0 && arr[j - 1].t > arr[j].t) {
			tmp = arr[j];
			arr[j] = arr[j - 1];
			arr[j - 1] = tmp;
			j--;
		}
	}
}

__device__ bool IntersectQuadrics(FW::Ray& r,
				Quadric* quadrics,
				int quadrics_count,
				Intersection& isect)
{
	if (quadrics_count < 1)
		return false;
	if (quadrics_count == 1) {
		FW::Vec2f t = RayBox(quadrics[0].b_min, quadrics[0].b_max, r);
		if (t.x <= t.y) {
			return IntersectQuadric(quadrics[0], r, isect);
		} else {
			return false;
		}
	}

	Tval items[MAX_VISIBLE_QUADRICS];
	int length = 0;
	for (int i = 0; i < quadrics_count; i++) {
		if (i > MAX_VISIBLE_QUADRICS)
			break;
		FW::Vec2f t = RayBox(quadrics[i].b_min, quadrics[i].b_max, r);
		if (t.x <= t.y) {
			Tval tv;
			tv.t = t.x;
			tv.i = i;
			items[length] = tv;
			length++;
		}
	}

	float min_t = TMAX;

	if (length > 0) {
		//IntersectQuadric(quadrics[0], r, isect);
		//min_t = 0;

		InsertionSort(items, length);
		Intersection test_isect;
		test_isect.t = TMAX;
		int idx;
		for (int i = 0; i < length; i++) {
			idx = items[i].i;
			if (items[i].t < min_t &&
			    IntersectQuadric(quadrics[idx], r, test_isect) &&
			    test_isect.t < min_t) {
				isect = test_isect;
				min_t = test_isect.t;
			}
		}

	}

	return min_t < TMAX;
}


FW_CUDA_FUNC bool Quadratic(float A, float B, float C, float *t0, float *t1)
{
	float discrim = B * B - 4.f * A * C;
	if (discrim < 0.) return false;
	float root_discrim = sqrtf(discrim);

	float q;
	if (B < 0) q = -.5f * (B - root_discrim);
	else       q = -.5f * (B + root_discrim);
	*t0 = q / A;
	*t1 = C / q;
	if (*t0 > *t1) {
		float temp = *t0;
		*t0 = *t1;
		*t1 = temp;
	}
	return true;
}

__device__ bool IntersectSphere(const FW::Ray& r, const Quadric& q,
						Intersection& isect)
{
	float radius = q.params[0];
	float zmin = q.params[1];
	float zmax = q.params[2];
	float phimax = q.params[3];
	float thetamin = q.params[4];
	float thetamax = q.params[5];
	
	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float tmin = ro.tmin;
	float tmax = ro.tmax;

	FW::Vec3f phit;

	float A = d.x * d.x + d.y * d.y + d.z * d.z;
	float B = 2 * (d.x * o.x + d.y * o.y + d.z * o.z);
	float C = o.x * o.x + o.y * o.y + o.z * o.z - radius * radius;

	float t0, t1;
	if (!Quadratic(A, B, C, &t0, &t1))
		return false;

	if (t0 > tmax || t1 < tmin)
		return false;
	float thit = t0;
	if (t0 < tmin) {
		thit = t1;
		if (thit > tmax) return false;
	}

	phit = o + d * thit;
	if (phit.x == 0.f && phit.y == 0.f) phit.x = 1e-5f * radius;
	float phi = atan2f(phit.y, phit.x);
	if (phi < 0.) phi += 2.f * M_PI;

	if ((zmin > -radius && phit.z < zmin) ||
	    (zmax <  radius && phit.z > zmax) || phi > phimax) {
		if (thit == t1) return false;
		if (t1 > tmax) return false;
		thit = t1;
		phit = o + d * thit;
		if (phit.x == 0.f && phit.y == 0.f) phit.x = 1e-5f * radius;
		phi = atan2f(phit.y, phit.x);
		if (phi < 0.) phi += 2.f*M_PI;
		if ((zmin > -radius && phit.z < zmin) ||
		   (zmax <  radius && phit.z > zmax) || phi > phimax)
			return false;
	}

	float u = phi / phimax;
	float theta = acosf(FW::clamp(phit.z / radius, -1.f, 1.f));
	float v = (theta - thetamin) / (thetamax - thetamin);
	isect.uv = FW::Vec2f(u, v);

	float zradius = sqrtf(phit.x * phit.x + phit.y * phit.y);
	float invzradius = 1.f / zradius;
	float cosphi = phit.x * invzradius;
	float sinphi = phit.y * invzradius;
	FW::Vec3f dpdu(-phimax * phit.y, phimax * phit.x, 0);
	FW::Vec3f dpdv = (thetamax - thetamin) *
			FW::Vec3f(phit.z * cosphi, phit.z * sinphi,
			-radius * sinf(theta));

	FW::Vec3f du = q.basis.transformV(dpdu);
	FW::Vec3f dv = q.basis.transformV(dpdv);
	FW::Vec3f n = FW::cross(du, dv);
	isect.n = n.normalized();
	isect.du = du.normalized();

	isect.p = q.basis.transformP(phit);
	isect.t = (isect.p - r.origin).length();
	isect.material_id = q.material_id;
	return true;
}


__device__ bool IntersectCone(const FW::Ray& r, const Quadric& q,
						Intersection& isect)
{
	float height = q.params[0];
	float radius = q.params[1];
	float phimax = q.params[2];
	
	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float tmin = ro.tmin;
	float tmax = ro.tmax;

	float phi;
	FW::Vec3f phit;

	float k = radius / height;
	k = k * k;
	float A = d.x * d.x + d.y * d.y - k * d.z * d.z;
	float B = 2 * (d.x * o.x + d.y * o.y - k * d.z * (o.z - height));
	float C = o.x * o.x + o.y * o.y - k * (o.z - height) * (o.z - height);

	float t0, t1;
	if (!Quadratic(A, B, C, &t0, &t1))
		return false;

	if (t0 > tmax || t1 < tmin)
		return false;
	float thit = t0;
		if (t0 < tmin) {
		thit = t1;
		if (thit > tmax) return false;
	}

	phit = o + d * thit;
	phi = atan2f(phit.y, phit.x);
	if (phi < 0.) phi += 2.f * M_PI;

	if (phit.z < 0 || phit.z > height || phi > phimax) {
		if (thit == t1) return false;
		thit = t1;
		if (t1 > tmax) return false;
		phi = atan2f(phit.y, phit.x);
		phit = o + d * thit;
		phi = atan2f(phit.y, phit.x);
		if (phi < 0.) phi += 2.f*M_PI;
		if (phit.z < 0 || phit.z > height || phi > phimax)
			return false;
	}

	float u = phi / phimax;
	float v = phit.z / height;
	isect.uv = FW::Vec2f(u, v);

	FW::Vec3f dpdu(-phimax * phit.y, phimax * phit.x, 0);
	FW::Vec3f dpdv(-phit.x / (1.f - v), -phit.y / (1.f - v), height);

	FW::Vec3f du = q.basis.transformV(dpdu);
	FW::Vec3f dv = q.basis.transformV(dpdv);
	FW::Vec3f n = FW::cross(du, dv);
	isect.n = n.normalized();
	isect.du = du.normalized();
	
	isect.p = q.basis.transformP(phit);
	isect.t = (isect.p - r.origin).length();
	isect.material_id = q.material_id;

	return true;
}


__device__ bool IntersectCylinder(const FW::Ray& r, const Quadric& q,
						Intersection& isect)
{
	float radius = q.params[0];
	float zmin = q.params[1];
	float zmax = q.params[2];
	float phimax = q.params[3];

	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float tmin = ro.tmin;
	float tmax = ro.tmax;

	float phi;
	FW::Vec3f phit;

	float A = d.x * d.x + d.y * d.y;
	float B = 2 * (d.x * o.x + d.y * o.y);
	float C = o.x * o.x + o.y * o.y - radius * radius;

	float t0, t1;
	if (!Quadratic(A, B, C, &t0, &t1))
		return false;

	if (t0 > tmax || t1 < tmin)
		return false;
	float thit = t0;
	if (t0 < tmin) {
		thit = t1;
		if (thit > tmax) return false;
	}

	phit = o + d * thit;
	phi = atan2f(phit.y, phit.x);
	if (phi < 0.) phi += 2.f * M_PI;

	if (phit.z < zmin || phit.z > zmax || phi > phimax) {
		if (thit == t1) return false;
		thit = t1;
		if (t1 > tmax) return false;
		phit = o + d * thit;
		phi = atan2f(phit.y, phit.x);
		if (phi < 0.) phi += 2.f * M_PI;
		if (phit.z < zmin || phit.z > zmax || phi > phimax)
			return false;
	}

	float u = phi / phimax;
	float v = (phit.z - zmin) / (zmax - zmin);
	isect.uv = FW::Vec2f(u, v);

	FW::Vec3f dpdu(-phimax * phit.y, phimax * phit.x, 0);
	FW::Vec3f dpdv(0, 0, zmax - zmin);

	FW::Vec3f du = q.basis.transformV(dpdu);
	FW::Vec3f dv = q.basis.transformV(dpdv);
	FW::Vec3f n = FW::cross(du, dv);
	isect.n = n.normalized();
	isect.du = du.normalized();

	isect.p = q.basis.transformP(phit);
	isect.t = (isect.p - r.origin).length();
	isect.material_id = q.material_id;

	return true;
}


__device__ bool IntersectDisk(const FW::Ray& r, const Quadric& q,
						Intersection& isect)
{
	float height = q.params[0];
	float radius = q.params[1];
	float phimax = q.params[2];

	float inner_radius = 0;
	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float tmin = ro.tmin;
	float tmax = ro.tmax;

	FW::Vec3f phit;

	if (fabsf(d.z) < 1e-7) return false;
	float thit = (height - o.z) / d.z;
	if (thit < tmin || thit > tmax)
		return false;

	phit = o + d * thit;
	float dist2 = phit.x * phit.x + phit.y * phit.y;
	if (dist2 > radius * radius || dist2 < inner_radius * inner_radius)
		return false;

	float phi = atan2f(phit.y, phit.x);
	if (phi < 0) phi += 2. * M_PI;
	if (phi > phimax)
		return false;

	float u = phi / phimax;
	float R = sqrtf(dist2);
	float one_minus_v = ((R - inner_radius) / (radius - inner_radius));
	float v = 1.f - one_minus_v;
	FW::Vec3f dpdu(-phimax * phit.y, phimax * phit.x, 0.);
	FW::Vec3f dpdv(phit.x, phit.y, 0.);
	dpdv *= (inner_radius - radius) / R;

	isect.uv = FW::Vec2f(u, v);
	
	FW::Vec3f du = q.basis.transformV(dpdu);
	FW::Vec3f dv = q.basis.transformV(dpdv);
	FW::Vec3f n = FW::cross(du, dv);
	isect.n = n.normalized();
	isect.du = du.normalized();

	isect.p = q.basis.transformP(phit);
	isect.t = (isect.p - r.origin).length();
	isect.material_id = q.material_id;

	return true;
};


__device__ bool IntersectParaboloid(const FW::Ray& r, const Quadric& q,
						Intersection& isect)
{
	float radius = q.params[0];
	float zmin = q.params[1];
	float zmax = q.params[2];
	float phimax = q.params[3];

	FW::Ray ro = RayToObject(r, q);
	FW::Vec3f o = ro.origin;
	FW::Vec3f d = ro.direction;
	float tmin = ro.tmin;
	float tmax = ro.tmax;

	float phi;
	FW::Vec3f phit;

	float k = zmax/ (radius * radius);
	float A = k * (d.x * d.x + d.y * d.y);
	float B = 2 * k * (d.x * o.x + d.y * o.y) - d.z;
	float C = k * (o.x * o.x + o.y * o.y) - o.z;

	float t0, t1;
	if (!Quadratic(A, B, C, &t0, &t1))
		return false;

	if (t0 > tmax || t1 < tmin)
		return false;
	float thit = t0;
	if (t0 < tmin) {
		thit = t1;
		if (thit > tmax) return false;
	}

	phit = o + d * thit;
	phi = atan2f(phit.y, phit.x);
	if (phi < 0.) phi += 2.f * M_PI;

	if (phit.z < zmin || phit.z > zmax || phi > phimax) {
		if (thit == t1) return false;
		thit = t1;
		if (t1 > tmax) return false;
		phit = o + d * thit;
		phi = atan2f(phit.y, phit.x);
		if (phi < 0.) phi += 2.f*M_PI;
		if (phit.z < zmin || phit.z > zmax || phi > phimax)
			return false;
	}

	float u = phi / phimax;
	float v = (phit.z-zmin) / (zmax-zmin);

	FW::Vec3f dpdu(-phimax * phit.y, phimax * phit.x, 0.);
	FW::Vec3f dpdv = (zmax - zmin) *
		FW::Vec3f(phit.x / (2.f * phit.z), phit.y / (2.f * phit.z), 1.);

	isect.uv = FW::Vec2f(u, v);

	FW::Vec3f du = q.basis.transformV(dpdu);
	FW::Vec3f dv = q.basis.transformV(dpdv);
	FW::Vec3f n = FW::cross(du, dv);
	isect.n = n.normalized();
	isect.du = du.normalized();

	isect.p = q.basis.transformP(phit);
	isect.t = (isect.p - r.origin).length();
	isect.material_id = q.material_id;


	return true;
}


} // namespace rt_pathtracer

