#pragma once

#include <iso646.h>

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

#define EPSILON 0.001f

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[1].p - vertices[0].p;
		e2 = vertices[2].p - vertices[0].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p); 
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		// from o + tD = A + uE1 + vE2 we have o - A = uE1 + vE2 - tD
		// define o - A = S
		Vec3 s = r.o - vertices[0].p; 
		// we define a vector P orthogonal to E2, P = D x E2
		Vec3 p = r.dir.cross(e2);
		// S · P = uE1 · P + vE2 · P - tD · P = uE1 · P
		// So we have u = S·P/E1·P
		//define Δ = e1 · P  = e1 · (d x e2)
		//note this is also equal to e2.dot(e1 x d) or d.dot(e2 x e1)
		float del = e1.dot(p); 
		if (std::abs(del) < EPSILON )
			return false;
		float invDel = 1 / del;
		//find an expression whose denominator is the same scalar triple product Δ so we define a formulation 
		Vec3 q = s.cross(e1);
		//now we have Q = vE2 x e1 - t(D × e1)
		//take the dot product of both sides with D we have Q · D = vD · (E2 x E1) - tD · (D x E1) = vD · (E2 x E1)
		//now we have v = Q · D / D · (E2 x e1) =  Q · D / Δ
		//for t, we have Q · E2  = -t · E2 (D x E1) = t · E2 (E1 x D) then t = Q · E2 / Δ
		u = s.dot(p) * invDel;
		if (u < 0.0f || u > 1.0f)
			return false;
		v = q.dot(r.dir) * invDel;
		if (v<0.0f || u+v > 1.0f)
			return false;
		t = q.dot(e2) * invDel;
		if (t < EPSILON )
			return false;
		return true;
	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 far = Max((max - r.o) * r.invDir, (min - r.o) * r.invDir);
		Vec3 near = Min((max - r.o) * r.invDir, (min - r.o) * r.invDir);
		float entry = std::max({near.x, near.y, near.z});
		float exit = std::min({far.x, far.y, far.z});
		if (exit >= 0 && exit >= entry )
		{
			t = entry;
			return true;
		}
			
		return false;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{

		Vec3 far = Max((max - r.o) * r.invDir, (min - r.o) * r.invDir);
		Vec3 near = Min((max - r.o) * r.invDir, (min - r.o) * r.invDir);
		float entry = std::max({near.x, near.y, near.z});
		float exit = std::min({far.x, far.y, far.z});
		if (exit >= 0 && exit >= entry )
			return true;
		return false;
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	unsigned int offset;
	unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
		bounds.reset();
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, int offsetLeft, int offsetRight)
	{
		// Add BVH building code here
		AABB centroidBounds;
		bounds.reset();
		for (int i=0; i< offsetRight - offsetLeft; ++i )
		{
			const Triangle& tmp = inputTriangles[i + offsetLeft];
			bounds.extend(tmp.vertices[0].p);
			bounds.extend(tmp.vertices[1].p);
			bounds.extend(tmp.vertices[2].p);
			centroidBounds.extend(tmp.centre());
		}
		offset = offsetLeft;
		num = offsetRight - offsetLeft;
		if (offsetRight - offsetLeft <= MAXNODE_TRIANGLES )
				return;
		int mid ;
		
		float bestCost = FLT_MAX;
		int bestAxis = -1;
		int bestBin = -1;
		float parentArea = bounds.area();
		
		for (int axis = 0; axis < 3; ++axis)
		{
			int binCount[BUILD_BINS] = {};
			AABB binBounds[BUILD_BINS];

			float axisStart = centroidBounds.min.coords[axis];
			float axisEnd = centroidBounds.max.coords[axis];
			if (axisEnd - axisStart < EPSILON)
				continue;
			float scale  =BUILD_BINS / ( axisEnd - axisStart )  ;
			
			for (int i=0; i<offsetRight - offsetLeft; ++i )
			{ 
				const Triangle& tmp = inputTriangles[i + offsetLeft];
				int position;
				position = std::min ( static_cast<int>((tmp.centre().coords[axis] - axisStart) * scale),BUILD_BINS - 1 );
				binCount[ position ] ++;
				binBounds[position].extend(tmp.vertices[0].p);
				binBounds[position].extend(tmp.vertices[1].p);
				binBounds[position].extend(tmp.vertices[2].p);
			}
			
			AABB leftBounds[BUILD_BINS];
			AABB rightBounds[BUILD_BINS];

			int leftCount[BUILD_BINS] = {};
			int rightCount[BUILD_BINS] = {};

			AABB currentLeft;
			int currentLeftCount = 0;

			for (int i = 0; i < BUILD_BINS; ++i)
			{
				if (binCount[i] > 0)
				{
					currentLeft.extend(binBounds[i].min);
					currentLeft.extend(binBounds[i].max);
					currentLeftCount += binCount[i];
				}

				if (currentLeftCount > 0)
					leftBounds[i] = currentLeft;

				leftCount[i] = currentLeftCount;
			}

			AABB currentRight;
			int currentRightCount = 0;

			for (int i = BUILD_BINS - 1; i >= 0; --i)
			{
				if (binCount[i] > 0)
				{
					currentRight.extend(binBounds[i].min);
					currentRight.extend(binBounds[i].max);
					currentRightCount += binCount[i];
				}

				if (currentRightCount > 0)
					rightBounds[i] = currentRight;

				rightCount[i] = currentRightCount;
			}
			
			for (int i = 0; i < BUILD_BINS-1; ++i)
			{
				float tmpCost;
				if (leftCount[i] == 0 || rightCount[i+1] == 0)
					continue;
				
				tmpCost = TRAVERSE_COST + 
					TRIANGLE_COST * 
						(leftBounds[i].area()/ parentArea * (float)leftCount[i] 
							+ rightBounds[i+1].area()/ parentArea * (float)rightCount[i+1]);
				if (tmpCost<bestCost)
				{
					bestCost = tmpCost;
					bestAxis = axis;
					bestBin = i;
					// mid = offsetLeft + leftCount[i];
				}
			}
			
		}
		
		if (bestAxis == -1 || bestCost > (float)(offsetRight - offsetLeft) * TRIANGLE_COST)
			return;
		
		float axisStart = centroidBounds.min.coords[bestAxis];
		float axisEnd = centroidBounds.max.coords[bestAxis];
		float scale  =BUILD_BINS / ( axisEnd - axisStart )  ;
		
		auto partitionIt = std::partition(inputTriangles.begin() + offsetLeft, inputTriangles.begin() + offsetRight
			,[bestAxis,bestBin,axisStart,scale](const Triangle& t)
				{
					int position = static_cast<int>(
						(t.centre().coords[bestAxis] - axisStart) * scale
					);
					position = std::min ( position,BUILD_BINS - 1 );
					return position <= bestBin;
				}
			);
		
		mid = static_cast<int>(partitionIt - inputTriangles.begin());
		
		if (mid == offsetLeft || mid == offsetRight)
		{
			std::cout << "BVH degenerate partition: "
					  << "left=" << offsetLeft
					  << ", mid=" << mid
					  << ", right=" << offsetRight
					  << '\n';
		}
		
		BVHNode* leftNode = new BVHNode();
		BVHNode* rightNode = new BVHNode();		
		
		leftNode->build(inputTriangles, offsetLeft, mid);
		rightNode->build(inputTriangles, mid, offsetRight);
		l=leftNode;
		r=rightNode;
		
	}
	
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
		float t;
		if (!bounds.rayAABB(ray,t) ||  t >= intersection.t )
			return;
		
		if (l == nullptr && r == nullptr )
		{
			for (int i = 0; i < num; ++i)
			{
				
				float u;
				float v;
				if (triangles[offset+i].rayIntersect(ray, t, u, v) && t < intersection.t)
				{
					intersection.t = t;
					intersection.ID = offset+i;
					intersection.alpha = 1.0f - u - v;
					intersection.beta = u;
					intersection.gamma = v;
				}
			}
			return;
		}
		
		if (l != nullptr && r != nullptr)
		{
			float lt,rt;
			bool lflag = l->bounds.rayAABB(ray,lt);
			bool rflag = r->bounds.rayAABB(ray,rt);
			
			if (!lflag && !rflag)
				return;
			
			if (!lflag)
			{
				r->traverse(ray, triangles, intersection);
				return;
			}
			if (!rflag){
				l->traverse(ray, triangles, intersection);
				return;
			}
			
			BVHNode* near = l;
			BVHNode* far = r;
			
			if (rt < lt) std::swap(near,far);
			near->traverse(ray, triangles, intersection);
			far->traverse(ray, triangles, intersection);
				
		}
			
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		float t,u,v;
		if (bounds.rayAABB(ray,t) && t < maxT)
		{
			
			if (r==nullptr && l==nullptr)
			{
				for (int i = 0; i < num; ++i)
				{
					if (triangles[offset+i].rayIntersect(ray,t,u,v) && t < maxT)
						return false;
				}
			}
			else if (l==nullptr )
			{
				return r->traverseVisible(ray,triangles,maxT);
			}
			else if (r==nullptr )
			{
				return l->traverseVisible(ray,triangles,maxT);
			}
			else
			{
				return r->traverseVisible(ray,triangles,maxT) && l->traverseVisible(ray,triangles,maxT);
			}
		}
		return true;
	}
};
