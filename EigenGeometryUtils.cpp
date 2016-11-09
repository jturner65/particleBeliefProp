#include "EigenGeometryUtils.h"
using namespace Eigen;
namespace AaltoGames
{

	Vector3f closestPointToTwoLines( const Vector3f &P0, const Vector3f &u, const Vector3f &Q0, const Vector3f &v )
	{
		Vector3f w0=P0-Q0;
		float a=u.dot(u);
		float b=u.dot(v);
		float c=v.dot(v);
		float d=u.dot(w0);
		float e=v.dot(w0);
		float s=(b*e-c*d)/(a*c-b*b);
		float t=(a*e-b*d)/(a*c-b*b);
		Vector3f result=0.5f*(P0 + s*u + Q0 + t*v); 
		return result;
	}

	Eigen::Vector3f linePlaneIntersection( const Eigen::Vector3f &linePoint, const Eigen::Vector3f &lineDir, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal )
	{
		//source: http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
		float d=(planePoint-linePoint).dot(planeNormal)/lineDir.dot(planeNormal);
		return linePoint+d*lineDir;
	}

}