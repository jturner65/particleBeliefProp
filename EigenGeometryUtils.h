#pragma once
#include <Eigen/Geometry> 

namespace AaltoGames
{


static Eigen::Vector3f closestPointOnRay(Eigen::Vector3f pt, Eigen::Vector3f rayOrigin, Eigen::Vector3f rayDir)
{
	return rayOrigin+(pt-rayOrigin).dot(rayDir)*rayDir;
}

//Lines L1 and L2 defined as: L1=P0+su, L2=Q0+tv, where u,v are the direction vectors, and s,t are scalars
//See: http://geomalgorithms.com/a07-_distance.html
//L1=P0+su, L2=Q0+tv, where u,v are the direction vectors, and s,t are scalars
//Let a=u*u, b=u*v,c=v*v,d=u*w0,e=v*w0, where w0=P0-Q0
//=> s=(be-cd)/(ac-b^2), t=(ae-bd)/(ac-b^2)
Eigen::Vector3f closestPointToTwoLines(const Eigen::Vector3f &P0, const Eigen::Vector3f &u, const Eigen::Vector3f &Q0, const Eigen::Vector3f &v);

Eigen::Vector3f linePlaneIntersection(const Eigen::Vector3f &linePoint, const Eigen::Vector3f &lineDir, const Eigen::Vector3f &planePoint, const Eigen::Vector3f &planeNormal);

void addEigenMatrixRow(Eigen::MatrixXf &m);


} //AaltoGames