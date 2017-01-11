#ifndef POINTCLOUDDATA_H
#define POINTCLOUDDATA_H

#include "Globals.h"
#include "Vector.h"
#include "Matrix.h"
#include "Camera.h"

class PointCloudData
{
public:
    PointCloudData(){}
    PointCloudData(const Camera& camera, const Matrix<double>& depth_map);

    Vector<double> X, Y, Z;
    int number_of_points;
};

#endif // POINTCLOUDDATA_H
