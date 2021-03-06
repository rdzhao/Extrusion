#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

const double PI = 3.14159265358;

using namespace std;
using namespace glm;

class Extrusion
{
public:
	void readData(string fn); // read point and color list
	void setResolution(int r);
	void setRadius(float r);
	void writeOBJ();
	void writePLY(); // write ply model file

	void computeTube();

private:
	void assembleTmpData();
	void computeDisplacement();
	void computeVertices();
	void assembleFacets(); 
	void addArrowHead();

	void setVerticalVector(vec3 v, vec3& n);
	void parallelTransport(vec3 v1, vec3 v2, vec3 n1, vec3& n2);
	void rotate(vec3 axis, vec3 n1, float theta, vec3& n2);
private:
	int _resolution;
	float _radius;

	vector<int> _indices;
	vector<vec3> _polyline; // a series of points
	vector<vec3> _colors;

	int _currentIdx;
	int _currentVertexNum;
	int _tmpVertexNum;
	vector<vec3> _tmpPolyline;
	vector<vec3> _tmpColors;
	vector<vec3> _displacement; // line segment based

	vector<vec3> _vertices;
	vector<vec3> _tubeColors;
	vector<ivec3> _facets;
};
