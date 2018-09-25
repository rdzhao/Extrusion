#include "Extrusion.h"

void Extrusion::readData(string fn)
{
	ifstream file(fn.c_str());

	string str;
	while (getline(file, str)) {
		stringstream ss(str);

		float x, y, z;
		float r, g, b;

		ss >> x >> y >> z;
		ss >> r >> g >> b;

		_polyline.push_back(vec3(x, y, z));
		_colors.push_back(vec3(r, g, b));

		//cout << x << " " << y << " " << z << endl;
		//cout << r << " " << g << " " << b << endl;
	}
}

void Extrusion::setResolution(int r)
{
	_resolution = r;
}

void Extrusion::setRadius(float r)
{
	_radius = r;
}

void Extrusion::writeOBJ() 
{
	ofstream out("tube.obj");

	for (int i = 0; i < _vertices.size(); ++i) {
		out << "v "
			<< _vertices[i].x << " "
			<< _vertices[i].y << " "
			<< _vertices[i].z << endl;
	}

	for (int i = 0; i < _facets.size(); ++i) {
		out << "f "
			<< _facets[i].x + 1 << " "
			<< _facets[i].y + 1 << " "
			<< _facets[i].z + 1 << endl;
	}
}

void Extrusion::computeTube()
{
	computeDisplacement();
	computeVertices();
	assembleFacets();
}

void Extrusion::computeDisplacement()
{
	for (int i = 0; i < _polyline.size() - 1; ++i) {
		if (i == 0) { // set first one
			vec3 v = _polyline[i + 1] - _polyline[i];
			vec3 n;
			setVerticalVector(v, n);

			_displacement.push_back(n);
			//cout << n.x<<" " << n.y << " " << n.z << endl;
		}
		else { // parallel transport the rest
			vec3 v1 = _polyline[i] - _polyline[i-1];
			vec3 v2 = _polyline[i + 1] - _polyline[i];
			vec3 n1 = _displacement[i - 1];
			vec3 n2;
			parallelTransport(v1, v2, n1, n2);

			_displacement.push_back(n2);
			cout << n2.x<<" " << n2.y << " " << n2.z << endl;
		}

	}
}

void Extrusion::setVerticalVector(vec3 v, vec3& n)
{
	// set
	if (v.x != 0) {
		n = vec3(-v.y, v.x, 0);
	}
	else {
		n = vec3(0, -v.z, v.y);
	}

	// normalize
	n /= sqrt(dot(n, n));
}

void Extrusion::parallelTransport(vec3 v1, vec3 v2, vec3 n1, vec3& n2)
{
	//cout << v1.x << " " << v1.y << " " << v1.z << endl;
	//cout << v2.x << " " << v2.y << " " << v2.z << endl;
	//cout << n1.x << " " << n1.y << " " << n1.z << endl;

	vec3 vt = cross(v1, v2) / (sqrt(dot(v1, v1))*sqrt(dot(v2, v2)));
	float s = sqrt(dot(vt, vt));

	if (s < 0.000001) {
		n2 = n1;
	}
	else {
		// get angle
		vec3 f1, f2, f3;
		//cout<< sqrt(dot(v1, v1)) <<endl;
		f1 = v1 / sqrt(dot(v1, v1));
		f3 = cross(v1, v2);
		f3 /= sqrt(dot(f3, f3));
		f2 = cross(f3, f1);
		//cout << f1.x << " " << f1.y << " " << f1.z << endl;
		//cout << f2.x << " " << f2.y << " " << f2.z << endl;
		//cout << f3.x << " " << f3.y << " " << f3.z << endl;

		float angle = atan2(dot(cross(v1, v2), f3), dot(v1, v2));
		//cout<<angle<<endl;
		mat4x4 rm = glm::rotate(angle, f3);
		vec4 nn1 = vec4(n1, 1);
		vec4 nn2 = rm * nn1;
		n2 = vec3(nn2.x, nn2.y, nn2.z);
	}
}

void Extrusion::rotate(vec3 axis, vec3 n1, float theta, vec3& n2)
{
	mat4x4 rm = glm::rotate(theta, axis);
	
	vec4 nn1 = vec4(n1, 1);
	vec4 nn2 = rm * nn1;
	n2 = vec3(nn2.x, nn2.y, nn2.z);
}

void Extrusion::computeVertices()
{
	float theta = 2 * PI / _resolution;

	for (int i = 0; i < _polyline.size(); ++i) {
		if (i == 0) {
			vec3 v = _polyline[i + 1] - _polyline[i];
			vec3 n = _displacement[0];

			int k = 0;
			do {
				_vertices.push_back(_polyline[i] + _radius*n);

				rotate(v, n, theta, n);
				++k;
			} while (k < _resolution);
		}
		else if (i == _polyline.size() - 1) {
			vec3 v = _polyline[i] - _polyline[i - 1];
			vec3 n = _displacement[i - 1];

			int k = 0;
			do {
				_vertices.push_back(_polyline[i] + _radius * n);

				rotate(v, n, theta, n);
				++k;
			} while (k < _resolution);
		}
		else
		{
			vec3 v1 = _polyline[i] - _polyline[i - 1];
			vec3 v2 = _polyline[i + 1] - _polyline[i];
			vec3 n1 = _displacement[i - 1];
			vec3 n2 = _displacement[i];

			int k = 0;
			do {
				vec3 n = n1 + n2;
				n /= sqrt(dot(n, n));
				_vertices.push_back(_polyline[i] + _radius * n);

				rotate(v1, n1, theta, n1);
				rotate(v2, n2, theta, n2);
				++k;
			} while (k < _resolution);
		}
	}

	//cout<<"************************"<<endl;
	//for (int i = 0; i < _vertices.size(); ++i) {
	//	cout<<_vertices[i].x<<" " << _vertices[i].y << " " << _vertices[i].z <<endl;
	//}

	// two end vertices;
	_vertices.push_back(_polyline[0]);
	_vertices.push_back(_polyline[_polyline.size() - 1]);
}

void Extrusion::assembleFacets()
{
	for (int i = 0; i < _polyline.size() - 1; ++i) {
		for (int j = 0; j < _resolution; ++j) {
			_facets.push_back(ivec3(i*_resolution + j,
									i*_resolution + (j + 1) % _resolution,
									(i + 1)*_resolution + (j + 1) % _resolution));
			_facets.push_back(ivec3(i*_resolution + j,
									(i + 1)*_resolution + (j + 1) % _resolution,
									(i + 1)*_resolution + j));
		}
	}

	// two end facets
	for (int i = 0; i < _resolution; ++i) {
		_facets.push_back(ivec3(_polyline.size()*_resolution,
								(i + 1) % _resolution,
								i));
		_facets.push_back(ivec3(_polyline.size()*_resolution + 1,
								(_polyline.size() - 1)*_resolution + i,
								(_polyline.size() - 1)*_resolution + (i + 1) % _resolution));
	}
}