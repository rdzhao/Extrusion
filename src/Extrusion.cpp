#include "Extrusion.h"

void Extrusion::readData(string fn)
{
	ifstream file(fn.c_str());

	string str;
	while (getline(file, str)) {
		stringstream ss(str);

		int idx;
		float x, y, z;
		float r, g, b;

		ss >> idx;
		ss >> x >> y >> z;
		ss >> r >> g >> b;

		_indices.push_back(idx);
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

void Extrusion::writePLY()
{
	std::ofstream out("CrossSection.ply");
	// formating ply files
	out << "ply" << std::endl;
	out << "format ascii 1.0" << std::endl;
	out << "element vertex " << _vertices.size() << std::endl;
	out << "property float32 x" << std::endl;
	out << "property float32 y" << std::endl;
	out << "property float32 z" << std::endl;
	out << "property uchar red" << std::endl;
	out << "property uchar green" << std::endl;
	out << "property uchar blue" << std::endl;
	out << "element face " << _facets.size() << std::endl;
	out << "property list uchar int vertex_index" << std::endl;
	out << "end_header" << std::endl;
	// data
	for (int i = 0; i < _vertices.size(); ++i) {
		out << _vertices[i].x << " "
			<< _vertices[i].y << " "
			<< _vertices[i].z << " "
			<< _tubeColors[i].x << " "
			<< _tubeColors[i].y << " "
			<< _tubeColors[i].z << " "
			<< std::endl;
	}
	for (int i = 0; i < _facets.size(); ++i) {
		out << "3 "
			<< _facets[i].x << " "
			<< _facets[i].y << " "
			<< _facets[i].z << std::endl;
	}
}

void Extrusion::computeTube()
{
	_currentIdx = 0;
	_currentVertexNum = 0;
	while (_currentIdx < _polyline.size()) {
		assembleTmpData();
		computeDisplacement();
		computeVertices();
		assembleFacets();
		addArrowHead();
	}
}

void Extrusion::assembleTmpData()
{
	// clear
	_tmpPolyline.clear();
	_tmpColors.clear();
	_displacement.clear();

	// assemble
	int cTag = _indices[_currentIdx];
	/*while (_indices[_currentIdx] == cTag) {
		_tmpPolyline.push_back(_polyline[_currentIdx]);
		_tmpColors.push_back(_colors[_currentIdx]);

		++_currentIdx;
	}*/
	while (1) {
		if (_currentIdx == _polyline.size())
			break;

		if (_indices[_currentIdx] == cTag) {
			_tmpPolyline.push_back(_polyline[_currentIdx]);
			_tmpColors.push_back(_colors[_currentIdx]);

			++_currentIdx;
		}
		else {
			break;
		}
	}

}

void Extrusion::computeDisplacement()
{
	for (int i = 0; i < _tmpPolyline.size() - 1; ++i) {
		if (i == 0) { // set first one
			vec3 v = _tmpPolyline[i + 1] - _tmpPolyline[i];
			vec3 n;
			setVerticalVector(v, n);

			_displacement.push_back(n);
			//cout << n.x<<" " << n.y << " " << n.z << endl;
		}
		else { // parallel transport the rest
			vec3 v1 = _tmpPolyline[i] - _tmpPolyline[i-1];
			vec3 v2 = _tmpPolyline[i + 1] - _tmpPolyline[i];
			vec3 n1 = _displacement[i - 1];
			vec3 n2;
			parallelTransport(v1, v2, n1, n2);

			_displacement.push_back(n2);
			//cout << n2.x<<" " << n2.y << " " << n2.z << endl;
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
	_tmpVertexNum = 0;

	float theta = 2 * PI / _resolution;

	for (int i = 0; i < _tmpPolyline.size(); ++i) {
		if (i == 0) {
			vec3 v = _tmpPolyline[i + 1] - _tmpPolyline[i];
			vec3 n = _displacement[0];

			int k = 0;
			do {
				_vertices.push_back(_tmpPolyline[i] + _radius*n);
				_tubeColors.push_back(_tmpColors[i]);
				++_tmpVertexNum;

				rotate(v, n, theta, n);
				++k;
			} while (k < _resolution);
		}
		else if (i == _tmpPolyline.size() - 1) {
			vec3 v = _tmpPolyline[i] - _tmpPolyline[i - 1];
			vec3 n = _displacement[i - 1];

			int k = 0;
			do {
				_vertices.push_back(_tmpPolyline[i] + _radius * n);
				_tubeColors.push_back(_tmpColors[i]);
				++_tmpVertexNum;

				rotate(v, n, theta, n);
				++k;
			} while (k < _resolution);
		}
		else
		{
			vec3 v1 = _tmpPolyline[i] - _tmpPolyline[i - 1];
			vec3 v2 = _tmpPolyline[i + 1] - _tmpPolyline[i];
			vec3 n1 = _displacement[i - 1];
			vec3 n2 = _displacement[i];

			int k = 0;
			do {
				vec3 n = n1 + n2;
				n /= sqrt(dot(n, n));
				_vertices.push_back(_tmpPolyline[i] + _radius * n);
				_tubeColors.push_back(_tmpColors[i]);
				++_tmpVertexNum;

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
	_vertices.push_back(_tmpPolyline[0]);
	_tubeColors.push_back(_tmpColors[0]);
	++_tmpVertexNum;
	_vertices.push_back(_tmpPolyline[_tmpPolyline.size() - 1]);
	_tubeColors.push_back(_tmpColors[_tmpPolyline.size() - 1]);
	++_tmpVertexNum;
}

void Extrusion::assembleFacets()
{
	for (int i = 0; i < _tmpPolyline.size() - 1; ++i) {
		for (int j = 0; j < _resolution; ++j) {
			_facets.push_back(ivec3(_currentVertexNum + i*_resolution + j,
				_currentVertexNum + i*_resolution + (j + 1) % _resolution,
				_currentVertexNum + (i + 1)*_resolution + (j + 1) % _resolution));
			_facets.push_back(ivec3(_currentVertexNum + i*_resolution + j,
				_currentVertexNum + (i + 1)*_resolution + (j + 1) % _resolution,
				_currentVertexNum + (i + 1)*_resolution + j));
		}
	}

	// two end facets
	for (int i = 0; i < _resolution; ++i) {
		_facets.push_back(ivec3(_currentVertexNum + _tmpPolyline.size()*_resolution,
			_currentVertexNum + (i + 1) % _resolution,
			_currentVertexNum + i));	
	}
	for (int i = 0; i < _resolution; ++i) {
		_facets.push_back(ivec3(_currentVertexNum + _tmpPolyline.size()*_resolution + 1,
			_currentVertexNum + (_tmpPolyline.size() - 1)*_resolution + i,
			_currentVertexNum + (_tmpPolyline.size() - 1)*_resolution + (i + 1) % _resolution));
	}

	_currentVertexNum += _tmpVertexNum;
}

void Extrusion::addArrowHead()
{
	// clear facets at end
	for (int i = 0; i < _resolution; ++i) {
		_facets.pop_back();
	}

	// add new vertices
	vec3 ep = _vertices.back();
	vec3 ec = _tubeColors.back();
	_vertices.pop_back();
	_tubeColors.pop_back();
	
	float theta = 2 * PI / _resolution;
	int s = _tmpPolyline.size() - 1;
	vec3 v = _tmpPolyline[s] - _tmpPolyline[s - 1];
	v /= sqrt(dot(v, v));
	vec3 n = _displacement[s-1];

	int k = 0;
	do {
		_vertices.push_back(_tmpPolyline[s] + 3.0f * _radius * n);
		_tubeColors.push_back(_tmpColors[s]);
		//++_tmpVertexNum;

		rotate(v, n, theta, n);
		++k;
	} while (k < _resolution);

	_vertices.push_back(ep + 16.0f * _radius * v);
	_tubeColors.push_back(ec);

	// add new facets
	_currentVertexNum -= 1;
	for (int i = 0; i < _resolution; ++i) {
		_facets.push_back(ivec3(_currentVertexNum + i,
			_currentVertexNum + i - (_resolution + 1),
			_currentVertexNum + (i + 1) % _resolution));
		_facets.push_back(ivec3(_currentVertexNum + (i + 1) % _resolution,
			_currentVertexNum + i - (_resolution + 1),
			_currentVertexNum + (i + 1) % _resolution - (_resolution + 1)));
	}

	for (int i = 0; i < _resolution; ++i) {
		_facets.push_back(ivec3(_currentVertexNum + i,
			_currentVertexNum + (i + 1) % _resolution,
			_currentVertexNum + (_resolution)));
	}

	_currentVertexNum += _resolution + 1;
}