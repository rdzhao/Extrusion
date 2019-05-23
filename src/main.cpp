#include "Extrusion.h"

int main(int argc, char* argv[])
{
	Extrusion et;
	
	et.readData(argv[1]);
	et.setResolution(8);
	et.setRadius(0.003);
	et.computeTube();
	//et.writeOBJ();
	et.writePLY();

	return 1;
}