#include "Extrusion.h"

int main(int argc, char* argv[])
{
	Extrusion et;
	
	et.readData(argv[1]);
	et.setResolution(16);
	et.setRadius(0.1);
	et.computeTube();
	et.writeOBJ();

	return 1;
}