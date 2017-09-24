#pragma once
//#include "spline.h"//comment out
#include "Matrix.h"

#define SURFACE_ASPHALT 0.5f
#define SURFACE_CONCRETE 1.5f
#define SURFACE_SNOW 2.5f
#define SURFACE_GRASS 3.5f
#define SURFACE_ICE 4.5f
#define SURFACE_WET_ASPHALT 5.5f
#define SURFACE_WET_CONCRETE 6.5f
#define SURFACE_DIRT 7.5f
#define SURFACE_GRAVEL 8.5f
#define SURFACE_MUD 9.5f


struct Vertex{
	float x, y, z;
};
struct Triangle{
	unsigned int A,B,C;
	float iSurfaceType;
};


class CTrackGeom
{
public:
	CTrackGeom(void);
	~CTrackGeom(void);

	Vertex *m_VertexArray;
	Triangle *m_TrisArray;
	
	unsigned int m_VertexArraySize;
	unsigned int m_TrisArraySize;
		
	unsigned int m_Width;
	unsigned int m_Length;
	bool m_bOpen;
	
	

//	int Process();

	void free(void);
//	void Process(CSpline *track);
//	int save( const char  *filename);
//	int save(FILE *fp);
	int load(char *filename);
	int load(FILE *fp);



};
