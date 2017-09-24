#pragma once
//#include "Ogre.h"
#include "ode/ode.h"
#include "TrackGeom.h"
#include "mxsplinesurface.h"

//#include "3ds.h"//add track's 3ds import... um forget 3ds.  I really just need the geometry
//write export for vert and indice in tracksweeper

//#include "Object3DS.h"

//#include "MeshInformer.h"//to get info for creating a TriMesh
//
//using namespace Ogre;
#define DllExport   __declspec( dllexport ) 

class DllExport CMxTrack
{
public:
	CMxTrack(dSpaceID spaceID, char *trackFileName, char *tireTrackFileName);
	~CMxTrack(void);
	void Reload(char *trackFileName, char *tireTrackFileName);
	void Render();
	void FindTriangleIntersect(Vector3p position, Vector3p direction,  float *t, float *u, float *v, Vector3p *triNormal, int *index);
	int intersect_triangle(float orig[3], float dir[3], float vert0[3], float vert1[3], float vert2[3],  float *t, float *u, float *v);
	bool intersect_triangle_index(Vector3p position, Vector3p direction, int triIndex,  float *t, float *u, float *v);

	float getDistanceToSpline(Vector3p position,  float u, float v, int triIndex);

	Vector3p getSplineNormalAt(int triIndex, float u, float v);
	void IterateUV(Vector3p position, Vector3p direction, float t, float *u, float *v,int triIndex);
	int intersect_quad(float orig[3], float dir[3], float vert0[3], float vert1[3], float vert2[3], float vert3[3],
                   float *t, float *u, float *v);
	
	void FindQuadIntersect(Vector3p position, Vector3p direction,  float *t, float *u, float *v, Vector3p *triNormal, int *index);

	float GetSufaceRollingResistance(int triIndex);
	float GetSufaceFriction(int triIndex);

	float GetBumpAt(int triIndex, float u, float v);

	CMxSplineSurface *m_Splines;
	
	dSpaceID m_spaceID;
	dGeomID m_trackgeomID;
	dTriMeshDataID m_trackMeshDataID;
	CTrackGeom *m_TrackGeom;
	CTrackGeom *m_TireSplineGeom;
	dGeomID m_TransformGeomT;



};
