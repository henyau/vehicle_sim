#include "StdAfx.h"
#include "mxtrack.h"
//#include "gl\gl.h"
//#include "gl\glu.h"
#define EPSILON 0.000001
#define CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];
#define SUB(dest,v1,v2)\
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2]; 
#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

//#define USE_SPLINEPATCHPOS

CMxTrack::CMxTrack(dSpaceID spaceID, char *trackFileName, char *tireTrackFileName)
{
	m_TrackGeom = new CTrackGeom;
	m_TireSplineGeom = new CTrackGeom;
	m_spaceID = spaceID;
	m_TrackGeom->load(trackFileName);
	m_TireSplineGeom->load(tireTrackFileName);

	Vector3p tempRotated;
	dVector3 *vertices = new dVector3[m_TrackGeom->m_VertexArraySize];
	for(int i = 0; i<m_TrackGeom->m_VertexArraySize; i++)
	{
		tempRotated.x = m_TrackGeom->m_VertexArray[i].x;
		tempRotated.y = m_TrackGeom->m_VertexArray[i].z;
		tempRotated.z = -m_TrackGeom->m_VertexArray[i].y;
		//do rotations and save
		m_TrackGeom->m_VertexArray[i].y = tempRotated.y;
		m_TrackGeom->m_VertexArray[i].z = tempRotated.z;
		vertices[i][0] = (dReal)tempRotated.x;
		vertices[i][1] = (dReal)tempRotated.y;
		vertices[i][2] = (dReal)tempRotated.z;	
	}
	int *indices = new int[m_TrackGeom->m_TrisArraySize*3];
	for(i =0 ; i<m_TireSplineGeom->m_VertexArraySize; i++)//should do this in track sweeper...NOTE
	{
		tempRotated.x = m_TireSplineGeom->m_VertexArray[i].x;
		tempRotated.y = m_TireSplineGeom->m_VertexArray[i].z;
		tempRotated.z = -m_TireSplineGeom->m_VertexArray[i].y;

		m_TireSplineGeom->m_VertexArray[i].y = tempRotated.y;
		m_TireSplineGeom->m_VertexArray[i].z = tempRotated.z;
	}

	for(i = 0; i<m_TrackGeom->m_TrisArraySize; i++)
	{
		indices[i*3] = m_TrackGeom->m_TrisArray[i].A;
		indices[i*3+1] = m_TrackGeom->m_TrisArray[i].B;
		indices[i*3+2] = m_TrackGeom->m_TrisArray[i].C;	
	}
	
	m_trackMeshDataID  = dGeomTriMeshDataCreate(); 
	dGeomTriMeshDataBuildSimple(m_trackMeshDataID, (const dReal*)vertices,  m_TrackGeom->m_VertexArraySize, indices,m_TrackGeom->m_TrisArraySize*3); 	
	
	
	
m_trackgeomID = dCreateTriMesh(m_spaceID, m_trackMeshDataID,0,0,0);
//test plane in Virtools Henry


//	m_trackgeomID = dCreatePlane(m_spaceID,0,1,0,0);
	//dMatrix3 mRotation;
	//mRotation[0] = 1.0f; mRotation[1] = 0.0f; mRotation[2] = 0.0f;
	//mRotation[4] = 0.0f;	mRotation[5] = 0.0f; mRotation[6] = 1.0f;
	//mRotation[8] = 0.0f;  mRotation[9] = -1.0f; mRotation[10] = 0.0f;	
	//dGeomSetRotation(m_trackgeomID, mRotation);


	//m_SplineLeft = new CMxSplineSurface(m_TrackGeom, 1);
	//m_SplineRight = new CMxSplineSurface(m_TrackGeom, 2);

	//m_SplineLeftLeft = new CMxSplineSurface(m_TrackGeom, 0);
	//m_SplineRightRight = new CMxSplineSurface(m_TrackGeom, 3);

	m_Splines = new CMxSplineSurface[m_TireSplineGeom->m_Width];
	for(i = 0; i<m_TireSplineGeom->m_Width; i++)
	{
		m_Splines[i].SetSplineFromTrack(m_TireSplineGeom, i);	
	}
	
}

CMxTrack::~CMxTrack(void)
{
	//dGeomTriMeshDataDestroy(m_TrackMashDataID);

	//for(int i = 0; i<m_TrackGeom->m_Width; i++)
		delete [] m_Splines;

	delete m_TrackGeom;
	dGeomDestroy(m_trackgeomID);

	/*delete m_SplineLeft;
	delete m_SplineRight;
	delete m_SplineLeftLeft;
	delete m_SplineRightRight;*/

	

}

void CMxTrack::Reload(char *trackFileName, char *tireTrackFileName)
{
	//for(int ii = 0; ii<m_TrackGeom->m_Width; ii++)
	delete []m_Splines;

	delete m_TrackGeom;
	delete m_TireSplineGeom;
	dGeomDestroy(m_trackgeomID);
	//delete m_SplineLeft;
	//delete m_SplineRight;

	//delete m_SplineLeftLeft;
	//delete m_SplineRightRight;


	m_TrackGeom = new CTrackGeom;
	m_TireSplineGeom = new CTrackGeom;
	//m_spaceID = spaceID;
	m_TrackGeom->load(trackFileName);
	m_TireSplineGeom->load(tireTrackFileName); //henry... why did i add this?

	Vector3p tempRotated;
//	Matrix3x3 matRotate270;
//	matRotate270.rpyRotation(270.0f*M_PI/180, 0.0f, 0.0f);

	dVector3 *vertices = new dVector3[m_TrackGeom->m_VertexArraySize];
	for(int i = 0; i<m_TrackGeom->m_VertexArraySize; i++)
	{
		tempRotated.x = m_TrackGeom->m_VertexArray[i].x;
		tempRotated.y = m_TrackGeom->m_VertexArray[i].z;
		tempRotated.z = -m_TrackGeom->m_VertexArray[i].y;

		m_TrackGeom->m_VertexArray[i].y = tempRotated.y;
		m_TrackGeom->m_VertexArray[i].z = tempRotated.z;

		vertices[i][0] = (dReal)tempRotated.x;
		vertices[i][1] = (dReal)tempRotated.y;
		vertices[i][2] = (dReal)tempRotated.z;	
	}
	int *indices = new int[m_TrackGeom->m_TrisArraySize*3];

	for(i =0 ; i<m_TireSplineGeom->m_VertexArraySize; i++)//should do this in track sweeper...NOTE
	{
		tempRotated.x = m_TireSplineGeom->m_VertexArray[i].x;
		tempRotated.y = m_TireSplineGeom->m_VertexArray[i].z;
		tempRotated.z = -m_TireSplineGeom->m_VertexArray[i].y;

		m_TireSplineGeom->m_VertexArray[i].y = tempRotated.y;
		m_TireSplineGeom->m_VertexArray[i].z = tempRotated.z;
	}

	for(i = 0; i<m_TrackGeom->m_TrisArraySize; i++)
	{
		indices[i*3] = m_TrackGeom->m_TrisArray[i].A;
		indices[i*3+1] = m_TrackGeom->m_TrisArray[i].B;
		indices[i*3+2] = m_TrackGeom->m_TrisArray[i].C;	
	}
	
	m_trackMeshDataID  = dGeomTriMeshDataCreate(); 
	dGeomTriMeshDataBuildSimple(m_trackMeshDataID, (const dReal*)vertices,  m_TrackGeom->m_VertexArraySize, indices,m_TrackGeom->m_TrisArraySize*3); 	
	
	m_trackgeomID = dCreateTriMesh(m_spaceID, m_trackMeshDataID,0,0,0);
	dMatrix3 mRotation;
	mRotation[0] = 1.0f; mRotation[1] = 0.0f; mRotation[2] = 0.0f;
	mRotation[4] = 0.0f;	mRotation[5] = 0.0f; mRotation[6] = 1.0f;
	mRotation[8] = 0.0f;  mRotation[9] = -1.0f; mRotation[10] = 0.0f;
	
	//dGeomSetRotation(m_trackgeomID, mRotation);


	//m_entityTrack = mSceneMgr->createEntity("Track", "Max.mesh");
	//m_entityTrack->setNormaliseNormals(true);
	//m_entityTrack->setCastShadows(false);
	//m_entityTrack->setMaterialName("DoorA/Door1");
	//m_nodeTrack = mSceneMgr->getRootSceneNode()->createChildSceneNode(m_entityTrack->getName());
	//m_nodeTrack->attachObject(m_entityTrack);
	//m_nodeTrack->setPosition(Vector3(0,-3.0,0));
	//m_nodeTrack->setOrientation(Quaternionp(Degree(-90.0f),Vector3(1,0,0)));
	//m_nodeTrack->setScale(1.0,1.0,1.0);
	//m_nodeTrack->showBoundingBox(true);

	//m_trackgeom = OgreOde::MxMeshInformer::createStaticTriangleMesh(m_nodeTrack,m_world->getDefaultSpace());

	////spline shit
	//m_SplineLeft = new CMxSplineSurface(m_TrackGeom,1);
	//m_SplineRight = new CMxSplineSurface(m_TrackGeom,2);

	
	//m_SplineLeftLeft = new CMxSplineSurface(m_TrackGeom, 0);
	//m_SplineRightRight = new CMxSplineSurface(m_TrackGeom, 3);

	m_Splines = new CMxSplineSurface[m_TireSplineGeom->m_Width];
	for(i = 0; i<m_TireSplineGeom->m_Width; i++)
	{
		m_Splines[i].SetSplineFromTrack(m_TireSplineGeom, i);	
	}
}

//needs to be called after load
void CMxTrack::Render()
{
	/*
	glPushMatrix();
	//glLoadIdentity();	
	//do transforms here
 	glRotatef(270.0f, 1,0,0); 

	m_TrackGeom->m_TrisArraySize;
 	m_TrackGeom->m_VertexArraySize;
	glDisable(GL_LIGHTING);
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_LINES);
	for(int i = 0; i< m_TrackGeom->m_TrisArraySize; i++)
	{
		glVertex3f(m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].A].x,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].A].y,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].A].z);
		glVertex3f(m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].B].x,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].B].y,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].B].z);
		glVertex3f(m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].C].x,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].C].y,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].C].z);
	}
	glEnd();

	//glEnable(GL_LIGHTING);
   	glColor4f(1.0f, 1.0f, 1.0f, 0.5f);
	glBegin(GL_TRIANGLES);
	for(int i = 0; i< m_TrackGeom->m_TrisArraySize; i++)
	{
		glVertex3f(m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].A].x,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].A].y,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].A].z);
		glVertex3f(m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].B].x,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].B].y,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].B].z);
		glVertex3f(m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].C].x,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].C].y,
			m_TrackGeom->m_VertexArray[m_TrackGeom->m_TrisArray[i].C].z);
	}
	glEnd();
	glPopMatrix();
	*/
}
void CMxTrack::FindTriangleIntersect(Vector3p position, Vector3p direction,  float *t, float *u, float *v, Vector3p *triNormal, int *index)
{//need some sort of tree to reduce number of potential triangles

	float pos[3],  dir[3];
	float vert0[3],  vert1[3],  vert2[3],  normal[3];
	float edge1[3], edge2[3];

	pos[0] = position.x;
	pos[1] = position.y;
	pos[2] = position.z;

	dir[0] = direction.x;
	dir[1] = direction.y;
	dir[2] = direction.z;
	
	for(int i = 0; i<m_TireSplineGeom->m_TrisArraySize; i++)
	{
		if(i&1)// if odd. need to rotate the indices
		{
  		vert0[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].x;
		vert0[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].y;
		vert0[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].z;

		vert1[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].x;
		vert1[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].y;
		vert1[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].z;

		vert2[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].x;
		vert2[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].y;
		vert2[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].z;

		}
		else
		{
		vert0[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].x;
		vert0[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].y;
		vert0[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].z;

		vert1[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].x;
		vert1[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].y;
		vert1[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].z;

		vert2[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].x;
		vert2[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].y;
		vert2[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].z;
		}

		if(intersect_triangle(pos, dir, vert0, vert1,vert2, t, u,v))//if found quit. else continue
		{
			SUB(edge1, vert1, vert0);
			SUB(edge2, vert2, vert0);
			CROSS(normal, edge1, edge2);
			triNormal->x = normal[0];
			triNormal->y = normal[1];
			triNormal->z = normal[2];
			triNormal->normalise();
			*index = i;

			return;
		}
		

	}
	//if you are down here, you didn't find a triangle.
	{
			*index = -1;	
	}

	
}


void CMxTrack::FindQuadIntersect(Vector3p position, Vector3p direction,  float *t, float *u, float *v, Vector3p *triNormal, int *index)
{//need some sort of tree to reduce number of potential triangles

	float pos[3],  dir[3];
	float vert0[3],  vert1[3],  vert2[3], vert3[3], normal[3];
	float edge1[3], edge2[3];

	pos[0] = position.x;
	pos[1] = position.y;
	pos[2] = position.z;

	dir[0] = direction.x;
	dir[1] = direction.y;
	dir[2] = direction.z;
	
	for(int i = 0; i<m_TireSplineGeom->m_TrisArraySize-1; i+=2)
	{
		vert0[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].x;
		vert0[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].y;
		vert0[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].A].z;

		vert1[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].x;
		vert1[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].y;
		vert1[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].B].z;

		vert2[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].x;
		vert2[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].y;
		vert2[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i].C].z;
		
		vert3[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i+1].C].x;
		vert3[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i+1].C].y;
		vert3[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[i+1].C].z;
		

		if(intersect_quad(pos, dir, vert0, vert1,vert2, vert3, t, u,v))//if found quit. else continue
		{
			SUB(edge1, vert1, vert0);
			SUB(edge2, vert2, vert0);
			CROSS(normal, edge1, edge2);
			triNormal->x = normal[0];
			triNormal->y = normal[1];
			triNormal->z = normal[2];
			triNormal->normalise();
			*index = i;

			return;
		}
		

	}
	//if you are down here, you didn't find a triangle.
	{
			*index = -1;	
	}

	
}


bool CMxTrack::intersect_triangle_index(Vector3p position, Vector3p direction, int triIndex,  float *t, float *u, float *v)
{
	if(triIndex<0 ||triIndex>=m_TireSplineGeom->m_TrisArraySize)
		return false;
	float pos[3],  dir[3];
	float vert0[3],  vert1[3],  vert2[3],  normal[3];
	float edge1[3], edge2[3];

	pos[0] = position.x;
	pos[1] = position.y;
	pos[2] = position.z;

	dir[0] = direction.x;
	dir[1] = direction.y;
	dir[2] = direction.z;
	
		if(triIndex&1)// if odd. need to rotate the indices
		{
  		vert0[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].C].x;
		vert0[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].C].y;
		vert0[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].C].z;

		vert1[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].A].x;
		vert1[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].A].y;
		vert1[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].A].z;

		vert2[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].B].x;
		vert2[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].B].y;
		vert2[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].B].z;

		}
		else
		{
		vert0[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].A].x;
		vert0[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].A].y;
		vert0[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].A].z;

		vert1[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].B].x;
		vert1[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].B].y;
		vert1[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].B].z;

		vert2[0] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].C].x;
		vert2[1] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].C].y;
		vert2[2] = m_TireSplineGeom->m_VertexArray[m_TireSplineGeom->m_TrisArray[triIndex].C].z;
		}

		if(intersect_triangle(pos, dir, vert0, vert1,vert2, t, u,v))//if found quit. else continue
		{
			//maybe use later as a backup to spline
			//SUB(edge1, vert1, vert0);
			//SUB(edge2, vert2, vert0);
			//CROSS(normal, edge1, edge2);
			//triNormal->x = normal[0];
			//triNormal->y = normal[1];
			//triNormal->z = normal[2];
			//triNormal->normalise();
			return true;
		}
		else
			return false;

}

int CMxTrack::intersect_triangle(float orig[3], float dir[3],
                   float vert0[3], float vert1[3], float vert2[3],
                   float *t, float *u, float *v)
{
   float edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
   float det,inv_det;

   /* find vectors for two edges sharing vert0 */
   SUB(edge1, vert1, vert0);
   SUB(edge2, vert2, vert0);

   /* begin calculating determinant - also used to calculate U parameter */
   CROSS(pvec, dir, edge2);

   /* if determinant is near zero, ray lies in plane of triangle */
   det = DOT(edge1, pvec);

//#ifdef TEST_CULL           /* define TEST_CULL if culling is desired */
   if (det < EPSILON)
      return 0;

   /* calculate distance from vert0 to ray origin */
   SUB(tvec, orig, vert0);

   /* calculate U parameter and test bounds */
   *u = DOT(tvec, pvec);
   if (*u < 0.0 || *u > det)
      return 0;

   /* prepare to test V parameter */
   CROSS(qvec, tvec, edge1);

    /* calculate V parameter and test bounds */
   *v = DOT(dir, qvec);
   if (*v < 0.0 || *u + *v > det)
      return 0;

   /* calculate t, scale parameters, ray intersects triangle */
   *t = DOT(edge2, qvec);
   inv_det = 1.0 / det;
   *t *= inv_det;
   *u *= inv_det;
   *v *= inv_det;
//#else                    /* the non-culling branch */
//   if (det > -EPSILON && det < EPSILON)
//     return 0;
//   inv_det = 1.0 / det;
//
//   /* calculate distance from vert0 to ray origin */
//   SUB(tvec, orig, vert0);
//
//   /* calculate U parameter and test bounds */
//   *u = DOT(tvec, pvec) * inv_det;
//   if (*u < 0.0 || *u > 1.0)
//     return 0;
//
//   /* prepare to test V parameter */
//   CROSS(qvec, tvec, edge1);
//
//   /* calculate V parameter and test bounds */
//   *v = DOT(dir, qvec) * inv_det;
//   if (*v < 0.0 || *u + *v > 1.0)
//     return 0;
//
//   /* calculate t, ray intersects triangle */
//   *t = DOT(edge2, qvec) * inv_det;
//#endif
   return 1;
}


int CMxTrack::intersect_quad(float orig[3], float dir[3],
                   float vert0[3], float vert1[3], float vert2[3], float vert3[3],
                   float *t, float *u, float *v)
{
   float edge1[3], edge2[3], edge3[3], tvec[3], pvec[3], qvec[3];
   float det,inv_det;

   //3 edges with v0
   SUB(edge1, vert1, vert0);
   SUB(edge2, vert2, vert0);
   SUB(edge3, vert3, vert0);

   /* begin calculating determinant - also used to calculate U parameter */
   CROSS(pvec, dir, edge2);

   /* if determinant is near zero, ray lies in plane of triangle */
   det = DOT(edge1, pvec);

   if (det < EPSILON)
      return 0;

   /* calculate distance from vert0 to ray origin */
   SUB(tvec, orig, vert0);

   /* calculate U parameter and test bounds */
   *u = DOT(tvec, pvec);
   if (*u < 0.0 || *u > det)
      return 0;

   /* prepare to test V parameter */
   CROSS(qvec, tvec, edge1);

    /* calculate V parameter and test bounds */
   *v = DOT(dir, qvec);
   if (*v < 0.0 || *u + *v > det)
      return 0;

   /* calculate t, scale parameters, ray intersects triangle */
   *t = DOT(edge2, qvec);
   inv_det = 1.0 / det;
   *t *= inv_det;
   *u *= inv_det;
   *v *= inv_det;

   return 1;
}



Vector3p CMxTrack::getSplineNormalAt(int triIndex, float u, float v)
{	 

	int latIndex = triIndex / ((m_TireSplineGeom->m_Length-1)*2);
	int longIndex = // 1+((triIndex+1)/2) % ((m_TireSplineGeom->m_Length-1));
		((triIndex-latIndex*(m_TireSplineGeom->m_Length-1)*2	)/2)%(m_TireSplineGeom->m_Length-1)+1;
	//longIndex+=1;


	Vector3p vLeftTan = m_Splines[latIndex].getTanAt(longIndex, v);

	vLeftTan.normalise();
	Vector3p vRightTan = m_Splines[latIndex+1].getTanAt(longIndex,v);
	vRightTan.normalise();

	//linear side
	Vector3p vLeftPos = m_Splines[latIndex].getPosAt(longIndex, v);
	Vector3p vRightPos = m_Splines[latIndex+1].getPosAt(longIndex,v);
	Vector3p vSide;
	vSide.sub(vLeftPos, vRightPos);
	//spline side vect
	//
/*	Vector3p vLeftLeftPos, vRightRightPos; 
	if(latIndex>0)
		vLeftLeftPos= m_Splines[latIndex-1].getPosAt(longIndex, v);
	else
		vLeftLeftPos = vLeftPos;

	if(latIndex<m_TireSplineGeom->m_Width-2)
		vRightRightPos = m_Splines[latIndex+2].getPosAt(longIndex,v);
	else
		vRightRightPos = vRightPos;	

	CMxSplineSurface lateralSpline(vRightRightPos,vRightPos, vLeftPos, vLeftLeftPos );
	vSide = lateralSpline.getTanAt(2, u);
*/

	//
		
	Vector3p linInterpTan;
	linInterpTan.sub( vLeftTan, vRightTan);
	linInterpTan.mult(u);
	linInterpTan.add(vLeftTan);
	
	Vector3p vNormal;
	vNormal.cross(vSide, linInterpTan);

	vNormal.normalise();
	return (vNormal);

/*

#define USE_SPLINESANDLINEAR
#ifdef USE_SPLINESANDLINEAR
	
    Vector3p vLeftTan = m_SplineLeft->getTanAt(triIndex, v);
	vLeftTan.normalise();
	Vector3p vRightTan = m_SplineRight->getTanAt(triIndex,v);
	vRightTan.normalise();

	Vector3p vLeftPos = m_SplineLeft->getPosAt(triIndex, v);
	Vector3p vRightPos = m_SplineRight->getPosAt(triIndex,v);
	Vector3p vSide;
	vSide.sub(vLeftPos, vRightPos);
		
	Vector3p linInterpTan;
	linInterpTan.sub( vLeftTan, vRightTan);
	linInterpTan.mult(u);
	linInterpTan.add(vLeftTan);
	
	Vector3p vNormal;
	vNormal.cross(vSide, linInterpTan);

	vNormal.normalise();
	return (vNormal);

#else //use 4 splines!!!!
    Vector3p vLeftTan = m_SplineLeft->getTanAt(triIndex, v);
	Vector3p vRightTan = m_SplineRight->getTanAt(triIndex,v);

    Vector3p vLeftLeftTan = m_SplineLeftLeft->getTanAt(triIndex, v);
	Vector3p vRightRightTan = m_SplineRightRight->getTanAt(triIndex,v);
vLeftTan.normalise();
vRightTan.normalise();
vLeftLeftTan.normalise();
vRightRightTan.normalise();

	Vector3p vLeftPos = m_SplineLeft->getPosAt(triIndex, v);
	Vector3p vRightPos = m_SplineRight->getPosAt(triIndex,v);

	Vector3p vLeftLeftPos = m_SplineLeftLeft->getPosAt(triIndex, v);
	Vector3p vRightRightPos = m_SplineRightRight->getPosAt(triIndex,v);

	CMxSplineSurface lateralSpline(vRightRightPos,vRightPos, vLeftPos, vLeftLeftPos );
	Vector3p vSide = lateralSpline.getTanAt(2, u);
		
	Vector3p linInterpTan;
	linInterpTan.sub( vLeftTan, vRightTan);
	linInterpTan.mult(u);
	linInterpTan.add(vLeftTan);
	
	Vector3p vNormal;
	vNormal.cross(vSide, linInterpTan);

	vNormal.normalise();
	return (vNormal);

#endif
*/

}

void CMxTrack::IterateUV(Vector3p position, Vector3p direction, float t, float *u, float *v,int triIndex)
{

	int latIndex = triIndex / ((m_TireSplineGeom->m_Length-1)*2);
	int longIndex = // 1+((triIndex+1)/2) % ((m_TireSplineGeom->m_Length-1));
		((triIndex-latIndex*(m_TireSplineGeom->m_Length-1)*2	)/2)%(m_TireSplineGeom->m_Length-1)+1;
	//longIndex+=1;

		Vector3p vLeftPos = m_Splines[latIndex].getPosAt(longIndex, *v);
		Vector3p vRightPos = m_Splines[latIndex+1].getPosAt(longIndex,*v);
		Vector3p vForward =  m_Splines[latIndex+1].getTanAt(longIndex,*v);

		//Vector3p vLeftPos = m_SplineLeft->getPosAt(triIndex, *v);
		//Vector3p vRightPos = m_SplineRight->getPosAt(triIndex,*v);
		//Vector3p vForward =  m_SplineRight->getTanAt(triIndex,*v);

		Vector3p vSide;
		Vector3p vCPos;
		vSide.sub(vLeftPos, vRightPos);
		vCPos = vSide;
		vCPos.mult(*u);
		vCPos.add(vRightPos); //contact position

		direction.mult(t);
		Vector3p vDist;
		position.add(direction); //real position
		vCPos.sub(position);
		vDist = vCPos;

//		iterations++;

        float du,dv,dLat,dLon;
		dLat=vSide.mod();
		dLon=vForward.mod();
        
		vForward.normalise();
		vSide.normalise();

		dv=vDist.dot(vForward);
        du=vDist.dot(vSide);

		*u-=du/dLat;
        *v-=dv/dLon;

}

float CMxTrack::getDistanceToSpline(Vector3p position,  float u, float v, int triIndex)
{//use linear interp+longSplines first then lateral splines
	int latIndex = triIndex / ((m_TireSplineGeom->m_Length-1)*2);
	int longIndex =// 1+((triIndex+1)/2) % ((m_TireSplineGeom->m_Length-1));
		((triIndex-latIndex*(m_TireSplineGeom->m_Length-1)*2	)/2)%(m_TireSplineGeom->m_Length-1)+1;
	//longIndex+=1;

	Vector3p vLeftPos = m_Splines[latIndex].getPosAt(longIndex, v);
	Vector3p vRightPos = m_Splines[latIndex+1].getPosAt(longIndex,v);
	///
	//from spline
#ifdef USE_SPLINEPATCHPOS
	Vector3p vLeftLeftPos, vRightRightPos, vInterpPos; 
	if(latIndex>0)
		vLeftLeftPos= m_Splines[latIndex-1].getPosAt(longIndex, v);
	else
		vLeftLeftPos = vLeftPos;

	if(latIndex<m_TireSplineGeom->m_Width-2)
		vRightRightPos = m_Splines[latIndex+2].getPosAt(longIndex,v);
	else
		vRightRightPos = vRightPos;	

	CMxSplineSurface lateralSpline(vRightRightPos,vRightPos, vLeftPos, vLeftLeftPos );
	vInterpPos = lateralSpline.getPosAt(2, u);

	position.sub(vInterpPos);
#else
	//from linear interp (need to project onto direction vector to be correct)?


	Vector3p linInterpPos;
	linInterpPos.sub(  vRightPos, vLeftPos);
	linInterpPos.mult(1.0f-u);
	linInterpPos.add(vLeftPos);

	position.sub(linInterpPos);
#endif
	return position.mod();

}


float CMxTrack::GetSufaceRollingResistance(int triIndex)
{//stub
	if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_ASPHALT*0.1f)<0.06f)
		return 0.73f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_CONCRETE*0.1f)<0.06f)
		return 0.66f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_SNOW*0.1f)<0.06f)
		return 1.00f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_GRASS*0.1f)<0.06f)
		return 0.35f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_ICE*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_WET_ASPHALT*0.1f)<0.06f)
		return 0.40f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_WET_CONCRETE*0.1f)<0.06f)
		return 0.48f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_DIRT*0.1f)<0.06f)
		return 0.47f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_GRAVEL*0.1f)<0.06f)
		return 1.42f;
	else //if(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_MUD)<0.001
		return 1.33f;
}
float CMxTrack::GetSufaceFriction(int triIndex)
{//kinetic friction
	if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_ASPHALT*0.1f)<0.06f)
		return 0.73f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_CONCRETE*0.1f)<0.06f)
		return 0.66f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_SNOW*0.1f)<0.06f)
		return 0.32f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_GRASS*0.1f)<0.06f)
		return 0.35f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_ICE*0.1f)<0.06f)
		return 0.24f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_WET_ASPHALT*0.1f)<0.06f)
		return 0.40f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_WET_CONCRETE*0.1f)<0.06f)
		return 0.48f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_DIRT*0.1f)<0.06f)
		return 0.47f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_GRAVEL*0.1f)<0.06f)
		return 0.42f;
	else //if(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_MUD)<0.001
		return 0.33f;
}

float CMxTrack::GetBumpAt(int triIndex, float u, float v)
{//stub... write noise functions later

	if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_ASPHALT*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_CONCRETE*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_SNOW*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_GRASS*0.1f)<0.06f)
		return sin(10*v)*0.05;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_ICE*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_WET_ASPHALT*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_WET_CONCRETE*0.1f)<0.06f)
		return 0.0f;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_DIRT*0.1f)<0.06f)
		return sin(20*v)*0.15;
	else if(fabs(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_GRAVEL*0.1f)<0.06f)
		return sin(100*v)*0.005;
	else //if(m_TireSplineGeom->m_TrisArray[triIndex].iSurfaceType - SURFACE_MUD)<0.001
		return sin(3*v)*0.05;

}