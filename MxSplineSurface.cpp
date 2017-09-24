#include "StdAfx.h"
#include "MxSplineSurface.h"
#define USE_CATMULL_SPLINE

CMxSplineSurface::CMxSplineSurface()
{

}
CMxSplineSurface::CMxSplineSurface(CTrackGeom *_trackGeom, int offset)
{
	//int WIDTH = _trackGeom->m_Width; // 2* loft segments
	m_SplineLength = _trackGeom->m_Length;//_trackGeom->m_VertexArraySize/WIDTH;
	m_ControlPoints = new Vector3p[m_SplineLength];
	int j = 0;
	for(int i = offset*(m_SplineLength); i<(offset+1)*(m_SplineLength); i++)
	{
		m_ControlPoints[j].x = _trackGeom->m_VertexArray[i].x;
		m_ControlPoints[j].y = _trackGeom->m_VertexArray[i].y; 
		m_ControlPoints[j].z = _trackGeom->m_VertexArray[i].z;
		j++;
	}
	m_bOpen = _trackGeom->m_bOpen;
}

void CMxSplineSurface::SetSplineFromTrack(CTrackGeom *_trackGeom, int offset)
{
	m_SplineLength = _trackGeom->m_Length;
	m_ControlPoints = new Vector3p[m_SplineLength];
	int j = 0;
	for(int i = offset*(m_SplineLength); i<(offset+1)*(m_SplineLength); i++)
	{
		m_ControlPoints[j].x = _trackGeom->m_VertexArray[i].x;
		m_ControlPoints[j].y = _trackGeom->m_VertexArray[i].y; 
		m_ControlPoints[j].z = _trackGeom->m_VertexArray[i].z;
		j++;
	}
	m_bOpen = _trackGeom->m_bOpen;


}
CMxSplineSurface::CMxSplineSurface(Vector3p A,Vector3p B, Vector3p C, Vector3p D)
{//single curve 

	m_ControlPoints = new Vector3p[4];
	m_SplineLength = 4;
	m_ControlPoints[0] = A;
	m_ControlPoints[1] = B;
	m_ControlPoints[2] = C;
	m_ControlPoints[3] = D;
	m_bOpen = true;

}
CMxSplineSurface::~CMxSplineSurface(void)
{
	if(m_ControlPoints)
	{
		delete []m_ControlPoints;
		m_ControlPoints = 0;
	}
}


  // the basis function for a cubic B spline
float CMxSplineSurface::BezierBasis(int i, float t) {
    switch (i) {
    case -2:
      return (((-t+3)*t-3)*t+1)/6;
    case -1:
      return (((3*t-6)*t)*t+4)/6;
    case 0:
      return (((-3*t+3)*t+3)*t+1)/6;
    case 1:
      return (t*t*t)/6;
    }
    return 0; //we only get here if an invalid i is specified
  }

float CMxSplineSurface::BezierTangentBasis(int i, float t) {
    switch (i) {
    case -2:
      return (-0.5f*t+1.0f)*t - 0.5f;
    case -1:
      return ((9*t-12)*t)/6.0f;
    case 0:
      return ((-9*t + 6) *t +3)/6.0f;
    case 1:
      return (0.5*t*t);
    }
    return 0; //we only get here if an invalid i is specified
  }

  
 float CMxSplineSurface::CatmullBasis(int i, float t) {
    switch (i) {
    case -2:
      return ((-t+2)*t-1)*t/2;
    case -1:
      return (((3*t-5)*t)*t+2)/2;
    case 0:
      return ((-3*t+4)*t+1)*t/2;
    case 1:
      return ((t-1)*t*t)/2;
    }
    return 0; //we only get here if an invalid i is specified
  }

  float CMxSplineSurface::CatmullTangetBasis(int i, float t) {
    switch (i) {
    case -2:
      return ((-1.5f*t + 2.0f)*t -0.5f);
    case -1:
      return (4.5f*t - 5.0f)*t ;
    case 0:
      return ((-4.5f*t +4.0f)*t + 0.5f );
    case 1:
      return (1.5f*t - 1.0f)*t;
    }
    return 0; //we only get here if an invalid i is specified
  }

  //evaluate a point on the B spline
Vector3p CMxSplineSurface::getPosAt(int i, float t) 
{
	float px=0;
    float py=0;
	float pz=0;
 //   for (int j = -2; j<=1; j++)
	//{
	//	px += CatmullBasis(j,t)*m_ControlPoints[i+j].x;
	//	py += CatmullBasis(j,t)*m_ControlPoints[i+j].y;
	//	pz += CatmullBasis(j,t)*m_ControlPoints[i+j].z;
	//	
	//}

	int index;
    for (int j = -2; j<=1; j++)
	{
		index = i+j;
		if(m_bOpen)
		{
			if(index<0)
				index = 0;
			else if(index>=m_SplineLength)
				index = m_SplineLength-1;
		}
		else //wrap around.
		{
			if(index<0)
				index += m_SplineLength;
			else if(index>=m_SplineLength)
				index -= m_SplineLength;
		}


#ifdef USE_CATMULL_SPLINE
		px += CatmullBasis(j,t)*m_ControlPoints[index].x;
		py += CatmullBasis(j,t)*m_ControlPoints[index].y;
		pz += CatmullBasis(j,t)*m_ControlPoints[index].z;
#else		
		px += BezierBasis(j,t)*m_ControlPoints[index].x;
		py += BezierBasis(j,t)*m_ControlPoints[index].y;
		pz += BezierBasis(j,t)*m_ControlPoints[index].z;
#endif
		
	}
	
	return Vector3p(px, py, pz);
}


Vector3p CMxSplineSurface::getTanAt(int i, float t) 
{
	float px=0;
    float py=0;
	float pz=0;
	int index;
    for (int j = -2; j<=1; j++)
	{
		index = i+j;
		if(m_bOpen)
		{
			if(index<0)
				index = 0;
			else if(index>=m_SplineLength)
				index = m_SplineLength-1;
		}
		else //wrap around.
		{
			if(index<0)
				index += m_SplineLength;
			else if(index>=m_SplineLength)
				index -= m_SplineLength;
		}

#ifdef USE_CATMULL_SPLINE
		px += CatmullTangetBasis(j,t)*m_ControlPoints[index].x;
		py += CatmullTangetBasis(j,t)*m_ControlPoints[index].y;
		pz += CatmullTangetBasis(j,t)*m_ControlPoints[index].z;
#else
		px += BezierTangentBasis(j,t)*m_ControlPoints[index].x;
		py += BezierTangentBasis(j,t)*m_ControlPoints[index].y;
		pz += BezierTangentBasis(j,t)*m_ControlPoints[index].z;
#endif
		
	}
	
	return Vector3p(px, py, pz);

	/*Vector3p vPos = getPosAt(i, t);
	Vector3p vPos2;
	if(t<0.9)
	{
		vPos2 = getPosAt(i, t+0.5);
		vPos.sub(vPos2, vPos);
	}
	else
	{
		vPos2 = getPosAt(i, t-0.5);
		vPos.sub(vPos, vPos2);
	}
		
	return (vPos);*/
	
}

