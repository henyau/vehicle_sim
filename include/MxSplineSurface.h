#pragma once
#include <math.h>
#include "Matrix.h"
#include "TrackGeom.h"

class CMxSplineSurface
{
	public:
		CMxSplineSurface(CTrackGeom *_trackGeom, int offset);
		CMxSplineSurface(Vector3p A,Vector3p B, Vector3p C, Vector3p D); //single segment constructor
		CMxSplineSurface(void);
		~CMxSplineSurface(void);

		void SetSplineFromTrack(CTrackGeom *_trackGeom, int offset);

		Vector3p getPosAt(int i, float t);
		Vector3p getTanAt(int i, float t);
		Vector3p *m_ControlPoints;

		int m_SplineLength;
		bool m_bOpen;

	private:
		float BezierBasis(int i, float t);// basis functions for spline
		float CatmullBasis(int i, float t);// basis functions for spline
		float CatmullTangetBasis(int i, float t);
		float BezierTangentBasis(int i, float t);
	
};
