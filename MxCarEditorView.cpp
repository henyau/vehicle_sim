// MxCarEditorView.cpp : CMxCarEditorView 類別的實作
//NOTE:: 3/08/05. Henry.
//Rewritting renderer to use OpenGL.  The physics modules are independent of the graphics.
//MxCarEditorView acts as Main().

#include "stdafx.h"
#include "MxCarEditor.h"

#include "MxCarEditorDoc.h"
#include "MxCarEditorView.h"

#include "DebugConsole.h"
//#include "Renderer.h"
#include "drawstuff/drawstuff.h"
#include <GL/glu.h>

#include <iostream>
using namespace std;

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#endif


// CMxCarEditorView
IMPLEMENT_DYNCREATE(CMxCarEditorView, CView)

BEGIN_MESSAGE_MAP(CMxCarEditorView, CView)
	//{{AFX_MSG_MAP(CMxCarEditorView)
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_ERASEBKGND()
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

CMxCarEditorView::CMxCarEditorView()
{
	//RedirectIOToConsole();//create out super console; 
	m_bisinit =false;
	m_input.InitDirectInput();	
	elaspedTime = 0.0f;
	simulationTime = 0.0f;
	startTime =  GetTickCount() ;
	maxStepSize = 0.005;

}

CMxCarEditorView::~CMxCarEditorView()
{
	delete m_sim;
 }

int CMxCarEditorView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	if ( !InitOpenGL() )
	{
		MessageBox( "Error setting up OpenGL!", "Init Error!",
			MB_OK | MB_ICONERROR );
		return -1;
	}

	return 0;
}

BOOL CMxCarEditorView::SetupPixelFormat()
{
	static PIXELFORMATDESCRIPTOR pfd = 
	{
		sizeof(PIXELFORMATDESCRIPTOR),    // size of this pfd
		1,                                // version number
		PFD_DRAW_TO_WINDOW |              // support window
		PFD_SUPPORT_OPENGL |              // support OpenGL
		PFD_DOUBLEBUFFER,                 // double buffered
		PFD_TYPE_RGBA,                    // RGBA type
		32,                               // 24-bit color depth
		0, 0, 0, 0, 0, 0,                 // color bits ignored
		8,                                // alpha buffer
		0,                                // shift bit ignored
		0,                                // no accumulation buffer
		0, 0, 0, 0,                       // accumulation bits ignored
		16,                               // 16-bit z-buffer
		0,                                // no stencil buffer
		0,                                // no auxiliary buffer
		PFD_MAIN_PLANE,                   // main layer
		0,                                // reserved
		0, 0, 0                           // layer masks ignored
	};

   int m_nPixelFormat = ::ChoosePixelFormat( m_pDC->GetSafeHdc(), &pfd );

    if ( m_nPixelFormat == 0 )
        return FALSE;

    return ::SetPixelFormat( m_pDC->GetSafeHdc(), m_nPixelFormat, &pfd );
}

BOOL CMxCarEditorView::InitOpenGL()
{
	//Get a DC for the Client Area
	m_pDC = new CClientDC(this);

	//Failure to Get DC
	if( m_pDC == NULL )
		return FALSE;

	if( !SetupPixelFormat() )
		return FALSE;

    //Create Rendering Context
	m_hRC = ::wglCreateContext( m_pDC->GetSafeHdc() );

    //Failure to Create Rendering Context
    if( m_hRC == 0 )
        return FALSE;

	//Make the RC Current
	if( ::wglMakeCurrent( m_pDC->GetSafeHdc(), m_hRC ) == FALSE )
		return FALSE;

	// Usual OpenGL stuff

	glClearColor( 0.8f, 0.8f, 1.0f, 0.0f );
	glClearDepth(1.0f);
	glEnable( GL_DEPTH_TEST );
	glEnable( GL_TEXTURE_2D );
	glEnable(GL_ALPHA_TEST);
	//test light
	//glEnable(GL_LIGHTING);
	glDisable(GL_LIGHTING);
	glShadeModel (GL_FLAT);
	//glShadeModel (GL_SMOOTH);

	GLfloat light_ambient[] = { 0.8,1.0, 1.0, 1.0 };
	GLfloat light_diffuse[] = { 0.6, 0.6, 0.7, 0.0 };
	GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat light_position[] = { 1.0, 5.0, 10.0, 0.0 };//directional light

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	glEnable(GL_LIGHT0);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	glLineWidth( 1.0f );
	glPointSize( 4.0f );


	//test init font
	HDC hDC;
	hDC = m_pDC->GetSafeHdc();
	
	font = glGenLists( 255 );
    HFONT courier = CreateFont( 20,8, 0, 0, FW_DEMIBOLD, FALSE,
                    FALSE, FALSE, ANSI_CHARSET, OUT_TT_PRECIS,
                    CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY,
                    FF_DONTCARE| DEFAULT_PITCH, "Courier New" );

    SelectObject( hDC, courier );
    wglUseFontBitmaps( hDC, 0, 255, font );

	//set View Frustum
	CRect cr;
	GetClientRect(&cr);
	OnSize(0, cr.Width(), cr.Height());

	return TRUE;
}
void CMxCarEditorView::OnSize(UINT nType, int cx, int cy)
{
	//CMxCarEditorView::OnSize(nType, cx, cy);

	if ( 0 >= cx || 0 >= cy || nType == SIZE_MINIMIZED )
		return;

	// Change the perspective viewing volume to
	// reflect the new dimensions of the window.
	SetContext();
	glViewport( 0, 0, cx, cy );
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluPerspective(45.0f, (float)(cx)/(float)(cy), 0.01f, 20000.0f);
	glMatrixMode( GL_MODELVIEW );

}


void CMxCarEditorView::RenderScene()
{//test for now.  clean later (put car rendering in its own class...)
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	CRect cr;
	GetClientRect(&cr);
	OnSize(0, cr.Width(), cr.Height());
	
	//draw track
	glLineWidth( 1.0f );
	m_sim->m_Track->Render();

  	//glEnable(GL_LIGHTING);
	float pos[3];
	float R[12];
	float sides[3];
	static float a = 0;
	//extern CMxCarAttribute m_CarAttr;

	for(int j = 0; j < m_sim->m_iNumCars; j++)
	{
	sides[0] = m_sim->m_Car[j]->mCarbody.m_rWidth;
	sides[1] = m_sim->m_Car[j]->mCarbody.m_rHeight;
	sides[2] =  m_sim->m_Car[j]->mCarbody.m_rLength;

	dsSetTexture (DS_WOOD);
 
	pos[0] = m_sim->m_Car[j]->mCarbody.linPos.x;
 	pos[1] =m_sim->m_Car[j]->mCarbody.linPos.y;
	pos[2] = m_sim->m_Car[j]->mCarbody.linPos.z;
	
	R[0] = m_sim->m_Car[j]->mCarbody.rotPos.m[0];
	R[1] = m_sim->m_Car[j]->mCarbody.rotPos.m[1];
	R[2] = m_sim->m_Car[j]->mCarbody.rotPos.m[2];

	R[4] = m_sim->m_Car[j]->mCarbody.rotPos.m[3];
	R[5] = m_sim->m_Car[j]->mCarbody.rotPos.m[4];
	R[6] = m_sim->m_Car[j]->mCarbody.rotPos.m[5];

	R[8] = m_sim->m_Car[j]->mCarbody.rotPos.m[6];
	R[9] = m_sim->m_Car[j]->mCarbody.rotPos.m[7];
	R[10] = m_sim->m_Car[j]->mCarbody.rotPos.m[8];
  

	//draw car body
	glLineWidth( 1.0f );
	//dsSetColor (0.3,0.7f,0.0f);
 	glColor3f(0.0f,0.0f, 0.0f);
	dsDrawBox(pos,R,sides);	
	//draw engine
	sides[0] = m_sim->m_Car[j]->mEngine.m_rWidth;
	sides[1] = m_sim->m_Car[j]->mEngine.m_rHeight;
	sides[2] = m_sim->m_Car[j]->mEngine.m_rLength;

	pos[0] += m_sim->m_Car[j]->mEngine.m_rPosx*R[0]   +m_sim->m_Car[j]->mEngine.m_rPosy*R[1]   +m_sim->m_Car[j]->mEngine.m_rPosz*R[2] ;
	pos[1] += m_sim->m_Car[j]->mEngine.m_rPosx*R[4]   +m_sim->m_Car[j]->mEngine.m_rPosy*R[5]   +m_sim->m_Car[j]->mEngine.m_rPosz*R[6] ;
  	pos[2] += m_sim->m_Car[j]->mEngine.m_rPosx*R[8]   +m_sim->m_Car[j]->mEngine.m_rPosy*R[9]   +m_sim->m_Car[j]->mEngine.m_rPosz*R[10] ;
 	dsDrawBox(pos,R,sides);	
      	
	
	glDisable(GL_LIGHTING);
//	glLineWidth( 3.0f );
   for(int i = 0; i<4; i++)
   {
	   //linearPos
		pos[0] = m_sim->m_Car[j]->mWheels[i]->positionWld.x;
 		pos[1] = m_sim->m_Car[j]->mWheels[i]->positionWld.y ;
		pos[2] = m_sim->m_Car[j]->mWheels[i]->positionWld.z;

		//rotationalPos
		R[0] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[0];
		R[1] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[1];
		R[2] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[2];

		R[4] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[3];
		R[5] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[4];
		R[6] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[5];

		R[8] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[6];
		R[9] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[7];
		R[10] = m_sim->m_Car[j]->mWheels[i]->rotPosWorld.m[8];
		
		//draw wheel
 		glPushMatrix();//no transformations are made, but just in case in the future...
 		if(fabs(m_sim->m_Car[j]->mWheels[i]->slipRatioPos)>1)
 			glColor4f(1.0f, 0.0f, 0.0f, 0.8f);
		else
			glColor3f(0.3f, 0.1f, 0.2f);
 		dsDrawCylinder(pos,R, m_sim->m_Car[j]->mWheels[i]->width, m_sim->m_Car[j]->mWheels[i]->radius);		


		if(j ==m_sim->m_iPlayerNumber)
		{

 		//draw forces
		glLineWidth( 1.0f );
		glColor3f(1.0, 0.0, 0.0);
		glBegin(GL_LINES);		
		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z);
		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x + m_sim->m_Car[j]->mWheels[i]->debugForceLong.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y + m_sim->m_Car[j]->mWheels[i]->debugForceLong.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z + m_sim->m_Car[j]->mWheels[i]->debugForceLong.z);
		glEnd();

 		glBegin(GL_LINES);
  		glColor4f(0.0, 1.0, 0.0, 1.0);
		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z);
		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x + m_sim->m_Car[j]->mWheels[i]->debugForceLat.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y + m_sim->m_Car[j]->mWheels[i]->debugForceLat.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z + m_sim->m_Car[j]->mWheels[i]->debugForceLat.z);
 		glEnd();

		glBegin(GL_LINES);
 		glColor4f(0.0, 0.0, 1.0, 1.0);
 		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z);
		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x + m_sim->m_Car[j]->mWheels[i]->debugForceLoad.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y + m_sim->m_Car[j]->mWheels[i]->debugForceLoad.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z + m_sim->m_Car[j]->mWheels[i]->debugForceLoad.z);
		glEnd();

		//total forces in ODE's force accumulator acting on the CoM of carbody.
		glBegin(GL_LINES);
 		glColor4f(1.0, 0.0, 1.0, 1.0);
		glVertex3f(m_sim->m_Car[j]->mCarbody.linPos.x,
			m_sim->m_Car[j]->mCarbody.linPos.y,
			m_sim->m_Car[j]->mCarbody.linPos.z);
		glVertex3f(m_sim->m_Car[j]->mCarbody.linPos.x + m_sim->m_Car[j]->mCarbody.linForce.x,
			m_sim->m_Car[j]->mCarbody.linPos.y +  m_sim->m_Car[j]->mCarbody.linForce.y,
 			m_sim->m_Car[j]->mCarbody.linPos.z +  m_sim->m_Car[j]->mCarbody.linForce.z);
		glEnd();

//draw velocities
		glBegin(GL_LINES);
  		glColor4f(0.3, 0.5, 0.5, 0.5);
 		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z);
		glVertex3f(m_sim->m_Car[j]->mWheels[i]->positionWld.x + m_sim->m_Car[j]->mWheels[i]->velocityWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y + m_sim->m_Car[j]->mWheels[i]->velocityWld.y,
 			m_sim->m_Car[j]->mWheels[i]->positionWld.z + m_sim->m_Car[j]->mWheels[i]->velocityWld.z);
		glEnd();

		glBegin(GL_LINES);
  		glColor4f(0.0, 0.0, 0.0, 1.0);
		glVertex3f(m_sim->m_Car[j]->mCarbody.linPos.x,
			m_sim->m_Car[j]->mCarbody.linPos.y,
			m_sim->m_Car[j]->mCarbody.linPos.z);
		glVertex3f(m_sim->m_Car[j]->mCarbody.linPos.x + m_sim->m_Car[j]->mCarbody.linVel.x,
			m_sim->m_Car[j]->mCarbody.linPos.y +  m_sim->m_Car[j]->mCarbody.linVel.y,
 			m_sim->m_Car[j]->mCarbody.linPos.z +  m_sim->m_Car[j]->mCarbody.linVel.z);
		glEnd();
		///

		///Draw contact pos/normal
		glBegin(GL_LINES);
  		glColor4f(0.0, 1.0, 1.0, 1.0);
		glVertex3f(m_sim->debugContact.pos[0],
			m_sim->debugContact.pos[1],
			m_sim->debugContact.pos[2]);
		glVertex3f(m_sim->debugContact.pos[0]+m_sim->debugContact.normal[0],
			m_sim->debugContact.pos[1]+m_sim->debugContact.normal[1],
			m_sim->debugContact.pos[2]+m_sim->debugContact.normal[2]);
		glEnd();


	glDisable(GL_DEPTH_TEST);

		//gl_printf(1.0f, 0.0f, 0.0f, 1.0f,
		//	m_sim->m_Car[j]->mCarbody.linPos.x,
		//	m_sim->m_Car[j]->mCarbody.linPos.y,
		//	m_sim->m_Car[j]->mCarbody.linPos.z,
		//	font,
		//	"%5.2f, %5.2f, %5.2f",
		//	(GLfloat)m_sim->m_Car[j]->mCarbody.linPos.x,
		//	(GLfloat)m_sim->m_Car[j]->mCarbody.linPos.y,
  //			(GLfloat)m_sim->m_Car[j]->mCarbody.linPos.z  );  
	
					
			
			gl_printf(0.0f,0.0f, 0.0f, 1.0f,
				m_sim->m_Car[j]->mWheels[i]->positionWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z,
			font,
			"%3.3f",
 			//m_sim->m_Car[j]->mWheels[i]->debugForceLat.mod()  );  
			//atan(m_sim->m_Car[j]->mWheels[i]->tanSlipAnglePos)*57.295779 );  
       			//m_sim->m_Car[j]->mWheels[i]->slipRatioPos  );	
				m_sim->m_Car[j]->mWheels[i]->debugForceLong.mod());  
}
/*
			gl_printf(1.0f, 0.0f, 0.0f, 1.0f,
			m_sim->m_Car[j]->mWheels[i]->positionWld.x,
			m_sim->m_Car[j]->mWheels[i]->positionWld.y,
			m_sim->m_Car[j]->mWheels[i]->positionWld.z,
			font,
			"%3.1f",
			m_sim->m_Car[j]->mWheels[i]->rotForce);  
*/


//	glEnable(GL_DEPTH_TEST);
		
		glPopMatrix();
   }
}
   
   glLoadIdentity();	

	//if(m_sim->m_Car[0]->mCarbody.linVel.mod()>2)
	//{
	Vector3p vLookAtVel;
	vLookAtVel.x = m_sim->m_Car[0]->mCarbody.rotPos.m[2];
	vLookAtVel.y = m_sim->m_Car[0]->mCarbody.rotPos.m[5];
  	vLookAtVel.z = m_sim->m_Car[0]->mCarbody.rotPos.m[8];

//	= m_sim->m_Car[0]->mCarbody.linVel;

	vLookAtVel.normalise();
	      	gluLookAt(//pos[0],pos[1]+7,pos[2]-7, 
		
			m_sim->m_Car[0]->mCarbody.linPos.x-vLookAtVel.x*10,
			m_sim->m_Car[0]->mCarbody.linPos.y+5,
			m_sim->m_Car[0]->mCarbody.linPos.z-vLookAtVel.z*10,

       		m_sim->m_Car[0]->mCarbody.linPos.x+vLookAtVel.x*20,
			m_sim->m_Car[0]->mCarbody.linPos.y,
			m_sim->m_Car[0]->mCarbody.linPos.z+vLookAtVel.z*20,
			
            			0,1,0);

	///screen coord printf
	glDisable(GL_DEPTH_TEST);
	GLdouble  modelMatrix[16];
	GLdouble  projMatrix[16];
	GLint  viewport[4];
	GLdouble screenx, screeny, screenz;
 
//	glLoadIdentity();
	// Position the camera
	//glTranslatef( m_sim->m_Car[0]->mCarbody.linPos.x-vLookAtVel.x*10, 
	//					 m_sim->m_Car[0]->mCarbody.linPos.y+2, 
	//					 m_sim->m_Car[0]->mCarbody.linPos.z-vLookAtVel.z*10);
 	GetClientRect(&cr);

	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	glGetIntegerv(GL_VIEWPORT,viewport);
 
  	gluUnProject(10.0f,cr.Height() - 20.0f, 0.0f, modelMatrix, projMatrix, viewport, &screenx, &screeny, &screenz);
	gl_printf(0,0,0,1, screenx, screeny, screenz,font,
      			"Thrl: %3.1f  Gr: %3.1f  EngVel:%3.1f CarVel:%3.1f	",
				 m_sim->m_Car[0]->mEngine.throttle, 
				 (float) m_sim->m_Car[0]->mEngine.m_iCurrentGear,
				 (m_sim->m_Car[0]->mEngine.vel)*9.54929658551f ,
				 m_sim->m_Car[0]->mCarbody.linVel.mod()*3.6f);
 

	gluUnProject(10.0f,cr.Height() - 40.0f, 0.0f, modelMatrix, projMatrix, viewport, &screenx, &screeny, &screenz);
	gl_printf(0,0,0,1, screenx, screeny, screenz,font,
 		"SimTime:%3.1f	  EngineTorque: %3.1f", (float) simulationTime, m_sim->m_Car[0]->mEngine.m_rTorque);
 

	
	//make a some sort of g-ball
	Vector3p forceCC;
	forceCC.transMult(m_sim->m_Car[0]->mCarbody.rotPos, m_sim->m_Car[0]->mCarbody.linForce);

	gluUnProject(cr.Width()/2 +forceCC.x*0.005f,
		cr.Height() - 70.0f-forceCC.z*0.005f,
    		0.0f, modelMatrix, projMatrix, viewport, &screenx, &screeny, &screenz);

 	glPointSize(10.0f);
  	glColor3f(0.2,0.4,0);
 	glBegin(GL_POINTS);
	glVertex3f(screenx, screeny, screenz);
	glEnd();

	glBegin(GL_LINES);
	glVertex3f(screenx, screeny, screenz);
 	gluUnProject(cr.Width()/2 ,
		cr.Height() - 70.0f,0.0f, modelMatrix, projMatrix, viewport, &screenx, &screeny, &screenz);
	glVertex3f(screenx, screeny, screenz);
	glEnd();



	glEnable(GL_DEPTH_TEST);
	//glPopMatrix();
	///


// }
	//else

	//{		
	//      	gluLookAt(//pos[0],pos[1]+7,pos[2]-7, 
	//			m_sim->m_Car[0]->mCarbody.linPos.x,
	//		m_sim->m_Car[0]->mCarbody.linPos.y+10,
	//		m_sim->m_Car[0]->mCarbody.linPos.z-20,

 //      		m_sim->m_Car[0]->mCarbody.linPos.x,
	//		m_sim->m_Car[0]->mCarbody.linPos.y,
	//		m_sim->m_Car[0]->mCarbody.linPos.z+40,
	//	
	//		0,1,0); }


		    
	
}
void CMxCarEditorView::OnDraw(CDC *pDC)
{		
	startTime	= GetTickCount();	

	//create sim if not already created. (was put here because CSim use to rely on OGRE initialization)
	//still need to move ODE things before can move this into the constructor
	if(m_sim->m_Inst == 0)
	{
		m_sim = new CSim();		
	}

	float wheelPos =m_input.GetWheelPosition();
		wheelPos -=65535.000f*.5f;
		wheelPos *=3.051804e-5;
	float brakePos =m_input.GetBrakePosition();
		brakePos -=  767.0f;// for my set... need to write calibration program
		brakePos*=1.54397233202e-5;
		if(brakePos<0)
			brakePos =0;
	float throttlePos =m_input.GetThrottlePosition();
		throttlePos-=1023.0f;//65535.000f;// for my set... need to write calibration program
		throttlePos*=1.55009920635e-5;
		if(throttlePos<0)
			throttlePos =0;
		throttlePos = fabs(throttlePos -1);//oops throttle is inverted, so fix...  rewrite later.
		brakePos = fabs(brakePos -1);//oops throttle is inverted, so fix...  rewrite later.

		if (m_input.ReloadCar())
		{
 			m_sim->m_Car[0]->Reload(0,3,0);
 		}

		//To determine step sizes
	

	//update forces acting on car (not ODE)
 	int numSteps = int(ceil(elaspedTime / maxStepSize)); 
 	if(numSteps <2)
 		numSteps = 2;
	if(numSteps >20)
 		numSteps = 20;
	if(elaspedTime<=0)
  		elaspedTime = 0.0001f;//what the fuck ... it should not reach here
	float step = elaspedTime / numSteps;
	if(step<=0)
		step = 0.000000001f;//prevent divide by zero

    	float variStepSize = elaspedTime/10.0f;

   	for(int i = 0; i <5; i++)//real loop wouldn't waste time on MFC stuff. find better way of finding frame time.
	{
		m_sim->SetControlInputs(variStepSize, throttlePos, brakePos, wheelPos, m_input.ButtonPressed(1), m_input.ButtonPressed(0));	// multiple samples per frame
		m_sim->Update(variStepSize);		
	}
	
	//FFeffect
   	m_input.InitFFEffect();

	//rendering
	SetContext();
	RenderScene();
	SwapGLBuffers();
	//end rendering	


  	   cout<<elaspedTime <<endl;
	//dos-style console window misc dump
	/*cout<<"throttle:  "<<m_sim->m_Car[0]->mEngine.throttle
		<<"   Gear:   "<<m_sim->m_Car[0]->mEngine.m_iCurrentGear
		<<"   engine vel:  "<<(m_sim->m_Car[0]->mEngine.vel)*9.54929658551f 
		<<"	 vehicle vel:  "<<m_sim->m_Car[0]->mCarbody.linVel.mod()*3.6f
    		<<endl;*/
	
	/*cout<<m_sim->m_Car[0]->mWheels[0]->debugForceLong.mod()<<"    "
		<<m_sim->m_Car[0]->mWheels[1]->debugForceLong.mod()<<"    "
		<<m_sim->m_Car[0]->mWheels[2]->debugForceLong.mod()<<"    "
		<<m_sim->m_Car[0]->mWheels[3]->debugForceLong.mod()<<"    "<<endl;*/
/*
	cout<<"0 slipRatio:"<<
	(m_sim->m_Car[0]->mWheels[0]->slipRatioPos)<<
		"    1 slipRatio:"<<
		(m_sim->m_Car[0]->mWheels[1]->slipRatioPos)<<
		"    2 slipRatio:"<<
		(m_sim->m_Car[0]->mWheels[2]->slipRatioPos)<<
		"    3 slipRatio:"<<
		(m_sim->m_Car[0]->mWheels[3]->slipRatioPos)<<
 	endl;*/
		/*
	cout<<"0SlipAng:"<<
		atan(m_sim->m_Car[0]->mWheels[0]->tanSlipAnglePos)*57.295779<<
		"    1SlipAng:"<<
		atan(m_sim->m_Car[0]->mWheels[1]->tanSlipAnglePos)*57.295779<<
		"    2SlipAng:"<<
		atan(m_sim->m_Car[0]->mWheels[2]->tanSlipAnglePos)*57.295779<<
		"    3SlipAng:"<<
		atan(m_sim->m_Car[0]->mWheels[3]->tanSlipAnglePos)*57.295779<<
 	endl;
*/

 	
	Invalidate();//force next frame redraw
	
 	elaspedTime = (GetTickCount() - startTime) / 1000.0f;   //how much time it takes to process and then render frame
   	//elaspedTime*=2;
   	simulationTime+=elaspedTime;
}


void CMxCarEditorView::DoDataExchange(CDataExchange* pDX)
{
	CView::DoDataExchange(pDX);
}


BOOL CMxCarEditorView::PreCreateWindow(CREATESTRUCT& cs) 
{
	cs.lpszClass = ::AfxRegisterWndClass(CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS | CS_OWNDC,
		::LoadCursor(NULL, IDC_CROSS), NULL, NULL);
	cs.style |= WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
	
	return CView::PreCreateWindow(cs);
}

void CMxCarEditorView::gl_printf( GLfloat red, GLfloat green, GLfloat blue, GLfloat alpha,
								            GLfloat x, GLfloat y, GLfloat z,GLuint font, const char *format, ... )
//                GLint x, GLint y, GLint z,GLuint font, const char *format, ... )
{
    va_list argp;
    char text[256];

    va_start( argp, format );
    vsprintf( text, format, argp );
    va_end( argp );
    glColor4f( red, green, blue, alpha );

     glRasterPos3f( x,  y,  z);
    glListBase( font );
    glCallLists( strlen( text ), GL_UNSIGNED_BYTE, text );

	
}

void CMxCarEditorView::OnDestroy()
{
	CView::OnDestroy();

	wglMakeCurrent(0,0);
	wglDeleteContext(m_hRC);
	if( m_pDC )
	{
		delete m_pDC;
	}
	m_pDC = NULL;
}

BOOL CMxCarEditorView::OnEraseBkgnd(CDC* pDC) 
{	
	return TRUE;
}


// CMxCarEditorView diagnostics

#ifdef _DEBUG
void CMxCarEditorView::AssertValid() const
{
	CView::AssertValid();
}

void CMxCarEditorView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}


#endif //_DEBUG


// CMxCarEditorView message handlers
