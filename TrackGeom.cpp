#include "StdAfx.h"
#include "trackgeom.h"

//Want to be able to save as a series of tri strips. (from the older version of the sweeper)
//But first just make sure it can export vert/indices

CTrackGeom::CTrackGeom(void)
{
	m_VertexArray = 0;
	m_TrisArray = 0;
	m_VertexArraySize = 0;
	m_TrisArraySize = 0;
	m_Width = 0;
	m_Length = 0;
}

CTrackGeom::~CTrackGeom(void)
{
	free();

}
void CTrackGeom::free(void)
{
	if(m_VertexArray){
		delete []m_VertexArray;
		m_VertexArray = 0;
	}

	if(m_TrisArray){
		delete []m_TrisArray;
		m_TrisArray = 0;
	}
	m_VertexArraySize = 0;
	m_TrisArraySize = 0;
}

//void CTrackGeom::Process(CSpline *track)//comment out when using with MxCarEditor
//{
//	free();
//	m_VertexArraySize = track->m_VertexArraySize;
//	m_TrisArraySize = track->m_TrisArraySize;
//	m_VertexArray = new Vertex[m_VertexArraySize];
//	m_TrisArray = new Triangle[m_TrisArraySize];
//
// 	for( int i=0;i<m_VertexArraySize;i++)
//	{
//		m_VertexArray[i].x = track->m_VertexArray[i][0];
//		m_VertexArray[i].y = track->m_VertexArray[i][1];
//		m_VertexArray[i].z = track->m_VertexArray[i][2];
//	}
//
//	for(unsigned int i=0;i<m_TrisArraySize;i++)	
//	{		
//		m_TrisArray[i].A  = track->m_TrisArray[i][0];
//		m_TrisArray[i].B  = track->m_TrisArray[i][1];
//		m_TrisArray[i].C  = track->m_TrisArray[i][2];
//	}
//}
//int CTrackGeom::save( const char  *filename)
//{
//
//    FILE *fp = fopen(filename, "wb");
//    if (!fp)
//        return 20;
//    if (save(fp))
//        return 20;
//    fclose(fp);
//    return 0;
//}
//
//int CTrackGeom::save(FILE *fp)
//{
//    if (fwrite("GEO ", 1, 4, fp) != 4)
//        return 20;
//    if (fwrite(&m_VertexArraySize, sizeof(m_VertexArraySize), 1, fp) != 1)
//        return 20;
//	if (fwrite(&m_TrisArraySize, sizeof(m_TrisArraySize), 1, fp) != 1)
//        return 20;
//	if (fwrite(m_VertexArray, sizeof(Vertex), m_VertexArraySize, fp) != m_VertexArraySize)
//        return 20;
//	if (fwrite(m_TrisArray, sizeof(Triangle), m_TrisArraySize, fp) != m_TrisArraySize)
//		return 20;
// 
//    if (fwrite("END ", 1, 4, fp) != 4)
//        return 20;
//    return 0;
//}

/******************************************************************************
*   load
*
*   params :
*       char *filename                  -   file to load
*   returns :
*       int                             -   0 OK, 20 error
*
******************************************************************************/

int CTrackGeom::load(char *filename)
{
    FILE *fp = fopen(filename, "rb");
    if (!fp)
        return 20;
    if (load(fp))
        return 20;
    fclose(fp);
    return 0;
}

/******************************************************************************
*   load
*
*   params :
*       FILE *fp                        -   file to load
*   returns :
*       int                             -   0 OK, 20 error
*
******************************************************************************/

int CTrackGeom::load(FILE *fp)
{
    free();
    char type[4];
    if (fread(type, 1, 4, fp) != 4)
        return 20;
    if (memcmp(type, "GEO ", 4)) {
        return 20;
    }

	if (fread(&m_Length, sizeof(unsigned int), 1, fp) != 1)
        return 20;
	if (fread(&m_Width, sizeof(unsigned int), 1, fp) != 1)
        return 20;
	if (fread(&m_bOpen, sizeof(bool), 1, fp) != 1)
        return 20;
	if (fread(&m_VertexArraySize, sizeof(m_VertexArraySize), 1, fp) != 1)
        return 20;
    if (fread(&m_TrisArraySize, sizeof(m_TrisArraySize), 1, fp) != 1)
        return 20;


    m_VertexArray = new Vertex[m_VertexArraySize];
	if (fread(m_VertexArray, sizeof(Vertex), m_VertexArraySize, fp) != m_VertexArraySize)
        return 20;
    m_TrisArray = new Triangle[m_TrisArraySize];

	if (fread(m_TrisArray, sizeof(Triangle), m_TrisArraySize, fp) != m_TrisArraySize)
            return 20;
  

    if (fread(type, 1, 4, fp) != 4)
        return 20;
    if (memcmp(type, "END ", 4)) {
        return 20;
    }
    return 0;
}

