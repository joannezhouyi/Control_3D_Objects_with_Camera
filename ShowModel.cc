/*  */




#include <cassert>
#include <iostream> // std::cout, std::endl
#include <memory>   // std::shared_ptr
#include <thread>   // std::this_thread
#include <stack>    // std::stack
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include <Python.h>
#include <boost/python.hpp>

#include <GL/glut.h>
#include <glm/glm.hpp>
#include <glm/gtx/component_wise.hpp>


using namespace std;
using namespace glm;

struct Vertex
{
    vec3 position;
    vec3 normal;
};

vector< Vertex > LoadM( istream& in )
{
    vector< Vertex > verts;

    map< int, vec3 > positions;

    string lineStr;
    while( getline( in, lineStr ) )
    {
        istringstream lineSS( lineStr );
        string lineType;
        lineSS >> lineType;

        // parse vertex line
        if( lineType == "Vertex" )
        {
            int idx = 0;
            float x = 0, y = 0, z = 0;
            lineSS >> idx >> x >> y >> z;
            positions.insert( make_pair( idx, vec3( x, y, z ) ) );
        }

        // parse face line
        if( lineType == "Face" )
        {
            int indexes[ 3 ] = { 0 };
            int idx = 0;
            lineSS >> idx >> indexes[0] >> indexes[1] >> indexes[2];

            // http://www.opengl.org/wiki/Calculating_a_Surface_Normal
            vec3 U( positions[ indexes[1] ] - positions[ indexes[0] ] );
            vec3 V( positions[ indexes[2] ] - positions[ indexes[0] ] );
            vec3 faceNormal = normalize( cross( U, V ) );

            for( size_t j = 0; j < 3; ++j )
            {
                Vertex vert;
                vert.position = vec3( positions[ indexes[j] ] );
                vert.normal   = faceNormal;
                verts.push_back( vert );
            }
        }
    }

    return verts;
}


// for rotation and translation
ivec2 curRot;
ivec2 curTrans;

// for zoom in/out
float gfScaling = 0.5;
float gfScalingAcc = 0.03;
float curScaling = 1.0;
void PassMotion(int iX, int iY)
{
    glutPostRedisplay();
}
void vMotion(int iSX1, int iSY1, int iSX2, int iSY2, int iFlag, int iStart)
{
    ivec2 PosRos(iSX1, iSY1);
    ivec2 PosTrans(iSX2, iSY2);
    ivec2 PosStart(iSX1, iSY1);
    ivec2 PosDiff(iSX2-iSX1, iSY2-iSY1);
    if(iFlag == 0 && iStart==0)
    {
        curRot = PosRos;
        curTrans = PosTrans;
    }
    else if(iStart==1)
    {
        curScaling = gfScaling + (float)( iSX2-iSX1 ) / 800.0f;
    }
    glutPostRedisplay();
}

vector< Vertex > model;
void display()
{
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    double w = glutGet( GLUT_WINDOW_WIDTH );
    double h = glutGet( GLUT_WINDOW_HEIGHT );
    double ar = w / h;
    // "pan"
    glTranslatef( curTrans.x / w * 2, curTrans.y / h * 2, 0 );
    gluPerspective( 60, ar, 0.1, 20 );

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    glTranslatef( 0, 0, -10 );

    glPushMatrix();
    // apply mouse rotation
    glRotatef( curRot.x % 360, 0, 1, 0 );
    glRotatef( -curRot.y % 360, 1, 0, 0 );

    // apply zoom in/out
    glScalef(curScaling, curScaling, curScaling);

    //glColor3ub( 255, 0, 0 );
    glColor3ub( 128, 128, 128 );

    // draw model
    glEnableClientState( GL_VERTEX_ARRAY );
    glEnableClientState( GL_NORMAL_ARRAY );
    glVertexPointer( 3, GL_FLOAT, sizeof(Vertex), &model[0].position );
    glNormalPointer( GL_FLOAT, sizeof(Vertex), &model[0].normal );
    glDrawArrays( GL_TRIANGLES, 0, model.size() );
    glDisableClientState( GL_VERTEX_ARRAY );
    glDisableClientState( GL_NORMAL_ARRAY );

    // draw bounding cube
    glDisable( GL_LIGHTING );
    glColor3ub( 255, 255, 255 );
    glutWireCube( 7 );
    glEnable( GL_LIGHTING );

    glPopMatrix();

    glutSwapBuffers();
}

// return the x/y/z min/max of some geometry
template< typename Vec >
pair< Vec, Vec > GetExtents
    ( 
    const Vec* pts, 
    size_t stride, 
    size_t count 
    )
{
    typedef typename Vec::value_type Scalar;
    Vec pmin( std::numeric_limits< Scalar >::max() );
    Vec pmax( std::min( std::numeric_limits< Scalar >::min(),
                        (Scalar)-std::numeric_limits< Scalar >::max() ) );

    // find extents
    unsigned char* base = (unsigned char*)pts;
    for( size_t i = 0; i < count; ++i )
    {
        const Vec& pt = *(Vec*)base;
        pmin = glm::min( pmin, pt );
        pmax = glm::max( pmax, pt );
        base += stride;
    }

    return make_pair( pmin, pmax );
}

// centers geometry around the origin
// and scales it to fit in a size^3 box
template< typename Vec > 
void CenterAndScale
    (
    Vec* pts, 
    size_t stride, 
    size_t count,
    const typename Vec::value_type& size
    )
{
    typedef typename Vec::value_type Scalar;

    // get min/max extents
    pair< Vec, Vec > exts = GetExtents( pts, stride, count );

    // center and scale 
    const Vec center = ( exts.first * Scalar( 0.5 ) ) + ( exts.second * Scalar( 0.5f ) );

    const Scalar factor = size / glm::compMax( exts.second - exts.first );
    unsigned char* base = (unsigned char*)pts;
    for( size_t i = 0; i < count; ++i )
    {
        Vec& pt = *(Vec*)base;
        pt = ((pt - center) * factor);
        base += stride;
    }    
}


/// @brief Guard that will acquire the GIL upon construction, and
///        restore its state upon destruction.
class with_gil
{
public:
    with_gil()  { state_ = PyGILState_Ensure(); }
    ~with_gil() { PyGILState_Release(state_);   }

    with_gil(const with_gil&)            = delete;
    with_gil& operator=(const with_gil&) = delete;
private:
    PyGILState_STATE state_;
};

/// @brief Guard that will unlock the GIL upon construction, and
///        restore its staet upon destruction.
class without_gil
{
public:
    without_gil()  { state_ = PyEval_SaveThread(); }
    ~without_gil() { PyEval_RestoreThread(state_); }

    without_gil(const without_gil&)            = delete;
    without_gil& operator=(const without_gil&) = delete;
private:
    PyThreadState* state_;
};

/// @brief Guard that provides higher-level GIL controls.
class gil_guard
{
public:
    struct no_acquire_t {} // tag type used for gil acquire strategy
    static no_acquire;

    gil_guard()             { acquire(); }
    gil_guard(no_acquire_t) { release(); }
    ~gil_guard()            { while (!stack_.empty()) { restore(); } }

    void acquire()          { stack_.emplace(new with_gil); }
    void release()          { stack_.emplace(new without_gil); }
    void restore()          { stack_.pop(); }

    static bool owns_gil()
    {
    // For Python 3.4+, one can use `PyGILState_Check()`.
    return _PyThreadState_Current == PyGILState_GetThisThreadState();
    }

    gil_guard(const gil_guard&)            = delete;
    gil_guard& operator=(const gil_guard&) = delete;

private:
    // Use std::shared_ptr<void> for type erasure.
    std::stack<std::shared_ptr<void>> stack_;
};


struct stShowModel
{
    void vReflashModel()
    {
        assert(gil_guard::owns_gil());
        gil_guard gil(gil_guard::no_acquire);
        assert(!gil.owns_gil());

        ifstream ifile( "/home/jxgu/github/CamControl_3D_Objects/gargoyle.m" );
        model = LoadM( ifile );
        if( model.empty() )
        {
            cerr << "Empty model!" << endl;
        }

        CenterAndScale( &model[0].position, sizeof( Vertex ), model.size(), 7 );

        int iArgc = 1;
        char *pcArgv = "Window";
        glutInit( &iArgc, &pcArgv );
        glutInitDisplayMode( GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE );
        glutInitWindowSize( 640, 480 );
        glutCreateWindow( "GLUT" );
        glutDisplayFunc( display );

        glutPassiveMotionFunc(PassMotion);

        glEnable( GL_DEPTH_TEST );

        // set up "headlamp"-like light
        glShadeModel( GL_SMOOTH );
        glEnable( GL_COLOR_MATERIAL );
        glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
        glEnable( GL_LIGHTING );
        glEnable( GL_LIGHT0 );
        glMatrixMode( GL_MODELVIEW );
        glLoadIdentity();
        GLfloat position[] = { 0, 0, 1, 0 };
        glLightfv( GL_LIGHT0, GL_POSITION, position );

        glPolygonMode( GL_FRONT, GL_FILL );
        glPolygonMode( GL_BACK, GL_LINE );
        glutMainLoop();
    };
    void vCameraControl(int iSX1, int iSY1, int iSX2, int iSY2, int iSteadyFlag, int iStart)
    {
        int iFlagRot = 0;
        if(iSteadyFlag == 1)
        {
            this->iSteadyReady = 1;
        }
        else if(iSteadyFlag == 2)
        {
            this->iSteadyReady = 0;
        }
        if(iSteadyReady == 1)
        {
            vMotion(iSX1, iSY1, iSX2, iSY2, iFlagRot, iStart);
        }
    }
    int iSteadyReady = 0;
};


BOOST_PYTHON_MODULE(LibShowModel)
{
    // Force the GIL to be created and initialized.  The current caller will
    // own the GIL.
    PyEval_InitThreads();

    using namespace boost::python;
    class_<stShowModel>("stShowModel")
        .def("vReflashModel", &stShowModel::vReflashModel)
        .def("vCameraControl", &stShowModel::vCameraControl)
        ;
}

/*
BOOST_PYTHON_MODULE(LibShowModel)
{
    // Force the GIL to be created and initialized.  The current caller will
    // own the GIL.
    PyEval_InitThreads();

    namespace python = boost::python;
    python::def("f", +[] {
        // For exposition, assert caller owns GIL before and after
        // invoking function `f()`.
        assert(gil_guard::owns_gil());
        f();
        assert(gil_guard::owns_gil());
    });
}
*/


