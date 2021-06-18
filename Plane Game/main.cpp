
#include<windows.h>
#include <GL/glut.h>
#include<bits/stdc++.h>
#include <stdlib.h>
#define rad (3.1416/180)
#define EN_SIZE 20
//#include "BmpLoder.h"

const double PI = 3.14159265389;

int anglex= 0, angley = 0, anglez = 0;
int score= 20;          //rotation angles
int window;
int wired=0;
int shcpt=1;
int animat = 0;
const int L=5;
const int dgre=3;
int ncpt=L+1;
int clikd=0;
const int nt = 40;				//number of slices along x-direction
const int ntheta = 20;

GLfloat ctrlpoints[L+1][3] =
{
     { 0.0, 0.0, 0.0},{0.1,1.5,0},{0.5,2.0,0},{0.9,3.0,0},{1.4,4,0},{1.9,5,0}

    /*{ 0.0, 0.0, 0.0}, { -0.3, 0.5, 0.0},
    { 0.1, 1.7, 0.0},{ 0.5, 1.5, 0.0},
    {1.0, 1.5, 0.0}, {1.4, 1.4, 0.0},
    {1.8, 0.4, 0.0},{2.2, 0.4, 0.0},
    {2.6, 1.5, 0.0}, {3.0, 1.4, 0.0},
    {3.4, 1.4, 0.0},{3.8, 1.4, 0.0},
    {4.2, 1.0, 0.0},{4.6, 1.0, 0.0},
    {5.0, 1.0, 0.0},{5.4, 1.0, 0.0},
    {5.8, 0.5, 0.0},{6.2, 0.5, 0.0},
    {6.6, 0.5, 0.0},{7.2, 0.2, 0.0},
    {6.8, 0.52, 0.0}*/
};

float wcsClkDn[3],wcsClkUp[3];
///////////////////////////////
class point1
{
public:
    point1()
    {
        x=0;
        y=0;
    }
    int x;
    int y;
} clkpt[2];
int flag=0;
GLint viewport[4]; //var to hold the viewport info
GLdouble modelview[16]; //var to hold the modelview info
GLdouble projection[16]; //var to hold the projection matrix info


void scsToWcs(float sx,float sy, float wcsv[3] );
void processMouse(int button, int state, int x, int y);
void matColor(float kdr, float kdg, float kdb,  float shiny, int frnt_Back=0, float ambFactor=1.0, float specFactor=1.0);
///////////////////////////


void scsToWcs(float sx,float sy, float wcsv[3] )
{

    GLfloat winX, winY, winZ; //variables to hold screen x,y,z coordinates
    GLdouble worldX, worldY, worldZ; //variables to hold world x,y,z coordinates

    //glGetDoublev( GL_MODELVIEW_MATRIX, modelview ); //get the modelview info
    glGetDoublev( GL_PROJECTION_MATRIX, projection ); //get the projection matrix info
    glGetIntegerv( GL_VIEWPORT, viewport ); //get the viewport info

    winX = sx;
    winY = (float)viewport[3] - (float)sy;
    winZ = 0;

    //get the world coordinates from the screen coordinates
    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &worldX, &worldY, &worldZ);
    wcsv[0]=worldX;
    wcsv[1]=worldY;
    wcsv[2]=worldZ;


}
void processMouse(int button, int state, int x, int y)
{
    if(button==GLUT_LEFT_BUTTON && state==GLUT_DOWN)
    {
        if(flag!=1)
        {
            flag=1;
            clkpt[0].x=x;
            clkpt[0].y=y;
        }


        scsToWcs(clkpt[0].x,clkpt[0].y,wcsClkDn);
        //cout<<"\nD: "<<x<<" "<<y<<" wcs: "<<wcsClkDn[0]<<" "<<wcsClkDn[1];
    }
    else if(button==GLUT_LEFT_BUTTON && state==GLUT_UP)
    {
        if (flag==1)
        {
            clkpt[1].x=x;
            clkpt[1].y=y;
            flag=0;
        }
        float wcs[3];
        scsToWcs(clkpt[1].x,clkpt[1].y,wcsClkUp);
        //cout<<"\nU: "<<x<<" "<<y<<" wcs: "<<wcsClkUp[0]<<" "<<wcsClkUp[1];

        clikd=!clikd;
    }
}

//control points
long long nCr(int n, int r)
{
    if(r > n / 2) r = n - r; // because C(n, r) == C(n, n - r)
    long long ans = 1;
    int i;

    for(i = 1; i <= r; i++)
    {
        ans *= n - r + i;
        ans /= i;
    }

    return ans;
}

//polynomial interpretation for N points
void BezierCurve ( double t,  float xy[2])
{
    double y=0;
    double x=0;
    t=t>1.0?1.0:t;
    for(int i=0; i<=L; i++)
    {
        int ncr=nCr(L,i);
        double oneMinusTpow=pow(1-t,double(L-i));
        double tPow=pow(t,double(i));
        double coef=oneMinusTpow*tPow*ncr;
        x+=coef*ctrlpoints[i][0];
        y+=coef*ctrlpoints[i][1];

    }
    xy[0] = float(x);
    xy[1] = float(y);

    //return y;
}

///////////////////////
void setNormal(GLfloat x1, GLfloat y1,GLfloat z1, GLfloat x2, GLfloat y2,GLfloat z2, GLfloat x3, GLfloat y3,GLfloat z3)
{
    GLfloat Ux, Uy, Uz, Vx, Vy, Vz, Nx, Ny, Nz;

    Ux = x2-x1;
    Uy = y2-y1;
    Uz = z2-z1;

    Vx = x3-x1;
    Vy = y3-y1;
    Vz = z3-z1;

    Nx = Uy*Vz - Uz*Vy;
    Ny = Uz*Vx - Ux*Vz;
    Nz = Ux*Vy - Uy*Vx;

    glNormal3f(-Nx,-Ny,-Nz);
}
void station_top()
{
    int i, j;
    float x, y, z, r;				//current coordinates
    float x1, y1, z1, r1;			//next coordinates
    float theta;

    const float startx = 0, endx = ctrlpoints[L][0];
    //number of angular slices
    const float dx = (endx - startx) / nt;	//x step size
    const float dtheta = 2*PI / ntheta;		//angular step size

    float t=0;
    float dt=1.0/nt;
    float xy[2];
    BezierCurve( t,  xy);
    x = xy[0];
    r = xy[1];
    //rotate about z-axis
    float p1x,p1y,p1z,p2x,p2y,p2z;
    for ( i = 0; i < 30; ++i )  			//step through x
    {
        theta = 0;
        t+=dt;
        BezierCurve( t,  xy);
        x1 = xy[0];
        r1 = xy[1];

        //draw the surface composed of quadrilaterals by sweeping theta
        glBegin( GL_QUAD_STRIP );
        //glBegin( GL_QUADS );

        for ( j = 0; j <= ntheta; ++j )
        {
            theta += dtheta;
            double cosa = cos( theta );
            double sina = sin ( theta );
            y = r * cosa;
            y1 = r1 * cosa;	//current and next y
            z = r * sina;
            z1 = r1 * sina;	//current and next z

            //edge from point at x to point at next x
            glVertex3f (x, y, z);

            if(j>0)
            {
                setNormal(p1x,p1y,p1z,p2x,p2y,p2z,x, y, z);
            }
            else
            {
                p1x=x;
                p1y=y;
                p1z=z;
                p2x=x1;
                p2y=y1;
                p2z=z1;

            }
            glVertex3f (x1, y1, z1);

            //forms quad with next pair of points with incremented theta value
        }
        glEnd();
        x = x1;
        r = r1;
    } //for i

}
void showControlPoints()
{    glPointSize(5.0);
    glColor3f(1.0, 0.0, 1.0);
    glBegin(GL_POINTS);
    for (int i = 0; i <=L; i++)
        glVertex3fv(&ctrlpoints[i][0]);
    glEnd();
}




/////////////////////////

#include<windows.h>
#include <GL/glut.h>
#include<bits/stdc++.h>
#include <stdlib.h>
#define rad (3.1416/180)
#define EN_SIZE 20
#include "BmpLoader.h"

//#include "RGBpixmap.cpp"

using namespace std;
unsigned int ID;
//RGBpixmap pix[6];

float zoom=4;
int tola[5000][5000];
float tX=0,tY=0,tZ=-8,rX=0,rY=0,rZ=4;
float tZ1=-20,tZ2=-40,tZ3=-60,tZ4=-80,tZ5=-100,tZ6=-120;
float rotX=0,rotY=0,rotZ=0;
float cosX=0,cosY=1,cosZ=0;
float angle=0;
float eye_x=0,eye_y=14.5,eye_z=30,c_x=0,c_y=4,c_z=0,up_x=0,up_y=1.0f,up_z=0.0f;
float xEye=0.0f,yEye=5.0f,zEye=30.0f;
float cenX=0,cenY=0,cenZ=0,roll=0;
float radius=0;
float theta=0,slope=0;
float speed = 0.3;
float angleBackFrac = 0.2;
bool saheedMinarVisible = false;
float spt_cutoff = 20;
bool sp_flag= false;
bool START = false,START1=false;
float r[] = {0.1,0.4,0.0,0.9,0.2,0.5,0.0,0.7,0.5,0.0};
float g[] = {0.2,0.0,0.4,0.5,0.2,0.0,0.3,0.9,0.0,0.2};
float b[] = {0.4,0.5,0.0,0.7,0.9,0.0,0.1,0.2,0.5,0.0};
int TIME=0;
//bool START = false;
float torusPosX[7] = {1,-2,3,-4,-2,0,2};
float torusPosY[7] = {2,3,10,6,7,4,1};

bool rot = false;

GLfloat mat_ambient1[] = { 0.2,0.2,0.0, 1.0 };
GLfloat mat_diffuse1[] = { 0.2,0.2,0.0, 1.0 };
GLfloat mat_specular1[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_shininess1[] = {60};

GLfloat mat_ambient2[] = { 0.0, 0.40, 0.0, 1.0 };
GLfloat mat_diffuse2[] = { 0.0, 0.40, 0.0, 1.0 };
GLfloat mat_specular2[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_shininess2[] = {60};
//white
GLfloat mat_ambient3[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_diffuse3[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_specular3[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_shininess3[] = {60};

///biman body
GLfloat mat_ambient4[] = { 0.4, 0.4, 0.2, 1.0 };
GLfloat mat_diffuse4[] = { 0.4, 0.4, 0.2, 1.0 };
GLfloat mat_specular4[] = { 0.4, 0.4, 0.2, 1.0 };
GLfloat mat_shininess4[] = {60};

///Black
GLfloat mat_ambient5[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat mat_diffuse5[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat mat_specular5[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat mat_shininess5[] = {60};
///Yellow
GLfloat mat_ambient6[] = { 0.8, 1.0, 0.0, 1.0 };
GLfloat mat_diffuse6[] = { 0.8, 1.0, 0.0, 1.0 };
GLfloat mat_specular6[] = { 0.8, 1.0, 0.0, 1.0 };
GLfloat mat_shininess6[] = {60};


GLfloat mat_ambient12[] = { 1.0, 0.6, 0.0, 1.0 };
GLfloat mat_diffuse12[] = { 1.0, 0.6, 0.0, 1.0 };
GLfloat mat_specular12[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_shininess12[] = {60};

GLfloat mat_ambient13[] = { 0.6, 0.8, 1.0, 1.0 };
GLfloat mat_diffuse13[] = { 0.6, 0.8, 1.0, 1.0 };
GLfloat mat_specular13[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_shininess13[] = {60};

GLfloat mat_ambient14[] = { 1.0, 0.6, 0.6, 1.0 };
GLfloat mat_diffuse14[] = { 1.0, 0.6, 0.6, 1.0 };
GLfloat mat_specular14[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat_shininess14[] = {60};


GLfloat mat_ambient15[] = { 0.5, 0.5, 0.5, 0.5 };
GLfloat mat_diffuse15[] = { 0.5, 0.5, 0.5, 0.5 };
GLfloat mat_specular15[] = { 0.5, 0.5, 0.5, 0.5 };
GLfloat mat_shininess15[] = {60};


GLfloat mat_ambient16[] = { 1.0, 0.0, 0.0, 0.0 };
GLfloat mat_diffuse16[] = { 1.0, 0.0, 0.0, 0.0 };
GLfloat mat_specular16[] = { 1.0, 0.0, 0.0, 0.0 };
GLfloat mat_shininess16[] = {60};

GLfloat mat_ambient17[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat mat_diffuse17[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat mat_specular17[] = { 0.0, 1.0, 0.0, 0.0 };
GLfloat mat_shininess17[] = {60};

GLfloat mat_ambient18[] = { 1.0, 0.5, 0.0, 0.0 };
GLfloat mat_diffuse18[] = { 1.0, 0.5, 0.0, 0.0 };
GLfloat mat_specular18[] = { 1.0, 0.5, 0.0, 0.0 };
GLfloat mat_shininess18[] = {60};


GLfloat mat_ambient19[] = { 0.1, 0.1, 0.1, 0.0 };
GLfloat mat_diffuse19[] = { 0.1, 0.1, 0.1, 0.0 };
GLfloat mat_specular19[] = { 0.1, 0.1, 0.1, 0.0 };
GLfloat mat_shininess19[] = {60};


GLfloat mat_ambient20[] = { 0.5, 0.5, 0.5, 0.0 };
GLfloat mat_diffuse20[] = { 0.5, 0.5, 0.5, 0.0 };
GLfloat mat_specular20[] = { 0.5, 0.5, 0.5, 0.0 };
GLfloat mat_shininess20[] = {60};









static void resize(int width, int height)
{
    const float ar = (float) width / (float) height;

    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-ar, ar, -1.0, 1.0, 2.0, 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

static GLfloat v_cube[8][3] =
{
    {0,0,0},
    {0,0,1},
    {0,1,0},
    {0,1,1},

    {1,0,0},
    {1,0,1},
    {1,1,0},
    {1,1,1}
};

static GLubyte c_ind[6][4] =
{
    {0,2,6,4},
    {1,5,7,3},
    {0,4,5,1},
    {2,3,7,6},
    {0,1,3,2},
    {4,6,7,5}
};

static void getNormal3p(GLfloat x1, GLfloat y1, GLfloat z1, GLfloat x2, GLfloat y2, GLfloat z2, GLfloat x3, GLfloat y3, GLfloat z3)
{
    GLfloat Ux, Uy, Uz, Vx, Vy, Vz, Nx, Ny, Nz;

    Ux = x2-x1;
    Uy = y2-y1;
    Uz = z2-z1;

    Vx = x3-x1;
    Vy = y3-y1;
    Vz = z3-z1;

    Nx = Uy*Vz - Uz*Vy;
    Ny = Uz*Vx - Ux*Vz;
    Nz = Ux*Vy - Uy*Vx;

    glNormal3f(Nx,Ny,Nz);
}

//void cube(float R=0.5, float G=0.5, float B=0.5, bool e=false, float alpha=1)
void cube()
{

    glBegin(GL_QUADS);
    for (GLint i = 0; i <6; i++)
    {
        getNormal3p(v_cube[c_ind[i][0]][0], v_cube[c_ind[i][0]][1], v_cube[c_ind[i][0]][2],
                    v_cube[c_ind[i][1]][0], v_cube[c_ind[i][1]][1], v_cube[c_ind[i][1]][2],
                    v_cube[c_ind[i][2]][0], v_cube[c_ind[i][2]][1], v_cube[c_ind[i][2]][2]);

        for (GLint j=0; j<4; j++)
        {
            glVertex3fv(&v_cube[c_ind[i][j]][0]);
        }
    }
    glEnd();
}
void Cube()
{

    glBegin(GL_QUADS);
    for (GLint i = 0; i <6; i++)
    {
        getNormal3p(v_cube[c_ind[i][0]][0], v_cube[c_ind[i][0]][1], v_cube[c_ind[i][0]][2],
                    v_cube[c_ind[i][1]][0], v_cube[c_ind[i][1]][1], v_cube[c_ind[i][1]][2],
                    v_cube[c_ind[i][2]][0], v_cube[c_ind[i][2]][1], v_cube[c_ind[i][2]][2]);

        for (GLint j=0; j<4; j++)
        {
            glVertex3fv(&v_cube[c_ind[i][0]][0]);glTexCoord2f(1,1);
            glVertex3fv(&v_cube[c_ind[i][1]][0]);glTexCoord2f(1,0);
            glVertex3fv(&v_cube[c_ind[i][2]][0]);glTexCoord2f(0,0);
            glVertex3fv(&v_cube[c_ind[i][3]][0]);glTexCoord2f(0,1);
        }
    }
    glEnd();
}


void fan(){

    glColor3d(0.5,1,0);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(1,1,0.7);
        glutSolidSphere(0.8,30,30);
    glPopMatrix();

    glColor3d(0.5,1,0);
    glPushMatrix();
        glTranslated(0,0,0);
        glRotated(5,0,1,0);
        glScaled(0.5,2.5,0.05);
        glutSolidSphere(1,30,30);
    glPopMatrix();

    glColor3d(0.5,1,0);
    glPushMatrix();
        glTranslated(0,0,0);
        glRotated(-5,0,1,0);
        glRotated(90,0,0,1);
        glScaled(0.5,2.5,0.05);
        glutSolidSphere(1,30,30);
    glPopMatrix();

}
void plane3()
{
    const double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double a = t*90.0;

   /// Main body
    //glColor3d(0.5,1,0);
    //glColor3d(0.4,0.4,0.2);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient16);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse16);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular16);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess16);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(3,0.4,0.5);
        glutSolidSphere(1,30,30);
    glPopMatrix();

    //glColor3d(0,0,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient6);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse6);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular6);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess6);
    glPushMatrix();
        glTranslated(1.7,0.1,0);
        glScaled(1.5,0.7,0.8);
        glRotated(40,0,1,0);
        glutSolidSphere(0.45,30,30);
    glPopMatrix();

    ///Samner Pakha

    ///Right
    //glColor3d(0.8,1,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
    glPushMatrix();
        glTranslated(0,0,1.2);
        glRotated(-50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,0.9);
        glRotated(90,0,1,0);

        /// FAN
 //       glPushMatrix();
//           glTranslated(0,0,0.5);
 //           glRotated(10*a,0,0,1);
 //           glScaled(0.1,0.1,0.1);
//            fan();
 //       glPopMatrix();

        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    ///Left
    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0,0,-1.2);
        glRotated(50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(-25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,-1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,-0.9);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();


    ///Pechoner pakha
    glPushMatrix();
        glTranslated(-2.8,0,0);
        glScaled(0.8,0.5,0.3);

        ///Right
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,1.5);
            glRotated(-30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();

        ///left
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,-1.5);
            glRotated(30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(-10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();
    glPopMatrix();

    /// Pesoner Uporer pakha
    //` glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-2.7,0.5,0);
        glRotated(45,0,0,1);
        glScaled(0.8,2,0.1);
        glRotated(-20,0,0,1);
        glutSolidCube(0.5);
    glPopMatrix();

//    glColor3d(0.8,1,0);
//    glPushMatrix();
//        glTranslated(-2.95,0.85,0);
//        glRotated(90,0,1,0);
//        glScaled(0.05,0.05,0.6);
//        glutSolidTorus(0.5,0.5,50,50);
//    glPopMatrix();


    ///FANS

//    glPushMatrix();
//        glTranslated(0,0,0);
//        glRotated(10*a,0,0,1);
//        //glRotated(90,1,0,0);
//        fan();
//    glPopMatrix();
}

void plane2()
{
    const double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double a = t*90.0;

   /// Main body
    //glColor3d(0.5,1,0);
    //glColor3d(0.4,0.4,0.2);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient19);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse19);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular19);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess19);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(3,0.4,0.5);
        glutSolidSphere(1,30,30);
    glPopMatrix();

    //glColor3d(0,0,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient20);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse20);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular20);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess20);
    glPushMatrix();
        glTranslated(1.7,0.1,0);
        glScaled(1.5,0.7,0.8);
        glRotated(40,0,1,0);
        glutSolidSphere(0.45,30,30);
    glPopMatrix();

    ///Samner Pakha

    ///Right
    //glColor3d(0.8,1,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient16);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse16);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular16);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess16);
    glPushMatrix();
        glTranslated(0,0,1.2);
        glRotated(-50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,0.9);
        glRotated(90,0,1,0);

        /// FAN
 //       glPushMatrix();
//           glTranslated(0,0,0.5);
 //           glRotated(10*a,0,0,1);
 //           glScaled(0.1,0.1,0.1);
//            fan();
 //       glPopMatrix();

        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    ///Left
    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0,0,-1.2);
        glRotated(50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(-25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,-1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,-0.9);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();


    ///Pechoner pakha
    glPushMatrix();
        glTranslated(-2.8,0,0);
        glScaled(0.8,0.5,0.3);

        ///Right
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,1.5);
            glRotated(-30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();

        ///left
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,-1.5);
            glRotated(30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(-10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();
    glPopMatrix();

    /// Pesoner Uporer pakha
    //` glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-2.7,0.5,0);
        glRotated(45,0,0,1);
        glScaled(0.8,2,0.1);
        glRotated(-20,0,0,1);
        glutSolidCube(0.5);
    glPopMatrix();

//    glColor3d(0.8,1,0);
//    glPushMatrix();
//        glTranslated(-2.95,0.85,0);
//        glRotated(90,0,1,0);
//        glScaled(0.05,0.05,0.6);
//        glutSolidTorus(0.5,0.5,50,50);
//    glPopMatrix();


    ///FANS

//    glPushMatrix();
//        glTranslated(0,0,0);
//        glRotated(10*a,0,0,1);
//        //glRotated(90,1,0,0);
//        fan();
//    glPopMatrix();
}

void plane1()
{
    const double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double a = t*90.0;

   /// Main body
    //glColor3d(0.5,1,0);
    //glColor3d(0.4,0.4,0.2);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient15);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse15);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular15);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess15);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(3,0.4,0.5);
        glutSolidSphere(1,30,30);
    glPopMatrix();

    //glColor3d(0,0,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient16);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse16);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular16);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess16);
    glPushMatrix();
        glTranslated(1.7,0.1,0);
        glScaled(1.5,0.7,0.8);
        glRotated(40,0,1,0);
        glutSolidSphere(0.45,30,30);
    glPopMatrix();

    ///Samner Pakha

    ///Right
    //glColor3d(0.8,1,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient17);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse17);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular17);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess17);
    glPushMatrix();
        glTranslated(0,0,1.2);
        glRotated(-50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,0.9);
        glRotated(90,0,1,0);

        /// FAN
 //       glPushMatrix();
//           glTranslated(0,0,0.5);
 //           glRotated(10*a,0,0,1);
 //           glScaled(0.1,0.1,0.1);
//            fan();
 //       glPopMatrix();

        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    ///Left
    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0,0,-1.2);
        glRotated(50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(-25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,-1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,-0.9);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();


    ///Pechoner pakha
    glPushMatrix();
        glTranslated(-2.8,0,0);
        glScaled(0.8,0.5,0.3);

        ///Right
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,1.5);
            glRotated(-30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();

        ///left
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,-1.5);
            glRotated(30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(-10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();
    glPopMatrix();

    /// Pesoner Uporer pakha
    //` glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-2.7,0.5,0);
        glRotated(45,0,0,1);
        glScaled(0.8,2,0.1);
        glRotated(-20,0,0,1);
        glutSolidCube(0.5);
    glPopMatrix();

//    glColor3d(0.8,1,0);
//    glPushMatrix();
//        glTranslated(-2.95,0.85,0);
//        glRotated(90,0,1,0);
//        glScaled(0.05,0.05,0.6);
//        glutSolidTorus(0.5,0.5,50,50);
//    glPopMatrix();


    ///FANS

//    glPushMatrix();
//        glTranslated(0,0,0);
//        glRotated(10*a,0,0,1);
//        //glRotated(90,1,0,0);
//        fan();
//    glPopMatrix();
}

void plane(){
    const double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double a = t*90.0;

   /// Main body
    //glColor3d(0.5,1,0);
    //glColor3d(0.4,0.4,0.2);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient4);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse4);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular4);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess4);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(3,0.4,0.5);
        glutSolidSphere(1,30,30);
    glPopMatrix();

    //glColor3d(0,0,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient5);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse5);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular5);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess5);
    glPushMatrix();
        glTranslated(1.7,0.1,0);
        glScaled(1.5,0.7,0.8);
        glRotated(40,0,1,0);
        glutSolidSphere(0.45,30,30);
    glPopMatrix();

    ///Samner Pakha

    ///Right
    //glColor3d(0.8,1,0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient6);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse6);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular6);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess6);
    glPushMatrix();
        glTranslated(0,0,1.2);
        glRotated(-50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,0.9);
        glRotated(90,0,1,0);

        /// FAN
 //       glPushMatrix();
//           glTranslated(0,0,0.5);
 //           glRotated(10*a,0,0,1);
 //           glScaled(0.1,0.1,0.1);
//            fan();
 //       glPopMatrix();

        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    ///Left
    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0,0,-1.2);
        glRotated(50,0,1,0);
        glScaled(0.7,0.1,3);
        glRotated(-25,0,1,0);
        glutSolidCube(1);
    glPopMatrix();

   // glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-0.3,-0.15,-1.5);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();

    //glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(0.2,-0.15,-0.9);
        glRotated(90,0,1,0);
        glScaled(0.1,0.1,0.9);
        glutSolidTorus(0.5,0.5,50,50);
    glPopMatrix();


    ///Pechoner pakha
    glPushMatrix();
        glTranslated(-2.8,0,0);
        glScaled(0.8,0.5,0.3);

        ///Right
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,1.5);
            glRotated(-30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();

        ///left
        //glColor3d(0.8,1,0);
        glPushMatrix();
            glTranslated(0.4,0,-1.5);
            glRotated(30,0,1,0);
            glScaled(0.7,0.1,3);
            glRotated(-10,0,1,0);
            glutSolidCube(1);
        glPopMatrix();
    glPopMatrix();

    /// Pesoner Uporer pakha
    //` glColor3d(0.8,1,0);
    glPushMatrix();
        glTranslated(-2.7,0.5,0);
        glRotated(45,0,0,1);
        glScaled(0.8,2,0.1);
        glRotated(-20,0,0,1);
        glutSolidCube(0.5);
    glPopMatrix();

//    glColor3d(0.8,1,0);
//    glPushMatrix();
//        glTranslated(-2.95,0.85,0);
//        glRotated(90,0,1,0);
//        glScaled(0.05,0.05,0.6);
//        glutSolidTorus(0.5,0.5,50,50);
//    glPopMatrix();


    ///FANS

//    glPushMatrix();
//        glTranslated(0,0,0);
//        glRotated(10*a,0,0,1);
//        //glRotated(90,1,0,0);
//        fan();
//    glPopMatrix();
}

void  single_building2(){
      //drawShohidMinar();
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,3);

    //glColor3d(1,1.0,1.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
	glPushMatrix();
        glTranslated(0,1.55,0);
        glScaled(1,2,0.8);
        //glutSolidCube(1);
        Cube();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

}

void  single_building1(){
      //drawShohidMinar();
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,2);

    //glColor3d(1,1.0,1.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
	glPushMatrix();
        glTranslated(0,1.55,0);
        glScaled(1,2,0.8);
        //glutSolidCube(1);
        Cube();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

}


void  single_building(){
      //drawShohidMinar();
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D,1);

    //glColor3d(1,1.0,1.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
	glPushMatrix();
        glTranslated(0,1.55,0);
        glScaled(1,2,0.8);
        //glutSolidCube(1);
        Cube();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);

}
void building(){
    ///1
    glPushMatrix();
        single_building1();
    glPopMatrix();

    ///2
    glPushMatrix();
        glTranslated(-2,0,0);
        single_building1();
    glPopMatrix();

    ///3
    glPushMatrix();
        glTranslated(-4,0,0);
        single_building2();
    glPopMatrix();

    ///4
    glPushMatrix();
        glTranslated(-6,0,0);
        single_building2();
    glPopMatrix();

}

void house_building2()
{
    glPushMatrix();
        glPushMatrix();
            glTranslated(0,1.55,0);
            /// base building
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient14);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse14);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular14);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess14);
                cube();
            glPopMatrix();
        /// window front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// window back
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(1.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// door front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(0.3,0.15,1.01);
                glScaled(0.4,0.7,0.01);
                cube();
            glPopMatrix();

            /// top floor of building;
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.1,1,-0.1);
                glScaled(1.2,0.02,1.2);
                cube();
            glPopMatrix();

        glPopMatrix();

        ///2nd floor
        glPushMatrix();
            glTranslated(0,2.57,0);
            /// base building
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient14);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse14);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular14);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess14);
                cube();
            glPopMatrix();
        /// window front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// window back
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(1.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// door front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(0.3,0.15,1.01);
                glScaled(0.4,0.7,0.01);
                cube();
            glPopMatrix();

            /// top floor of building;
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.1,1,-0.1);
                glScaled(1.2,0.02,1.2);
                cube();
            glPopMatrix();

        glPopMatrix();
    glPopMatrix();
}

void house_building1()
{
    glPushMatrix();
        glPushMatrix();
            glTranslated(0,1.55,0);
            /// base building
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient13);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse13);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular13);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess13);
                cube();
            glPopMatrix();
        /// window front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// window back
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(1.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// door front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(0.3,0.15,1.01);
                glScaled(0.4,0.7,0.01);
                cube();
            glPopMatrix();

            /// top floor of building;
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.1,1,-0.1);
                glScaled(1.2,0.02,1.2);
                cube();
            glPopMatrix();

        glPopMatrix();

        ///2nd floor
        glPushMatrix();
            glTranslated(0,2.57,0);
            /// base building
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient13);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse13);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular13);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess13);
                cube();
            glPopMatrix();
        /// window front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// window back
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(1.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// door front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(0.3,0.15,1.01);
                glScaled(0.4,0.7,0.01);
                cube();
            glPopMatrix();

            /// top floor of building;
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.1,1,-0.1);
                glScaled(1.2,0.02,1.2);
                cube();
            glPopMatrix();

        glPopMatrix();

        ///3rd floor
        glPushMatrix();
            glTranslated(0,3.59,0);
            /// base building
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient13);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse13);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular13);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess13);
                cube();
            glPopMatrix();
        /// window front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// window back
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(1.01,0.3,0.3);
                glScaled(0.01,0.4,0.4);
                cube();
            glPopMatrix();

            /// door front
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(0.3,0.15,1.01);
                glScaled(0.4,0.7,0.01);
                cube();
            glPopMatrix();

            /// top floor of building;
            glPushMatrix();
                //house_building1();
                glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient3);
                glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse3);
                glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular3);
                glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess3);
                glTranslated(-0.1,1,-0.1);
                glScaled(1.2,0.02,1.2);
                cube();
            glPopMatrix();

        glPopMatrix();

    glPopMatrix();




}
void house_building()
{    ///2
    glPushMatrix();
        glTranslated(0,0,-0.25);
        house_building2();
    glPopMatrix();
    ///2
    glPushMatrix();
        glTranslated(-2,0,-0.25);
        house_building2();
    glPopMatrix();
    ///2
    glPushMatrix();
        glTranslated(-4,0,-0.25);
        house_building2();
    glPopMatrix();

    ///2
     glPushMatrix();
        glTranslated(-6,0,-0.25);
        house_building2();
    glPopMatrix();

    ///1
    glPushMatrix();
        glTranslated(-1.5,0,-1.75);
        house_building1();
    glPopMatrix();
    ///1
    glPushMatrix();
        glTranslated(-3.5,0,-1.75);
        house_building1();
    glPopMatrix();
    ///1
    glPushMatrix();
        glTranslated(-5,0,-1.75);
        house_building1();
    glPopMatrix();
}
void curveair()
{
    if(wired)
    {
        glPolygonMode( GL_FRONT, GL_LINE ) ;
        glPolygonMode( GL_BACK, GL_LINE ) ;

    }
    else
    {
        glPolygonMode( GL_FRONT,GL_FILL ) ;
        glPolygonMode( GL_BACK, GL_FILL ) ;
    }
    glPushMatrix();
            glTranslated(-2,0,0);
            glPushMatrix();
                glTranslated(0,1.55,0);
                glScaled(0.05,1.45,0.05);
                Cube();
            glPopMatrix();
            glPushMatrix();
                   /* glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient11);
                    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse11);
                    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular11);
                    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess11);*/
                glTranslated(0,3.88,0);
                glScaled(0.75,0.750,0.5);
                glRotated(-90,0,0,1);
                station_top();
            glPopMatrix();
        glPopMatrix();

        glPushMatrix();
            ///1
            glPushMatrix();
                glTranslated(-2.5,2.1 ,-2);
                //glRotated(-90,0,1,0);
                //glScaled(0.3,0.3,0.3);
                glScaled(0.3,0.3,0.3);
                //car();
                plane1();
            glPopMatrix();


            ///2
            glPushMatrix();
                glTranslated(-0.5,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                plane2();
            glPopMatrix();

            ///3
            glPushMatrix();
                glTranslated(-5,2.1 ,0.75);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                plane3();
            glPopMatrix();
            ///4
            glPushMatrix();
                glTranslated(-2,2.1 ,-1);
                //glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                plane();
            glPopMatrix();
            ///5
            glPushMatrix();
                glTranslated(-3,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();
            ///6
            glPushMatrix();
                glTranslated(-4,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();

            ///7
            glPushMatrix();
                glTranslated(-5,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();

        glPopMatrix();



        ///
        glPushMatrix();
            glTranslated(0,0,-2);
            ///1
            glPushMatrix();
                glTranslated(1,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();

            ///2
            glPushMatrix();
                glTranslated(0,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();

            ///3
            glPushMatrix();
                glTranslated(-1,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();
            ///4
            glPushMatrix();
                glTranslated(-2,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();
            ///5
            glPushMatrix();
                glTranslated(-3,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();
            ///6
            glPushMatrix();
                glTranslated(-4,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.3,0.3,0.3);
                //car();
                //plane();
            glPopMatrix();

            ///7
            glPushMatrix();
                glTranslated(-5,2.1 ,0);
                glRotated(-90,0,1,0);
                glScaled(0.2,0.2,0.2);
                //car();
                //plane();
            glPopMatrix();

        glPopMatrix();

}
void air_station2()
{
        /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();

        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient6);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse6);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular6);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess6);
    glPushMatrix();
        glTranslated(0,3.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();
/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        //house_building();
        curveair();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(7,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        //house_building();
          curveair();
        //drawShohidMinar();
    glPopMatrix();

}

void air_station()
{
        /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();


        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient6);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse6);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular6);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess6);

    glPushMatrix();
        glTranslated(0,3.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();


/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        //house_building();
        curveair();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(8,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        //house_building();
          curveair();
        //drawShohidMinar();
    glPopMatrix();

}

void soheedMinarEnv2(){
    /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();

        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient6);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse6);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular6);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess6);
    glPushMatrix();
        glTranslated(1.5,4.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();

/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        house_building();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(8,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        house_building();
        //drawShohidMinar();
    glPopMatrix();
}


void soheedMinarEnv(){
    /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();

        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient6);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse6);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular6);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess6);
    glPushMatrix();
        glTranslated(-1.5,4.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();

/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        house_building();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(8,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        house_building();
        //drawShohidMinar();
    glPopMatrix();
}

void tree(){
	/// ground
	glPushMatrix();
	//glTranslated(-2,0,3.5);
    ///base body;
	glPushMatrix();
	    //glColor3d(0,0.40,0.0);
        //glTranslated(-2,0,3.5);
        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient2);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse2);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular2);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess2);
        glTranslated(0,1.55,0);
        glScaled(4,0.05,3);
        glRotated(-90,1,0,0);
        //glutSolidCube(1);
        //Cube();
        GLUquadricObj *quadratic;
        quadratic = gluNewQuadric();
        gluCylinder(quadratic, 0.0125, 0.0125, 20, 20, 8);
    glPopMatrix();

    ///top leaf stand cylinder;
    glPushMatrix();// start leaf with stand;
    glTranslated(0,2.55,0);
    //stand
        glPushMatrix();
            //glColor3d(0,0.40,0.0);
            //glTranslated(-1.5,2.55,-2.5);
            //glTranslated(0,2.55,0);
            glScaled(4,0.05,3);
            glRotated(-90,1,0,0);
            //glutSolidCube(1);
            //Cube();
            //GLUquadricObj *quadratic;
            quadratic = gluNewQuadric();
            gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
        glPopMatrix();

        ///top leaf sphare
        glPushMatrix();
            //glColor3d(0,0.40,0.0);
            //glTranslated(-1.5,2.55,-2.5);
            glTranslated(0,.30,0);
            glScaled(0.18,0.18,0.18);
            glutSolidSphere(1,30,30);
        glPopMatrix();

    glPopMatrix();// finished leaf with stand;

    glPushMatrix();// down layer of leaf start

        ///road side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(45,1,0,0);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

       ///road back side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(-45,1,0,0);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

        ///road 1st side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(45,0,0,1);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

        ///road 2st side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(-45,0,0,1);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

    glPopMatrix(); //down layer finished

    ///

    glPushMatrix();// up layer of leaf start
        glTranslated(0,0.35,0);

        ///road side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(45,1,0,0);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

        ///road back side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(-45,1,0,0);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

        ///road 1st side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(45,0,0,1);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;

        ///road 2st side leaf stand cylinder;
        glPushMatrix();// start leaf with stand;
             glTranslated(0,2.0,0);
             glRotated(-45,0,0,1);
            /// stand
            glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                //glTranslated(0,2.55,0);
                glScaled(4,0.05,3);
                glRotated(-90,1,0,0);
                //glutSolidCube(1);
                //Cube();
                //GLUquadricObj *quadratic;
                quadratic = gluNewQuadric();
                gluCylinder(quadratic, 0.005, 0.005,3 , 20, 8);
            glPopMatrix();
            ///top leaf sphare
                glPushMatrix();
                //glColor3d(0,0.40,0.0);
                //glTranslated(-1.5,2.55,-2.5);
                glTranslated(0,.30,0);
                glScaled(0.18,0.18,0.18);
                glutSolidSphere(1,30,30);
            glPopMatrix();

        glPopMatrix();// finished leaf with stand;




    glPopMatrix(); //up layer finished


    glPopMatrix();

}
 void tree_building(){
     ///building 1
     glPushMatrix();
        single_building();
        //tree();
     glPopMatrix();
     /// tree 1
     glPushMatrix();
        //single_building();
        glTranslated(-1,0,1);
        tree();
     glPopMatrix();
 /// building 2
    glPushMatrix();
        glTranslated(-2.5,0,0);
        single_building();
        //tree();
     glPopMatrix();
///tree 2
        glPushMatrix();
        //single_building();
        glTranslated(-3,0,1);
        tree();
     glPopMatrix();

      /// building 3
    glPushMatrix();
        glTranslated(-5,0,0);
        single_building();
        //tree();
     glPopMatrix();
/// tree 3
        glPushMatrix();
        //single_building();
        glTranslated(-5,0,1);
        tree();
     glPopMatrix();


 }
void tree_with_building(){
    /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();

        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient16);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse16);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular16);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess16);
    glPushMatrix();
        glTranslated(-1.5,2.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();
/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        tree_building();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(8,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        //drawShohidMinar();
        tree_building();
    glPopMatrix();
}

void only_tree1()
{
    ///tree1
   glPushMatrix();
        tree();
   glPopMatrix();
   ///tree2
   glPushMatrix();
    glTranslated(-1,0,0);
        tree();
   glPopMatrix();
   ///tree3
   glPushMatrix();
    glTranslated(-2,0,0);
        tree();
   glPopMatrix();
   ///tree4
   glPushMatrix();
    glTranslated(-3,0,0);
        tree();
   glPopMatrix();

    ///tree5
   glPushMatrix();
    glTranslated(-4,0,0);
        tree();
   glPopMatrix();

    ///tree6
   glPushMatrix();
    glTranslated(-5,0,0);
        tree();
   glPopMatrix();

     ///tree7
   glPushMatrix();
    glTranslated(-6,0,0);
        tree();
   glPopMatrix();



   ///house_building1
    glPushMatrix();
    glTranslated(-0.1,0,-1.75);
       house_building2();
    glPopMatrix();

    ///house_building2
    glPushMatrix();
    glTranslated(-2.1,0,-1.75);
       house_building2();
    glPopMatrix();

    ///house_building3
    glPushMatrix();
    glTranslated(-4.1,0,-1.75);
       house_building2();
    glPopMatrix();

     ///house_building4
    glPushMatrix();
    glTranslated(-6.1,0,-1.75);
       house_building2();
    glPopMatrix();

    ///house_building5
    glPushMatrix();
    glTranslated(-1.51,0,-3.75);
       house_building1();
    glPopMatrix();

    ///house_building5
    glPushMatrix();
    glTranslated(-3.71,0,-3.75);
       house_building1();
    glPopMatrix();

     ///house_building5
    glPushMatrix();
    glTranslated(-5.51,0,-3.75);
       house_building1();
    glPopMatrix();


}
void only_tree5()
{
       /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();

        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient16);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse16);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular16);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess16);
    glPushMatrix();
        glTranslated(-1.5,4.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();

/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        //tree_building();
        only_tree1();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(8,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        //drawShohidMinar();
        only_tree1();
    glPopMatrix();
}

void only_tree()
{
       /// Ground
    //glColor3d(0,0.5,0.1);
    //glColor3d(0.2,0.2,0.0);
    glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient1);
    glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse1);
    glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular1);
    glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess1);
    glPushMatrix();
        glTranslated(0,0,0);
        glScaled(EN_SIZE*2,0.3,EN_SIZE*2);
        glutSolidCube(1);
    glPopMatrix();

        glMaterialfv( GL_FRONT, GL_AMBIENT, mat_ambient16);
        glMaterialfv( GL_FRONT, GL_DIFFUSE, mat_diffuse16);
        glMaterialfv( GL_FRONT, GL_SPECULAR, mat_specular16);
        glMaterialfv( GL_FRONT, GL_SHININESS, mat_shininess16);
    glPushMatrix();
        glTranslated(1.5,4.55,0);
        glScaled(0.5,0.5,0.5);
        glutSolidTorus(0.2,1.5,20,20);
    glPopMatrix();

/// left side building
    glPushMatrix();
        glTranslated(-8,-2.7,-5);
        glRotated(90,0,1,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //drawShohidMinar();
        //building();
        //tree_building();
        only_tree1();
    glPopMatrix();
/// right side building;
    glPushMatrix();
        glTranslated(8,-2.7,-5);
        glRotated(-90,0,1,0);
        glTranslated(6,0,0);
        //glRotated(15,0,1,0);
        glScaled(2,2,2);
        //building();
        //drawShohidMinar();
        only_tree1();
    glPopMatrix();
}

void draw(){
    double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double a = t*90.0;

    TIME = t;

    ///Plane
    if(rotX>11)rotX=11;
    if(rotX<-11)rotX=-11;
    if(rotZ>10)rotZ=10;
    if(rotZ<-15)rotZ=-15;

    glPushMatrix();
        glTranslated(0,1,0);
        glRotated(90,0,1,0);
        glRotated(5,0,0,1);
        glRotated(rotX,1,0,0);
        glRotated(rotY,0,1,0);
        glRotated(rotZ,0,0,1);

        glScaled(0.4,0.4,0.4);
        plane();
    glPopMatrix();

    ///Environment
    //if(tX>=4.1)tX=4.1;
    //if(tX<=-4.1)tX=-4.1;
    if(tX>=5){START=false;START1= true;tX=0;}//tX=4.1;
    if(tX<=-4.1) {START=false;START1= true;tX=0;}
    if(tY>0.1)tY= 0.1;
    if(tY<-15)tY= -15;
///1
    glPushMatrix();
        glTranslated(tX,tY,tZ);
        //environment(2);
        //tree_with_building();
        //soheedMinarEnv();
        //only_tree();
        air_station();
    glPopMatrix();
///2
    glPushMatrix();
        glTranslated(tX,tY,tZ1);
        //soheedMinarEnv();

        tree_with_building();
         //air_station();
    glPopMatrix();
///3
    glPushMatrix();
        glTranslated(tX,tY,tZ2);
        //environment(3);
        only_tree();
         //tree_with_building();
         //soheedMinarEnv();
    glPopMatrix();
///4
    glPushMatrix();
        glTranslated(tX,tY,tZ3);
        //environment(1);
         //only_tree();
         //air_station2();
         soheedMinarEnv();
    glPopMatrix();
///5
    glPushMatrix();
        glTranslated(tX,tY,tZ4);
        //tree_with_building();
        //env_ironment(5);
         //soheedMinarEnv();
         air_station2();
    glPopMatrix();
///6
    glPushMatrix();
        glTranslated(tX,tY,tZ5);
        //environment(4);
         //soheedMinarEnv();
         //only_tree();
         only_tree5();
    glPopMatrix();
///7
    glPushMatrix();
        glTranslated(tX,tY,tZ6);
        //environment(2);
         //soheedMinarEnv();
         soheedMinarEnv2();
         //only_tree();
         //air_station();
    glPopMatrix();

    ///
    if(tZ<0.5&& tZ>-1)
    {
        if(tX<2&& tX>-2)
        {
            if(tY<-2. && tY>-3.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }

        if(tZ1<0.5&& tZ1>-1)
    {
        if(tX<1.7&& tX>0.6)
        {
            if(tY<-1. && tY>-2.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }

    if(tZ2<0.5&& tZ2>-1)
    {
        if(tX<-0.6&& tX>-1.7)
        {
            if(tY<-3. && tY>-4.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }

        if(tZ3<0.5&& tZ3>-1)
    {
        if(tX<1.7&& tX>0.6)
        {
            if(tY<-3. && tY>-4.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }

        if(tZ4<0.5&& tZ4>-1)
    {
        if(tX<2&& tX>-2)
        {
            if(tY<-2. && tY>-3.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }


        if(tZ5<0.5&& tZ5>-1)
    {
        if(tX<1.7&& tX>0.6)
        {
            if(tY<-1. && tY>-2.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }

    if(tZ6<0.5&& tZ6>-1)
    {
        if(tX<-0.6&& tX>-1.7)
        {
            if(tY<-3. && tY>-4.1 )//&& tY> 1.5)
            {
              score=score+2;
            }

        }
    }




    ///

    tZ+=speed;
    tZ1+=speed;
    tZ2+=speed;
    tZ3+=speed;
    tZ4+=speed;
    tZ5+=speed;
    tZ6+=speed;

    if(tZ>=20)tZ=-110;
    if(tZ1>=20)tZ1=-110;
    if(tZ2>=20)tZ2=-110;
    if(tZ3>=20)tZ3=-110;
    if(tZ4>=20)tZ4=-110;
    if(tZ5>=20)tZ5=-110;
    if(tZ6>=20)tZ6=-110;

    if(rotX>0)rotX-=angleBackFrac;
    if(rotX<0)rotX+=angleBackFrac;
    if(rotY>0)rotY-=angleBackFrac;
    if(rotY<0)rotY+=angleBackFrac;
    if(rotZ>0)rotZ-=angleBackFrac;
    if(rotZ<0)rotZ+=angleBackFrac;

    cout<<tX<<" "<<tY<<" "<<tZ<<endl;
    cout<<rotX<<" "<<rotY<<" "<<rotZ<<endl;

    speed += 0.004;
    if(speed>=1.7)speed=1.7;
}


void drawBitmapText(char *str,float x,float y,float z)
{
	char *c;
	glRasterPos3f(x,y+8,z);

	for (c=str; *c != '\0'; c++)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, *c);
	}
}

void drawStrokeText(char* str,int x,int y,int z)
{
	  char *c;
	  glPushMatrix();
	  glTranslatef(x, y+8,z);
	  glScalef(0.002f,0.002f,z);

	  for (c=str; *c != '\0'; c++)
	  {
    		glutStrokeCharacter(GLUT_STROKE_ROMAN , *c);
	  }
	  glPopMatrix();
}

void drawStrokeText2(char* str,int x,int y,int z)
{
	  char *c;
	  glPushMatrix();
	  glTranslatef(x, y+8,z);
	  glScalef(0.005f,0.005f,z);

	  for (c=str; *c != '\0'; c++)
	  {
    		glutStrokeCharacter(GLUT_STROKE_ROMAN , *c);
	  }
	  glPopMatrix();
}
void drawStrokeChar(char c,float x,float y,float z)
{
	  glPushMatrix();
          glTranslatef(x, y+8,z);
          glScalef(0.002f,0.002f,z);
          glutStrokeCharacter(GLUT_STROKE_ROMAN , c);
	  glPopMatrix();
}
///light 02
GLfloat light_ambients[] = {0.3, 0.3, 0.3, 1};
GLfloat light_diffuses[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_speculars[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_positions[] = { 0,0.5,4.5 ,1.0 };


static void display(void)
{
    const double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
    double a = t*90.0;
    double aa=a;

    if(!rot){
        a=0;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    //gluLookAt(eye_x,eye_y,eye_z,c_x,c_y,c_z,up_x,up_y,up_z);

    /*gluLookAt(	0.0, 14.5, 30.0,
                0, 4, 0,
                0, 1.0f, 0.0f);*/
    gluLookAt(eye_x,eye_y,eye_z,c_x,c_y,c_z,up_x,up_y,up_z);

    if(START==true){
    //if(1){

        glPushMatrix();
            glTranslated(0,0,0);
            glScaled(zoom,zoom,zoom);
            glRotated(a,0,1,0);
            draw();
        glPopMatrix();

        //drawStrokeText("UP: W, DOWN: S, LEFT: A, RIGHT: D, MAIN MENU: M",-8,0.9,0);
        drawStrokeText("POINTS : ",3,9,2);

        int pp=score, mod,number=0;

        float tmp=0;
        //drawStrokeChar(10,4+tmp,9,2);
        //while(score){
        for( int i=0;i<3;i++){
            //mod=Time%10;
            mod=score%10;
            drawStrokeChar(mod+48,4.6-tmp,9,2);
            //drawStrokeChar(score+48,4+tmp,9,2);
            score/=10;

            tmp+=0.2;
        }
        score=pp;
    }
    else{

        glPushMatrix();
            glTranslated(0,3,0);
            glRotated(aa,0,1,0);
            glScaled(1.5,1.5,1.5);
            plane();
        glPopMatrix();

        drawStrokeText("Press G to Start",-1,-1,0);
        drawStrokeText2("Plane Game",-2,0,0);
        drawStrokeText2("Left: A, Right: D, Up: W, Down: S",-6,2,0);
    }



    glLightfv(GL_LIGHT1, GL_AMBIENT,  light_ambients);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  light_diffuses);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_speculars);
    glLightfv(GL_LIGHT1, GL_POSITION, light_positions);
    //glLightfv(GL_LIGHT1, GL_POSITION, light_positions);
    if(sp_flag==true)
    {
        glEnable(GL_LIGHT1);
        GLfloat l_spt[] = {0.0,0,-1,0.0};
        GLfloat spt_ct[] = {spt_cutoff};
        glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, l_spt);
        glLightfv(GL_LIGHT1, GL_SPOT_CUTOFF, spt_ct);
    }


    glutSwapBuffers();
}
const GLfloat no_light[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_ambient[]  = { 0.30f, 0.30f, 0.30f, 1.0f };
const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
//const GLfloat light_diffuse[]  = { 0.30f, 0.30f, 0.30f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 2.0f, 5.0f, 5.0f, 0.0f };

static void idle(void)
{
    glutPostRedisplay();
}

void LoadTexture(const char*filename)
{
    glGenTextures(1, &ID);
    glBindTexture(GL_TEXTURE_2D, ID);
    glPixelStorei(GL_UNPACK_ALIGNMENT, ID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    BmpLoader bl(filename);
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, bl.iWidth, bl.iHeight, GL_RGB, GL_UNSIGNED_BYTE, bl.textureData );
}

static void key(unsigned char key, int x, int y)
{
    float frac = 0.3;
    float rotFrac = 1;
    switch (key)
    {
        case 27 :
        case 'q':
            exit(0);
            break;
        case 'r':
            rot=true;
            break;
        case 't':
            rot=false;
            break;
        case 'z':
            zoom+=0.05;
            break;
        case 'Z':
            zoom-=0.05;
        case 'w':
            //tY+=0.1;
            tY-=frac;
            rotZ+=rotFrac;
            break;
        case 's':
            //tY+=0.1;
            tY+=frac;
            rotZ-=rotFrac;
            break;
        case 'a':
            tX+=frac;
            rotX-=rotFrac*3;
            rotY+=rotFrac/2;
            break;
        case 'd':
            tX-=frac;
            rotX+=rotFrac*3;
            rotY-=rotFrac/2;
            break;
//        case 'y':
//            rotX-=rotFrac;
//            break;
//        case 'h':
//            rotX+=rotFrac;
//            break;
//        case 'g':
//            rotY+=rotFrac;
//            break;
//        case 'j':
//            rotY-=rotFrac;
//            break;
        case 'g':
            START=true;
            break;
        case 'm':
            START=false;
            break;

        case 'v':
            glDisable(GL_LIGHT0);
            break;
        case 'V':
            glEnable(GL_LIGHT0);

        case 'b':
            glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
            glLightfv(GL_LIGHT0, GL_DIFFUSE,  no_light);
            glLightfv(GL_LIGHT0, GL_SPECULAR, no_light);
            break;

        case 'n':
            glLightfv(GL_LIGHT0, GL_AMBIENT,  no_light);
            glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
            glLightfv(GL_LIGHT0, GL_SPECULAR, no_light);
            break;
        case 'M':
            glLightfv(GL_LIGHT0, GL_AMBIENT,  no_light);
            glLightfv(GL_LIGHT0, GL_DIFFUSE,  no_light);
            glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);;
            break;
//        case 'o':
//            cosX-=frac*cos(rotX*rad);
//            cosY+=frac*cos(rotY*rad);
//            cosZ-=frac*cos(rotZ*rad);
//            //cout<<"Front : "<<cosX<<" "<<cosY<<" "<<cosZ<<endl;
//            break;
//        case 'l':
//            cosX+=frac*cos(rotX*rad);
//            cosY-=frac*cos(rotY*rad);
//            cosZ+=frac*cos(rotZ*rad);
//            //cout<<"Back : "<<cosX<<" "<<cosY<<" "<<cosZ<<endl;
//            break;
            case 'l':
            sp_flag=true;
            break;

        case 'L':
            glDisable(GL_LIGHT1);
            sp_flag=false;
            break;



        case '3':
            speed =0;
            break;

        case 'h':
        case 'H':
            wired=!wired;
            break;

        case '7':
            eye_x+=0.1;
            break;
       case '4':
            eye_x-=0.1;
            break;

        case '6':
            eye_y-=0.1;
            break;
        case '9':
            eye_y+=0.1;
            break;
        case '8':
            eye_z-=.1;
            break;
        case '5':
            eye_z+=.1;
            break;



    }

    glutPostRedisplay();
}


//const GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 1.0f };
//const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0f };
//const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
//const GLfloat high_shininess[] = { 100.0f };

/* Program entry point */



int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitWindowPosition(0,0);
	glutInitWindowSize(1366,720);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA);

    glutCreateWindow("GLUT Shapes");
    LoadTexture("C:\\Users\\HP\\Desktop\\Grafix Lab\\firoz_texture\\f1.bmp");
    LoadTexture("C:\\Users\\HP\Desktop\\Grafix Lab\\firoz_texture\\f1.bmp");
    LoadTexture("C:\\Users\\HP\\Desktop\\Grafix Lab\\firoz_texture\\f3.bmp");
    //LoadTexture("C:\\Users\\HP\\Desktop\\Grafix Lab\\Texaring Building\\firoz1.bmp");


        glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    //glutMouseFunc(processMouse);
    glutIdleFunc(idle);

    //PlaySound("starwars.wav", NULL, SND_ASYNC|SND_FILENAME|SND_LOOP);

    //glClearColor(1,1,1,1);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    //glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    //glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    //glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    //glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    //glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);

    glutMainLoop();

    return EXIT_SUCCESS;
}

