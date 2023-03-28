
#include "AreaLight.hpp"


namespace FW {


void AreaLight::draw(const Mat4f& worldToCamera, const Mat4f& projection) {
    glUseProgram(0);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf((float*)&projection);
    glMatrixMode(GL_MODELVIEW);
    Mat4f S = Mat4f::scale(Vec3f(m_size,1));
    Mat4f M = worldToCamera *m_xform * S;
    glLoadMatrixf((float*)&M);
    glBegin(GL_TRIANGLES);
    glColor3fv( &m_E.x );
    glVertex3f(1,1,0); glVertex3f(1,-1,0); glVertex3f( -1,-1,0 );
    glVertex3f(1,1,0); glVertex3f( -1,-1,0 ); glVertex3f(-1,1,0); 
    glEnd();
}
float vanDerCorput(int n, const int& base = 2)
{
	float rand = 0, denom = 1, invBase = 1.f / base;
	while (n) {
		denom *= base;  //2, 4, 8, 16, etc, 2^1, 2^2, 2^3, 2^4 etc. 
		rand += (n % base) / denom;
		n *= invBase;  //divide by 2 
	}
	return rand;
}
void AreaLight::sample(float& pdf, Vec3f& p, int i, Random& rnd) {
    // YOUR CODE HERE (R2):
    // You should draw a random point on the light source and evaluate the PDF.
    // Store the results in "pdf" and "p".
    // 
    // Note: The "size" member is _one half_ the diagonal of the light source.
    // That is, when you map the square [-1,1]^2 through the scaling matrix
    // 
    // S = ( size.x    0   )
    //     (   0    size.y )
    // 
    // the result is the desired light source quad (see draw() function above).
    // This means the total area of the light source is 4*size.x*size.y.
    // This has implications for the computation of the PDF.

    // For extra credit, implement QMC sampling using some suitable sequence.
    // Use the "base" input for controlling the progression of the sequence from
    // the outside. If you only implement purely random sampling, "base" is not required.

    // (this does not do what it's supposed to!)
    //pdf = 1.0f;
    //p = Vec4f(m_xform.getCol(3)).getXYZ();
    
	//Quasi-monto carlo: Halton
	Vec2f samplePoint(vanDerCorput(i, 2), vanDerCorput(i, 3));
	Mat4f S = Mat4f::scale(Vec3f(m_size, 1.0f));
	p = (m_xform * S * Vec4f(samplePoint.x, samplePoint.y, 0.0f, 1.0f)).getXYZ();
	pdf = 1.0 / 4.0f / m_size.x / m_size.y;

	//Purely Random
	//Mat4f S = Mat4f::scale(Vec3f(m_size, 1.0f));
	//p = (m_xform * S * Vec4f(rnd.getF32(-1.0f, 1.0f), rnd.getF32(-1.0f, 1.0f),0.0f,1.0f)).getXYZ();
	//pdf = 1.0 / 4.0f / m_size.x / m_size.y;
}


} // namespace FW
