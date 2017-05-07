/* Triangle/triangle intersection test routine,
 * by Tomas Moller, 1997.
 * See article "A Fast Triangle-Triangle Intersection Test",
 * Journal of Graphics Tools, 2(2), 1997
 *
 * int tri_tri_intersect(double V0[3],double V1[3],double V2[3],
 *                         double U0[3],double U1[3],double U2[3])
 *
 * parameters: vertices of triangle 1: V0,V1,V2
 *             vertices of triangle 2: U0,U1,U2
 * result    : returns 1 if the triangles intersect, otherwise 0
 *
 */

#include <math.h>

#ifndef ALGROB_TRINTERSECT
#define ALGROB_TRINTERSECT

namespace tri_intersect{
	int coplanar_tri_tri(
		double N[3],double V0[3],double V1[3],double V2[3],
		double U0[3],double U1[3],double U2[3]
	);
	int tri_tri_intersect(
		double V0[3],double V1[3],double V2[3],
		double U0[3],double U1[3],double U2[3]
       	);
}

#endif
