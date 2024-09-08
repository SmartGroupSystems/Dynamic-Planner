#ifndef  _EDTTRANSFORM_H
#define  _EDTTRANSFORM_H

#include <iostream>
#include <float.h>
#include <string.h>

// Removal Function - FVs that are the Voronoi sites of Voronoi cells that do not intersect Rd are removed
inline bool RemoveEDT(const double du, const double dv, const double dw, const double u, const double v, const double w);
inline int constructPartialVoronoi(double* D, double* g, int* h, int length);
inline void queryPartialVoronoi(const double* g, const int* h, const int ns, double* D, int length);
void voronoiEDT(double* D, double* g, int* h, int length);
inline void processDimN(int width, int height/*const mwSize *dims, const mwSize ndims*/, double *D);
void computeEDT(double *mxD, const double *mxI, int width, int height);

#endif
