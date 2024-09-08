#include <bspline_race/EdtTransform.h>
// Removal Function - FVs that are the Voronoi sites of Voronoi cells that do not intersect Rd are removed
inline bool RemoveEDT(const double du, const double dv, const double dw, const double u, const double v, const double w)
{
    double a = v - u;
    double b = w - v;
    double c = w - u;

    // See Eq. 2 from Maurer et al., 2003
    return ((c * dv - b * du - a * dw) > (a * b * c));
}

inline int constructPartialVoronoi(double* D, double* g, int* h, int length)
{
    //
    // Construct partial voronoi diagram (see Maurer et al., 2003, Fig. 5, lines 1-14)
    // Note: short variable names are used to mimic the notation of the paper
    //
    int el = -1;

    for (int k = 0; k < length; k++)
    {
        double fk = D[k];
        if (fk != -1.0f)
        {
            if (el < 1)
            {
                el++;
                g[el] = fk;
                h[el] = k;
            }
            else
            {
                while ((el >= 1) && RemoveEDT(g[el - 1], g[el], fk, h[el - 1], h[el], k))
                {
                    el--;
                }
                el++;
                g[el] = fk;
                h[el] = k;
            }
        }
    }

    return(el);
}

inline void queryPartialVoronoi(const double* g, const int* h, const int ns, double* D, int length)
{
    //
    // Query partial Voronoi diagram (see Maurer et al., 2003, Fig. 5, lines 18-24)
    //
    int el = 0;

    for (int k = 0; k < length; k++)
    {
        while ((el < ns) && ((g[el] + ((h[el] - k)*(h[el] - k))) >(g[el + 1] + ((h[el + 1] - k)*(h[el + 1] - k)))))
        {
            el++;
        }
        D[k] = g[el] + (h[el] - k)*(h[el] - k);
    }
}

void voronoiEDT(double* D, double* g, int* h, int length)
{
    // Note: g and h are working vectors allocated in calling function
    int ns = constructPartialVoronoi(D, g, h, length);
    if (ns == -1)
        return;

    queryPartialVoronoi(g, h, ns, D, length);

    return;
}

inline void processDimN(int width, int height/*const mwSize *dims, const mwSize ndims*/, double *D)
{
    // Create temporary vectors for processing along dimension N

    double* g = new double[width];
    int* h = new int[width];

    // 若为二维数组，则得到第一维元素总数，即行数。注意，matlab按列存储，C按行存储。
    for (int k = 0; k < height; k++)
    {
        // Process vector
        voronoiEDT(&D[k*width], g, h, width);
    }

    delete[] g;
    delete[] h;
}

// mxI为输入，double*类型，mxD为输出，double* 类型
// 输入为二值化图像，大于阈值的不为0，小于阈值的为0。
// 这里逻辑好像有点问题，但是这么写完就很stable了，就这样吧，，，，
void computeEDT(double *mxD, const double *mxI, int width, int height)
{
    // double* D1 = new double[width];
    // double* g = new double[width];
    // int* h = new int[width];
    double* D1 = new double[height];
    double* g = new double[height];
    int* h = new int[height];
    
    for (int k = 0; k < width; k++)
    {
        for (int i = 0; i < height; i++)
        {
            D1[i] = (mxI[i*width + k] == 1) ? 0.0f : -1.0f;
        }

        voronoiEDT(D1, g, h, height);

        for (int i = 0; i < height; i++)
        {
            mxD[i*width + k] = D1[i];
        }
    }

    delete[] D1;
    delete[] g;
    delete[] h;

    processDimN(width, height, mxD);
}