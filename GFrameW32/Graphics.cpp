#include "StdAfx.h"
#include "GF.h"
#include <vector>
#include <stack>

#ifndef M_PI
const double M_PI = 3.1415926535897932384626433832795;
#endif


void DrawLine(int x1, int y1, int x2, int y2, RGBPIXEL color) {
    int x = x1, y = y1; //current point
    int dx = x2 - x1, dy = y2 - y1;
    int ix, iy; // vlaue of increments (-1, 0, 1) along the coordinates
    int e;
    int i;

    if (dx > 0) ix = 1;
    else if (dx < 0) { ix = -1; dx = -dx; }
    else ix = 0;

    if (dy > 0) iy = 1;
    else if (dy < 0) { iy = -1; dy = -dy; }
    else iy = 0;

    if (dx >= dy) { // each step x increased by ix, increament of y depends on error e
        e = 2 * dy - dx;
        if (iy >= 0) {
            for (i = 0; i <= dx; i++) {
                gfSetPixel(x, y, color);
                if (e >= 0) {
                    y += iy;
                    e -= 2 * dx;
                }
                x += ix;
                e += dy * 2;
            }
        }
        else { // iy == -1
            for (i = 0; i <= dx; i++) {
                gfSetPixel(x, y, color);
                if (e > 0) {
                    y += iy;
                    e -= 2 * dx;
                }
                x += ix;
                e += dy * 2;
            }
        }
    }
    else { // dx < dy (each step y increased by iy, increament of x depends on error e)
        e = 2 * dx - dy;
        if (ix >= 0) {
            for (i = 0; i <= dy; i++) {
                gfSetPixel(x, y, color);
                if (e >= 0) {
                    x += ix;
                    e -= 2 * dy;
                }
                y += iy;
                e += dx * 2;
            }
        }
        else { // ix == -1
            for (i = 0; i <= dy; i++) {
                gfSetPixel(x, y, color);
                if (e > 0) {
                    x += ix;
                    e -= 2 * dy;
                }
                y += iy;
                e += dx * 2;
            }
        }
    }
}

void MatchPoints(const std::vector<iPoint>& points, RGBPIXEL color) {
    int n = points.size();
    for (int i = 0; i < n; i++) {
        DrawLine(points[i].x, points[i].y, points[(i + 1) % n].x, points[(i + 1) % n].y, color);
    }
}

void MoveObject(std::vector<iPoint>& obj, int x_axis, int y_axis) {
    int n = obj.size();
    for (int i = 0; i < n; i++) {
        obj[i].x += x_axis; // move start along x axis
        obj[i].y += y_axis; // move start along y axis
    }
}

// Add limit range to generate points
std::vector<iPoint> GenerartePoligon(int n, int size, RGBPIXEL color)
{
    srand(time(NULL));
    std::vector<iPoint> points;
    for (int i = 0; i < n; i++)
    {
        iPoint point;
        /*point.x = rand() % gfGetWindowWidth();
        point.y = rand() % gfGetWindowHeight();*/
        point.x = rand() % size;
        point.y = rand() % size;
        points.push_back(point);
    }
    return points;
}

int CrossProduct(const std::vector<iPoint>& A) {
    int X1 = (A[1].x - A[0].x);
    int Y1 = (A[1].y - A[0].y);
    int X2 = (A[2].x - A[0].x);
    int Y2 = (A[2].y - A[0].y);
    return (X1 * Y2 - Y1 * X2);
}

bool IsConvex(const std::vector<iPoint>& points) {
    int N = points.size();
    int prev = 0;
    int curr = 0;
    for (int i = 0; i < N; i++) {
        std::vector<iPoint> temp = { points[i], points[(i + 1) % N], points[(i + 2) % N] };
        curr = CrossProduct(temp);
        if (curr != 0) {
            if (curr * prev < 0) {
                return false;
            }
            else {
                prev = curr;
            }
        }
    }
    return true;
}

bool OnSegment(iPoint p, iPoint q, iPoint r) {
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
        return true;
    return false;
}

int Orientation(iPoint p, iPoint q, iPoint r) {
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;    // collinear
    return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

bool IsIntersect(iPoint p1, iPoint q1, iPoint p2, iPoint q2) {
    int o1 = Orientation(p1, q1, p2);
    int o2 = Orientation(p1, q1, q2);
    int o3 = Orientation(p2, q2, p1);
    int o4 = Orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && OnSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && OnSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && OnSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && OnSegment(p2, q1, q2)) return true;

    return false;  // Doesn't fall in any of the above cases
}

bool IsSimplePoligon(std::vector<iPoint> points) {
    int n = points.size();

    if (n < 3) return false;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i != j && (j != (i + 1) % n) && (j != (i + n - 1) % n)) {
                // not consider same segment and 2 neighbors of that segment
                if (IsIntersect(points[i], points[(i + 1) % n], points[j], points[(j + 1) % n])) {
                    return false;
                }
            }
        }
    }
    return true;
}

void PrintIsConvex(std::vector<iPoint> obj, int x, int y) {
    if (IsConvex(obj)) {
        gfDrawText(x, y - 15, "Is Convex!", RGBPIXEL(255, 255, 0));
    }
    else {
        gfDrawText(x, y - 15, "Is not Convex!", RGBPIXEL(255, 255, 0));
    }
    // gfDrawText(x, y - 30, "Is Simple!", RGBPIXEL(255, 255, 0));
    if (IsSimplePoligon(obj)) {
        gfDrawText(x, y - 30, "Is Simple!", RGBPIXEL(255, 255, 0));
    }
    else {
        gfDrawText(x, y - 30, "Is not Simple!", RGBPIXEL(255, 255, 0));
    }
}

void FloodFill4(int x, int y, int width, int height, RGBPIXEL newColor, RGBPIXEL oldColor) {
    if (!(x >= 0 && x < width && y >= 0 && y < height && gfGetPixel(x, y) == oldColor)) {
        // User provided invalid coordinates
        return;
    }

    static const int dx[4] = { 0, 1, 0, -1 };
    static const int dy[4] = { 1, 0, -1, 0 };
    std::stack<std::pair<int, int>> pstack;
    pstack.push(std::make_pair(x, y));
    while (!pstack.empty()) {
        std::pair<int, int> pixel = pstack.top();
        pstack.pop();
        int i = pixel.first;
        int j = pixel.second;
        gfSetPixel(i, j, newColor);
        for (int k = 0; k < 4; k++) {
            int nx = i + dx[k];
            int ny = j + dy[k];
            if (nx >= 0 && nx < width && ny >= 0 && ny < height && gfGetPixel(nx, ny) == oldColor) {
                pstack.push(std::make_pair(nx, ny));
            }
        }
    }
}

const std::vector<iPoint> TRIANGLE = { iPoint(0, 0), iPoint(120, 0), iPoint(60, 100) };
const std::vector<iPoint> STAR = { iPoint(0, 30), iPoint(30, 30), iPoint(40, 0), iPoint(50, 30), iPoint(80, 30),
                                  iPoint(50, 45), iPoint(60, 80), iPoint(40, 55), iPoint(20, 80), iPoint(30, 45) };
const std::vector<iPoint> SQUARE = { iPoint(0, 0), iPoint(100, 0), iPoint(100, 100), iPoint(0, 100) };
const std::vector<iPoint> OBJECT1 = { {0, 0}, {0, 100}, {50, 50}, {100, 100}, {100, 0} };

void Lab1(int width, int height) {
    std::vector<iPoint> square = SQUARE;
    int x_square_pos = 100;
    int y_square_pos = 100;
    MoveObject(square, x_square_pos, y_square_pos);
    PrintIsConvex(square, x_square_pos, y_square_pos);
    MatchPoints(square, RGBPIXEL::Yellow());
    FloodFill4(x_square_pos + 10, y_square_pos + 10, width, height, RGBPIXEL::Blue(), RGBPIXEL::Black());

    std::vector<iPoint> star = STAR;
    int x_star_pos = 250;
    int y_star_pos = 100;
    MoveObject(star, x_star_pos, y_star_pos);
    PrintIsConvex(star, x_star_pos, y_star_pos);
    MatchPoints(star, RGBPIXEL::Yellow());
    FloodFill4(x_star_pos + 40, y_star_pos + 30, width, height, RGBPIXEL::Red(), RGBPIXEL::Black());

    std::vector<iPoint> obj1 = OBJECT1;
    int x_obj1_pos = 100;
    int y_obj1_pos = 250;
    MoveObject(obj1, x_obj1_pos, y_obj1_pos);
    PrintIsConvex(obj1, x_obj1_pos, y_obj1_pos);
    MatchPoints(obj1, RGBPIXEL::Yellow());
    FloodFill4(x_obj1_pos + 40, y_obj1_pos + 30, width, height, RGBPIXEL::DkGreen(), RGBPIXEL::Black());

    std::vector<iPoint> triangle = TRIANGLE;
    int x_triangle_pos = 250;
    int y_triangle_pos = 250;
    MoveObject(triangle, x_triangle_pos, y_triangle_pos);
    PrintIsConvex(triangle, x_triangle_pos, y_triangle_pos);
    MatchPoints(triangle, RGBPIXEL::Yellow());
    FloodFill4(x_triangle_pos + 10, y_triangle_pos + 10, width, height, RGBPIXEL::Yellow(), RGBPIXEL::Black());

    int rndp_size = 110;
    std::vector<iPoint> random_polygon = GenerartePoligon(10, rndp_size, RGBPIXEL::Yellow());
    int x_rnd_pos = 400, y_rnd_pos = 100;
    MoveObject(random_polygon, x_rnd_pos, y_rnd_pos);
    PrintIsConvex(random_polygon, x_rnd_pos, y_rnd_pos);
    MatchPoints(random_polygon, RGBPIXEL::Yellow());
}

bool gfInitScene()
{
    int width = 640;
    int height = 480;

    gfSetWindowSize(width, height);

    //gfSetPixel( 20, 20, RGBPIXEL(255, 255, 255) );

    //gfDrawRectangle( 100, 120, 170, 150, RGBPIXEL(255, 255, 255) );

    gfDrawText(20, 20, "Hello World!", RGBPIXEL(255, 255, 0));

    //DrawLine(10, 10, 10, 500, RGBPIXEL::Yellow());

    //gfDisplayMessage("Message!");

    Lab1(width, height);

    bool star_simple = IsSimplePoligon(STAR);
    bool square_simple = IsSimplePoligon(SQUARE);
    bool obj1_simple = IsSimplePoligon(OBJECT1);
    bool triangle_simple = IsSimplePoligon(TRIANGLE);

    return true;
}

// Вызывается в цикле до момента выхода из приложения.
// Следует использовать для создания анимационных эффектов
void gfDrawScene()
{
    //gfClearScreen(RGBPIXEL::Black());

    //static int x = 0;
    //gfDrawRectangle(x, 100, x + 50, 130, RGBPIXEL::Blue());
    //x = (x + 1) % gfGetWindowWidth() ;

    //int x = gfGetMouseX(),
    //    y = gfGetMouseY();
    //gfDrawRectangle(x - 10, y - 10, x + 10, y + 10, RGBPIXEL::Green());
}

// Вызывается один раз перед выходом из приложения.
// Следует использовать для освобождения выделенных
// ресурсов (памяти, файлов и т.п.)
void gfCleanupScene()
{
}

// Вызывается когда пользователь нажимает левую кнопку мыши
void gfOnLMouseClick( int x, int y )
{
    x; y;
    gfDrawRectangle(x - 10, y - 10, x + 10, y + 10, RGBPIXEL::Green());
}

// Вызывается когда пользователь нажимает правую кнопку мыши
void gfOnRMouseClick( int x, int y )
{
    x; y;
}

// Вызывается когда пользователь нажимает клавишу на клавиатуре
void gfOnKeyDown( UINT key )
{
    key;

    if( key == 'A' )
        gfDisplayMessage( "'A' key has been pressed" );
}

// Вызывается когда пользователь отжимает клавишу на клавиатуре
void gfOnKeyUp( UINT key )
{
    key;

    //if( key == 'B' )
    //    gfDisplayMessage( "'B' key has been un-pressed" );
}
