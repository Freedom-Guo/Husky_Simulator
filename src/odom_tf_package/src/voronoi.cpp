//
// Created by freedomguo on 2021/3/4.
//
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <vector>

using namespace std;

#define pi 3.14156

float * get_outer_circle(float A[], float B[], float C[])
{
    float xa=A[0], ya=A[1], xb=B[0], yb=B[1], xc=C[0], yc=C[1];
    float kab, kbc, ab, bc;
    float kabm, kbcm, b1, b2;
    float xab=(xa+xb)/2.0f, yab=(ya+yb)/2.0f, xbc=(xb+xc)/2.0f, ybc=(yb+yc)/2.0f;
    float x, y,r;

    if(xb != xa)
        kab = (yb-ya) / (xb-xa);
    else
        kab = NULL;

    if(xc != xb)
        kbc = (yc - yb) / (xc - xb);
    else
        kbc = NULL;

    if(kab != NULL)
        ab = atan(kab);
    else
        ab = pi / 2.0f;

    if(kbc != NULL)
        bc = atan(kbc);
    else
        bc = pi / 2.0f;

    if(ab == 0){
        kabm - NULL;
        b1 = 0;
        x = xab;}
    else{
        kabm = tan(ab + pi / 2.0f);
        b1 = yab * 1.0f - xab * kabm * 1.0f;}

    if(bc == 0){
        kbcm - NULL;
        b2 = 0;
        x = xbc;}
    else{
        kbcm = tan(bc + pi / 2.0f);
        b2 = ybc * 1.0f - xbc * kbcm * 1.0f;}

    if(kabm != NULL && kbcm != NULL){
        x = (b2-b1) * 1.0f / (kabm-kbcm);}

    if(kabm != NULL)
        y = kabm * x + b1;
    else
        y = kbcm * x + b2;

    r = sqrt(pow(x-xa, 2) + pow(y-ya, 2));

    float result[3] = {x, y, r};
    return result;
}

void * get_intersect_point(float result[5], float a, float b, float c, float bound[])
{
    float flag=0, x1=0, y1=0, x2=0, y2=0;
    if(b == 0){
        x1 = -c / a;
        x2 = -c / a;
        y1 = bound[2];
        y2 = bound[3];}
    else{
        if(bound[3] >= (-c - a * bound[0]) / b >= bound[2]){
            if(flag == 0){
                x1 = bound[0];
                y1 = (-c-a*bound[0])/b;
                flag = 1;}
            else{
                x2 = bound[0];
                y2 = (-c-a*bound[0])/b;
                flag = 2;}}
        if(bound[3] >= (-c - a * bound[1]) / b >= bound[2]){
            if(flag == 0){
                x1 = bound[1];
                y1 = (-c-a*bound[1])/b;
                flag = 1;}
            else{
                x2 = bound[1];
                y2 = (-c-a*bound[1])/b;
                flag = 2;}}
        if(bound[1] >= (-c - b * bound[2]) / a >= bound[0]){
            if(flag == 0){
                x1 = bound[2];
                y1 = (-c-a*bound[2])/b;
                flag = 1;}
            else{
                x2 = bound[2];
                y2 = (-c-a*bound[2])/b;
                flag = 2;}}
        if(bound[1] >= (-c - b * bound[3]) / a >= bound[0]){
            if(flag == 0){
                x1 = bound[3];
                y1 = (-c-a*bound[3])/b;
                flag = 1;}
            else{
                x2 = bound[3];
                y2 = (-c-a*bound[3])/b;
                flag = 2;}}
        if(flag == 1){
            x2 = x1;
            y2 = y1;}
        result[0] = flag;
        result[1] = x1;
        result[2] = y1;
        result[3] = x2;
        result[4] = y2;}
}

float * intersect(float A[], float B[], float bound[])
{
    float C[2] = {0, 0};
    if(bound[0] <= A[0] <= bound[1] && bound[2] <= A[1] <= bound[3]){
        if(bound[0] <= B[0] <= bound[1] and bound[2] <= B[1] <= bound[3]){
            float flag = 1;
            float result[5] = {A[0], A[1], B[0], B[1], flag};
            return result;}
        else{
            float flag = 1;
            if(A[0] == B[0]){
                if(B[1] > bound[3]){
                    C[0] = A[0];
                    C[1] = bound[3];}
                else{
                    C[0] = A[0];
                    C[1] = bound[2];}}
            else{
                float a = A[1] - B[1];
                float b = B[0] - A[0];
                float c = B[1] * A[0] - A[1] * B[0];
                float *temp = new float [5];
                get_intersect_point(temp, a, b, c, bound);
                if(min(A[0], B[0]) <= temp[1] <= max(A[0], B[0]) and min(A[1], B[1]) <= temp[2] <= max(A[1], B[1])){
                    C[0] = temp[1];
                    C[1] = temp[2];}
                else{
                    C[0] = temp[2];
                    C[1] = temp[3];}}
            float result[5] = {A[0], A[1], C[0], C[1], flag};
            return result;}}
    else{
        if(bound[0] <= B[0] <= bound[1] and bound[2] <= B[1] <= bound[3]){
            float flag = 1;
            if(A[0] == B[0]){
                if(A[1] > bound[3]){
                    C[0] = B[0];
                    C[1] = bound[3];}
                else{
                    C[0] = B[0];
                    C[1] = bound[2];}}
            else{
                float a = A[1] - B[1];
                float b = B[0] - A[0];
                float c = B[1] * A[0] - A[1] * B[0];
                float *temp = new float [5];
                get_intersect_point(temp, a, b, c, bound);
                if(min(A[0], B[0]) <= temp[1] <= max(A[0], B[0]) and min(A[1], B[1]) <= temp[2] <= max(A[1], B[1])){
                    C[0] = temp[1];
                    C[1] = temp[2];}
                else{
                    C[0] = temp[3];
                    C[1] = temp[4];}}
            float result[5] = {B[0], B[1], C[0], C[1], flag};
            return result;}
        else{
            float flag = 0;
            if(A[0] == B[0]){
                float result[5] = {A[0], A[1], B[0], B[1], flag};
                return result;}
            else{
                float a = A[1] - B[1];
                float b = B[0] - A[0];
                float c = B[1] * A[0] - A[1] * B[0];
                float *temp = new float [5];
                get_intersect_point(temp, a, b, c, bound);
                if(temp[0] > 0){
                    float result[5] = {temp[1], temp[2], temp[3], temp[4], flag};
                    return result;
                }
                else{
                    float result[5] = {A[0], A[1], B[0], B[1], flag};
                    return result;}}}}
}

bool isintersec(float p1[], float p2[], float p3[], float p4[])
{
    float a = p2[1] - p1[1];
    float b = p1[0] - p2[0];
    float c = p2[0] * p1[1] - p1[0] * p2[1];
    if((a * p3[0] + b * p3[1] + c) * (a * p4[0] + b * p4[1] + c) <= 0)
        return true;
    else
        return false;
}

float * midline(float A[], float B[], float C[], float bound[])
{
    float a = 2 * (B[0] - A[0]);
    float b = 2 * (B[1] - A[1]);
    float c = pow(A[0], 2) - pow(B[0], 2) + pow(A[1], 2) - pow(B[1], 2);
    float *temp = new float [5];
    get_intersect_point(temp, a, b, c, bound);
    float D[2] = {temp[1], temp[2]};
    if(isintersec(A, B, C, D)){
        D[0] = temp[1];
        D[1] = temp[2];}
    else {
        D[0] = temp[3];
        D[1] = temp[4];
    }
    return D;
}

float cross(float p1[], float p2[])
{
    float temp1 = p1[0] * p2[1];
    float temp2 = p1[1] * p2[0];
    return temp1 - temp2;
}

float * get_l_point(float A[], float B[], float C[], float D[])
{
    float x1 = (C[0] + D[0]) / 2.0f;
    float y1 = (C[1] + D[1]) / 2.0f;
    float x2, y2;
    float v1[2] = {D[0] - A[0], D[1] - A[1]};
    float v2[2] = {D[0] - B[0], D[1] - B[1]};
    float v3[2] = {D[0] - C[0], D[1] - C[1]};
    if(cross(v1, v3) > 0 && cross(v2, v3) > 0)
    {
        if((pow((A[0] - x1), 2) + pow(A[1] - y1, 2) ) > (pow(B[0] - x1, 2) + pow(B[1] - y1, 2))){
            x2 = A[0];
            y2 = A[1];
        }
        else{
            x2 = B[0];
            y2 = B[1];
        }
    }
    else if(cross(v1, v3) > 0){
        x2 = A[0];
        y2 = A[1];
    }
    else{
        x2 = B[0];
        y2 = B[1];
    }
    float result[4] = {x1, y1, x2, y2};
    return result;
}

int agent_num = 5;
vector<int> death;

float * voronoi(float position[][2])
{
    vector<vector<float> > af_points;
    for(int i=0; i<agent_num; i++)
    {
        if(find(death.begin(), death.end(), i) != death.end())
        {
            vector<float> point;
            point.push_back(position[i][0]);
            point.push_back(position[i][1]);
            af_points.push_back(point);
        }
    }
}

