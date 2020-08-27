#ifndef GET_ADJ
#define GET_ADJ

#include <bits/stdc++.h>
#include <mve/scene.h>
#include <graph.h>

using namespace std;
#define Infinity 1.7976931348623158e+100
struct Edge
{
    size_t begin;
    size_t end;
    Edge(size_t b, size_t e):begin(min(b,e)), end(max(b,e)){};

    //必须实现这个否则无法使用map<Edge,>
    bool operator< (const Edge &cmp) const
    {
       if(begin != cmp.begin){
           return (begin < cmp.begin);
       }
       else{
           return (end < cmp.end);
       }
    }
};

struct Edges
{
    vector<Edge> edges;
    map<Edge, vector<size_t> > adj;
   // Edges(vector<Edge> &edge,  map<Edge, vector<size_t> > &adj):edge(edge),adj(adj){}; 
};

Edges getadjacent(vector<math::Vec3f> &vertices, vector<size_t> &faces);

double alpha_expansion(size_t label, Edges &Adj, int face_num, vector<vector<size_t> > &faceslabel, vector<vector<size_t> > &p, vector<vector<double> > &W, vector<vector<size_t> > &newfacelabel);

double costD(size_t face_index, int label, vector<vector<size_t> > &p);

double costV(size_t fi1, size_t fi2, int fl1, int fl2, vector<vector<double> > &W, double s=1.0);

double Energy(Edges &Adj, vector<vector<size_t> > &tmplabel, vector<vector<double> > &W, vector<vector<size_t> > &p, double s=1.0);
#endif
