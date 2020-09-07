
#include "alpha_expansion.h"
Eigen::MatrixXd N_faces;
void add( vector<Edge> &edges, map<Edge, vector<size_t> > &adj, Edge e, size_t face_index)
{
    if(adj.find(e) == adj.end()){
             //  cout<<4<<endl;
               adj[e].push_back(face_index);
             //  cout<<4<<endl;
               edges.push_back(e);
           }
           else{
             //  cout<<5<<endl;
             //adj[e].insert()
               adj[e].push_back(face_index);
             //  cout<<5<<endl;
           }
}

double costV(size_t fi1, size_t fi2, int fl1, int fl2, vector<vector<double> > &W, double s)
{
    //cout<<"costV"<<endl;
    double sum= 0 ;
    if (fl1 != fl2)
        return 1;
    for(int i=0; i<6 ; i++)
    {
      //  cout<<"costDW: "<<W[fi1][i]<<' '<<W[fi2][i]<<endl;
        sum += (W[fi1][i] - W[fi2][i]) * (W[fi1][i] - W[fi2][i]);

    }
    //sum = sqrt(sum);
    sum = sum*s;
  //  cout<<"sum: "<<sum<<endl;
   // cout<<"costV: "<<sum<<endl;
    return min(1.0, sum);
}
double costVN(size_t fi1, size_t fi2, int fl1, int fl2, vector<vector<double> > &W, double s)
{
    //cout<<"costV"<<endl;
    double sum= 0 ;
    if (fl1 != fl2)
        return 1;
    // for(int i=0; i<6 ; i++)
    // {
    //   //  cout<<"costDW: "<<W[fi1][i]<<' '<<W[fi2][i]<<endl;
    //     sum += (W[fi1][i] - W[fi2][i]) * (W[fi1][i] - W[fi2][i]);

    // }
    sum += (N_faces(fi1,0)*N_faces(fi2,0)+N_faces(fi1,1)*N_faces(fi2,1)+N_faces(fi1,2)*N_faces(fi2,2));

    sum = sqrt(sum);
    sum = sum*0.2;
  //  cout<<"sum: "<<sum<<endl;
   // cout<<"costV: "<<sum<<endl;
    return min(1.0, sum);
}
double costD(size_t face_index, int  label, int labelnum, vector<vector<size_t> > &p)
{
   // cout<<"costD"<<endl;
//     if(p[face_index].size() == 0)
//     return 1;

//     double cnt  = 0;
//     for(int i=0;  i<p[face_index].size(); i++)
//     {
//         int l = p[face_index][i];
//         if(l==label)
//         cnt += 1.0;
//     }
//   //  cout<<"costD: "<<1-(cnt/p[face_index].size())<<endl;
//     return 1-(cnt/p[face_index].size());


     double sump = 0, fp = 0;
     fp = p[face_index][label];
     for (int i = 0; i < labelnum+1; i++)
     {
         sump += p[face_index][i];
     }
     if(sump == 0)
     return 1;
    //  cout<<"labelname: "<<label<<endl;
    //  cout<<"fp: "<<fp<<endl;
    //  cout<<"sump: "<<sump<<endl;
     return 1 - fp*1.0/sump;

}

Edges  getadjacent(vector<math::Vec3f> &vertices, vector<size_t> &faces)
{
    //cout<<3<<endl;
    vector<Edge> edges;
    map<Edge, vector<size_t> > adj;
    vector<size_t> tmpface; 
    for(int i = 0; i < faces.size(); i++){
        tmpface.emplace_back(faces[i]);
        int in = i+1;
        if(in%3 == 0){
           int v1 = tmpface[0];
           int v2 = tmpface[1];
           int v3 = tmpface[2];
           
           Edge e1(v1, v2);
           Edge e2(v2, v3);
           Edge e3(v1, v3);

           add(edges, adj, e1, i/3);
           add(edges, adj, e2, i/3);
           add(edges, adj, e3, i/3);
           tmpface.clear();
        }
        
    }
    Edges res;
    res.edges = edges;
    res.adj = adj;
    return res;
}
double Energy(int labelnum, Edges &Adj, vector<vector<size_t> > &tmplabel, vector<vector<double> > &W, vector<vector<size_t> > &p, double s)
{
    cout<<"Energy"<<endl;
    double sumcost = 0;
    int face_num = tmplabel.size();
   // cout<<face_num<<endl;
    //cout<<tmplabel[0][0]<<endl;
    for(int i = 0; i < face_num; i++)
    {
     //   cout<<"tmplabel: "<<tmplabel[i][0]<<endl;
        sumcost += (costD(i, tmplabel[i][0], labelnum, p));
        //cout<<"sumcost:"<<sumcost<<endl;
    }
    cout<<"D: "<<sumcost;
    for(int i=0; i<Adj.edges.size(); i++)
    {
        Edge t = Adj.edges[i];
        if(Adj.adj[t].size() != 2)
        continue;
        int face_index1 = Adj.adj[t][0];
        int face_index2 = Adj.adj[t][1];
     //   cout<<"f: "<<face_index1<<' '<<face_index2<<endl;
        sumcost = sumcost + 0.2*costV(face_index1, face_index2, tmplabel[face_index1][0], tmplabel[face_index2][0], W, 0.2);
    }
    cout<<"   sumcost: "<< sumcost<<endl;
    return sumcost;
}
// void get_adjacent_faces(int fi, Edge &Adj， )
// {

// }
double alpha_expansion(size_t label, Edges &Adj, int face_num, int labelnum, vector<vector<size_t> > &faceslabel, vector<vector<size_t> > &p, vector<vector<double> > &W, vector<vector<size_t> > &newfacelabel)
{
    // if(label == 3)
    // cout<<"What fuck2"<<endl;
  //  cout<<"alpha expansion"<<endl;
    int extrapoint = 0;
    typedef Graph<double,double,double> GraphType;
	GraphType *g = new GraphType(/*estimated # of nodes*/ face_num, /*estimated # of edges*/ 6*face_num);
    g -> add_node(face_num);
    //添加邻接边
    for(int i=0; i<Adj.edges.size(); i++)
    {
        Edge t = Adj.edges[i];
        if(Adj.adj[t].size() == 2)
        {
            int face_index1 = Adj.adj[t][0];
            int face_index2 = Adj.adj[t][1];
            if(faceslabel[face_index1] != faceslabel[face_index2])
            {
                g->add_node();
                double costv1 = costV(face_index1, face_index2, faceslabel[face_index1][0], label, W, 0.2);
                double costv2 = costV(face_index1, face_index2, label, faceslabel[face_index2][0], W, 0.2);
                double costv3 = costV(face_index1, face_index2, faceslabel[face_index1][0], faceslabel[face_index2][0], W, 0.2);
             //  cout<<"edge: "<<costv1<<' '<<costv2<<endl;
             //  cout<<"tedge: "<<costv3<<endl;
                g -> add_edge(face_index1, face_num+extrapoint,    /* capacities */  costv1, costv1);
                g -> add_edge(face_num+extrapoint, face_index2, costv2, costv2);
                g -> add_tweights(face_num+extrapoint, 0, costv3);
                extrapoint ++;
            }
            else
            {
                
                double costv = costV(face_index1, face_index2, faceslabel[face_index1][0], label, W, 0.2);
                g -> add_edge(face_index1, face_index2, costv, costv);
            }


        }
        if(Adj.adj[t].size() > 2)
        cout<<"what fuck:"<<Adj.adj[t].size()<<endl;
    } 
    //添加source->vertex target->vertex
    for(int i = 0; i<face_num; i++)
    {
        if(faceslabel[i][0] == label)
        {
            double  costd1 = costD(i, label, labelnum, p);
            g -> add_tweights(i, costd1, Infinity);
        }
        else
        {
            double costd2 = costD(i, faceslabel[i][0], labelnum,  p);
            double  costd1 = costD(i, label, labelnum, p);
           // cout<<"costD: "<<costd1<<' '<<costd2<<endl;
            g -> add_tweights(i, costd1, costd2);
        }
    }
    double flow = g -> maxflow();
  //  vector<vector<size_t> > newfacelabel(faceslabel);
    int cnt = 0;
    for(int i = 0; i<face_num; i++)
    {
        if(g -> what_segment(i) != GraphType::SOURCE)
        newfacelabel[i][0] = label,cnt++;  
        else
        newfacelabel[i][0] = faceslabel[i][0];
    }
    cout<<"giabian: "<<cnt<<endl;
    // double sumcost = 0;
    // for(int i = 0; i < face_num; i++)
    // {
    //     sumcost += costD(i, newfacelabel[i], p);
    // }
    // for(int i=0; i<Adj.edges.size(); i++)
    // {
    //     int face_index1 = Adj.adj[t][0];
    //     int face_index2 = Adj.adj[t][1];
    //     sumcost = sumcost + s*costV(face_index1, face_index2, newfacelabel[face_index1], newfacelabel[face_index2], W);
    // }
    double sumcost = Energy(labelnum, Adj, newfacelabel, W, p);
    // if(sumcost < costnow)
    // {
    //     costnow = sumcost;
    //     for(int i = 0; i<face_num; i++)
    //     {
    //         tmplabel[i][0] = newfacelabel[i][0];
    //     }
    //    return true;
    // }
    // else
    // {    int flag = 0;
    delete g;
    return sumcost;
}