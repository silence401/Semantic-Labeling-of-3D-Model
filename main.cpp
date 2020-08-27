#include <bits/stdc++.h>
#include <future>
#include <typeinfo>

#include <acc/bvh_tree.h>
#include <util/timer.h>
#include <util/arguments.h>
#include <mve/scene.h>
#include <mve/camera.h>
#include <mve/image.h>
#include <mve/image_io.h>

#include <igl/avg_edge_length.h>
#include <igl/cotmatrix.h>
#include <igl/invert_diag.h>
#include <igl/massmatrix.h>
#include <igl/parula.h>
#include <igl/per_corner_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/principal_curvature.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/readPLY.h>

#include "get_adjacent.h"

using namespace std;
//template<class T>
inline size_t max_num(vector<size_t> &A) {
	//遍历，获得每个元素出现的次数
    //cout<<1<<endl;
    size_t res = 5;
    int maxcnt = 0;
     for(auto a : A)
     {
        
         int cnt = count(A.begin(), A.end(), a);
         if(cnt > maxcnt)
         {
             maxcnt = cnt;
             res = a;
         }
     }
     return res;
}
int main()
{
    //read 3D model V:vertices F:faces, Change them from igbl format to std::vector
    Eigen::MatrixXd V;
    Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> F;
    igl::readOBJ("/home/silence401/wujinbo/rayinsection/mesh/test.obj",V,F);
    vector<size_t> faces(F.data(), F.data() + F.size());
    vector<math::Vec3f> vertices;
    for( auto row = 0; row<V.rows(); row++){
        
        math::Vec3f tmp(V(row, 0), V(row, 1), V(row, 2));
        vertices.push_back(tmp);
    }
    //read and change over

    //get adjacent
     Edges Adj = getadjacent(vertices, faces);
    //  for(int i=0; i<Adj.edges.size(); i++)
    //  {W
    //      Edge t = Adj.edges[i];
    //      cout<<t.begin<<' '<<t.end<<" :"<<endl;
    //      for(int j=0; j<Adj.adj[t].size(); j++)
    //      {
    //          cout<<Adj.adj[t][j]<<endl;
    //      }
    //  }
    //  return 0;
     /**沙盒***********************/
     cout<<"沙盒开始"<<endl;
    //  Eigen::MatrixXd HN;
    // Eigen::SparseMatrix<double> L,M,Minv;
    // igl::cotmatrix(V,F,L);
    // igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_VORONOI,M);
    // igl::invert_diag(M,Minv);
    // // Laplace-Beltrami of position
    //  HN = -Minv*(L*V);
    // // Extract magnitude as mean curvature
    //  Eigen::VectorXd H = HN.rowwise().norm();
    //  cout<<V.rows()<<' '<<V.cols()<<endl;
    //  cout<<F.rows()<<' '<<F.cols()<<endl;
    //  cout<<HN.rows()<<' '<<HN.cols()<<endl;
    //  cout<<"沙盒结束"<<endl;
     Eigen::MatrixXd PD1,PD2;
     Eigen::VectorXd PV1,PV2;
     igl::principal_curvature(V,F,PD1,PD2,PV1,PV2);
     cout<<PD1.rows()<<' '<<PD2.cols()<<endl; 
     cout<<"沙盒结束"<<endl;
     /***********沙盒************/
    //vector<double>W[faces.size()/3+1];
    vector<vector<double> > W(faces.size()/3+1);
    vector<size_t> tmpfaces;
    for(int i = 0; i < faces.size() ; i++){
        
        tmpfaces.emplace_back(faces[i]);
        int in = i+1;
        if(in%3==0){
            int v1 = tmpfaces[0];
            int v2 = tmpfaces[1];
            int v3 = tmpfaces[2];
    //  cout<<typeid(PV1(v1)*PD1.row(v1)+PV1(v2)*PD1.row(v2)+PV1(v3)*PD1.row(v3)).name()<<endl;
           // cout<<PV1(0)*PD1.row(0)<<endl;
           // 以三角面片三个顶点的主曲率平均值作为该面片的曲率
             Eigen::Vector3d pv1 = (PV1(v1)*PD1.row(v1)+PV1(v2)*PD1.row(v2)+PV1(v3)*PD1.row(v3));
             Eigen::Vector3d pv2 = (PV2(v1)*PD2.row(v1)+PV2(v2)*PD2.row(v2)+PV2(v3)*PD2.row(v3));

            // //pv1.insert(pv1.end(), pv2.begin(), pv2.end());
            vector<double> tmpp;
            for(int k=0; k<3; k++)
            tmpp.emplace_back(pv1(k));
            for(int k=0; k<3; k++)
            tmpp.emplace_back(pv2(k));
            
            W[i/3].insert(W[i/3].end(), tmpp.begin(), tmpp.end());
            //cout<<"W: "<<W[i/3][0]<<' '<<W[i/3][1]<<' '<<W[i/3][2]<<endl;
            tmpfaces.clear();
        }
    }
    cout<<W[0].size()<<endl;
    cout<<faces.size()<<endl;
    cout<<vertices.size()<<endl;
    cout<<V(0)<<endl;
    cout<<F(0)<<endl;
    //Build BVH Tree
    cout << "Building BVH from" << "face size: " << faces.size()/3 << endl;
    acc::BVHTree bvhtree(faces, vertices);
    cout<<"done. " << endl;
    //Build Over
    

    //read MVE format scene and get views
    mve::Scene::Ptr scene;
    try{
        scene = mve::Scene::create("/home/silence401/wujinbo/rayinsection/MVE");
    }catch (exception& e){
        cerr << "could not open scene" << endl;
    }
    vector<mve::View::Ptr> views;
    mve::Scene::ViewList const & aviews = scene->get_views();
    //read MVE over
    
    //image_name is the label image_name.
    string image_name = "label1";
    //vector<size_t> p[faces.size()/3+1];
    vector<vector<size_t> >p(faces.size()/3+1);

    int num = 0;
    for(mve::View::Ptr view : aviews){

        mve::CameraInfo const & camera = view->get_camera();
        mve::View::ImageProxy const * proxy = view->get_image_proxy(image_name);
        //cv::Mat img = cv::imread(view->get_directory()+'/'+image_name+".png");
        mve::ByteImage::Ptr image = view->get_byte_image(image_name);
       //  mve::ByteImage::Ptr image = mve::ByteImage::create(view->get_byte_image(image_name));
    
        if(image == NULL){
            cerr<<image_name<<" is not exsits"<<endl;
            continue;
        }

        //get the camera extrinic and intrinsic and get the projection matrix
        math::Vec3f origin;
        camera.fill_camera_pos(*origin);
        math::Matrix3f invproj;
        camera.fill_inverse_calibration(*invproj, proxy->width, proxy->height);
        math::Matrix3f c2w_rot;
        camera.fill_cam_to_world_rot(*c2w_rot);
       

        #pragma omp parallel for
        for(int y = 0; y < proxy->height; y++){
            for(int x = 0; x < proxy->width; x++){
                acc::Ray ray;
                ray.origin = origin;
                math::Vec3f v = invproj * math::Vec3f ((float)x + 0.5f, (float)y + 0.5f, 1.0f);
                ray.dir = c2w_rot.mult(v.normalized()).normalize();
                ray.tmin = 0.0f;
                ray.tmax = numeric_limits<float>::infinity();
                acc::BVHTree::Hit hit;
                if (bvhtree.intersect(ray, &hit)){
                //   if(hit.idx == 0)
                //   cerr<<"face from 0"<<endl;
                  int r = int((*image)(x, y, 0));
                  int g = int((*image)(x, y, 1));
                  int b = int((*image)(x, y, 2));
                  //待修改
                  if(r == 0 && g == 128 && b == 0)
                  p[hit.idx].push_back(0);
                  if(r == 128 && g == 128 && b == 0)
                  p[hit.idx].push_back(1);
                  if(r == 128 && g == 0 && b == 0)
                  p[hit.idx].push_back(2);
                  if(r == 0 && g == 0 && b == 0)
                  p[hit.idx].push_back(3);
                }

            }
           
        }
    num++;

    if(num == 26)
    break;
    }
    
    fstream objFile;

    try{
        objFile.open("./result/outobj.obj", ios::out);
    }catch (exception& e){
        cerr << "could not create objfile" << endl;
    }
    //scout<<p[60960/3].size()<<endl;
    objFile<<"mtllib ./test.obj.mtl"<<endl;
    objFile<<"vt "<<0.5<<' '<<25.0/256<<endl;
    objFile<<"vt "<<0.5<<' '<<75.0/256<<endl;
    objFile<<"vt "<<0.5<<' '<<125.0/256<<endl;
    objFile<<"vt "<<0.5<<' '<<175.0/256<<endl;
    objFile<<"vt "<<0.5<<' '<<225.0/256<<endl;


    for(auto v : vertices){
        objFile<<'v'<<' '<<v<<endl;
    }

    cout<<"?"<<endl;cout<<"what fuck"<<endl;
    vector<size_t> outF;
    vector<vector<size_t> > faceslabel(faces.size()/3);
    int cnt = 0;
    int backgound = 3;
    for(auto f : faces){
       // objFile<<f<<" ";
       
        ++cnt;
        if(cnt%3==0){
          if(p[(cnt-1)/3].size()>0){
            // size_t t = max_num(p[(cnt-1)/3]);
             size_t t = p[(cnt-1)/3][0];
             faceslabel[(cnt-1)/3].push_back(t);
          }
          else{
             faceslabel[(cnt-1)/3].push_back(backgound);
          }
        }
    }
   // cout<<"here1"<<endl;
            // cout<<"2"<<endl;
           
        //      objFile<<"f "<<outF[0]+1<<'/'<<t<<' '<<outF[1]+1<<'/'<<t<<' '<<outF[2]+1<<'/'<<t<<endl;
        //  }ost = costV() + costD()
    double cost = Energy(Adj, faceslabel, W, p);
    int face_num = faces.size()/3;
    vector<vector<size_t> >tmplabel(faceslabel);
    vector<vector<size_t> >reslabel(faceslabel);
    int labelnum = 4;
    int epoch = 10;
    cout<<"cost: "<<cost<<endl;
    while(epoch--){
        cout<<"epoch: "<<epoch<<endl;
        int flag = 0;
        double costnow = cost;
      //  cout<<"cost: "<<cost<<endl;
        for(int i = 0; i < labelnum; i++){
       //     cout<<"here2"<<endl;
            double tmpcost = alpha_expansion(i, Adj, face_num, tmplabel, p, W, reslabel);
         //   cout<<"here3"<<endl;
            cout<<"tmpcost: "<<tmpcost<<endl;
            if (tmpcost < costnow)
            {
                costnow = tmpcost;
                tmplabel = reslabel;

            }
            
        }
        cout<<"cost: "<<cost<<endl;
       // cout<<"costnow: "<<costnow<<endl;
       if(cost > costnow)
       {
           flag  = 1;
           cost = costnow;
           faceslabel = tmplabel;
       }
        if(!flag)
        break;
    }
    cnt = 0;
    for(auto f : faces){
         outF.emplace_back(f);
         ++cnt;
         if(cnt%3==0){
             size_t t = faceslabel[(cnt-1)/3][0] + 1;
             objFile<<"f "<<outF[0]+1<<'/'<<t<<' '<<outF[1]+1<<'/'<<t<<' '<<outF[2]+1<<'/'<<t<<endl;
             outF.clear();
         }
    }
    // igl::opengl::glfw::Viewer viewer;
    // viewer.data().set_mesh(V, F);
    // viewer.launch();
   // acc::BVHTree bvhtree(faces, vertices);

}