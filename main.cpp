#include <bits/stdc++.h>
#include <future>
#include <typeinfo>
#include <io.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

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

#include "alpha_expansion.h"

using namespace std;
#define MODE (S_IRWXU | S_IRWXG | S_IRWXO)
//Eigen::MatrixXd N_faces;

struct Arguments {
    string in_scene;
    string in_mesh;
    string out_dir;
    int labelnum;
    string image;
    int max_epoch;

};

Arguments parse_args(int argc, char **argv){
    util::Arguments args;
    args.set_exit_on_error(true);
    args.set_nonopt_maxnum(6);
    args.set_nonopt_minnum(6);
    args.set_usage("Usage: " + std::string(argv[0]) + " [OPTS] IN_SCENE  IN_MESH OUT_DIR LABELNUM LABEL_IMAGE MAX_epoch");
    args.parse(argc, argv);

    Arguments conf;
    conf.in_scene = args.get_nth_nonopt(0);
    conf.in_mesh = args.get_nth_nonopt(1);
    conf.out_dir = args.get_nth_nonopt(2);
    conf.labelnum = args.get_nth_nonopt_as<int>(3);
    conf.image = args.get_nth_nonopt(4);
    conf.max_epoch = args.get_nth_nonopt_as<int>(5);

    return conf;
}
//template<class T>
inline size_t max_num(vector<size_t> &A, int labelnum) {
	//遍历，获得每个元素出现的次数
    //cout<<1<<endl;
    size_t res = labelnum;
    int maxcnt = 0;
    //  for(auto a : A)
    //  {
        
    //      int cnt = count(A.begin(), A.end(), a);
    //      if(cnt > maxcnt)
    //      {
    //          maxcnt = cnt;
    //          res = a;
    //      }
    //  }
    //  return res;
    for(int i=0; i<labelnum+1; i++)
    {
          if(A[i] > maxcnt)
          {
              maxcnt = A[i];
              res = i;
          }
    }
    return res;
}
int main(int argc, char **argv)
{
    Arguments conf = parse_args(argc, argv);

    //read 3D model V:vertices F:faces
    Eigen::MatrixXd V;
    Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> F;
    igl::read_triangle_mesh(conf.in_mesh, V, F);
    cout<<"Mesh Read Over!"<<endl;
    cout<<V.rows()<<endl;
    //change the Eigen::format to std::vector format
    vector<size_t> faces(F.data(), F.data() + F.size());
    cout<<faces.size()<<endl;
    vector<math::Vec3f> vertices;

    for( auto row = 0; row<V.rows(); row++){
        
        math::Vec3f tmp(V(row, 0), V(row, 1), V(row, 2));
        vertices.push_back(tmp);
    }
    cout<<vertices.size()<<endl;

    //get adjacent information see more detail in the defination of Edges
     Edges Adj = getadjacent(vertices, faces);
     cerr<<"Get Adjacent Information Done!"<<endl;
     //conculate the principal curvatures
     Eigen::MatrixXd PD1,PD2;
     Eigen::VectorXd PV1,PV2;
     vector<int> bad_vertices;

     igl::per_face_normals(V, F, N_faces);
     cerr<<N_faces.size()<<endl;
     cerr<<F.size()<<endl;
     cerr<<"here over!!!!"<<endl;
     igl::principal_curvature(V,F,PD1,PD2,PV1,PV2, bad_vertices);
     cerr<<"here1"<<endl;
    //  igl::per_face_normals(V, F, N_faces);
    //  cout<<N_faces.size()<<endl;
    //  cout<<F.size()<<endl;
     cerr<<"here over!!!!"<<endl;

    //get every face's principal curvatures (In here I definate the face's prin-cur as mean of its three vertex)
    vector<vector<double> > W(faces.size()/3+1);
    vector<size_t> tmpfaces;
    for(int i = 0; i < faces.size() ; i++){
        
        tmpfaces.emplace_back(faces[i]);
        int in = i+1;
        if(in%3==0){
            int v1 = tmpfaces[0];
            int v2 = tmpfaces[1];
            int v3 = tmpfaces[2];
  
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
         //   cout<<"W: "<<W[i/3][0]<<' '<<W[i/3][1]<<' '<<W[i/3][2]<<endl;
            tmpfaces.clear();
        }
    }
    cout<<"Princal Curvature Calculate Done!"<<endl;


    //Build BVH Tree
    cout << "Building BVH from" << "face size: " << faces.size()/3 << endl;
    acc::BVHTree bvhtree(faces, vertices);
    cout<<"done. " << endl;
    //Build Over
    

    //read MVE format scene and get views
    mve::Scene::Ptr scene;
    try{
        scene = mve::Scene::create(conf.in_scene);
    }catch (exception& e){
        cerr << "could not open scene" << endl;
    }
    vector<mve::View::Ptr> views;
    mve::Scene::ViewList const & aviews = scene->get_views();
    //read MVE over
    

    //Get the face and its "labels"! 
    vector<vector<size_t> >p(faces.size()/3+1, vector<size_t>(conf.labelnum+2, 0));
    map<color, int> mlabelindex;
    int num = 0;
    int labelindex = 0;
    for(mve::View::Ptr view : aviews){
        cerr<<"第 "<<num<<"个view"<<endl;
        mve::CameraInfo const & camera = view->get_camera();
        mve::View::ImageProxy const * proxy = view->get_image_proxy(conf.image);
        //cv::Mat img = cv::imread(view->get_directory()+'/'+image_name+".png");
        mve::ByteImage::Ptr image = view->get_byte_image(conf.image);
       //  mve::ByteImage::Ptr image = mve::ByteImage::create(view->get_byte_image(image_name));
    
        if(image == NULL){
            cerr<<conf.image<<" is not exsits"<<endl;
            continue;
        }

        //get the camera extrinic and intrinsic and get the projection matrix
        math::Vec3f origin;
        camera.fill_camera_pos(*origin);
        math::Matrix3f invproj;
        camera.fill_inverse_calibration(*invproj, proxy->width, proxy->height);
        math::Matrix3f c2w_rot;
        camera.fill_cam_to_world_rot(*c2w_rot);
       // map<color, int> colorindex;
       // int labelindex = 0;
        #pragma omp parallel for
        for(int y = 0; y < proxy->height; y++){
           if(y%4 == 0)
           continue;
            for(int x = 0; x < proxy->width; x++){
                if(x%4 == 0)
                continue;
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
                  color tcolor(r, g, b);

                  //待修改

                #pragma omp critical
                {
                   if(mlabelindex.find(tcolor) == mlabelindex.end())
                    mlabelindex[tcolor] = labelindex ++ ,cout<<"r:"<<r<<"g:"<<g<<"b:"<<b<<endl;
                 //  p[hit.idx].emplace_back(colorindex[tcolor]);
                   p[hit.idx][mlabelindex[tcolor]] ++;
                }
        

                //   if(r==128&&g==128&&b==0)
                //     p[hit.idx].emplace_back(0);

                //   else if(r==128&&g==0&&b==0)
                //     p[hit.idx].emplace_back(1);

                //   else if(r==0&&g==128&&b==0)
                //     p[hit.idx].emplace_back(2);

                //   else
                //     p[hit.idx].emplace_back(3);
                }

            }
           
        }
           ++num;
           //if(num == 200)
           //break;

    }
    cerr<<"labelindex: "<<labelindex<<endl;
    fstream objFile;
    if(access(conf.out_dir.c_str(), 0) == -1)
    mkdir(conf.out_dir.c_str(), MODE);

    try{
        objFile.open(conf.out_dir+"/output.obj", ios::out);
    }catch (exception& e){
        cerr << "could not create objfile" << endl;
    }
    //scout<<p[60960/3].size()<<endl;
    objFile<<"mtllib ./label.mtl"<<endl;
    for(int i = 0; i <= conf.labelnum; i++)
    {
        objFile<<"vt "<<0.5<<' '<<(i+1.0)/(conf.labelnum+1)<<endl;
    }

    for(auto v : vertices){
        objFile<<'v'<<' '<<v<<endl; // objFile<<"vt "<<0.5<<' '<<75.0/256<<endl;
        // objFile<<"vt "<<0.5<<' '<<125.0/256<<endl;
        // objFile<<"vt "<<0.5<<' '<<175.0/256<<endl;
        // objFile<<"vt "<<0.5<<' '<<225.0/256<<endl;
       // objFile<<f<<" ";
    
    }
    int cnt = 0;
    int backgound = 3;
    vector<vector<size_t> > faceslabel(faces.size()/3);
    for(auto f : faces){
        ++cnt;
        // if(cnt%3==0){
          
        //      //size_t t = max_num(p[(cnt-1)/3], conf.labelnum);
        //      if(p[(cnt-1)/3].size() == 0)
        //      p[(cnt-1)/3].push_back(3);
        //     // cout<<"here"<<endl;
        //      size_t t = p[(cnt-1)/3][0];
        //    //  cout<<(cnt-1)/3<<endl;
        //      faceslabel[(cnt-1)/3].push_back(t);

        // }
        if(cnt%3==0){
        //   if(p[(cnt-1)/3].size()>0){
        //    //  size_t t = max_num(p[(cnt-1)/3], labelnum);
        //      size_t t = p[(cnt-1)/3][0];
        //      faceslabel[(cnt-1)/3].push_back(t);
        //   }
        //   else{
        //      faceslabel[(cnt-1)/3].push_back(backgound);
        //   }
             faceslabel[(cnt-1)/3].emplace_back(max_num(p[(cnt-1)/3], conf.labelnum));
        }
    }
   // cout<<"here1"<<endl;
            // cout<<"2"<<endl;
           
        //      objFile<<"f "<<outF[0]+1<<'/'<<t<<' '<<outF[1]+1<<'/'<<t<<' '<<outF[2]+1<<'/'<<t<<endl;
        //  }ost = costV() + costD()
    double cost = Energy(conf.labelnum, Adj, faceslabel, W, p);
    int face_num = faces.size()/3;
    vector<vector<size_t> >tmplabel(faceslabel);
    vector<vector<size_t> >reslabel(faceslabel);

    cout<<"cost: "<<cost<<endl;

    int epoch = conf.max_epoch;
    while(epoch--){
        cout<<"epoch: "<<conf.max_epoch - epoch<<endl;
        int flag = 0;
        double costnow = cost;
      //  cout<<"cost: "<<cost<<endl;
        for(int i = 0; i <= conf.labelnum; i++){
       //     cout<<"here2"<<endl;
            double tmpcost = alpha_expansion(i, Adj, face_num, conf.labelnum, tmplabel, p, W, reslabel);
         //   cout<<"here3"<<endl;
          //  cout<<"tmpcost: "<<tmpcost<<endl;
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
       //待删
       faceslabel = tmplabel;
        if(!flag)
        break;
    }
    
    cnt = 0;
    vector<size_t> outF;
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