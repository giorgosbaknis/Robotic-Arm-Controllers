#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <iostream>

/*
Model name: iiwa14
Model dimension of the configuration vector representation: 7
Model Dimension of the velocity vector space: 7
Model Number of joints: 8
Model Number of bodies: 8
Model Number of operational frames: 21
*/

int main()
{
    using namespace std;
    using namespace pinocchio;
    
    string urdf_filename = "/home/baknis/Documents/dartProject/iiwa.urdf";

    Model model;
    pinocchio::urdf::buildModel(urdf_filename,model);
    Data data(model);
    

    cout<<"Model name: "<<model.name<<endl;
    cout<<"Model dimension of the configuration vector representation: "<<model.nq<<endl;
    cout<<"Model Dimension of the velocity vector space: "<<model.nv<<endl;
    cout<<"Model Number of joints: "<<model.njoints<<endl;
    cout<<"Model Number of bodies: "<<model.nbodies<<endl;
    cout<<"Model Number of operational frames: "<<model.nframes<<endl;
    
    cout<<"Model of joint *i*, encapsulated in a JointModelAccessor"<<endl;
    for(int i=0; i<=7; i++)
    {
        cout<<endl; 
        cout<<"Joint Number "<<i<<endl;
        cout<<"Model of joint: "<<model.joints[i]<<endl;
        cout<<"Index of the joint (In the configuration space): "<<model.idx_qs[i]<<endl;
        cout<<"Dimension of the joint (In the conf subspace)"<<model.nqs[i]<<endl;
        cout<<"Starting index of the joint (In the tangent conf space) "<<model.idx_vs[i]<<endl;
        cout<<"Dimension of the joint (In the tangent subspace) "<<model.nvs[i]<<endl;
    }
    

    for(int i=0; i<=7; i++)
    {
        cout<<endl; 
        cout<<"Joint Number "<<i<<endl;
        cout<<"Joint Name: "<<model.names[i]<<endl; 
        cout<<"Data joint velocity: "<<data.a[i];
    }
    cout<<endl;
        

}