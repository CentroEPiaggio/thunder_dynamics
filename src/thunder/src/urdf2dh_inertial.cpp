#include "../library/urdf2dh_inertial.h"

namespace thunder_ns{
    
    void transformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body){
        Eigen::Vector3d OuGi; 
        Eigen::Vector3d OiGi;
        Eigen::Vector3d dist_i;
        Eigen::Matrix3d Riu = rpyRot(rpy_i);
        Eigen::Matrix3d Rub = rpyRot(body_urdf.rpy);
        Eigen::Matrix3d Rib = Riu*Rub;

        Eigen::Matrix3d IGi_B = createI(body_urdf.parI);
        Eigen::Matrix3d IOi_i;
        Eigen::Matrix3d d_hat;

        OuGi << body_urdf.xyz[0], body_urdf.xyz[1],body_urdf.xyz[2]; 
        dist_i << d_i[0], d_i[1], d_i[2];
        OiGi = dist_i + Riu*OuGi;
        d_hat = hat(OiGi);

        IOi_i = Rib*IGi_B*Rib.transpose();

        body.mass = body_urdf.mass;
        body.xyz = {OiGi(0),OiGi(1),OiGi(2)};
        body.parI = {IOi_i(0,0),IOi_i(0,1),IOi_i(0,2),IOi_i(1,1),IOi_i(1,2),IOi_i(2,2)};
        body.name = body_urdf.name;
    }

    void mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody){
        Eigen::Vector3d G1Gnew;
        Eigen::Vector3d G2Gnew;
        Eigen::Vector3d O1G1;
        Eigen::Vector3d O2G2;
        Eigen::Vector3d newCoM;        
        Eigen::Matrix3d d_hat1;
        Eigen::Matrix3d d_hat2;
        Eigen::Matrix3d newI;
        Eigen::Matrix3d IG1 = createI(body1.parI);
        Eigen::Matrix3d IG2 = createI(body2.parI);
        
        O1G1 << body1.xyz[0], body1.xyz[1], body1.xyz[2]; 
        O2G2 << body2.xyz[0], body2.xyz[1], body2.xyz[2]; 
        
        newCoM = (body1.mass*O1G1 + body2.mass*O2G2)/(body1.mass + body2.mass);

        G1Gnew = newCoM-O1G1;
        G2Gnew = newCoM-O2G2;
        
        d_hat1 = hat(G1Gnew);
        d_hat2 = hat(G2Gnew);

        newI = IG1 + body1.mass*d_hat1*d_hat1.transpose() + IG2 + body2.mass*d_hat2*d_hat2.transpose();

        newBody.mass = body1.mass + body2.mass;
        newBody.xyz = {newCoM(0),newCoM(1),newCoM(2)};
        newBody.parI = {newI(0,0),newI(0,1),newI(0,2),newI(1,1),newI(1,2),newI(2,2)};
    }

    Eigen::Matrix3d hat(const Eigen::Vector3d v){
        Eigen::Matrix3d vhat;
                
        // chech
        if(v.size() != 3 ){
            std::cout<<"in function hat of class FrameOffset invalid dimension of input"<<std::endl;
        }
        
        vhat(0,0) = 0;
        vhat(0,1) = -v[2];
        vhat(0,2) = v[1];
        vhat(1,0) = v[2];
        vhat(1,1) = 0;
        vhat(1,2) = -v[0];
        vhat(2,0) = -v[1];
        vhat(2,1) = v[0];
        vhat(2,2) = 0;

        return vhat;
    }

    Eigen::Matrix3d rpyRot(const std::vector<double> rpy){
        Eigen::Matrix3d rotTr;
        
        double cy = cos(rpy[2]);
        double sy = sin(rpy[2]);
        double cp = cos(rpy[1]);
        double sp = sin(rpy[1]);
        double cr = cos(rpy[0]);
        double sr = sin(rpy[0]);

        //template R yaw-pitch-roll
        rotTr(0,0)=cy*cp;
        rotTr(0,1)=cy*sp*sr-sy*cr;
        rotTr(0,2)=cy*sp*cr-sy*sr;
        rotTr(1,0)=sy*cp;
        rotTr(1,1)=sy*sp*sr+cy*cr;
        rotTr(1,2)=sy*sp*cr-cy*sr;
        rotTr(2,0)=-sp;
        rotTr(2,1)=cp*sr;
        rotTr(2,2)=cp*cr;

        return rotTr;
    }
    
    Eigen::Matrix3d createI(const std::vector<double> parI){
        Eigen::Matrix3d I;
        I(0, 0) = parI[0];
        I(0, 1) = parI[1];
        I(0, 2) = parI[2];
        I(1, 0) = parI[1];
        I(1, 1) = parI[3];
        I(1, 2) = parI[4];
        I(2, 0) = parI[2];
        I(2, 1) = parI[4];
        I(2, 2) = parI[5];

        return I;
    }

}

