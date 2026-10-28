// Same sampled turn/forward geometry as planner.py, in float64.
// One candidate owns its complete visibility/bisection calculation.
#define INF __longlong_as_double(0x7ff0000000000000LL)
#define D2R 0.017453292519943295
#define R2D 57.29577951308232

__device__ void boundary(double v, int* ambiguous) {
    if (fabs(v) <= 1e-9) *ambiguous = 1;
}

__device__ double angular(double x, double z, double yaw, double turn,
                         double forward, double half, int* ambiguous) {
    int nt=max(1,(int)ceil(fabs(turn)/SAMPLE_STEP));
    int nf=max(1,(int)ceil(fabs(forward)/0.05));
    double dx=HALF_WIDTH*cos(yaw*D2R), dz=-HALF_WIDTH*sin(yaw*D2R);
    double margin=INF;
    for (int j=0;j<nt+1+nf;j++) {
        double t=(j<=nt?turn*j/nt:turn)*D2R;
        double f=j<=nt?0.0:forward*(j-nt)/nf;
        double c=cos(t),s=sin(t);
        for(int k=0;k<2;k++) {
            double q=k==0?-1.0:1.0;
            double px=x+q*dx-CENTER_X,pz=z+q*dz-CENTER_Z;
            double xx=c*px-s*pz+CENTER_X,zz=s*px+c*pz+CENTER_Z-f;
            boundary(zz-MIN_DEPTH,ambiguous);
            if(zz<=MIN_DEPTH)return -INF;
            margin=fmin(margin,half-fabs(atan2(xx,zz)*R2D));
        }
    }
    boundary(margin,ambiguous);
    return margin;
}

__device__ double pixels(const double* r, const double* corners,
                        double x,double z,double yaw,double turn,double forward,
                        double edge,int* ambiguous) {
    int mode=(int)r[16];
    if(!mode)return INF;
    int nt=max(1,(int)ceil(fabs(turn)/SAMPLE_STEP));
    int nf=max(1,(int)ceil(fabs(forward)/0.05));
    double cy=cos(CAMERA_YAW*D2R),sy=sin(CAMERA_YAW*D2R);
    double dx=HALF_WIDTH*cos(yaw*D2R),dz=-HALF_WIDTH*sin(yaw*D2R);
    double margin=INF;
    for(int j=0;j<nt+1+nf;j++) {
        double t=(j<=nt?turn*j/nt:turn)*D2R;
        double f=j<=nt?0.0:forward*(j-nt)/nf;
        double c=cos(t),s=sin(t);
        for(int k=0;k<(mode==2?4:2);k++) {
            double vx,vz,y=0.0;
            if(mode==2) {
                vx=cy*corners[3*k]+sy*corners[3*k+2];
                vz=-sy*corners[3*k]+cy*corners[3*k+2];
                y=corners[3*k+1];
            }else {
                double q=k==0?-1.0:1.0;
                vx=x+q*dx;vz=z+q*dz;
            }
            double px=vx-CENTER_X,pz=vz-CENTER_Z;
            double xx=c*px-s*pz+CENTER_X,zz=s*px+c*pz+CENTER_Z-f;
            double xc=cy*xx-sy*zz,zc=sy*xx+cy*zz;
            boundary(zc-MIN_DEPTH,ambiguous);
            if(zc<=MIN_DEPTH)return -INF;
            double u=r[10]*xc/zc+r[12],left=edge*r[14];
            margin=fmin(margin,fmin((u-left)/r[14],(r[14]-left-u)/r[14]));
            if(mode==2) {
                double v=r[11]*y/zc+r[13],top=edge*r[15];
                margin=fmin(margin,fmin((v-top)/r[15],(r[15]-top-v)/r[15]));
            }
        }
    }
    boundary(margin,ambiguous);
    return margin;
}

__device__ bool visible(const double* r,const double* corners,
                        double x,double z,double yaw,double turn,double forward,
                        double half,double edge,int* ambiguous) {
    if(angular(x,z,yaw,turn,forward,half,ambiguous)<0.0)return false;
    return pixels(r,corners,x,z,yaw,turn,forward,edge,ambiguous)>=0.0;
}

extern "C" __global__ void evaluate(const double* data,double* out,int n,int kind) {
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=n)return;
    const double* r=data+64*i;
    double* answer=out+4*i;
    int ambiguous=0;
    if(kind==1) {
        answer[0]=angular(r[0],r[1],r[2],r[3],r[4],r[5],&ambiguous);
        answer[1]=ambiguous;
        return;
    }
    answer[0]=0.0;answer[1]=0.0;
    if(visible(r,r+17,r[0],r[1],r[2],r[3],0.0,r[5],r[6],&ambiguous)
       && visible(r,r+29,r[7],r[8],r[9],0.0,0.0,r[41],r[42],&ambiguous)) {
        answer[0]=1.0;
        double upper=r[4],low=0.0,high=upper;
        if(visible(r,r+29,r[7],r[8],r[9],0.0,upper,r[41],r[42],&ambiguous)) {
            low=upper;
        }else {
            for(int j=0;j<FORWARD_ITERATIONS;j++) {
                double middle=0.5*(low+high);
                if(visible(r,r+29,r[7],r[8],r[9],0.0,middle,r[41],r[42],&ambiguous))low=middle;
                else high=middle;
            }
        }
        answer[1]=low;
    }
    answer[2]=ambiguous;
}
