#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include <iostream>

using namespace std;

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};

void FeatureMatch(const cv::Mat &refImg,const cv::Mat &curImg,vector<cv::KeyPoint> &refKeyPoint,
                                            vector<cv::KeyPoint> &curKeyPoint,vector<cv::DMatch> &GoodMatch);
void EstimationP3P(vector<cv::Point3f> refPoint,vector<cv::Point3f> curPoint,cv::Mat &R , cv::Mat &t);

void EstimationOptimization(vector<cv::Point3f> pts1,vector<cv::Point3f> pts2,cv::Mat &R,cv::Mat &t);

void useLK(cv::Mat img,cv::Mat lastImg,cv::Mat lastDepthImg);

cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    return cv::Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


int main(int argc , char** argv)
{
    cv::Mat refImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/1.png" , CV_LOAD_IMAGE_COLOR);
    cv::Mat curImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/2.png" , CV_LOAD_IMAGE_COLOR);
    vector<cv::KeyPoint> refKeyPoint;
    vector<cv::KeyPoint> curKeyPoint;
    vector<cv::DMatch> GoodMatch;
    FeatureMatch(refImg,curImg,refKeyPoint,curKeyPoint,GoodMatch);

    cv::Mat refDepthImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/1_depth.png" , CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat curDepthImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/2_depth.png" , CV_LOAD_IMAGE_UNCHANGED);

    cv::imshow("asdf",refDepthImg);

    vector<cv::Point3f> refPoint,curPoint;
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( cv::DMatch m:GoodMatch )
    {
        //ushort d1 = refDepthImg.ptr<unsigned short> ( int ( refKeyPoint[m.queryIdx].pt.y ) ) [ int ( refKeyPoint[m.queryIdx].pt.x ) ];
        //ushort d2 = curDepthImg.ptr<unsigned short> ( int ( curKeyPoint[m.trainIdx].pt.y ) ) [ int ( curKeyPoint[m.trainIdx].pt.x ) ];
        ushort d1 = refDepthImg.at<unsigned short>(int(refKeyPoint[m.queryIdx].pt.y),int ( refKeyPoint[m.queryIdx].pt.x ));
        ushort d2 = curDepthImg.at<unsigned short>(int(curKeyPoint[m.trainIdx].pt.y),int ( curKeyPoint[m.trainIdx].pt.x ));
        //cout<<d1<<","<<d11<<endl<<d2<<","<<d22<<endl;
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        cv::Point2d p1 = pixel2cam ( refKeyPoint[m.queryIdx].pt, K );
        cv::Point2d p2 = pixel2cam ( curKeyPoint[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /5000.0;
        float dd2 = float ( d2 ) /5000.0;
        refPoint.push_back ( cv::Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        curPoint.push_back ( cv::Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }

    cv::Mat R,t;
    EstimationP3P(refPoint,curPoint,R,t);
    cout<<R<<endl<<t<<endl;
    cv::waitKey(0);
    EstimationOptimization(refPoint,curPoint,R,t);

    cout<<endl<<"test"<<R.at<double>(0,0)<<endl;
    cout<<endl<<"test"<<R.at<int>(0,0)<<endl;

    //LK
    useLK(curImg,refImg,refDepthImg);

    //2D-2D
    

    return 0;
}

void EstimationP3P(vector<cv::Point3f> refPoint,vector<cv::Point3f> curPoint,cv::Mat &R , cv::Mat &t)
{
    //ji suan zhi xin  he  qu zhi xin zuo biao 
    cv::Point3f pm1,pm2;
    int  N = refPoint.size();
    for(int i=0;i<refPoint.size();i++)
    {
        pm1 += refPoint[i];
        pm2 += curPoint[i];
    }
    pm1 = cv::Point3f( cv::Vec3f(pm1) / N);
    pm2 = cv::Point3f( cv::Vec3f(pm2) / N);

    vector<cv::Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for(int i=0;i<N;i++)
    {
        q1[i] =refPoint[i] - pm1;
        q2[i] = curPoint[i]-pm2;
    }
    cout<<"?"<<endl;
    
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i=0;i<N;i++)
    {
        W += Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z) * Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    if (U.determinant() * V.determinant() < 0)
	{
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	}
    Eigen::Matrix3d R_ = U*V.transpose();
    Eigen::Vector3d t_ = Eigen::Vector3d(pm1.x,pm1.y,pm1.z) - R_*Eigen::Vector3d(pm2.x,pm2.y,pm2.z);

    R = ( cv::Mat_<double>(3,3) <<
            R_(0,0), R_(0,1), R_(0,2),
            R_(1,0), R_(1,1), R_(1,2),
            R_(2,0), R_(2,1), R_(2,2)
            );
    t = ( cv::Mat_<double>(3,1) << t_(0,0), t_(1,0), t_(2,0) );
}

void FeatureMatch(const cv::Mat &refImg,const cv::Mat &curImg,vector<cv::KeyPoint> &refKeyPoint,
                                            vector<cv::KeyPoint> &curKeyPoint,vector<cv::DMatch> &GoodMatch)
{
    cv::Mat descriptors_1, descriptors_2;
    // FAST 
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    //BRIEF
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    //Match
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    

    detector->detect(refImg,refKeyPoint);
    detector->detect(curImg,curKeyPoint);

    cv::Mat refDescriptions,curDescriptions;
    descriptor->compute(refImg,refKeyPoint,refDescriptions);
    descriptor->compute(curImg,curKeyPoint,curDescriptions);

    vector<cv::DMatch> matches;
    matcher->match(refDescriptions,curDescriptions,matches);

    //matches filters
    double max_dist = 0.0, min_dist = 100000.0;

    for(int i=0;i<matches.size();i++)
    {
        if(matches[i].distance<min_dist)
            min_dist = matches[i].distance;
        if(matches[i].distance>max_dist)
            max_dist = matches[i].distance;
    }

    for(int i=0;i<matches.size();i++)
    {
        if(matches[i].distance<=cv::max(2*min_dist,30.0))
        {
            GoodMatch.push_back(matches[i]);
        }
    }
    /*cv::Mat img_match;
    cv::Mat img_goodmatch;
    cv::drawMatches ( refImg, refKeyPoint, curImg, curKeyPoint, matches, img_match );
    cv::drawMatches ( refImg, refKeyPoint, curImg, curKeyPoint,GoodMatch, img_goodmatch );
    cv::imshow ( "所有匹配点对", img_match );
    cv::imshow ( "优化后匹配点对", img_goodmatch );
    cv::waitKey(0);*/
}

void EstimationOptimization(vector<cv::Point3f> pts1,vector<cv::Point3f> pts2,cv::Mat &R,cv::Mat &t)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
    //std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverD);
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>() );
    std::unique_ptr<Block> solver_ptr (new Block( std::move(linearSolver) ));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
    g2o::SparseOptimizer optimizer;   
    optimizer.setAlgorithm( solver );   
    optimizer.setVerbose( true ); 

    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0)));
    pose->setId(0);
    optimizer.addVertex(pose);

    for(int i=0;i<pts1.size();i++)
    {
        EdgeProjectXYZRGBDPoseOnly *pPoint = new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts2[i].x,pts2[i].y,pts2[i].z));
        pPoint->setId(i);
        pPoint->setVertex(0,pose);
        pPoint->setMeasurement(Eigen::Vector3d(pts1[i].x,pts1[i].y,pts1[i].z));
        pPoint->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(pPoint);
    }

    optimizer.initializeOptimization();//初始化
    optimizer.optimize(100);//迭代次数
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;
}

void useLK(cv::Mat img,cv::Mat lastImg,cv::Mat lastDepthImg)
{
    //detect FAST
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

    vector<cv::KeyPoint> lastKeyPoints;
    detector->detect(lastImg,lastKeyPoints);

    //use LK
    list<cv::Point2f> keyPoints;//last image KeyPoints in list form
    list<cv::Point2f> lPoints;
    vector<cv::Point2f> Points,lastPoints;
    vector<unsigned char> status;
    vector<float> error;
    //cv::KepPoint -> cv::Point2f
    for(auto k:lastKeyPoints)
        lastPoints.push_back(k.pt);
    //vector -> list
    for(auto k:lastKeyPoints)
        keyPoints.push_back(k.pt);

    //LK
    cv::calcOpticalFlowPyrLK(lastImg,img,lastPoints,Points,status,error);
    //draw
    cv::Mat temp=lastImg.clone();
    cv::Mat temp1 = img.clone();
    for(auto p:lastPoints)
        cv::circle(temp, p, 10, cv::Scalar(0, 240, 0), 1);
    for(auto p:Points)
        cv::circle(temp1, p, 10, cv::Scalar(0, 240, 0), 1);
    cv::imshow("LK",temp);
    cv::imshow("LK1",temp1);
    cv::waitKey(0);
    cout<<"after LK lastPoints"<<lastPoints.size()<<endl;
    cout<<"after LK Points"<<Points.size()<<endl;
    //vector -> list
    for(auto p:Points)
        lPoints.push_back(p);


    //PnP RANSAC
    //cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );

    //delete unTracking Points in lastImg
    //list is convinent for delete or insert
    int i=0;
    for(auto iter = keyPoints.begin();iter != keyPoints.end();++i)
    {
        if(status[i]==0)
        {
            //delete
            iter = keyPoints.erase(iter);
            continue;
        }
        ++iter;
    }

    i=0;
    for(auto iter = lPoints.begin();iter != lPoints.end();++i)
    {
        if(status[i]==0)
        {
            //delete
            iter = lPoints.erase(iter);
            continue;
        }
        ++iter;
    }
    cout<<"after delete lPoints"<<lPoints.size()<<endl;
    //turn KeyPoints to 3DPoints
    //first turn KeyPoints to vec
    lastPoints.clear();
    for(auto iter = keyPoints.begin();iter != keyPoints.end();++iter)
    {
        lastPoints.push_back(*iter);
    }
    keyPoints.clear();

    /*Points.clear();
    for(auto iter = lPoints.begin();iter != lPoints.end();++iter)
    {
        Points.push_back(*iter);
    }
    cout<<"after delete lPoints"<<lPoints.size()<<endl;
    lPoints.clear();*/

    //turn lastPoints to 3Dpoints
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<cv::Point3f> lastMapPoints;
    vector<cv::Point2f> uvPoints; 
    for(int i=0;i<Points.size();i++)
    {
        cv::Point2f tempP;
        tempP = pixel2cam(lastPoints[i],K);
        ushort d1 = lastDepthImg.at<unsigned short>(int(lastPoints[i].y),int ( lastPoints[i].x ));
        if(d1 == 0)
        {
            continue;
        }
        float dd1;
        dd1 = float(d1)/5000.0;
        lastMapPoints.push_back(cv::Point3f(tempP.x*dd1,tempP.y*dd1,dd1));
        uvPoints.push_back(Points[i]);
    }

    cout<<"before PnP lastMapPoints"<<lastMapPoints.size()<<endl;
    cout<<"before PnP Points"<<uvPoints.size()<<endl;

    cv::Mat rvec, tvec;
    vector<int> inliers;
    cv::solvePnPRansac(lastMapPoints,uvPoints,K,cv::Mat(),rvec,tvec,false,100,4.0,0.99,inliers,0);

    for(auto i:inliers)
    {
        cout<<i<<endl;
    }
    cout<<"size"<<inliers.size()<<endl;
    //rvec -> R
    cv::Mat R;
    cv::Rodrigues(rvec,R);
    cout<<"R:"<<R<<endl;
    cout<<"t:"<<tvec<<endl;
}












