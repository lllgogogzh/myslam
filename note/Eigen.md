# Eigen基础

## 一、头文件以及功能

```c++
#include <Eigen/Dense> //包含了下面的Core/Geometry/LU/Cholesky/SVD/QR/Eigenvalues模块
#include <Eigen/Core> // 包含Matrix和Array类，基础的线性代数运算和数组操作
#include <Eigen/LU> // 包含求逆，行列式，LU分解
#include <Eigen/Geometry> //包含旋转，平移，缩放，2维和3维的各种变换。
#include <Eigen/Cholesky> //包含LLT和LDLT Cholesky分解
#include <Eigen/SVD> //包含SVD分解
#include <Eigen/QR> //包含QR分解
#include <Eigen/Eigenvalues> //包含特征值，特征向量分解

#include <Eigen/Sparse> //包含稀疏矩阵的存储和运算

#include <Eigen/Eigen> //包含Dense和Sparse

```

## 二、初始化

### 1、矩阵初始化

#### i. Matrix直接初始化

比较方便

```c++
Eigen::Matrix<double,2,2> m;//声明一个矩阵，元素类型为double ，维度2x2
m << 1,2,3,4;//初始化

Eigen::MatrixXf m1(2,3);//typedef Eigen::Matrix<float,-1,-1> Eigen::MatrixXf
m1 << 1,2,3,
	  4,5,6;
//声明一个矩阵A，元素类型为double，但是维度未知
//可以将任意维度矩阵赋值给A
//A没被赋值时候，不能使用
Eigen::Matrix<double,-1,-1> A;
A=m;//A现在是m
A=m1;//A现在是m1

//另一种初始化方式
//8x8的零矩阵
Eigen::Matrix<double,-1,-1> A(Eigen::Matrix<double,-1,-1>::Zero(8,8));

Eigen::Matrix3d m2 = Eigen::Matrix3d::Identity();//Eigen::Matrix3d::Zero();

Eigen::Matrix3d m3 = Eigen::Matrix3d::Random(); //随机初始化

Eigen::Vector3f v1 = Eigen::Vector3f::Zero();

Eigen::Vector3d v2(1.0, 2.0, 3.0);

Eigen::VectorXf v3(20); //维度为20的向量,未初始化.
v3 << 1.0 , 2.0 , 3.0;
```

### 2、向量初始化

#### i. VectorXd 初始化

```c++
int main()
{
    Eigen::VectorXd R;//当然也可以直接 R(3)，这么做是为了测试根据a的值赋值长度
    int a=3;
    R=Eigen::VectorXd(a);
    R(0)=1;
    R(1)=2;
    R(2)=3;
    cout<<R<<endl; 
    return 0;
}
```

### 3、其他

## 三、取矩阵部分函数block

```c++
M.block(1,1,2,2);
//M矩阵的第二行第二列开始，取两行两列的子块
//比如
//M=[1,2,3]
//  [4,5,6]
//  [7,8,9]
//则上述子矩阵操作取出：
//  [5,6]
//  [8,9]
```



## 四、























