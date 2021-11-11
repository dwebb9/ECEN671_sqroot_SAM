#include <iostream>
#inlcude <Eigen/Cholesky>

void qRDecomposition(Eigen::MatrixXd &A,Eigen::MatrixXd &Q, Eigen::MatrixXd &R)
{
    /*
        A=QR
        Q: is orthogonal matrix-> columns of Q are orthonormal
        R: is upper triangulate matrix

        this is possible when columns of A are linearly indipendent

    */

    Eigen::MatrixXd thinQ(A.rows(),A.cols() ), q(A.rows(),A.rows());

    Eigen::HouseholderQR<Eigen::MatrixXd> householderQR(A);
    q = householderQR.householderQ();
    thinQ.setIdentity();
    Q = householderQR.householderQ() * thinQ;
    R=Q.transpose()*A;
}

void qRExample()
{

    Eigen::MatrixXd A;
    A.setRandom(3,4);

    std::cout<<"A" <<std::endl;
    std::cout<<A <<std::endl;
    Eigen::MatrixXd Q(A.rows(),A.rows());
    Eigen::MatrixXd R(A.rows(),A.cols());

    /////////////////////////////////HouseholderQR////////////////////////
    Eigen::MatrixXd thinQ(A.rows(),A.cols() ), q(A.rows(),A.rows());

    Eigen::HouseholderQR<Eigen::MatrixXd> householderQR(A);
    q = householderQR.householderQ();
    thinQ.setIdentity();
    Q = householderQR.householderQ() * thinQ;

    std::cout << "HouseholderQR" <<std::endl;

    std::cout << "Q" <<std::endl;
    std::cout << Q <<std::endl;

    R = householderQR.matrixQR().template  triangularView<Eigen::Upper>();
    std::cout << R<<std::endl;
    std::cout << R.rows()<<std::endl;
    std::cout << R.cols()<<std::endl;


    R=Q.transpose()*A;
// 	std::cout << "R" <<std::endl;
// 	std::cout << R<<std::endl;

    std::cout << "A-Q*R" <<std::endl;
    std::cout << A-Q*R <<std::endl;

    /////////////////////////////////ColPivHouseholderQR////////////////////////
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> colPivHouseholderQR(A.rows(), A.cols());
    colPivHouseholderQR.compute(A);
    //R = colPivHouseholderQR.matrixR().template triangularView<Upper>();
    R = colPivHouseholderQR.matrixR();
    Q = colPivHouseholderQR.matrixQ();

    std::cout << "ColPivHouseholderQR" <<std::endl;

    std::cout << "Q" <<std::endl;
    std::cout << Q <<std::endl;

    std::cout << "R" <<std::endl;
    std::cout << R <<std::endl;

    std::cout << "A-Q*R" <<std::endl;
    std::cout << A-Q*R <<std::endl;

    /////////////////////////////////FullPivHouseholderQR////////////////////////
    std::cout << "FullPivHouseholderQR" <<std::endl;

    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> fullPivHouseholderQR(A.rows(), A.cols());
    fullPivHouseholderQR.compute(A);
    Q=fullPivHouseholderQR.matrixQ();
    R=fullPivHouseholderQR.matrixQR().template  triangularView<Eigen::Upper>();

    std::cout << "Q" <<std::endl;
    std::cout << Q <<std::endl;

    std::cout << "R" <<std::endl;
    std::cout << R <<std::endl;

    std::cout << "A-Q*R" <<std::endl;
    std::cout << A-Q*R <<std::endl;

}

void lDUDecomposition()
{
/*

    L: lower triangular matrix L
    U: upper triangular matrix U
    D: is a diagonal matrix
    A=LDU


*/
}

void lUDecomposition()
{
/*
    L: lower triangular matrix L
    U: upper triangular matrix U
    A=LU
*/
}

double exp(double x) // the functor we want to apply
{
    std::setprecision(5);
        return std::trunc(x);
}

void gramSchmidtOrthogonalization(Eigen::MatrixXd &matrix,Eigen::MatrixXd &orthonormalMatrix)
{
/*
    In this method you make every column perpendicular to it's previous columns,
    here if a and b are representation vector of two columns, c=b-((b.a)/|a|).a
        ^
       /
    b /
     /
    /
    ---------->
        a

        ^
       /|
    b / |
     /  | c
    /   |
    ---------->
        a

    you just have to normilze every vector after make it perpendicular to previous columns
    so:
    q1=a.normalized();
    q2=b-(b.q1).q1
    q2=q2.normalized();
    q3=c-(c.q1).q1 - (c.q2).q2
    q3=q3.normalized();


    Now we have Q, but we want A=QR so we just multiply both side by Q.transpose(), since Q is orthonormal, Q*Q.transpose() is I
    A=QR;
    Q.transpose()*A=R;
*/
    Eigen::VectorXd col;
    for(int i=0;i<matrix.cols();i++)
    {
        col=matrix.col(i);
        col=col.normalized();
        for(int j=0;j<i-1;j++)
        {
            //orthonormalMatrix.col(i)
        }

        orthonormalMatrix.col(i)=col;
    }
    Eigen::MatrixXd A(4,3);

    A<<1,2,3,-1,1,1,1,1,1,1,1,1;
    Eigen::Vector4d a=A.col(0);
    Eigen::Vector4d b=A.col(1);
    Eigen::Vector4d c=A.col(2);

    Eigen::Vector4d q1=  a.normalized();
    Eigen::Vector4d q2=b-(b.dot(q1))*q1;
    q2=q2.normalized();

    Eigen::Vector4d q3=c-(c.dot(q1))*q1 - (c.dot(q2))*q2;
    q3=q3.normalized();

    std::cout<< "q1:"<<std::endl;
    std::cout<< q1<<std::endl;
    std::cout<< "q2"<<std::endl;
    std::cout<< q2<<std::endl;
    std::cout<< "q3:"<<std::endl;
    std::cout<< q3<<std::endl;

    Eigen::MatrixXd Q(4,3);
    Q.col(0)=q1;
    Q.col(1)=q2;
    Q.col(2)=q3;

    Eigen::MatrixXd R(3,3);
    R=Q.transpose()*(A);


    std::cout<<"Q"<<std::endl;
    std::cout<< Q<<std::endl;


    std::cout<<"R"<<std::endl;
    std::cout<< R.unaryExpr(std::ptr_fun(exp))<<std::endl;



    //MatrixXd A(4,3), thinQ(4,3), Q(4,4);

    Eigen::MatrixXd thinQ(4,3), q(4,4);

    //A.setRandom();
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(A);
    q = qr.householderQ();
    thinQ.setIdentity();
    thinQ = qr.householderQ() * thinQ;
    std::cout << "Q computed by Eigen" << "\n\n" << thinQ << "\n\n";
    std::cout << q << "\n\n" << thinQ << "\n\n";


}

void gramSchmidtOrthogonalizationExample()
{
    Eigen::MatrixXd matrix(3,4),orthonormalMatrix(3,4) ;
    matrix=Eigen::MatrixXd::Random(3,4);////A.setRandom();


    gramSchmidtOrthogonalization(matrix,orthonormalMatrix);
}

void choleskyDecompositionExample()
{
/*
Positive-definite matrix:
    Matrix Mnxn is said to be positive definite if the scalar zTMz is strictly positive for every non-zero
    column vector z n real numbers. zTMz>0


Hermitian matrix:
    Matrix Mnxn is said to be positive definite if the scalar z*Mz is strictly positive for every non-zero
    column vector z n real numbers. z*Mz>0
    z* is the conjugate transpose of z.

Positive semi-definite same as above except zTMz>=0 or  z*Mz>=0


    Example:
            ┌2  -1  0┐
        M=	|-1  2 -1|
            |0 - 1  2|
            └        ┘
          ┌ a ┐
        z=| b |
          | c |
          └	  ┘
        zTMz=a^2 +c^2+ (a-b)^2+ (b-c)^2

Cholesky decomposition:
Cholesky decomposition of a Hermitian positive-definite matrix A is:
    A=LL*

    L is a lower triangular matrix with real and positive diagonal entries
    L* is the conjugate transpose of L
*/

    Eigen::MatrixXd A(3,3);
    A << 6, 0, 0, 0, 4, 0, 0, 0, 7;
    Eigen::MatrixXd L( A.llt().matrixL() );
    Eigen::MatrixXd L_T=L.adjoint();//conjugate transpose

    std::cout << "L" << std::endl;
    std::cout << L << std::endl;
    std::cout << "L_T" << std::endl;
    std::cout << L_T << std::endl;
    std::cout << "A" << std::endl;
    std::cout << A << std::endl;
    std::cout << "L*L_T" << std::endl;
    std::cout << L*L_T << std::endl;

}