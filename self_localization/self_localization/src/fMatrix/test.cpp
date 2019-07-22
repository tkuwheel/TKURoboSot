#include <stdlib.h>
#include <iostream>
#include "fMatrix.h"
// #include "SVDdecomposition.h"

using namespace std;

extern fMatrix  Outer       (const fVector &, const fVector &);
extern fMatrix  Identity    ( int nSize );
extern fMatrix  Diag        ( const fVector & );
extern fMatrix  Diag        ( Float, Float, Float );
extern fMatrix  Cov         ( const fVector & );
void testVectorFuns()
{
    Float A[3] = {1.1, 2.2, 3.3};
    Float B[3] = {4.4, 5.5, 6.6};

    fVector VecA(A, 3);
    fVector VecB(B, 3);

    cout << "\nVecA = " << endl;
    VecA.Show();

    cout << "\nVecB = " << endl;
    VecB.Show();

    cout << "\nStarts to test vector operators..." << endl;
    // 1. A+B
    cout << "\n1. A+B" << endl;
    (VecA+VecB).Show();

    // 2. A-B
    cout << "\n2. A-B" << endl;
    (VecA-VecB).Show();

    // 3. -A
    cout << "\n3. -A" << endl;
    (-VecA).Show();

    // 4. A-2
    cout << "\n4. A-2" << endl;
    (VecA-2).Show();

    // 5. 2-A
    cout << "\n5. 2-A" << endl;
    (2-VecA).Show();

    // 6. A*2
    cout << "\n6. A*2" << endl;
    (VecA*2).Show();

    // 7. 0.5*B
    cout << "\n7. 0.5*B" << endl;
    (0.5*VecB).Show();

    // 8. A/4
    cout << "\n8. A/4" << endl;
    (VecA/4).Show();

    // 9. A/B
    cout << "\n9. A/B" << endl;
    (VecA/VecB).Show();

    // 10. A*B
    cout << "\n10. A*B" << endl;
    cout << "A*B = " << VecA*VecB << endl;

    // 11. A^B
    cout << "\n11. A^B" << endl;
    (VecA^VecB).Show(RowVec);

    // 12. A+=B
    cout << "\n12. A+=B" << endl;
    VecA += VecB;
    ShowVector(VecA, RowVec);
    (VecA).Show(ColVec);

    // 13. A-=B
    cout << "\n13. A-=B" << endl;
    (VecA -= VecB).Show(ColVec);

    // 14. A*=5
    cout << "\n14. A*=5" << endl;
    (VecA*=5).Show();

    // 15. A/=5
    cout << "\n15. A/=5" << endl;
    (VecA/=5).Show();

    cout << "\nStarts to test vector functions..." << endl;

    // 16. Min(A,B)
    cout << "\n16. Min(A,B)" << endl;
    (Min(VecA,VecB)).Show();

    // 17. Max(A,B)
    cout << "\n17. Max(A,B)" << endl;
    (Max(VecA,VecB)).Show();

    // 18. Dist(A,B)
    cout << "\n18. Dist(A,B)" << endl;
    cout << "Dist(A,B) = " << Dist(VecA,VecB) << endl; 

    // 19. Normalize(A,B)
    cout << "\n19. Normalize(A)" << endl;
    (Normalize(VecA)).Show();

    // 20. OneNorm(A)
    cout << "\n20. OneNorm(A)" << endl;
    cout << "OneNorm(A) = " << OneNorm(VecA) << endl; 

    // 21. TwoNorm(A)
    cout << "\n21. TwoNorm(A)" << endl;
    cout << "TwoNorm(A) = " << TwoNorm(VecA) << endl; 

    // 22. TwoNormSqr(A)
    cout << "\n22. TwoNormSqr(A)" << endl;
    cout << "TwoNormSqr(A) = " << TwoNormSqr(VecA) << endl; 

    // 23. Sqrt(A,B)
    cout << "\n23. Sqrt(A)" << endl;
    //VecA.Show();
    (Sqrt(VecA)).Show();

    // 24. Mean(A)
    cout << "\n24. Mean(A)" << endl;
    cout << "Mean(A) = " << Mean(VecA) << endl; 

    // 25. Var(A)
    cout << "\n25. Var(A)" << endl;
    cout << "Var(A) = " << Var(VecA) << endl; 

    // 26. Std(A)
    cout << "\n26. Std(A)" << endl;
    cout << "Std(A) = " << Std(VecA) << endl; 
}

void testMatrixFuns()
{
    Float A[3] = {1.1, 2.2, 3.3};
    Float B[3] = {4.4, 5.5, 6.6};
    Float C[9] = {0.9649, 0.9572, 0.1419, 
                  0.1576, 0.4854, 0.4218, 
                  0.9706, 0.8003, 0.9157};
    Float D[9] = {0.8147, 0.9134, 0.2785, 
                  0.9058, 0.6324, 0.5469, 
                  0.1270, 0.0975, 0.9575};
    Float X[15] = {0.7922, 0.9340, 0.6555, 
                   0.9595, 0.6787, 0.1712, 
                   0.6557, 0.7577, 0.7060, 
                   0.0357, 0.7431, 0.0318, 
                   0.8491, 0.3922, 0.2769};
    fVector VecA(A, 3);
    fVector VecB(B, 3);
    fVector VecC(VecA);
    fMatrix MatA(C, 3, 3);
    fMatrix MatB(D, 3, 3);
    fMatrix MatC(MatA);
    fMatrix MatX(X, 5, 3);
    fMatrix MatXt = Transp(MatX);

    cout << "\nMatA = " << endl;
    MatA.Show();

    cout << "\nMatB = " << endl;
    MatB.Show();

    cout << "\nMatX = " << endl;
    MatX.Show();

    cout << "\nStarts to test matrix operators..." << endl;
    // 1. A+B
    cout << "\n1. A+B" << endl;
    (MatA+MatB).Show();

    // 2. A-B
    cout << "\n2. A-B" << endl;
    (MatA-MatB).Show();

    // 3. -B
    cout << "\n3. -B" << endl;
    (-MatB).Show();

    // 4. 2*A
    cout << "\n4. 2*A" << endl;
    (2*MatA).Show();

    // 5. A*0.5
    cout << "\n5. A*0.5" << endl;
    (MatA*0.5).Show();

    // 6. A/2
    cout << "\n6. A/2" << endl;
    (MatA/2).Show();

    // 7. A*B
    cout << "\n7. A*B" << endl;
    (MatA*MatB).Show();

    // 8. MatA*VecA
    cout << "\n8. MatA*VecA" << endl;
    (MatA*VecA).Show(RowVec);

    // 9. VecA*MatA
    cout << "\n9. VecA*MatA" << endl;
    (VecA*MatA).Show();

    // 10. A+=B
    cout << "\n10. A+=B" << endl;
    (MatA+=MatB).Show();

    // 11. A-=B
    cout << "\n11. A-=B" << endl;
    (MatA-=MatB).Show();

    // 12. A*=2
    cout << "\n12. A*=2" << endl;
    (MatA*=2).Show();

    // 13. A/=2
    cout << "\n13. A/=2" << endl;
    (MatA/=2).Show();

    // 14. A*=B
    cout << "\n14. A*=B" << endl;
    (MatA*=MatB).Show();
    MatA = MatC;

    // 15. VecA*=MatA
    cout << "\n15. VecA*=MatA" << endl;
    (VecA*=MatA).Show(ColVec);
    VecA = VecC;

    cout << "\nStarts to test matrix functions..." << endl;

    // 16. Xt
    cout << "\n16. Xt" << endl;
    Transp(MatX).Show();

    // 17. X*Xt
    cout << "\n17. X*Xt" << endl;
    AATransp(MatX).Show();

    // 18. Xt*X
    cout << "\n18. Xt*X" << endl;
    ATranspA(MatX).Show();

    // 19, VecA*VecBt
    cout << "\n19. VecA*VecBt" << endl;
    //fMatrix tmp = Outer(VecA, VecB);
    (Outer(VecA, VecB)).Show();

    // 20, Identity(4)
    cout << "\n20. Identity(4)" << endl;
    (Identity(4)).Show();

    // 21, Diag(VecA)
    cout << "\n21. Diag(VecA)" << endl;
    (Diag(VecA)).Show();

    // 22, Diag(MatA)
    cout << "\n22. Diag(MatA)" << endl;
    (Diag(MatA)).Show(RowVec);

    // 23, Diag(0.1, 0.2, 0.3)
    cout << "\n23. Diag(0.1, 0.2, 0.3)" << endl;
    (Diag(0.1, 0.2, 0.3)).Show();

    // 24. Determinant(MatA)
    cout << "\n24. Determinant(MatA) = " << Determinant(MatA) << endl; 

    // 25. Trace(MatA)
    cout << "\n25. Trace(MatA) = " << Trace(MatA) << endl; 

    // 26. OneNorm(MatA)
    cout << "\n26. OneNorm(MatA) = " << OneNorm(MatA) << endl; 

    // 27. InfNorm(MatA)
    cout << "\n27. InfNorm(MatA) = " << InfNorm(MatA) << endl; 

    // 28. Inverse(MatA)
    cout << "\n28. Inverse(MatA) = " << endl; 
    (Inverse(MatA)).Show();

    // 29. Cholesky(AATransp(MatA))
    cout << "\n29. Cholesky(AATransp(MatA)) = " << endl;
    (Cholesky(AATransp(MatA))).Show();
    // 30. Mean(MatA)
    cout << "\n30. Mean(MatA) = " << endl; 
    (Mean(MatA)).Show();

    // 31. Cov(MatA)
    cout << "\n31. Cov(MatA) = " << endl; 
    (Cov(MatA)).Show();

    // 32. Cov(VecA)
    cout << "\n32. Cov(VecA) = " << endl; 
    (Cov(VecA)).Show();
    //#ifdef TEST
    cout << "\n33. SVDdcmp of MatX" << endl;
    // fMatrix MatU(MatX);
    // fVector VecD(MatX.Cols());
    // fMatrix MatV(MatX.Cols(), MatX.Cols());
    // SVDcmp(MatU, VecD, MatV);
    // cout << "\nMatU" << endl; MatU.Show();
    // cout << "\nMatD" << endl; Diag(VecD).Show();
    // cout << "\nMatV" << endl; MatV.Show();
    /*Float M[20] = { 1, 0, 0, 0, 2,
                      0, 0, 3, 0, 0,
                      0, 0, 0, 0, 0,
                      0, 4, 0, 0, 0};*/
    Float M2[6] = { 3, 2,  2,
                    2, 3, -2 };
    fMatrix MatM(MatX);
    fMatrix MatAU( MatX.Rows(), MatX.Rows());
    fMatrix MatW( MatX.Rows(), MatX.Cols());
    fMatrix MatV( MatX.Cols(), MatX.Cols());
    SVDcmp(MatM, MatAU, MatW, MatV);
    cout << "\nMatM" << endl; MatM.Show();
    cout << "\nMatAU" << endl; MatAU.Show();
    cout << "\nMatW" << endl; MatW.Show();
    cout << "\nMatV" << endl; MatV.Show();
    cout << "\nMatVT" << endl; Transp(MatV).Show();
    //#endif

}

int main(int argc, char* argv[])
{
    //testVectorFuns();
    //system("pause");

    testMatrixFuns();
    system("pause");

    return 0;
}

