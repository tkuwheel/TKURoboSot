#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include "fMatrix.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/*-------------------------------------------------------------------------*
*                                                                         *
*  C O N S T R U C T O R S  & D E S T R U C T O R S                       *
*                                                                         *
*-------------------------------------------------------------------------*/
//extern fMatrix  transVec2Mat(const fVector &, VecType);
int fMatrix::nMatCount = 0;
fMatrix::fMatrix(int n_rows, int n_cols) :
    rows(n_rows),
    cols(n_cols),
    elem(new Float[n_rows*n_cols])
{
    for (int i = 0; i < rows*cols; i++) {
        elem[i] = 0;
    }
    nMatCount++;
}
fMatrix::fMatrix(Float *Array, int n_rows, int n_cols) :
    rows(n_rows),
    cols(n_cols),
    elem(new Float[n_rows*n_cols])
{
    for (int i = 0; i < n_rows*n_cols; i++) {
        elem[i] = Array[i];
    }
    nMatCount++;
}
fMatrix::fMatrix(int n_rows, int n_cols, Float *Array) :
    rows(n_rows),
    cols(n_cols),
    elem(new Float[n_rows*n_cols])
{
    for (int i = 0; i < n_rows*n_cols; i++) {
        elem[i] = Array[i];
    }
    nMatCount++;
}
fMatrix::fMatrix(const fMatrix &a) :
    rows(a.rows),
    cols(a.cols),
    elem(new Float[rows*cols])
{
    for (int i = 0; i < rows*cols; i++) {
        elem[i] = a.elem[i];
    }
    nMatCount++;
}
fMatrix::~fMatrix(void)
{
    delete[] elem;
    nMatCount--;
}

/*-------------------------------------------------------------------------*/
void fMatrix::SetCol(int col, const fVector &a)
{
    if (col < cols && col >= 0) {
        Float* fcol;
        fcol = a.Array();
        for (int i = 0; i < rows; i++) {
            elem[i*cols + col] = fcol[i];
            //elem[i*cols + col] = fcol[i*cols + col];
        }
    }
    else {
        printf("input col out of range");
        exit(1);
    }
}
void fMatrix::SetRow(int row, const fVector &a)
{
    if (row < rows && row >= 0) {
        Float* frow;
        frow = a.Array();
        for (int i = 0; i < cols; i++)
            elem[row*cols + i] = frow[i];
    }
    else {
        printf("input row out of range");
        exit(1);
    }
}
void fMatrix::SetBlock(int imin, int imax, int jmin, int jmax, const fMatrix &a)
{
    if (imin >= 0 && imax < rows || jmin >= 0 && jmax < cols) {
        int row_size = (imax - imin) + 1;
        int cols_size = (jmax - jmin) + 1;
        for (int i = imin; i <= imax; i++) {
            for (int j = jmin; j <= jmax; j++) {
                elem[i*cols + j] = a.elem[j - jmin + (i - imin)*a.cols];
            }
        }
    }
    else {
        printf("input out of range\n");
        exit(1);
    }
}
void fMatrix::SetSize(int rows, int cols)
{
    this->rows = rows;
    this->cols = cols;
}

fVector  fMatrix::GetCol(int col) const
{
    if (col < cols && col >= 0) {
        Float* fcol;
        fcol = new Float[rows];
        for (int i = 0; i < rows; i++) {
            fcol[i] = elem[i*cols + col];
        }
        return(fVector(rows, fcol));
    }
    else {
        printf("input out of range\n");
        exit(1);
    }

}
fVector  fMatrix::GetRow(int row) const
{
    if (row < rows && row >= 0) {
        Float* frow;
        frow = new Float[cols];
        for (int i = 0; i < cols; i++)
        {
            frow[i] = elem[cols*row + i];
        }
        return(fVector(cols, frow));
    }
    else {
        printf("input out of range\n");
        exit(1);
    }
}
fMatrix  fMatrix::GetBlock(int imin, int imax, int jmin, int jmax) const
{
    if (imin >= 0 && imax < rows || jmin >= 0 && jmax < cols) {
        int row_size = (imax - imin) + 1;
        int cols_size = (jmax - jmin) + 1;

        Float* elem_temp = new Float[row_size*cols_size];
        for (int i = imin; i <= imax; i++)
            for (int j = jmin; j <= jmax; j++)
                elem_temp[(i - imin)*cols_size + (j - jmin)] = elem[i*cols + j];
        return fMatrix(row_size, cols_size, elem_temp);
    }
    else {
        printf("input out of range\n");
        exit(1);
    }
}
void fMatrix::Show(void)const
{
    std::cout << " ┌ ";
    for (int i = 0; i < cols; i++) {
        std::cout << std::setw(9) << " ";
    }
    std::cout << "┐ " << std::endl;
    for (int i = 0; i < rows; i++) {
        std::cout << " │";
        for (int j = 0; j < cols; j++) {
            std::cout << std::fixed << std::setprecision(4) << std::setw(9) << elem[i*cols + j];
        }
        printf(" │\n");
    }
    printf(" └ ");
    for (int i = 0; i < cols; i++) {
        std::cout << std::setw(9) << " ";
    }
    printf("┘\n");
}
/*-------------------------------------------------------------------------*/
// 1. A+B
fMatrix  operator +  (const fMatrix &a, const fMatrix &b)
{
    if (a.rows == b.rows && a.cols == b.cols) {
        fMatrix dst(a.rows, a.cols);
        for (int i = 0; i < dst.rows; i++) {
            for (int j = 0; j < dst.cols; j++) {
                dst.elem[i*dst.cols + j] = a.elem[i*a.cols + j] + b.elem[i*b.cols + j];
            }
        }
        return dst;
    }
    else {
        printf("dimension error in matrix operator - !!\n");
        exit(1);
    }
}
// 2. -A
fMatrix  operator -  (const fMatrix & a)
{
    fMatrix dst(a.rows, a.cols);
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            dst.elem[i*dst.cols + j] = -a.elem[i*a.cols + j];
        }
    }
    return dst;
}
// A-B
fMatrix  operator -  (const fMatrix &a, const fMatrix &b)
{
    if (a.rows == b.rows && a.cols == b.cols) {
        fMatrix dst(a.rows, a.cols);
        for (int i = 0; i < dst.rows; i++) {
            for (int j = 0; j < dst.cols; j++) {
                dst.elem[i*dst.cols + j] = a.elem[i*a.cols + j] - b.elem[i*b.cols + j];
            }
        }
        return dst;
    }
    else {
        printf("dimension error in matrix operator - !!\n");
        exit(1);
    }
}
// 3. A*num
fMatrix  operator *  (const fMatrix &a, Float num)
{
    fMatrix dst(a.rows, a.cols);
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            dst.elem[i*dst.cols + j] = a.elem[i*a.cols + j] * num;
        }
    }
    return dst;
}
// num*A
fMatrix  operator *  (Float num, const fMatrix &a)
{
    fMatrix dst(a.rows, a.cols);
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            dst.elem[i*dst.cols + j] = a.elem[i*a.cols + j] * num;
        }
    }
    return dst;
}
// 4. A/num
fMatrix  operator /  (const fMatrix &a, Float num)
{
    fMatrix dst(a.rows, a.cols);
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            dst.elem[i*dst.cols + j] = a.elem[i*a.cols + j] / num;
        }
    }
    return dst;
}
// 5. A*B
fMatrix  operator *  (const fMatrix &a, const fMatrix &b)
{
    fMatrix dst(a.rows, b.cols);
    if (a.cols == b.rows) {
        for (int i = 0; i < dst.rows; i++) {
            for (int j = 0; j < dst.cols; j++) {
                dst.elem[i*dst.cols + j] = a.GetRow(i) * b.GetCol(j);
            }
        }
        return dst;
    }
    else {
        printf("dimension error in matrix operator *!!\n");
        exit(1);
    }
}
fVector  operator *  (const fMatrix &matrix, const fVector &vector)
{
    if (matrix.rows == vector.Size()) {
        fVector dst(matrix.rows);
        for (int i = 0; i < matrix.rows; i++) {
            dst(i) = matrix.GetRow(i) * vector;
        }
        return dst;
    }
    else {
        printf("dimension error in matrix operator *!!\n");
        exit(1);
    }
}
fVector  operator *  (const fVector &vector, const fMatrix &matrix)
{

    if (matrix.cols == vector.Size()) {
        fVector dst(matrix.rows);
        for(int i=0;i<matrix.rows;i++){
            dst(i) = matrix.GetCol(i) * vector;
        }
        return dst;
    }
    else {
        printf("dimension error in matrix operator *!!\n");
        exit(1);
    }
}
/*-------------------------------------------------------------------------*/

fMatrix& operator += (fMatrix &a, const fMatrix &b)
{
    if (a.rows == b.rows && a.cols == b.cols)
    {
        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.cols; j++) {
                a.elem[i*a.cols + j] += b.elem[i*b.cols + j];
            }
        }
        return a;
    }
    else {
        printf("dimension error in matrix operator += !!\n");
        exit(1);
    }
}
fMatrix& operator -= (fMatrix &a, const fMatrix &b)
{
    if (a.rows == b.rows && a.cols == b.cols) {
        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.cols; j++) {
                a.elem[i*a.cols + j] -= b.elem[i*b.cols + j];
            }
        }
        return a;
    }
    else {
        printf("dimension error in matrix operator -= !!\n");
        exit(1);
    }
}
fMatrix& operator *= (fMatrix &a, Float num)
{
    for (int i = 0; i < a.rows; i++) {
        for (int j = 0; j < a.cols; j++) {
            a.elem[i*a.cols + j] *= num;
        }
    }
    return a;
}
fMatrix& operator *= (fMatrix &a, const fMatrix &b)
{
    if (a.cols == b.rows) {
        fMatrix dst(a.rows, b.cols);
        for (int i = 0; i < dst.rows; i++) {
            for (int j = 0; j < dst.cols; j++) {
                dst.elem[i*a.cols + j] = a.GetRow(i) * b.GetCol(j);
            }
        }
        a = dst;
        return a;
    }
    else {
        printf("dimension error in matrix operator *!!\n");
        exit(1);
    }
}
fVector& operator *= (fVector &vector, const fMatrix &matrix)
{
    if (matrix.rows == vector.Size()) {
        fVector dst(matrix.cols);
        for (int i = 0; i < matrix.cols; i++) {
            dst(i) = vector * matrix.GetCol(i);
        }
        vector = dst;
        return vector;
    }
    else {
        printf("dimension error in matrix operator *!!\n");
        exit(1);
    }
}
fMatrix& operator /= (fMatrix &matrix1, Float num)
{
    fMatrix dst(matrix1.rows, matrix1.cols);
    for (int i = 0; i < dst.rows; i++) {
        for (int j = 0; j < dst.cols; j++) {
            dst.elem[i*dst.cols + j] = matrix1.elem[i*dst.cols + j] / num;
        }
    }
    matrix1 = dst;
    return matrix1;
}
/*-------------------------------------------------------------------------*/
fMatrix& fMatrix::operator=(const fMatrix &a)
{
    delete[] elem;
    cols = a.cols;
    rows = a.rows;
    elem = new Float[cols*rows];
    for (int i = 0; i < cols*rows; i++) {
        elem[i] = a.elem[i];
    }
    return *this;
}
fMatrix& fMatrix::operator=(Float num)
{
    for (int i = 0; i < cols*rows; i++) {
        elem[i] = num;
    }
    return *this;
}
/*-------------------------------------------------------------------------*/
// 7. Swap
fMatrix& fMatrix::SwapRows(int i1, int i2)
{
    if (i1 < rows || i2 < rows) {
        fVector tmp(GetRow(i1));
        SetRow(i1, GetRow(i2));
        SetRow(i2, tmp);
        return *this;
    }
    else {
        printf("input out of range");
        exit(1);
    }
}
fMatrix& fMatrix::SwapCols(int j1, int j2)
{
    //printf("j1 = %d, j2 = %d, cols = %d\n", j1, j2, cols);
    if (j1 <= cols && j2 <= cols) {
        fVector tmp(GetCol(j1));
        SetCol(j1, GetCol(j2));
        SetCol(j2, tmp);
        return *this;
    }
    else {
        printf("input out of range");
        exit(1);
    }
}
// 8. Inverse
fMatrix& fMatrix::Inv(void)
{
    if (rows == cols) {
        int n = rows;
        //單位矩陣
        fMatrix dst(n, n);
        for (int i = 0; i < n; i++) {
            dst.elem[i * dst.cols + i] = 1.0;
        }
        //建立上三角矩陣
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                if (elem[i * cols + i] == 0) {
                    printf("The inverse matrix does not exist1\n");
                    system("pause");
                    exit(1);
                }
                else {
                    double m = elem[j * cols + i] / elem[i * cols + i];
                    for (int k = 0; k < n; k++) {
                        elem[j * cols + k] -= m * elem[i * cols + k];
                        dst.elem[j * dst.cols + k] -= m * dst.elem[i * dst.cols + k];
                    }
                }
            }
        }
        //printf("\n");
        //反向疊代
        for (int i = n - 1; i > 0; i--) {
            for (int j = i - 1; j >= 0; j--) {
                if (elem[i * cols + i] == 0) {
                    printf("The inverse matrix does not exist2\n");
                    system("pause");
                    exit(1);
                }
                else {
                    double m = elem[j * cols + i] / elem[i * cols + i];
                    for (int k = 0; k < n; k++) {
                        elem[j * cols + k] -= m * elem[i * cols + k];
                        dst.elem[j * dst.cols + k] -= m * dst.elem[i * dst.cols + k];
                    }
                }
            }
        }
        //整理反矩陣
        for (int i = 0; i < n; i++) {
            if (elem[i * cols + i] == 0) {
                printf("The inverse matrix does not exist3\n");
                system("pause");
                exit(1);
            }
            else {
                double m = elem[i * cols + i];
                for (int j = 0; j < n; j++) {
                    dst.elem[i * dst.cols + j] /= m;
                }
            }
        }

        *this = dst;
        return *this;
    }
    else {
        printf("Inverse matrix ERROR\n");
        exit(1);
    }
}
/*-------------------------------------------------------------------------*/

fMatrix Transp(const fMatrix &a)
{
    // Transpose of a matrix
    // 行列交換
    fMatrix dst(a.cols, a.rows);
    for (int i = 0; i < dst.rows; i++)
        for (int j = 0; j < dst.cols; j++)
            dst.elem[i*dst.cols + j] = a.GetCol(i)(j);
    return dst;
}
fMatrix AATransp(const fMatrix &a)
{
    // Computes A * Transp(A).
    return    fMatrix(a*Transp(a));
};
fMatrix ATranspA(const fMatrix &a)
{
    // Computes Transp(A) * A.
    return    fMatrix(Transp(a)*a);
}
fMatrix Outer(const fVector &a, const fVector &b)
{
    // a(RowVec)*b(ColVec)
    fMatrix dst(a.Size(), b.Size());
    for (int i = 0; i < a.Size(); i++) {
        for (int j = 0; j < dst.cols; j++) {
            dst.elem[i*dst.cols + j] = a(i) * b(j);
        }
    }
    return dst;
}

fMatrix Identity(int n)
{
    // 單位矩陣
    fMatrix dst(n, n);
    for (int i = 0; i < n; i++) {
        dst.elem[i * dst.cols + i] = 1.0;
    }
    return dst;
}
fMatrix Diag(const fVector & a)
{
    // Returns the square matrix with the elements of the vector d along its diagonal.
    // 將vector值放入matrix主對角線
    fMatrix dst = Identity(a.Size());
    for (int i = 0; i < dst.cols; i++) {
        dst.elem[i*dst.cols + i] = a.Array()[i];
    }
    return dst;
}
fVector Diag(const fMatrix & a)
{
    // 將matrix主對角線值放入vector值
    //printf("flag2\n");
    if (a.rows == a.cols) {
        int n = a.rows;
        fVector dst(n);
        for(int i = 0; i<n; i++){
            dst(i)=a.elem[i * a.cols + i];
        }
        return dst;
    }
    else {
        printf("input isn't square matrix!!\n");
        exit(1);
    }

}
fMatrix Diag(Float a, Float b, Float c)
{
    // Returns the 3 x 3 diagonal matrix with x, y, and z as its diagonal elements.
    // 建立3x3矩陣對角線數值分別為a,b,c
    //printf("flag3\n");
    fMatrix dst(3, 3);
    fVector temp_vector(a, b, c);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (i == j) {
                dst.elem[i * dst.cols + j] = temp_vector(i);
            }
            else {
                dst.elem[i * dst.cols + j] = 0;
            }
        }
    }
    return dst;
}

double Determinant(const fMatrix &a)
{
    // Computes the determinant of a square matrix
    // 計算方陣的行列式值 det(A)
    double determinat = 0;
    if (a.rows == a.cols) {
        if (a.rows == 2) {
            return a.elem[0] * a.elem[3] - a.elem[1] * a.elem[2];
        }
        else {
            for (int i = 0; i < a.cols; i++) {
                fMatrix temp(a.cols - 1, a.rows - 1);
                if (i == 0) {
                    temp = a.GetBlock(i + 1, a.cols - 1, i + 1, a.cols - 1);
                }
                else if (i == a.cols - 1) {
                    temp = a.GetBlock(1, a.rows - 1, 0, i - 1);
                }
                else {
                    temp.SetBlock(0, temp.rows - 1, 0, i - 1, a.GetBlock(1, a.rows - 1, 0, i - 1));
                    temp.SetBlock(0, temp.rows - 1, i, temp.cols - 1, a.GetBlock(1, a.rows - 1, i + 1, a.cols - 1));
                }
                determinat += pow(-1, i)*a.elem[i] * Determinant(temp);
            }
        }
    }
    return determinat;
}
double Trace(const fMatrix &a)
{
    // Computes the trace of a square matrix
    // 主對角線值相加 
    double sum = 0;
    for (int i = 0; i < a.rows; i++) {
        sum += a.elem[i * a.cols + i];
    }
    return sum;
}
double OneNorm(const fMatrix &a)
{
    // Computes the L1-norm of the matrix A, which is the maximum absolute column sum.
    double max = 0;
    for (int i = 0; i < a.cols; i++) {
        if (max < OneNorm(a.GetCol(i))) {
            max = OneNorm(a.GetCol(i));
        }
    }
    return max;
}
double InfNorm(const fMatrix &a)
{
    // Computes the Inf-norm of the matrix A, which is the maximum absolute row sum.
    double max(0);
    for (int i = 0; i < a.rows; i++)
        if (max < OneNorm(a.GetRow(i)))
            max = OneNorm(a.GetRow(i));
    return max;
}

fMatrix Inverse(const fMatrix &a)
{
    if (a.rows == a.cols) {
        fMatrix tmp(a);
        int n = tmp.rows;
        //單位矩陣
        fMatrix dst(n, n);
        for (int i = 0; i < n; i++) {
            dst.elem[i * dst.cols + i] = 1.0;
        }
        //建立上三角矩陣
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                if (tmp.elem[i * tmp.cols + i] == 0) {
                    printf("The inverse matrix does not exist1\n");
                    system("pause");
                    exit(1);
                }
                else {
                    double m = tmp.elem[j * tmp.cols + i] / tmp.elem[i * tmp.cols + i];
                    for (int k = 0; k < n; k++) {
                        tmp.elem[j * tmp.cols + k] -= m * tmp.elem[i * tmp.cols + k];
                        dst.elem[j * dst.cols + k] -= m * dst.elem[i * dst.cols + k];
                    }
                }
            }
        }
        printf("\n");
        //反向疊代
        for (int i = n - 1; i > 0; i--) {
            for (int j = i - 1; j >= 0; j--) {
                if (tmp.elem[i * tmp.cols + i] == 0) {
                    printf("The inverse matrix does not exist2\n");
                    system("pause");
                    exit(1);
                }
                else {
                    double m = tmp.elem[j * tmp.cols + i] / tmp.elem[i * tmp.cols + i];
                    for (int k = 0; k < n; k++) {
                        tmp.elem[j * tmp.cols + k] -= m * tmp.elem[i * tmp.cols + k];
                        dst.elem[j * dst.cols + k] -= m * dst.elem[i * dst.cols + k];
                    }
                }
            }
        }
        //整理反矩陣
        for (int i = 0; i < n; i++) {
            if (tmp.elem[i * tmp.cols + i] == 0) {
                printf("The inverse matrix does not exist3\n");
                system("pause");
                exit(1);
            }
            else {
                double m = tmp.elem[i * tmp.cols + i];
                for (int j = 0; j < n; j++) {
                    dst.elem[i * dst.cols + j] /= m;
                }
            }
        }
        return dst;
    }
    else {
        printf("Inverse matrix ERROR\n");
        exit(1);
    }
}

fMatrix Cholesky(const fMatrix &a)
{
    // Computes Cholesky decomposition of a square matrix.
    // Cholesky分解 當矩陣A為實數時 則求矩陣L也為實數矩陣, Cholesky分解可寫成 A=L*LT
    // A為半正定矩陣 且為對稱矩陣 Cholesky分解有解
    //a.Show();
    if (a.rows == a.cols)
    {
        // 判斷是否對稱
        int n = a.rows;
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (a.elem[i * a.cols + j] != a.elem[j * a.cols + i]) {
                    printf("Matrix asymmetry\n");
                    system("pause");
                    exit(1);
                }
            }
        }
        // 判斷是否為正定矩陣
        for (int i = 0; i < n; i++) {
            double sum = Determinant(a.GetBlock(0, i, 0, i));
            if (sum < 0) {
                printf("Non-definite matrix\n");
                system("pause");
                exit(1);
            }
        }
        //https://blog.csdn.net/billbliss/article/details/78559387
        fMatrix L(n, n);
        for (int k = 0; k < n; k++) {
            double sum = 0;
            for (int i = 0; i < k; i++) {
                sum += L.elem[k * L.cols + i] * L.elem[k * L.cols + i];
            }
            sum = a.elem[k * a.cols + k] - sum;
            L.elem[k * L.cols + k] = sqrt(sum > 0 ? sum : 0);
            for (int i = k + 1; i < n; i++) {
                sum = 0;
                for (int j = 0; j < k; j++) {
                    sum += L.elem[i * L.cols + j] * L.elem[k * L.cols + j];
                }
                L.elem[i * L.cols + k] = (a.elem[i * a.cols + k] - sum) / L.elem[k * L.cols + k];
            }
            for (int j = 0; j < k; j++) {
                L.elem[j * L.cols + k] = 0;
            }
        }
        return L;
    }
    else {
        printf("input isn't a square matrix!!\n");
    }
}

fVector Mean(const fMatrix &a)
{
    // Computes column mean value of a matrix.
    fVector dst(a.cols);
    for (int i = 0; i < a.cols; i++) {
        dst(i) = Mean(a.GetCol(i));
    }
    return dst;
}

fMatrix Cov(const fMatrix &a)
{
    // Returns a covariance matrix of a square matrix.
    fMatrix aa(a);
    for (int i = 0; i < a.cols; i++) {
        for (int j = 0; j < a.rows; j++) {
            aa.elem[i*a.rows + j] -= Mean(a)(j);
        }
    }
    return ATranspA(aa) / (a.cols - 1);
}
fMatrix Cov(const fVector &a)
{
    // Returns a covariance matrix of a vector, using outer product.
    return Cov(Outer(a, a));
}
void SVDcmp(const fMatrix &M, fMatrix &AU, fMatrix &W, fMatrix &V)
{
    // Computes SVD decomposition of a matrix.

    CvMat *Matrix1 = cvCreateMat(M.rows, M.cols, CV_64FC1);
    CvMat *W_ = cvCreateMat(M.rows, M.cols, CV_64FC1);
    CvMat *V_ = cvCreateMat(M.cols, M.cols, CV_64FC1);
    CvMat *U_ = cvCreateMat(M.rows, M.rows, CV_64FC1);
    //int rows = M.rows,cols = M.cols;
    //double Array1[4][5];
    double* Array1 = new double[M.rows*M.cols];
    for (int i = 0; i < M.rows; i++) {
        for (int j = 0; j < M.cols; j++) {
            Array1[i*M.cols + j] = M.elem[i*M.cols + j];
            //std::cout << M.elem[i*M.cols + j];
        }
        //std::cout << std::endl;
    }
    cvSetData(Matrix1, Array1, Matrix1->step);

    cvSVD(Matrix1, W_, U_, V_);

    for (int i = 0; i < M.rows; i++) {
        for (int j = 0; j < M.cols; j++) {
            M.elem[i*M.cols + j] = cvGet2D(Matrix1, i, j).val[0];
        }
    }
    for (int i = 0; i < AU.rows; i++) {
        for (int j = 0; j < AU.cols; j++) {
            AU.elem[i*AU.cols + j] = cvGet2D(U_, i, j).val[0];
        }
    }
    for (int i = 0; i < W.rows; i++) {
        for (int j = 0; j < W.cols; j++) {
            W.elem[i*W.cols + j] = cvGet2D(W_, i, j).val[0];
        }
    }
    for (int i = 0; i < V.rows; i++) {
        for (int j = 0; j < V.cols; j++) {
            V.elem[i*V.cols + j] = cvGet2D(V_, i, j).val[0];
        }
    }
}
void ShowMatrix(const fMatrix &a) {
    // Print a matrix on screen.
    std::cout << " ┌ ";
    for (int i = 0; i < a.cols; i++) {
        std::cout << std::setw(9) << " ";
    }
    std::cout << "┐ " << std::endl;
    for (int i = 0; i < a.rows; i++) {
        std::cout << " │";
        for (int j = 0; j < a.cols; j++) {
            std::cout << std::fixed << std::setprecision(4) << std::setw(9) << a.elem[i*a.cols + j];
        }
        printf(" │\n");
    }
    printf(" └ ");
    for (int i = 0; i < a.cols; i++) {
        std::cout << std::setw(9) << " ";
    }
    printf("┘\n");
};
