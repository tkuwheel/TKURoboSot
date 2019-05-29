/*
*	fMatrix.h
*
*	Description:
*		Basic matrix class with some associated methods.
*
*
*
* 	History:
*	 	Author			Date			Modify Reason
*		----------------------------------------------------------------
*		Chi-Yi Tsai		2015/02/26		File Creation
*
*/

#ifndef __MATRIX_INCLUDED__
#define __MATRIX_INCLUDED__

#include "fVector.h"

class fMatrix {
    friend class fVector;
    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  FRIEND OPERATORS                                                       *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    // 1. A+B
    friend fMatrix  operator +  (const fMatrix &, const fMatrix &);
    // 2. A-B
    friend fMatrix  operator -  (const fMatrix &);
    friend fMatrix  operator -  (const fMatrix &, const fMatrix &);
    // 3. c*A or A*c
    friend fMatrix  operator *  (const fMatrix &, Float);
    friend fMatrix  operator *  (Float, const fMatrix &);
    // 4. A/c
    friend fMatrix  operator /  (const fMatrix &, Float);
    // 5. A*B
    friend fMatrix  operator *  (const fMatrix &, const fMatrix &);
    friend fVector  operator *  (const fMatrix &, const fVector &);
    friend fVector  operator *  (const fVector &, const fMatrix &);

    friend fMatrix& operator += (fMatrix &, const fMatrix &);
    friend fMatrix& operator -= (fMatrix &, const fMatrix &);
    friend fMatrix& operator *= (fMatrix &, Float);
    friend fMatrix& operator *= (fMatrix &, const fMatrix &);
    friend fVector& operator *= (fVector &, const fMatrix &);
    friend fMatrix& operator /= (fMatrix &, Float);

    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  FRIEND FUNCTIONS                                                       *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    friend fMatrix  Transp(const fMatrix &);	// Transpose of a matrix
    friend fMatrix  AATransp(const fMatrix &);	// Computes A * Transp(A).
    friend fMatrix  ATranspA(const fMatrix &);	// Computes Transp(A) * A.
    friend fMatrix  Outer(const fVector &, const fVector &);  // Computes the outer product of two vectors.
    friend fMatrix  Identity(int nSize); // Returns an nSizexnSize identity matrix.

    friend fMatrix  Diag(const fVector &);// Returns the square matrix with the elements of the vector d along its diagonal.
    friend fVector  Diag(const fMatrix &);// Returns the vector consisting of the diagonal elements of the matrix M
    friend fMatrix  Diag(Float, Float, Float);// Returns the 3 x 3 diagonal matrix with x, y, and z as its diagonal elements.

    friend double   Determinant(const fMatrix &);// Computes the determinant of a square matrix
    friend double   Trace(const fMatrix &);// Computes the trace of a square matrix
    friend double   OneNorm(const fMatrix &);// Computes the L1-norm of the matrix A, which is the maximum absolute column sum.
    friend double   InfNorm(const fMatrix &);// Computes the Inf-norm of the matrix A, which is the maximum absolute row sum.

    friend fMatrix  Inverse(const fMatrix &);// Computes the inverse of a square matrix.
    friend fMatrix  Cholesky(const fMatrix &);// Computes Cholesky decomposition of a square matrix.	
    friend fVector  Mean(const fMatrix &);// Computes column mean value of a matrix.	
    friend fMatrix  Cov(const fMatrix &);// Returns a covariance matrix of a square matrix.
    friend fMatrix  Cov(const fVector &);// Returns a covariance matrix of a vector, using outer product.
    friend void     SVDcmp(const fMatrix &M, fMatrix &AU, fMatrix &W, fMatrix &V); // Computes SVD decomposition of a matrix.

    friend void     ShowMatrix(const fMatrix &);// Print a matrix on screen.

public:
    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  C O N S T R U C T O R S  & D E S T R U C T O R S                       *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    // Initinalize constructor.
    fMatrix(int n_rows = 0, int n_cols = 0);
    // Assign constructor.
    fMatrix(Float *Array, int n_rows, int n_cols);
    fMatrix(int n_rows, int n_cols, Float *Array);
    // Copy constructor.
    fMatrix(const fMatrix &);
    // Destructor
    ~fMatrix(void);

    static  const fMatrix Null;

    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  A S S I G N M E N T    O P E R A T O R S                               *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    // 6. A=B
    fMatrix  &operator=(const fMatrix &M);
    fMatrix  &operator=(Float s);

    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  MATRIX OPERATION FUNCTIONS                                             *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    // 7. Swap
    fMatrix &SwapRows(int i1, int i2);
    fMatrix &SwapCols(int j1, int j2);
    // 8. Inverse
    fMatrix &Inv(void);

    void    SetCol(int col, const fVector &);
    void    SetRow(int row, const fVector &);
    void    SetBlock(int imin, int imax, int jmin, int jmax, const fMatrix &);
    void    SetBlock(int imin, int imax, int jmin, int jmax, const fVector &);
    void    SetSize(int rows, int cols = 0);

    fVector  GetCol(int col) const;
    fVector  GetRow(int row) const;
    fMatrix  GetBlock(int imin, int imax, int jmin, int jmax) const;

    void	Show() const;
    inline int    Cols() const { return cols; }
    inline int    Rows() const { return rows; }
private:
    int    rows; // Number of rows in the matrix.
    int    cols; // Number of columns in the matrix.
    Float *elem; // Pointer to the actual data.

    static int nMatCount;
};

#endif // __MATRIX_INCLUDED__
