/*
*	fVector.h
*
*	Description:
*		Basic vector class with some associated methods.
*
*
*
* 	History:
*	 	Author			Date			Modify Reason
*		----------------------------------------------------------------
*		Chi-Yi Tsai		2015/02/26		File Creation
*
*/

#ifndef __VECTOR_INCLUDED__
#define __VECTOR_INCLUDED__

#ifndef DOUBLE_PRECISION
#define DOUBLE_PRECISION
#endif

#ifndef Float
#ifdef DOUBLE_PRECISION
#define Float double
#else
#define Float float
#endif
#endif

enum VecType { ColVec = 1, RowVec };

class fVector
{
    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  FRIEND OPERATORS                                                       *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    friend fVector  operator +  (const fVector &, const fVector &);
    friend fVector  operator -  (const fVector &, const fVector &); // Binary minus.
    friend fVector  operator -  (const fVector &);  // Unary minus.
    friend fVector  operator -  (const fVector &, Float);
    friend fVector  operator -  (Float, const fVector &);
    friend fVector  operator *  (const fVector &, Float);
    friend fVector  operator *  (Float, const fVector &);
    friend fVector  operator /  (const fVector &, Float);
    friend fVector  operator /  (const fVector &, const fVector &); // Element-wise division
    friend double   operator *  (const fVector &, const fVector &); // Inner-product between two vectors
    friend fVector  operator ^  (const fVector &, const fVector &); // Cross-product between two vectors
    friend fVector& operator += (fVector &, const fVector &);
    friend fVector& operator -= (fVector &, const fVector &);
    friend fVector& operator *= (fVector &, Float);
    friend fVector& operator /= (fVector &, Float);

    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  FRIEND FUNCTIONS                                                       *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    friend fVector  Min(const fVector &, const fVector &); // Element-wise minimum-element extraction between two vectors
    friend fVector  Max(const fVector &, const fVector &); // Element-wise maximum-element extraction between two vectors
    friend double   Dist(const fVector &, const fVector &); // Returns two norm distance between two vectors
    friend fVector  Normalize(const fVector &); // Normalizes a vector into an unit vector
    friend double   OneNorm(const fVector &); // Returns one norm value of a vector
    friend double   TwoNorm(const fVector &); // Returns two norm value of a vector
    friend double   TwoNormSqr(const fVector &); // Returns square of the two norm value of a vector

    friend fVector  Pow(const fVector &, int); // Element-wise square of a vector
    friend fVector  Sqrt(const fVector &); // Element-wise square root of a vector
    friend double   Mean(const fVector &); // Mean value of a vector.
    friend double   Var(const fVector &); // Variance of a vector. 
    friend double   Std(const fVector &); // Standard derivation of a vector.    	

    friend void     ShowVector(const fVector &, VecType Type = ColVec);

public:
    /*-------------------------------------------------------------------------*
    *                                                                         *
    *  C O N S T R U C T O R S  & D E S T R U C T O R S                       *
    *                                                                         *
    *-------------------------------------------------------------------------*/
    // Initinalize constructor.
    fVector(int size = 0);
    // Copy constructor.
    fVector(const fVector &);
    // Assign constructor.
    fVector(const Float *x, int n);
    fVector(int n, const Float *x);
    fVector(Float, Float);
    fVector(Float, Float, Float);
    ~fVector();

    static  const fVector Null;

public:
    fVector &operator=(const fVector &);
    void    operator=(Float);
    void    SetSize(int);

    fVector &Swap(int i, int j);
    fVector GetBlock(int i, int j) const;
    void    SetBlock(int i, int j, const fVector &);
    void	Show(VecType Type = ColVec) const;

public: // Inlined functions.
    inline Float  operator()(int i) const { return elem[i]; }
    inline Float &operator()(int i) { return elem[i]; }
    inline Float *Array() const { return elem; }
    inline int    Size() const { return size; }

private:
    int    size;
    Float* elem;

    static int nVecCount;
};

#endif // __VECTOR_INCLUDED__