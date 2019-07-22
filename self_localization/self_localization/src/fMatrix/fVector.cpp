#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <iomanip>
#include"fVector.h"

/*-------------------------------------------------------------------------*
*                                                                         *
*  C O N S T R U C T O R S  & D E S T R U C T O R S                       *
*                                                                         *
*-------------------------------------------------------------------------*/
//Vector 計數
int fVector::nVecCount = 0;

//創建大小size的 空的 Vector
fVector::fVector(int size) :
    size(size),
    elem(new Float[size])
{
    for (int i = 0; i < size; i++) {
        elem[i] = 0;
    }
    nVecCount++;
}

//複製 Vector
fVector::fVector(const fVector &a) :
    size(a.Size()),
    elem(new Float[a.Size()])
{
    for (int i = 0; i < size; i++) {
        elem[i] = a.elem[i];
    }
    nVecCount++;
}

//創建一個大小為 n ，值為 x 陣列的 Vector
fVector::fVector(const Float *x, int n) :
    size(n),
    elem(new Float[n])
{
    for (int i = 0; i < n; i++) {
        elem[i] = *(x + i);
    }
    nVecCount++;
}

//創建一個大小為 n ，值為 x 陣列的 Vector
fVector::fVector(int n, const Float *x) :
    size(n),
    elem(new Float[n])
{
    for (int i = 0; i < n; i++) {
        elem[i] = *(x + i);
    }
    nVecCount++;
}

//創建一個大小為2，值為a與b
fVector::fVector(Float a, Float b) :
    size(2),
    elem(new Float[2])
{
    elem[0] = a;
    elem[1] = b;
    nVecCount++;
}

//創建一個大小為3，值為a,b,c
fVector::fVector(Float a, Float b, Float c) :
    size(3),
    elem(new Float[3])
{
    elem[0] = a;
    elem[1] = b;
    elem[2] = c;
    nVecCount++;
}

//釋放 Vector
fVector::~fVector()
{
    delete[] elem;
    nVecCount--;
}

// 給Vector
fVector &fVector::operator=(const fVector &a)
{
    delete[] elem;
    size = a.Size();
    elem = new Float[a.Size()];
    for (int i = 0; i < a.Size(); i++) {
        elem[i] = a(i);
    }
    return *this;
}

//給Num
void fVector::operator=(Float num)
{
    delete[] elem;
    size = 1;
    elem[0] = num;
}

//設定大小
void fVector::SetSize(int n)
{
    delete[] elem;
    size = n;
    elem = new Float[n];
}

//交換
fVector &fVector::Swap(int a, int b)
{
    Float tmp;
    tmp = elem[a];
    elem[a] = elem[b];
    elem[b] = tmp;
    return *this;
}

//得到[a, b]的Vector
fVector fVector::GetBlock(int a, int b) const
{
    int start = (a <= b ? a : b);
    int end = (a >= b ? a : b);
    int _size = end - start + 1;
    fVector dst(_size);
    for (int i = 0; i < dst.Size(); i++) {
        dst(i) = elem[a + i];
    }
    return dst;
}

//重新設置[a, b]的Vector
void fVector::SetBlock(int a, int b, const fVector &src_vector)
{
    int start = (a <= b ? a : b);
    int end = (a >= b ? a : b);
    int _size = end - start + 1;
    for (int i = start; i <= end; i++) {
        elem[i - start] = src_vector.elem[i];
    }
}

//顯示Vector
void fVector::Show(VecType Type) const
{
    switch (Type) {
    case(ColVec):
        std::cout << " [";
        for (int i = 0; i < size; i++) {
            std::cout << std::fixed << std::setprecision(4) << std::setw(9) << elem[i];
        }
        std::cout<<" ]\n";
        break;
    case(RowVec):
        for (int i = 0; i < size; i++) {
            if (i == 0) std::cout<<" ┌    "<< std::setw(9)<<"┐\n";
            std::cout<<" │"<< std::setw(9)<<elem[i]<<" │\n";
            if (i == size - 1) std::cout << " └    " << std::setw(9) << "┘\n";
        }
        break;
    default:
        break;
    }
}
/*-------------------------------------------------------------------------*
*                                                                         *
*  FRIEND OPERATORS                                                       *
*                                                                         *
*-------------------------------------------------------------------------*/

// Vector 加法
fVector operator+(const fVector &a, const fVector &b)
{
    if (a.Size() == b.Size()) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            dst(i) = a(i) + b(i);
        }
        return dst;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// Vector 減法
fVector operator-(const fVector &a, const fVector &b)
{
    if (a.Size() == b.Size()) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            dst(i) = a(i) - b(i);
        }
        return dst;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// Vector 加負號
fVector operator-(const fVector &a)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = -a(i);
    }
    return dst;
}

// Vector 的值全減Num
fVector operator-(const fVector &a, Float num)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = a(i) - num;
    }
    return dst;
}

// 創建一個大小與Vector a相同之Vector其值都為Num，並與Vector a相減
fVector operator-(Float num, const fVector &a)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = num - a(i);
    }
    return dst;
}

// Vector中的值全部乘num倍
fVector operator*(const fVector &a, Float num)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = a(i) * num;
    }
    return dst;
}

// Vector中的值全部乘num倍
fVector operator*(Float num, const fVector &a)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = a(i) * num;
    }
    return dst;
}

// Vector中的值全部除num倍
fVector operator/(const fVector &a, Float num)
{
    if (num != 0) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            dst(i) = a(i) / num;
        }
        return dst;
    }
    else {
        printf("scale can't set to zero\n");
        exit(1);
    }
}

// 兩Vector相除
fVector operator/(const fVector &a, const fVector &b)
{
    //dst = [a1/b1, a2/b2, ... , an/bn]
    if (a.Size() == b.Size()) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            if (b(i) != 0) {
                dst(i) = a(i) / b(i);
            }
            else {
                printf("div value can't set as zero\n");
                exit(1);
            }
        }
        return dst;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// 兩Vector內積
double operator*(const fVector &a, const fVector &b)
{
    //a*b = a1b1 + a2b2 + ... + anbn
    if (a.Size() == b.Size()) {
        double dst = 0.0;
        for (int i = 0; i < a.Size(); i++) {
            dst += a(i) * b(i);
        }
        return dst;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// 兩Vector外積
fVector operator^(const fVector &a, const fVector &b)
{
    //axb = (a2b3-a3b2)i + (a3b1-a1b3)j + (a1b2-a2b1)k

    if (a.Size() == 3 && b.Size() == 3) {
        fVector dst(3);
        dst(0) = a(1) * b(2) - a(2) * b(1);
        dst(1) = a(2) * b(0) - a(0) * b(2);
        dst(2) = a(0) * b(1) - a(1) * b(0);
        return dst;
    }
    else {
        printf("both vectors must be 3 in size\n");
        exit(1);
    }

}

// 兩Vector做加等於運算 (a = a + b)
fVector &operator+=(fVector &a, const fVector &b)
{

    if (a.Size() == b.Size()) {
        for (int i = 0; i < a.Size(); i++) {
            a.elem[i] = a(i) + b(i);
            //printf("%lf  %lf  %lf \n", a(i),b(i),dst(i));
        }
        return a;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// 兩Vector做減等於運算。(a = a - b)
fVector &operator-=(fVector &a, const fVector &b)
{
    if (a.Size() == b.Size()) {
        for (int i = 0; i < a.Size(); i++) {
            a.elem[i] = a(i) - b(i);
        }
        return a;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// Vector與變數b做乘等於運算 (src = src * b)
fVector &operator*=(fVector &a, Float num)
{
    for (int i = 0; i < a.Size(); i++) {
        a.elem[i] = a(i) * num;
    }
    return a;
}

// Vector與變數b做除等於運算 (a = a / b)
fVector &operator/=(fVector &a, Float num)
{
    if (num != 0) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            a.elem[i] = a(i) / num;
        }
        return a;
    }
    else {
        printf("scale can't set to zero\n");
        exit(1);
    }
}
/*-------------------------------------------------------------------------*
*                                                                         *
*  FRIEND FUNCTIONS                                                       *
*                                                                         *
*-------------------------------------------------------------------------*/

// 比較兩Vector回傳較小者
fVector Min(const fVector &a, const fVector &b)
{
    if (a.Size() == b.Size()) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            dst(i) = (a(i) <= b(i)) ? a(i) : b(i);
        }
        return dst;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// 比較兩Vector回傳較大者
fVector Max(const fVector &a, const fVector &b)
{
    if (a.Size() == b.Size()) {
        fVector dst(a.Size());
        for (int i = 0; i < a.Size(); i++) {
            dst(i) = (a(i) >= b(i)) ? a(i) : b(i);
        }
        return dst;
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// 計算兩Vector間之距離
double Dist(const fVector &a, const fVector &b)
{
    //dst = √((a1-b1)^2 + (a2-b2)^2 + ... + (an-bn)^2)
    if (a.Size() == b.Size()) {
        double sum = 0.0;
        for (int i = 0; i < a.Size(); i++) {
            sum += pow(a(i) - b(i), 2);
        }
        return sqrt(sum);
    }
    else {
        printf("two vectors are not equal in size\n");
        exit(1);
    }
}

// 計算one-norm距離 vector值加總的絕對值
double OneNorm(const fVector &a)
{
    double distance = 0;
    for (int i = 0; i < a.Size(); i++) {
        distance += fabs(a(i));
    }
    return distance;
}

// 計算two-norm距離 √a*a
double TwoNorm(const fVector &a)
{
    return sqrt(a * a);
}

// 將Vector進行正規化
fVector Normalize(const fVector &a)
{
    double distance = TwoNorm(a);
    if (distance != 0) {
        return a / distance;
    }
    else {
        printf("the twonorm is zero\n");
        exit(1);
    }
}

// 計算2 - norm距離平方
//Returns square of the two norm value of a vector
double TwoNormSqr(const fVector &a)
{
    return pow(TwoNorm(a), 2);
}
fVector  Pow(const fVector &a, int num)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = pow(a(i), num);
    }
    return dst;
}
// 計算Vector數值平方根
fVector Sqrt(const fVector &a)
{
    fVector dst(a.Size());
    for (int i = 0; i < a.Size(); i++) {
        dst(i) = sqrt(a(i));
    }
    return dst;
}

// 計算Vector數值平均
double Mean(const fVector &a)
{
    if (a.Size()) {
        double dst = 0;
        for (int i = 0; i < a.Size(); i++) {
            dst += a(i);
        }
        dst = dst / a.Size();
        return dst;
    }
    else {
        printf("the vector size is zero\n");
        exit(1);
    }
}

// 計算Vector數值變異數
double Var(const fVector &dst)
{
    if (dst.Size()) {
        return OneNorm(Pow(dst - Mean(dst),2)) / (dst.Size() - 1);
    }
    else {
        printf("the vector size is zero\n");
        exit(1);
    }
}

// 計算Vector數值標準差
double Std(const fVector &dst)
{
    if (dst.Size()) {
        return sqrt(Var(dst));
    }
    else {
        printf("the vector size is zero\n");
        exit(1);
    }
}

// 顯示Vector
void ShowVector(const fVector &a, VecType Type)
{
    switch (Type) {
    case(ColVec):
        std::cout << " [";
        for (int i = 0; i < a.size; i++) {
            std::cout << std::fixed << std::setprecision(4) << std::setw(9) << a.elem[i];
        }
        std::cout << " ]\n";
        break;
    case(RowVec):
        for (int i = 0; i < a.size; i++) {
            if (i == 0) std::cout << " ┌    " << std::setw(9) << "┐\n";
            std::cout << " │" << std::setw(9) << a.elem[i] << " │\n";
            if (i == a.size - 1) std::cout << " └    " << std::setw(9) << "┘\n";
        }
        break;
    default:
        break;
    }
}
