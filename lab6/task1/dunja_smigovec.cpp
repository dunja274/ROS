#include <stdio.h>
#include <iostream>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
using namespace std;
using namespace boost::numeric::ublas;
#define N 5

void getCofactor(matrix<double> *m, matrix<double> *temp, int p, int q, int n)
{
    int i = 0, j = 0;
    for (int row = 0; row < n; row++)
    {
        for (int col = 0; col < n; col++)
        {
            if (row != p && col != q)
            {
                (*temp)(i, j++) = (*m)(row, col);
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

int determinantOfMatrix(matrix<double> *m, int n)
{
    if (n == 1)
        return (*m)(0, 0);
    
    int D = 0;

    matrix<double> temp(n, n);

    int sign = 1;
    for (int f = 0; f < n; f++)
    {
        getCofactor(m, &temp, 0, f, n);
        D += sign * (*m)(0, f) * determinantOfMatrix(&temp, n - 1);
        sign = -sign;
    }

    return D;
}

void adjoint(matrix<double> *m, matrix<double> *adj)
{
    if (N == 1)
    {
        (*adj)(0, 0) = 1;
        return;
    }

    int sign = 1;
    matrix<double> temp(5, 5);

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            getCofactor(m, &temp, i, j, N);
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            (*adj)(j, i) = (sign) * (determinantOfMatrix(&temp, N - 1));
        }
    }
}

bool inverse(matrix<double> *m, matrix<double> *inverse)
{
    int det = determinantOfMatrix(m, N);
    if (det == 0)
    {
        return false;
    }

    matrix<double> adj(N, N);
    adjoint(m, &adj);

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            (*inverse)(i, j) = adj(i, j) / double(det);

    return true;
}

int main()
{

    boost::numeric::ublas::vector<double> v (5);
    matrix<double> m1(5, 5);
    matrix<double> m2(5, 5);
    v.insert_element(0, 1);
    v.insert_element(1, 6);
    v.insert_element(2, 2);
    v.insert_element(3, 3);
    v.insert_element(4, 9);
    identity_matrix<int> identity(5);

    for (int i = 0; i < m1.size1(); ++i)
    {
        for (int j = 0; j < m1.size2(); ++j)
        {
            if (j == 0 || j == 1)
            {
                m1(i, j) = 0;
            }
            else if (j == 2)
            {
                m1(i, j) = 3;
            }
            else if (j == 3)
            {
                m1(i, j) = 6;
            }
            else if (j == 4)
            {
                m1(i, j) = 5;
            }
        }
    }
    m2 = m1 + identity;
    matrix<int> m(5, 1);
    m(0, 0) = 1;
    m(1, 0) = 6;
    m(2, 0) = 2;
    m(3, 0) = 3;
    m(4, 0) = 9;
    
    cout << prod(m2,v) << endl;
    cout << prod(v,m) << endl;
    cout << "determinant: " << determinantOfMatrix(&m2, 5) << endl;

    matrix<double> adj(5, 5);
    matrix<double> inv(5, 5);

    adjoint(&m2, &adj);
    inverse(&m2, &inv);
    cout << "inverse: " << inv << endl;

    return 0;
}
