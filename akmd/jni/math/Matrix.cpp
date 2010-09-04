/*
 * Copyright Antti S. Lankila, 2010.
 * Licensed under the GPL (userspace counterpart of kernel).
 */
#include <android/log.h>
#include <math.h>
#include "math/Matrix.hpp"

namespace akmd {

using namespace std;

static const float EPSILON = 1e-10f;

void Matrix::swapRows(int r1, int r2) {
    float* tmp = values[r1];
    values[r1] = values[r2];
    values[r2] = tmp;
}

Matrix::Matrix(int r, int c) {
    rows = r;
    cols = c;

    values = new float*[rows];
    for (int i = 0; i < rows; i ++) {
        values[i] = new float[cols];
    }
}

Matrix::~Matrix() {
    for (int i = 0; i < rows; i ++) {
        delete[] values[i];
    }
    delete[] values;
}

void Matrix::set(int r, int c, float v) {
    values[r][c] = v;
}

Matrix* Matrix::multiply(Matrix* o) {
    Matrix* m = new Matrix(rows, o->cols);
    for (int r1 = 0; r1 < rows; r1 ++) {
        for (int c1 = 0; c1 < o->cols; c1 ++) {
            float v = 0;
            for (int x = 0; x < cols; x ++) {
                v += values[r1][x] * o->values[x][c1];
            }
            m->values[r1][c1] = v;
        }
    }
    return m;
}

Matrix* Matrix::transpose() {
    Matrix* t = new Matrix(cols, rows);
    for (int r = 0; r < rows; r ++) {
        for (int c = 0; c < cols; c ++) {
            t->values[c][r] = values[r][c];
        }
    }
    return t;
}

Matrix* Matrix::augment(Matrix *o) {
    Matrix* a = new Matrix(rows, cols + o->cols);

    for (int r = 0; r < rows; r ++) {
        for (int c1 = 0; c1 < cols; c1 ++) {
            a->values[r][c1] = values[r][c1];
        }
        for (int c2 = 0; c2 < o->cols; c2 ++) {
            a->values[r][cols + c2] = o->values[r][c2];
        }
    }

    return a;
}

bool Matrix::calculateReducedRowEchelonForm() {
    for (int r1 = 0; r1 < rows-1; r1 ++) {
        int maxrow = r1;
        for (int r2 = r1 + 1; r2 < rows; r2 ++) {
            if (fabsf(values[r2][r1]) > fabsf(values[maxrow][r1])) {
                maxrow = r2;
            }
        }
        swapRows(r1, maxrow);
        
        if (fabsf(values[r1][r1]) <= EPSILON) {
            return false;
        }
            
        for (int r2 = r1 + 1; r2 < rows; r2 ++) {
            float f = values[r2][r1] / values[r1][r1];
            for (int c = r1; c < cols; c ++) {
                values[r2][c] -= values[r1][c] * f;
            }
        }
    }
    
    for (int r1 = rows-1; r1 >= 0; r1 --) {
        float f = values[r1][r1];
        for (int r2 = 0; r2 < r1; r2 ++) {
            for (int c = cols-1; c >= r1; c --) {
                values[r2][c] -= values[r1][c] * values[r2][r1] / f;
            }
        }
        
        values[r1][r1] /= f;
        for (int c = rows; c < cols; c ++) {
            values[r1][c] /= f;
        }
    }
        
    return true;
}

float* Matrix::getColumn(int col) {
    float* v = new float[rows];
    for (int r = 0; r < rows; r ++) {
        v[r] = values[r][col];
    }
    return v;
}

float* Matrix::solve(Matrix* a, Matrix* b) {
    Matrix* v = a->augment(b);
    if (! v->calculateReducedRowEchelonForm()) {
        return 0;
    }
    float* x = v->getColumn(a->cols);
    delete v;
    return x;
}
    
float* Matrix::leastSquares(Matrix* a, Matrix* b) {
    Matrix* at = a->transpose();
    Matrix* a2 = at->multiply(a);
    Matrix* b2 = at->multiply(b);
    float* x = solve(a2, b2);
    delete at;
    delete a2;
    delete b2;
    return x;
}

}
