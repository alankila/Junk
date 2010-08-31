#pragma once

namespace akmd {

class Matrix {
    private:
    int rows;
    int cols;
    float **values;

    void swapRows(int i1, int i2);

    public:
    Matrix(int r, int c);
    ~Matrix();
    Matrix* multiply(Matrix* o);
    Matrix* transpose();
    Matrix* augment(Matrix *o);
    bool calculateReducedRowEchelonForm();
    float* getColumn(int col);
    void set(int r, int c, float value);
    static float* solve(Matrix* a, Matrix* b);
    static float* leastSquares(Matrix* a, Matrix* b);
};

}
