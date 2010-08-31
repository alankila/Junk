#include <math.h>

using namespace std;

static const float EPSILON = 1e-10f;

class Matrix {
    private:
    int rows, cols;
    float **values;

    void swapRows(int i1, int i2) {
        float *tmp = values[i1];
        values[i1] = values[i2];
        values[i2] = tmp;
    }

    public:
    Matrix(int r, int c) {
        rows = r;
        cols = c;

        values = new float*[rows];
        for (int i = 0; i < rows; i ++) {
            values[i] = new float[cols];
        }
    }

    ~Matrix() {
        for (int i = 0; i < rows; i ++) {
            delete[] values[i];
        }
        delete[] values;
    }

    Matrix* multiply(Matrix* o) {
        Matrix* m = new Matrix(rows, o->cols);
        for (int r = 0; r < rows; r ++) {
            for (int c1 = 0; c1 < o->cols; c1 ++) {
                float v = 0;
                for (int c2 = 0; c2 < cols; c2 ++) {
                    v += values[r][c2] * o->values[c2][c1];
                }
                m->values[r][c1] = v;
            }
        }
        return m;
    }

    Matrix* transpose() {
        Matrix* t = new Matrix(cols, rows);
        for (int r = 0; r < rows; r ++) {
            for (int c = 0; c < cols; c ++) {
                t->values[c][r] = values[r][c];
            }
        }
        return t;
    }

    Matrix* augment(Matrix *o) {
        Matrix *v = new Matrix(rows, cols + o->cols);

        for (int r = 0; r < rows; r ++) {
            for (int c = 0; c < cols; c ++) {
                v->values[r][c] = values[r][c];
            }
            for (int c = 0; c < o->cols; c ++) {
                v->values[r][cols + c] = o->values[r][c];
            }
        }

        return v;
    }

    bool calculateReducedRowEchelonForm() {
        for (int r1 = 0; r1 < rows-1; r1 ++) {
            int maxrow = r1;
            for (int r2 = r1 + 1; r2 < rows; r2 ++) {
                if (fabs(values[r2][r1]) > fabs(values[maxrow][r1])) {
                    maxrow = r2;
                }
            }
            swapRows(r1, maxrow);
            
            if (fabs(values[r1][r1]) <= EPSILON) {
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

    float* getColumn(int col) {
        float *v = new float[rows];
        for (int r = 0; r < rows; r ++) {
            v[r] = values[rows][col];
        }
        return v;
    }

    static float* solve(Matrix* a, Matrix* b) {
        Matrix *v = a->augment(b);
        if (! v->calculateReducedRowEchelonForm()) {
            return 0;
        }
        float *x = v->getColumn(a->cols);
        delete v;
        return x;
    }
    
    static float* leastSquares(Matrix* a, Matrix* b) {
        Matrix* at = a->transpose();
        Matrix* a2 = at->multiply(a);
        Matrix* b2 = at->multiply(b);
        float *x = solve(a2, b2);
        delete at;
        delete a2;
        delete b2;
        return x;
    }
};
