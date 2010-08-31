class Matrix {
    private:
    int rows, cols;

    float **data;

    public:
    float* getColumn(int idx);

    Matrix& augment(Matrix& a, Matrix& b);

    bool calculateReducedRowEchelonForm();

    Matrix& transpose();
};
