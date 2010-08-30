package com.bel.android.magneticflux;

public class MatrixUtil {
	/**
	 * Solve a Ax = b system of linear equations given A and b.
	 * 
	 * @param a The n*m matrix
	 * @param b The n*1 vector
	 * @return x The m*1 vector.
	 */
	public static Matrix solve(Matrix a, Matrix b) {
		Matrix v = a.augment(b);
		v.calculateReducedRowEchelonForm();
		return new Matrix(v.getColumn(a.cols));
	}
	
	/**
	 * Approximate the solution to an overdetermined, unsolvable system of
	 * linear equations in Ax = b form, or system where b is not in the
	 * column space of A. The solution is reached by multiplying A and b
	 * with A's transpose, and solving that equation instead.
	 * 
	 * Multiplying A by its transpose makes it a square matrix with
	 * same number of rows as there were columns in the original matrix.
	 * That the resulting equation is the least squares approximate follows
	 * from the fact that difference between least-squares solution of Ax and
	 * b must be a vector in the orthogonal space of the column space of A,
	 * which must be A^T's null space, and that therefore the least squares
	 * solution must be A^T (Ax - b) = 0.
	 * 
	 * @param a the n*m matrix
	 * @param b the n*1 vector
	 * @return x the m*1 vector
	 */
	public static Matrix leastSquares(Matrix a, Matrix b) {
		Matrix at = a.transpose();
		return solve(at.multiply(a), at.multiply(b));
	}
}
