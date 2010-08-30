package com.bel.android.magneticflux;

/**
 * Matrix arithmetics support.
 * 
 * The constructed matrices are immutable apart from setRow/setColumn that
 * can modify the values in matrix. All instance methods make copies of values. All
 * returned values are copies.
 * 
 * @author alankila
 */
public class Matrix {
	private static final float EPSILON = 1e-10f;
	
	final protected int rows, cols;
	final protected float[][] values;
	
	/**
	 * Construct a new matrix of specified size, initialized to 0
	 * 
	 * @param r rows
	 * @param c columns
	 */
	public Matrix(int r, int c) {
		rows = r;
		cols = c;
		values = new float[r][c];
	}

	/**
	 * Construct a new matrix from array.
	 * 
	 * @param v matrix data in v[row][column] order
	 */
	public Matrix(float[][] v) {
		rows = v.length;
		cols = v[0].length;
		values = v;
	}
	
	/**
	 * Construct a n*1 matrix (vector)
	 * 
	 * @param v vector data
	 */
	public Matrix(float[] v) {
		rows = v.length;
		cols = 1;
		values = new float[v.length][1];
		for (int i = 0; i < v.length; i ++) {
			values[i][0] = v[i];
		}
	}
	
	/**
	 * Construct the identity square matrix of given size
	 * 
	 * @param rc Number of rows and columns
	 * @return the identity matrix
	 */
	public static Matrix identity(int rc) {
		Matrix m = new Matrix(rc, rc);
		for (int i = 0; i < rc; i ++) {
			m.values[i][i] = 1;
		}
		return m;
	}

	/**
	 * Return a row vector of matrix. (Numbered from 0.)
	 * 
	 * @param r row
	 * @return the row vector
	 */
	public float[] getRow(int r) {
		float[] rV = new float[cols];
		for (int i = 0; i < cols; i ++) {
			rV[i] = values[r][i];
		}
		return rV;
	}

	/**
	 * Set the matrix row vector. (Numbered from 0.)
	 * 
	 * @param r row
	 * @param rV the row vector
	 */
	public void setRow(int r, float[] rV) {
		for (int i = 0; i < cols; i ++) {
			values[r][i] = rV[i];
		}
	}
	
	/**
	 * Return a column vector of matrix. (Numbered from 0.)
	 * 
	 * @param c column
	 * @return the column vector
	 */
	public float[] getColumn(int c) {
		float[] cV = new float[rows];
		for (int i = 0; i < rows; i ++) {
			cV[i] = values[i][c];
		}
		return cV;
	}
	
	/**
	 * Set the matrix column vector. (Numbered from 0.)
	 * 
	 * @param c column
	 * @param cV the column vector
	 */
	public void setColumn(int c, float[] cV) {
		for (int i = 0; i < rows; i ++) {
			values[i][c] = cV[i];
		}
	}
	
	/**
	 * Add two matrices
	 * 
	 * @param o the other matrix
	 * @return the sum of two matrices
	 */
	public Matrix add(Matrix o) {
		Matrix a = new Matrix(rows, cols);
		for (int r = 0; r < rows; r ++) {
			/* Go through the columns of right-hand matrix. */
			for (int c = 0; c < cols; c ++) {
				a.values[r][c] = values[r][c] + o.values[r][c];
			}
		}
		return a;
	}
	
	/**
	 * Multiply two matrices. The new matrix takes the number
	 * of rows from this object, and the columns from the passed
	 * Matrix object. The dimensions must of the two matrices must be
	 * compatible. (n * m multiplied by m * n matrix.)
	 * 
	 * @param o the other matrix
	 * @return The multiplication result
	 */
	public Matrix multiply(Matrix o) {
		/* The new matrix will have as many rows as the left-hand matrix,
		 * and as many columns as the right-hand matrix. */
		Matrix p = new Matrix(rows, o.cols);
		/* Go through the rows of left-hand matrix. */
		for (int r1 = 0; r1 < rows; r1 ++) {
			/* Go through the columns of right-hand matrix. */
			for (int c1 = 0; c1 < o.cols; c1 ++) {
				/* Calculate the dot product of selected column on left-hand-side,
				 * and row on right-hand-side, and store it on the new matrix. */
				for (int x = 0; x < cols; x ++) {
					p.values[r1][c1] += values[r1][x] * o.values[x][c1];
				}
			}
		}
		return p;
	}
	
	/**
	 * Return the transpose of matrix.
	 * 
	 * @return the transpose
	 */
	public Matrix transpose() {
		Matrix t = new Matrix(cols, rows);
		for (int r = 0; r < rows; r ++) {
			/* Go through the columns of right-hand matrix. */
			for (int c = 0; c < cols; c ++) {
				t.values[c][r] = values[r][c];
			}
		}
		return t;
	}

	/**
	 * Augment a matrix by concatenating it with another side-by-side.
 	 *
	 * @param o the other matrix
	 * @return augmented matrix
	 */
	public Matrix augment(Matrix o) {
		Matrix a = new Matrix(rows, cols + o.cols);
		for (int r = 0; r < rows; r ++) {
			for (int c1 = 0; c1 < cols; c1 ++) {
				a.values[r][c1] = values[r][c1];
			}
			for (int c2 = 0; c2 < o.cols; c2 ++) {
				a.values[r][cols + c2] = o.values[r][c2];
			}
		}
		return a;
	}

	/**
	 * Use Gauss-Jordan elimination to put matrix into the reduced row echelon form.
	 * 
	 * @return true if elimination was possible (Matrix was not singular)
	 */
	public boolean calculateReducedRowEchelonForm() {
		for (int r1 = 0; r1 < rows; r1 ++) {
			int maxrow = r1;
			for (int r2 = r1 + 1; r2 < rows; r2 ++) {
				if (Math.abs(values[r2][r1]) > Math.abs(values[maxrow][r1])) {
					maxrow = r2;
				}
			}
			swapRows(r1, maxrow);
			
			if (Math.abs(values[r1][r1]) <= EPSILON) {
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
	
	private void swapRows(int r1, int r2) {
		float[] tmp = values[r1];
		values[r1] = values[r2];
		values[r2] = tmp;
	}
	
	/**
	 * [a b]
	 * [c d]
	 */
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for (int r = 0; r < rows; r ++) {
			sb.append("\n");
			sb.append("[");
			for (int c = 0; c < cols; c ++) {
				if (c != 0) {
					sb.append(" ");
				}
				sb.append(String.format("%10.2f", values[r][c]));
			}
			sb.append("]");
		}
		sb.append("\n");
		return sb.toString();
	}
}