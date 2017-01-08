/*
 * MyMatrix.h
 *
 *  Created on: 2016Äê11ÔÂ17ÈÕ
 *      Author: looke
 */

#ifndef MYMATRIX_H_
#define MYMATRIX_H_

class MyMatrix
{
public:
	MyMatrix();
	MyMatrix(int inputRowNum, int inputColumnNum);

	int rowNum;
	int columnNum;
	void copyMatrix(float **targetMatrix);
	void setMatrixElement(int rNum, int cNum, float val);
	//float** getMatrixPointer();

	void printMatrix();
	void printMatrixRoots();

	void swapRow(int from, int to);
	int findPivotRow(int startRow);
	void normalizePivotRow(int pivotRow);
	void eliminateSubMatrix(int pivotRow);

	void gaussElim_ColmnPrin();

	int coRank();
	int arguRank();

	void calcRoots();
	float* getRoots();

private:
	//float **matrixPointer;
	float matrixPointer[4][5];
	void initMatrix();

	bool isAllZeroForArgumentRow(int arguRowNo);
	bool isAllZeroForCoRow(int coRowNo);
	bool isAllZero(int rowNo, bool isArgument);
	float roots[4];
};



#endif /* MYMATRIX_H_ */
