/*
 * MyMatrix.cpp
 *
 *  Created on: 2016年11月17日
 *      Author: looke
 */
#include "MyMatrix.h"
//#include <iostream>
//#include <malloc.h>
#include "math.h"

//using namespace std;


MyMatrix::MyMatrix()
{
	//默认4x5矩阵
	this->rowNum = 4;
	this->columnNum = 5;

	this->initMatrix();

};

/*
MyMatrix::MyMatrix(int inputRowNum, int inputColumnNum)
{
	if(inputRowNum > 0 && inputColumnNum >0)
	{
		this->rowNum = inputRowNum;
		this->columnNum = inputColumnNum;
		//call the default constructor
		this->initMatrix();
	}
	else
	{
		cout<<"Illegal in put row/column numbers!" << endl;
		cout<<"Using default row/column number construct Matrix!" << endl;
		MyMatrix();
	}
};
*/
void MyMatrix::initMatrix()
{
	//matrixPointer = &matrix[0][0];


	//this->matrixPointer = (float **) malloc(this->rowNum*sizeof(float *));

	//for(int i=0; i<this->rowNum; i++)
	//{
	//	this->matrixPointer[i] = (float *) malloc(this->columnNum*sizeof(float));
	//}

	for(int i=0; i<this->rowNum; i++)
	{
		for(int j=0; j<this->columnNum; j++)
		{
			this->matrixPointer[i][j] = i;
		}
	}

	this->roots[0] = 0;
	this->roots[1] = 1;
	this->roots[2] = 2;
	this->roots[3] = 3;

};
/*
void MyMatrix::printMatrix()
{
	//print each row
	for(int i=0;i<this->rowNum;i++)
	{
		for(int j=0;j<this->columnNum;j++)
		{
			cout<<this->matrixPointer[i][j]<<"\t";
		}
		cout<<endl;
	}
};

void MyMatrix::printMatrixRoots()
{
	cout<<"Jacobi Matrix Roots are:" << endl;
	cout<<this->roots[0] << ";"<<this->roots[1]<< ";"<<this->roots[2]<< ";"<<this->roots[3]<< endl;
};
*/
void MyMatrix::copyMatrix(float **targetMatrix)
{
	for(int i=0;i<this->rowNum;i++)
	{
		for(int j=0;j<this->columnNum;j++)
		{
			this->matrixPointer[i][j] = targetMatrix[i][j];
		}
	}

	//cout<<"Copy Done!"<<endl;
	//cout<<"Current Matrix:"<<endl;
	//this->printMatrix();
};

void MyMatrix::setMatrixElement(int rNum, int cNum, float val)
{
	if(rNum < this->rowNum && cNum < this->columnNum)
	{
		this->matrixPointer[rNum][cNum] = val;
	}
};
/*
float** MyMatrix::getMatrixPointer()
{
	return this->matrixPointer;
};
*/
void MyMatrix::swapRow(int from, int to)
{
	if(from < this->rowNum && to < this->rowNum)
	{
		float temp[this->columnNum];
		for(int i=0;i<this->columnNum;i++)
		{
			temp[i] = matrixPointer[from][i];
		}

		for(int i=0;i<this->columnNum;i++)
		{
			matrixPointer[from][i] = matrixPointer[to][i];
		}

		for(int i=0;i<this->columnNum;i++)
		{
			matrixPointer[to][i] = temp[i];
		}
	}
	else
	{
		//cout<<"Illegal row number." << endl;
	}
};

int MyMatrix::findPivotRow(int startRow)
{
	if(startRow >= this->rowNum)
	{
		//cout<<"Illegal row number." << endl;
		return -1;
	}
	if(startRow >= this->columnNum)
	{
		//cout<<"Pivot not exist in this sub matrix" << endl;
		return -1;
	}

	int pivotRow = startRow;
	float maxElement = 0.0;

	for(int i=startRow; i<this->rowNum; i++)
	{
		if(i >= this->columnNum)
		{
			break;
		}
		if(fabs(matrixPointer[i][startRow]) > fabs(maxElement))
		{
			maxElement = matrixPointer[i][startRow];
			pivotRow = i;
		}
	}

	return pivotRow;
};

void MyMatrix::normalizePivotRow(int pivotRow)
{
	if(pivotRow >= this->rowNum)
	{
		//cout<<"Illegal row number." << endl;
		return ;
	}
	if(pivotRow >= this->columnNum)
	{
		//cout<<"Can not normalize sub matrix." << endl;
		//cout<<"PivotRow:" << pivotRow <<endl;
		//cout<<"Column number:" << this->columnNum <<endl;
		return ;
	}
	float pivotValue = matrixPointer[pivotRow][pivotRow];
	//高斯消元过程中有个Bug，在每行做归一化的时候需要先判断主元是否为0，否则会出现分母为0的情况
	if(pivotValue == 0)
	{
		//跳过归一化
		return;
	}
	for(int i=pivotRow;i<this->columnNum;i++)
	{
		matrixPointer[pivotRow][i] = matrixPointer[pivotRow][i]/pivotValue;
	}
};

void MyMatrix::eliminateSubMatrix(int pivotRow)
{
	if(pivotRow >= this->rowNum)
	{
		//cout<<"Illegal row number." << endl;
		return ;
	}
	if(pivotRow >= this->columnNum)
	{
		//cout<<"Can not eliminate sub matrix." << endl;
		//cout<<"PivotRow:" << pivotRow <<endl;
		//cout<<"Column number:" << this->columnNum <<endl;
		return ;
	}
	for(int i=pivotRow+1;i<this->rowNum;i++)
	{
		float currentPivotValue = matrixPointer[i][pivotRow];
		for(int j=pivotRow; j<this->columnNum; j++)
		{
			matrixPointer[i][j] = matrixPointer[i][j] - currentPivotValue*matrixPointer[pivotRow][j];
		}
	}
};


void MyMatrix::gaussElim_ColmnPrin()
{
	int pivotRow;
	for(int i=0; i<this->rowNum; i++)
	{
		//cout << "LOOP:" << i << endl;

		pivotRow = this->findPivotRow(i);
		//cout << "PivotRow:" << pivotRow << endl;

		this->swapRow(i, pivotRow);
		//cout << "After swap"<< endl;
		//this->printMatrix();

		this->normalizePivotRow(i);
		//cout << "After Normalization" << endl;
		//this->printMatrix();

		this->eliminateSubMatrix(i);
		//cout << "After elimination" << endl;
		//this->printMatrix();
	}
};

bool MyMatrix::isAllZeroForArgumentRow(int arguRowNo)
{
	if(arguRowNo >= this->rowNum)
	{
		//cout << "Illegal input row No." << endl;
		return false;
	}

	return this->isAllZero(arguRowNo, true);
};

bool MyMatrix::isAllZeroForCoRow(int coRowNo)
{
	if(coRowNo >= this->rowNum)
	{
		//cout << "Illegal input row No." << endl;
		return false;
	}

	return this->isAllZero(coRowNo, false);
};

bool MyMatrix::isAllZero(int rowNo, bool isArgument)
{
	int length = this->columnNum;
	if(!isArgument)
	{
		length--;
	}

	int zeroNumber = 0;

	for(int i=0; i<length; i++)
	{
		if(this->matrixPointer[rowNo][i] == 0)
		{
			zeroNumber++;
		}
	}

	if(zeroNumber == length)
	{
		return true;
	}
	else
	{
		return false;
	}
};

int MyMatrix::coRank()
{
	int rank = 0;
	for(int i=0; i<this->rowNum; i++)
	{
		if(!isAllZeroForCoRow(i))
		{
			rank++;
		}
	}
	return rank;
};

int MyMatrix::arguRank()
{
	int rank = 0;
	for(int i=0; i<this->rowNum; i++)
	{
		if(!isAllZeroForArgumentRow(i))
		{
			rank++;
		}
	}
	return rank;
};

void MyMatrix::calcRoots()
{
	if(this->rowNum < this->columnNum-1)
	{
		//cout << "Do not have enough row to solve." << endl;
		return;
	}


	int arguRank = this->arguRank();
	int coRank = this->coRank();
	//cout << "Argument Rank of Matrix:" << arguRank << " . Co Rank of Matrix:" << coRank <<endl;
	if(arguRank > coRank)
	{
		//cout << "No root exist." <<endl;
		return;
	}

	if(arguRank == coRank)
	{
		//cout << "Only one root exist." <<endl;
	}

	for(int i=this->rowNum-1; i > 0; i--)
	{
		float temp = this->matrixPointer[i][this->columnNum-1];

		for(int j=i-1; j>=0; j--)
		{
			matrixPointer[j][this->columnNum-1] = matrixPointer[j][this->columnNum-1] - matrixPointer[j][i]*temp;
			matrixPointer[j][i] = 0;
		}
	}

	//cout << "Root resolved like below:" << endl;
	//this->printMatrix();
	this->roots[0] = matrixPointer[0][4];
	this->roots[1] = matrixPointer[1][4];
	this->roots[2] = matrixPointer[2][4];
	this->roots[3] = matrixPointer[3][4];
	//this->printMatrixRoots();

};

float* MyMatrix::getRoots()
{
	return this->roots;
};
