/*

Aalto University Game Tools license

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#ifndef EIGENMATHUTILS_H
#define EIGENMATHUTILS_H


#include <Eigen/Eigen> 

//Various math utilities depending on the Eigen library.
//Note that for maximum compatibility, there's also plain float array versions of may of the functions.

namespace AaltoGames
{
	void addEigenMatrixRow( Eigen::MatrixXd &m );

	void calcMeanWeighed(const double *input, const double *inputWeights, int vectorLength, int nVectors, Eigen::VectorXd &out_mean);
	//input vectors as columns (Eigen defaults to column major storage)
	void calcMeanWeighed(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::VectorXd &out_mean);
	void calcCovarWeighed(const double *input, const double *inputWeights, int vectorLength, int nVectors, Eigen::MatrixXd &out_covMat, const Eigen::VectorXd &mean);
	void calcCovarWeighed(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::MatrixXd &out_covMat, const Eigen::VectorXd &out_mean);
	void calcMeanAndCovarWeighed(const double *input, const double *inputWeights, int vectorLength, int nVectors, Eigen::MatrixXd &out_covMat, Eigen::VectorXd &out_mean);
	void calcMeanAndCovarWeighed(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::MatrixXd &out_covMat, Eigen::VectorXd &out_mean);
	//faster but not as accurate, as only single precision accumulator is used
	void calcMeanAndCovarWeighedVectorized(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::MatrixXd &out_covMat, Eigen::VectorXd &out_mean,Eigen::MatrixXd &temp);
	//Tries to find x for which the residuals are zero, regularized towards the xb (with residuals rb)
	//X contains samples of x as rows
	void gaussNewtonFromSamplesWeighed(const Eigen::VectorXd &xb, const Eigen::VectorXd &rb, const Eigen::MatrixXd &X, const Eigen::VectorXd &weights, const Eigen::VectorXd &residuals, float regularization, Eigen::VectorXd &out_result);

}//AaltoGames


#endif
