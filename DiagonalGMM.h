/*

Aalto University Game Tools license

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef DIAGONALGMM_H
#define DIAGONALGMM_H
#include <Eigen/Eigen> 
#include <vector>


namespace AaltoGames
{
	class DynamicPdfSampler;  //forward declare

	class DiagonalGMM
	{
	public:
		DiagonalGMM();
		//Initialize a diagonal Gaussian mixture model with the given weights, means and covariance matrices.
		//The each row of the 'means' matrix is a mean vector of a mixture component
		//Only the diagonals in the 'cov' matrices are used.
		DiagonalGMM(const Eigen::VectorXd& weights,const Eigen::MatrixXd& means, const std::vector<Eigen::MatrixXd>& cov);
		//Initialize a diagonal Gaussian mixture model with the given weights, means and covariance matrices.
		//The each row of the 'means' matrix is a mean vector of a mixture component
		DiagonalGMM(const Eigen::VectorXd& weights,const Eigen::MatrixXd& means, const std::vector<Eigen::VectorXd>& cov);
		void resize(int nComponents, int nDimensions);
		void resample(DiagonalGMM &dst, int nDstComponents);
		void copyFrom(DiagonalGMM &src);
		int sampleComponent();
		void sample(Eigen::VectorXd &dst);
		//Sample within limits. Note that this is only an approximation, as the component normalizing constants are currently computed without limits
		void sampleWithLimits( Eigen::VectorXd &dst, const Eigen::VectorXd &minValues, const Eigen::VectorXd &maxValues  );
		void sampleWithLimits( Eigen::Map<Eigen::VectorXd> &dst, const Eigen::VectorXd &minValues, const Eigen::VectorXd &maxValues  );

		//Call this after manipulating the weights vector. Normalizes the weights and updates the internal data for sampling from the GMM
		void weightsUpdated();
		//inits the GMM with a single component of infinite variance
		//		void setUniform(int nDimensions);
		void setStds(const Eigen::VectorXd &src);
		double p(const Eigen::VectorXd &v);
		std::vector<Eigen::VectorXd> mean;
		std::vector<Eigen::VectorXd> std;
		//note: after you manipulate the weights, call weigthsUpdated() to normalize them and update the internal data needed for the sampling functions
		Eigen::VectorXd weights;
		//Note: src1 and src2 may also have 0 or DBL_MAX as std, corresponding to fixed or unconstrained variables.
		static void multiply(DiagonalGMM &src1, DiagonalGMM &src2, DiagonalGMM &dst);
		//fixedVars contains all the known variables (the lowest indices). Returns the sum of weights before normalizing to 1
		double makeConditional(const Eigen::VectorXd& fixedVars, DiagonalGMM &dst);
	protected:
		DynamicPdfSampler *sampler;
		int nDimensions;
		int nComponents;
	};

} //AaltoGames


#endif //DIAGONALGMM_H