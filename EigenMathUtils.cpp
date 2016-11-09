#include "EigenMathUtils.h"
#include "Debug.h"
using namespace Eigen;
namespace AaltoGames
{
	

	void addEigenMatrixRow( Eigen::MatrixXd &m )
	{
		Eigen::MatrixXd temp=m;
		m.resize(m.rows()+1,m.cols());
		m.setZero();
		m.block(0,0,temp.rows(),temp.cols())=temp;
	}

	void calcMeanWeighed(const double *input, const double *inputWeights, int vectorLength, int nVectors, Eigen::VectorXd &out_mean)
	{
		//compute mean
		for (int j=0; j<vectorLength; j++){
			double avg=0;
			double wSum=0;
			for (int i=0; i<nVectors; i++){
				double w=inputWeights[i];
				avg+=w*input[i*vectorLength+j];
				wSum+=w;
			}
			avg/=wSum;
			out_mean[j]=(float)avg;
		}
	}
	//input vectors as columns (Eigen defaults to column major storage)
	void calcMeanWeighed(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::VectorXd &out_mean)
	{
		AALTO_ASSERT1(!input.IsRowMajor);
		calcMeanWeighed(input.data(),inputWeights.data(),input.rows(),input.cols(),out_mean);
	}
	void calcCovarWeighed(const double *input, const double *inputWeights, int vectorLength, int nVectors, Eigen::MatrixXd &out_covMat, const Eigen::VectorXd &mean)
	{
		//compute covariance matrix
		out_covMat.setZero();

		//Eigen::MatrixXd inMat = Eigen::MatrixXd::Zero(vectorLength,nVectors);
		//for (int i=0;i<vectorLength;i++){
		//	for(int k=0;k<nVectors;k++){
		//		inMat(i,k) = ((float)inputWeights[k])*(input[k*vectorLength+i]-mean[i]);
		//	}
		//}

		//out_covMat = (inMat*inMat.transpose())*(1.0/((float)nVectors-1.0));

		for (int i=0; i<vectorLength; i++){
			for (int j=0; j<vectorLength; j++){
				double avg=0;
				double iMean=mean[i];
				double jMean=mean[j];
				double wSum=0;
				for (int k=0; k<nVectors; k++){
					double w=inputWeights[k];
					avg+=w*(input[k*vectorLength+i]-iMean)*(input[k*vectorLength+j]-jMean);
					wSum+=w;
				}
				avg/=wSum;
				out_covMat(i,j)=(float)avg;
			}
		}
	}
	void calcCovarWeighed(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::MatrixXd &out_covMat, const Eigen::VectorXd &out_mean)
	{
		calcCovarWeighed(input.data(),inputWeights.data(),input.rows(),input.cols(),out_covMat,out_mean);
	}
	void calcMeanAndCovarWeighed(const double *input, const double *inputWeights, int vectorLength, int nVectors, Eigen::MatrixXd &out_covMat, Eigen::VectorXd &out_mean)
	{
		calcMeanWeighed(input,inputWeights,vectorLength,nVectors,out_mean);
		calcCovarWeighed(input,inputWeights,vectorLength,nVectors,out_covMat,out_mean);
	}
	//input vectors as columns (Eigen defaults to column major storage)
	void calcMeanAndCovarWeighed(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::MatrixXd &out_covMat, Eigen::VectorXd &out_mean)
	{
		calcMeanAndCovarWeighed(input.data(),inputWeights.data(),input.rows(),input.cols(),out_covMat,out_mean);

	}

	void calcMeanAndCovarWeighedVectorized(const Eigen::MatrixXd &input, const Eigen::VectorXd &inputWeights, Eigen::MatrixXd &out_covMat, Eigen::VectorXd &out_mean,Eigen::MatrixXd &temp)
	{
		out_mean=input.col(0); //to resize
		out_mean.setZero();
		double wSumInv=1.0/inputWeights.sum();
		for (int k=0;k<inputWeights.size();k++){
			double w=inputWeights[k];
			out_mean+=input.col(k)*(float)(w*wSumInv);
		}
		out_mean = input.rowwise().mean();
		temp = (input.colwise() - out_mean);
		for (int k=0;k<inputWeights.size();k++){
			temp.col(k) *= (float)(sqrt(inputWeights(k)*wSumInv));	//using square roots, as we only want the normalized weights to be included once for each result element in the multiplication below
		}
		out_covMat = temp*temp.transpose();
	}


	void gaussNewtonFromSamplesWeighed(const Eigen::VectorXd &xb, const Eigen::VectorXd &rb, const Eigen::MatrixXd &X, const Eigen::VectorXd &weights, const Eigen::VectorXd &residuals, float regularization, Eigen::VectorXd &out_result)
	{
		//Summary:
		//out_result=xb - G rb
		//xb is the best sample, rb is the best sample residual vector
		//G=AB'inv(BB'+kI)
		//A.col(i)=weights[i]*(X.row(i)-best sample)'
		//B.col(i)=weights[i]*(residuals - rb)'
		//k=regularization

		//Get xb, r(xb)
		//cv::Mat xb=X.row(bestIndex);
		//cv::Mat rb=residuals.row(bestIndex);

		//Compute A and B
		MatrixXd A=X.transpose();
		MatrixXd B=residuals.transpose();
		for (int i=0; i<A.cols(); i++)
		{
			A.col(i)=weights[i]*(X.row(i).transpose()-xb);
			B.col(i)=weights[i]*(residuals.row(i).transpose()-rb);
		}
		MatrixXd I=MatrixXd::Identity(B.rows(),B.rows());
		I=I*regularization;
		MatrixXd G=(A*B.transpose())*(B*B.transpose()+I).inverse();
		out_result=xb - G * rb;
	}

}//AaltoGames