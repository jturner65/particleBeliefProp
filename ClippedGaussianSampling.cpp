#include "ClippedGaussianSampling.h"
#include <math.h>
#include <vector>
#include "Mathutils.h"
#include "Debug.h"
#include "truncated_normal.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace AaltoGames
{

	class ClippedGaussianSampler{

	public:

		//Contructor, sets the default parameters
		ClippedGaussianSampler();

		//Sample a Gaussian random number between minValue and maxValue with the given mean and stdev.
		double sample(double mean, double stdev, double minValue, double maxValue );



	private:
		//Lookup table for the standard normal distribution cdf values.
		std::vector<double>	standardCdfTable;
		//Lookup table for the uniform distribution to inverse standardNormal distribution 
		std::vector<double> inverseStandardCdfTable;
		//The interval for the computation of the inverse cdf lookup table values
		double deltaInverse;
		//Lower limit for standardCdfTable
		double lowLim;
		//The interval for the computation of the cdf lookup table values
		double delta;
		//The upper limit for standardCdfTable
		double upLim;
		//standardCdfTable has the values from N(X < lowLim) to N(X < upLim). The values are computed with the interval dX = delta.

		//seed for truncated_normal library
		int seed;

		//This function computes the standard normal distribution cdf values to the table standardCdfTable
		void computeCdfTable(void);

		//This function computes the inverse standard normal distribution cdf values to the table standardCdfTable
		void computeinverseCdfTable(void);

		//If x ~ N(mean,stdDev) the function returns y ~ N(0,1)
		double nonstandardToStandard(double x,double mean, double stdDev);

		//If x ~ N(0,1) the function returns y ~ N(mean,stdDev)
		double standardToNonstandard(double x,double mean, double stdDev);

	};

	//This function computes the standard normal distribution cdf values to the table standardCdfTable
	//standardCdfTable has the values from N(X < lowLim) to N(X < upLim). The values are computed with the interval dX = delta.
	void ClippedGaussianSampler::computeCdfTable(void){
		standardCdfTable.clear();
		inverseStandardCdfTable.clear();
		double temp = 0.0;
		double uniTemp =0.0;
		double scalingConst = 1.0f/sqrtf(2.0f*(double)M_PI);
		for (double position = lowLim; position < upLim; position = position + delta){
			temp += delta*scalingConst*expf(-0.5f*position*position);
			while(uniTemp < temp){
				inverseStandardCdfTable.push_back(position-delta);
				uniTemp += deltaInverse;
			}
			standardCdfTable.push_back(temp);
		}
	}

	//Contructor, sets the default parameters
	ClippedGaussianSampler::ClippedGaussianSampler(){
		//Lower limit for standardCdfTable
		lowLim = -6.0;
		//The upper limit for standardCdfTable
		upLim = 6.0;
		//The interval for the computation of the values
		delta = 1.0/256.0;
		time_t timer;
		struct tm y2k = { 0 };
		double seconds;

		y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
		y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;

		time(&timer);

		seconds = difftime(timer, mktime(&y2k));
		srand((int)seconds);	//set random seed for rand()

		seed = abs((123456789 * rand() + 9999999999) % 9999999999);
		std::cout << "init clipped gauss seed : " << seed << "\n";
		deltaInverse = 1.0 / 2048.0;
		standardCdfTable.clear();
		inverseStandardCdfTable.clear();
	}

	//If x ~ N(mean,stdDev) the function returns y ~ N(0,1)
	double ClippedGaussianSampler::nonstandardToStandard(double x,double mean, double stdDev){
		return (x-mean)/stdDev;
	}

	//If x ~ N(0,1) the function returns y ~ N(mean,stdDev)
	double ClippedGaussianSampler::standardToNonstandard(double x,double mean, double stdDev){
		return (mean + x*stdDev);
	}

	//Clamp x between lower limit lowLim and upper limit upLim.
	double clamp(double x, double lowLim, double upLim){
		AALTO_ASSERT1(lowLim <= upLim);
		return clipMinMaxd(x, lowLim, upLim);
		//return std::max(lowLim,std::min(upLim,x));
	}

	std::mutex cpbp_mx;	//global mutex
	//Sample a Gaussian random number between minValue and maxValue with the given mean and stdev. This is done using cdf inverting.
	double ClippedGaussianSampler::sample(double mean, double stdev, double minValue, double maxValue ){//this is the only sampler used for the actual algorithm
		//If the lookup table is empty, populate it.

		//below is replacement JT_added
		std::lock_guard<std::mutex> lock(cpbp_mx);
		double res;
		mean = (mean < minValue ? minValue : (mean > maxValue ? maxValue : mean));
		res = truncated_normal_ab_sample(mean, stdev, minValue, maxValue, seed);
		//cdf = truncated_normal_ab_cdf(res, mean, sigma, a, b);
		//x2 = truncated_normal_ab_cdf_inv(cdf, mean, sigma, a, b);
		////Test that the number fullfils the requirements
		if (res < minValue) {//dbug
			//AALTO_ASSERT1(res >= minValue);
			std::cout << "Min Res violated in clipped gaussian : mean : " << mean << " std : " << stdev << " min v : " << minValue << " max v : " << maxValue << " res :" << res << "\n";
			res = clipMinMaxd(res, minValue, maxValue);
		}
		else if (res > maxValue) {
			//AALTO_ASSERT1(res <= maxValue);
			std::cout << "Max Res violated in clipped gaussian : mean : " << mean << " std : " << stdev << " min v : " << minValue << " max v : " << maxValue << " res :" << res << "\n";
			res = clipMinMaxd(res, minValue, maxValue);
		}
		return res;
		//end replacement JT_added

		//if (standardCdfTable.empty()){
		//	computeCdfTable();
		//}

		////Map the values to be used with standard normal distribution
		//double minValStd = nonstandardToStandard(minValue,mean,stdev);
		//double maxValStd = nonstandardToStandard(maxValue,mean,stdev);

		////Find the indices of the places corresponding to the minimum and maximum allowed value
		//int minPlace = (int)ceil( (minValStd - lowLim)/delta );
		//int maxPlace = (int)floor( (maxValStd - lowLim)/delta );

		////Find the standard normal distribution cdf values corresponding to the  minimum and maximum allowed value
		//minValStd = standardCdfTable[clipMinMaxi(minPlace,0,(int)(standardCdfTable.size()-1))]; 
		//maxValStd = standardCdfTable[clipMinMaxi(maxPlace,0,(int)(standardCdfTable.size()-1))];

		//double transRand, position;

		////Sample a uniformly distributed random number from interval [0,1]
		//transRand = ((double) rand() / (RAND_MAX));

		////Scale the random number appropriately
		//transRand = (maxValStd - minValStd)*transRand + minValStd;

		//int invCdfIndex = (int)(transRand/deltaInverse);

		//invCdfIndex=clipMinMaxi(invCdfIndex,0,inverseStandardCdfTable.size()-1); //to avoid index out of bounds errors on the next line
		//position = inverseStandardCdfTable[invCdfIndex];

		////Scale position properly to obtain a truncated Gaussian random number from the originally specified normal distribution
		//transRand = standardToNonstandard(position,mean,stdev);


		//////Position will correspond to the sampled value in standard normal distribution
		////double position = lowLim;
		//////Index for the cdf lookup table
		////int temp = 0;
		//////The table size
		////int tabSize = standardCdfTable.size();
		//////Find the standard normal distribution cdf value that is greater than the sampled and scaled uniformly distributed random number transRand.
		////while (standardCdfTable[temp] < transRand && temp < tabSize){
		////	temp++;
		////};
		//////Transform position to a truncated gaussian random number
		////position = position + temp*delta;
		//Test that the number fullfils the requirements
		//if (transRand < minValue) {//dbug
		//	//AALTO_ASSERT1(transRand >= minValue);
		//	cout << "Min Res violated in clipped gaussian : mean : " << mean << " std : " << stdev << " min v : " << minValue << " max v : " << maxValue << " transRand :" << transRand << "\n";
		//	transRand = clipMinMaxd(transRand, minValue, maxValue);
		//}
		//else if (transRand > maxValue) {
		//	//AALTO_ASSERT1(transRand <= maxValue);
		//	cout << "Max Res violated in clipped gaussian : mean : " << mean << " std : " << stdev << " min v : " << minValue << " max v : " << maxValue << " transRand :" << transRand << "\n";
		//	transRand = clipMinMaxd(transRand, minValue, maxValue);
		//}
		////Test that the number fullfils the requirements
		//AALTO_ASSERT1(transRand >= minValue);
		//AALTO_ASSERT1(transRand <= maxValue);

		////Clamp just in case of some doubleing point imprecision (the asserts above don't work in release builds)
		//transRand = clipMinMaxd(transRand,minValue,maxValue);

		////Return the value
		//return transRand;

	}

	static ClippedGaussianSampler s_sampler;
	double randGaussianClipped( double mean, double stdev, double minValue, double maxValue )
	{
		return s_sampler.sample(mean,stdev,minValue,maxValue);
	}

	double randGaussian( double mean, double stdev)
	{
		return s_sampler.sample(mean,stdev,mean-stdev*10.0f,mean+stdev*10.0f);
	}

}