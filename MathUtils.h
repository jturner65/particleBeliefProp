/*

Aalto University Game Tools license

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <math.h>
#include <stdlib.h>
#include <limits>
#include <float.h>


//for new random code
#include <random>
#include <time.h>
#include <thread>

namespace AaltoGames{


	template<class X> inline X squared(X x)
	{
		return x*x;
	}


	///Swaps a and b
	template<class X> inline void _swap(X &a, X &b){
		X temp=a;
		a=b;
		b=temp;
	}



	///Clip val in the range minVal...maxVal (double version)
	static inline double clipMinMaxd(double val, double minVal, double maxVal){
		if (val<minVal)
			return minVal;
		if (val>maxVal)
			return maxVal;
		return val;
	}


	///Clip val in the range minVal...maxVal (floating point version)
	static inline float clipMinMaxf(float val, float minVal, float maxVal){
		//if (_isnan(val) || !_finite(val))
		//	val=0.5*(minVal+maxVal);
		if (val<minVal)
			return minVal;
		if (val>maxVal)
			return maxVal;
		return val;
	}

	///Clip val in the range minVal...maxVal (integer version)
	static inline int clipMinMaxi(int val, int minVal, int maxVal){
		if (val<minVal)
			return minVal;
		if (val>maxVal)
			return maxVal;
		return val;
	}


	// define _min and _max if they have not been defined yet!
#ifndef _min
#define _min(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef _max
#define _max(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef _fsign
#define _fsign(x) (x<0 ? -1.0 : 1.0)
#endif // fsign

#ifndef _sign
#define _sign(x) (x<0 ? -1 : 1)
#endif //sign


	template<class X> inline X min3(X x, X y, X z)
	{
		X result=_min(x,y);
		result=_min(result,z);
		return result;
	}

	template<class X> inline X max3(X x, X y, X z)
	{
		X result=_max(x,y);
		result=_max(result,z);
		return result;
	}

	///Returns false if val is infinite or NaN
	static inline bool validFloat(float val){
		if (!(val==std::numeric_limits<float>::infinity())
			&& !(val==-std::numeric_limits<float>::infinity())
			&& val==val
			){
				return true;
		}
		else{ 
			return false;
		}
	}

	static inline bool validDouble(double val){
		if (!(val==std::numeric_limits<double>::infinity())
			&& !(val==-std::numeric_limits<double>::infinity())
			&& val==val
			){
				return true;
		}
		else{ 
			return false;
		}
	}

	///Returns zero if val is denormalized, useful for optimization
	static inline float clipDenormalizedToZero(float flt){
		if ( !(flt != 0 && fabsf( flt ) < std::numeric_limits<float>::min() )) 
		{
			return flt;
		}
		else
		{
			return 0;
		}
	}

	static inline double clipDenormalizedToZero(double flt){
		if ( !(flt != 0 && fabs( flt ) < std::numeric_limits<double>::min() )) 
		{
			return flt;
		}
		else
		{
			return 0;
		}
	}
//FROM STACK OVERFLOW http://stackoverflow.com/questions/21237905/how-do-i-generate-thread-safe-uniform-random-numbers
//thread safe random numbers
//#if defined (_MSC_VER)  // Visual studio
//#define thread_local __declspec( thread )
//#elif defined (__GCC__) // GCC
//#define thread_local __thread
//#endif



	/* Thread-safe function that returns a random number between min and max (inclusive).
	This function takes ~142% the time that calling rand() would take. For this extra
	cost you get a better uniform distribution and thread-safety. */
	static int intRand(const int min, const int max) {
		static __declspec(thread) std::mt19937* generatorI = nullptr;
		if (!generatorI) generatorI = new std::mt19937(clock() + std::hash<std::thread::id>()(std::this_thread::get_id()));// std::this_thread::get_id().hash());
		std::uniform_int_distribution<int> distribution(min, max);
		return distribution(*generatorI);
	}

	static double doubleRand(const double min, const double max) {
		static __declspec(thread) std::mt19937* generator = nullptr;
		if (!generator) generator = new std::mt19937(clock() + std::hash<std::thread::id>()(std::this_thread::get_id()));// std::this_thread::get_id().hash());
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(*generator);
	}

	static inline int rand01()
	{
		return rand()/(RAND_MAX/2+1);
	}

	static inline float randomf()
	{
		return ((float)rand())/(float)RAND_MAX;
	}

	static inline double random()
	{
		double randVal = doubleRand(0.0, 1.0);
		return randVal;
		//return ((double)rand())/(double)RAND_MAX;
	}

	//range is inclusive
	static inline int randInt(int rangeMin, int rangeMax)
	{
		int randVal = intRand(rangeMin, rangeMax);
		return randVal;
		//return rangeMin+(rand() % (rangeMax-rangeMin+1));
	}

#ifndef F_PI
#define F_PI 3.14159265359f
#endif

	//Compute the univariate normal distribution that is the product of the two given
	//normal distributions.
	static inline void productNormalDist(float mean0, float stdev0, float mean1, float stdev1, float &mean01, float &stdev01){
		float var0=stdev0*stdev0;
		float var1=stdev1*stdev1;
		float denom=1.0f/(var1+var0);
		mean01=(var0*mean1+var1*mean0)*denom;
		stdev01=sqrtf(var1*var0*denom);
	}


	//Compute the univariate normal distribution that is the product of the two given
	//normal distributions.JT_Added - doubles
	static inline void productNormalDist(double mean0, double stdev0, double mean1, double stdev1, double &mean01, double &stdev01) {
		double var0 = stdev0*stdev0;
		double var1 = stdev1*stdev1;
		double denom = 1.0f / (var1 + var0);
		mean01 = (var0*mean1 + var1*mean0)*denom;
		stdev01 = sqrtf(var1*var0*denom);
	}


	static inline double gaussianPdf(int N, const float *x, const float *mean, const float *std)
	{
		double acc=0;
		double scale=1;
		for (int i=0; i<N; i++)
		{
			scale *= std[i] * sqrt(2 * F_PI);
			acc+=squared(x[i]-mean[i])/squared(std[i]);
		}
		return exp(-0.5*acc)/scale;
	}

	static const float deg2rad = 2.0f*F_PI / 360.0f;


}//AaltoGames


#endif //MATHUTILS_H