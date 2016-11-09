/*

Aalto University Game Tools license

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#ifndef DYNAMICPDFSAMPLER_H
#define DYNAMICPDFSAMPLER_H 
#include <vector>
#include "MathUtils.h"

namespace AaltoGames
{

	///Sampler for sampling from a discrete and dynamically changing pdf.
	///This is implemented using a tree structure where the discrete probabilities are propagated towards the root
	class DynamicPdfSampler
	{
	public:
		DynamicPdfSampler(int N, DynamicPdfSampler *parent=NULL);
		~DynamicPdfSampler();
		void setDensity(int idx, double density);
		double getDensity(int idx);
		void addUniformBias(float uniformSamplingProbability);
		int __fastcall sample();
		void normalize(double sum=1.0);
	protected:
		DynamicPdfSampler *children[2];
		DynamicPdfSampler *parent;
		DynamicPdfSampler *root;
		double probability;
		bool hasChildren;
		int elemIdx;
		std::vector<DynamicPdfSampler *> leaves;
	};

}

#endif