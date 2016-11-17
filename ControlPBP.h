/*

Aalto University Game Tools license

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


#ifndef CONTROLPBP_H
#define CONTROLPBP_H
#include <Eigen/Eigen> 
#include <vector>
#include "DiagonalGMM.h"
#include "DynamicPdfSampler.h"
#include "TrajectoryOptimization.h"

#ifdef SWIG
#define __stdcall  //SWIG doesn't understand __stdcall, but fortunately c# assumes it for virtual calls
#endif


namespace AaltoGames
{
	//Defines how controls affect system state at each step 
	typedef void (*TransitionFunction)(int step, const double *state, const double *controls, double *out_state);
	//Evaluates the cost for a state, e.g., penalizes not hitting a target. Returned in the form of squared cost, which can be transformed into a potential as exp(-sqCost) by probabilistic optimizers.
	typedef double (*StateCostFunction)(int stepIdx, const double *state);
	//Optional notification callback after the update algorithm has processed a single step (node) of the graph
	typedef void (*OnStepDone)();
	
	//Implements the C-PBP algorithm as in the Hämäläinen et al. 2015 paper "Online Control of Simulated Humanoids Using Particle Belief Propagation"
	//Note: although we use Eigen vectors internally, the public interface uses only plain double arrays for maximal portability
	class ControlPBP : public ITrajectoryOptimization
	{
	public:
		ControlPBP();
		~ControlPBP();
		//minValues and maxValues contain first the bounds for state variables, then for control variables
		//stateKernelStd==NULL corresponds to the special case of Q=0
		//Note that instead of specifying the Q and sigmas of the paper, the values are provided as float standard deviations corresponding to the diagonal elements, 
		//controlPriorStd=sqrt(diag. of \sigma_{0}^2 C_u), controlPriorDiffStd = sqrt(diag. of \sigma_{1}^2 C_u), controlPriorDiffDiffStd = sqrt(diag. of \sigma_{2}^2 C_u)
		void init(int nParticles, int nSteps, int nStateDimensions, int nControlDimensions, const double *controlMinValues, const double *controlMaxValues, const double *controlMean, const double *controlPriorStd, const double *controlDiffPriorStd, const double *controlDiffDiffPriorStd, double controlMutationStdScale, const double *stateKernelStd, int nMORComponents=0);
		//Note that this is just a convenience method that internally calls startIteration(), getControl() etc.
		virtual void __stdcall update(const double *currentState, TransitionFunction transitionFwd, StateCostFunction statePotential, OnStepDone onStepDone=NULL);
		virtual double __stdcall getBestTrajectoryCost();
		virtual void __stdcall getBestControl(int timeStep, double *out_control);
		virtual void __stdcall getBestControlState( int timeStep, double *out_state );
		//Returns the original state cost for the best trajectory passed from the client to ControlPBP for the given timestep. This is mainly for debug and testing purposes.
		virtual double __stdcall getBestTrajectoryOriginalStateCost( int timeStep);
		virtual void __stdcall setSamplingParams(const double *controlPriorStd, const double *controlDiffPriorStd, const double *controlDiffDiffPriorStd, double controlMutationStdScale, const double *stateKernelStd);

		//returns the prior GMM for the given time step
#ifndef SWIG
		void getConditionalControlGMM(int timeStep, const Eigen::Ref<const Eigen::VectorXd>&state, DiagonalGMM &dst);
#endif
		/*
		Below, an interface for operation without callbacks. This is convenient for C# integration and custom multithreading. See InvPendulumTest.cpp for the usage.
		*/
		virtual void __stdcall startIteration(bool advanceTime, const double *initialState);
		virtual void __stdcall startPlanningStep(int stepIdx);
		//typically, this just returns sampleIdx. However, if there's been a resampling operation, multiple new samples may link to the same previous sample (and corresponding state)
		virtual int __stdcall getPreviousSampleIdx(int sampleIdx);
		//samples a new control, considering an optional gaussian prior with diagonal covariance (this corresponds to the \mu_p, \sigma_p, C_u in the paper, although the C_u is computed on Unity side and raw stdev and mean arrays are passed to the optimizer)
		virtual void __stdcall getControl(int sampleIdx, double *out_control, const double *priorMean=0, const double *priorStd=0);
		virtual void __stdcall updateResults(int sampleIdx, const double *finalControl, const double *newState, double squaredStateCost, const double *priorMean=0, const double *priorStd=0);
		virtual void __stdcall endPlanningStep(int stepIdx);
		virtual void __stdcall endIteration();
		//uniformBias: portion of no-prior samples in the paper (0..1)
		//resampleThreshold: resampling threshold, same as in the paper. Default 0.5
		//useGaussianBackPropagation: if true, the Gaussian local refinement (Algorithm 2 of the paper) is used. 
		//gbpRegularization: the regularization of Algorithm 2. Default 0.001 
		virtual void __stdcall setParams(double uniformBias, double resampleThreshold, bool useGaussianBackPropagation,double gbpRegularization);
		//this function is public only for debug drawing
		void gaussianBackPropagation(TransitionFunction transitionFwd=NULL, OnStepDone onStepDone=NULL);
		virtual int __stdcall getBestSampleLastIdx();
		double getGBPRegularization();
		//Eigen::MatrixXd marginalDataToMatrix(void);
		//Eigen::MatrixXd stateAndControlToMatrix(void);

		//a vector of MarginalSamples for each graph node, representing a GMM of joint state and control 
		//This is made public for easier debug visualizations, but you should treat this as read-only data.
#ifndef SWIG
		std::vector<std::vector<MarginalSample> > marginals;
#endif
	private:
		virtual void __stdcall getControlWithoutStateKernel(int sampleIdx, double *out_control, const double *priorMean=0, const double *priorStd=0);
		//The transitions in a matrix. Each row has one transition. The first nStateDimensions columns have the previous state, the next nStateDimensions have the current state, the next nControlDimensions have the controls of the transition.
		Eigen::MatrixXd transitionData;


		std::vector<Eigen::VectorXd> fullSamples;
		std::vector<MarginalSample> oldBest;
		std::vector<double> fullSampleCosts;
		double bestCost;
		Eigen::VectorXd gaussianBackPropagated;
		std::vector<DiagonalGMM> prior;
		Eigen::VectorXd controlMin,controlMax,controlMean, controlPriorStd,controlDiffPriorStd,controlDiffDiffPriorStd,controlMutationStd,stateKernelStd;
		int nSteps;
		DiagonalGMM staticPrior;
		int nSamples;
		int nStateDimensions;
		int nControlDimensions;
		int iterationIdx;
		bool resample;
		int currentStep;
		int nextStep;
		int nInitialGuesses;
		bool useStateKernels;
		int bestFullSampleIdx;
		bool timeAdvanced;

		DynamicPdfSampler *selector;
		DynamicPdfSampler *conditionalSelector;
		void finalizeWeights(int stepIdx);
		double uniformBias;
		double resampleThreshold;  //0 = never resample, >1 = always resample
		bool useGaussianBackPropagation;
		double gbpRegularization;

	};

} //namespace AaltoGames


#endif //CONTROLPBP_H


