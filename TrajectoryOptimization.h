#ifndef TRAJECTORY_OPTIMIZATION_H
#define TRAJECTORY_OPTIMIZATION_H
#include <Eigen/Eigen> 

namespace AaltoGames
{

#ifdef SWIG
#define __stdcall  //SWIG doesn't understand __stdcall, but fortunately c# assumes it for virtual calls
#endif

class ITrajectoryOptimization
{
public:
	//returns total cost, including both control and state cost
	virtual double __stdcall getBestTrajectoryCost()=0;
	virtual void __stdcall getBestControl(int timeStep, double *out_control)=0;
	virtual void __stdcall startIteration(bool advanceTime, const double *initialState)=0;
	virtual void __stdcall startPlanningStep(int stepIdx)=0;
	//typically, this just returns sampleIdx. However, if there's been a resampling operation, multiple new samples may link to the same previous sample (and corresponding state)
	virtual int __stdcall getPreviousSampleIdx(int sampleIdx)=0;
	//samples a new control, considering an optional gaussian prior with diagonal covariance
	virtual void __stdcall getControl(int sampleIdx, double *out_control, const double *priorMean=0, const double *priorStd=0)=0;
	virtual void __stdcall updateResults(int sampleIdx, const double *finalControl, const double *newState, double stateCost, const double *priorMean=0, const double *priorStd=0)=0;
	virtual void __stdcall endPlanningStep(int stepIdx)=0;
	virtual void __stdcall endIteration()=0;
};


class MarginalSample
{
public:
	Eigen::VectorXd state;
	Eigen::VectorXd physicsState;
	Eigen::VectorXd previousState;
	Eigen::VectorXd previousPreviousState;
	Eigen::VectorXd control;
	Eigen::VectorXd previousControl;
	Eigen::VectorXd previousPreviousControl;
	//double weight;
	double forwardBelief;
	double belief;
	double fwdMessage;
	double bwdMessage;
	double stateCost;
	double originalStateCostFromClient;
	double statePotential;
	double fullCost;
	double costToGo;
	double fullControlCost;
	double controlCost;
	double controlPotential;
	double bestStateCost;
	double stateDeviationCost;
	double costSoFar; //argmin_i cost so far of segment i in previous step + transition cost (i.e., log of Gaussian kernel) + this segment's cost. Forward equivalent of costToGo
	double priorProbability;
	int fullSampleIdx;
	int previousMarginalSampleIdx;
	//only used in the no kernels mode
	int priorSampleIdx; 
	int nForks;
	bool physicsStateValid;
	void init(int nStateDimensions,int nControlDimensions){
		state.resize(nStateDimensions);
		previousState.resize(nStateDimensions);
		previousPreviousState.resize(nStateDimensions);
		control.resize(nControlDimensions);
		control.setZero();
		previousControl.resize(nControlDimensions);
		previousControl.setZero();
		//weight=1.0;
		fullCost=0.0;
		stateCost=0;
		originalStateCostFromClient=0;
		statePotential=1;
		fullControlCost=0;
		bestStateCost=0;
		controlCost=0;
		stateDeviationCost=0;
		belief=fwdMessage=bwdMessage=1;
		forwardBelief=1.0;
		nForks=0;
	}
};

} //AaltoGames


#endif //safeguard