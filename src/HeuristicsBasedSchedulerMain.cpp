//#define DEBUG
//#define PEAK_MINIMIZATION_PROBLEM
//#define COST_MINIMIZATION_PROBLEM

//#define PACMGF

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>  
#include <vector>
#include <algorithm>
#include <limits>

#include <unistd.h>
#include <getopt.h>
#include <string.h>

//#include <boost/lambda/lambda.hpp>

#include <momh/tlistset.h>
#include <momh/nondominatedset.h>

#include <math.h>
#include <time.h>
using namespace std;

#include "SchedulingProblemReader.h"
#include "Signal.h"
#include "Task.h"
#include "util.h"
#include "Tab.h"
#include "ILPScheduler.h"
#include "SchedulingProblemResult.h"

//#define L 20


void usage( void )
{
  puts( "Usage for heursec: heursec [--time-limit time_limit [--gap-limit gap_limit]]" );
  /* ... */
  exit( EXIT_FAILURE );
}



/*
 * The optString global tells getopt_long() which options we
 * support, and which options have arguments.
 * 
 * The longOpts global tells getopt_long() which long options
 * we support, and which long options have arguments.
 */

struct globalArgs_t {
  double timeLimit;				// -t option
  double gapLimit;                              // -g option
} globalArgs;

static const char *optString = "t:g:h?";

static const struct option longOpts[] = {
  { "time-limit", required_argument, NULL, 't' },
  { "gap-limit", required_argument, NULL, 'g'},
  { "help", no_argument, NULL, 'h' },
  { NULL, no_argument, NULL, 0 }
};

void print_args( void )
{
  printf("heursec is executed with the following parameters:\n");
  printf( "timeLimit: %f\n", globalArgs.timeLimit );
  printf( "gapLimit: %f\n", globalArgs.gapLimit );
}

bool verify_args()
{
  bool proper = true;
  if (globalArgs.gapLimit != 100 && globalArgs.timeLimit == -1)
    {
      cout << "Error: Specify either only time limit or both time limit and gap limit. e.g., -t 1 OR -t 1 -g 10" << endl;
      return false;
    }

  return proper;

}


int execute()
{
  //>>>>>>>>>>>>>>Sorting Tasks<<<<<<<<<<<<<<<<<//
  //TODO:sorting
  //	int a = 0;
  //	cout << "For sorting tasks according to their total energy\t\t press 1\n"
  //			"For sorting tasks according to their Load Profiles Peak Value\t press 2\n"
  //			"For sorting tasks according to their PR value\t\t\t press 3\n"
  //			"For NO sorting operation\t\t\t\t\t press -1 "<< endl;
  //	cin >> a;
  //
  //	printTasks(J);
  //	switch (a)
  //	{
  //	case 1:	sort(J.begin(), J.end(), sortTasksViaEnergy);	break;
  //	case 2:	sort(J.begin(), J.end(), sortTasksViaPeakValue);	break;
  //	case 3: sort(J.begin(), J.end(), sortTasksViaPreemption);	break;
  //	case -1: break;
  //	default: cout << "Wrong entry.. Try it again.." << endl;
  //	}
  //	printTasks(J);

  //Orders the Tasks according to their total energy! if J_1=5, J_2=10 ==> new ordering {J_2 , J_1}
  //	sort(J.begin(), J.end(), sortTasksViaEnergy);
  //Sorts the tasks according to their very own Peak Value! if J_1's power profile's peak value is highest then it will be the first.
  //	sort(J.begin(), J.end(), sortTasksViaPeakValue);
  //Sorts the tasks according to their preemption status; NP tasks goes first..
  //	sort(J.begin(), J.end(), sortTasksViaPreemption);



  // 	check their validity
  // 	TODO: put into the paper as a weak admittance test
  //	int max_d_i = checkTasksValidity(J, P_max);

  
  //vector<Signal<int>*>* minTab = new vector<Signal<int>*>();
  //double minCost = numeric_limits<double>::max();
  //double peakValue = numeric_limits<double>::max();


  // Read problem
  SchedulingProblemReader* spr = new SchedulingProblemReader();
  SchedulingProblem* sp = NULL;
  sp = spr->read();
  //  sp->print();

  /*  
  // Test sorting
  sp->print();
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaPeakValue
  sp->print();
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPeakValue sortTasksViaPeakValue
  sp->print();
  return 0;
  */

  // All the relevant results are stored in pr.
  SchedulingProblemResult pr( sp );

  // solution-details.txt stores the details of solutions, e.g. schedules
  ofstream foutRes("solution-peak-and-cost-minimization-details.txt");
  sp->print(foutRes);




  // STEP 0
  // - Find the result by ASAP
  // =======================================================================
  // >>>>>>>>>>>>>>>> ASAP<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Schdeduling as soon as task arrives.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  TNondominatedSet* pNondominatedSet0 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab0;
	
  // ASAP Scheduling
  sp->setPMax( numeric_limits<double>::max() );
  allocTab( NP_ASAP_SCHEDULING, P_ASAP_SCHEDULING, *(sp->J()), emptyTab0, pNondominatedSet0, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // Peak minTab 
  vector<Signal<int>*>* minPeakTab0 = NULL;
	
  if (pNondominatedSet0->iSetSize == 0)
    {
      pr.isFeasibleASAP = false;
      cout << "No feasible schedule found!" << endl;
    }
  else
    {
      pr.isFeasibleASAP = true;
      // pareto set contains 1 solution if feasible.
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) pNondominatedSet0->at(0);
      pr.minPeakASAP = msIt->getPeak();
      pr.minCostASAP = msIt->getCost();
      minPeakTab0 = msIt->getTab();
    }
	
  // write the result to file
  foutRes << "------------------------" << endl;
  foutRes << "Scheduling results by ASAP" << endl;
  foutRes << "------------------------" << endl;
  if(pr.isFeasibleASAP)
    {
      foutRes << "Min. peak: " << pr.minPeakASAP << endl;
      foutRes << "Min. cost: " << pr.minCostASAP << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab0);
    }
  else
    foutRes << "Infeasible solution by ASAP!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet0->DeleteAll();
  delete pNondominatedSet0;
  ////////////////////////////////////////////////////////////////////
  // End of ASAP - STEP 0 /////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////





#ifdef PEAK_MINIMIZATION_PROBLEM
  ///////////////////////////////////////////////
  // COST MINIMIZATION WITH PEAK CONSTRAINT //
  ///////////////////////////////////////////////


  // STEP 1
  // - First find optimum minimum peak without Pmax constraint by ILP
  // =======================================================================
  // >>>>>>>>>>>>>>>> ILP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================

  sp->setPMax( numeric_limits<double>::max() );
  ILPScheduler ilps(globalArgs.timeLimit, globalArgs.gapLimit, sp);  
  //  SchedulingSolution* minCostSol = ilps.schedule(MINIMIZE, COST);
  SchedulingSolution* minPeakSol = ilps.schedule(MINIMIZE, PEAK);
  SchedulingSolution* minPeakParetoSol = minPeakSol;

  if (minPeakSol->getStatus() == OPTIMAL_SOLUTION || minPeakSol->getStatus() == FEASIBLE_SOLUTION)
    {
      pr.isFeasibleILP = true;
      pr.minPeakOfPeakParetoILP = minPeakSol->getPeak();

      // Find the Pareto with min peak.
      minPeakParetoSol = ilps.schedule(MINIMIZE, COST, EQ, minPeakSol->getPeak() );
      pr.minCostOfPeakParetoILP = minPeakParetoSol->getCost();
    }
  else
    pr.isFeasibleILP = false;

  // write the result in file
  foutRes << "------------------------" << endl;
  foutRes << "Peak minimization by ILP" << endl;
  foutRes << "------------------------" << endl;
  minPeakParetoSol->print(foutRes);
  foutRes << endl;

  // Relevant results of the ILP solution
  // minCostSol->getCost();
  // minPeakSol->getPeak()


  // cout << *minCostSol << endl;
  // cout << *minPeakSol << endl;

  //delete minCostSol;
  delete minPeakSol;
  if (pr.isFeasibleILP)
    delete minPeakParetoSol;
  ////////////////////////////////////////////////////////////////////
  // End of ILP - STEP 1 /////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////




  // STEP 2
  // - Find the min peak obtained by Lee
  // =======================================================================
  // >>>>>>>>>>>>>>>> Lee <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Peak minimization with full tree search for NP and job pieces to min Ptot slots greedy heuristics for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet = new TListSet < SchedulingMOMHSolution >;
  //      MOS(J, emptyTab, pNondominatedSet, P_max, p_min, P_H);
  vector<Signal<int>*> emptyTab;
	
  // PACM: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // TODO: Sort jobs by Energy / (TODO for later: by Peak values)
  // allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // ASAP Scheduling
  //allocTab( NP_ASAP_SCHEDULING, P_ASAP_SCHEDULING, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // GG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // TODO: Sort jobs by Energy / (TODO for later: by Peak values)
  //allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // Lee: Peak minimization with full tree search for NP and job pieces to min Ptot slots greedy heuristics for P jobs.
  // Sorts the tasks according to their preemption status; NP tasks goes first..
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemption);
  sp->setPMax( numeric_limits<double>::max() );
  allocTab( NP_PEAK_MIN_WITH_TREE_SEARCH, P_PEAK_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
	
  double mCost = numeric_limits<double>::max();
  // Cost minTab
  //vector<Signal<int>*>* minCostTab = NULL;
  double mPeak = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minPeakTab = NULL;
	
  if (pNondominatedSet->iSetSize == 0)
    {
      pr.isFeasibleLee = false;
      cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasibleLee = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet->begin(); it != pNondominatedSet->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
	  
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mPeak > msIt->getPeak())
	{
	  mPeak = msIt->getPeak();
	  mCost = msIt->getCost();
	  minPeakTab = msIt->getTab();
	}
      //delete msIt;
    }
  pr.minPeakLee = mPeak;
  pr.minCostLee = mCost;

  // write the result to file
  foutRes << "------------------------" << endl;
  foutRes << "Peak minimization by Lee" << endl;
  foutRes << "------------------------" << endl;
  if(pr.isFeasibleLee)
    {
      foutRes << "Min. peak: " << pr.minPeakLee << endl;
      foutRes << "Min. cost: " << pr.minCostLee << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab);
    }
  else
    foutRes << "Infeasible solution by Lee!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet->DeleteAll();
  delete pNondominatedSet;
  ////////////////////////////////////////////////////////////////////
  // End of Lee - STEP 2 /////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////








  // STEP 3.1
  // - Find the peak by our PACMFG heuristic given Lee's peak as the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> PACMFG <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet2 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab2;
	
  // PACMFG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( pr.minPeakLee );

  double mPeak2 = numeric_limits<double>::max();
  double mCost2 = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minPeakTab2 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MIN_WITH_TREE_SEARCH3, *(sp->J()), emptyTab2, pNondominatedSet2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
  //allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab2, pNondominatedSet2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
    //double mCost = numeric_limits<double>::max();
    // Cost minTab
    //vector<Signal<int>*>* minCostTab = NULL;
	
  if (pNondominatedSet2->iSetSize == 0)
    {
      pr.isFeasiblePACMFG_wLee_Pmax = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasiblePACMFG_wLee_Pmax = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet2->begin(); it != pNondominatedSet2->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
      
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mPeak2 > msIt->getPeak())
	{
	  mPeak2 = msIt->getPeak();
	  mCost2 = msIt->getCost();
	  minPeakTab2 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakPACMFG_wLee_Pmax = mPeak2;
  pr.minCostPACMFG_wLee_Pmax = mCost2;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasiblePACMFG_wLee_Pmax = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Peak minimization by PACMFG with Pmax = Lee" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasiblePACMFG_wLee_Pmax)
    {
      foutRes << "Min. peak: " << pr.minPeakPACMFG_wLee_Pmax << endl;
      foutRes << "Min. cost: " << pr.minCostPACMFG_wLee_Pmax << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab2);
    }
  else
    foutRes << "Infeasible solution by PACMFG with Pmax = Lee!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet2->DeleteAll();
  delete pNondominatedSet2;
  ////////////////////////////////////////////////////////////////////
  // End of PACMFG - STEP 3.1 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////





  // STEP 3.2
  // - Find the peak by our PACMGG heuristic given Lee's peak as the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> PACMGG <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet2_2 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab2_2;
	
  // PACMGG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( pr.minPeakLee );

  double mPeak2_2 = numeric_limits<double>::max();
  double mCost2_2 = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minPeakTab2_2 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MIN_WITH_TREE_SEARCH3, *(sp->J()), emptyTab2_2, pNondominatedSet2_2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  if (pNondominatedSet2_2->iSetSize == 0)
    {
      pr.isFeasiblePACMGG_wLee_Pmax = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasiblePACMGG_wLee_Pmax = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet2_2->begin(); it != pNondominatedSet2_2->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
      
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mPeak2_2 > msIt->getPeak())
	{
	  mPeak2_2 = msIt->getPeak();
	  mCost2_2 = msIt->getCost();
	  minPeakTab2_2 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakPACMGG_wLee_Pmax = mPeak2_2;
  pr.minCostPACMGG_wLee_Pmax = mCost2_2;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasiblePACMGG_wLee_Pmax = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Peak minimization by PACMGG with Pmax = Lee" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasiblePACMGG_wLee_Pmax)
    {
      foutRes << "Min. peak: " << pr.minPeakPACMGG_wLee_Pmax << endl;
      foutRes << "Min. cost: " << pr.minCostPACMGG_wLee_Pmax << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab2_2);
    }
  else
    foutRes << "Infeasible solution by PACMGG with Pmax = Lee!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet2_2->DeleteAll();
  delete pNondominatedSet2_2;
  ////////////////////////////////////////////////////////////////////
  // End of PACMGG - STEP 3.2 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////




#ifdef PACMGF
  // STEP 3.3
  // - Find the peak by our PACMGF heuristic given Lee's peak as the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> PACMGF <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Cost minimization with Greedy search for NP jobs and peak to min price greedy heuristic for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet2_3 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab2_3;
	
  // PACMGF: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( pr.minPeakLee );

  double mPeak2_3 = numeric_limits<double>::max();
  double mCost2_3 = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minPeakTab2_3 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MIN_WITH_TREE_SEARCH2, *(sp->J()), emptyTab2_3, pNondominatedSet2_3, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  if (pNondominatedSet2_3->iSetSize == 0)
    {
      pr.isFeasiblePACMGF_wLee_Pmax = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasiblePACMGF_wLee_Pmax = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet2_3->begin(); it != pNondominatedSet2_3->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
      
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mPeak2_3 > msIt->getPeak())
	{
	  mPeak2_3 = msIt->getPeak();
	  mCost2_3 = msIt->getCost();
	  minPeakTab2_3 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakPACMGF_wLee_Pmax = mPeak2_3;
  pr.minCostPACMGF_wLee_Pmax = mCost2_3;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasiblePACMGF_wLee_Pmax = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Peak minimization by PACMGF with Pmax = Lee" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasiblePACMGF_wLee_Pmax)
    {
      foutRes << "Min. peak: " << pr.minPeakPACMGF_wLee_Pmax << endl;
      foutRes << "Min. cost: " << pr.minCostPACMGF_wLee_Pmax << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab2_3);
    }
  else
    foutRes << "Infeasible solution by PACMGF with Pmax = Lee!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet2_3->DeleteAll();
  delete pNondominatedSet2_3;
  ////////////////////////////////////////////////////////////////////
  // End of PACMGF - STEP 3.3 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////
#endif




  // // STEP 4
  // // - Find the peak by our PACM heuristic given ILP's optimum minimum peak as the Pmax constraint
  // // NOTE: step 4 is commented because it takes some time to do the tree search to find a solution as good as the ILP's.
  // // =======================================================================
  // // >>>>>>>>>>>>>>>> PACM <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // // =======================================================================
  // // Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // // =======================================================================

  // // define the problem
  // // fill in the global variables of MOMH lib.
  // // specify objectives
  // NumberOfObjectives = 2;
  // Objectives.resize(NumberOfObjectives);
  // // comp.time objective
  // Objectives[0].ObjectiveType = _Min;
  // Objectives[0].bActive = true;
  // // comm.cost objective
  // Objectives[1].ObjectiveType = _Min;
  // Objectives[1].bActive = true;

  // // constraints
  // NumberOfConstraints = 0;
  // Constraints.resize(NumberOfConstraints);

  // ////////////////////////////////////////////////////////////////////
  // TNondominatedSet* pNondominatedSet3 = new TListSet < SchedulingMOMHSolution >;
  // vector<Signal<int>*> emptyTab3;
	
  // // PACM: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // // Sort jobs by Energy / PeakValue / Preemption
  // sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak, sortTasksViaPreemptionAndFreedom
  // sp->setPMax( pr.minPeakOfPeakParetoILP );

  // double mPeak3 = numeric_limits<double>::max();
  // double mCost3 = numeric_limits<double>::max();
  // // Peak minTab 
  // vector<Signal<int>*>* minPeakTab3 = NULL;

  // //  try {
  // allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MIN_WITH_TREE_SEARCH3, *(sp->J()), emptyTab3, pNondominatedSet3, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
  // //allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab3, pNondominatedSet3, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
  //   //double mCost = numeric_limits<double>::max();
  //   // Cost minTab
  //   //vector<Signal<int>*>* minCostTab = NULL;
  	
  // if (pNondominatedSet3->iSetSize == 0)
  //   {
  //     pr.isFeasiblePACM_wILP_Pmax = false;
  // 	// cout << "No feasible schedule found!" << endl;
  //   }
  // else
  //   pr.isFeasiblePACM_wILP_Pmax = true;
	
  // for(std::vector<TSolution*>::iterator it=pNondominatedSet3->begin(); it != pNondominatedSet3->end(); it++)
  //   {
  //     SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
  // 	//cout << "Pareto\t" << *msIt << endl;
  // 	//printTab( *(msIt->getTab()) );
	  
  // 	/*if (mCost > msIt->getCost())
  // 	  {
  // 	  mCost = msIt->getCost();
  // 	  //minCostTab = msIt->getTab();
  // 	  }*/
  //     if (mPeak3 > msIt->getPeak())
  // 	{
  // 	  mPeak3 = msIt->getPeak();
  // 	  mCost3 = msIt->getCost();
  // 	  minPeakTab3 = msIt->getTab();
  // 	}  	    
  // 	//delete msIt;
  //   }
  // pr.minPeakPACM_wILP_Pmax = mPeak3;
  // pr.minCostPACM_wILP_Pmax = mCost3;

  // /*
  // } // END try
  // catch(InfeasibleSolutionException& snfe)
  //   {
  //     pr.isFeasiblePACM_wILP_Pmax = false;
  //   }
  // */

  // // write the result to file
  // foutRes << "-----------------------------------------" << endl;
  // foutRes << "Peak minimization by PACM with Pmax = ILP" << endl;
  // foutRes << "-----------------------------------------" << endl;
  // if(pr.isFeasiblePACM_wILP_Pmax)
  //   {
  //     foutRes << "Min. peak: " << pr.minPeakPACM_wILP_Pmax << endl;
  //     foutRes << "Min. cost: " << pr.minCostPACM_wILP_Pmax << endl;
  //     printSchedule(foutRes, *(sp->J()), *minPeakTab3);
  //   }
  // else
  //   foutRes << "Infeasible solution by PACM with Pmax = ILP!" << endl;
  // foutRes << endl;

  // // delete solutions. not needed because delete msIt above already deletes them.
  // pNondominatedSet3->DeleteAll();
  // delete pNondominatedSet3;
  // ////////////////////////////////////////////////////////////////////
  // // End of PACM - STEP 4 ////////////////////////////////////////////
  // ////////////////////////////////////////////////////////////////////


  // END of COST MINIMIZATION WITH PEAK CONSTRAINT PROBLEM
#endif









  // TODO delete this duplicate
  // STEP 2
  // - Find the min peak obtained by Lee
  // =======================================================================
  // >>>>>>>>>>>>>>>> Lee <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Peak minimization with full tree search for NP and job pieces to min Ptot slots greedy heuristics for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet = new TListSet < SchedulingMOMHSolution >;
  //      MOS(J, emptyTab, pNondominatedSet, P_max, p_min, P_H);
  vector<Signal<int>*> emptyTab;
	
  // PACM: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // TODO: Sort jobs by Energy / (TODO for later: by Peak values)
  // allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // ASAP Scheduling
  //allocTab( NP_ASAP_SCHEDULING, P_ASAP_SCHEDULING, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // GG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // TODO: Sort jobs by Energy / (TODO for later: by Peak values)
  //allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  // Lee: Peak minimization with full tree search for NP and job pieces to min Ptot slots greedy heuristics for P jobs.
  // Sorts the tasks according to their preemption status; NP tasks goes first..
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemption);
  sp->setPMax( numeric_limits<double>::max() );
  allocTab( NP_PEAK_MIN_WITH_TREE_SEARCH, P_PEAK_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
	
  double mCost = numeric_limits<double>::max();
  // Cost minTab
  //vector<Signal<int>*>* minCostTab = NULL;
  double mPeak = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minPeakTab = NULL;
	
  if (pNondominatedSet->iSetSize == 0)
    {
      pr.isFeasibleLee = false;
      cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasibleLee = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet->begin(); it != pNondominatedSet->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
	  
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mPeak > msIt->getPeak())
	{
	  mPeak = msIt->getPeak();
	  mCost = msIt->getCost();
	  minPeakTab = msIt->getTab();
	}
      //delete msIt;
    }
  pr.minPeakLee = mPeak;
  pr.minCostLee = mCost;

  // write the result to file
  foutRes << "------------------------" << endl;
  foutRes << "Peak minimization by Lee" << endl;
  foutRes << "------------------------" << endl;
  if(pr.isFeasibleLee)
    {
      foutRes << "Min. peak: " << pr.minPeakLee << endl;
      foutRes << "Min. cost: " << pr.minCostLee << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab);
    }
  else
    foutRes << "Infeasible solution by Lee!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet->DeleteAll();
  delete pNondominatedSet;
  ////////////////////////////////////////////////////////////////////
  // End of Lee - STEP 2 /////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////


  // TODO: To be moved above but now we just want the result of this.
  // STEP 2.5
  // - Find minimum cost with Pmax constraint = Lee by ILP
  // =======================================================================
  // >>>>>>>>>>>>>>>> ILP with peak = Lee <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  sp->setPMax( numeric_limits<double>::max() );
  ILPScheduler ilps_costmin_wLee_Pmax(globalArgs.timeLimit, globalArgs.gapLimit, sp);  
  // Find the min cost Pareto with peak <= Lee
  SchedulingSolution* minCostSol_wLee_Pmax = ilps_costmin_wLee_Pmax.schedule(MINIMIZE, COST, LEQ, pr.minPeakLee );

  if (minCostSol_wLee_Pmax->getStatus() == OPTIMAL_SOLUTION || minCostSol_wLee_Pmax->getStatus() == FEASIBLE_SOLUTION)
    {
      pr.isFeasibleILP_wLee_Pmax = true;
      pr.minPeakILP_wLee_Pmax = minCostSol_wLee_Pmax->getPeak();
      pr.minCostILP_wLee_Pmax = minCostSol_wLee_Pmax->getCost();
    }
  else
    pr.isFeasibleILP_wLee_Pmax = false;

  // write the result in file
  foutRes << "------------------------" << endl;
  foutRes << "Cost minimization with peak = Lee by ILP" << endl;
  foutRes << "------------------------" << endl;
  minCostSol_wLee_Pmax->print(foutRes);
  foutRes << endl;

  // Relevant results of the ILP solution
  // minCostSol->getCost();
  // minPeakSol->getPeak()


  // cout << *minCostSol << endl;
  // cout << *minPeakSol << endl;

  delete minCostSol_wLee_Pmax;
  ////////////////////////////////////////////////////////////////////
  // End of ILP cost minimization with peak = Lee - STEP 2.5 /////////
  ////////////////////////////////////////////////////////////////////








#ifdef COST_MINIMIZATION_PROBLEM
  ///////////////////////////////////////////////
  // COST MINIMIZATION WITHOUT PEAK CONSTRAINT //
  ///////////////////////////////////////////////

  // STEP 5
  // - First find optimum minimum cost without Pmax constraint by ILP
  // =======================================================================
  // >>>>>>>>>>>>>>>> ILP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================

  sp->setPMax( numeric_limits<double>::max() );
  ILPScheduler ilps_costmin(globalArgs.timeLimit, globalArgs.gapLimit, sp);  
  SchedulingSolution* minCostSol = ilps_costmin.schedule(MINIMIZE, COST);
  SchedulingSolution* minCostParetoSol = minCostSol;

  if (minCostSol->getStatus() == OPTIMAL_SOLUTION || minCostSol->getStatus() == FEASIBLE_SOLUTION)
    {
      pr.isFeasibleILP_costmin = true;
      pr.minCostOfCostParetoILP = minCostSol->getCost();

      // Find the Pareto with min cost.
      minCostParetoSol = ilps_costmin.schedule(MINIMIZE, PEAK, EQ, minCostSol->getCost() );
      pr.minPeakOfCostParetoILP = minCostParetoSol->getPeak();
    }
  else
    pr.isFeasibleILP_costmin = false;

  // write the result in file
  foutRes << "------------------------" << endl;
  foutRes << "Cost minimization by ILP" << endl;
  foutRes << "------------------------" << endl;
  minCostParetoSol->print(foutRes);
  foutRes << endl;

  delete minCostSol;
  if (pr.isFeasibleILP_costmin)
    delete minCostParetoSol;
  ////////////////////////////////////////////////////////////////////
  // End of ILP - STEP 5 /////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////





  // STEP 6.1
  // - Find the cost by our PACMFG heuristic without the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> PACMFG <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet6_1 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab6_1;
	
  // PACMFG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( numeric_limits<double>::max() );

  double mPeak6_1 = numeric_limits<double>::max();
  double mCost6_1 = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minCostTab6_1 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MIN_WITH_TREE_SEARCH3, *(sp->J()), emptyTab6_1, pNondominatedSet6_1, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
  //allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab2, pNondominatedSet2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
    //double mCost = numeric_limits<double>::max();
    // Cost minTab
    //vector<Signal<int>*>* minCostTab = NULL;
	
  if (pNondominatedSet6_1->iSetSize == 0)
    {
      pr.isFeasiblePACMFG_costmin = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasiblePACMFG_costmin = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet6_1->begin(); it != pNondominatedSet6_1->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
      
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mCost6_1 > msIt->getCost())
	{
	  mPeak6_1 = msIt->getPeak();
	  mCost6_1 = msIt->getCost();
	  minCostTab6_1 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakPACMFG_costmin = mPeak6_1;
  pr.minCostPACMFG_costmin = mCost6_1;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasiblePACMFG_costmin = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Cost minimization by PACMFG" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasiblePACMFG_costmin)
    {
      foutRes << "Min. peak: " << pr.minPeakPACMFG_costmin << endl;
      foutRes << "Min. cost: " << pr.minCostPACMFG_costmin << endl;
      printSchedule(foutRes, *(sp->J()), *minCostTab6_1);
    }
  else
    foutRes << "Infeasible solution by PACMFG!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet6_1->DeleteAll();
  delete pNondominatedSet6_1;
  ////////////////////////////////////////////////////////////////////
  // End of PACMFG - STEP 6.1 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////





  // STEP 6.2
  // - Find the cost by our PACMGG heuristic without the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> PACMGG <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet6_2 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab6_2;
	
  // PACMGG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( numeric_limits<double>::max() );

  double mPeak6_2 = numeric_limits<double>::max();
  double mCost6_2 = numeric_limits<double>::max();
  // Cost minTab 
  vector<Signal<int>*>* minCostTab6_2 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MIN_WITH_TREE_SEARCH3, *(sp->J()), emptyTab6_2, pNondominatedSet6_2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  if (pNondominatedSet6_2->iSetSize == 0)
    {
      pr.isFeasiblePACMGG_costmin = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasiblePACMGG_costmin = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet6_2->begin(); it != pNondominatedSet6_2->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
      
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mCost6_2 > msIt->getCost())
	{
	  mPeak6_2 = msIt->getPeak();
	  mCost6_2 = msIt->getCost();
	  minCostTab6_2 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakPACMGG_costmin = mPeak6_2;
  pr.minCostPACMGG_costmin = mCost6_2;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasiblePACMGG_costmin = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Cost minimization by PACMGG" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasiblePACMGG_costmin)
    {
      foutRes << "Min. peak: " << pr.minPeakPACMGG_costmin << endl;
      foutRes << "Min. cost: " << pr.minCostPACMGG_costmin << endl;
      printSchedule(foutRes, *(sp->J()), *minCostTab6_2);
    }
  else
    foutRes << "Infeasible solution by PACMGG!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet6_2->DeleteAll();
  delete pNondominatedSet6_2;
  ////////////////////////////////////////////////////////////////////
  // End of PACMGG - STEP 6.2 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////




#ifdef PACMGF
  // STEP 6.3
  // - Find the cost by our PACMGF heuristic without the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> PACMGF <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================
  // Cost minimization with Greedy search for NP jobs and peak to min price greedy heuristic for P jobs.
  // =======================================================================

  // define the problem
  // fill in the global variables of MOMH lib.
  // specify objectives
  NumberOfObjectives = 2;
  Objectives.resize(NumberOfObjectives);
  // comp.time objective
  Objectives[0].ObjectiveType = _Min;
  Objectives[0].bActive = true;
  // comm.cost objective
  Objectives[1].ObjectiveType = _Min;
  Objectives[1].bActive = true;

  // constraints
  NumberOfConstraints = 0;
  Constraints.resize(NumberOfConstraints);

  ////////////////////////////////////////////////////////////////////
  TNondominatedSet* pNondominatedSet6_3 = new TListSet < SchedulingMOMHSolution >;
  vector<Signal<int>*> emptyTab6_3;
	
  // PACMGF: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndFreedom); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( numeric_limits<double>::max() );

  double mPeak6_3 = numeric_limits<double>::max();
  double mCost6_3 = numeric_limits<double>::max();
  // Cost minTab 
  vector<Signal<int>*>* minCostTab6_3 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MIN_WITH_TREE_SEARCH2, *(sp->J()), emptyTab6_3, pNondominatedSet6_3, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

  if (pNondominatedSet6_3->iSetSize == 0)
    {
      pr.isFeasiblePACMGF_costmin = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasiblePACMGF_costmin = true;
	
  for(std::vector<TSolution*>::iterator it=pNondominatedSet6_3->begin(); it != pNondominatedSet6_3->end(); it++)
    {
      SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
      //cout << "Pareto\t" << *msIt << endl;
      //printTab( *(msIt->getTab()) );
      
      /*if (mCost > msIt->getCost())
	{
	mCost = msIt->getCost();
	//minCostTab = msIt->getTab();
	}*/
      if (mCost6_3 > msIt->getCost())
	{
	  mPeak6_3 = msIt->getPeak();
	  mCost6_3 = msIt->getCost();
	  minCostTab6_3 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakPACMGF_costmin = mPeak6_3;
  pr.minCostPACMGF_costmin = mCost6_3;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasiblePACMGF_wLee_Pmax = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Cost minimization by PACMGF" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasiblePACMGF_costmin)
    {
      foutRes << "Min. peak: " << pr.minPeakPACMGF_costmin << endl;
      foutRes << "Min. cost: " << pr.minCostPACMGF_costmin << endl;
      printSchedule(foutRes, *(sp->J()), *minCostTab6_3);
    }
  else
    foutRes << "Infeasible solution by PACMGF!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet6_3->DeleteAll();
  delete pNondominatedSet6_3;
  ////////////////////////////////////////////////////////////////////
  // End of PACMGF - STEP 6.3 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////
#endif

  // END OF COST MINIMIZATION PROBLEM
#endif









  // print the peak and cost minimization results
  pr.print(cout);

  // write the peak and cost minimization results in a file
  ofstream fout("solution-peak-and-cost-minimization.txt");
  fout << pr << endl;
  fout.close();

  // close details file
  foutRes.close();

  // ofstream fout("solution-cost-minimization.txt");

  // delete problem and problem reader
  delete sp;
  delete spr;






















  //        cout << "Minimum cost is " << minCost << endl;
  //        cout << "Peak Value of Minimum Costed P_tot is " << peakValue << endl;
  //	}


  // print 'P_i's for gnuplot to see it nicely
  //On the other hand this scheme will give us the graphic with scheduled tasks..
  //	for( unsigned int i = 0; i < J.size(); i++ )
  //	{
  //		Signal<double>* P__i = P_i( J[i]->getL(), minTab->at(i) );
  //		P__i->printIntoFile();
  //		delete P__i;
  //		//ONDEBUG( P__i->print() );
  //	}
  //	Signal<double>* P_tot = P__tot(*minTab, J);
  //	P_tot->printIntoFile();

  //===============================================================================================
  //>>>>>>>>>>>>>>>>>> Lee's Implementation <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //		LeesTab(J, emptyTab, &minCost, &peakValue, P_max, p_min, P_H, minTab);
  //		cout << "Minimum Peak Value is " << peakValue << endl;
  //		cout << "Cost of Ptot for Minimum Peak Value is " << minCost << endl;

  //===============================================================================================
  //>>>>>>>>>>>>>>>>>> Exhaustive Full Search <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //		fullSearch(J, P_max, p_min, P_H);
  //	}
  //	t3 = clock();

  // 	cout << "Time of our schedule in milliseconds " << (t2-t1)*1000/CLOCKS_PER_SEC << endl;
  // 	cout << "Time of old schedule in milliseconds " << (t3-t2)*1000/CLOCKS_PER_SEC << endl;

  //===============================================================================================
  //>>>>>>>>>>>>>>>>>> Minimized Cost Selective Imple. <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  //	CostSchedule csminBCS;
  //	csminBCS.C = numeric_limits<double>::max();
  //	csminBCS.S = emptyTab;

  //	bestCostSelectiveFunc(J, /*emptyTab,*/ csminBCS, &minCost, P_max, p_min, P_H);
  //	cout << "min Cost selection tree's Cost is " << minCost << endl;
}







int main( int argc, char *argv[] )
{
  int opt = 0;
  int longIndex = 0;
	
  /* Initialize globalArgs before we get to work. */
  globalArgs.timeLimit = -1;
  globalArgs.gapLimit = 100;
	
  /* Process the arguments with getopt_long(), then 
   * populate globalArgs. 
   */
  opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
  while( opt != -1 ) {
    switch( opt ) {
    case 't':
      globalArgs.timeLimit = atof(optarg);
      break;
    case 'g':
      globalArgs.gapLimit = atof(optarg);
      break;
    case 'h':	/* fall-through is intentional */
    case '?':
      usage();
      break;
	    	    
    default:	    /* You won't actually get here. */
      break;
    }
	  
    opt = getopt_long( argc, argv, optString, longOpts, &longIndex );
  }

  if( !verify_args() )
    {
      usage();
      return EXIT_FAILURE;
    }

  print_args();
	
  execute();

  return EXIT_SUCCESS;
}

