#define DEBUG

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>  
#include <vector>
#include <algorithm>
#include <limits>
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

int main()
{

  /** Test with given schedules
  double l_j1[4]  = {0.617624, 0.737464, 0.617624, 0.497784};
  double l_j2[18] = {0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621};
  double l_j3[10] = {1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185};
  double l_j4[15] = {1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993};
  double l_j5[4]  = {1.43065, 1.43065, 1.43065, 1.43065};
  double l_j6[12]  = {0.883876, 0.891267, 0.876485, 0.883876, 0.891267, 0.876485, 0.883876, 0.891267, 0.876485, 0.883876, 0.891267, 0.876485};
  double l_j7[1] = {1.20987};
  double l_j8[16]  = {1.19792, 1.28938, 1.19792, 1.10647, 1.19792, 1.28938, 1.19792, 1.10647, 1.19792, 1.28938, 1.19792, 1.10647, 1.19792, 1.28938, 1.19792, 1.10647};
  double l_j9[6] = {1.31315, 1.41079, 1.2155, 1.31315, 1.41079, 1.2155};
  double l_j10[20] = {1.04765, 1.19587, 1.13925, 0.956042, 0.899427, 1.04765, 1.19587, 1.13925, 0.956042, 0.899427, 1.04765, 1.19587, 1.13925, 0.956042, 0.899427, 1.04765, 1.19587, 1.13925, 0.956042, 0.899427};

  Signal<double> l1("l1", l_j1, L);
  Signal<double> l2("l2", l_j2, L);
  Signal<double> l3("l3", l_j3, L);
  Signal<double> l4("l4", l_j4, L);
  Signal<double> l5("l5", l_j5, L);
  Signal<double> l6("l6", l_j6, L);
  Signal<double> l7("l7", l_j7, L);
  Signal<double> l8("l8", l_j8, L);
  Signal<double> l9("l9", l_j9, L);
  Signal<double> l10("l10", l_j10, L);

  // ilp schedule
  int s_ilp_j1[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0};
  int s_ilp_j2[L] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0};
  int s_ilp_j3[L] = {1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j4[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
  int s_ilp_j5[L] = {1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j6[L] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
  int s_ilp_j7[L] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j8[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
  int s_ilp_j9[L] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j10[L]= {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  Signal<int> s1("j1", s_ilp_j1, L);
  Signal<int> s2("j2", s_ilp_j2, L);
  Signal<int> s3("j3", s_ilp_j3, L);
  Signal<int> s4("j4", s_ilp_j4, L);
  Signal<int> s5("j5", s_ilp_j5, L);
  Signal<int> s6("j6", s_ilp_j6, L);
  Signal<int> s7("j7", s_ilp_j7, L);
  Signal<int> s8("j8", s_ilp_j8, L);
  Signal<int> s9("j9", s_ilp_j9, L);
  Signal<int> s10("j10", s_ilp_j10, L);


  // FTSG Pmax=ILP schedule
  int s_ftsg2_j1[L] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j2[L] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0};
  int s_ftsg2_j3[L] = {1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j4[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
  int s_ftsg2_j5[L] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j6[L] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
  int s_ftsg2_j7[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j8[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
  int s_ftsg2_j9[L] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j10[L]= {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

  Signal<int> ss1("j1", s_ftsg2_j1, L);
  Signal<int> ss2("j2", s_ftsg2_j2, L);
  Signal<int> ss3("j3", s_ftsg2_j3, L);
  Signal<int> ss4("j4", s_ftsg2_j4, L);
  Signal<int> ss5("j5", s_ftsg2_j5, L);
  Signal<int> ss6("j6", s_ftsg2_j6, L);
  Signal<int> ss7("j7", s_ftsg2_j7, L);
  Signal<int> ss8("j8", s_ftsg2_j8, L);
  Signal<int> ss9("j9", s_ftsg2_j9, L);
  Signal<int> ss10("j10", s_ftsg2_j10, L);


  // ILP Ptot calc.
  Signal<double>* P_1 = P_i(&l1, &s1);
  Signal<double>* P_2 = P_i(&l2, &s2);
  Signal<double>* P_3 = P_i(&l3, &s3);
  Signal<double>* P_4 = P_i(&l4, &s4);
  Signal<double>* P_5 = P_i(&l5, &s5);
  Signal<double>* P_6 = P_i(&l6, &s6);
  Signal<double>* P_7 = P_i(&l7, &s7);
  Signal<double>* P_8 = P_i(&l8, &s8);
  Signal<double>* P_9 = P_i(&l9, &s9);
  Signal<double>* P_10 = P_i(&l10, &s10);

  Signal<double>* P_0 = new Signal<double>("all 0s", L, 0);
  Signal<double>* P_tot = P_0;
  P_tot = *P_tot + *P_1;
  P_tot = *P_tot + *P_2;
  P_tot = *P_tot + *P_3;
  P_tot = *P_tot + *P_4;
  P_tot = *P_tot + *P_5;
  P_tot = *P_tot + *P_6;
  P_tot = *P_tot + *P_7;
  P_tot = *P_tot + *P_8;
  P_tot = *P_tot + *P_9;
  P_tot = *P_tot + *P_10;
  cout << max(*P_tot) << endl;


  // FTSG Pmax=ILP Ptot calc.
  Signal<double>* PP_1 = P_i(&l1, &ss1);
  Signal<double>* PP_2 = P_i(&l2, &ss2);
  Signal<double>* PP_3 = P_i(&l3, &ss3);
  Signal<double>* PP_4 = P_i(&l4, &ss4);
  Signal<double>* PP_5 = P_i(&l5, &ss5);
  Signal<double>* PP_6 = P_i(&l6, &ss6);
  Signal<double>* PP_7 = P_i(&l7, &ss7);
  Signal<double>* PP_8 = P_i(&l8, &ss8);
  Signal<double>* PP_9 = P_i(&l9, &ss9);
  Signal<double>* PP_10 = P_i(&l10, &ss10);

  Signal<double>* PP_0 = new Signal<double>("all 0s", L, 0);
  Signal<double>* PP_tot = PP_0;
  PP_tot = *PP_tot + *PP_1;
  PP_tot = *PP_tot + *PP_2;
  PP_tot = *PP_tot + *PP_3;
  PP_tot = *PP_tot + *PP_4;
  PP_tot = *PP_tot + *PP_5;
  PP_tot = *PP_tot + *PP_6;
  PP_tot = *PP_tot + *PP_7;
  PP_tot = *PP_tot + *PP_8;
  PP_tot = *PP_tot + *PP_9;
  PP_tot = *PP_tot + *PP_10;
  cout << max(*PP_tot) << endl;

  PP_tot->printAsArray(cout);

  return 0;
*/

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
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndPeak); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaPeakValue
  sp->print();
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemptionAndPeak); // sortTasksViaPeakValue sortTasksViaPeakValue
  sp->print();
  return 0;
  */

  // All the relevant results are stored in pr.
  SchedulingProblemResult pr( sp->name() );

  // solution-details.txt stores the details of solutions, e.g. schedules
  ofstream foutRes("solution-peak-minimization-details.txt");
  sp->print(foutRes);

  // STEP 1
  // - First find optimum minimum peak without Pmax constraint by ILP
  // =======================================================================
  // >>>>>>>>>>>>>>>> ILP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // =======================================================================

  sp->setPMax( numeric_limits<double>::max() );
  ILPScheduler ilps(/*globalArgs.timeLimit, globalArgs.gapLimit,*/ sp);  
  //  SchedulingSolution* minCostSol = ilps.schedule(MINIMIZE, COST);
  SchedulingSolution* minPeakSol = ilps.schedule(MINIMIZE, PEAK);

  if (minPeakSol->getStatus() == OPTIMAL_SOLUTION || minPeakSol->getStatus() == FEASIBLE_SOLUTION)
    {
      pr.isFeasibleILP = true;
      pr.minPeakILP = minPeakSol->getPeak();
    }
  else
    pr.isFeasibleILP = false;

  // write the result in file
  foutRes << "------------------------" << endl;
  foutRes << "Peak minimization by ILP" << endl;
  foutRes << "------------------------" << endl;
  minPeakSol->print(foutRes);
  foutRes << endl;

  // Relevant results of the ILP solution
  // minCostSol->getCost();
  // minPeakSol->getPeak()


  // cout << *minCostSol << endl;
  // cout << *minPeakSol << endl;

  //delete minCostSol;
  delete minPeakSol;
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
	
  // FTSG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
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
	
	
  //double mCost = numeric_limits<double>::max();
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
	  minPeakTab = msIt->getTab();
	}
      //delete msIt;
    }
  pr.minPeakLee = mPeak;

  // write the result to file
  foutRes << "------------------------" << endl;
  foutRes << "Peak minimization by Lee" << endl;
  foutRes << "------------------------" << endl;
  if(pr.isFeasibleLee)
    {
      foutRes << "Min. peak: " << pr.minPeakLee << endl;
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






  // STEP 3
  // - Find the peak by our FTSG heuristic given Lee's peak as the Pmax constraint
  // =======================================================================
  // >>>>>>>>>>>>>>>> FTSG <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
	
  // FTSG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // Sort jobs by Energy / PeakValue / Preemption
  sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemption); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  sp->setPMax( pr.minPeakLee );

  double mPeak2 = numeric_limits<double>::max();
  // Peak minTab 
  vector<Signal<int>*>* minPeakTab2 = NULL;

  //  try {
  allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MIN_WITH_TREE_SEARCH, *(sp->J()), emptyTab2, pNondominatedSet2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
  //allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab2, pNondominatedSet2, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
    //double mCost = numeric_limits<double>::max();
    // Cost minTab
    //vector<Signal<int>*>* minCostTab = NULL;
	
  if (pNondominatedSet2->iSetSize == 0)
    {
      pr.isFeasibleFTSG_wLee_Pmax = false;
      //cout << "No feasible schedule found!" << endl;
    }
  else
    pr.isFeasibleFTSG_wLee_Pmax = true;
	
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
	  minPeakTab2 = msIt->getTab();
	}  	    
      //delete msIt;
    }
  pr.minPeakFTSG_wLee_Pmax = mPeak2;
  
  /*
  } // END try
  catch(InfeasibleSolutionException& snfe)
    {
      pr.isFeasibleFTSG_wLee_Pmax = false;
    }
  */

  // write the result to file
  foutRes << "-----------------------------------------" << endl;
  foutRes << "Peak minimization by FTSG with Pmax = Lee" << endl;
  foutRes << "-----------------------------------------" << endl;
  if(pr.isFeasibleFTSG_wLee_Pmax)
    {
      foutRes << "Min. peak: " << pr.minPeakFTSG_wLee_Pmax << endl;
      printSchedule(foutRes, *(sp->J()), *minPeakTab2);
    }
  else
    foutRes << "Infeasible solution by FTSG with Pmax = Lee!" << endl;
  foutRes << endl;

  // delete solutions. not needed because delete msIt above already deletes them.
  pNondominatedSet->DeleteAll();
  delete pNondominatedSet2;
  ////////////////////////////////////////////////////////////////////
  // End of FTSG - STEP 3 ////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////





  // // STEP 4
  // // - Find the peak by our FTSG heuristic given ILP's optimum minimum peak as the Pmax constraint
  // // =======================================================================
  // // >>>>>>>>>>>>>>>> FTSG <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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
	
  // // FTSG: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
  // // Sort jobs by Energy / PeakValue / Preemption
  // sort(sp->J()->begin(), sp->J()->end(), sortTasksViaPreemption); // sortTasksViaPreemption sortTasksViaPeakValue sortTasksViaEnergy sortTasksViaPreemptionAndPeak
  // sp->setPMax( pr.minPeakILP );

  // double mPeak3 = numeric_limits<double>::max();
  // // Peak minTab 
  // vector<Signal<int>*>* minPeakTab3 = NULL;

  // //  try {
  // allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MIN_WITH_TREE_SEARCH, *(sp->J()), emptyTab3, pNondominatedSet3, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
  // //allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab3, pNondominatedSet3, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
  //   //double mCost = numeric_limits<double>::max();
  //   // Cost minTab
  //   //vector<Signal<int>*>* minCostTab = NULL;
  	
  // if (pNondominatedSet3->iSetSize == 0)
  //   {
  //     pr.isFeasibleFTSG_wILP_Pmax = false;
  // 	// cout << "No feasible schedule found!" << endl;
  //   }
  // else
  //   pr.isFeasibleFTSG_wILP_Pmax = true;
	
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
  // 	  minPeakTab3 = msIt->getTab();
  // 	}  	    
  // 	//delete msIt;
  //   }
  // pr.minPeakFTSG_wILP_Pmax = mPeak3;

  // /*
  // } // END try
  // catch(InfeasibleSolutionException& snfe)
  //   {
  //     pr.isFeasibleFTSG_wILP_Pmax = false;
  //   }
  // */

  // // write the result to file
  // foutRes << "-----------------------------------------" << endl;
  // foutRes << "Peak minimization by FTSG with Pmax = ILP" << endl;
  // foutRes << "-----------------------------------------" << endl;
  // if(pr.isFeasibleFTSG_wILP_Pmax)
  //   {
  //     foutRes << "Min. peak: " << pr.minPeakFTSG_wILP_Pmax << endl;
  //     printSchedule(foutRes, *(sp->J()), *minPeakTab3);
  //   }
  // else
  //   foutRes << "Infeasible solution by FTSG with Pmax = ILP!" << endl;
  // foutRes << endl;

  // // delete solutions. not needed because delete msIt above already deletes them.
  // pNondominatedSet->DeleteAll();
  // delete pNondominatedSet3;
  // ////////////////////////////////////////////////////////////////////
  // // End of FTSG - STEP 4 ////////////////////////////////////////////
  // ////////////////////////////////////////////////////////////////////


  // print the peak minimization results
  pr.print();

  // write the peak minimization results in a file
  ofstream fout("solution-peak-minimization.txt");
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
