//#define DEBUG

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

#include "SchedulingProblemReader.h"
#include "Signal.h"
#include "Task.h"
#include "util.h"
#include "Tab.h"
#include "MOS.h"
#include <math.h>
#include <time.h>
using namespace std;

int main()
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
  //sp->print();


//===============================================================================================
//>>>>>>>>>>>>>>>>>> Our heuristic <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//===============================================================================================
//>>>>>>>>>>>>>>>>>> Multi-Objective Scheduling <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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

        TNondominatedSet* pNondominatedSet = new TListSet < SchedulingMOMHSolution >;
//      MOS(J, emptyTab, pNondominatedSet, P_max, p_min, P_H);
	vector<Signal<int>*> emptyTab;
	sp->setPMax( sp->PMaxHigh() );
	
	// C-NP_FTS-P_PminH: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
	// TODO: Sort jobs by Energy / (TODO for later: by Peak values)
	allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

	// C-NP_PminH-P_PminH: Cost minimization with Full tree search for NP jobs and peak to min price greedy heuristic for P jobs.
	// TODO: Sort jobs by Energy / (TODO for later: by Peak values)
	//allocTab( NP_COST_MIN_WITH_GREEDY_SEARCH, P_COST_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );

	// (Lee) P-NP_FTS-P_PtotH: Peak minimization with full tree search for NP and job pieces to min Ptot slots greedy heuristics for P jobs.
	// TODO: sort jobs from non-preemptable to preemptable
	//allocTab( NP_PEAK_MIN_WITH_TREE_SEARCH, P_PEAK_MINIMIZATION, *(sp->J()), emptyTab, pNondominatedSet, sp->PMax(), *(sp->pMin()), *(sp->PH()) );
	
	/*
	double mCost = numeric_limits<double>::max();
        // Cost minTab
        vector<Signal<int>*>* minCostTab = NULL;
        double mPeak = numeric_limits<double>::max();
        // Peak minTab 
        vector<Signal<int>*>* minCostTab = NULL;
	*/
        
	for(std::vector<TSolution*>::iterator it=pNondominatedSet->begin(); it != pNondominatedSet->end(); it++)
        {
          SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
          cout << "Pareto\t" << *msIt << endl;
	  printTab( *(msIt->getTab()) );
	  
	  /*
	  if (mCost > msIt->getCost())
	    {
              mCost = msIt->getCost();
              minCostTab = msIt->getTab();
            }
          if (mPeak > msIt->getPeak())
            {
              mPeak = msIt->getPeak;
              minPeakTab = msIt->getTab();
            }
          */

	  /*
	  for( vector<Signal<int>*>::iterator it = msIt->getTab()->begin(); it != msIt->getTab()->end(); it++ )
            {
	      cout<<"a" << endl;
              (*it)->print();
            }
	  */
	    
          delete msIt;
        }

        delete pNondominatedSet;


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
