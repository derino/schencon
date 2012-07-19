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

#include "Signal.h"
#include "Task.h"
#include "util.h"
#include "Tab.h"
#include "MOS.h"
#include <math.h>
#include <time.h>

//  TODO: Fix this, this should come from time.h actually
#define CLOCKS_PER_SEC  1000000l

using namespace std;
//using namespace boost::lambda;

  // We may assume the scheduling to be done once for 
  // all tasks acc. to the predictions at the beginning.
  // Or, we may run the scheduling solver at a specific 
  // period acc. to the updated predictions and the 
  // new/remaining task set.

  // the time-dependent variables will be represented by 
  // their values at discrete times.
  // How to choose the sampling period(T)?
  // Assumption is that every task is pausable/resumable in T time.
  // Also power profiles for every task are given at T resolution.

  // t=0 => n=0 is the min of all arrival times.
  // n_max is the (max of all deadlines)/T
  // all time dependent variables are arrays of size n_max.
  
  // We assume that preemption doesn't introduce extra energy costs. (PP is not affected by preemption.)

  // p_min: min. price signal (hour, CHF?)
  // 0 0.10
  // 1 0.09
  // 2 0.09
  // ...
  // 23 0.11
  // e.g. if price signal changes in every hour and for T = 1 min
  // p_min = {0.10, 0.10, 0.10, 0.10, ... (60 of them) ...,  0.09, 0.09, 0.09, ... (60 of them) ...}

  // P_H: harvested power

  // J: task set
  // j_i: {a_i(arrival time), d_i(deadline), PP_i(power profile), pr(preemptability)}
  // power profile for washing machine program A: (min(should be divisible by T), watts)
  // 0 10.0
  // 5 4.5
  // 20 15.0
  // 23 0
  // implies pp = {10, 10, 10, 10, 10, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5, 4.5,
  //               4.5, 4.5, 4.5, 4.5, 4.5, 15, 15, 15} for T = 1 min.

  // P_max: max. power that can be withdrawn by the total of tasks.
  


// use signals from experiment 2
//extern string ex = "ex2/";


int main()
{
	// P_max is fixed
	/*// for ex1
	double P_max = 5;*/

	// for case_study
	double P_max = 15;

	// generate a price signal array p_min
	// read price signal
	Signal<double> p_min("p_min", "p_min.txt");
	//  ONDEBUG( p_min.printIntoMatFile() );

	// generate a harvested power array P_H
	// read harvested power
	Signal<double> P_H("P_H", "P_H.txt");
	// ONDEBUG( P_H.printIntoMatFile() );

	// generate tasks
	// read jobs
	vector<Task*> J; // keeps the list of tasks

/*	// Task set for ex1
	Task J_1("J_1", 0, 9, false, "L_1.txt");
	J.push_back(&J_1);
	Task J_2("J_2", 2, 10, true, "L_2.txt");
	J.push_back(&J_2);
	Task J_3("J_3", 3, 7, false, "L_3.txt");
	J.push_back(&J_3);*/

	// Task set for ex2
//	Task J_1("J_1", 0, 9, false, "L_1.txt");
//	J.push_back(&J_1);
//	Task J_3("J_3", 3, 7, false, "L_3.txt");
//	J.push_back(&J_3);
//	Task J_2("J_2", 2, 10, true, "L_2.txt");
//	J.push_back(&J_2);

/*	// Task set for case_study
	Task J_1("J_1", 180, 540, false, "L_1.txt");
	J.push_back(&J_1);
	Task J_2("J_2", 0, 720, true, "L_2.txt");
	J.push_back(&J_2);
	Task J_3("J_3", 60, 420, false, "L_3.txt");
	J.push_back(&J_3);*/

//	// Task set for case_study_120_min
//	Task J_1("J_1", 10, 100, false, "L_1.txt");
//	J.push_back(&J_1);
//	Task J_2("J_2", 0, 120, true, "L_2.txt");
//	J.push_back(&J_2);
//	Task J_3("J_3", 40, 110, false, "L_3.txt");
//	J.push_back(&J_3);

    //Task set for case_study_72_slots
    Task J_1("J_1", 0, 70, false, "L_1.txt");
    J.push_back(&J_1);
    Task J_2("J_2", 5, 72, true, "L_2.txt");
    J.push_back(&J_2);
    Task J_3("J_3", 15, 72, false, "L_3.txt");
    J.push_back(&J_3);
    Task J_4("J_4", 22, 69, false, "L_4.txt");
    J.push_back(&J_4);
    Task J_5("J_5", 20, 70, true, "L_5.txt");
    J.push_back(&J_5);

//  //Task set for case_study_T20_min
//  Task J_1("J_1", 3, 20, false, "L_1.txt");
//  J.push_back(&J_1);
//  Task J_2("J_2", 0, 21, true, "L_2.txt");
//  J.push_back(&J_2);
//  Task J_3("J_3", 2, 18, false, "L_3.txt");
//  J.push_back(&J_3);

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
	int max_d_i = checkTasksValidity(J, P_max);

	//clock_t t1, t2, t3;
	//t1 = clock();
//	for (int i=0; i<1000; i++) {

		vector<Signal<int>*> emptyTab;
		//vector<Signal<int>*>* minTab = new vector<Signal<int>*>();
		//double minCost = numeric_limits<double>::max();
		//double peakValue = numeric_limits<double>::max();

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
        allocTab( NP_COST_MIN_WITH_TREE_SEARCH, P_PEAK_MINIMIZATION, J, emptyTab, pNondominatedSet, P_max, p_min, P_H );

        for(std::vector<TSolution*>::iterator it=pNondominatedSet->begin(); it != pNondominatedSet->end(); it++)
        {
          SchedulingMOMHSolution* msIt = (SchedulingMOMHSolution*) *it;
          cout << "Pareto\t" << *msIt << endl;
          //printTab( *(msIt->getTab()) );
          for( vector<Signal<int>*>::iterator it = msIt->getTab()->begin(); it != msIt->getTab()->end(); it++ )
            {
              (*it)->print();
            }
          delete msIt;
        }

        delete pNondominatedSet;

//      t2 = clock();
//      for (int i=0; i<1000; i++) {

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
