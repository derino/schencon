/*
 * MOS.h
 *
 *  Created on: Dec 17, 2011
 *      Author: yuc
 */

#ifndef MOS_H_
#define MOS_H_

#include <momh/tlistset.h>
#include <momh/nondominatedset.h>
#include "SchedulingMOMHSolution.hpp"


void MOS(vector<Task*>& J, vector<Signal<int>*>& inputTab, TNondominatedSet* pNondominatedSet,
		double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber=0) {


	int schedulesLength = p_min.size();

	if (taskNumber == (signed)J.size()) {
		cout << "no more tasks so we have reached to the leaf!!" <<endl;
		// Reached leaf

		// calculate cost, compare with minimum cost
		double cost = evaluateCost(inputTab, J, P_max, p_min, P_H);
		cout << "Cost for this tree path: " << cost << endl;

		double peak = evaluateCostForLees(inputTab, J);

		SchedulingMOMHSolution* sol = new SchedulingMOMHSolution(peak, cost);
		if ( pNondominatedSet->Update(*sol) == true )
		{
			cout << "new pareto point" << endl;
		}

		// return to the "for loop" which last called allocTab
		return;
	}

	Task* task = J[taskNumber];
	// TODO: cout << task->getName() << endl; bunu koyarken "endl'i sil "moving to following task -"...daki..

	if(task->getPr())
	{
		vector<Signal<int>*> tab(inputTab);

		Signal<double>* P_tot = P__tot2(tab, J);

	  // sort task parts ...
	  vector<Pair> vTaskParts;
	  for(int i = 0; i<task->getL()->size(); i++ )
	    {
	      Pair p;
	      p.index = i;
	      p.value = (*task->getL())[i];
	      vTaskParts.push_back(p);
	    }
	  //cout << ">>>>>>test<<<<<<" << endl;
	  //for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
	  //cout << "task part:" << (*it).index << " " << (*it).value << endl;
	  sort(vTaskParts.begin(), vTaskParts.end(), compare);
	  reverse(vTaskParts.begin(), vTaskParts.end());
	  //for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
	  //cout << "sorted task part:" << (*it).index << " " << (*it).value << endl;
	  //cout << ">>>>>/test<<<<<<" << endl;

	  // sort p_min ...
	  vector<Pair> vPMin;
	  for(int i = 0; i < p_min.size(); i++ )
	    {
	      Pair p;
	      p.index = i;
	      p.value = p_min[i];
	      vPMin.push_back(p);
	    }
	  //cout << ">>>>>test<<<<<<" << endl;
	  //for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
	  //  cout << "p_min:" << (*it).index << " " << (*it).value << endl;
	  sort(vPMin.begin(), vPMin.end(), compare);
	  //for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
	  //  cout << "sorted p_min:" << (*it).index << " " << (*it).value << endl;
	  //cout << ">>>>>/test<<<<<<" << endl;

	  //Schedule of task [-1,-1, ... ,1 ,-1, 2, -1, ...] -1:for empty slots; id: for scheluded task part id.
	  vector<int> s_task;
	  for(int i = 0; i < schedulesLength; i++)
	  {
		  s_task.push_back(-1);
	  }

	  for(int k=0; k < task->getL()->size(); k++)
	  {
		  Range range1, range2;
		  range1.left = task->getA() + vTaskParts[k].index;
		  range1.right = task->getD() - (task->getL()->size() - vTaskParts[k].index);
//		  cout<< "ilk part: " << vTaskParts[k].index << endl;
		  int l = getLeftScheduled(vTaskParts[k].index, s_task);
		  int r = getRightScheduled(vTaskParts[k].index, s_task, task);

		  if(l == numeric_limits<int>::min())
			  range2.left = l;
		  else
		  {
			  int s_task_l = getScheduleIndexOf(l, s_task);
			  range2.left = s_task_l + (vTaskParts[k].index - l);
		  }

		  if(r == numeric_limits<int>::max())
			  range2.right = r;
		  else
		  {
			  int s_task_r = getScheduleIndexOf(r, s_task);
			  range2.right = s_task_r - (r - vTaskParts[k].index);
		  }

		  Range rangeTask = intersect(range1, range2);
	  //Till here, range calculation

		  for(int j = 0; j < schedulesLength; j++)
		  {

			  if( isInInterval(vPMin[j].index, rangeTask) )
			  {
				  // TODO: check if Pmax is violated for selected slot...
				  //vPMin[j].index'te bir deger varmis gibi yeni bir s yaratilacak.
				  //s taba eklenip Ptot hesaplanacak o slottaki
				  // En son da eklenen schedulei tabtan sil
				  if( P_tot->at(vPMin[j].index) + task->getL()->at(k) <= P_max )
				  {
					  s_task[vPMin[j].index] = vTaskParts[k].index;
//					  cout << "s_task[" << vPMin[j].index << "]: " << vTaskParts[k].index << endl;
					  break;
				  }
			  }
		  }
	  }
	  cout << "Found Schedule: ";
	  for(int i = 0; i < s_task.size(); i++)
	  {
		  cout << s_task[i] << " ";
	  }
	  cout<< endl;
	  // TODO: Exception ekle.

	  int* s_values = scheduleNorm(s_task);
	  Signal<int>* s = new Signal<int>("s_" + task->getName(), s_values, schedulesLength);

	  tab.push_back(s);
	  //printTab(tab);
	  MOS(J, tab, pNondominatedSet, P_max, p_min, P_H, taskNumber+1);

	}
	else {
		// Non-preemptive task scheduling
		cout << "Non-preemptive task #" << taskNumber << " " << task->getName() << " is in process.." << endl;
		//alttaki iki satir daha iyi anlamak icin eklendi gereksiz islemler aslinda..
		task->print();

		for(int startTime = task->getA(); startTime < schedulesLength - task->getL()->size(); startTime++) {

			cout << task->getName() << " icin inputTab olusturuldu..." << endl;
			vector<Signal<int>*> tab(inputTab);

			cout << "Task " << task->getName() <<" is scheduling now for startTime: " << startTime << ".." << endl;

			Signal<int>* scheduleForStartTime = getNonPreemptiveSchedule(*task, startTime, schedulesLength);

			tab.push_back(scheduleForStartTime);

			cout << "latest schedule for task: " << task->getName() << " has been copied to the tab.. for start time: " << startTime << endl;
			//printTab(tab);

			if (isConstraintSatisfied(tab, J, P_max)) {
				if(taskNumber+1 != (signed)J.size()){
					cout << "Moving to following task - Jumping the following tree branch-" << endl;
					MOS(J, tab, pNondominatedSet, P_max, p_min, P_H, taskNumber+1);
				}
				else
					MOS(J, tab, pNondominatedSet, P_max, p_min, P_H, taskNumber+1);
			}
			if(startTime + task->getL()->size() >= task->getD())
				break;
			cout << "finished scheduling for non-preemptive task: " << task->getName() << " for start time: " << startTime << endl;
		}
	}
}


#endif /* MOS_H_ */
