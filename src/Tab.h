/*
 * tab.h
 *
 *  Created on: Dec 17, 2011
 *      Author: yuc
 */

#ifndef TAB_H_
#define TAB_H_

#include <momh/tlistset.h>
#include <momh/nondominatedset.h>
#include "SchedulingMOMHSolution.hpp"
#include "InfeasibleSolutionException.h"

enum NPHeuristicType {NP_COST_MIN_WITH_TREE_SEARCH=0, NP_COST_MIN_WITH_GREEDY_SEARCH, NP_PEAK_MIN_WITH_TREE_SEARCH, NP_ASAP_SCHEDULING};

enum PHeuristicType {P_PEAK_MINIMIZATION=0, P_COST_MINIMIZATION, P_COST_MIN_WITH_TREE_SEARCH, P_COST_MIN_WITH_TREE_SEARCH2, P_COST_MIN_WITH_TREE_SEARCH3, P_ASAP_SCHEDULING /*p_max aware cost minimization*/,
  P_PMAX_AWARE/*doesn't care about cost minimization*/};



void
npTreeSearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber);

void
npCostMinWithGreedySearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);

// finds one schedule for the preemptable task with cost minimization in mind
// but that schedule can be violating the Pmax constraint.
// very fast but likely to be infeasible
void
pCostMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);

// for each preemptable task, finds a feasible schedule for the task and
// continues with the next task.
// fast but likely to be infeasible
void
pCostMinWithTreeSearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);
bool scheduleTaskPart(Task* task, double P_max, Signal<double>* P_tot, vector<Pair>& vPMin, vector<Pair>& vTaskParts, vector<int>& s_task_to_fill, 
		      Signal<int>*& sResult, int taskPartNumber);

// evaluates all possible feasible schedules of preemptable tasks
// tree search of each preemptable task is connected to lower tasks tree search
// very slow and consumes a lot of memory but finds optimal solution if completes.
void
pCostMinWithTreeSearch2(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);
void scheduleTaskPart2(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab, TNondominatedSet* pNondominatedSet, Signal<double>& p_min, Signal<double>& P_H, int taskNumber,
		       Task* task, double P_max, Signal<double>* P_tot, vector<Pair>& vPMin, vector<Pair>& vTaskParts, vector<int>& s_task_to_fill, 
		       Signal<int>*& sResult, int taskPartNumber);


// stops searching when a feasible schedule is found for all of the preemptable tasks
bool
pCostMinWithTreeSearch3(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);
bool scheduleTaskPart3(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab, TNondominatedSet* pNondominatedSet, Signal<double>& p_min, Signal<double>& P_H, int taskNumber,
		       Task* task, double P_max, Signal<double>* P_tot, vector<Pair>& vPMin, vector<Pair>& vTaskParts, vector<int>& s_task_to_fill, 
		       Signal<int>*& sResult, int taskPartNumber);



void
pPeakMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J,
    vector<Signal<int>*>& inputTab, TNondominatedSet* pNondominatedSet,
    double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber);


void
ASAPScheduling(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber);






////////////////////////////////////////////////////////////////////////////////////////
void
allocTab(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber = 0)
{
  //  if(taskNumber < 8)
  //    cout << "task number: " << taskNumber << endl;


  if (taskNumber == (signed) J.size())
    {
      // cout << "no more tasks so we have reached to the leaf!!" << endl;
      // Reached leaf
      double cost, peak;

      if (isConstraintSatisfied(inputTab, J, P_max))
	{
	  // calculate cost, compare with minimum cost
	  cost = evaluateCost(inputTab, J, P_max, p_min, P_H);
	  // cout << "Cost for this tree path: " << cost << endl;
	  
	  peak = evaluateCostForLees(inputTab, J);
	}
      else // !(isConstaintSatisfied)
	{
	  return; // TODO why didn't we put this return before?!?!
	  cost = numeric_limits<double>::max();
	  peak = numeric_limits<double>::max();
	}
      
      //TODO:add tab to SchedulingMOMHSolution
	  
      vector<Signal<int>*>* tab = new vector<Signal<int>*>();
      for ( vector<Signal<int>*>::iterator it = inputTab.begin(); it != inputTab.end(); it++)
	{
	  Signal<int>* sit = *it;
	  Signal<int>* sitCopy = new Signal<int>(*sit);
	  tab->push_back(sitCopy);
	}
      // vector<Signal<int>*>* tab = new vector<Signal<int>*>(inputTab);
	  
      SchedulingMOMHSolution* sol = new SchedulingMOMHSolution(peak, cost);
      sol->setTab(tab);
      if ( pNondominatedSet->Update(*sol) == true )
      {
	//cout << "#paretos:" << pNondominatedSet->iSetSize << endl;
	delete sol; // because a copy is made inside Update if the solution is added.
      }
      else
	delete sol;
      return;
    } // END of leaf

  Task* task = J[taskNumber];
    // TODO: cout << task->getName() << endl; bunu koyarken "endl'i sil "moving to following task -"...daki..

  if (task->getPr())
    {
      switch (pHeurType)
        {
      case P_PEAK_MINIMIZATION:
        pPeakMinimization(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max,
            p_min, P_H, taskNumber);
        break;
      case P_COST_MINIMIZATION:
        pCostMinimization(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max,
            p_min, P_H, taskNumber);
        break;
      case P_COST_MIN_WITH_TREE_SEARCH:
        pCostMinWithTreeSearch(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max,
            p_min, P_H, taskNumber);
        break;
      case P_COST_MIN_WITH_TREE_SEARCH2:
        pCostMinWithTreeSearch2(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max,
            p_min, P_H, taskNumber);
        break;
      case P_COST_MIN_WITH_TREE_SEARCH3:
        pCostMinWithTreeSearch3(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max,
            p_min, P_H, taskNumber);
        break;
      case P_ASAP_SCHEDULING:
        ASAPScheduling(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min,
            P_H, taskNumber);
	break;
      default:
        cout << "ERROR: undefined heuristic type!" << endl;
        break;

        }
    }

  else
    {
      switch (npHeurType)
        {
      case NP_COST_MIN_WITH_TREE_SEARCH:
        npTreeSearch(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min,
            P_H, taskNumber);
        break;
      case NP_PEAK_MIN_WITH_TREE_SEARCH:
        npTreeSearch(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min,
            P_H, taskNumber);
        break;
      case NP_COST_MIN_WITH_GREEDY_SEARCH:
        npCostMinWithGreedySearch(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min,
            P_H, taskNumber);
	break;
      case NP_ASAP_SCHEDULING:
        ASAPScheduling(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min,
            P_H, taskNumber);
        break;
      default:
        cout << "ERROR: undefined heuristic type!" << endl;
        break;

        }
    }

}
// END OF ALLOCTAB





bool
allocTabBool(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber = 0)
{
  //  if(taskNumber < 8)
  //    cout << "task number: " << taskNumber << endl;


  if (taskNumber == (signed) J.size())
    {
      // cout << "no more tasks so we have reached to the leaf!!" << endl;
      // Reached leaf
      double cost, peak;

      if (isConstraintSatisfied(inputTab, J, P_max))
	{
	  // calculate cost, compare with minimum cost
	  cost = evaluateCost(inputTab, J, P_max, p_min, P_H);
	  // cout << "Cost for this tree path: " << cost << endl;
	  
	  peak = evaluateCostForLees(inputTab, J);
	}
      else // !(isConstaintSatisfied)
	{
	  return false; // TODO why didn't we put this return before?!?!
	  cost = numeric_limits<double>::max();
	  peak = numeric_limits<double>::max();
	}
      
      //TODO:add tab to SchedulingMOMHSolution
	  
      vector<Signal<int>*>* tab = new vector<Signal<int>*>();
      for ( vector<Signal<int>*>::iterator it = inputTab.begin(); it != inputTab.end(); it++)
	{
	  Signal<int>* sit = *it;
	  Signal<int>* sitCopy = new Signal<int>(*sit);
	  tab->push_back(sitCopy);
	}
      // vector<Signal<int>*>* tab = new vector<Signal<int>*>(inputTab);
	  
      SchedulingMOMHSolution* sol = new SchedulingMOMHSolution(peak, cost);
      sol->setTab(tab);
      if ( pNondominatedSet->Update(*sol) == true )
      {
	//cout << "#paretos:" << pNondominatedSet->iSetSize << endl;
	delete sol; // because a copy is made inside Update if the solution is added.
	return true;
      }
      else 
	delete sol;
      return true;
    } // END of leaf

  Task* task = J[taskNumber];
    // TODO: cout << task->getName() << endl; bunu koyarken "endl'i sil "moving to following task -"...daki..

  if (task->getPr())
    {
      switch (pHeurType)
        {
	case P_COST_MIN_WITH_TREE_SEARCH3:
	  return pCostMinWithTreeSearch3(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H, taskNumber);
	  break;
	default:
	  cout << "ERROR: undefined heuristic type!" << endl;
	  break;
	  
        }
    }
  
}
// END OF ALLOCTABBOOL








////////////////////////////////////////////////////////////////////////////////////////
//>>>>>>>>>>>>>Lee's Preemptable Scheduling<<<<<<<<<<<<<<<<<
void
pPeakMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J,
    vector<Signal<int>*>& inputTab, TNondominatedSet* pNondominatedSet,
    double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  //If this task is the FIRST processed PR task then we have to create tab
  //vector<Signal<int>*> tab(inputTab);
  Signal<double>* P_tot;
  if(inputTab.size() == 0)
    P_tot = new Signal<double>("all 0s", schedulesLength, 0);
  else
    P_tot = P__tot2(inputTab, J);

  // we may print the tot here.
  //P_tot->print();

  vector<Pair> vPTotInterval;

  for (int i = task->getA(); i < task->getD(); i++)
    {
      Pair p;
      p.index = i;
      p.value = P_tot->at(i);
      vPTotInterval.push_back(p);
    }
  delete P_tot;

  sort(vPTotInterval.begin(), vPTotInterval.end(), compare);
  /*
  cout << "sorted p_tot: [";
  for (vector<Pair>::iterator it = vPTotInterval.begin(); it
      != vPTotInterval.end(); ++it)
    cout << "(" << (*it).index << "," << (*it).value << ")";
  cout << "]" << endl;
  */
  vector<int> vSelectedIndices;

  for (int i = 0; i < task->getL()->size(); i++)
    {
      int indice = vPTotInterval.at(i).index;
      vSelectedIndices.push_back(indice);
    }

  sort(vSelectedIndices.begin(), vSelectedIndices.end());

  int* s_values = new int[schedulesLength];
  for (int i = 0; i < schedulesLength; i++)
    {
      s_values[i] = 0;
    }

  for (vector<int>::iterator it = vSelectedIndices.begin(); it
      != vSelectedIndices.end(); it++)
    s_values[(*it)] = 1;

  Signal<int>* s = new Signal<int> ("s_" + task->getName(), s_values,
      schedulesLength);

  inputTab.push_back(s);
  // cout << "hey!" << endl;
  // printTab(inputTab);
  // cout << "hey 2 !" << endl;
  allocTab(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H, taskNumber + 1);

  // TODO: fix memory leakage
  vector<Signal<int>*>::iterator it = inputTab.end();
  it--;
  delete (*it);
  inputTab.pop_back();

  //TODO: Adamin cost function i olarak: P_tot'undaki maksimum deger bulunacak.

}





////////////////////////////////////////////////////////////////////////////////////////
//ASAP scheduling
void
ASAPScheduling(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  vector<Signal<int>*> tab(inputTab);

  Signal<int>* scheduleForArrivalTime = getNonPreemptiveSchedule(*task, task->getA(), schedulesLength);

  tab.push_back(scheduleForArrivalTime);

  if (isConstraintSatisfied(tab, J, P_max))
        {
	  allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H, taskNumber + 1);

          // TODO: fix memory leakage
          vector<Signal<int>*>::iterator it = tab.end();
          it--;
          delete (*it);
          tab.pop_back();
        }
}







////////////////////////////////////////////////////////////////////////////////////////
void
pCostMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  //vector<Signal<int>*> tab(inputTab);

  //cout << ">>>>>>>> tab:" << endl;
  //printTab(inputTab);
  //cout << ">>>>>>>> task size:" << J.size() << endl;

  // if inputTab is empty, initialize P_tot as zero signal.
  Signal<double>* P_tot;
  if(inputTab.size() == 0)
    P_tot = new Signal<double>("all 0s", schedulesLength, 0);
  else
    P_tot = P__tot2(inputTab, J);

  // sort task parts ...
  vector<Pair> vTaskParts;
  for (int i = 0; i < task->getL()->size(); i++)
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
  for (int i = 0; i < p_min.size(); i++)
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
  for (int i = 0; i < schedulesLength; i++)
    {
      s_task.push_back(-1);
    }

  for (int k = 0; k < task->getL()->size(); k++)
    {
      Range range1, range2;
      range1.left = task->getA() + vTaskParts[k].index;
      range1.right = task->getD()
          - (task->getL()->size() - vTaskParts[k].index);
      //              cout<< "ilk part: " << vTaskParts[k].index << endl;
      int l = getLeftScheduled(vTaskParts[k].index, s_task);
      int r = getRightScheduled(vTaskParts[k].index, s_task, task);

      if (l == numeric_limits<int>::min())
        range2.left = l;
      else
        {
          int s_task_l = getScheduleIndexOf(l, s_task);
          range2.left = s_task_l + (vTaskParts[k].index - l);
        }

      if (r == numeric_limits<int>::max())
        range2.right = r;
      else
        {
          int s_task_r = getScheduleIndexOf(r, s_task);
          range2.right = s_task_r - (r - vTaskParts[k].index);
        }

      Range rangeTask = intersect(range1, range2);
      //Till here, range calculation

      for (int j = 0; j < schedulesLength; j++)
        {

          if (isInInterval(vPMin[j].index, rangeTask))
            {
              // checking if Pmax is violated for selected slot...
              // vPMin[j].index'te bir deger varmis gibi yeni bir s yaratilacak.
              // s taba eklenip Ptot hesaplanacak o slottaki
              // En son da eklenen schedulei tabtan sil
              if (P_tot->at(vPMin[j].index) + task->getL()->at(k) <= P_max)
                {
                  s_task[vPMin[j].index] = vTaskParts[k].index;
                  // cout << "s_task[" << vPMin[j].index << "]: " << vTaskParts[k].index << endl;
                  break;
                }
            }
        }
    }
  //  cout << "Found Schedule: ";
  /*
  for (unsigned int i = 0; i < s_task.size(); i++)
    {
      cout << s_task[i] << " ";
    }
  cout << endl;
  */
  // TODO: Exception ekle.

  int* s_values = scheduleNorm(s_task);
  Signal<int>* s = new Signal<int> ("s_" + task->getName(), s_values,
      schedulesLength);

  inputTab.push_back(s);
  //printTab(tab);
  allocTab(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H,
      taskNumber + 1);

  // TODO: fix memory leakage
  vector<Signal<int>*>::iterator it = inputTab.end();
  it--;
  delete (*it);
  inputTab.pop_back();
}




////////////////////////////////////////////////////////////////////////////////////////
void
pCostMinWithTreeSearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  //vector<Signal<int>*> tab(inputTab);

#ifdef DEBUG
  if(taskNumber >= 6)
    {
      cout << ">>>>>>>> tab:" << endl;
      printTab(inputTab);
      cout << ">>>>>>>> task size:" << J.size() << endl;
    }
#endif

  // if inputTab is empty, initialize P_tot as zero signal.
  Signal<double>* P_tot;
  if(inputTab.size() == 0)
    P_tot = new Signal<double>("all 0s", schedulesLength, 0);
  else
    P_tot = P__tot2(inputTab, J);

  // sort task parts ...
  vector<Pair> vTaskParts;
  for (int i = 0; i < task->getL()->size(); i++)
    {
      Pair p;
      p.index = i;
      p.value = (*task->getL())[i];
      vTaskParts.push_back(p);
    }

#ifdef DEBUG
  cout << ">>>>>>test<<<<<<" << endl;
  for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
    cout << "task part:" << (*it).index << " " << (*it).value << endl;
#endif

  sort(vTaskParts.begin(), vTaskParts.end(), compare);
  reverse(vTaskParts.begin(), vTaskParts.end());

#ifdef DEBUG
  for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
    cout << "sorted task part:" << (*it).index << " " << (*it).value << endl;
  cout << ">>>>>/test<<<<<<" << endl;
#endif

  // sort p_min ...
  vector<Pair> vPMin;
  for (int i = 0; i < p_min.size(); i++)
    {
      Pair p;
      p.index = i;
      p.value = p_min[i];
      vPMin.push_back(p);
    }

#ifdef DEBUG
  cout << ">>>>>test<<<<<<" << endl;
  for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
    cout << "p_min:" << (*it).index << " " << (*it).value << endl;
#endif

  sort(vPMin.begin(), vPMin.end(), compare);

#ifdef DEBUG
  for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
    cout << "sorted p_min:" << (*it).index << " " << (*it).value << endl;
  cout << ">>>>>/test<<<<<<" << endl;
#endif

  //Schedule of task [-1,-1, ... ,1 ,-1, 2, -1, ...] -1:for empty slots; id: for scheluded task part id.
  vector<int> s_task;
  for (int i = 0; i < schedulesLength; i++)
    {
      s_task.push_back(-1);
    }

  // the resulting feasible schedule. to be only written by the leaf of the search tree.
  Signal<int>* sResult;

  //  for (int k = 0; k < vTaskParts.size(); k++)
  //  {
  bool foundFeasible = false;
  foundFeasible = scheduleTaskPart(task, P_max, P_tot, vPMin, vTaskParts, s_task, sResult, 0); // start scheduling from first task part (index 0)





  //    } // END of for all task parts
  //  cout << "Found Schedule: ";
  /*
  for (unsigned int i = 0; i < s_task.size(); i++)
    {
      cout << s_task[i] << " ";
    }
  cout << endl;
  */
  // TODO: Exception ekle.

  //int* s_values = scheduleNorm(s_task);
  //Signal<int>* s = new Signal<int> ("s_" + task->getName(), s_values,
  //    schedulesLength);

  if(foundFeasible)
    {
      cout << "Schedule found!" << endl;
#ifdef DEBUG
      cout << "sResult: ";
      for (unsigned int i = 0; i < sResult->size(); i++)
	{
	  cout << sResult->at(i) << " ";
	}
      cout << endl;
#endif
      inputTab.push_back(sResult);

      // continue the outer tree search with the next task by calling allocTab if preemptable task is scheduled feasibly.
      allocTab(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H, taskNumber + 1);
      // TODO: fix memory leakage
      vector<Signal<int>*>::iterator it = inputTab.end();
      it--;
      delete (*it);
      inputTab.pop_back();
    }
  else
    {
      cout << "Schedule not found! Prune outer tree search." << endl; // do not call allocTab anymore.
      //throw InfeasibleSolutionException();
    }

  //printTab(tab);

}

bool scheduleTaskPart(Task* task, double P_max, Signal<double>* P_tot, vector<Pair>& vPMin, vector<Pair>& vTaskParts, vector<int>& s_task_to_fill, 
		      Signal<int>*& sResult, int taskPartNumber)
{
#ifdef DEBUG
    cout << "scheduleTaskPart is called: task=" << task->getName() << " taskPart=" << taskPartNumber << endl;
#endif
  vector<int> s_task(s_task_to_fill);
  int schedulesLength = s_task.size();
  
#ifdef DEBUG
  cout << "s_task: [";
  for (unsigned int i = 0; i < s_task.size(); i++)
    {
      cout << s_task[i] << " ";
    }
  cout << "]" << endl;
#endif
  
  // check if all task parts are scheduled.
  if( taskPartNumber == vTaskParts.size() )
    {
      // check if Ptot <= Pmax given s_task of the preemptable task and Ptot of all other scheduled tasks
      // if true, return true, else return false.
      int* s_values = scheduleNorm(s_task);
      Signal<int>* s = new Signal<int> ("s_" + task->getName(), s_values, schedulesLength );
      Signal<double>* P__i = P_i( task->getL(), s);
      Signal<double>* Ptot = *P_tot + *P__i;
      if ( max(*Ptot) <= P_max )
	{
#ifdef DEBUG
	  cout << "leaf satisfies Pmax: " << max(*Ptot) << " <= " << P_max << endl;
#endif
	  sResult = s;
	  delete Ptot;
	  delete P__i;
	  return true;
	}
      else
	{
#ifdef DEBUG
	  cout << "leaf does not satisfy Pmax: " << max(*Ptot) << " <= " << P_max << endl;
#endif
	  delete Ptot;
	  delete P__i;
	  return false;
	}
    }
  
  // calculate range
  Range range1, range2;
  range1.left = task->getA() + vTaskParts[taskPartNumber].index;
  range1.right = task->getD()
    - (task->getL()->size() - vTaskParts[taskPartNumber].index);

#ifdef DEBUG
  cout<< "vTaskParts[taskPartNumber].index: " << vTaskParts[taskPartNumber].index << endl;
#endif
  
  int l = getLeftScheduled(vTaskParts[taskPartNumber].index, s_task);
  int r = getRightScheduled(vTaskParts[taskPartNumber].index, s_task, task);
  
  if (l == numeric_limits<int>::min())
    range2.left = l;
  else
    {
      int s_task_l = getScheduleIndexOf(l, s_task);
      range2.left = s_task_l + (vTaskParts[taskPartNumber].index - l);
    }
  
  if (r == numeric_limits<int>::max())
    range2.right = r;
  else
    {
      int s_task_r = getScheduleIndexOf(r, s_task);
      range2.right = s_task_r - (r - vTaskParts[taskPartNumber].index);
    }
  
#ifdef DEBUG
  cout << "range1: [" << range1.left << ", " << range1.right << "]    range2: [" << range2.left << ", " << range2.right << "]" << endl;
#endif
  
  Range rangeTask = intersect(range1, range2);

#ifdef DEBUG
  cout << "Range: [" << rangeTask.left << ", " << rangeTask.right << "]" << endl;
#endif
  //Till here, range calculation
      


  vector<Pair> vPminInRange; // will keep an ordered list (low to high wrt Pmin) of time slots in the calculated range 
  for (int j = 0; j < schedulesLength; j++)
    {
      if (isInInterval(vPMin[j].index, rangeTask))
	{
	  Pair p;
	  p.index = vPMin[j].index;
	  p.value =  vPMin[j].value;
	  vPminInRange.push_back(p); // fixed. not j.
	}
    }
  //  sort(vPminInRange.begin(), vPminInRange.end());

#ifdef DEBUG
  cout << "sorted p_min in range: [";
  for(vector<Pair>::iterator it = vPminInRange.begin(); it != vPminInRange.end(); ++it)
    cout << "(" << (*it).index << ", " << (*it).value << "), " << endl;
#endif

  // for all time slots in the range ordered from low price to high, assign the current task part its time slot and continue scheduling next task parts by recursion
  bool foundFeasible = false;
  for(vector<Pair>::iterator it = vPminInRange.begin(); it != vPminInRange.end(); it++)
    {
      s_task[(*it).index] = vTaskParts[taskPartNumber].index;
#ifdef DEBUG
      cout << "Tree search given " << taskPartNumber << "-th task part is scheduled at " << (*it).index << endl;
#endif
      foundFeasible = scheduleTaskPart(task, P_max, P_tot, vPMin, vTaskParts, s_task, sResult, taskPartNumber+1);
      if(foundFeasible)
	break;
      
      // IMPORTANT: before continuing with the next possible slot of the task part, deschedule it first!
      s_task[(*it).index] = -1;
    }
  return foundFeasible;

}

















////////////////////////////////////////////////////////////////////////////////////////
void
pCostMinWithTreeSearch2(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  //vector<Signal<int>*> tab(inputTab);

#ifdef DEBUG
  if(taskNumber >= 6)
    {
      cout << ">>>>>>>> tab:" << endl;
      printTab(inputTab);
      cout << ">>>>>>>> task size:" << J.size() << endl;
    }
#endif

  // if inputTab is empty, initialize P_tot as zero signal.
  Signal<double>* P_tot;
  if(inputTab.size() == 0)
    P_tot = new Signal<double>("all 0s", schedulesLength, 0);
  else
    P_tot = P__tot2(inputTab, J);

  // sort task parts ...
  vector<Pair> vTaskParts;
  for (int i = 0; i < task->getL()->size(); i++)
    {
      Pair p;
      p.index = i;
      p.value = (*task->getL())[i];
      vTaskParts.push_back(p);
    }

#ifdef DEBUG
  cout << ">>>>>>test<<<<<<" << endl;
  for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
    cout << "task part:" << (*it).index << " " << (*it).value << endl;
#endif

  sort(vTaskParts.begin(), vTaskParts.end(), compare);
  reverse(vTaskParts.begin(), vTaskParts.end());

#ifdef DEBUG
  for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
    cout << "sorted task part:" << (*it).index << " " << (*it).value << endl;
  cout << ">>>>>/test<<<<<<" << endl;
#endif

  // sort p_min ...
  vector<Pair> vPMin;
  for (int i = 0; i < p_min.size(); i++)
    {
      Pair p;
      p.index = i;
      p.value = p_min[i];
      vPMin.push_back(p);
    }

#ifdef DEBUG
  cout << ">>>>>test<<<<<<" << endl;
  for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
    cout << "p_min:" << (*it).index << " " << (*it).value << endl;
#endif

  sort(vPMin.begin(), vPMin.end(), compare);

#ifdef DEBUG
  for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
    cout << "sorted p_min:" << (*it).index << " " << (*it).value << endl;
  cout << ">>>>>/test<<<<<<" << endl;
#endif

  //Schedule of task [-1,-1, ... ,1 ,-1, 2, -1, ...] -1:for empty slots; id: for scheluded task part id.
  vector<int> s_task;
  for (int i = 0; i < schedulesLength; i++)
    {
      s_task.push_back(-1);
    }

  // the resulting feasible schedule. to be only written by the leaf of the search tree.
  Signal<int>* sResult;

  //  bool foundFeasible = false;
  //foundFeasible = 
  scheduleTaskPart2(npHeurType, pHeurType, J, inputTab, pNondominatedSet, p_min, P_H, taskNumber, task, P_max, P_tot, vPMin, vTaskParts, s_task, sResult, 0); // start scheduling from first task part (index 0)


  //printTab(tab);

} // END OF pCostMinWithTreeSearch2





void scheduleTaskPart2(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, Signal<double>& p_min, Signal<double>& P_H, int taskNumber,
		       Task* task, double P_max, Signal<double>* P_tot, vector<Pair>& vPMin, vector<Pair>& vTaskParts, vector<int>& s_task_to_fill, 
		      Signal<int>*& sResult, int taskPartNumber)
{
#ifdef DEBUG
    cout << "scheduleTaskPart is called: task=" << task->getName() << " taskPart=" << taskPartNumber << endl;
#endif
  vector<int> s_task(s_task_to_fill);
  int schedulesLength = s_task.size();
  
#ifdef DEBUG
  cout << "s_task: [";
  for (unsigned int i = 0; i < s_task.size(); i++)
    {
      cout << s_task[i] << " ";
    }
  cout << "]" << endl;
#endif
  
  // check if all task parts are scheduled.
  if( taskPartNumber == vTaskParts.size() )
    {
      // check if Ptot <= Pmax given s_task of the preemptable task and Ptot of all other scheduled tasks
      // if true, return true, else return false.
      int* s_values = scheduleNorm(s_task);
      Signal<int>* s = new Signal<int> ("s_" + task->getName(), s_values, schedulesLength );
      Signal<double>* P__i = P_i( task->getL(), s);
      Signal<double>* Ptot = *P_tot + *P__i;
      if ( max(*Ptot) <= P_max )
	{
#ifdef DEBUG
	  cout << "leaf satisfies Pmax: " << max(*Ptot) << " <= " << P_max << endl;
#endif
	  sResult = s;
	  delete Ptot;
	  delete P__i;

	  inputTab.push_back(sResult);

	  // continue the outer tree search with the next task by calling allocTab if preemptable task is scheduled feasibly.
	  allocTab(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H, taskNumber + 1);
	  // TODO: fix memory leakage
	  vector<Signal<int>*>::iterator it = inputTab.end();
	  it--;
	  delete (*it);
	  inputTab.pop_back();

	  return;
	}
      else
	{
#ifdef DEBUG
	  cout << "leaf does not satisfy Pmax: " << max(*Ptot) << " <= " << P_max << endl;
#endif
	  delete Ptot;
	  delete P__i;
	  return;
	}
    }
  
  // calculate range
  Range range1, range2;
  range1.left = task->getA() + vTaskParts[taskPartNumber].index;
  range1.right = task->getD()
    - (task->getL()->size() - vTaskParts[taskPartNumber].index);

#ifdef DEBUG
  cout<< "vTaskParts[taskPartNumber].index: " << vTaskParts[taskPartNumber].index << endl;
#endif
  
  int l = getLeftScheduled(vTaskParts[taskPartNumber].index, s_task);
  int r = getRightScheduled(vTaskParts[taskPartNumber].index, s_task, task);
  
  if (l == numeric_limits<int>::min())
    range2.left = l;
  else
    {
      int s_task_l = getScheduleIndexOf(l, s_task);
      range2.left = s_task_l + (vTaskParts[taskPartNumber].index - l);
    }
  
  if (r == numeric_limits<int>::max())
    range2.right = r;
  else
    {
      int s_task_r = getScheduleIndexOf(r, s_task);
      range2.right = s_task_r - (r - vTaskParts[taskPartNumber].index);
    }
  
#ifdef DEBUG
  cout << "range1: [" << range1.left << ", " << range1.right << "]    range2: [" << range2.left << ", " << range2.right << "]" << endl;
#endif
  
  Range rangeTask = intersect(range1, range2);

#ifdef DEBUG
  cout << "Range: [" << rangeTask.left << ", " << rangeTask.right << "]" << endl;
#endif
  //Till here, range calculation
      


  vector<Pair> vPminInRange; // will keep an ordered list (low to high wrt Pmin) of time slots in the calculated range 
  for (int j = 0; j < schedulesLength; j++)
    {
      if (isInInterval(vPMin[j].index, rangeTask))
	{
	  Pair p;
	  p.index = vPMin[j].index;
	  p.value =  vPMin[j].value;
	  vPminInRange.push_back(p); // fixed. not j.
	}
    }
  //  sort(vPminInRange.begin(), vPminInRange.end());

#ifdef DEBUG
  cout << "sorted p_min in range: [";
  for(vector<Pair>::iterator it = vPminInRange.begin(); it != vPminInRange.end(); ++it)
    cout << "(" << (*it).index << ", " << (*it).value << "), " << endl;
#endif

  // for all time slots in the range ordered from low price to high, assign the current task part its time slot and continue scheduling next task parts by recursion
  //bool foundFeasible = false;
  for(vector<Pair>::iterator it = vPminInRange.begin(); it != vPminInRange.end(); it++)
    {
      s_task[(*it).index] = vTaskParts[taskPartNumber].index;
#ifdef DEBUG
      cout << "Tree search given " << taskPartNumber << "-th task part is scheduled at " << (*it).index << endl;
#endif
      //foundFeasible = 
      scheduleTaskPart2(npHeurType, pHeurType, J, inputTab, pNondominatedSet, p_min, P_H, taskNumber, task, P_max, P_tot, vPMin, vTaskParts, s_task, sResult, taskPartNumber+1);
      //if(foundFeasible)
      //break;
      
      // IMPORTANT: before continuing with the next possible slot of the task part, deschedule it first!
      s_task[(*it).index] = -1;
    }
  //return foundFeasible;

}
// END OF SCHEDULETASKPART2













////////////////////////////////////////////////////////////////////////////////////////
bool
pCostMinWithTreeSearch3(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  //vector<Signal<int>*> tab(inputTab);

#ifdef DEBUG
  if(taskNumber >= 6)
    {
      cout << ">>>>>>>> tab:" << endl;
      printTab(inputTab);
      cout << ">>>>>>>> task size:" << J.size() << endl;
    }
#endif

  // if inputTab is empty, initialize P_tot as zero signal.
  Signal<double>* P_tot;
  if(inputTab.size() == 0)
    P_tot = new Signal<double>("all 0s", schedulesLength, 0);
  else
    P_tot = P__tot2(inputTab, J);

  // sort task parts ...
  vector<Pair> vTaskParts;
  for (int i = 0; i < task->getL()->size(); i++)
    {
      Pair p;
      p.index = i;
      p.value = (*task->getL())[i];
      vTaskParts.push_back(p);
    }

#ifdef DEBUG
  cout << ">>>>>>test<<<<<<" << endl;
  for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
    cout << "task part:" << (*it).index << " " << (*it).value << endl;
#endif

  sort(vTaskParts.begin(), vTaskParts.end(), compare);
  reverse(vTaskParts.begin(), vTaskParts.end());

#ifdef DEBUG
  for(vector<Pair>::iterator it = vTaskParts.begin(); it != vTaskParts.end(); ++it)
    cout << "sorted task part:" << (*it).index << " " << (*it).value << endl;
  cout << ">>>>>/test<<<<<<" << endl;
#endif

  // sort p_min ...
  vector<Pair> vPMin;
  for (int i = 0; i < p_min.size(); i++)
    {
      Pair p;
      p.index = i;
      p.value = p_min[i];
      vPMin.push_back(p);
    }

#ifdef DEBUG
  cout << ">>>>>test<<<<<<" << endl;
  for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
    cout << "p_min:" << (*it).index << " " << (*it).value << endl;
#endif

  sort(vPMin.begin(), vPMin.end(), compare);

#ifdef DEBUG
  for(vector<Pair>::iterator it = vPMin.begin(); it != vPMin.end(); ++it)
    cout << "sorted p_min:" << (*it).index << " " << (*it).value << endl;
  cout << ">>>>>/test<<<<<<" << endl;
#endif

  //Schedule of task [-1,-1, ... ,1 ,-1, 2, -1, ...] -1:for empty slots; id: for scheluded task part id.
  vector<int> s_task;
  for (int i = 0; i < schedulesLength; i++)
    {
      s_task.push_back(-1);
    }

  // the resulting feasible schedule. to be only written by the leaf of the search tree.
  Signal<int>* sResult;

  bool foundFeasible = false;
  foundFeasible = scheduleTaskPart3(npHeurType, pHeurType, J, inputTab, pNondominatedSet, p_min, P_H, taskNumber, task, P_max, P_tot, vPMin, vTaskParts, s_task, sResult, 0); // start scheduling from first task part (index 0)

  delete P_tot;
  //printTab(tab);

  return foundFeasible;
} // END OF pCostMinWithTreeSearch3





bool scheduleTaskPart3(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, Signal<double>& p_min, Signal<double>& P_H, int taskNumber,
		       Task* task, double P_max, Signal<double>* P_tot, vector<Pair>& vPMin, vector<Pair>& vTaskParts, vector<int>& s_task_to_fill, 
		      Signal<int>*& sResult, int taskPartNumber)
{
#ifdef DEBUG
    cout << "scheduleTaskPart is called: task=" << task->getName() << " taskPart=" << taskPartNumber << endl;
#endif
  vector<int> s_task(s_task_to_fill);
  int schedulesLength = s_task.size();
  
#ifdef DEBUG
  cout << "s_task: [";
  for (unsigned int i = 0; i < s_task.size(); i++)
    {
      cout << s_task[i] << " ";
    }
  cout << "]" << endl;
#endif
  
  // check if all task parts are scheduled.
  if( taskPartNumber == vTaskParts.size() )
    {
      // check if Ptot <= Pmax given s_task of the preemptable task and Ptot of all other scheduled tasks
      // if true, return true, else return false.
      int* s_values = scheduleNorm(s_task);
      Signal<int>* s = new Signal<int> ("s_" + task->getName(), s_values, schedulesLength );
      Signal<double>* P__i = P_i( task->getL(), s);
      Signal<double>* Ptot = *P_tot + *P__i;
      if ( max(*Ptot) <= P_max )
	{
#ifdef DEBUG
	  cout << "leaf satisfies Pmax: " << max(*Ptot) << " <= " << P_max << endl;
#endif
	  sResult = s;
	  //	  delete P_tot; // not needed anymore, since the task is completely scheduled. will be recreated when allocTabBool is called below.
	  delete Ptot;
	  delete P__i;

	  inputTab.push_back(sResult);

	  // continue the outer tree search with the next task by calling allocTab if preemptable task is scheduled feasibly.
	  bool isLastPTaskScheduled = false;
	  isLastPTaskScheduled = allocTabBool(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H, taskNumber + 1);
	  // TODO: fix memory leakage
	  vector<Signal<int>*>::iterator it = inputTab.end();
	  it--;
	  delete (*it);
	  inputTab.pop_back();

	  return isLastPTaskScheduled;
	}
      else
	{
#ifdef DEBUG
	  cout << "leaf does not satisfy Pmax: " << max(*Ptot) << " <= " << P_max << endl;
#endif
	  delete Ptot;
	  delete P__i;
	  delete s;
	  return false;
	}
    } // END of if last task part
  
  // calculate range
  Range range1, range2;
  range1.left = task->getA() + vTaskParts[taskPartNumber].index;
  range1.right = task->getD()
    - (task->getL()->size() - vTaskParts[taskPartNumber].index);

#ifdef DEBUG
  cout<< "vTaskParts[taskPartNumber].index: " << vTaskParts[taskPartNumber].index << endl;
#endif
  
  int l = getLeftScheduled(vTaskParts[taskPartNumber].index, s_task);
  int r = getRightScheduled(vTaskParts[taskPartNumber].index, s_task, task);
  
  if (l == numeric_limits<int>::min())
    range2.left = l;
  else
    {
      int s_task_l = getScheduleIndexOf(l, s_task);
      range2.left = s_task_l + (vTaskParts[taskPartNumber].index - l);
    }
  
  if (r == numeric_limits<int>::max())
    range2.right = r;
  else
    {
      int s_task_r = getScheduleIndexOf(r, s_task);
      range2.right = s_task_r - (r - vTaskParts[taskPartNumber].index);
    }
  
#ifdef DEBUG
  cout << "range1: [" << range1.left << ", " << range1.right << "]    range2: [" << range2.left << ", " << range2.right << "]" << endl;
#endif
  
  Range rangeTask = intersect(range1, range2);

#ifdef DEBUG
  cout << "Range: [" << rangeTask.left << ", " << rangeTask.right << "]" << endl;
#endif
  //Till here, range calculation
      


  vector<Pair> vPminInRange; // will keep an ordered list (low to high wrt Pmin) of time slots in the calculated range 
  for (int j = 0; j < schedulesLength; j++)
    {
      if (isInInterval(vPMin[j].index, rangeTask))
	{
	  Pair p;
	  p.index = vPMin[j].index;
	  p.value =  vPMin[j].value;
	  vPminInRange.push_back(p); // fixed. not j.
	}
    }
  //  sort(vPminInRange.begin(), vPminInRange.end());

#ifdef DEBUG
  cout << "sorted p_min in range: [";
  for(vector<Pair>::iterator it = vPminInRange.begin(); it != vPminInRange.end(); ++it)
    cout << "(" << (*it).index << ", " << (*it).value << "), " << endl;
#endif

  // for all time slots in the range ordered from low price to high, assign the current task part its time slot and continue scheduling next task parts by recursion
  bool foundFeasible = false;
  for(vector<Pair>::iterator it = vPminInRange.begin(); it != vPminInRange.end(); it++)
    {
      s_task[(*it).index] = vTaskParts[taskPartNumber].index;
#ifdef DEBUG
      cout << "Tree search given " << taskPartNumber << "-th task part is scheduled at " << (*it).index << endl;
#endif
      foundFeasible = scheduleTaskPart3(npHeurType, pHeurType, J, inputTab, pNondominatedSet, p_min, P_H, taskNumber, task, P_max, P_tot, vPMin, vTaskParts, s_task, sResult, taskPartNumber+1);
      if(foundFeasible)
	break;
      
      // IMPORTANT: before continuing with the next possible slot of the task part, deschedule it first!
      s_task[(*it).index] = -1;
    }
  return foundFeasible;

}
// END OF SCHEDULETASKPART3























////////////////////////////////////////////////////////////////////////////////////////
void
npTreeSearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber = 0)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();
  // Non-preemptive task scheduling
  // cout << "Non-preemptive task #" << taskNumber << " " << task->getName()
  //    << " is in process.." << endl;
  // alttaki iki satir daha iyi anlamak icin eklendi gereksiz islemler aslinda..
  // task->print();

  for (int startTime = task->getA(); startTime <= task->getD()
      - task->getL()->size(); startTime++)
    {

      // cout << task->getName() << " icin inputTab olusturuldu..." << endl;
      vector<Signal<int>*> tab(inputTab);

      //cout << "Task " << task->getName()
      //    << " is scheduling now for startTime: " << startTime << ".." << endl;

      Signal<int>* scheduleForStartTime = getNonPreemptiveSchedule(*task,
          startTime, schedulesLength);

      tab.push_back(scheduleForStartTime);

      //cout << "latest schedule for task: " << task->getName()
      //    << " has been copied to the tab.. for start time: " << startTime
      //    << endl;
      //printTab(tab);

      if (isConstraintSatisfied(tab, J, P_max))
        {
	  allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
		   taskNumber + 1);
          /*
	  if (taskNumber + 1 != (signed) J.size())
            {
              //cout
	      //  << "Moving to following task - Jumping the following tree branch-"
	      //  << endl;
              allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
                  taskNumber + 1);
            }
          else
            allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
                taskNumber + 1);
	  */

          // DONE: fix memory leakage
          vector<Signal<int>*>::iterator it = tab.end();
          it--;
          delete (*it);
          tab.pop_back();
        } // END if Pmax constraint satisfied

//      if (startTime + task->getL()->size() > task->getD())
//        break;

      //cout << "finished scheduling for non-preemptive task: "
      //  << task->getName() << " for start time: " << startTime << endl;
    } // END of for all possible start times
}





////////////////////////////////////////////////////////////////////////////////////////
void
npCostMinWithGreedySearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber = 0)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();
  vector<Signal<int>*> tab(inputTab);
  vector<Task*> Jv1;
  Jv1.push_back(J[taskNumber]);
  double minCostForThisTask = numeric_limits<double>::max();
  int startTimeForMinCost = 0;

  for (int startTime = task->getA();
        startTime <= task->getD() - task->getL()->size(); startTime++)
    {
      vector<Signal<int>*> tabForThisTask;
      Signal<int>* scheduleForStartTime = getNonPreemptiveSchedule(*task,
          startTime, schedulesLength);
      tabForThisTask.push_back(scheduleForStartTime);
      double cost = evaluateCost(tabForThisTask, Jv1, P_max, p_min, P_H);
      delete scheduleForStartTime;

      if (cost < minCostForThisTask)
        {
          minCostForThisTask = cost;
          startTimeForMinCost = startTime;
        }
    }

  Signal<int>* scheduleForStartTime = getNonPreemptiveSchedule(*task,
            startTimeForMinCost, schedulesLength);

  tab.push_back(scheduleForStartTime);

  if (taskNumber + 1 != (signed) J.size())
    {
      //cout << "Moving to following task: ";
      allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
          taskNumber + 1);
    }
  else
    {
      //cout << "no more tasks so we have reached to the leaf!!" << endl;
      allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
          taskNumber + 1);
    }

  // TODO: fix memory leakage
  vector<Signal<int>*>::iterator it = tab.end();
  it--;
  delete (*it);
  tab.pop_back();
}




#endif /* TAB_H_ */
