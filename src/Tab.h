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


enum NPHeuristicType {NP_COST_MIN_WITH_TREE_SEARCH=0, NP_COST_MIN_WITH_GREEDY_SEARCH, NP_PEAK_MIN_WITH_TREE_SEARCH}; // NP_ASAP

enum PHeuristicType {P_PEAK_MINIMIZATION=0, P_COST_MINIMIZATION /*p_max aware cost minimization*/,
  P_PMAX_AWARE/*doesn't care about cost minimization*/}; // P_ASAP



void
npTreeSearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber);

void
npCostMinWithGreedySearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);

void
pCostMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber);

void
pPeakMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J,
    vector<Signal<int>*>& inputTab, TNondominatedSet* pNondominatedSet,
    double P_max, Signal<double>& p_min, Signal<double>& P_H, int taskNumber);





void
allocTab(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber = 0)
{



  if (taskNumber == (signed) J.size())
    {
      cout << "no more tasks so we have reached to the leaf!!" << endl;
      // Reached leaf
      // calculate cost, compare with minimum cost
      double cost = evaluateCost(inputTab, J, P_max, p_min, P_H);
      cout << "Cost for this tree path: " << cost << endl;

      double peak = evaluateCostForLees(inputTab, J);
      //TODO:add tab to SchedulingMOMHSolution

      vector<Signal<int>*>* tab = new vector<Signal<int>*>();
      for ( vector<Signal<int>*>::iterator it = inputTab.begin(); it != inputTab.end(); it++)
        {
          Signal<int>* sit = *it;
          Signal<int>* sitCopy = new Signal<int>(*sit);
          tab->push_back(sitCopy);
        }
//      vector<Signal<int>*>* tab = new vector<Signal<int>*>(inputTab);

      SchedulingMOMHSolution* sol = new SchedulingMOMHSolution(peak, cost);
      sol->setTab(tab);
      if ( pNondominatedSet->Update(*sol) == true )
      {
          cout << "new pareto point" << endl;
      }
      else
        delete sol;

      return;
    }

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
      default:
        cout << "ERROR: undefined heuristic type!" << endl;
        break;

        }
    }

}

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

  Signal<double>* P_tot = P__tot2(inputTab, J);

  P_tot->print();

  vector<Pair> vPTotInterval;

  for (int i = task->getA(); i < task->getD(); i++)
    {
      Pair p;
      p.index = i;
      p.value = P_tot->at(i);
      vPTotInterval.push_back(p);
    }

  sort(vPTotInterval.begin(), vPTotInterval.end(), compare);
  cout << "sorted p_tot: [";
  for (vector<Pair>::iterator it = vPTotInterval.begin(); it
      != vPTotInterval.end(); ++it)
    cout << "(" << (*it).index << "," << (*it).value << ")";
  cout << "]" << endl;
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

  //printTab(tab);

  allocTab(npHeurType, pHeurType, J, inputTab, pNondominatedSet, P_max, p_min, P_H, taskNumber + 1);

  // TODO: fix memory leakage
  vector<Signal<int>*>::iterator it = inputTab.end();
  it--;
  delete (*it);
  inputTab.pop_back();

  //TODO: Adamin cost function i olarak: P_tot'undaki maksimum deger bulunacak.

}





void
pCostMinimization(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();

  //vector<Signal<int>*> tab(inputTab);

  Signal<double>* P_tot = P__tot2(inputTab, J);

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
              // TODO: check if Pmax is violated for selected slot...
              //vPMin[j].index'te bir deger varmis gibi yeni bir s yaratilacak.
              //s taba eklenip Ptot hesaplanacak o slottaki
              // En son da eklenen schedulei tabtan sil
              if (P_tot->at(vPMin[j].index) + task->getL()->at(k) <= P_max)
                {
                  s_task[vPMin[j].index] = vTaskParts[k].index;
                  //                                          cout << "s_task[" << vPMin[j].index << "]: " << vTaskParts[k].index << endl;
                  break;
                }
            }
        }
    }
  cout << "Found Schedule: ";
  for (unsigned int i = 0; i < s_task.size(); i++)
    {
      cout << s_task[i] << " ";
    }
  cout << endl;
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


void
npTreeSearch(NPHeuristicType npHeurType, PHeuristicType pHeurType, vector<Task*>& J, vector<Signal<int>*>& inputTab,
    TNondominatedSet* pNondominatedSet, double P_max, Signal<double>& p_min,
    Signal<double>& P_H, int taskNumber = 0)
{
  Task* task = J[taskNumber];
  int schedulesLength = p_min.size();
  // Non-preemptive task scheduling
  cout << "Non-preemptive task #" << taskNumber << " " << task->getName()
      << " is in process.." << endl;
  //alttaki iki satir daha iyi anlamak icin eklendi gereksiz islemler aslinda..
  task->print();

  for (int startTime = task->getA(); startTime <= task->getD()
      - task->getL()->size(); startTime++)
    {

      cout << task->getName() << " icin inputTab olusturuldu..." << endl;
      vector<Signal<int>*> tab(inputTab);

      cout << "Task " << task->getName()
          << " is scheduling now for startTime: " << startTime << ".." << endl;

      Signal<int>* scheduleForStartTime = getNonPreemptiveSchedule(*task,
          startTime, schedulesLength);

      tab.push_back(scheduleForStartTime);

      cout << "latest schedule for task: " << task->getName()
          << " has been copied to the tab.. for start time: " << startTime
          << endl;
      //printTab(tab);

      if (isConstraintSatisfied(tab, J, P_max))
        {
          if (taskNumber + 1 != (signed) J.size())
            {
              cout
                  << "Moving to following task - Jumping the following tree branch-"
                  << endl;
              allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
                  taskNumber + 1);
            }
          else
            allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
                taskNumber + 1);

          // TODO: fix memory leakage
          vector<Signal<int>*>::iterator it = tab.end();
          it--;
          delete (*it);
          tab.pop_back();
        }
//      if (startTime + task->getL()->size() > task->getD())
//        break;


      cout << "finished scheduling for non-preemptive task: "
          << task->getName() << " for start time: " << startTime << endl;
    }
}


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
      cout << "Moving to following task: ";
      allocTab(npHeurType, pHeurType, J, tab, pNondominatedSet, P_max, p_min, P_H,
          taskNumber + 1);
    }
  else
    {
      cout << "no more tasks so we have reached to the leaf!!" << endl;
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
