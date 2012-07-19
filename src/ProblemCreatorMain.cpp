#include <stdio.h>
#include <stdlib.h>
#include <fstream>  
#include <vector>
#include <algorithm>
#include <limits>
#include <sstream>
#include <math.h>
//#include <boost/lambda/lambda.hpp>

#include "Signal.h"
#include "Task.h"
#include "SchedulingProblem.h"
using namespace std;
//using namespace boost::lambda;



// Global Variables

int M = 96;


float rand01()
{
  return ((float) rand()) / RAND_MAX;
}

Task* createTask(int i, bool preemptive, float par)
{
  Task* j = new Task();

  // - set task's name
  stringstream jNameStr;
  jNameStr << "J_" << i;
  j->setName( jNameStr.str() );

  // - set task's arrival time
  j->setA(0);

  // - set task's deadline
  // Task period
  int tp = 2 + rand() % 22;

  //  Max # of repeating periods = maxR
  int maxR = M/tp; // floor of the division. not necessary due to dividing two ints.

  //  # of repeating periods = R
  int R = 1 + rand() % maxR;          //gives a number in [1,maxR]

  // Randomly deadline assigning
  j->setD( ((M - tp)*rand01()) + tp );

  // - set task's load profile
  //  average power of sinosoid: aS
  float aS = 0.5 + rand01();
  float bS = (par-1)*aS;

  stringstream lNameStr;
  lNameStr << "L_" << i;
  Signal<double>* l = new Signal<double>(lNameStr.str(), tp*R, 0);

  for( int n=0; n<tp*R; n++)
    {
      // calculate value at n
      double val = aS + bS * sin (2 * M_PI * n / tp);
      l->setValueAt(n, val);
    }

  j->setL(l);

  return j;
}


int main()
{

  //>>>>>>>> Synthetic Task Set Creator <<<<<<<<<

  /* initialize random seed: */
  srand ( time(NULL) );


  // Preemptive Ratio
  float prr_arr[6] = {0, 0.2, 0.4, 0.6, 0.8, 1};

  // Task Set Size
  int tss = 10;

  // Peak to Average Ratio
  float lpar_arr[3] = { 0, 0.5, 1};

  //task features
  bool preemptive;
  float par;

  for (int a = 0; a < 6; a++)
    {
      cout << endl << "prr_arr[" << a << "] = " << prr_arr[a] << endl;

      for (int b = 0; b < 3; b++)
	{
	  cout << endl << "\t lpar_arr[" << b << "] = " << lpar_arr[b] << endl;

	  // create problem
	  SchedulingProblem* sp = new SchedulingProblem();
	  sp->setPrR( prr_arr[a] );
	  sp->setLPAR( lpar_arr[b] );
	  sp->setN(tss);

	  for (int i = 0; i < tss; i++)
	    {
	      float r = rand01();
	      if(r < prr_arr[a])
		preemptive = true;
	      else
		preemptive = false;

	      float r2 = rand01();
	      if(r2 < lpar_arr[b])
		par = 1 + 0.2*rand01();
	      else
		par = 1.8 + 0.2*rand01();

	      Task* j_i = createTask(i,preemptive, par);
	      sp->J()->push_back(j_i);

	      //cout << preemptive << "\t" << par << endl;
	    }
	  //      TODO:Problem ismine uygun folder olusturup, bir onceki loopta olusan taski icine at
	  // create problem folder
	  // change curr. dir. to the problem folder
	  // write problem files
	  sp->write();

	}
    }
  return 0;
  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Syntetic Task Set Creater<<<<<<<<<<<<<<<<<<<<<<<<<<<<

}
