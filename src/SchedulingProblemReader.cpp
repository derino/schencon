/** 
  * Copyright (c) 2012, Universita' della Svizzera Italiana,
  * All rights reserved.
  * 
  * Redistribution and use in source and binary forms, with or without modification, 
  * are permitted provided that the following conditions are met:
  * 
  * 1. Redistributions of source code must retain the above copyright notice, this list 
  * of conditions and the following disclaimer.
  * 
  * 2. Redistributions in binary form must reproduce the above copyright notice, this 
  * list of conditions and the following disclaimer in the documentation and/or other 
  * materials provided with the distribution.
  * 
  * 3. Neither the name of Universita' della Svizzera Italiana nor the names of its 
  * contributors may be used to endorse or promote products derived from this software 
  * without specific prior written permission.
  * 
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
  * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
  * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
  * SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * 
  * The licenses for the third party libraries used in this software can be found in 
  * the folders of the third party libraries.
  *
  * Author(s): Onur Derin <oderin@users.sourceforge.net>
  * 
  */
#include "SchedulingProblemReader.h"

SchedulingProblemReader::SchedulingProblemReader()
{
}

SchedulingProblemReader::~SchedulingProblemReader()
{
}

SchedulingProblem* SchedulingProblemReader::read()
{
  SchedulingProblem* sp = new SchedulingProblem();

  // read problem.txt
  ifstream fin("problem.txt");
  if ( !fin.is_open())
    {
      cout << "Unable to open file problem.txt" << endl; 
      exit(EXIT_FAILURE);
    }
  // - read N (number of tasks)
  int N = -1;
  fin >> N;
  sp->setN(N);
  // - read L (schedule length)
  int L = -1;
  fin >> L;
  sp->setL(L);
  fin.close();

  // set PH as all zeros
  sp->setPH();

  // read P_max.txt
  ifstream fin2("P_max.txt");
  if ( !fin2.is_open())
    {
      cout << "Unable to open file P_max.txt" << endl; 
      exit(EXIT_FAILURE);
    }
  double PMaxLow = -1;
  double PMaxMedium = -1;
  double PMaxHigh = -1;
  fin2 >> PMaxLow;
  fin2 >> PMaxMedium;
  fin2 >> PMaxHigh;
  sp->setPMaxRange(PMaxLow, PMaxMedium, PMaxHigh);
  fin2.close();

  // read price signal
  Signal<double>* pMin = new Signal<double>("p_min", "p_min.txt");
  sp->setPMin(pMin);

  // Task set for case_study_T20_min
  for(int i=1; i<N+1; i++)
    {
      Task* t = new Task(i); // assumes the presence of a task file with name, for ex,  J1.txt for i=1
      sp->J()->push_back(t);
    }

  return sp;
}

