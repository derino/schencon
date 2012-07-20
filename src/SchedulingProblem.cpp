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
#include "SchedulingProblem.h"

SchedulingProblem::SchedulingProblem()
{
}

void SchedulingProblem::setPMin(Signal<double>* pMin)
{
  _pMin = pMin;
}

double SchedulingProblem::PMax()
{
  return _PMax;
}

void SchedulingProblem::setPMax(double PMax)
{
  _PMax = PMax;
}

int SchedulingProblem::N()
{
  return _N;
}

void SchedulingProblem::setN(int N)
{
  _N = N;
}

int SchedulingProblem::L()
{
  return _L;
}

void SchedulingProblem::setL(int L)
{
  _L = L;
}

void SchedulingProblem::setLPAR(double lpar)
{
  _LPAR = lpar;
}

void SchedulingProblem::setPrR(double prr)
{
  _PrR = prr;
}

vector<Task*>* SchedulingProblem::J()
{
  return &_J;
}

string SchedulingProblem::name()
{
  // TODO add other parameters of the problem such as LPAR, PRR etc.
  std::stringstream _str;
  _str << "Problem_N" << _N << "_LPAR" << _LPAR << "_PrR" << _PrR;
  return _str.str();
}

void SchedulingProblem::print()
{

  // print problem name
  cout << name() << endl;

  // print P_max
  cout << "P_max:" << endl;
  cout << _PMax << endl;

  // print p_min
  _pMin->print();

  for(int i=0; i< _J.size(); i++)
    {
      cout << "TASK" << endl;
      _J.at(i)->print();
    }
}

void SchedulingProblem::write()
{
  // TODO Yucel

  // write problem.txt file.
  ofstream fout1("problem.txt");
  //ofstream fout1( (filename+".txt").c_str() );
  
  fout1 << "TaskSet Size:\t" << _N << endl
    	<< "Low Par Ration:\t" << _LPAR << endl
  	<< "Preemptive Ratio:\t" << _PrR << endl;
  // 	<< E() << endl
  // 	<< endl;
  fout1.close();

}
