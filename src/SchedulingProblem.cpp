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

Signal<double>* SchedulingProblem::pMin()
{
  return _pMin;
}

void SchedulingProblem::setPMin(Signal<double>* pMin)
{
  _pMin = pMin;
}

Signal<double>* SchedulingProblem::PH()
{
  return _PH;
}


double SchedulingProblem::PMax()
{
  return _PMax;
}


double SchedulingProblem::PMaxLow()
{
  return _PMaxLow;
}

double SchedulingProblem::PMaxMedium()
{
  return _PMaxMedium;
}

double SchedulingProblem::PMaxHigh()
{
  return _PMaxHigh;
}


void SchedulingProblem::setPMax(double PMax)
{
  _PMax = PMax;
}


void SchedulingProblem::setPMaxRange(double PMaxLow, double PMaxMedium, double PMaxHigh)
{
  _PMaxLow = PMaxLow;
  _PMaxMedium = PMaxMedium;
  _PMaxHigh = PMaxHigh;

  // default P_max value is _PMaxMedium
  _PMax = _PMaxMedium;
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

void SchedulingProblem::setPH()
{
  // assume P_H is a 0 signal.
  _PH = new Signal<double>("P_H", _L, 0.0);
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
  _str << "Problem_N" << _N << "_L" << _L << "_LPAR" << _LPAR << "_PrR" << _PrR;
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
  
  fout1 << _N << endl // task set size
  	<< _L << endl // schedule length
    	<< _LPAR << endl // low peak to average ration
  	<< _PrR << endl; // ratio of preemptible tasks
  // 	<< E() << endl
  // 	<< endl;
  fout1.close();

  for (int n = 0; n < _N; n++)
  {
    ofstream fout2( (_J.at(n)->getName() + ".txt").c_str() );
    fout2 << _J.at(n)->getA() << endl
	  << _J.at(n)->getD() << endl
	  << _J.at(n)->getPr() << endl;
    fout2.close();

    ofstream fout3( (_J.at(n)->getL()->getName() + ".txt").c_str() );
    _J.at(n)->getL()->print(fout3);
    fout3.close();
  } 

  // write p_min
  ofstream fout4( "p_min.txt" );
  _pMin->print(fout4);
  fout4.close();

  // write Pmax
  ofstream fout5( "P_max.txt" );
  fout5 << _PMaxLow << endl
	<< _PMaxMedium << endl
	<< _PMaxHigh << endl;
  fout5.close();
  
  // TODO
}
