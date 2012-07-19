#ifndef TASK_H
#define TASK_H

#include <iostream>
#include <fstream>  
#include <sstream>  
#include <assert.h>

using namespace std;

#include "Signal.h"

class Task
{
public:
  Task();
  Task(string, int, int, bool, string);
  Task(int fileID);
  ~Task();
  string getName();
  void setName(string);
  int getA();
  void setA(int);
  int getD();
  void setD(int);
  bool getPr();
  void setPr(bool);
  Signal<double>* getL();
  void setL(Signal<double>*);
  void print();

private:
  string _name;
  int _a;
  int _d;
  bool _pr;
  Signal<double>* _L;
};



#endif
