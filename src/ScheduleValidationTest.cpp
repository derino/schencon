#include <limits>
#include <iostream>
#include <cstring>
using namespace std;

#include "Signal.h"
#include "util.h"

#define L 20

template <typename T>
T* copy(T* src, int size)
{
  T* dst = new T[size];
  memcpy(dst, src, size*sizeof(T) );
  return dst;
}

int main()
{
/** Test with given schedules*/


////////////////////////////////////////////////////////////////
// input LOAD PROFILES
////////////////////////////////////////////////////////////////
  double l_j1[7]  = {1.17334, 1.17817, 1.17937, 1.17602, 1.17065, 1.16731, 1.1685};
  double l_j2[9] = {1.07826, 1.09386, 1.06266, 1.07826, 1.09386, 1.06266, 1.07826, 1.09386, 1.06266};
  double l_j3[10] = {0.925576, 0.925576, 0.925576, 0.925576, 0.925576, 0.925576, 0.925576, 0.925576, 0.925576, 0.925576};
  double l_j4[2] = {0.884668, 0.884668};
  double l_j5[4]  = {0.882262, 0.882262, 0.882262, 0.882262};
  double l_j6[9]  = {0.609946, 0.614305, 0.616625, 0.61582, 0.612266, 0.607626, 0.604072, 0.603266, 0.605586};
  double l_j7[6] = {0.69698, 0.69698, 0.69698, 0.69698, 0.69698, 0.69698};
  double l_j8[3]  = {1.19248, 1.32107, 1.0639};
  double l_j9[3] = {0.774788, 0.785141, 0.764435};
  double l_j10[3] = {0.629583, 0.701232, 0.557934};

  Signal<double> l1("l1", copy(l_j1, sizeof(l_j1)/sizeof(double)), L);
  Signal<double> l2("l2", copy(l_j2, sizeof(l_j2)/sizeof(double)), L);
  Signal<double> l3("l3", copy(l_j3, sizeof(l_j3)/sizeof(double)), L);
  Signal<double> l4("l4", copy(l_j4, sizeof(l_j4)/sizeof(double)), L);
  Signal<double> l5("l5", copy(l_j5, sizeof(l_j5)/sizeof(double)), L);
  Signal<double> l6("l6", copy(l_j6, sizeof(l_j6)/sizeof(double)), L);
  Signal<double> l7("l7", copy(l_j7, sizeof(l_j7)/sizeof(double)), L);
  Signal<double> l8("l8", copy(l_j8, sizeof(l_j8)/sizeof(double)), L);
  Signal<double> l9("l9", copy(l_j9, sizeof(l_j9)/sizeof(double)), L);
  Signal<double> l10("l10", copy(l_j10, sizeof(l_j10)/sizeof(double)), L);


////////////////////////////////////////////////////////////////
// input ILP schedule
////////////////////////////////////////////////////////////////
  int s_ilp_j1[L] = {1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j2[L] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j3[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0};
  int s_ilp_j4[L] = {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j5[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0};
  int s_ilp_j6[L] = {1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j7[L] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j8[L] = {0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j9[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
  int s_ilp_j10[L]= {0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};

  Signal<int> s1("j1", copy(s_ilp_j1, sizeof(s_ilp_j1)/sizeof(int)), L);
  Signal<int> s2("j2", copy(s_ilp_j2, sizeof(s_ilp_j2)/sizeof(int)), L);
  Signal<int> s3("j3", copy(s_ilp_j3, sizeof(s_ilp_j3)/sizeof(int)), L);
  Signal<int> s4("j4", copy(s_ilp_j4, sizeof(s_ilp_j4)/sizeof(int)), L);
  Signal<int> s5("j5", copy(s_ilp_j5, sizeof(s_ilp_j5)/sizeof(int)), L);
  Signal<int> s6("j6", copy(s_ilp_j6, sizeof(s_ilp_j6)/sizeof(int)), L);
  Signal<int> s7("j7", copy(s_ilp_j7, sizeof(s_ilp_j7)/sizeof(int)), L);
  Signal<int> s8("j8", copy(s_ilp_j8, sizeof(s_ilp_j8)/sizeof(int)), L);
  Signal<int> s9("j9", copy(s_ilp_j9, sizeof(s_ilp_j9)/sizeof(int)), L);
  Signal<int> s10("j10", copy(s_ilp_j10, sizeof(s_ilp_j10)/sizeof(int)), L);


////////////////////////////////////////////////////////////////
// input FTSG Pmax=ILP schedule
////////////////////////////////////////////////////////////////
  int s_ftsg2_j1[L] = {1, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j2[L] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j3[L] = {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0};
  int s_ftsg2_j4[L] = {0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j5[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j6[L] = {1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j7[L] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j8[L] = {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j9[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
  int s_ftsg2_j10[L]= {0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};

  Signal<int> ss1("j1", copy(s_ftsg2_j1, sizeof(s_ftsg2_j1)/sizeof(int)), L);
  Signal<int> ss2("j2", copy(s_ftsg2_j2, sizeof(s_ftsg2_j2)/sizeof(int)), L);
  Signal<int> ss3("j3", copy(s_ftsg2_j3, sizeof(s_ftsg2_j3)/sizeof(int)), L);
  Signal<int> ss4("j4", copy(s_ftsg2_j4, sizeof(s_ftsg2_j4)/sizeof(int)), L);
  Signal<int> ss5("j5", copy(s_ftsg2_j5, sizeof(s_ftsg2_j5)/sizeof(int)), L);
  Signal<int> ss6("j6", copy(s_ftsg2_j6, sizeof(s_ftsg2_j6)/sizeof(int)), L);
  Signal<int> ss7("j7", copy(s_ftsg2_j7, sizeof(s_ftsg2_j7)/sizeof(int)), L);
  Signal<int> ss8("j8", copy(s_ftsg2_j8, sizeof(s_ftsg2_j8)/sizeof(int)), L);
  Signal<int> ss9("j9", copy(s_ftsg2_j9, sizeof(s_ftsg2_j9)/sizeof(int)), L);
  Signal<int> ss10("j10", copy(s_ftsg2_j10, sizeof(s_ftsg2_j10)/sizeof(int)), L);


//   // PROBLEM
// ////////////////////////////////////////////////////////////////
// // input LOAD PROFILES
// ////////////////////////////////////////////////////////////////
//   double l_j1[4]  = {0.617624, 0.737464, 0.617624, 0.497784};
//   double l_j2[18] = {0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621, 0.957563, 1.050510, 0.864621};
//   double l_j3[10] = {1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185, 1.17185};
//   double l_j4[15] = {1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993, 1.23139, 1.25285, 1.20993};
//   double l_j5[4]  = {1.43065, 1.43065, 1.43065, 1.43065};
//   double l_j6[12]  = {0.883876, 0.891267, 0.876485, 0.883876, 0.891267, 0.876485, 0.883876, 0.891267, 0.876485, 0.883876, 0.891267, 0.876485};
//   double l_j7[1] = {1.20987};
//   double l_j8[16]  = {1.19792, 1.28938, 1.19792, 1.10647, 1.19792, 1.28938, 1.19792, 1.10647, 1.19792, 1.28938, 1.19792, 1.10647, 1.19792, 1.28938, 1.19792, 1.10647};
//   double l_j9[6] = {1.31315, 1.41079, 1.2155, 1.31315, 1.41079, 1.2155};
//   double l_j10[20] = {1.04765, 1.19587, 1.13925, 0.956042, 0.899427, 1.04765, 1.19587, 1.13925, 0.956042, 0.899427, 1.04765, 1.19587, 1.13925, 0.956042, 0.899427, 1.04765, 1.19587, 1.13925, 0.956042, 0.899427};

//   Signal<double> l1("l1", copy(l_j1, sizeof(l_j1)/sizeof(double)), L);
//   Signal<double> l2("l2", copy(l_j2, sizeof(l_j2)/sizeof(double)), L);
//   Signal<double> l3("l3", copy(l_j3, sizeof(l_j3)/sizeof(double)), L);
//   Signal<double> l4("l4", copy(l_j4, sizeof(l_j4)/sizeof(double)), L);
//   Signal<double> l5("l5", copy(l_j5, sizeof(l_j5)/sizeof(double)), L);
//   Signal<double> l6("l6", copy(l_j6, sizeof(l_j6)/sizeof(double)), L);
//   Signal<double> l7("l7", copy(l_j7, sizeof(l_j7)/sizeof(double)), L);
//   Signal<double> l8("l8", copy(l_j8, sizeof(l_j8)/sizeof(double)), L);
//   Signal<double> l9("l9", copy(l_j9, sizeof(l_j9)/sizeof(double)), L);
//   Signal<double> l10("l10", copy(l_j10, sizeof(l_j10)/sizeof(double)), L);


// ////////////////////////////////////////////////////////////////
// // input ILP schedule
// ////////////////////////////////////////////////////////////////
//   int s_ilp_j1[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0};
//   int s_ilp_j2[L] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0};
//   int s_ilp_j3[L] = {1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ilp_j4[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
//   int s_ilp_j5[L] = {1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ilp_j6[L] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
//   int s_ilp_j7[L] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ilp_j8[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
//   int s_ilp_j9[L] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ilp_j10[L]= {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

//   Signal<int> s1("j1", copy(s_ilp_j1, sizeof(s_ilp_j1)/sizeof(int)), L);
//   Signal<int> s2("j2", copy(s_ilp_j2, sizeof(s_ilp_j2)/sizeof(int)), L);
//   Signal<int> s3("j3", copy(s_ilp_j3, sizeof(s_ilp_j3)/sizeof(int)), L);
//   Signal<int> s4("j4", copy(s_ilp_j4, sizeof(s_ilp_j4)/sizeof(int)), L);
//   Signal<int> s5("j5", copy(s_ilp_j5, sizeof(s_ilp_j5)/sizeof(int)), L);
//   Signal<int> s6("j6", copy(s_ilp_j6, sizeof(s_ilp_j6)/sizeof(int)), L);
//   Signal<int> s7("j7", copy(s_ilp_j7, sizeof(s_ilp_j7)/sizeof(int)), L);
//   Signal<int> s8("j8", copy(s_ilp_j8, sizeof(s_ilp_j8)/sizeof(int)), L);
//   Signal<int> s9("j9", copy(s_ilp_j9, sizeof(s_ilp_j9)/sizeof(int)), L);
//   Signal<int> s10("j10", copy(s_ilp_j10, sizeof(s_ilp_j10)/sizeof(int)), L);


// ////////////////////////////////////////////////////////////////
// // input FTSG Pmax=ILP schedule
// ////////////////////////////////////////////////////////////////
//   int s_ftsg2_j1[L] = {0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ftsg2_j2[L] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0};
//   int s_ftsg2_j3[L] = {1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ftsg2_j4[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
//   int s_ftsg2_j5[L] = {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ftsg2_j6[L] = {0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0};
//   int s_ftsg2_j7[L] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ftsg2_j8[L] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0};
//   int s_ftsg2_j9[L] = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//   int s_ftsg2_j10[L]= {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

//   Signal<int> ss1("j1", copy(s_ftsg2_j1, sizeof(s_ftsg2_j1)/sizeof(int)), L);
//   Signal<int> ss2("j2", copy(s_ftsg2_j2, sizeof(s_ftsg2_j2)/sizeof(int)), L);
//   Signal<int> ss3("j3", copy(s_ftsg2_j3, sizeof(s_ftsg2_j3)/sizeof(int)), L);
//   Signal<int> ss4("j4", copy(s_ftsg2_j4, sizeof(s_ftsg2_j4)/sizeof(int)), L);
//   Signal<int> ss5("j5", copy(s_ftsg2_j5, sizeof(s_ftsg2_j5)/sizeof(int)), L);
//   Signal<int> ss6("j6", copy(s_ftsg2_j6, sizeof(s_ftsg2_j6)/sizeof(int)), L);
//   Signal<int> ss7("j7", copy(s_ftsg2_j7, sizeof(s_ftsg2_j7)/sizeof(int)), L);
//   Signal<int> ss8("j8", copy(s_ftsg2_j8, sizeof(s_ftsg2_j8)/sizeof(int)), L);
//   Signal<int> ss9("j9", copy(s_ftsg2_j9, sizeof(s_ftsg2_j9)/sizeof(int)), L);
//   Signal<int> ss10("j10", copy(s_ftsg2_j10, sizeof(s_ftsg2_j10)/sizeof(int)), L);




////////////////////////////////////////////////////////////////
// output ILP peak
////////////////////////////////////////////////////////////////
  Signal<double>* P_1 = P_i(&l1, &s1);
  Signal<double>* P_2 = P_i(&l2, &s2);
  Signal<double>* P_3 = P_i(&l3, &s3);
  Signal<double>* P_4 = P_i(&l4, &s4);
  Signal<double>* P_5 = P_i(&l5, &s5);
  Signal<double>* P_6 = P_i(&l6, &s6);
  Signal<double>* P_7 = P_i(&l7, &s7);
  Signal<double>* P_8 = P_i(&l8, &s8);
  Signal<double>* P_9 = P_i(&l9, &s9);
  Signal<double>* P_10 = P_i(&l10, &s10);

  Signal<double>* P_0 = new Signal<double>("all 0s", L, 0);
  Signal<double>* P_tot = P_0;
  P_tot = *P_tot + *P_1;
  P_tot = *P_tot + *P_2;
  P_tot = *P_tot + *P_3;
  P_tot = *P_tot + *P_4;
  P_tot = *P_tot + *P_5;
  P_tot = *P_tot + *P_6;
  P_tot = *P_tot + *P_7;
  P_tot = *P_tot + *P_8;
  P_tot = *P_tot + *P_9;
  P_tot = *P_tot + *P_10;
  cout << max(*P_tot) << endl;

////////////////////////////////////////////////////////////////
// output FTSG Pmax=ILP peak
////////////////////////////////////////////////////////////////
  Signal<double>* PP_1 = P_i(&l1, &ss1);
  Signal<double>* PP_2 = P_i(&l2, &ss2);
  Signal<double>* PP_3 = P_i(&l3, &ss3);
  Signal<double>* PP_4 = P_i(&l4, &ss4);
  Signal<double>* PP_5 = P_i(&l5, &ss5);
  Signal<double>* PP_6 = P_i(&l6, &ss6);
  Signal<double>* PP_7 = P_i(&l7, &ss7);
  Signal<double>* PP_8 = P_i(&l8, &ss8);
  Signal<double>* PP_9 = P_i(&l9, &ss9);
  Signal<double>* PP_10 = P_i(&l10, &ss10);

  Signal<double>* PP_0 = new Signal<double>("all 0s", L, 0);
  Signal<double>* PP_tot = PP_0;
  PP_tot = *PP_tot + *PP_1;
  PP_tot = *PP_tot + *PP_2;
  PP_tot = *PP_tot + *PP_3;
  PP_tot = *PP_tot + *PP_4;
  PP_tot = *PP_tot + *PP_5;
  PP_tot = *PP_tot + *PP_6;
  PP_tot = *PP_tot + *PP_7;
  PP_tot = *PP_tot + *PP_8;
  PP_tot = *PP_tot + *PP_9;
  PP_tot = *PP_tot + *PP_10;
  cout << max(*PP_tot) << endl;

  //  PP_tot->printAsArray(cout);

  return 0;
}
