// Copyright (c) 2000 by the University of Southern California
// All rights reserved.
//
// Permission to use, copy, modify, and distribute this software and its
// documentation in source and binary forms for non-commercial purposes
// and without fee is hereby granted, provided that the above copyright
// notice appear in all copies and that both the copyright notice and
// this permission notice appear in supporting documentation. and that
// any documentation, advertising materials, and other materials related
// to such distribution and use acknowledge that the software was
// developed by the University of Southern California, Information
// Sciences Institute.  The name of the University may not be used to
// endorse or promote products derived from this software without
// specific prior written permission.
//
// THE UNIVERSITY OF SOUTHERN CALIFORNIA makes no representations about
// the suitability of this software for any purpose.  THIS SOFTWARE IS
// PROVIDED "AS IS" AND WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// Other copyrights might apply to parts of this software and are so
// noted when applicable.
//

/*****************************************************************/
/*  diff_sink.h : Header file for sink agent                     */
/*  By     : Chalermek Intanagonwiwat (USC/ISI)                  */
/*  Last Modified : 8/21/99                                      */
/*****************************************************************/


#ifndef ns_uw_sink_h
#define ns_uw_sink_h

#include <tcl.h>

#include "agent.h"
#include "tclcl.h"
#include "packet.h"
#include "ip.h"
//#include "uwvb_header.h"
//#include "diffusion.h" //!!! need to be changed!!
//#include "uw_hash_table.h"
#include "underwatersensornode.h"
#include "underwatersensor/uw_routing/vectorbasedforward.h"



class UWSinkAgent;

// Timer for packet rate

class UWSink_Timer : public TimerHandler {
 public:
	UWSink_Timer(UWSinkAgent *a) : TimerHandler() { a_ = a; }
 protected:
	virtual void expire(Event *e);
	UWSinkAgent *a_;
};


// For periodic report of avg and var pkt received.

class UWReport_Timer : public TimerHandler {
 public:
	UWReport_Timer(UWSinkAgent *a) : TimerHandler() { a_ = a; }
 protected:
	virtual void expire(Event *e);
	UWSinkAgent *a_;
};


// Timer for periodic interest

class UWPeriodic_Timer : public TimerHandler {
 public:
	UWPeriodic_Timer(UWSinkAgent *a) : TimerHandler() { a_ = a; }
 protected:
	virtual void expire(Event *e);
	UWSinkAgent *a_;
};


// Class SinkAgent as source and sink for directed diffusion

class UWSinkAgent : public Agent {

 public:
  UWSinkAgent();
  int command(int argc, const char*const* argv);
  virtual void timeout(int);

  void report();
  void recv(Packet*, Handler*);
  void reset();
  void set_addr(ns_addr_t);
  int get_pk_count();
  void incr_pk_count();
  Packet *create_packet();

 protected:
  bool APP_DUP_;
  bool periodic_;
  //bool always_max_rate_;
  int pk_count;
  unsigned int data_type_;
  int num_recv;
  int num_send;
  int RecvPerSec; //? what's this for

  /*used ti indicate if the sink is active, send out interest first or 
passive, it gets the data ready and then sends out the interest. 1 is passive 
and 0 is active.*/
  
  int passive;


 
  ns_addr_t target_id;
  double target_x;
  double target_y;
  double target_z;
  double range_;
  char   f_name[80];

  UnderwaterSensorNode* node;
 
  double cum_delay;
  double last_arrival_time;

  UWData_Hash_Table DataTable;
  bool IsDeviation();
  void Terminate();
  void bcast_interest();
  void source_deny(ns_addr_t,double,double,double);
  void data_ready();
  void start();
  void stop();
  virtual void sendpkt();

  int running_;
  int random_;
  int maxpkts_;

  double interval_; // interval to send data pkt
  double explore_interval;
  double data_interval;
  int data_rate;
  int explore_rate;
  int data_counter;
  int  explore_counter;
  int explore_status;
  


  //int simple_report_rate;
    //  int data_counter;
 
  UWSink_Timer sink_timer_;
  UWPeriodic_Timer periodic_timer_;
  UWReport_Timer report_timer_;

  friend class UWPeriodic_Timer;
};


#endif











