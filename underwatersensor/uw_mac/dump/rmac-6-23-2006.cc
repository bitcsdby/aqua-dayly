#include "packet.h"
#include "random.h"
#include "underwatersensor/uw_common/underwatersensornode.h"
#include "underwatersensor/uw_routing/vectorbasedforward.h"
#include "mac.h"
#include "rmac.h"
#include "underwaterphy.h"
#include "random.h"



void 
TransmissionBuffer::AddNewPacket(Packet* p){
  buffer_cell* t2;
  buffer_cell* t1=new buffer_cell;

  t1->packet=p;
  t1->next=NULL;

  if(head_==NULL) {
     tail_=t1;
     head_=t1;
  }
  else{
  tail_->next=t1;
  tail_=t1;
  }
  
  num_of_packet++;
}


Packet* 
TransmissionBuffer::head(){
  buffer_cell* t1;
  buffer_cell* t2;
  Packet* p;
  
  if(!head_) return NULL;
  else return head_->packet;
}


Packet* 
TransmissionBuffer::dehead(){
  buffer_cell* t1;
  buffer_cell* t2;
  Packet* p;
  
  if(!head_) return NULL;
   p=head_->packet;
   t1=head_->next;
   t2=head_;
 
   head_=t1;
   num_of_packet--;
   
   if(head_==NULL) tail_=NULL; 
    delete t2;
   return p;
}


Packet* 
TransmissionBuffer::next(){
  Packet* p;
  if(!current_p) return NULL;
  p=current_p->packet;
  current_p=current_p->next;
  Packet* p1=p->copy();
   return p1;
}


int 
TransmissionBuffer::DeletePacket(Packet* p){
  buffer_cell* t1;
  buffer_cell* t2;
  
  // insert this packet at the head of the link
  t2=head_;
  

  if (!t2) return 0;//0 no such point, 1:delete this point

  if (p==t2->packet){
    printf("underwatermac: the packet is at the head of list\n");
    head_=t2->next;
    num_of_packet--;

   if(head_==NULL) tail_=NULL;
   
    Packet::free(p);  
     delete t2;
    
    return 1;
}
  
  int modified=0;
  while(t2->next){
    if ((t2->next)->packet!=p) t2=t2->next;
    else{
    
     t1=t2->next;
     t2->next=t1->next;

     if(t1==tail_) tail_=t2;
     num_of_packet--;
    delete t1;
    Packet::free(p);   
    modified=1;
    }
  }
  
  return modified;
}


buffer_cell*  
TransmissionBuffer::lookup(Packet* p){
  buffer_cell* t2;
  t2=head_;  
  while((t2->packet!=p)&&(!t2)) t2=t2->next;
  return t2;
}


void 
TransmissionBuffer::LockBuffer(){
  current_p=head_;
  lock_p=tail_;
  lock=true;
}


void 
TransmissionBuffer::UnlockBuffer(){
  lock=false;
  lock_p=NULL;
}


bool 
TransmissionBuffer::IsEmpty(){
  return(0==num_of_packet);
}

bool 
TransmissionBuffer::ToBeFull(){
  return((MAXIMUM_BUFFER-1)==num_of_packet);
}




bool 
TransmissionBuffer::IsEnd(){
  if (lock_p) return (lock_p->next==current_p);
  return(NULL==current_p);
}



bool 
TransmissionBuffer::IsFull(){
  return(MAXIMUM_BUFFER==num_of_packet);
}



int hdr_rmac::offset_;

/*
int hdr_nd::offset_;
int hdr_ack_nd::offset_;
int hdr_syn::offset_;
int hdr_rev::offset_;
int hdr_ack_rev::offset_;
int hdr_rmac_data::offset_;
*/

static class RMACHeaderClass: public PacketHeaderClass{
 public:
  RMACHeaderClass():PacketHeaderClass("PacketHeader/RMAC",sizeof(hdr_rmac))
{
 bind_offset(&hdr_rmac::offset_);
}
} class_rmachdr;


/*

static class RMAC_REV_HeaderClass: public PacketHeaderClass{
 public:
  RMAC_REV_HeaderClass():PacketHeaderClass("Packetheader/RMAC_REV",sizeof(hdr_rev))
{
 bind_offset(&hdr_rev::offset_);
}
} class_rmac_rev_hdr;



static class RMAC_ACKREV_HeaderClass: public PacketHeaderClass{
 public:
  RMAC_ACKREV_HeaderClass():PacketHeaderClass("Packetheader/RMAC_ACKREV",sizeof(hdr_ack_rev))
{
 bind_offset(&hdr_ack_rev::offset_);
}
} class_rmac_ack_rev_hdr;



static class RMAC_ND_HeaderClass: public PacketHeaderClass{
 public:
 RMAC_ND_HeaderClass():PacketHeaderClass("Packetheader/RMAC_ND",sizeof(hdr_nd))
{
 bind_offset(&hdr_nd::offset_);
}
} class_rmac_nd_hdr;


static class RMAC_ACKND_HeaderClass: public PacketHeaderClass{
 public:
  RMAC_ACKND_HeaderClass():PacketHeaderClass("Packetheader/RMAC_ACKND",sizeof(hdr_ack_nd))
{
 bind_offset(&hdr_ack_nd::offset_);
}
} class_rmac_ack_nd_hdr;


static class RMAC_SYN_HeaderClass: public PacketHeaderClass{
 public:
 RMAC_SYN_HeaderClass():PacketHeaderClass("Packetheader/RMAC_SYN",sizeof(hdr_syn))
{
 bind_offset(&hdr_syn::offset_);
}
} class_rmac_syn_hdr;

*/

MACRECVHandler::MACRECVHandler(RMac* p): mac_(p), duration(0), status(0){}

void 
MACRECVHandler::handle(Event* e)
{
  mac_->StartRECV(duration, status);
}

ClearChannelHandler::ClearChannelHandler(RMac* p): mac_(p){}

void 
ClearChannelHandler::handle(Event* e)
{
  mac_->ClearChannel();
}




ACKREVHandler::ACKREVHandler(RMac* p): mac_(p){}

void 
ACKREVHandler::handle(Event* e)
{
  printf("ACKREVHandler: node%d handle ackrev\n",mac_->index_);
  mac_->TxACKRev((Packet*) e);
}


TimeoutHandler::TimeoutHandler(RMac* p): mac_(p){}

void 
TimeoutHandler::handle(Event* e)
{
 
  mac_->ResetMacStatus();
}


SleepHandler::SleepHandler(RMac* p): mac_(p){}

void 
SleepHandler::handle(Event* e)
{
  mac_->ProcessSleep();
}

WakeupHandler::WakeupHandler(RMac* p): mac_(p){}

void 
WakeupHandler::handle(Event* e)
{
  mac_->Wakeup();
}



NDBackoffHandler::NDBackoffHandler(RMac* p):mac_(p),window_(0),counter_(0){}
 
void NDBackoffHandler::handle(Event*e)
{
  counter_++;
  if(counter_<MAXIMUMBACKOFF)
    mac_->TxND((Packet*)e, window_);
  else 
    {
    clear();
  printf("Rmac:backoff:too many backoffs\n");
  Packet::free((Packet*)e);
    }
}
 // void SetStartTime(double);

ReserveHandler::ReserveHandler(RMac* p):mac_(p){}
 
void ReserveHandler::handle(Event*e)
{
  mac_->TxRev(e);
}

ACKDATAHandler::ACKDATAHandler(RMac* p):mac_(p){}
 
void ACKDATAHandler::handle(Event*e)
{
  mac_->TxACKData(e);
}



TransmissionHandler::TransmissionHandler(RMac* p):mac_(p), receiver(0){}
 
void TransmissionHandler::handle(Event*e)
{
  
  mac_->TxData(receiver);
}




void NDBackoffHandler::clear(){
counter_=0;
}


NDStatusHandler::NDStatusHandler(RMac* p):mac_(p),status_(SLEEP){}
void NDStatusHandler::handle(Event* e)
{
  if(status_!=SLEEP) mac_->StatusProcess(e,status_);
  else mac_->Poweroff();
}


void NDStatusHandler::SetStatus(TransmissionStatus  status)
{
  status_=status;
}




ShortNDHandler::ShortNDHandler(RMac* p):mac_(p){}

void ShortNDHandler::handle(Event* e)
{  
  
    mac_->cycle_start_time=NOW;
    mac_->SendND(mac_->short_packet_size_);
}



ShortAckNDWindowHandler::ShortAckNDWindowHandler(RMac* p):mac_(p){}

void ShortAckNDWindowHandler::handle(Event* e)
{ 
   mac_->SendShortAckND();
}


ACKNDHandler::ACKNDHandler(RMac* p):mac_(p){}

void ACKNDHandler::handle(Event* e)
{ 
    mac_->TxND((Packet*) e, mac_->ACKND_window_);
}




PhaseOneHandler::PhaseOneHandler(RMac* p):mac_(p){}

void PhaseOneHandler::handle(Event* e)
{ 
    mac_->InitPhaseOne(mac_->ND_window_,mac_->ACKND_window_, mac_->PhaseOne_window_);
}



PhaseTwoHandler::PhaseTwoHandler(RMac* p):mac_(p){}

void PhaseTwoHandler::handle(Event* e)
{ 
    mac_->SendSYN();
}

PhaseThreeHandler::PhaseThreeHandler(RMac* p):mac_(p){}

void PhaseThreeHandler::handle(Event* e)
{ 
    mac_->InitPhaseThree();
}

/* ======================================================================
    RMAC for  underwater sensor
   ====================================================================== */
static class RMacClass : public TclClass {
public:
 RMacClass():TclClass("Mac/UnderwaterMac/RMac") {}
   TclObject* create(int, const char*const*) {
	  return (new RMac());
   }
} class_rmac;


RMac::RMac() :UnderwaterMac(),backoff_handler(this),short_nd_handler(this),short_acknd_window_handler(this), status_handler(this), phaseone_handler(this),acknd_handler(this), phasetwo_handler(this), phasethree_handler(this), sleep_handler(this), wakeup_handler(this), reserve_handler(this), ackrev_handler(this),mac_recv_handler(this),timeout_handler(this),transmission_handler(this),ackdata_handler(this), clear_channel_handler(this)
{
  num_send=0;
  num_data=0;
  large_packet_size_=30;
  short_packet_size_=10;
 
  short_latency_table_index=0;

  reserved_time_table_index=0;
  reservation_table_index=0;

  period_table_index=0;

  next_period=0;  
  ack_rev_pt=NULL;
  recv_busy=false;

  //  SIF_=0;

  //  next_hop=0;
  // setHopStatus=0;

  
 for(int i=0;i<TABLE_SIZE;i++){
  

    short_latency_table[i].node_addr=-1;
    short_latency_table[i].num=0;
    short_latency_table[i].last_update_time=0.0;

    period_table[i].node_addr=-1;
    period_table[i].difference=0.0;
    period_table[i].last_update_time=0.0; 
  }

    arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
    arrival_table[i].node_addr=-1;



 
  bind("PhaseOne_window_",&PhaseOne_window_);
  bind("PhaseTwo_window_",&PhaseTwo_window_);
  bind("duration_",&duration_);
  bind("ND_window_",&ND_window_); 
  bind("ACKND_window_",&ACKND_window_); 
  bind("PhyOverhead_",&PhyOverhead_);
  bind("large_packet_size_",&large_packet_size_);
  bind("short_packet_size_",&short_packet_size_);
  bind("PhaseOne_cycle_",&PhaseOne_cycle_);
  bind("IntervalPhase2Phase3_",&IntervalPhase2Phase3_);
  bind("PeriodInterval_",&PeriodInterval_);
  bind("transmission_time_error_",&transmission_time_error_);
  bind("SIF_",&SIF_);
  bind("ACKRevInterval_",&ACKRevInterval_);

  max_short_packet_transmissiontime=((1.0*short_packet_size_)/bit_rate_)*(1+transmission_time_error_);
   max_large_packet_transmissiontime=((1.0*large_packet_size_)/bit_rate_)*(1+transmission_time_error_);
  //  printf("size of table is %d, packet size is %d\n", sizeof(arrival_table),packet_size_); 
  //interval_ND_ACKND=Latency_window_-ND_window_-ACKND_window_;

  InitPhaseOne(ND_window_,ACKND_window_, PhaseOne_window_);
  
}

void 
RMac::InitPhaseOne(double t1,double t2, double t3)
{
  
   printf("RMac: Phaseone cycle: %d...\n",PhaseOne_cycle_);

   if(PhaseOne_cycle_)
    { 
      PhaseStatus=PHASEONE;
    Scheduler& s=Scheduler::instance();
    InitND(t1,t2,t3);
    s.schedule(&phaseone_handler,&phaseone_event,t3);
    PhaseOne_cycle_--;
    return;
    }
  
   // PrintTable();
   InitPhaseTwo();
   return;
}




void 
RMac::TxACKRev(Packet* pkt){

  printf("rmac Txackrev: node %d at time %f\n",index_,NOW);     
  DeleteBufferCell(pkt);
   
  hdr_cmn* cmh=HDR_CMN(pkt);
  hdr_rmac* ackh = HDR_RMAC(pkt); 

  printf("rmacTxackrev: node %d is transmitting a packet st=%f at time %f\n",index_,ackh->st,NOW);     
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();
  printf("rmacTxackrev: node %d is transmitting a packet st=%f tx=%f at time %f\n",index_,ackh->st,txtime,NOW);   

  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND); 
  cmh->ts_=NOW;

  sendDown(pkt);

  status_handler.SetStatus(SLEEP);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(IDLE==n->TransmissionStatus()){
  
    n->SetTransmissionStatus(SEND);
     cmh->ts_=NOW;
     sendDown(pkt);
  //  printf("broadcast %d Tx Idle set timer at %f tx is %f\n",node_->nodeid(),NOW,txtime);

  status_handler.SetStatus(IDLE);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(RECV==n->TransmissionStatus())
    {
      
      InterruptRecv(txtime);
      cmh->ts_=NOW;
      sendDown(pkt);

      status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);
     return;
    }


if (SEND==n->TransmissionStatus())
  {
    printf("rmac: queue send data too fas\n");
    Packet::free(pkt);
      return;
  }
}



void 
RMac::DeleteBufferCell(Packet* p)
{
 printf("Rmac: node %d ack_rev link\n",index_);
  buffer_cell* t1;
  buffer_cell* t2;
  t1=ack_rev_pt;
 
  if(!t1){
  printf("Rmac: there is no ack_rev link\n");
  return;
  }

 
  if(t1->next) t2=t1->next; 


  if(t1->packet==p) {
           ack_rev_pt=ack_rev_pt->next;
           delete t1;
           return;
  }

  /*
  if(t2){

    Packet* t=t2->packet;
    hdr_ack_rev*  th=HDR_ACK_REV(t);
   printf("Rmac:node %d  !!!!sender_addr=%d\n", index_, th->sender_addr); 
  }
  */

  while(t2){
    if(p==t2->packet) {

      t1->next=t2->next;
      delete t2;
      return;
    }
    t1=t2;
    t2=t2->next;
  }

  return;
}




void 
RMac::InitPhaseThree(){

 
  printf("RMac: this is InitPhaseThree\n"); 

   PrintTable();

  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;
  if(n->TransmissionStatus()==SLEEP) Poweron();
  
  mac_status=RMAC_IDLE;
  Scheduler &s=Scheduler::instance();
   s.schedule(&sleep_handler,&sleep_event,duration_);
    return;
}


void 
RMac::PrintTable(){

 
  printf("RMac: the short latency table in node%d...\n",index_); 

  for (int i=0;i<TABLE_SIZE;i++)
{
  printf("node addr is%d and short latency is%f\n",short_latency_table[i].node_addr,short_latency_table[i].latency);
}


printf("RMac: the period table in node%d...\n",index_); 

  for (int i=0;i<TABLE_SIZE;i++)
{
  printf("node addr is%d and difference is%f\n",period_table[i].node_addr,period_table[i].difference);
}

}

void 
RMac::ProcessSleep(){
  printf("RMac: node %d  is ProcessSleep at %f and wake up afte %f -%f\n", index_,NOW,PeriodInterval_,duration_); 

    
   Scheduler &s=Scheduler::instance();
   s.schedule(&wakeup_handler,&wakeup_event,(PeriodInterval_-duration_));

   if(mac_status==RMAC_RECV) return; 
     Poweroff();
  
     double largest_duration=0;
   if(reserved_time_table_index!=0) 
     {

   for (int i=0;i<reserved_time_table_index;i++){
    double nst=reserved_time_table[i].start_time-duration_;
    double lt=reserved_time_table[i].duration;
    double end_time=nst+lt;
    if(end_time>largest_duration) largest_duration=end_time;
    if (nst<=0) 
   { 
      if((lt+nst)<=0) {
        DeleteRecord(i);
	i--;    
      }
      else 
	{ // nst>=
       mac_status=RMAC_FORBIDDED;
     
	}
    }//nst<0
    else {
      // nst>0
            mac_status=RMAC_FORBIDDED;
      //  if (nst<=(PeriodInterval_-duration_)) mac_status=RMAC_FORBIDDED;
    }
   }
   
   if (mac_status==RMAC_FORBIDDED){
   Scheduler& s=Scheduler::instance();
   s.cancel(&clear_channel_event);
   //printf("processsleep!!! node %d is set clear time %f at %f\n",index_,largest_duration,NOW);
   s.schedule(&clear_channel_handler,&clear_channel_event, largest_duration);      
      ClearACKRevLink(); 
      CancelReservation();
     CancelREVtimeout();
     return;
   }
     }


   if((mac_status==RMAC_IDLE)&&(reservation_table_index!=0))
   {
printf("RMac: node %d process_sleep reservation table is not empty\n",index_); 
     mac_status=RMAC_ACKREV;
     ArrangeReservation();
 
    return;
   }
  
   return;
}




void 
RMac:: CancelREVtimeout(){ 
  printf("rmac:CancelREVtimeout node %d \n",index_);
   Scheduler& s=Scheduler::instance();
   s.cancel(&timeout_event);      
    return;
}



void 
RMac:: ClearChannel(){ 
  printf("rmac:ClearChannel node %d at %f\n",index_,NOW);
  if(NewData()){
    if(mac_status==RMAC_FORBIDDED){// avoid overlap
     MakeReservation();
     mac_status=RMAC_REV;
     }
  }
  else mac_status=RMAC_IDLE;
    return;
}






void 
RMac:: CancelReservation(){
   printf("rmac:cancelReservation: node %d\n",index_);
   //  if (reservation_table_index==0) return;
   // else {
   //  reservation_table_index=0;
    for (int i=0;i<TABLE_SIZE;i++){
      reservation_table[i].node_addr=-1;
       } 
    return;
    // }
}



void 
RMac::StartRECV(double dt, int id){
   printf("rmac:StartRECV: node %d at %f \n",index_,NOW);
  if(id==0) 
    {
   data_sender=-12;

   for (int i=0;i<MAXIMUM_BUFFER;i++) bit_map[i]=0;
   printf("rmac:StartRECV: node %d at %f to power on\n",index_,NOW);
   Poweron();
   recv_busy=false;

   mac_status=RMAC_RECV;
   mac_recv_handler.status=1;
  
   double t=2*max_large_packet_transmissiontime;
  
   Scheduler& s=Scheduler::instance();
   s.schedule(&timeout_handler, &timeout_event,dt);  
   s.schedule(&mac_recv_handler, &mac_recv_event,dt);  
    }
  else {

 printf("rmac:StartRECV: node %d at %f to power off\n",index_,NOW);
 ScheduleACKData();
     Poweroff();
  }
	 return ;
}


void 
RMac::ArrangeReservation(){
    printf("rmac:ArrangeReservation: node %d\n",index_);
       int sender_index=-1;
       sender_index=SelectReservation();
 //printf("Rmac:Arrangereservation: sender_index %d selected!!\n",sender_index); 
       if(sender_index==-1){
	 printf("Rmac:Arrangereservation: no reservation selected!!\n");
	 return;
       }
       else{
         int sender=reservation_table[sender_index].node_addr;
         double dt=reservation_table[sender_index].required_time;   
printf("Rmac:Arrangereservation: sender %d and dutation %f is scheduled\n",sender,dt);         
         ScheduleACKREV(sender,dt);
       }

}


void 
RMac::ScheduleACKREV(int receiver, double duration)
{
  printf("rmac:ScheduleACKREV: node %d\n",index_);
  int i=0;
  double Number_Period=0;
  double last_time=0.0;

    printf("rmac:scheduleACKRev: node %d is scheduling ackrev, duration=%f\n",index_,duration);
  while ((period_table[i].node_addr!=-1)&&(i<TABLE_SIZE))
      {
        if (period_table[i].node_addr!=receiver)
	  { 
	
      int receiver_addr=period_table[i].node_addr;

      double l=CheckLatency(short_latency_table, receiver_addr);
       
      double dt=CheckDifference(period_table,receiver_addr);
 
      double elapsed_time=NOW-cycle_start_time;
          


       if(dt<=0) dt=PeriodInterval_+dt-elapsed_time;
       else dt=dt-elapsed_time;


       if (dt>l-3*max_short_packet_transmissiontime) //??why 2 here
	  {

   double t1=dt-(l-3*max_short_packet_transmissiontime);
   Packet* ackrev=GenerateACKRev(receiver_addr,receiver, duration);
 
   t1=t1+Random::uniform()*(duration_-2*max_short_packet_transmissiontime);
 
printf("rmac:scheduleACKRev: node %d and node %d t1=%f the time is %f\n",index_, receiver_addr,t1,NOW);
       
       InsertACKRevLink(ackrev,t1);   

       Scheduler& s=Scheduler::instance();
       s.schedule(&ackrev_handler, (Event*) ackrev,t1);
 
       if (Number_Period<1) Number_Period=1;
       if(t1>last_time) last_time=t1;
	  }
        else {
       double t1=PeriodInterval_+dt-(l-2*max_short_packet_transmissiontime);
       Packet* ackrev=GenerateACKRev(receiver_addr,receiver,duration);
          
  
       t1=t1+Random::uniform()*(duration_-2*max_short_packet_transmissiontime);
       
printf("rmac:scheduleACKRev: node %d and node %d t1=%f the time is %f\n",index_, receiver_addr,t1,NOW);
         
        InsertACKRevLink(ackrev,t1);      
        Scheduler& s=Scheduler::instance();
        s.schedule(&ackrev_handler, (Event*) ackrev,t1);

        if (Number_Period<2) Number_Period=2;
	if(t1>last_time) last_time=t1;
	}
	  }
	i++;

    }// end of all the neighbors


     // now for the sender of ACKREV

      int receiver_addr=-1;
      double l=0;
      double dt=0;
      double elapsed_time=NOW-cycle_start_time;
      double t=0.0;
      double t1=0.0;

    for (int i=0;i<TABLE_SIZE;i++)  
    if (period_table[i].node_addr==receiver)
	  { 	  
      receiver_addr=period_table[i].node_addr;
      l=CheckLatency(short_latency_table, receiver_addr);
      dt=CheckDifference(period_table,receiver_addr);
      elapsed_time=NOW-cycle_start_time;
	  }


       if(dt<=0) dt=PeriodInterval_+dt-elapsed_time;
       else dt=dt-elapsed_time;

       if (dt>=(l-2*max_short_packet_transmissiontime)) 
	  {
    t1=dt-(l-2*max_short_packet_transmissiontime);
    t1=t1+Random::uniform()*(duration_-2*max_short_packet_transmissiontime); 
          }
	else{ 
        t1=PeriodInterval_+dt-(l-2*max_short_packet_transmissiontime);
        t1=t1+Random::uniform()*(duration_-2*max_short_packet_transmissiontime); 
	}

        t=last_time+ACKRevInterval_;   
      
        i=1;

       while (t1<t) {t1+=PeriodInterval_;i++;}
 printf("rmac:scheduleACKRev: node %d and node %d t1=%f the time is %f\n",index_, receiver,t1,NOW);  
        Packet* ackrevpacket=GenerateACKRev(receiver,receiver,duration);
        InsertACKRevLink(ackrevpacket,t1);   

        Scheduler& s=Scheduler::instance();
        s.schedule(&ackrev_handler, (Event*) ackrevpacket,t1);  
   
        // decide the start time of reservation

        double st=0;
	double t3=t1+2*ACKRevInterval_;
        double t4=i*PeriodInterval_-elapsed_time+duration_;

        if(t4>t3)  st=t4;
        else  st=t3;
        
        SetStartTime(ack_rev_pt,st);
 printf("rmac:scheduleACKRev: node %d set mac_recv after %f at %f\n",index_,st,NOW);
        mac_recv_handler.status=0;

	// 2 times max_large_packet_transmissiontime is enough
        mac_recv_handler.duration=(duration+3*max_large_packet_transmissiontime);

        s.schedule(&mac_recv_handler, &mac_recv_event,st);  
}


Packet* 
RMac::GenerateACKRev(int receiver, int intended_receiver, double duration){

  printf("rmac:GenerateACKREV: node %d\n",index_);
      Packet* pkt =Packet::alloc();
      hdr_rmac* ackrevh = HDR_RMAC(pkt);
      hdr_cmn*  cmh = HDR_CMN(pkt);
 

       cmh->next_hop()=receiver;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_RMAC;
       cmh->size()=short_packet_size_;      

       ackrevh->ptype=P_ACKREV; 
      ackrevh->pk_num = num_send;
      ackrevh->receiver_addr=intended_receiver;
      ackrevh->duration=duration;
      ackrevh->sender_addr=index_;
      num_send++;
      return pkt;
}



void 
RMac::SetStartTime(buffer_cell* ack_rev_pt, double st){
 printf("rmac setstarttime: node %d \n",index_);
  buffer_cell* t1;
  t1=ack_rev_pt;
  while(t1){
    hdr_rmac*  ackrevh=HDR_RMAC(t1->packet);
    double d=t1->delay;
    ackrevh->st=st-d;
 printf("rmac setstarttime: node %d interval to recv =%f\n",index_,ackrevh->st);
    t1=t1->next;
  }
}
 


void 
RMac::InsertACKRevLink(Packet* p, double d){
   printf("rmac:InsertACKREVLink: node %d\n",index_);
  buffer_cell* t1=new buffer_cell;
  t1->packet=p;
  t1->delay=d;
  t1->next=NULL;  

  if(ack_rev_pt==NULL){
    ack_rev_pt=t1;  
    printf("node %d ackrev link is empty\n", index_);
    return;
  }
  else 
    {
      buffer_cell* t2=ack_rev_pt;
      ack_rev_pt=t1;
      t1->next=t2;
      printf("node %d ackrev link is not empty\n", index_);
    
  return;
    }
}



void 
RMac::ResetReservationTable(){
   printf("rmac:ReserReservation: node %d\n",index_);
  for(int i=0;i<TABLE_SIZE;i++){
    reservation_table[i].node_addr=-1;
    reservation_table[i].required_time=0.0;
  }
}


int 
RMac::SelectReservation(){
  printf("rmac:selectReservation: node %d\n",index_);
  int index=-1;
  double dt=-1.0;
  int i=0;

  while(!(reservation_table[i].node_addr==-1))
   {
     printf("rmac:select reservation: node %d, request id is%d \n",index_,reservation_table[i].node_addr);
    if (reservation_table[i].required_time>dt) index=i;
    i++;
   }
  return index;
}





void 
RMac::ResetMacStatus(){
  printf("node %d timeout at %f!\n\n",index_,NOW);
  if((mac_status==RMAC_WAIT_ACKREV)||(mac_status==RMAC_FORBIDDED)) {
    txbuffer.UnlockBuffer();
    //ResumeTxProcess();
    printf("ResetMacStatus: node %d unlock txbuffer\n",index_);
    
  }
  if(mac_status==RMAC_RECV){
  printf("ResetMacStatus: node %d don't receive the data packet at %f\n",index_,NOW);
  Poweroff();  
  }
 mac_status=RMAC_IDLE;
}


void 
RMac::Wakeup(){

  printf("\n. ..WakeUp node %d wake up at %f  and the number of packet is %d...\n\n",index_,NOW,txbuffer.num_of_packet);

 
  Poweron();
  cycle_start_time=NOW;
  
  for(int i=0;i<TABLE_SIZE;i++){
    reservation_table[i].node_addr=-1;
  }


  reservation_table_index=0;

 
  ProcessReservedTimeTable();
 
  
  switch (mac_status){
  case RMAC_IDLE: 
              
                 if (NewData()){ 
                  printf("WakeUp: There is new data in node %d and the number of packet is %d\n", index_,txbuffer.num_of_packet);
	          mac_status=RMAC_REV;
                 MakeReservation();
                        }
                 break;
  case RMAC_FORBIDDED:
   printf("WakeUp NODE %d is in state RMAC_FORBIDDED\n",index_); 
   ClearACKRevLink();
    break;
  case RMAC_WAIT_ACKREV:  
    printf("WakeUp NODE %d is in state RMAC_WAIT_ACKREV\n",index_);
    break;
  case RMAC_RECV:  
    printf("WakeUp NODE %d is in state RMAC_RECV\n",index_);
    break;
 case RMAC_TRANSMISSION:  
    printf("WakeUp NODE %d is in state RMAC_TRANSMISSION\n",index_);
    break;
 case RMAC_REV:  
    printf("WakeUp NODE %d is in state RMAC_REV\n",index_);
    break;
  case RMAC_ACKREV:  
    printf("WakeUp NODE %d is in state RMAC_ACKREV\n",index_);
    break;
   case RMAC_WAIT_ACKDATA:  
    printf("WakeUp NODE %d is in state RMAC_WAIT_ACKDATA\n",index_);
    break;
  default:  printf("WakeUp node %d don't expect to be in this state\n",index_);
    break;
  }

printf("\n. ..WakeUp node %d schedule sleep after %f at%f...\n\n",index_,duration_,NOW);
  Scheduler &s=Scheduler::instance();
  s.schedule(&sleep_handler,&sleep_event,duration_);
   return;
}


void 
RMac::ClearACKRevLink(){
 printf("rmac clearACKREV: node %d\n",index_);
  if(!ack_rev_pt) return;
  buffer_cell* t1;
  buffer_cell* t2;
  Event * e1;
   Scheduler &s=Scheduler::instance();
 


  // t1=ack_rev_pt->next;
  t1=ack_rev_pt;
  while (t1){
    t2=t1->next;
    e1=(Event*)t1->packet;
    s.cancel(e1);
    Packet::free(t1->packet);
    delete t1;
    t1=t2;
    ack_rev_pt=t1;
  }
   
}




void 
RMac::ProcessReservedTimeTable(){
 printf("rmac:ProcessReservedtimetable: node %d index=%d\n",index_, reserved_time_table_index);
  int i=0;
   double largest_duration=0;

  while(i<reserved_time_table_index){
   printf("rmac:ProcessReservedtimetable: node %d index=%d\n",index_, reserved_time_table_index);
    double nst=reserved_time_table[i].start_time-PeriodInterval_;
    double lt=reserved_time_table[i].duration;
    int addr=reserved_time_table[i].node_addr;
    double  l=CheckLatency(short_latency_table,addr);
    double t1=l-2*max_short_packet_transmissiontime;
    nst=nst-t1;
   
    double end_time=nst+lt;

    if(end_time>largest_duration)largest_duration=end_time;

    if (nst<0) {
      if((lt+nst)<=0) {
        DeleteRecord(i);
	i--;    
      }
      else 
	{ // nst>=
       mac_status=RMAC_FORBIDDED;
 printf("RMac:ProcessReservedTimeTable: node %d sets reserved time, inetrval=%f and duration is%f\n",index_,0,(lt+nst));
       reserved_time_table[i].start_time=0;
       reserved_time_table[i].duration=lt+nst;
	}
    }//nst<0
    else {
      // nst>0
      if (nst<=PeriodInterval_) mac_status=RMAC_FORBIDDED;
    printf("RMac:ProcessReservedTimeTable: node %d sets reserved time, inetrval=%f and duration is%f\n",index_,nst,lt);
      reserved_time_table[i].start_time=nst;
       reserved_time_table[i].duration=lt;
     
    }
    i++;
  }

  if(mac_status==RMAC_FORBIDDED){
   Scheduler& s=Scheduler::instance();
   s.cancel(&clear_channel_event);
  
   s.schedule(&clear_channel_handler,&clear_channel_event,largest_duration);   
  }


  if((reserved_time_table_index==0)&&(mac_status==RMAC_FORBIDDED)) 
          mac_status=RMAC_IDLE; 
}


void 
RMac::DeleteRecord(int index){
 
  for(int i=index;i<reserved_time_table_index;i++)
    {
      reserved_time_table[i].node_addr= reserved_time_table[i+1].node_addr;
      reserved_time_table[i].start_time= reserved_time_table[i+1].start_time;
      reserved_time_table[i].duration= reserved_time_table[i+1].duration;
      reserved_time_table_index--;
    }
 printf("rmac:deleteRecord: node %d the reserved index=%d\n",index_, reserved_time_table_index);
}




bool 
RMac::NewData(){
  return (!txbuffer.IsEmpty());//?think about it
}


void 
RMac::MakeReservation(){
 
printf("rmac MakeReservation: node %d MakeReservation \n",index_);
// mac_status=RMAC_WAIT_ACKREV;
  Packet* p=txbuffer.head();
  hdr_cmn*  cmh = HDR_CMN(p);
  int receiver_addr=cmh->next_hop();
  
   txbuffer.LockBuffer();
   int num=txbuffer.num_of_packet;

 printf("rmac MakeReservation: node %d lock txbuffer \n",index_);


  int sender_addr=index_;
  double dt=num*(((large_packet_size_*encoding_efficiency_+PhyOverhead_)/bit_rate_)+transmission_time_error_);

  // Generate a Rev Packet

      Packet* pkt =Packet::alloc();
      hdr_rmac* revh = HDR_RMAC(pkt);
      cmh = HDR_CMN(pkt);
 

       cmh->next_hop()=receiver_addr;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_RMAC;
      
       revh->ptype=P_REV;
       revh->pk_num = num_send;
       revh->duration=dt;
       revh->sender_addr=index_;
       num_send++;
 
 
  // printf("rmac:Makereservation node %d num=%d size=%d, encoding_efficiency=%f phy=%d and bit_rate= %f\n",index_,num, large_packet_size_,encoding_efficiency_,PhyOverhead_, bit_rate_);  
       
      
        
    
       //      printf("rmac:Makereservation node %d send a reservation to node %d, duration is %f and type is %d\n",index_, receiver_addr,revh->duration, cmh->ptype_);  
   
      double l=CheckLatency(short_latency_table, receiver_addr);
          dt=CheckDifference(period_table,receiver_addr);
	  double elapsed_time=NOW-cycle_start_time;

 printf("rmac:Makereservation node %d latency=%f and dt=%f now is %f\n",index_,l,dt,NOW);    

       if(dt<=0) dt=PeriodInterval_+dt;
       dt=dt-elapsed_time;
double t2=Random::uniform()*(duration_-2*max_short_packet_transmissiontime);     

      if (dt>l-3*max_short_packet_transmissiontime)
	  {
       double t1=dt-(l-3*max_short_packet_transmissiontime);

     printf("rmac:Makereservation node %d will send a reservation after %f now time is %f\n",index_,(t1+t2),NOW);  
       Scheduler& s=Scheduler::instance();
       s.schedule(&reserve_handler, (Event*) pkt,(t1+t2));
	  }
        else {
        double t1=PeriodInterval_+dt-(l-3*max_short_packet_transmissiontime);
    printf("rmac:Makereservation node %d will send a reservation after %f, now time is %f\n",index_,(t1+t2),NOW);  
       Scheduler& s=Scheduler::instance();
       s.schedule(&reserve_handler, (Event*) pkt,(t1+t2));
	}

}

double 
RMac::CheckLatency(latency_record* table,int addr)
{
  int i=0;
double d=0.0;
 
 while((table[i].node_addr!=addr)&&(i<TABLE_SIZE))
{
  //printf("node addr is%d and latency is%f\n",table[i].node_addr,table[i].latency);
 i++;
}
 if (i==TABLE_SIZE) return d;
 else return table[i].latency;
}




double 
RMac:: CheckDifference(period_record* table,int addr)
{
  int i=0;
double d=-0.0;
 
 while((table[i].node_addr!=addr)&&(i<TABLE_SIZE))i++;

 if (i==TABLE_SIZE) return d;
 else return table[i].difference;
}


void 
RMac::TxRev(Event* e){
  printf("RMac TxREv node %d at %f\n",index_,NOW);
  Packet* pkt=(Packet*) e;
  //   printf("RMac TxREv node %d at %f\n",index_,NOW);
  hdr_cmn* cmh=HDR_CMN(pkt);
  //  hdr_syn* synh = HDR_SYN(pkt); 
  

  if(mac_status==RMAC_FORBIDDED) {
 printf("TxREV, node %d is in  RMAC_FORBIDDED, cancel sending REV at %f\n",index_,NOW);
 Packet::free(pkt);
 return;
  }


 

  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();


   mac_status=RMAC_WAIT_ACKREV;

  printf("TxREV, node %d is in  RMAC_WAIT_ACKREV at %f\n",index_,NOW);

   double t=3*PeriodInterval_;

    Scheduler& s=Scheduler::instance();
    //printf("TxREV, node %d is in before  at %f\n",index_,NOW);
   s.schedule(&timeout_handler,&timeout_event,t);

  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND); 
  cmh->ts_=NOW;

  sendDown(pkt);
  printf("TxREV, node %d is in sleep at %f\n",index_,NOW);
  status_handler.SetStatus(SLEEP);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(IDLE==n->TransmissionStatus()){
  
    n->SetTransmissionStatus(SEND);
 
     cmh->ts_=NOW;

     sendDown(pkt);
     // printf("broadcast %d Tx Idle set timer at %f tx is %f\n",node_->nodeid(),NOW,txtime);

     printf("TxREV, node %d is in idle at %f\n",index_,NOW);
  status_handler.SetStatus(IDLE);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(RECV==n->TransmissionStatus())
    {
      
      InterruptRecv(txtime);
      cmh->ts_=NOW;
      sendDown(pkt);
      printf("TxREV, node %d is in recv at %f\n",index_,NOW);
      status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);
     return;
    }


if (SEND==n->TransmissionStatus())
  {
    printf("rmac: queue send data too fast\n");
    Packet::free(pkt);
      return;
  }

}


void 
RMac::InitPhaseTwo(){



   double delay=Random::uniform()*PhaseTwo_window_;
   PhaseStatus=PHASETWO;

    cycle_start_time=NOW;
    next_period=IntervalPhase2Phase3_+PhaseTwo_window_+delay;
 
    printf("rmac initphasetwo: the phasethree of node %d is scheduled at %f\n",index_,NOW+next_period);
    Scheduler& s=Scheduler::instance();
    s.schedule(&phasetwo_handler, &phasetwo_event,delay);
    return;
}

void 
RMac::SendSYN(){

       Packet* pkt =Packet::alloc();
       hdr_rmac* synh = HDR_RMAC(pkt); 
       hdr_cmn*  cmh = HDR_CMN(pkt);
      
       cmh->size()=short_packet_size_;
       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_RMAC;
      
      
       synh->ptype=P_SYN;
       synh->pk_num = num_send;
       synh->sender_addr= node_->address();
     
       synh->duration=duration_;
        num_send++;

	printf("rmac SendSYN:node(%d) send SYN packet at %f\n", synh->sender_addr,NOW);
      TxND(pkt, PhaseTwo_window_);  
}


void 
RMac::InitND(double t1,double t2, double t3)
{  
 
  double delay=Random::uniform()*t1;
  double itval=(t3-t2-t1)/2.0;
  double delay3=t1+itval;
 
  Scheduler& s=Scheduler::instance();

   s.schedule(&short_nd_handler, &short_nd_event, delay);
   s.schedule(&short_acknd_window_handler,&short_acknd_event,delay3);
  return;
}

void 
RMac::SendND(int pkt_size)
{

      Packet* pkt =Packet:: alloc();
      hdr_rmac* ndh = HDR_RMAC(pkt);
    
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
   
     // additional 2*8 denotes the size of type,next-hop of the packet and 
     // timestamp
  
      //       cmh->size()=sizeof(hdr_nd)+3*8;
      //  printf("old size is %d\n",cmh->size());
        cmh->size()=pkt_size;

       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_RMAC;
      
       
     
      ndh->ptype=P_ND;
      ndh->pk_num = num_send;
      ndh->sender_addr= node_->address();

      num_send++;

      // iph->src_.addr_=node_->address();
      // iph->dst_.addr_=node_->address();
      //iph->dst_.port_=255;     

 printf("rmac SendND:node(%d) send ND type is %d at %f\n", ndh->sender_addr,cmh->ptype_, NOW);
      TxND(pkt, ND_window_);  
}





void 
RMac::SendShortAckND()
{
    printf("rmac:SendShortND: node %d\n",index_);
  if (arrival_table_index==0) return;// not ND received


  while(arrival_table_index>0){ 
      Packet* pkt = Packet::alloc();
      hdr_rmac* ackndh = HDR_RMAC(pkt);
    
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
      ackndh->ptype=P_SACKND;
      ackndh->pk_num = num_send;
      ackndh->sender_addr=node_->address();
      num_send++;

      cmh->ptype_=PT_RMAC;
        
         int index1=-1;
        index1=rand()%arrival_table_index; 
        double t2=-0.1;
        double t1=-0.1;
    
        int receiver=arrival_table[index1].node_addr;
         t2=arrival_table[index1].arrival_time; 
         t1=arrival_table[index1].sending_time; 

	 for(int i=index1;i<arrival_table_index;i++){
	   arrival_table[i].node_addr=arrival_table[i+1].node_addr;
           arrival_table[i].sending_time=arrival_table[i+1].sending_time;
           arrival_table[i].arrival_time=arrival_table[i+1].arrival_time;
	 }
   
          ackndh->arrival_time=t2;
          ackndh->ts=t1;
  // additional 2*8 denotes the size of type,next-hop of the packet and 
  // timestamp
    //  cmh->size()=sizeof(hdr_ack_nd)+3*8;
    

      cmh->size()=short_packet_size_;
      cmh->next_hop()=receiver;
      cmh->direction()=hdr_cmn::DOWN; 
      cmh->addr_type()=NS_AF_ILINK;

      
         Scheduler& s=Scheduler::instance();
         double delay=Random::uniform()*ACKND_window_;
         s.schedule(&acknd_handler, (Event*) pkt, delay);

	 arrival_table_index--;
  }


     arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
    arrival_table[i].node_addr=-1;

          return; 
}


void 
RMac::TxND(Packet* pkt, double window)
{
  //  printf("RMac TxND node %d\n",index_); 
  hdr_cmn* cmh=HDR_CMN(pkt);
   hdr_rmac* synh = HDR_RMAC(pkt); 
  
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();

  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND);
  cmh->ts_=NOW;

  if(PhaseStatus==PHASETWO){

    double t=NOW-cycle_start_time;

    synh->interval=next_period-t;
 
    
    Scheduler & s=Scheduler::instance();
    s.schedule(&phasethree_handler,&phasethree_event,synh->interval);
  }

  sendDown(pkt);
  backoff_handler.clear();

  status_handler.SetStatus(IDLE);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(IDLE==n->TransmissionStatus()){
  
  n->SetTransmissionStatus(SEND);
 
  //printf("TxND the data type is %d\n",MAC_BROADCAST);
  //printf("broadcast : I am going to send the packet down tx is %f\n",txtime);
     cmh->ts_=NOW;

  if(PhaseStatus==PHASETWO){

   double t=NOW-cycle_start_time;
   synh->interval=next_period-t; 

  
    Scheduler & s=Scheduler::instance();
    s.schedule(&phasethree_handler,&phasethree_event,synh->interval);
  }

  sendDown(pkt);
  backoff_handler.clear();
  //  printf("broadcast %d Tx Idle set timer at %f tx is %f\n",node_->nodeid(),NOW,txtime);

   status_handler.SetStatus(IDLE);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(RECV==n->TransmissionStatus())
    {
      Scheduler& s=Scheduler::instance();
      double d1=window-(NOW-cycle_start_time);
 
      if(d1>0){
      double backoff=Random::uniform()*d1;
      backoff_handler.window_=window;
   // printf("broadcast Tx set timer at %f backoff is %f\n",NOW,backoff);
      s.schedule(&backoff_handler,(Event*) pkt,backoff);
      return;
      }
      else {
          backoff_handler.clear();
          printf("Rmac:backoff:no time left \n");
          Packet::free(pkt);
      }

    }

if (SEND==n->TransmissionStatus())
{
  // this case is supposed not to  happen 
    printf("rmac: queue send data too fas\n");
    Packet::free(pkt);
      return;
}

}



void 
RMac::ProcessACKRevPacket(Packet* pkt)
{
  printf("RMac:ProcessACKRevPacket: node %d at time %f\n",index_,NOW);

    hdr_rmac* ackrevh=HDR_RMAC(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int receiver_addr=ackrevh->receiver_addr;
    double dt=ackrevh->duration;
    double st=ackrevh->st;
    int sender_addr=ackrevh->sender_addr;
 
   
    double  l=CheckLatency(short_latency_table,sender_addr);
    double  it=st-l;
    double elapsedtime=NOW-cycle_start_time;
  printf("rmac:ProcessAckRevPacket:node %d I get the ACK REV packet interval is %f and duration=%f present time is %f\n",index_, it,dt,NOW);     
    //printf("rmac:ProcessAckRevPacket: node %d I get the ACK REV packet interval is %f \n",index_, it);

    if(it<0) 
     printf("rmac:ProcessAckRevPacket: the notification is too short\n");
    
    Packet::free(pkt);

    if(receiver_addr!=index_) {
     
printf("rmac:ProcessAckRevPacket: node %d this ACKREV is not for me\n",index_);
      if(reserved_time_table_index>=TABLE_SIZE){
	printf("rmac:ProcessAckRev: the reserved_time_table is full\n");
        return;
      }
      reserved_time_table[reserved_time_table_index].node_addr=sender_addr;
      reserved_time_table[reserved_time_table_index].start_time=elapsedtime+it;
      reserved_time_table[reserved_time_table_index].duration=dt;
      reserved_time_table_index++;
    }
    else {
      if(mac_status!=RMAC_WAIT_ACKREV) {
     printf("rmac:ProcessAckRevPacket:status change, I quit this chance\n");  
     return;
      }
printf("rmac:ProcessAckRevPacket: node %d this ACKREV is for me\n",index_); 
         
        num_data=0;
        double  l2=CheckLatency(short_latency_table,sender_addr);
        double  it1=it-l2+2*max_short_packet_transmissiontime;
      mac_status=RMAC_TRANSMISSION;
      Scheduler& s=Scheduler::instance();
printf("rmac:ProcessAckRevPacket: node %d schedule Txdata after %f at time %f, latency is %f\n",index_,it1,NOW,l2);    
      s.cancel(&timeout_event);// cancel the timer of rev

      transmission_handler.receiver=sender_addr;      
      s.schedule(&transmission_handler,&transmission_event,it1);
    }
  
    return;
}


void 
RMac::ClearTxBuffer(){
 printf("RMac: ClearTxBuffer: node %d clear its buffer\n",index_);

  Packet*  p1[MAXIMUM_BUFFER];
  
  for (int i=0;i<MAXIMUM_BUFFER;i++)p1[i]=NULL;
  buffer_cell* bp=txbuffer.head_;
  int i=0;
  while(bp){
    p1[i]=bp->packet;
    bp=bp->next;
    i++;
  }

 

  for (int i=0;i<MAXIMUM_BUFFER;i++){
    //   printf("ClearTxBuffer the poniter is%d\n",p1[i]);
    if (bit_map[i]==1) txbuffer.DeletePacket(p1[i]);
  

  }
 
  /*
  printf("ClearTxBuffer: show the queue****************after txbufferdelete\n");
      t->showqueue();  
  */

  printf("txbuffer is cleared, there are %d packets left\n",txbuffer.num_of_packet);

  return; 
}



void 
RMac::ProcessACKDataPacket(Packet* pkt)
{
printf("rmac:ProcessACKDATAPacket: node %d process ACKDATA packets at time %f duration_=%f\n",index_,NOW,duration_);
    hdr_rmac* ackrevh=HDR_RMAC(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);
 
    Scheduler& s=Scheduler::instance();
printf("rmac:ProcessAckData: node %d cancel timeout dutation=%f\n",index_,duration_);    
      s.cancel(&timeout_event);// cancel the timer of data
  

  
    
    for (int i=0;i<MAXIMUM_BUFFER;i++)bit_map[i]=0;

    memcpy(bit_map, pkt->accessdata(),sizeof(bit_map));
  
  printf("rmac:ProcessACKDATAPacket: node %d received the butmap is:\n",index_);
  

  for (int i=0;i<MAXIMUM_BUFFER;i++) printf("bmap[%d]=%d ",i,bit_map[i]);
  printf("\n");

  printf("txbuffer will be cleared, there are %d packets in queue and duration=%f\n",txbuffer.num_of_packet,duration_);

      Packet::free(pkt);   


 /*
!!!!
This part should consider the retransmission state, in this implementation, we don't consider the packets loss, therefore, we just ignore it, it should be added later. 

  */

    ClearTxBuffer(); 

    txbuffer.UnlockBuffer();
    mac_status=RMAC_IDLE;
printf("rmac:ProcessACKDATAPacket: node %d unlock txbuffer duration_=%f\n",index_,duration_);
   ResumeTxProcess();
 
    return;
}


void 
RMac::ProcessRevPacket(Packet* pkt)
{
 printf("RMac:ProcessRevPacket: node %d is processing rev\n",index_);
    hdr_rmac* revh=HDR_RMAC(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int sender_addr=revh->sender_addr;
    double dt=revh->duration;

    Packet::free(pkt);
   
   

    if(mac_status==RMAC_IDLE)
    {     
      //       mac_status=RMAC_ACKREV;
    if(reservation_table_index <TABLE_SIZE){  
    reservation_table[reservation_table_index].node_addr=sender_addr;
    reservation_table[reservation_table_index].required_time=dt;           
    reservation_table_index++;
    }
    else {
    printf("ramc:ProcessRevPacket: too many reservation, drop the packet\n");
    }
    }
    else {
   printf("rmac:ProcessRevPacket: I am not in idle state, drop this packet\n"); 
    }
      return;
}




void 
RMac::ProcessNDPacket(Packet* pkt)
{
  // printf("rmac:ProcessNDPacket: node %d\n",index_);
    hdr_rmac* ndh=HDR_RMAC(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=ndh->sender_addr;
    double time=NOW;
    if(arrival_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      Packet::free(pkt);
      return;
    }
    arrival_table[arrival_table_index].node_addr=sender;
    arrival_table[arrival_table_index].arrival_time=time;
    arrival_table[arrival_table_index].sending_time=cmh->ts_;
    arrival_table_index++;
    Packet::free(pkt);
      return;
}

void 
RMac::ProcessDataPacket(Packet* pkt)
{

 printf("rmac:ProcessDataPacket: node %d get data packet\n",index_);

  hdr_uwvb* vbh=HDR_UWVB(pkt);
  hdr_rmac* rmach=HDR_RMAC(pkt);

  data_sender=rmach->sender_addr;
  int num=rmach->data_num;

 
  recv_busy=true;
      Scheduler& s=Scheduler::instance();
      s.cancel(&timeout_event);

    MarkBitMap(num);
    uptarget_->recv(pkt,this);
      return;
}

void 
RMac::MarkBitMap(int num){
  if(num<MAXIMUM_BUFFER) bit_map[num]=1;
}


void 
RMac::ScheduleACKData()
{
  printf("rmac Schdeule ACKDATA: node %d at %f\n",index_,NOW);
  
  if(data_sender<0) {
    printf("Ramc:ScheduleACKData: invalid sender address\n");
    return; 
  }

  Packet* pkt=Packet::alloc(sizeof(bit_map)); 
  hdr_cmn*  cmh = HDR_CMN(pkt);
   hdr_rmac* revh = HDR_RMAC(pkt);
 
       memcpy(pkt->accessdata(),bit_map,sizeof(bit_map));

       printf("rmac Schdeule ACKDATA: node %d return bitmap is \n",index_);
  for(int i=0;i<MAXIMUM_BUFFER;i++) printf("bit_map[%d]=%d ",i,bit_map[i]);

       printf("\n");         


 
       cmh->next_hop()=data_sender;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_RMAC;
       cmh->size()=short_packet_size_;     
     
       revh->ptype=P_ACKDATA;
       revh->pk_num = num_send;
       revh->sender_addr=index_;
       num_send++;
 
 
   
         double l=CheckLatency(short_latency_table, data_sender);
         double elapsed_time=NOW-cycle_start_time;         
         double dt=CheckDifference(period_table,data_sender)-elapsed_time;

	 while(dt<=0) dt=PeriodInterval_+dt;
         double t1=0.0;       
 
      if (dt>l-2*max_short_packet_transmissiontime)
        t1=dt-(l-2*max_short_packet_transmissiontime);
        else 
        t1=PeriodInterval_+dt-(l-2*max_short_packet_transmissiontime);
  
    printf("rmac Schdeule ACKDATA: node %d  schedule ackdata after %f at %f\n",index_,t1,NOW);
       Scheduler& s=Scheduler::instance();
       s.schedule(&ackdata_handler, (Event*) pkt,t1);
}


void  
RMac::TxACKData(Event* e){
 printf("RMac TxACKData node %d at %f\n",index_,NOW);

  Packet* pkt=(Packet*) e;
  hdr_cmn* cmh=HDR_CMN(pkt);
 
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();


  mac_status=RMAC_IDLE;

  printf("TxACKData, node %d is in  RMAC_IDLE at %f\n",index_,NOW);

 
  if(SLEEP==n->TransmissionStatus()) {
  Poweron();
  n->SetTransmissionStatus(SEND); 
  cmh->ts_=NOW;

  sendDown(pkt);

  printf("RMac TxACKData node %d at %f\n",index_,NOW);
  status_handler.SetStatus(SLEEP);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(IDLE==n->TransmissionStatus()){
  
    n->SetTransmissionStatus(SEND);
 
     cmh->ts_=NOW;

     sendDown(pkt);
  printf("RMac TxACKData node %d at %f\n",index_,NOW);
  status_handler.SetStatus(IDLE);
  Scheduler& s=Scheduler::instance();
  s.schedule(&status_handler,&status_event,txtime);
  return;
  }

  if(RECV==n->TransmissionStatus())
    {
      
      InterruptRecv(txtime);
      cmh->ts_=NOW;
      sendDown(pkt);
printf("RMac TxACKData node %d at %f\n",index_,NOW);
      status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);
     return;
    }


if (SEND==n->TransmissionStatus())
  {
    printf("rmac: node%d send data too fast\n",index_);
    Packet::free(pkt);
      return;
  }

}






void 
RMac::ProcessShortACKNDPacket(Packet* pkt)
{
  // printf("rmac:ProcessshortACKNDPacket: node %d\n",index_);
    hdr_rmac* ackndh=HDR_RMAC(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=ackndh->sender_addr;
    double t4=NOW;
    double t3=cmh->ts_;
    int myaddr=node_->address();
 
    double t2=ackndh->arrival_time;
    double t1=ackndh->ts;

double latency=((t4-t1)-(t3-t2))/2.0;
bool newone=true;   

Packet::free(pkt);
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (short_latency_table[i].node_addr==sender)
      {
       short_latency_table[i].sumLatency+=latency;
       short_latency_table[i].num++;
       short_latency_table[i].last_update_time=NOW;
       short_latency_table[i].latency = 
                  short_latency_table[i].sumLatency/short_latency_table[i].num;
       newone=false;
      }
 
 if(newone)
{

    if(short_latency_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }

    short_latency_table[short_latency_table_index].node_addr=sender;
    short_latency_table[short_latency_table_index].sumLatency+=latency;
    short_latency_table[short_latency_table_index].num++;
    short_latency_table[short_latency_table_index].last_update_time=NOW;
    short_latency_table[short_latency_table_index].latency = 
          short_latency_table[short_latency_table_index].sumLatency/short_latency_table[short_latency_table_index].num;
    short_latency_table_index++;
}
 for(int i=0;i<short_latency_table_index;i++)
   printf("node (%d) to node (%d) short latency is %f and number is %d\n", myaddr, short_latency_table[i].node_addr, short_latency_table[i].latency,short_latency_table[i].num); 
 
 return;

}


void 
RMac::ProcessSYN(Packet* pkt)
{
  // printf("rmac:ProcessSYN: node %d\n",index_);
    hdr_rmac* synh=HDR_RMAC(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int  sender=synh->sender_addr;
    double interval=synh->interval;
    double tduration=synh->duration;
      Packet::free(pkt);


    double t1=-1.0;
 for (int i=0;i<TABLE_SIZE;i++)
 if (short_latency_table[i].node_addr==sender)
     t1=short_latency_table[i].latency;

 if(t1==-1.0) {
   printf("Rmac:ProcessSYN: I receive a SYN from unknown neighbor\n");
   return; 
 }

 interval-=t1;
 double t2=next_period-(NOW-cycle_start_time);
 double d=interval-t2;

if (d>=0.0) {
   while (d>=PeriodInterval_) d-=PeriodInterval_;
 }
 else 
   {
     while (d+PeriodInterval_<=0.0) d+=PeriodInterval_;
   }



 bool newone=true;     
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (period_table[i].node_addr==sender)
      {
       period_table[i].difference=d;
       period_table[i].last_update_time=NOW;
       period_table[i].duration =tduration; 
       newone=false;
      }
 
 if(newone)
{

    if(period_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessSYN:period_table is full\n");
      return;
    }


    period_table[period_table_index].node_addr=sender;
    period_table[period_table_index].difference=d;
    period_table[period_table_index].last_update_time=NOW;
    period_table[period_table_index].duration=tduration;
    period_table_index++;
}

 for(int i=0;i<period_table_index;i++)
   printf("node (%d) to node (%d) period difference  is %f \n",index_,period_table[i].node_addr, period_table[i].difference); 
 
 return;

}


/*
 this program is used to handle the received packet, 
it should be virtual function, different class may have 
different versions.
*/


void 
RMac::
RecvProcess(Packet* pkt){
    printf("rmac:node %d  gets a broadcast packet at  %f\n",index_,NOW);
   hdr_cmn* cmh=HDR_CMN(pkt);
   hdr_rmac* cmh1=HDR_RMAC(pkt);
   int dst=cmh->next_hop();
   int ptype=cmh1->ptype; 
   
  if(dst==MAC_BROADCAST){
    printf("rmac:node %d  gets a broadcast packet at  %f and type is %d\n",index_,NOW, cmh->ptype_);
    if (ptype==P_ND) ProcessNDPacket(pkt); //this is ND packet

        // this is ACK_ND packet  
    if (ptype==P_SYN) ProcessSYN(pkt);
   
    // uptarget_->recv(pkt, this);
    return;
  }

   if(dst==index_){
 printf("rmac:node %d  gets a packet at  %f and type is %d and %d\n",index_,NOW, cmh->ptype_,cmh1->ptype);
   if (ptype==P_SACKND) ProcessShortACKNDPacket(pkt); 
   if (ptype==P_REV) ProcessRevPacket(pkt);
   if (ptype==P_ACKREV) ProcessACKRevPacket(pkt);
   if (ptype==P_DATA) ProcessDataPacket(pkt);
   if(ptype==P_ACKDATA) ProcessACKDataPacket(pkt);
     // printf("underwaterbroadcastmac:this is my packet \n");
     //  uptarget_->recv(pkt, this);
    return;
}
    printf("rmac: node%d this is neither broadcast nor my packet %d, just drop it at %f\n",index_,dst, NOW);
   Packet::free(pkt);
   return;
}


void 
RMac::TxData(int receiver)
{
  printf("RMac:node %d TxData at time %f\n",index_,NOW);

  if (txbuffer.IsEmpty()) 
{
printf("Rmac:TxData: what a hell! I don't have data to send\n");
return;
}


  if(mac_status!=RMAC_TRANSMISSION) {
 printf("Rmac:TxData: node %d is not in transmission state\n");
      return;
  }

   UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;
  if(n->TransmissionStatus()==SLEEP) Poweron();

     mac_status=RMAC_TRANSMISSION;

    Packet* pkt=txbuffer.next();
    /*      
    hdr_cmn* cmh=HDR_CMN(pkt);
    hdr_rmac* datah = HDR_RMAC(pkt);
    hdr_uwvb* hdr2=HDR_UWVB(pkt);
    */
    hdr_cmn* cmh=hdr_cmn::access(pkt);
    hdr_rmac* datah =hdr_rmac::access(pkt);
    hdr_uwvb* hdr2=hdr_uwvb::access(pkt);

    // printf("RMac:node %d TxData at time %f data type is %d offset is%d and size is %d and offset is %d and size is%d uwvb offset is %d and size is %d\n",index_,NOW,hdr2->mess_type,cmh,sizeof(hdr_cmn),datah,sizeof(hdr_rmac),hdr2,sizeof(hdr_uwvb));
         datah->ptype=P_DATA;  

         datah->sender_addr=index_;
   
         datah->pk_num=num_send;
         datah->data_num=num_data;
      num_send++;
      num_data++;
  
          cmh->size()=large_packet_size_;

            cmh->next_hop()=receiver;
	   
            cmh->direction()=hdr_cmn::DOWN; 
            cmh->addr_type()=NS_AF_ILINK;
            cmh->ptype_=PT_RMAC; 

	    

  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();

 printf("RMac:node %d TxData at time %f data type is %d packet data_num=%d class data_num=%d \n",index_,NOW,hdr2->mess_type,datah->data_num,num_data);
  TransmissionStatus status=n->TransmissionStatus();

 

 if(IDLE==status)
 {
  n->SetTransmissionStatus(SEND); 
        sendDown(pkt);
 printf("RMac:node %d TxData at %f\n ",index_,NOW);
        status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);  
 }

 if(RECV==status)
    {
      InterruptRecv(txtime);
      
      sendDown(pkt);
 printf("RMac:node %d TxData at %f\n ",index_,NOW);
      status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);
    }

 if (SEND==status)
    { 
    printf("rmac:Txdata: queue send data too fast\n");
    Packet::free(pkt);
    }

  
  if (txbuffer.IsEnd()) {
   printf("rmac:Txdata: node %d is in state MAC_WAIT_ACKDATA\n",index_);
   mac_status=RMAC_WAIT_ACKDATA; 
  
   double txtime=2*PeriodInterval_;  
 
   printf("RMac:node %d TxData at %f\n ",index_,NOW);
   Scheduler& s=Scheduler::instance();
   s.schedule(&timeout_handler,&timeout_event,txtime);
  
   Poweroff();
  }
  else {
  double it=SIF_+txtime;   
 
 printf("rmac:Txdata: node%d schedule  next data packet , interval=%f at time%f\n",index_,it,NOW);
  Scheduler& s=Scheduler::instance();
  s.schedule(&transmission_handler,&transmission_event,it);
  }

}


void 
RMac::ResumeTxProcess(){

  printf("rmac:ResumeTxProcess: node %d at %f\n",index_,NOW);

  if(!txbuffer.IsFull()) 
  if(callback_) callback_->handle(&status_event);
  return;
}



/*
 this program is used to handle the transmitted packet, 
it should be virtual function, different class may have 
different versions.
*/

void 
RMac::TxProcess(Packet* pkt){

  hdr_uwvb* hdr=HDR_UWVB(pkt);

  printf("rmac:TxProcess: node %d type is %d\n",index_,hdr->mess_type);
 UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;
  if (n->setHopStatus){
   hdr_cmn* cmh=HDR_CMN(pkt);
   cmh->next_hop()=n->next_hop;
   // printf("rmac:TxProcess: node %d set next hop to %d\n",index_,cmh->next_hop());
  }
 

  txbuffer.AddNewPacket(pkt);
  printf("rmac:TxProcess: node %d put new data packets in txbuffer\n",index_);
  if(!txbuffer.IsFull()) 
  if(callback_) callback_->handle(&status_event);
  return;
}


 
void 
RMac::StatusProcess(Event* p, TransmissionStatus  state)
{

   printf("RMac StatusProcess node %d \n",index_);
 UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;
 
 if(SLEEP==n->TransmissionStatus()) return;

    n->SetTransmissionStatus(state);

 return;
}




int
RMac::command(int argc, const char*const* argv)
{


     if(argc == 3) {
		TclObject *obj;
                 if (strcmp(argv[1], "node_on") == 0) {
		   Node* n1=(Node*) TclObject::lookup(argv[2]);
		   if (!n1) return TCL_ERROR;
		   node_ =n1; 
		   return TCL_OK;
		 }

		 /*
               if (strcmp(argv[1], "set_next_hop") == 0) {
		 setHopStatus=1;
		 next_hop=atoi(argv[2]);
		   return TCL_OK;
	       }
	      */
     }

	return UnderwaterMac::command(argc, argv);
}
