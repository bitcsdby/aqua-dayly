#include "packet.h"
#include "random.h"
#include "underwatersensor/uw_common/underwatersensornode.h"
#include "underwatersensor/uw_common/uwvb_header.h"
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
  // insert this packet at the head of the link
  t2=head_;
  head_=t1;
  t1->next=t2;
  
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
   delete t2;
   head_=t1;
   num_of_packet--;

   return p;
}

int 
TransmissionBuffer::DeletePacket(Packet* p){
  buffer_cell* t1;
  buffer_cell* t2;
  
  // insert this packet at the head of the link
  t2=head_;

  if (!t2) return 0;//0 no such point, 1:delete this point

  if (p==t2->packet){
    //printf("underwatermac: the packet is at the head of list\n");
    head_=t2->next;
    num_of_packet--;
    delete t2;
    return 1;
}
  
  int modified=0;
  while(t2->next){
    if ((t2->next)->packet!=p) t2=t2->next;
    else{
    
     t1=t2->next;
     t2->next=t1->next;
     num_of_packet--;
    delete t1;
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


bool 
TransmissionBuffer::IsEmpty(){
  return(0==num_of_packet);
}


bool 
TransmissionBuffer::IsFull(){
  return(MAXIMUM_BUFFER==num_of_packet);
}




int hdr_nd::offset_;
int hdr_ack_nd::offset_;
int hdr_syn::offset_;
int hdr_rev::offset_;
int hdr_ack_rev::offset_;
int hdr_rmac_data::offset_;


static class RMAC_DATA_HeaderClass: public PacketHeaderClass{
 public:
  RMAC_DATA_HeaderClass():PacketHeaderClass("Packetheader/RMAC_DATA",sizeof(hdr_rmac_data))
{
 bind_offset(&hdr_rmac_data::offset_);
}
} class_rmac_data_hdr;


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


MACRECVHandler::MACRECVHandler(RMac* p): mac_(p), duration(0), status(0){}

void 
MACRECVHandler::handle(Event* e)
{
  mac_->StartRECV(duration, status);
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
  mac_->TxRev( e);
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


LargeNDHandler::LargeNDHandler(RMac* p):mac_(p){}

void LargeNDHandler::handle(Event* e)
{ 
 
     mac_->cycle_start_time=NOW;
    mac_->SendND(mac_->large_packet_size_);
}


ShortNDHandler::ShortNDHandler(RMac* p):mac_(p){}

void ShortNDHandler::handle(Event* e)
{  
  
    mac_->cycle_start_time=NOW;
    mac_->SendND(mac_->short_packet_size_);
}



LargeAckNDWindowHandler::LargeAckNDWindowHandler(RMac* p):mac_(p){}

void LargeAckNDWindowHandler::handle(Event* e)
{ 
    mac_->SendLargeAckND();
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


RMac::RMac() :UnderwaterMac(),backoff_handler(this),large_nd_handler(this),short_nd_handler(this),large_acknd_window_handler(this),short_acknd_window_handler(this), status_handler(this), phaseone_handler(this),acknd_handler(this), phasetwo_handler(this), phasethree_handler(this), sleep_handler(this), wakeup_handler(this), reserve_handler(this), ackrev_handler(this),mac_recv_handler(this),timeout_handler(this),transmission_handler(this)
{
  num_send=0;
  large_packet_size_=30;
  short_packet_size_=10;
  large_latency_table_index=0;
  short_latency_table_index=0;

  reserved_time_table_index=0;
  reservation_table_index=0;

  period_table_index=0;

  next_period=0;  
  ack_rev_pt=NULL;
  //  SIF_=0;

  //  next_hop=0;
  // setHopStatus=0;

  
 for(int i=0;i<TABLE_SIZE;i++){
    large_latency_table[i].node_addr=-1;
    large_latency_table[i].num=0;
    large_latency_table[i].last_update_time=0.0;

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
    s.schedule(&phaseone_handler,&phaseone_event,t3*2);
    PhaseOne_cycle_--;
    return;
    }
  
   //   PrintTable();
   InitPhaseTwo();
   return;
}




void 
RMac::TxACKRev(Packet* pkt){

 
  DeleteBufferCell(pkt);
   
  hdr_cmn* cmh=HDR_CMN(pkt);
  hdr_ack_rev* ackh = HDR_ACK_REV(pkt); 

  printf("rmacTxackrev: node %d is transmitting a packet st=%f at time %f\n",index_,ackh->st,NOW);     
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();


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

  //  PrintTable();

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
  printf("node addr is%d and latency is%f\n",short_latency_table[i].node_addr,short_latency_table[i].latency);
}

 printf("RMac: the large latency table in node %d......\n",index_); 
 for (int i=0;i<TABLE_SIZE;i++)
{
  printf("node addr is%d and latency is%f\n",large_latency_table[i].node_addr,large_latency_table[i].latency);
}

printf("RMac: the period table in node%d...\n",index_); 

  for (int i=0;i<TABLE_SIZE;i++)
{
  printf("node addr is%d and difference is%f\n",period_table[i].node_addr,period_table[i].difference);
}

}

void 
RMac::ProcessSleep(){
   printf("RMac: node %d  is ProcessSleep at %f\n", index_,NOW); 

    Poweroff();
   if(reserved_time_table_index!=0) 
     {

   for (int i=0;i<reserved_time_table_index;i++){
    double nst=reserved_time_table[i].start_time-duration_;
    double lt=reserved_time_table[i].duration;
    if (nst<0) 
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
      if (nst<=(PeriodInterval_-duration_)) mac_status=RMAC_FORBIDDED;
    }
   }
   
   if (mac_status==RMAC_FORBIDDED){
     ClearACKRevLink(); 
     CancelReservation();
  
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
  
 
  Scheduler &s=Scheduler::instance();
  s.schedule(&wakeup_handler,&wakeup_event,(PeriodInterval_-duration_));
   return;
}


void 
RMac:: CancelReservation(){
   printf("rmac:cancelReservation: node %d\n",index_);
  if (reservation_table_index==0) return;
  else {
    reservation_table_index=0;
    for (int i=0;i<TABLE_SIZE;i++){
      reservation_table[i].node_addr=-1;
    } 
    return;
  }
}



void 
RMac::StartRECV(double dt, int id){

  if(id==0) 
    {
 printf("rmac:StartRECV: node %d at %f to power on\n",index_,NOW);
   Poweron();
   mac_status=RMAC_RECV;
   mac_recv_handler.status=1;
   Scheduler& s=Scheduler::instance();
   s.schedule(&mac_recv_handler, &mac_recv_event,dt);  
    }
  else {
 printf("rmac:StartRECV: node %d at %f to power off\n",index_,NOW);
     Poweroff();
  }
	 return ;
}


void 
RMac::ArrangeReservation(){
  // printf("rmac:ArrangeReservation: node %d\n",index_);
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
  //printf("rmac:ScheduleACKREV: node %d\n",index_);
  int i=0;
  double Number_Period=0;
  double last_time=0.0;

    printf("rmac:scheduleACKRev: node %d is scheduling ackrev\n",index_);
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


       if (dt>l-2*max_short_packet_transmissiontime) //??why 2 here
	  {

   double t1=dt-(l-2*max_short_packet_transmissiontime);
   Packet* ackrev=GenerateACKRev(receiver_addr,receiver, duration);
 
        t1=t1+Random::uniform()*(duration_-max_short_packet_transmissiontime);
 
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
          
  
       t1=t1+Random::uniform()*(duration_-max_short_packet_transmissiontime);
       
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
    t1=t1+Random::uniform()*(duration_-max_short_packet_transmissiontime); 
          }
	else{ 
        t1=PeriodInterval_+dt-(l-2*max_short_packet_transmissiontime);
        t1=t1+Random::uniform()*(duration_-max_short_packet_transmissiontime); 
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
        mac_recv_handler.duration=duration;

        s.schedule(&mac_recv_handler, &mac_recv_event,st);  
}


Packet* 
RMac::GenerateACKRev(int receiver, int intended_receiver, double duration){

  // printf("rmac:GenerateACKREV: node %d\n",index_);
      Packet* pkt =Packet::alloc();
      hdr_ack_rev* ackrevh = HDR_ACK_REV(pkt);
      hdr_cmn*  cmh = HDR_CMN(pkt);
 

       cmh->next_hop()=receiver;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_ACK_REV;
      
       
      ackrevh->pk_num = num_send;
      ackrevh->receiver_addr=intended_receiver;
      ackrevh->duration=duration;
      ackrevh->sender_addr=index_;
      num_send++;
      return pkt;
}



void 
RMac::SetStartTime(buffer_cell* ack_rev_pt, double st){

  buffer_cell* t1;
  t1=ack_rev_pt;
  while(t1){
    hdr_ack_rev*  ackrevh=HDR_ACK_REV(t1->packet);
    double d=t1->delay;
    ackrevh->st=st-d;
 printf("rmac setstarttime: node %d interval to recv =%f\n",index_,ackrevh->st);
    t1=t1->next;
  }
}
 


void 
RMac::InsertACKRevLink(Packet* p, double d){
  // printf("rmac:InsertACKREVLink: node %d\n",index_);
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
  // printf("rmac:ReserReservation: node %d\n",index_);
  for(int i=0;i<TABLE_SIZE;i++){
    reservation_table[i].node_addr=-1;
    reservation_table[i].required_time=0.0;
  }
}


int 
RMac::SelectReservation(){
  //printf("rmac:selectReservation: node %d\n",index_);
  int index=-1;
  double dt=-1.0;
  int i=0;

  while(!(reservation_table[i].node_addr==-1))
   {
    if (reservation_table[i].required_time>dt) index=i;
    i++;
   }
  return index;
}





void 
RMac::ResetMacStatus(){
  printf("node %d timeout at %f!\n\n",index_,NOW);
  mac_status=RMAC_IDLE;
}


void 
RMac::Wakeup(){

  printf("WakeUp node %d wake up at %f %d\n",index_,NOW);

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
                  printf("WakeUp: There is new data in node %d\n", index_);
                 mac_status=RMAC_REV;
                 MakeReservation();
                        }
                 break;
  case RMAC_FORBIDDED: ClearACKRevLink();
    break;
  case RMAC_WAIT_ACKREV:  
    printf("WakeUp NODE %d is in state RMAC_WAIT_ACK_REV\n",index_);
    break;
  default:  printf("WakeUp node %d don't expect to be in this state\n",index_);
    break;
  }


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
  
  t1=ack_rev_pt->next;
  while (t1){
    t2=t1->next;
    Packet::free(t1->packet);
    delete t1;
    t1=t2;
    ack_rev_pt=t1;
  }
   
}




void 
RMac::ProcessReservedTimeTable(){
  //   printf("rmac:ProcessReservedtimetable: node %d\n",index_);
  int i=0;
  while(i<reserved_time_table_index){
    double nst=reserved_time_table[i].start_time-PeriodInterval_;
    double lt=reserved_time_table[i].duration;
    if (nst<0) {
      if((lt+nst)<=0) {
        DeleteRecord(i);
	i--;    
      }
      else 
	{ // nst>=
       mac_status=RMAC_FORBIDDED;
       reserved_time_table[i].start_time=0;
       reserved_time_table[i].duration=lt+nst;
	}
    }//nst<0
    else {
      // nst>0
      if (nst<=PeriodInterval_) mac_status=RMAC_FORBIDDED;
    
      reserved_time_table[i].start_time=nst;
       reserved_time_table[i].duration=lt;
    }
    i++;
  }

  // if(reserved_time_table_index==0) mac_status=RMAC_IDLE;
}


void 
RMac::DeleteRecord(int index){
  // printf("rmac:deleteRecord: node %d\n",index_);
  for(int i=index;i<reserved_time_table_index;i++)
    {
      reserved_time_table[i].node_addr= reserved_time_table[i+1].node_addr;
      reserved_time_table[i].start_time= reserved_time_table[i+1].start_time;
      reserved_time_table[i].duration= reserved_time_table[i+1].duration;
      reserved_time_table_index--;
    }
}




bool 
RMac::NewData(){
  return (!txbuffer.IsEmpty());//?think about it
}


void 
RMac::MakeReservation(){
  //printf("rmac MakeReservation: node %d\n",index_);
  int num=txbuffer.num_of_packet;
  Packet* p=txbuffer.head();
  hdr_cmn*  cmh = HDR_CMN(p);
  int receiver_addr=cmh->next_hop();

  int sender_addr=index_;
  double dt=(num*(large_packet_size_*encoding_efficiency_+PhyOverhead_))/bit_rate_;

  // Generate a Rev Packet

      Packet* pkt =Packet::alloc();
      hdr_rev* revh = HDR_REV(pkt);
      cmh = HDR_CMN(pkt);
 

       cmh->next_hop()=receiver_addr;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_REV;
      
       revh->pk_num = num_send;
       revh->duration=dt;
       revh->sender_addr=index_;
       num_send++;
 
 
  // printf("rmac:Makereservation node %d num=%d size=%d, encoding_efficiency=%f phy=%d and bit_rate= %f\n",index_,num, large_packet_size_,encoding_efficiency_,PhyOverhead_, bit_rate_);  
       
      
        
    
       //      printf("rmac:Makereservation node %d send a reservation to node %d, duration is %f and type is %d\n",index_, receiver_addr,revh->duration, cmh->ptype_);  
   
      double l=CheckLatency(short_latency_table, receiver_addr);
          dt=CheckDifference(period_table,receiver_addr);

	  // printf("rmac:Makereservation node %d latency=%f and dt=%f\n",index_,l,dt);    

       if(dt<=0) dt=PeriodInterval_+dt;
double t2=Random::uniform()*(duration_-max_short_packet_transmissiontime);     

      if (dt>l-2*max_short_packet_transmissiontime)
	  {
       double t1=dt-(l-2*max_short_packet_transmissiontime);

       //printf("rmac:Makereservation node %d will send a reservation after %f now time is %f\n",index_,(t1+t2),NOW);  
       Scheduler& s=Scheduler::instance();
       s.schedule(&reserve_handler, (Event*) pkt,(t1+t2));
	  }
        else {
        double t1=PeriodInterval_+dt-(l-2*max_short_packet_transmissiontime);
	//printf("rmac:Makereservation node %d will send a reservation after %f, now time is %f\n",index_,(t1+t2),NOW);  
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

  Packet* pkt=(Packet*) e;
  //  printf("RMac TxREv node %d at %f\n",index_,NOW);
  hdr_cmn* cmh=HDR_CMN(pkt);
  //  hdr_syn* synh = HDR_SYN(pkt); 
  

 
  // printf("RMac TxREv node %d at %f packet type is %d\n",index_,NOW,cmh->ptype_);
  assert(initialized());
  UnderwaterSensorNode* n=(UnderwaterSensorNode*) node_;

 
  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();


  mac_status=RMAC_WAIT_ACKREV;

  printf("TxREV, node %d is in  RMAC_WAIT_ACKREV at %f\n",index_,NOW);

  double t=3*PeriodInterval_;

   Scheduler& s=Scheduler::instance();
   s.schedule(&timeout_handler,&timeout_event,t);

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
       hdr_syn* synh = HDR_SYN(pkt); 
       hdr_cmn*  cmh = HDR_CMN(pkt);
      
       cmh->size()=short_packet_size_;
       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_SYN;
      
      
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

  
  // cycle_start_time=NOW;
  double delay=Random::uniform()*t1;
  double itval=(t3-t2-t1)/2.0;
  double delay2=(t1+itval)+Random::uniform()*t2;
  double delay3=(t3+t1+itval);

 
    Scheduler& s=Scheduler::instance();
   s.schedule(&large_nd_handler, &large_nd_event, delay);
   s.schedule(&large_acknd_window_handler,&large_acknd_event,delay2);

   s.schedule(&short_nd_handler, &short_nd_event, (t3+delay));
   s.schedule(&short_acknd_window_handler,&short_acknd_event,delay3);
  return;
}

void 
RMac::SendND(int pkt_size)
{

      Packet* pkt =Packet:: alloc();
      hdr_nd* ndh = HDR_ND(pkt);
      // hdr_ip* iph = HDR_IP(pkt);
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
   
     // additional 2*8 denotes the size of type,next-hop of the packet and 
     // timestamp
  
      //       cmh->size()=sizeof(hdr_nd)+3*8;
      //  printf("old size is %d\n",cmh->size());
        cmh->size()=pkt_size;

       cmh->next_hop()=MAC_BROADCAST;
       cmh->direction()=hdr_cmn::DOWN; 
       cmh->addr_type()=NS_AF_ILINK;
       cmh->ptype_=PT_ND;
      
       
      // ndh->type = UW_ND;
      ndh->pk_num = num_send;
      // ndh->ts=NOW;
      ndh->sender_addr= node_->address();

      num_send++;
      //      lastND_time=NOW;

      // iph->src_.addr_=node_->address();
      // iph->dst_.addr_=node_->address();
      //iph->dst_.port_=255;     

 printf("rmac SendND:node(%d) send ND type is %d real is %d at %f\n", ndh->sender_addr,cmh->ptype_,PT_ND, NOW);
      TxND(pkt, ND_window_);  
}



void 
RMac::SendLargeAckND()
{
      Packet* pkt = Packet::alloc(sizeof(arrival_table));
      hdr_ack_nd* ackndh = HDR_ACK_ND(pkt);
    
      // initialize the arrival table


      hdr_cmn*  cmh = HDR_CMN(pkt);
      
    // ackndh->type = UW_ACK_ND;
       ackndh->pk_num = num_send;
      // ackndh->ts=NOW;
      ackndh->sender_addr=node_->address();
      num_send++;

      cmh->ptype_=PT_ACK_ND;

 // additional 2*8 denotes the size of type,next-hop of the packet and 
 // timestamp
      //cmh->size()=sizeof(hdr_ack_nd)+3*8;
       // printf("old size is %d\n",cmh->size());

       cmh->size()=large_packet_size_;


      cmh->next_hop()=MAC_BROADCAST;
      cmh->direction()=hdr_cmn::DOWN; 
      cmh->addr_type()=NS_AF_ILINK;

     

      memcpy(pkt->accessdata(),arrival_table,sizeof(arrival_table));

      // iph->src_.addr_=node_->address();
      // iph->dst_.addr_=node_->address();
      // iph->dst_.port_=255;     


 

  printf("rmac SendND:node(%d) send Large ACKND  at %f\n", ackndh->sender_addr, NOW);

  /*
    for(int i=0;i<TABLE_SIZE;i++){
       printf("node %d and arrival time %f\n", arrival_table[i].node_addr,arrival_table[i].arrival_time);	
     }
  */
       arrival_table_index=0; 
  for(int i=0;i<TABLE_SIZE;i++)
   arrival_table[i].node_addr=-1;
  
  

      TxND(pkt,ACKND_window_);  
}




void 
RMac::SendShortAckND()
{
  // printf("rmac:SendShortND: node %d\n",index_);
  if (arrival_table_index==0) return;// not ND received


  while(arrival_table_index>0){ 
      Packet* pkt = Packet::alloc();
      hdr_ack_nd* ackndh = HDR_ACK_ND(pkt);
    
      hdr_cmn*  cmh = HDR_CMN(pkt);
      
      ackndh->pk_num = num_send;
      ackndh->sender_addr=node_->address();
      num_send++;

      cmh->ptype_=PT_ACK_ND;
        
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
  hdr_syn* synh = HDR_SYN(pkt); 
  
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
    hdr_ack_rev* ackrevh=HDR_ACK_REV(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int receiver_addr=ackrevh->receiver_addr;
    double dt=ackrevh->duration;
    double st=ackrevh->st;
    int sender_addr=ackrevh->sender_addr;
 

    //printf("rmac:ProcessAckRevPacket:node %d I get the ACK REV packet interval is %f \n",index_, st);   
    double  l=CheckLatency(short_latency_table,sender_addr);
    double  it=st-l;
    double elapsedtime=NOW-cycle_start_time;
    
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

        double  l2=CheckLatency(short_latency_table,sender_addr);
        double  it1=it-l2+max_large_packet_transmissiontime;
      mac_status=RMAC_TRANSMISSION;
      Scheduler& s=Scheduler::instance();
printf("rmac:ProcessAckRevPacket: node %d schedule Txdata after %f at time %f\n",index_,it1,NOW);    
      s.cancel(&timeout_event);// cancel the timer of rev
      //      transmission_handler.receiver=sender_addr;      
      s.schedule(&transmission_handler,&transmission_event,it1);
    }
  
    return;
}


void 
RMac::ProcessRevPacket(Packet* pkt)
{
    hdr_rev* revh=HDR_REV(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);

    int sender_addr=revh->sender_addr;
    double dt=revh->duration;

    Packet::free(pkt);
   
    // printf("RMac:ProcessRevPacket: node %d is processing rev\n",index_);

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
    hdr_nd* ndh=HDR_ND(pkt);
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
  hdr_uwvb* vbh=HDR_UWVB(pkt);
   
  printf("rmac:ProcessDataPacket: node %d get data packet type is %d send up\n",index_,vbh->mess_type);
    uptarget_->recv(pkt,this);
      return;
}


void 
RMac::ProcessLargeACKNDPacket(Packet* pkt)
{
  // printf("rmac:ProcessLargeACKNDPacket: node %d\n",index_);
    hdr_ack_nd* ackndh=HDR_ACK_ND(pkt);
    hdr_cmn* cmh=HDR_CMN(pkt);
    time_record*  arrival_t=new time_record[TABLE_SIZE];

    memcpy((unsigned char*)arrival_t,pkt->accessdata(),sizeof(arrival_table));
     

    int  sender=ackndh->sender_addr;
    double t4=NOW;
    double t3=cmh->ts_;
   
    int myaddr=node_->address();
 
    double t2=-1.0;
    double t1=-1.0;
    

    for (int i=0;i<TABLE_SIZE;i++)
    if (arrival_t[i].node_addr==myaddr)
      {
         t2=arrival_t[i].arrival_time;
         t1=arrival_t[i].sending_time;
      }

    // printf("!!!node is %d  t1= %f t2=%f t3=%f t4=%f\n",myaddr,t1,t2,t3,t4);
    delete arrival_t;

     Packet::free(pkt);
       
    if(-1.0==t2) {
      printf("rmac:ProcessLargeACKNDPacket, node %d doesn't get my(%d) ND\n",sender,myaddr);
      return;
    }
  

double latency=((t4-t1)-(t3-t2))/2.0;
bool newone=true;   
   
 for (int i=0;i<TABLE_SIZE;i++)
 if (large_latency_table[i].node_addr==sender)
      {
       large_latency_table[i].sumLatency+=latency;
       large_latency_table[i].num++;
       large_latency_table[i].last_update_time=NOW;
       large_latency_table[i].latency = large_latency_table[i].sumLatency/large_latency_table[i].num;
       newone=false;
      }
 
 if(newone)
{

    if(large_latency_table_index>=TABLE_SIZE){ 
      printf("rmac:ProcessNDPacket:arrival_table is full\n");
      return;
    }

    large_latency_table[large_latency_table_index].node_addr=sender;
    large_latency_table[large_latency_table_index].sumLatency+=latency;
    large_latency_table[large_latency_table_index].num++;
    large_latency_table[large_latency_table_index].last_update_time=NOW;
    large_latency_table[large_latency_table_index].latency = large_latency_table[large_latency_table_index].sumLatency/large_latency_table[large_latency_table_index].num;
    large_latency_table_index++;
}

 
  for (int i=0;i<large_latency_table_index;i++)
    printf("node %d and node %d  large latency is %f and number is %d\n",myaddr,large_latency_table[i].node_addr,large_latency_table[i].latency,large_latency_table[i].num);
 
 return;

}

void 
RMac::ProcessShortACKNDPacket(Packet* pkt)
{
  // printf("rmac:ProcessshortACKNDPacket: node %d\n",index_);
    hdr_ack_nd* ackndh=HDR_ACK_ND(pkt);
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
    hdr_syn* synh=HDR_SYN(pkt);
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
RMac::RecvProcess(Packet* pkt){
   hdr_cmn* cmh=HDR_CMN(pkt);
   int dst=cmh->next_hop();
 
   
  if(dst==MAC_BROADCAST){
    printf("rmac:node %d  gets a broadcast packet at  %f and type is %d\n",index_,NOW, cmh->ptype_);
    if (cmh->ptype_==PT_ND) ProcessNDPacket(pkt); //this is ND packet

        // this is ACK_ND packet  
    if (cmh->ptype_==PT_ACK_ND) ProcessLargeACKNDPacket(pkt); 
    if (cmh->ptype_==PT_SYN) ProcessSYN(pkt);
   
    // uptarget_->recv(pkt, this);
    return;
  }

   if(dst==index_){
 printf("rmac:node %d  gets a packet at  %f and type is %d\n",index_,NOW, cmh->ptype_);
   if (cmh->ptype_==PT_ACK_ND) ProcessShortACKNDPacket(pkt); 
   if (cmh->ptype_==PT_REV) ProcessRevPacket(pkt);
   if(cmh->ptype_==PT_ACK_REV) ProcessACKRevPacket(pkt);
   if(cmh->ptype_==PT_RMAC_DATA) ProcessDataPacket(pkt);
     // printf("underwaterbroadcastmac:this is my packet \n");
     //  uptarget_->recv(pkt, this);
    return;
}
    printf("rmac: this is neither broadcast nor my packet(%d), just drop it at %f\n",index_, NOW);
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

    Packet* pkt=txbuffer.dehead();
          
    hdr_cmn* cmh=HDR_CMN(pkt);
    hdr_rmac_data* datah = HDR_DATA(pkt);
    hdr_uwvb* hdr2=HDR_UWVB(pkt);



   printf("RMac:node %d TxData at time %f data type is %d\n",index_,NOW,hdr2->mess_type);

      datah->sender_addr=index_;
 printf("RMac:node %d TxData at time %f data type is %d and pk_num=%d\n",index_,NOW,hdr2->mess_type,datah->pk_num);
    datah->pk_num=num_send;
 printf("RMac:node %d TxData at time %f data type is %d and pkm=%d\n",index_,NOW,hdr2->mess_type,datah->pk_num);
      num_send++;
   
  
          cmh->size()=large_packet_size_;

 printf("RMac:node %d TxData at time %f data type is %d\n",index_,NOW,hdr2->mess_type);
            cmh->next_hop()=receiver;
printf("RMac:node %d TxData at time %f data type is %d\n",index_,NOW,hdr2->mess_type);
            cmh->direction()=hdr_cmn::DOWN; 
            cmh->addr_type()=NS_AF_ILINK;
            cmh->ptype_=PT_RMAC_DATA;          

  hdr_cmn::access(pkt)->txtime()=(cmh->size()*encoding_efficiency_+
                                   PhyOverhead_)/bit_rate_;

  double txtime=hdr_cmn::access(pkt)->txtime();

 printf("RMac:node %d TxData at time %f data type is %d\n",index_,NOW,hdr2->mess_type);
  TransmissionStatus status=n->TransmissionStatus();

 

 if(IDLE==status)
 {
  n->SetTransmissionStatus(SEND); 
        sendDown(pkt);
        status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);  
 }

 if(RECV==status)
    {
      InterruptRecv(txtime);
      
      sendDown(pkt);

      status_handler.SetStatus(IDLE);
      Scheduler& s=Scheduler::instance();
      s.schedule(&status_handler,&status_event,txtime);
    }

 if (SEND==status)
    { 
    printf("rmac:Txdata: queue send data too fast\n");
    Packet::free(pkt);
    }
  

  if (txbuffer.IsEmpty()) {
   printf("rmac:Txdata: no data in queue of node %d\n",index_);
   mac_status=RMAC_IDLE; 
   Poweroff();
  }
  else {
  double it=SIF_+txtime;   

  Scheduler& s=Scheduler::instance();
  s.schedule(&transmission_handler,&transmission_event,it);
  }

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
 printf("rmac:TxProcess: node %d type is %d\n",index_,hdr->mess_type);
  }
 
  // printf("Rmac:TxProcess node%d received a data packet\n",index_);
  txbuffer.AddNewPacket(pkt);
  if(!txbuffer.IsFull())  
  if(callback_) callback_->handle(&status_event);
  return;
}


 
void 
RMac::StatusProcess(Event* p, TransmissionStatus  state)
{

  //  printf("RMac StatusProcess node %d \n",index_);
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
