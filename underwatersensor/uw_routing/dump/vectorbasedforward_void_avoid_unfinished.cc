#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <float.h>
#include <stdlib.h>

#include <tcl.h>


//#include "underwatersensor/uw_common/uwvb_header.h"
#include "agent.h"
#include "tclcl.h"
#include "ip.h"
#include "config.h"
#include "packet.h"
#include "trace.h"
#include "random.h"
#include "classifier.h"
#include "node.h"
#include "vectorbasedforward.h"
#include "god.h"
#include  "underwatersensor/uw_mac/underwaterpropagation.h"
#include "underwatersensor/uw_mac/underwaterchannel.h"

int hdr_uwvb::offset_;

static class UWVBHeaderClass: public PacketHeaderClass{
 public:
  UWVBHeaderClass():PacketHeaderClass("PacketHeader/UWVB",sizeof(hdr_uwvb))
{
 bind_offset(&hdr_uwvb::offset_);
} 
} class_uwvbhdr;




void UWPkt_Hash_Table::reset()
{
  neighborhood *hashPtr;
  Tcl_HashEntry *entryPtr;
  Tcl_HashSearch searchPtr;

  entryPtr = Tcl_FirstHashEntry(&htable, &searchPtr);
  while (entryPtr != NULL) {
    hashPtr = (neighborhood *)Tcl_GetHashValue(entryPtr);
     delete hashPtr;
    Tcl_DeleteHashEntry(entryPtr);
    entryPtr = Tcl_NextHashEntry(&searchPtr);
  }
}



neighborhood* UWPkt_Hash_Table::GetHash(ns_addr_t sender_id, 
					unsigned int pk_num)
{
  unsigned int key[3];

  key[0] = sender_id.addr_;
  key[1] = sender_id.port_;
  key[2] = pk_num;

  Tcl_HashEntry *entryPtr = Tcl_FindHashEntry(&htable, (char *)key);

  if (entryPtr == NULL )
     return NULL;

  return (neighborhood *)Tcl_GetHashValue(entryPtr);
}



void UWPkt_Hash_Table::put_in_hash(hdr_uwvb * vbh)
{
    Tcl_HashEntry *entryPtr;
    // Pkt_Hash_Entry    *hashPtr;
    neighborhood* hashPtr;
    unsigned int key[3];
    int newPtr;

    key[0]=(vbh->sender_id).addr_;
    key[1]=(vbh->sender_id).port_;
    key[2]=vbh->pk_num;


     int  k=key[2]-window_size;
    if(k>0)
      {
      for (int i=0;i<k;i++)
	{
          key[2]=i;
       entryPtr=Tcl_FindHashEntry(&htable, (char *)key);
      if (entryPtr)
     {
	hashPtr=(neighborhood*)Tcl_GetHashValue(entryPtr);
	delete hashPtr;
	Tcl_DeleteHashEntry(entryPtr);
      }
	}
      }     
     
  key[2]=vbh->pk_num;
    entryPtr = Tcl_CreateHashEntry(&htable, (char *)key, &newPtr);
    if (!newPtr){
     hashPtr=GetHash(vbh->sender_id,vbh->pk_num);
    int m=hashPtr->number;
    if (m<MAX_NEIGHBOR){
        hashPtr->number++;
	hashPtr->neighbor[m].x=0;
        hashPtr->neighbor[m].y=0;
	hashPtr->neighbor[m].z=0;
    }
      return;
}
    hashPtr=new neighborhood[1];
    hashPtr[0].number=1;
    hashPtr[0].neighbor[0].x=0;
    hashPtr[0].neighbor[0].y=0;
    hashPtr[0].neighbor[0].z=0;
    Tcl_SetHashValue(entryPtr, hashPtr);
   
}


void UWPkt_Hash_Table::put_in_hash(hdr_uwvb * vbh, position* p)
{
    Tcl_HashEntry *entryPtr;
    // Pkt_Hash_Entry    *hashPtr;
    neighborhood* hashPtr;
    unsigned int key[3];
    int newPtr;

    key[0]=(vbh->sender_id).addr_;
    key[1]=(vbh->sender_id).port_;
    key[2]=vbh->pk_num;


     int  k=key[2]-window_size;
    if(k>0)
      {
	for (int i=0;i<k;i++){
	  key[2]=i;
       entryPtr=Tcl_FindHashEntry(&htable, (char *)key);
      if (entryPtr)
     {
	hashPtr=(neighborhood*)Tcl_GetHashValue(entryPtr);
	delete hashPtr;
	Tcl_DeleteHashEntry(entryPtr);
      }

      }       
      }
       key[2]=vbh->pk_num;
    entryPtr = Tcl_CreateHashEntry(&htable, (char *)key, &newPtr);

    if (!newPtr)
{
     hashPtr=GetHash(vbh->sender_id,vbh->pk_num);
    int m=hashPtr->number;
  // printf("hash_table: this is not old item, there are %d item inside\n",m); 
    if (m<MAX_NEIGHBOR){
	hashPtr->neighbor[m].x=p->x;
        hashPtr->neighbor[m].y=p->y;
	hashPtr->neighbor[m].z=p->z;
          hashPtr->number++;
    }
    else {
	for(int i=1;i<MAX_NEIGHBOR;i++)
	{
	    hashPtr->neighbor[i-1].x=hashPtr->neighbor[i].x;
            hashPtr->neighbor[i-1].y=hashPtr->neighbor[i].y;
            hashPtr->neighbor[i-1].z=hashPtr->neighbor[i].z;
	}

	hashPtr->neighbor[MAX_NEIGHBOR-1].x=p->x;
        hashPtr->neighbor[MAX_NEIGHBOR-1].y=p->y;
	hashPtr->neighbor[MAX_NEIGHBOR-1].z=p->z;
    }

        return;
}
    hashPtr=new neighborhood[1];
    hashPtr[0].number=1;
    hashPtr[0].neighbor[0].x=p->x;
    hashPtr[0].neighbor[0].y=p->y;
    hashPtr[0].neighbor[0].z=p->z;
    Tcl_SetHashValue(entryPtr, hashPtr);
    return;   
}


void UWDelayTimer:: expire(Event* e)
{
  a_->timeout(packet, forward_type);
}


void UWVoidAvoidanceTimer:: expire(Event* e)
{
  a_->process_void_avoidance_timeout(source_addr,s_position,t_position,pkt_num,timer_type);
}

void UWFloodingTimer::expire(Event* e)
{
  a_->process_flooding_timeout(source_addr,pkt_num);
}

void UWFloodingForwardTimer::expire(Event* e)
{
  a_->process_flooding_forward_timeout(pkt);
}


static class VectorbasedforwardClass : public TclClass {
public:
  VectorbasedforwardClass() : TclClass("Agent/Vectorbasedforward") {}
  TclObject* create(int argc, const char*const* argv) {
    return(new VectorbasedforwardAgent());
  }
} class_vectorbasedforward;



VectorbasedforwardAgent::VectorbasedforwardAgent() : Agent(PT_UWVB),
	    delaytimer(this),buffer(),void_avoidance_timer(this)
{

  pk_count = 0;
  target_ = 0;
   node = NULL;
  tracetarget = NULL;
  width=0;
  counter=0;
  // priority=1.5;
 bind("width",& width);
  
}


void VectorbasedforwardAgent::recv(Packet* packet, Handler*)
{
  if (node->failure_status()==1){    
  printf ("vectorbasedforward%d: I fails!!!!\n ",here_.addr_);
  return;
  }
  
  hdr_uwvb* vbh = HDR_UWVB(packet);
  unsigned char msg_type =vbh->mess_type;
  unsigned int dtype = vbh->data_type;
  double t1=vbh->ts_;
  position * p1;
  ns_addr_t source=vbh->sender_id;
  ns_addr_t forwarder=vbh->forward_agent_id;  
  int pkt_num=vbh->pk_num; 

  if(msg_type==V_SHIFT){
 printf("vectrobasedforward node %d: recv  a v_shift packet from the sender %d\n",
      here_.addr_,vbh->forward_agent_id.addr_);
 Packet* p=void_avoidance_buffer.LookupCopy(source,pkt_num);

 if (!p){
     ConsiderNew(p,FORWARDER_CENTER);
 }
 return;
  }

       position p, p1;
      p.x=vbh->info.ox;
      p.y=vbh->info.oy;
      p.z=vbh->info.oz;
   
      p1.x=vbh->info.fx;
      p1.y=vbh->info.fy;
      p1.z=vbh->info.fz;    

  if(msg_type==FLOODING){

  neighborhood *hashPtr= CenterPktTable.GetHash(vbh->sender_id, vbh->pk_num);
    //printf("vectrobasedforward node %d: recv  packet %d from the sender %d\n",here_.addr_,vbh->pk_num,vbh->sender_id.addr_); 
     // Received this packet before ?

    if (hashPtr != NULL) {       
      CenterPktTable.put_in_hash(vbh);
      printf("vectrobasedforward node %d: this is duplicate flooding packet %d from the sender %d\n",here_.addr_,vbh->pk_num,vbh->sender_id.addr_); 
        Packet::free(packet);
                      }
      else {
     CenterPktTable.put_in_hash(vbh);
     processFloodingPacket(packet);     
      }
}

  if (IsVoidAvoidancePacket(pkt)){

       if(IsWorthForwarding(pkt)){
           VectorPktTable.put_in_hash(vbh,&p);
           ConsiderNew(packet); 
       }
       else 
        Packet::free(packet);
       
   return;
  // end of voidAvoidance Packet
  }

    neighborhood *hashPtr= PktTable.GetHash(vbh->sender_id, vbh->pk_num);
    //printf("vectrobasedforward node %d: recv  packet %d from the sender %d\n",here_.addr_,vbh->pk_num,vbh->sender_id.addr_); 
     // Received this packet before ?

    if (hashPtr != NULL) {       
      PktTable.put_in_hash(vbh,&p1);
    
      printf("vectrobasedforward node %d: this is duplicate packet %d from the sender %d\n",here_.addr_,vbh->pk_num,vbh->sender_id.addr_); 
        Packet::free(packet);
                      }
      else {
 
     PktTable.put_in_hash(vbh,&p1);
     VectorPktTable.put_in_hash(vbh,&p);
     // Take action for a new pkt
     ConsiderNew(packet);     
      }

}


bool VectorbasedforwardAgent::IsVoidAvoidancePacket(Packet* pkt){
// to test if the original source is the start of the vector
 

  hdr_uwvb* vbh = HDR_UWVB(pkt);
   

  position original_p, start_p;


  original_p.x=vbh->info.sx;
  original_p.y=vbh->info.sy;
  original_p.z=vbh->info.sz;

  start_p.x=vbh->info.ox;
  start_p.y=vbh->info.oy;
  start_p.z=vbh->info.oz;

  if((abs(start_p.x-original_p.x)<ERROR)&&(abs(start_p.y-original_p.y)<ERROR)&&
     (abs(start_p.z-original_p.z)<ERROR)) return false;
  else return true;
}

bool VectorbasedforwardAgent::IsWorthForwarding(packet* pkt)
{
 hdr_uwvb* vbh=HDR_UWVB(pkt);
 neighborhood *hashPtr= VectorPktTable.GetHash(vbh->sender_id, vbh->pk_num);
 if(!hashPtr) return true;
 ns_addr_t  source;
 int pkt_num, num_of_neighbors;
 
 source=vbh->sender_id;
 pkt_num=vbh->pk_num;
 
 position sp,tp,mp;

 tp.x=vbh->info.tx;
 tp.y=vbh->info.ty;
 tp.z=vbh->info.tz;

 my.x=node->CX();
 my.y=node->CY();
 my.z=node->CZ();
 
 sp.x=vbh->info.ox;
 sp.y=vbh->info.oy;
 sp.z=vbh->info.oz;

 double project1=projection(&sp,&tp,&mp);

 num_of_neighbors=hashPtr->number;
 position* position_list=hashPtr->neighbor;

 for(int i=0;i<num_of_neighbors){
     double project2=projection(&(position_list[i]),&tp,&mp);  
    
}

}

void VectorbasedforwardAgent::processFloodingPacket(Packet* pkt)
{
  hdr_uwvb* vbh = HDR_UWVB(pkt);
  unsigned char msg_type =vbh->mess_type;
  unsigned int dtype = vbh->data_type; 
  
  ns_addr_t   from_nodeID, target_id;
  int num=vbh->pk_num;

  target_id=vbh->target_id.addr_;
  from_nodeID=vbh->vbh->sender_id;  

 
  position * p1;
  p1=new position[1];

  p1[0].x=vbh->info.fx;
  p1[0].y=vbh->info.fy;
  p1[0].z=vbh->info.fz;    

          calculatePosition(pkt);	     
	      
	 if (THIS_NODE.addr_==target_id.addr_)
               {
	 // printf("Vectorbasedforward: %d is the target\n", here_.addr_);
	      DataForSink(pkt); // process it
	       }

	else{ 
  double delay=calculateFloodingDesirableness(pkt);
  set_flooding_forward_timer(pkt,delay); 	
	}

}


void VectorbasedforwardAgent::ConsiderNew(Packet *pkt, int method_type)
{
  hdr_uwvb* vbh = HDR_UWVB(pkt);
  unsigned char msg_type =vbh->mess_type;
  unsigned int dtype = vbh->data_type; 
  
//  double l,h;
  
  //Pkt_Hash_Entry *hashPtr;
   neighborhood * hashPtr;
  //  Agent_List *agentPtr;
  // PrvCurPtr  RetVal;
   ns_addr_t   from_nodeID, forward_nodeID, target_nodeID;

  Packet *gen_pkt;
  hdr_uwvb *gen_vbh;
  position p1,p;
  

  p.x=vbh->info.ox;
  p.y=vbh->info.oy;
  p.z=vbh->info.oz;

  p1.x=vbh->info.fx;
  p1.y=vbh->info.fy;
  p1.z=vbh->info.fz;    

  printf ("vectorbasedforward:(id :%d) forward:(%d ,%d) sender is(%d,%d,%d), (%f,%f,%f) the relative position is (%f ,%f,%f) forward position is is (%f,%f,%f) at time %f type is %d real one is %d\n",here_.addr_, vbh->forward_agent_id.addr_, vbh->forward_agent_id.port_,vbh->sender_id.addr_,vbh->sender_id.port_,vbh->pk_num,node->X(),node->Y(),node->Z(),vbh->info.dx,vbh->info.dy,vbh->info.dz, vbh->info.fx,vbh->info.fy,vbh->info.fz,NOW,vbh->mess_type,DATA);
 
  
  switch (msg_type) {
    case INTEREST : 
      // printf("Vectorbasedforward:it is interest packet!\n");
      hashPtr = PktTable.GetHash(vbh->sender_id, vbh->pk_num);

      // Check if it comes from sink agent of  this node
      // If so we have to keep it in sink list 

      from_nodeID = vbh->sender_id;
      forward_nodeID = vbh->forward_agent_id;
      //  printf("Vectorbasedforward:it the from_nodeid is %d %d  and theb this node id is %d ,%d!\n", from_nodeID.addr_,from_nodeID.port_,THIS_NODE.addr_,THIS_NODE.port_ );

      if (THIS_NODE.addr_ == from_nodeID.addr_) {       
   
      MACprepare(pkt);
      MACsend(pkt,0); 
      //      MACsend(pkt,Random::uniform()*JITTER); 
      printf("vectorbasedforward: after MACprepare(pkt)\n");
      }
      else
       {
          calculatePosition(pkt);
	 //printf("vectorbasedforward: This packet is from different node\n");
	 if (IsTarget(pkt)) 
           { 
            // If this node is target?
    	      l=advance(pkt);
        
	   //  send_to_demux(pkt,0);
	      //  printf("vectorbasedforward:%d send out the source-discovery \n",here_.addr_);
	     vbh->mess_type=SOURCE_DISCOVERY;
	     set_delaytimer(pkt,l*JITTER);
                 // !!! need to re-think
	   }
	 else{ 
	   // calculatePosition(pkt);
	   // No the target forwared
          l=advance(pkt);
          h=projection(pkt);
        if (IsCloseEnough(pkt)){
	  // printf("vectorbasedforward:%d I am close enough for the interest\n",here_.addr_);
      MACprepare(pkt);
      MACsend(pkt,Random::uniform()*JITTER);//!!!! need to re-think
	}
	else { 
	  //  printf("vectorbasedforward:%d I am not close enough for the interest  \n",here_.addr_);
         Packet::free(pkt);}
	 }
       }
      // Packet::free(pkt); 
      return;

  case TARGET_DISCOVERY: 
// from other nodes hitted by the packet, it is supposed
// to be the one hop away from the sink 

// printf("Vectorbasedforward(%d,%d):it is target-discovery  packet(%d)! it target id is %d  coordinate is %f,%f,%f and range is %f\n",here_.addr_,here_.port_,vbh->pk_num,vbh->target_id.addr_,vbh->info.tx, vbh->info.ty,vbh->info.tz,vbh->range);    
    if (THIS_NODE.addr_==vbh->target_id.addr_) {
      //printf("Vectorbasedforward(%d,??%d):it is target-discovery  packet(%d)! it target id is %d  coordinate is %f,%f,%f and range is %f\n",here_.addr_,here_.port_,vbh->pk_num,vbh->target_id.addr_,vbh->info.tx, vbh->info.ty,vbh->info.tz,vbh->range);    
      // ns_addr_t *hashPtr= PktTable.GetHash(vbh->sender_id, vbh->pk_num);
     // Received this packet before ?
      // if (hashPtr == NULL) { 

       calculatePosition(pkt);
       DataForSink(pkt);
       //	 printf("Vectorbasedforward: %d is the target\n", here_.addr_);
       // } //New data Process this data 
       // 
    } else  {Packet::free(pkt);}
   return;

  case SOURCE_DISCOVERY:
      Packet::free(pkt); 
// other nodes already claim to be the source of this interest
    //   SourceTable.put_in_hash(vbh);
    return;


 case DATA_READY :
   //  printf("Vectorbasedforward(%d,%d):it is data ready packet(%d)! it target id is %d \n",here_.addr_,here_.port_,vbh->pk_num,vbh->target_id.addr_);    
      from_nodeID = vbh->sender_id;
      if (THIS_NODE.addr_ == from_nodeID.addr_) {       
	// come from the same node, broadcast it
      MACprepare(pkt);
      MACsend(pkt,Random::uniform()*JITTER); 
      return;      
          }
          calculatePosition(pkt);
      if (THIS_NODE.addr_==vbh->target_id.addr_)
               {
        printf("Vectorbasedforward: %d is the target\n", here_.addr_);
	      DataForSink(pkt); // process it
	       } 
	else{
	  // printf("Vectorbasedforward: %d is the not  target\n", here_.addr_); 
      MACprepare(pkt);
      MACsend(pkt, Random::uniform()*JITTER);
	}
      return;

 
    case DATA :
      //     printf("Vectorbasedforward(%d,%d):it is data packet(%d)! it target id is %d  coordinate is %f,%f,%f and range is %f\n",here_.addr_,here_.port_,vbh->pk_num,vbh->target_id.addr_,vbh->info.tx, vbh->info.ty,vbh->info.tz,vbh->range);  

      // printf("Vectorbasedforward(%d) the traget address is %d\n",THIS_NODE.addr_,vbh->sender_id.addr_);   
  
      from_nodeID = vbh->sender_id;
      if (THIS_NODE.addr_ == from_nodeID.addr_) {       
	// come from the same node, broadcast it
      MACprepare(pkt);
      MACsend(pkt,0); 
      return;      
}

          calculatePosition(pkt);	     
	  // l=advance(pkt);
	  // h=projection(pkt);
       
	 if (THIS_NODE.addr_==vbh->target_id.addr_)
               {
		 // printf("Vectorbasedforward: %d is the target\n", here_.addr_);
	      DataForSink(pkt); // process it
	       }

	else{
     
	 if (IsCloseEnough(pkt, method_type)){

          if(IsWorthForwarding(pkt)){
           VectorPktTable.put_in_hash(vbh,&p);
	  }
          else Packet::free(pkt);
       
        void_avoidance_buffer.CopyNewPacket(pkt);
double delay=calculateDesirableness(pkt,method_type);
double d2=(UnderwaterChannel::Transmit_distance()-distance(pkt))/SPEED_OF_SOUND_IN_WATER;
double d3=(UnderwaterChannel::Transmit_distance())/SPEED_OF_SOUND_IN_WATER;
        
	   set_delaytimer(pkt,(sqrt(delay)*DELAY+d2*2),method_type); 
           set_shift_timer(pkt,V_SHIFT,DELAY+d3*2); // set the time for shift_vector 
           set_flooding_timer(source, pkt_num,DELAY*2+d3*4);
	  }
	  else { 
       // put the data packet into its buffer to wait for void-avoidance use 
	      void_avoidance_buffer.AddNewPacket(pkt);
   }
	    
	}
      return;

    default : 
      
      Packet::free(pkt);        
      break;
  }
}
void VectorbasedforwardAgent::reset()
{
  PktTable.reset();
  /*
  for (int i=0; i<MAX_DATA_TYPE; i++) {
    routing_table[i].reset();
  }
  */
}


void VectorbasedforwardAgent::Terminate() 
{
#ifdef DEBUG_OUTPUT
	printf("node %d: remaining energy %f, initial energy %f\n", THIS_NODE, 
	       node->energy_model()->energy(), 
	       node->energy_model()->initialenergy() );
#endif
}


void VectorbasedforwardAgent::StopSource()
{
  /*
  Agent_List *cur;

  for (int i=0; i<MAX_DATA_TYPE; i++) {
    for (cur=routing_table[i].source; cur!=NULL; cur=AGENT_NEXT(cur) ) {
      SEND_MESSAGE(i, AGT_ADDR(cur), DATA_STOP);
    }
  }
  */
}


Packet * VectorbasedforwardAgent:: create_packet()
{
  Packet *pkt = allocpkt();

  if (pkt==NULL) return NULL;

  hdr_cmn*  cmh = HDR_CMN(pkt);
  cmh->size() = 36;

  hdr_uwvb* vbh = HDR_UWVB(pkt);
  vbh->ts_ = NOW;
   
  //!! I add new part
 
  vbh->info.ox=node->CX();
  vbh->info.oy=node->CY(); 
  vbh->info.oz=node->CZ(); 
  vbh->info.fx=node->CX(); 
  vbh->info.fy=node->CY();
  vbh->info.fz=node->CZ();



  return pkt;
}


Packet *VectorbasedforwardAgent::prepare_message(unsigned int dtype, ns_addr_t to_addr,  int msg_type)

{
  Packet *pkt;
  hdr_uwvb *vbh;
  //hdr_ip *iph;

    pkt = create_packet();
    vbh = HDR_UWVB(pkt);
    // iph = HDR_IP(pkt);
    
    vbh->mess_type = msg_type;
    vbh->pk_num = pk_count;
    pk_count++;
    vbh->sender_id = here_;
    vbh->data_type = dtype;
    vbh->forward_agent_id = here_;

    vbh->ts_ = NOW;
    //    vbh->num_next = 1;
    // I am not sure if we need this
    // vbh->next_nodes[0] = to_addr.addr_;


    // I am not sure if we need it?    
    /*
    iph->src_ = here_;
    iph->dst_ = to_addr;
    */
    return pkt;
}

void VectorbasedforwardAgent::MACprepare(Packet *pkt, int method_type)
{

  hdr_uwvb* vbh = HDR_UWVB(pkt);
  hdr_cmn* cmh = HDR_CMN(pkt);

  if(method_type==FORWARDER_CENTER){
      vbh->info.ox=node->CX();
      vbh->info.oy=node>-CY();
      vbh->info.oz=node->CZ();
      CenterPktTable.put_in_hash(vbh);
  }

  //  hdr_ip*  iph = HDR_IP(pkt); // I am not sure if we need it


  vbh->forward_agent_id = here_; 
 
  cmh->xmit_failure_ = 0;
  // printf("vectorbased: the mac_Broadcast is:%d\n",MAC_BROADCAST);
  cmh->next_hop() = MAC_BROADCAST; 
  cmh->addr_type() = NS_AF_ILINK;  
  // cmh->txtime()=0;
  // printf("vectorbased: the address type is :%d and suppose to be %d and  nexthop %d MAC_BROAD %d\n", cmh->addr_type(),NS_AF_ILINK,cmh->next_hop(),MAC_BROADCAST);
  cmh->direction() = hdr_cmn::DOWN;
  // cmh->ptype_==PT_UWVB;
  // printf("vectorbased: the packet type is :%d\n", cmh->ptype_);
  //  printf("ok\n");

  //if (node) printf("ok, node is not empty\n");
  //printf("vectorbasedforward: inside MACprepare%d %d %d \n",node->X(),node->Y(),node->Z());
  

  // iph->src_ = here_;
  //iph->dst_.addr_ = MAC_BROADCAST;
  //iph->dst_.port_ = ROUTING_PORT;

  //  vbh->num_next = 1;
  // vbh->next_nodes[0] = MAC_BROADCAST;


  if(!node->sinkStatus()){       //!! I add new part
  vbh->info.fx=node->CX();
  vbh->info.fy=node->CY();
  vbh->info.fz=node->CZ();
  }
  else{
    vbh->info.fx=node->X();
    vbh->info.fy=node->Y();
    vbh->info.fz=node->Z();
}

}


void VectorbasedforwardAgent::MACsend(Packet *pkt, Time delay)
{
  hdr_cmn*  cmh = HDR_CMN(pkt);
  hdr_uwvb* vbh = HDR_UWVB(pkt);

 
 if (vbh->mess_type == DATA)
  cmh->size() = DATA_PACKET_SIZE ;
  else
      cmh->size() =CONTROL_PACKET_SIZE;
  Scheduler::instance().schedule(ll, pkt, delay);
}


void VectorbasedforwardAgent::DataForSink(Packet *pkt)
{

  //  printf("DataforSink: the packet is send to demux\n");
      send_to_dmux(pkt, 0);

}



void VectorbasedforwardAgent::trace (char *fmt,...)
{
  va_list ap;

  if (!tracetarget)
    return;

  va_start (ap, fmt);
  vsprintf (tracetarget->pt_->buffer(), fmt, ap);
  tracetarget->pt_->dump ();
  va_end (ap);
}

void VectorbasedforwardAgent::set_delaytimer(Packet* pkt, double c, int method_type){
 delaytimer.forward_type=method_type;
 delaytimer.packet=pkt; 
 delaytimer.resched(c);
}
 
void VectorbasedforwardAgent::set_shift_timer(ns_addr_t s_addr, int type, double c){
 void_avoidance_timer.source_addr=s_addr;
 void_avoidance_timer.timer_type=type;
 void_avoidance_timer.resched(c);
}

void VectorbasedforwardAgent::set_flooding_forward_timer(Packet* pkt, double delay)
{
    hdr_uwvb* vbh = HDR_UWVB(pkt);
    flooding_forward_timer.packet=pkt;
    flooding_forward_timer.resched(delay);
}

void VectorbasedforwardAgent::set_shift_timer(Packet* pkt, int type, int num, double c){
 
 hdr_uwvb* vbh = HDR_UWVB(pkt);
 
 void_avoidance_timer.source_addr=vbh->sender_id;

 void_avoidance_timer.s_position.x=vbh->info.ox;
 void_avoidance_timer.s_position.y=vbh->info.oy;
 void_avoidance_timer.s_position.z=vbh->info.oz;

 void_avoidance_timer.t_position.x=vbh->info.tx;
 void_avoidance_timer.t_position.y=vbh->info.ty;
 void_avoidance_timer.t_position.z=vbh->info.tz;

 void_avoidance_timer.pkt_num=vbh->pk_num;
 void_avoidance_timer.timer_type=type;
 void_avoidance_timer.resched(c);
}

void VectorbasedforwardAgent::process_flooding_timeout(ns_addr_t source,int pkt_num)
{
    position* forwarder_position;
    int num_of_forwarder;

    neighborhood *hashPtr= PktTable.GetHash(source, pkt_num);
    if(!hashPtr){
  printf ("vectorbasedforward%d: The packet for flooding is not in the buffer any more\n ",here_.addr_);
  return;
    }

    forwarder_position= hashPtr->neighbor;       
    num_of_forwarder=hashPtr->number;
    Packet* pkt=void_avoidance_timer.DeQueue(source,pkt_num);
    hdr_uwvb *vbh=HDR_UWVB(pkt);
    position sp,tp;

    sp.x=node->CX();
    sp.y=node->CY();
    sp.z=node->CZ();

    tp.x=vbh->info.tx;
    tp.y=vbh->info.ty;
    tp.z=vbh->info.tz;

    if  (IsVoidNode(forwarder_position,sp,tp,sp,num_of_forwarder))
    {       
	sendFloodingPacket(pkt);
    }
    else Packet::free(pkt);
    return;
}


void VectorbasedforwardAgent::process_flooding_forward_time(Packet* pkt)
{
    position* forwarder_position;
    int num_of_forwarder;
    ns_addr_t source;
    int pkt_num;
    hdr_uwvb* vbh=HDR_UWVB(pkt);   
  
    source=vbh->sender_id;
    pkt_num=vbh->pk_num;

    neighborhood *hashPtr= VectorPktTable.GetHash(source, pkt_num);

    void_avoidance_buffer.CopyNewPacket(pkt);
    double d3=(UnderwaterChannel::Transmit_distance())/SPEED_OF_SOUND_IN_WATER;
    set_flooding_timer(source, pkt_num,DELAY+d3*2);

  if(!hashPtr){
    MACprepare(pkt,FORWARDER_CENTER);
    MACsend(pkt,0);
       return;
    }

    forwarder_position= hashPtr->neighbor;       
    num_of_forwarder=hashPtr->number;

    position mp;
    position tp;

    mp.x=node->CX();
    mp.y=node->CY();
    mp.z=node->CZ();

    tp.x=vbh->tx;
    tp.y=vbh->ty;
    tp.z=vbh->tz;

 if  (IsValidFloodingForwardNode(forwarder_position,&tp,&mp,num_of_forwarder))
    {
       MACprepare(pkt,FORWARDER_CENTER);
       MACsend(pkt,0);
    }
    return;
}


bool VectorbasedforwardAgent::IsValidFloodingForwardNode(position* neighbor_list, const position* tp,const position* mp, int num);
// tp right now is  not used yet, think about it later
{
    if (num<1) return true;
    position p1;
    for(int i=0;i<num;i++){
    	 p1.x=neighbor_list[i].x;
	 p1.y=neighbor_list[i].y;
         p1.z=neighbor_list[i].z;
	 double distance=sqrt((p1.x-mp->x)*(p1.x-mp->x)+(p1.y-mp->y)*(p1.y-mp->y)+(p1.z-mp->z)*(p1.z-mp->z));
         if (distance<MIN_STEP) return false;
    }
    return true;
}



void VectorbasedforwardAgent::timeout(Packet * pkt, int method){

 hdr_uwvb* vbh = HDR_UWVB(pkt);
 unsigned char msg_type =vbh->mess_type;
 neighborhood  *hashPtr;
 //Packet* p1;

 switch (msg_type){
  
 case DATA:
       hashPtr= PktTable.GetHash(vbh->sender_id, vbh->pk_num);
	if (hashPtr != NULL) {
          int num_neighbor=hashPtr->number;          
	  // printf("vectorbasedforward: node %d have received %d when wake up at %f\n",here_.addr_,num_neighbor,NOW);
	  if (num_neighbor!=1){
      // Some guys transmit the data before me
	    if (num_neighbor==MAX_NEIGHBOR) {
             //I have too many neighbors, I quit
                  Packet::free(pkt);
		  return;  
	    }
	    else //I need to calculate my delay time again 
             {  
	       int i=0;
	       position* tp;
               tp=new position[1];

	      	 tp[0].x=hashPtr->neighbor[i].x;   
	         tp[0].y=hashPtr->neighbor[i].y;   
        	 tp[0].z=hashPtr->neighbor[i].z; 
                double tdelay=calculateDelay(pkt,tp);
                // double tdelay=5;
		i++;
                double c=1;
	         while (i<num_neighbor){
		   c=c*2;
		   tp[0].x=hashPtr->neighbor[i].x;   
		   tp[0].y=hashPtr->neighbor[i].y;   
		   tp[0].z=hashPtr->neighbor[i].z;   
		 double t2delay=calculateDelay(pkt,tp);
		 if (t2delay>tdelay) tdelay=t2delay;
		 	 i++; 
		  }
 
	       delete tp;
               if(tdelay<=(MINIMUM_THRESHOLD/c)) { 
		   if (       
		       //   makeCopy(pkt); 
               MACprepare(pkt, method);
               MACsend(pkt,0);      
		 }
               else
	   Packet::free(pkt);//to much overlap, don;t send 
	     }// end of calculate my new delay time 
	  }

 else{// I am the only neighbor
	       position* tp;
               tp=new position[1];
		 tp[0].x=vbh->info.fx;   
	         tp[0].y=vbh->info.fy;   
        	 tp[0].z=vbh->info.fz;   
		 double delay=calculateDelay(pkt,tp);
                 delete tp;
		 if (delay<=MINIMUM_THRESHOLD){
		   // printf("vectorbasedforward: !!%f\n",delay);
		     // makeCopy(pkt);
                    MACprepare(pkt,method);
                    MACsend(pkt,0);      
		              }
		 else  Packet::free(pkt);     
	 // printf("vectorbasedforward:  I%d am the only neighbor, I send it out at %f\n",here_.addr_,NOW);
	return;
 }
	}
	break; 
 default:
    Packet::free(pkt);  
   break;
 }
}

//not necessary
 void VectorbasedforwardAgent::makeCopy(Packet* pkt){
     Packet p1=pkt->copy();
     void_avoidance_buffer.AddNewPacket(pkt);        
 }


 void VectorbasedforwardAgent::sendFloodingPacket(Packet* pkt){
  
     hdr_uwvb* vbh=HDR_UWVB(pkt);
    
     vbh->message_type=FLOODING;   
     vbh->info.fx=node->CX();
     vbh->info.fy=node->CY();
     vbh->info.fz=node->CZ();
       
     MACprepare(pkt,FLOODING_TYPE);    
	 MACsend(pkt,0);
     return;
 }

void VectorbasedforwardAgent::process_void_avoidance_timeout(ns_addr_t souce,
                      position s_p, position t_p,int pkt_num,int timer_type)
{

    position* forwarder_position;
    int num_of_forwarder;

    neighborhood *hashPtr= PktTable.GetHash(source, pkt_num);

  if((!hashPtr)&&(timer_type==FLOODING)){
  	sendVectorShiftPacket(source,pkt_num);
       return;
    }

    if (!hashPtr){
  printf ("vectorbasedforward%d: The packet for vector-shift is not in the buffer any more\n ",here_.addr_);
  return;
    }

    forwarder_position= hashPtr->neighbor;       
    num_of_forwarder=hashPtr->number;

    position mp;
    mp.x=node->CX();
    mp.y=node->CY();
    mp.z=node->CZ();

    if  (IsVoidNode(forwarder_position,s_p,t_p,mp,num_of_forwarder))
    {
	sendVectorShiftPacket(source,pkt_num);
  
    }
    return;
}

 void VectorbasedforwardAgent::set_flooding_timer(ns_addr_t source, 
                                       int pkt_num, double c)
{
    flooding_timer.source=source;
    flooding_timer.pkt_num=pkt_num;
    flooding_timer.resched(c);
 }


void VectorbasedforwardAgent::sendVectorShiftPacket(ns_addr_t source,int pkt_num){

      Packet * v_shift=Packet::alloc();

      hdr_uwvb* vbh = HDR_UWVB(pkt);
      hdr_ip* iph = HDR_IP(pkt);
      hdr_cmn*  cmh = HDR_CMN(pkt);
     

      cmh->ptype()=PT_UWVB;
      cmh->size() = size_;

      iph->src_=here_;
      iph->dst_.addr_=here_.addr_;
      iph->dst_.port_=255;


      vbh->mess_type =V_SHIFT;
      vbh->pk_num = pkt_num;
      vbh->ts_=NOW;     
      vbh->sender_id = source;
      vbh->forward_agent_id=here_;       
 
        cmh->xmit_failure_ = 0;
        cmh->next_hop() = MAC_BROADCAST; 
        cmh->addr_type() = NS_AF_ILINK;  
        cmh->direction() = hdr_cmn::DOWN;
   
        FloodingPkTable.put_in_hash(vbh); // keep track of the slef-center packet

        MACsend(pkt, 0);
       printf("vbf (%d,%d) send packet v_shift packet %d at %lf\n ",
                        here_.addr_,here_.port_,pkt_num,NOW);
          
}


double VectorbasedforwardAgent::calculateFloodingDesirableness(const Packet* pkt)
{
    hdr_uwvb* vbh = HDR_UWVB(pkt);
    position sp,tp,fp, mp;
    double mx,my,mz;
    double fx,fy,fz;
   
    mx=node->CX();
    my=node->CY();
    mz=node->CZ();

   fx=vbh->info.fx;
   fy=vbh->info.fy;
   fz=vbh->info.fz;
   
   double d1=saqr((mx-fx)*(mx-fx)+(my-fy)*(my-fy)+(mz-fz)*(mz-fz));
   double d2=(UnderwaterChannel::Transmit_distance()-distance(pkt))
                         /SPEED_OF_SOUND_IN_WATER;

// this is not correct, change later, the delay should be inversely proportinal to the distance
   double  d1*DELAY+d2;       
}


double VectorbasedforwardAgent::calculateDesirablenss(const Packet* pkt, 
                                                     int method_type)
{

    hdr_uwvb* vbh = HDR_UWVB(pkt);
    position sp,tp,fp, mp;
 
    if (method_type==SOURCE_CENTER)
    {
	sp.x=vbh->info.ox;
        sp.y=vbh->info.oy;
        sp.z=vbh->info.oz;
    }
    
   if  (method_type==FORWARDER_CENTER)
   {
       sp.x=vbh->info.fx;
       sp.y=vbh->info.fy;
       sp.z=vbh->info.fz
   }
  
   tp.x=vbh->info.tx;
   tp.y=vbh->info.ty;
   tp.z=vbh->info.tz;

   fp.x=vbh->info.fx;
   fp.y=vbh->info.fy;
   fp.z=vbh->info.fz;


   mp.x=node->CX();
   mp.y=node->CY();
   mp.z=node->CZ();

   return calculateDelay(&sp,&tp,&mp,&fp);
}




bool VectorbasedforwardAgent::IsVoidNode(position* neighbor_list,position s_p,
                          position t_p, position mp,int num_of_neighbor)
{
//    if (num_of_neighbor==1) return true;
//    double* desirableness_factor=new double[num_of_neighbor];

    for(int i=0;i<num_of_neighbor;i++){
  double desirableness_factor=calculateDelay(&s_p,&t_p,neighbor_list[i],&mp);
        if (MINIMUM_PRIORITY>=my_desirableness_factor) return false;
    }
    return true;
}


double VectorbasedforwardAgent::calculateDelay(const position* sp,const position*  tp,
                                                const position* p, const position* myp)
{
// the two calculateDelay should be merged
 double fx=myp->x;
 double fy=myp->y;
 double fz=myp->z;

 double dx=p->x-fx; 
 double dy=p->y-fy;
 double dz=p->z-fz;

  
 double tx=tp->x;
 double ty=tp->y;
 double tz=tp->z; 

 double dtx=tx-fx;
 double dty=ty-fy;
 double dtz=tz-fz;  

 double dp=dx*dtx+dy*dty+dz*dtz;

 double p=projection(sp,tp,p);
 double d=sqrt((dx*dx)+(dy*dy)+ (dz*dz));
 double l=sqrt((dtx*dtx)+(dty*dty)+ (dtz*dtz));
 double cos_theta=dp/(d*l);

   double delay=(p/width) +((UnderwaterChannel::Transmit_distance()-d*cos_theta)/UnderwaterChannel::Transmit_distance());
 
  //printf("vectorbased: node(%d) projection is %f, and cos is %f, and d is %f)\n",here_.addr_,p, cos_theta, d);
   return delay;
}


double VectorbasedforwardAgent::projection(const position* sp, const position* tp,const position * p)
{
// two projection functions should be merged later
 double tx=tp->x;
 double ty=tp->y;
 double tz=tp->z;
 

 double ox=sp->x;
 double oy=sp->y;
 double oz=sp->z;

 double x=p->x;
 double y=p->y;
 double z=p->z;
 
 double wx=tx-ox;
 double wy=ty-oy;
 double wz=tz-oz;

 double vx=x-ox;
 double vy=y-oy;
 double vz=z-oz;

 double cross_product_x=vy*wz-vz*wy;
 double cross_product_y=vz*wx-vx*wz;
 double cross_product_z=vx*wy-vy*wx;
  
 double area=sqrt(cross_product_x*cross_product_x+ 
          cross_product_y*cross_product_y+cross_product_z*cross_product_z);
 double length=sqrt((tx-ox)*(tx-ox)+(ty-oy)*(ty-oy)+ (tz-oz)*(tz-oz));
 // printf("vectorbasedforward: the area is %f and length is %f\n",area,length);
 return area/length;
}


void VectorbasedforwardAgent::calculatePosition(Packet* pkt)
{
 
 hdr_uwvb     *vbh  = HDR_UWVB(pkt); 
 double fx=vbh->info.fx;
 double fy=vbh->info.fy;
 double fz=vbh->info.fz;

 double dx=vbh->info.dx;
 double dy=vbh->info.dy;
 double dz=vbh->info.dz;

 node->CX_=fx+dx;
 node->CY_=fy+dy;
 node->CZ_=fz+dz;
 // printf("vectorbased: my position is computed as (%f,%f,%f)\n",node->CX_, node->CY_,node->CZ_);
}

double VectorbasedforwardAgent::calculateDelay(Packet* pkt,position* p1)
{
 
 hdr_uwvb     *vbh  = HDR_UWVB(pkt); 
 double fx=p1->x;
 double fy=p1->y;
 double fz=p1->z;

 double dx=node->CX_-fx; 
 double dy=node->CY_-fy;
 double dz=node->CZ_-fz;

  
 double tx=vbh->info.tx;
 double ty=vbh->info.ty;
 double tz=vbh->info.tz; 

 double dtx=tx-fx;
 double dty=ty-fy;
 double dtz=tz-fz;  

 double dp=dx*dtx+dy*dty+dz*dtz;

 // double a=advance(pkt);
 double p=projection(pkt);
 double d=sqrt((dx*dx)+(dy*dy)+ (dz*dz));
 double l=sqrt((dtx*dtx)+(dty*dty)+ (dtz*dtz));
 double cos_theta=dp/(d*l);
 // double delay=(TRANSMISSION_DISTANCE-d*cos_theta)/TRANSMISSION_DISTANCE;
   double delay=(p/width) +((UnderwaterChannel::Transmit_distance()-d*cos_theta)/UnderwaterChannel::Transmit_distance());
 // double delay=(p/width) +((TRANSMISSION_DISTANCE-d)/TRANSMISSION_DISTANCE)+(1-cos_theta);
  //printf("vectorbased: node(%d) projection is %f, and cos is %f, and d is %f)\n",here_.addr_,p, cos_theta, d);
   return delay;
}


double VectorbasedforwardAgent::distance(Packet* pkt)
{
 
 hdr_uwvb     *vbh  = HDR_UWVB(pkt); 
 double tx=vbh->info.fx;
 double ty=vbh->info.fy;
 double tz=vbh->info.fz;
 // printf("vectorbased: the target is %lf,%lf,%lf \n",tx,ty,tz);
 double x=node->CX(); //change later
 double y=node->CY();// printf(" Vectorbasedforward: I am in advanced\n");
 double z=node->CZ();
 // printf("the target is %lf,%lf,%lf and my coordinates are %lf,%lf,%lf\n",tx,ty,tz,x,y,z);
 return sqrt((tx-x)*(tx-x)+(ty-y)*(ty-y)+ (tz-z)*(tz-z));
}


double VectorbasedforwardAgent::advance(Packet* pkt)
{
 
 hdr_uwvb     *vbh  = HDR_UWVB(pkt); 
 double tx=vbh->info.tx;
 double ty=vbh->info.ty;
 double tz=vbh->info.tz;
 // printf("vectorbased: the target is %lf,%lf,%lf \n",tx,ty,tz);
 double x=node->CX(); //change later
 double y=node->CY();// printf(" Vectorbasedforward: I am in advanced\n");
 double z=node->CZ();
 // printf("the target is %lf,%lf,%lf and my coordinates are %lf,%lf,%lf\n",tx,ty,tz,x,y,z);
 return sqrt((tx-x)*(tx-x)+(ty-y)*(ty-y)+ (tz-z)*(tz-z));
}


double VectorbasedforwardAgent::projection(Packet* pkt, int method_type)
{

 hdr_uwvb     *vbh  = HDR_UWVB(pkt);
 
 double tx=vbh->info.tx;
 double ty=vbh->info.ty;
 double tz=vbh->info.tz;
 
 if(method_type==SOURCE_CENTER)
 {
 double ox=vbh->info.ox;
 double oy=vbh->info.oy;
 double oz=vbh->info.oz;
 }

 if(method_type==FPRWARDER_CENTER)
 {
 double ox=vbh->info.fx;
 double oy=vbh->info.fy;
 double oz=vbh->info.fz;
 }


 double x=node->CX();
 double y=node->CY();
 double z=node->CZ();
 
 double wx=tx-ox;
 double wy=ty-oy;
 double wz=tz-oz;

 double vx=x-ox;
 double vy=y-oy;
 double vz=z-oz;

 double cross_product_x=vy*wz-vz*wy;
 double cross_product_y=vz*wx-vx*wz;
 double cross_product_z=vx*wy-vy*wx;
  
 double area=sqrt(cross_product_x*cross_product_x+ 
          cross_product_y*cross_product_y+cross_product_z*cross_product_z);
 double length=sqrt((tx-ox)*(tx-ox)+(ty-oy)*(ty-oy)+ (tz-oz)*(tz-oz));
 return area/length;
}

bool VectorbasedforwardAgent::IsTarget(Packet* pkt)
{
  hdr_uwvb * vbh=HDR_UWVB(pkt);

  if (vbh->target_id.addr_==0){

  //  printf("vectorbased: advanced is %lf and my range is %f\n",advance(pkt),vbh->range);
    return (advance(pkt)<vbh->range);
}
  else return(THIS_NODE.addr_==vbh->target_id.addr_);


}



bool VectorbasedforwardAgent::IsCloseEnough(Packet* pkt, int method_type)
{
  hdr_uwvb     *vbh  = HDR_UWVB(pkt);
  double range=vbh->range;

 
  position sp, tp,p;

  if (method_type==SOURCE_CENTER){
  sp.x=vbh->info.ox;
  sp.y=vbh->info.oy;
  sp.z=vbh->info.oz;
  }
  if (method_type==FORWARDER_CENTER){
  sp.x=vbh->info.fx;
  sp.y=vbh->info.fy;
  sp.z=vbh->info.fz;
  }
 
  tp.x=vbh->info.tx;
  tp.y=vbh->info.ty;
  tp.z=vbh->info.tz;

  p.x=node->CX();
  p.y=node->CY();
  p.z=node->CZ();


 if ((projection(&sp,&tp,&p)<=(c*width)))  return true;
 return false;

}


int VectorbasedforwardAgent::command(int argc, const char*const* argv)
{  
  Tcl& tcl =  Tcl::instance();

  if (argc == 2) {

    if (strcasecmp(argv[1], "reset-state")==0) {
      
      reset();
      return TCL_OK;
    }

    if (strcasecmp(argv[1], "reset")==0) {
      
      return Agent::command(argc, argv);
    }

    if (strcasecmp(argv[1], "start")==0) {
      return TCL_OK;
    }

    if (strcasecmp(argv[1], "stop")==0) {
      return TCL_OK;
    }

    if (strcasecmp(argv[1], "terminate")==0) {
      Terminate();
      return TCL_OK;
    }

   if (strcasecmp(argv[1], "name")==0) {
     printf("vectorbased \n");
      return TCL_OK;
    }
    if (strcasecmp(argv[1], "stop-source")==0) {
      StopSource();
      return TCL_OK;
    }

  } else if (argc == 3) {

    if (strcasecmp(argv[1], "on-node")==0) {
      //   printf ("inside on node\n");
      node = (UnderwaterSensorNode *)tcl.lookup(argv[2]);
      return TCL_OK;
    }
    /*
      if (strcasecmp(argv[1], "set-port")==0) {
      printf ("inside on node\n");
      port_number=atoi(argv[2]);
      return TCL_OK;
    }
    */
    if (strcasecmp(argv[1], "add-ll") == 0) {

      TclObject *obj;

      if ( (obj = TclObject::lookup(argv[2])) == 0) {
    fprintf(stderr, "Vectorbasedforwarding Node: %d lookup of %s failed\n", THIS_NODE.addr_, argv[2]);
	return TCL_ERROR;
      }
      ll = (NsObject *) obj;

     return TCL_OK;
    }

    if (strcasecmp (argv[1], "tracetarget") == 0) {
      TclObject *obj;
      if ((obj = TclObject::lookup (argv[2])) == 0) {
	  fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
		   argv[2]);
	  return TCL_ERROR;
      }

      tracetarget = (Trace *) obj;
      return TCL_OK;
    }

    if (strcasecmp(argv[1], "port-dmux") == 0) {
      // printf("vectorbasedforward:port demux is called \n");
      TclObject *obj;

      if ( (obj = TclObject::lookup(argv[2])) == 0) {
	fprintf(stderr, "VB node Node: %d lookup of %s failed\n", THIS_NODE.addr_, argv[2]);
	return TCL_ERROR;
      }
      port_dmux = (NsObject *) obj;
      return TCL_OK;
    }

  } 

  return Agent::command(argc, argv);
}


// Some methods for Flooding Entry

/*
void Vectorbasedforward_Entry::reset()
{
    clear_agentlist(source);
    clear_agentlist(sink);
    source = NULL;
    sink = NULL;
}

void Vectorbasedforward_Entry::clear_agentlist(Agent_List *list)
{
  Agent_List *cur=list;
  Agent_List *temp = NULL;

  while (cur != NULL) {
    temp = AGENT_NEXT(cur);
    delete cur;
    cur = temp;
  }
}

*/
