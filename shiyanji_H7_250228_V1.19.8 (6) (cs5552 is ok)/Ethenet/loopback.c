#include <stdio.h>
#include <string.h>
#include "loopback.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "gParameter.h"

//#define DEBUG2	

#if LOOPBACK_MODE == LOOPBACK_MAIN_NOBLCOK

int32_t loopback_tcps(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t ret;
   uint16_t size = 0, sentsize=0;

#ifdef _LOOPBACK_DEBUG_
   uint8_t destip[4];
   uint16_t destport;
#endif

   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(sn) & Sn_IR_CON)
         {
#ifdef _LOOPBACK_DEBUG_
			getSn_DIPR(sn, destip);
			destport = getSn_DPORT(sn);

			printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			setSn_IR(sn,Sn_IR_CON);
         }
		 if((size = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
         {
			if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
			ret = recv(sn, buf, size);

			if(ret <= 0) return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
			size = (uint16_t) ret;
			sentsize = 0;
			
			buf[size]=0x00;
			printf("%s\r\n",buf);
					 
			while(size != sentsize)
			{
				ret = send(sn, buf+sentsize, size-sentsize);
				if(ret < 0)
				{
					close(sn);
					return ret;
				}
				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
			}
         }
         break;
      case SOCK_CLOSE_WAIT :
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:CloseWait\r\n",sn);
#endif
         if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Socket Closed\r\n", sn);
#endif
         break;
      case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG_
    	 printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
#endif
         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:TCP server loopback start\r\n",sn);
#endif
         if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:Socket opened\r\n",sn);
#endif
         break;
      default:
         break;
   }
   return 1;
}


int32_t loopback_tcpc(uint8_t sn, uint8_t* buf, uint8_t* destip, uint16_t destport)
{
   int32_t ret; // return value for SOCK_ERRORs
   uint16_t size = 0, sentsize=0;
#ifdef DEBUG2	
	static uint32_t flag=0;
	printf("flags:%d\r\n",++flag);
#endif
   // Destination (TCP Server) IP info (will be connected)
   // >> loopback_tcpc() function parameter
   // >> Ex)
   //	uint8_t destip[4] = 	{192, 168, 0, 214};
   //	uint16_t destport = 	5000;

   // Port number for TCP client (will be increased)
   static uint16_t any_port = 	50000;

   // Socket Status Transitions
   // Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :
			#ifdef DEBUG2		
				printf("SOCK_ESTABLISHED\r\n");
			#endif
         if(getSn_IR(sn) & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
         {
#ifdef _LOOPBACK_DEBUG_
			printf("%d:Connected to - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			setSn_IR(sn, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
         }

         //////////////////////////////////////////////////////////////////////////////////////////////
         // Data Transaction Parts; Handle the [data receive and send] process
         //////////////////////////////////////////////////////////////////////////////////////////////
		 if((size = getSn_RX_RSR(sn)) > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
         {
			if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE; // DATA_BUF_SIZE means user defined buffer size (array)
			ret = recv(sn, buf, size); // Data Receive process (H/W Rx socket buffer -> User's buffer)

			if(ret <= 0) return ret; // If the received data length <= 0, receive failed and process end
			size = (uint16_t) ret;
			sentsize = 0;

			buf[size]=0x00;
			printf("%s\r\n",buf);		 
			
			// Data sentsize control
			while(size != sentsize)
			{
				ret = send(sn, buf+sentsize, size-sentsize); // Data send process (User's buffer -> Destination through H/W Tx socket buffer)
				if(ret < 0) // Send Error occurred (sent data length < 0)
				{
					close(sn); // socket close
					return ret;
				}
				sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
			}
         }
		 //////////////////////////////////////////////////////////////////////////////////////////////
         break;

      case SOCK_CLOSE_WAIT :
			#ifdef DEBUG2	
				printf("SOCK_CLOSE_WAIT\r\n");
			#endif
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:CloseWait\r\n",sn);
#endif
         if((ret=disconnect(sn)) != SOCK_OK) return ret;
			  close(sn);
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Socket Closed\r\n", sn);
#endif
         break;

      case SOCK_INIT :
#ifdef DEBUG2	
				printf("SOCK_INIT\r\n");
#endif
#ifdef _LOOPBACK_DEBUG_
    	 printf("%d:Try to connect to the %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
    	 if( (ret = connect(sn, destip, destport)) != SOCK_OK) return ret;	//	Try to TCP connect to the TCP server (destination)
         break;

      case SOCK_CLOSED:
#ifdef DEBUG2	
				printf("SOCK_CLOSED\r\n");
#endif
    	  close(sn);
    	  if((ret=socket(sn, Sn_MR_TCP, any_port++, 0x00)) != sn){
         if(any_port == 0xffff) any_port = 50000;
         return ret; // TCP socket open with 'any_port' port number
        } 
#ifdef _LOOPBACK_DEBUG_
    	 //printf("%d:TCP client loopback start\r\n",sn);
         //printf("%d:Socket opened\r\n",sn);
#endif
         break;
      default:
         break;
   }
   return 1;
}


int32_t loopback_udps(uint8_t sn, uint8_t* buf, uint16_t port)
{
   int32_t  ret;
   uint16_t size, sentsize;
   uint8_t  destip[4];
   uint16_t destport;

   switch(getSn_SR(sn))
   {
      case SOCK_UDP :
         if((size = getSn_RX_RSR(sn)) > 0)
         {
            if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
            ret = recvfrom(sn, buf, size, destip, (uint16_t*)&destport);
            if(ret <= 0)
            {
#ifdef _LOOPBACK_DEBUG_
               printf("%d: recvfrom error. %d\r\n",sn,ret);
#endif
               return ret;
            }
            size = (uint16_t) ret;
            sentsize = 0;
						
												buf[size]=0x00;
			      printf("%s\r\n",buf);
            while(sentsize != size)
            {
               ret = sendto(sn, buf+sentsize, size-sentsize, destip, destport);
               if(ret < 0)
               {
#ifdef _LOOPBACK_DEBUG_
                  printf("%d: sendto error. %d\r\n",sn,ret);
#endif
                  return ret;
               }
               sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
            }
         }
         break;
      case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:UDP loopback start\r\n",sn);
#endif
         if((ret = socket(sn, Sn_MR_UDP, port, 0x00)) != sn)
            return ret;
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Opened, UDP loopback, port [%d]\r\n", sn, port);
#endif
         break;
      default :
         break;
   }
   return 1;
}

int32_t utc_tcps(uint8_t sn, uint8_t* revbuf,uint16_t* revsize,uint8_t* sendbuf,uint16_t* sendsize,uint16_t port)
{
   int32_t ret;
	uint16_t i,j;

#ifdef _LOOPBACK_DEBUG_
   uint8_t destip[4];
   uint16_t destport;
#endif

   switch(getSn_SR(sn))
   {
      case SOCK_ESTABLISHED :
         if(getSn_IR(sn) & Sn_IR_CON)
         {
#ifdef _LOOPBACK_DEBUG_
			getSn_DIPR(sn, destip);
			destport = getSn_DPORT(sn);

			printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			setSn_IR(sn,Sn_IR_CON);
         }
				 //ETH receive
					 if((*revsize = getSn_RX_RSR(sn)) > 0) // Don't need to check SOCKERR_BUSY because it doesn't not occur.
							 {
						if(*revsize > DATA_BUF_SIZE) *revsize = DATA_BUF_SIZE;
						ret = recv(sn, revbuf, *revsize);

						if(ret <= 0) return ret;      // check SOCKERR_BUSY & SOCKERR_XXX. For showing the occurrence of SOCKERR_BUSY.
						*revsize = (uint16_t) ret;
								 
				 //	send incoming ETH data by usart	
						revbuf[*revsize]=0x00;
//						printf("%s\r\n",revbuf);
							 }
				//ETH transmit		
					 if(*sendsize != 0)
					 {
							ret = send(sn, sendbuf, *sendsize);
							if(ret < 0)
							{
								close(sn);
								return ret;
							}
					//从缓冲区中删除已传输的数据
							for(i=0,j=0;i<*sendsize;i++)
							{	
								if(i<ret)
									sendbuf[i]=0x00;
								else
								{
									sendbuf[j++]=sendbuf[i];
								  sendbuf[i]=0x00;
								}
							}
							if((*sendsize - ret)>=0)
							*sendsize = *sendsize - ret;//subtract the length of the sent data
							else
								*sendsize =0;
						}
         break;
      case SOCK_CLOSE_WAIT :
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:CloseWait\r\n",sn);
#endif
         if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Socket Closed\r\n", sn);
#endif
         break;
      case SOCK_INIT :
#ifdef _LOOPBACK_DEBUG_
    	 printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
#endif
         if( (ret = listen(sn)) != SOCK_OK) return ret;
         break;
      case SOCK_CLOSED:
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:TCP server loopback start\r\n",sn);
#endif
         if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:Socket opened\r\n",sn);
#endif
         break;
      default:
         break;
   }
   return 1;
}



int32_t utc_tcpc(uint8_t sn,uint8_t* destip, uint16_t destport)
{
   int32_t ret; // return value for SOCK_ERRORs
#ifdef DEBUG2	
//	static uint32_t flag=0;
//	printf("flags:%d\r\n",++flag);
#endif
   static uint16_t any_port = 	50000; //可固定可以自增

   // Socket Status Transitions
   // Check the W5500 Socket n status register (Sn_SR, The 'Sn_SR' controlled by Sn_CR command or Packet send/recv status)
   switch(stateFlag.eth_scoket_state=getSn_SR(sn))
   {
		 
      case SOCK_ESTABLISHED :
			#ifdef DEBUG2		
				printf("SOCK_ESTABLISHED\r\n");
			#endif
         if(getSn_IR(sn) & Sn_IR_CON)	// Socket n interrupt register mask; TCP CON interrupt = connection with peer is successful
         {
#ifdef _LOOPBACK_DEBUG_
			printf("%d:Connected to - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
			setSn_IR(sn, Sn_IR_CON);  // this interrupt should be write the bit cleared to '1'
         }
         break;

      case SOCK_CLOSE_WAIT :
			#ifdef DEBUG2	
				printf("SOCK_CLOSE_WAIT\r\n");
			#endif
#ifdef _LOOPBACK_DEBUG_
         //printf("%d:CloseWait\r\n",sn);
#endif
         if((ret=disconnect(sn)) != SOCK_OK) return ret;
			  close(sn);
#ifdef _LOOPBACK_DEBUG_
         printf("%d:Socket Closed\r\n", sn);
#endif
         break;

      case SOCK_INIT :
#ifdef DEBUG2	
				printf("SOCK_INIT\r\n");
#endif
#ifdef _LOOPBACK_DEBUG_
    	 printf("%d:Try to connect to the %d.%d.%d.%d : %d\r\n", sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
    	 if( (ret = connect(sn, destip, destport)) != SOCK_OK) return ret;	//	Try to TCP connect to the TCP server (destination)
         break;

      case SOCK_CLOSED:
#ifdef DEBUG2	
				printf("SOCK_CLOSED\r\n");
#endif
    	  close(sn);
    	  if((ret=socket(sn, Sn_MR_TCP, any_port, 0x00)) != sn){
         if(any_port == 0xffff) any_port = 50000;
         return ret; // TCP socket open with 'any_port' port number
        } 
#ifdef _LOOPBACK_DEBUG_
    	 //printf("%d:TCP client loopback start\r\n",sn);
         //printf("%d:Socket opened\r\n",sn);
#endif
         break;
      default:
#ifdef DEBUG2	
				printf("%d\r\n",getSn_SR(sn));
#endif
         break;
   }
   return 1;
}


void PhyLink_OFF_handle(uint8_t sn)
{
		close(sn);
		stateFlag.eth_scoket_state = getSn_SR(sn);
}

int32_t UTC_TcpC_Rev(uint8_t sn, uint8_t* revbuf,uint16_t *revsize)
{
	int32_t ret=0;
		 if((*revsize = getSn_RX_RSR(sn)) > 0) // Sn_RX_RSR: Socket n Received Size Register, Receiving data length
     {
			if(*revsize > DATA_BUF_SIZE) *revsize = DATA_BUF_SIZE; // DATA_BUF_SIZE means user defined buffer size (array)
			ret = recv(sn, revbuf, *revsize); // Data Receive process (H/W Rx socket buffer -> User's buffer)
			if(ret <= 0) return ret; // If the received data length <= 0, receive failed and process end
			*revsize = (uint16_t) ret;
			//send incoming ETH data by usart
			//revbuf[*revsize]=0x00;
		 }
	return ret;			 
}

int32_t UTC_TcpC_Send(uint8_t sn, uint8_t* sendbuf,uint16_t* sendsize)
{
	int32_t ret=0;
	uint16_t i = 0, j=0;
	if(*sendsize != 0)
			 {
				 ret = send(sn, sendbuf, *sendsize);
				if(ret < 0)
					{
						close(sn);
						return ret;
					}
				//从缓冲区中删除已传输的数据
					for(i=0,j=0;i<*sendsize;i++)
					{	
						if(i<ret)
							sendbuf[i]=0x00;
						else
						{
							sendbuf[j++]=sendbuf[i];
							sendbuf[i]=0x00;
						}
					}
					if((*sendsize - ret)>=0)
					*sendsize = *sendsize - ret;//subtract the length of the sent data
					else
					*sendsize =0;
			}	
	return ret;			 
}
#endif
