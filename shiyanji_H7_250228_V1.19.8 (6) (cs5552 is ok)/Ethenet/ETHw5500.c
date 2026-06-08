#include "ETHw5500.h"
#include "usart.h"
#include "EthProtocol.h"
#include "gParameter.h"
#include <elog.h>

//#define  fifo_debug_send

UTC_ETH utc_eth;

uint8_t gDATABUF[DATA_BUF_SIZE];
wiz_NetInfo gWIZNETINFO = { .mac = {0x00, 0x08, 0xdc,0x00, 0xab, 0xcd},
                            .ip = {192, 168, 0, 11},
                            .sn = {255,255,255,0},
                            .gw = {192, 168, 0, 1},
                            .dns = {8,8,8,8},
                            .dhcp = NETINFO_STATIC};
uint8_t memsize[16] = { 2, 2, 2, 2, 2, 2, 2, 2, // 8 channel tx
			2, 2, 2, 2, 2, 2, 2, 2 }; // 8 channel rx
uint8_t tmp;
int8_t ret;
uint8_t dest_ip[4] = {192, 168, 0, 10};
uint16_t dest_port = 5001;		

void network_init(void);
void W5500_Restart(void) ;

void networkParamConfigInit(void){
	uint8_t i = 0;
	//MAC
	for(i = 0;i < 6;i++){
		gWIZNETINFO.mac[i] = ethConfig.LocalMac[i];
	}
	//IP
	for(i = 0;i < 4;i++){
		gWIZNETINFO.ip[i] = ethConfig.LocalIp[i];
	}
	//ServerIP
	for(i = 0;i < 4;i++){
		dest_ip[i] = ethConfig.ServerIp[i];
	}
	//ServerPort
	dest_port = ethConfig.ServerPort;
	//utc sn & mode
	utc_eth.sn = 0;//使用SOCK0通道
	if(ethConfig.EthMode == TCP || ethConfig.EthMode == UDP)
		utc_eth.Mode = ethConfig.EthMode;//设置为TCP客户端模式***
	else
		utc_eth.Mode = TCP;
}

//网络初始化（DHCP）
void network_init(void)
{
	uint8_t tmpstr[6] = {0,};
	wiz_NetInfo netinfo;
 
	// Set Network information from netinfo structure
	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);
 
	// Get Network information
	ctlnetwork(CN_GET_NETINFO, (void*)&netinfo);
 
	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);
 
	if(netinfo.dhcp == NETINFO_DHCP) printf("\r\n=== %s NET CONF : DHCP ===\r\n",(char*)tmpstr);
	else printf("\r\n=== %s NET CONF : Static ===\r\n",(char*)tmpstr);
 
	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",netinfo.mac[0],netinfo.mac[1],netinfo.mac[2],
			netinfo.mac[3],netinfo.mac[4],netinfo.mac[5]);
	printf("SIP: %d.%d.%d.%d\r\n", netinfo.ip[0],netinfo.ip[1],netinfo.ip[2],netinfo.ip[3]);
	printf("GAR: %d.%d.%d.%d\r\n", netinfo.gw[0],netinfo.gw[1],netinfo.gw[2],netinfo.gw[3]);
	printf("SUB: %d.%d.%d.%d\r\n", netinfo.sn[0],netinfo.sn[1],netinfo.sn[2],netinfo.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", netinfo.dns[0],netinfo.dns[1],netinfo.dns[2],netinfo.dns[3]);
	printf("===========================\r\n");
 
}

void W5500_Restart(void)
{
	HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST, GPIO_PIN_RESET);
	HAL_Delay(1);  // delay 1ms
	HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST, GPIO_PIN_SET);
	HAL_Delay(1600);  // delay 1600ms
}
/**
 * @note Fun_1811
 * @brief ethernet init
*/
void ETHw5500_init(void)
{
	reg_wizchip_cris_cbfunc(NULL, NULL); // 注册临界区函数
	reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);// 注册片选函数
	/* SPI Read & Write callback function */
	reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte);	//注册读写函数
	W5500_Restart();			//w5500启动前要restart，不然无法配置相关网络参数
	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
		 printf("WIZCHIP Initialized fail.\r\n");  //设置对应故障码
	}
	/* Network initialization */
	networkParamConfigInit();
	network_init();
		printf("network_init pass\r\n");
	
}

void ETHw5500_Looptest(void)
{
		//TCP服务器回环测试
		if( (ret = loopback_tcps(SOCK_TCPS, gDATABUF, 5000)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
		//TCP客户端回环测试
		if( (ret = loopback_tcpc(SOCK_TCPC, gDATABUF, dest_ip, dest_port)) < 0) {
			printf("SOCKET ERROR : %d\r\n", ret);
		}
		
		//UDP回环测试
		if( (ret = loopback_udps(SOCK_UDPS, gDATABUF, 6000)) < 0) {
			printf("SOCKET ERROR1 : %d\r\n", ret);
		}
		
		if( (ret = loopback_udps(SOCK_UDPS3, gDATABUF, 6001)) < 0) {
			printf("SOCKET ERROR1 : %d\r\n", ret);
		}
		
		if( (ret = loopback_udps(SOCK_UDPS4, gDATABUF, 6002)) < 0) {
			printf("SOCKET ERROR1 : %d\r\n", ret);
		}
}

/* Fun_182 */
void ETHw5500_Func(void)
{
	// uint8_t i=0;
	// uint8_t recordStep = 0;
#ifdef  fifo_debug_send
	uint8_t k=0,l=0;
#endif
	//物理层检测
		/* PHY link status check */
		 if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1)
		{
			stateFlag.eth_phy_state = PHY_LINK_UNKNOW;
		 }
		else if(tmp == PHY_LINK_OFF)
		{
			stateFlag.eth_phy_state = PHY_LINK_OFF;
			PhyLink_OFF_handle(utc_eth.sn);//物理层断开检测
		}
		else
		{
			stateFlag.eth_phy_state = PHY_LINK_ON;
			//TCP服务器 接收与发送服务
			//UTC作为TCP客户端 连接状态更新
			if( (ret = utc_tcpc(utc_eth.sn,dest_ip, dest_port)) < 0) 
				log_e("UTC_ETH SOCKET ERROR : %d", ret);
			if(stateFlag.eth_scoket_state == SOCK_ESTABLISHED)
			{
				//从socket接受数据帧
				UTC_TcpC_Rev(utc_eth.sn,utc_eth.Revbuf,&utc_eth.revsize);
				//数据帧拆包分析
				UTC_revbuf_process(&utc_eth);
				while(send_fifo.state != FIFO_EMPTY)
				{
#ifdef fifo_debug_send
							for(k=0;k<SEND_FIFO_SIZE;k++)
									{
										printf("FifoSend %d  ",k);
											for(l=0;l<50;l++)
											printf("%2x",send_fifo_data[k][l]);
										printf("\r\n");
									}
									printf("\r\n \r\n");
#endif
					//发送队列出队
					send_ring_fifo_pop(buf_data_send);
					//发送数据帧打包
					ret = UTC_sendbuf_process(buf_data_send,&utc_eth);
				}
				//给socket发送数据帧
				UTC_TcpC_Send(utc_eth.sn,utc_eth.Sendbuf,&utc_eth.sendsize);
			}
		}
		
	 ETH_StatePrintf();//网络状态更新显示
}

void ETH_StatePrintf(void)
{	
	static uint8_t first_in=0;
	//PHY
	if(stateFlag.eth_phy_state != stateFlagLast.eth_phy_state || first_in == 0)
	{
		switch(stateFlag.eth_phy_state)
		{
			case PHY_LINK_OFF:
		printf("Eth_phy_state:PHY_LINK_OFF\r\n");
			break;
			case PHY_LINK_ON:
		printf("Eth_phy_state:PHY_LINK_ON\r\n");		
			break;
			case PHY_LINK_UNKNOW:
		printf("Eth_phy_state:PHY_LINK_UNKNOW\r\n");		
			break;
			default:
				break;
		}
	}
	stateFlagLast.eth_phy_state = stateFlag.eth_phy_state;
	//SOCKET
	if(stateFlag.eth_scoket_state != stateFlagLast.eth_scoket_state || first_in == 0)
	{
		switch(stateFlag.eth_scoket_state)
		{
			case SOCK_CLOSED:
			printf("Eth_scoket_state:SOCK_CLOSED\r\n");	
				break;
			case SOCK_INIT:
			printf("Eth_scoket_state:SOCK_INIT\r\n");		
				break;
			case SOCK_ESTABLISHED:
			printf("Eth_scoket_state:SOCK_ESTABLISHED\r\n");		
				break;
			case SOCK_CLOSE_WAIT:
			printf("Eth_scoket_state:SOCK_CLOSE_WAIT\r\n");		
				break;
			case SOCK_SYNSENT:
			printf("Eth_scoket_state:SOCK_SYNSENT\r\n");		
				break;
			default:
			printf("Eth_scoket_state others:%2x\r\n",stateFlag.eth_scoket_state);	
				break;
		}
		
	}
	stateFlagLast.eth_scoket_state = stateFlag.eth_scoket_state;
	if(first_in == 0) first_in++;
	
}

void Eth_test(void)
{
/*回环测试*/
//  	uint16_t i=0;
//	  int8_t ret=0;
//  for(i=0;i<utc_eth.revsize;i++)
//		utc_eth.Sendbuf[i]=utc_eth.Revbuf[i];
//		utc_eth.sendsize=utc_eth.revsize;
	
//流量测试
//		utc_eth.sendsize = 2048;
//    for(i=0;i<utc_eth.sendsize/2;i++)
//			{
//				utc_eth.Sendbuf[i*2]=(uint8_t)(i>>8);
//				utc_eth.Sendbuf[i*2+1]=(uint8_t)i;
//			}
}


