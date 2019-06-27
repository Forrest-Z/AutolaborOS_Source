//��Ϊʾ���Դ���,������Լ�ϵͳ��غ��������ʵ��޸�
//��Ҫ����Ϊ�ڴ��ڽ����жϺ�����,�Ӵ��ڽ������ݻ�������ÿ�ζ���һ���ֽ�ֱ��������Ϊ��,�������ݰ���ʽ�ж�����ͷ��CRC�����.
//CRCУ����ȷ���payload���ݰ��ڴ濽����ʽ��䵽sAHRSData�ṹ����,����sAHRSData��������һ������̬������
//����ֱ���Խṹ�����ķ�ʽ������Ӧ������.
//�й�LRESULT CSkylarkDlg::OnReceiveChar(WPARAM pPackage, LPARAM lParam),���е�pPackage�Ƚṹ����Ӧ�Ĵ������������й�,
//��������ʾ�⹦�ܶ�Ӧ�����Լ�����ϵͳ��Ӧ�ĺ�������.

#define DL_CLASS_MINIAHRS                               0x0F //��̬����������
#define DL_MINIAHRS_ATTITUDE_AND_SENSORS                0x01 //��̬����ID��

#define DATA_LINK_MAX_LENGTH  200   //datalink ���ݰ�payload�ֵ

//DL Read Return Err
#define DL_NO_ERR              0x00
#define DL_UNKNOW_MESSAGE      0x01
#define DL_CHECKSUM_ERR        0x02
#define DL_PAYLOAD_LENGTH_ERR  0x04

#pragma pack(1)
typedef struct 
{
  unsigned char      Flags;            //״̬λ���ݲ��ô���
  float       Euler[3];         // ����ŷ����
  float       Gyro[3];          //��У׼��,ȥbias��Ľ��ٶ�ֵ,����
  float       Acc[3];           //��У׼��,���ٶȵ�λg
  float       Mag[3];           //��У׼���ָ����ֵ,������
}sAHRSData;
#pragma pack()

//DL package
#pragma pack(1)
typedef struct 
{
  unsigned  char   Header1;  //'T'
  unsigned char   Header2;  //'M'
  unsigned char   Class;   
  unsigned char   ID;
  unsigned short   Length;
  unsigned char   Payload[DATA_LINK_MAX_LENGTH];     
  unsigned char   CheckSum[2]; 
}sDataLink;
#pragma pack()

sAHRSData AHRSData;//AHRS����ȫ�ֱ���
sDataLink DataLink;//�������ݽṹ



//1:�˺���ΪVC����Ӧ�Ĵ��ڽ����жϺ���.��Ҫ�ӽ��ջ�������������,Ȼ����״̬ת������������,���������ݰ���������.
//������Ϣ����
LRESULT CSkylarkDlg::OnReceiveChar(WPARAM pPackage, LPARAM lParam)
{
  int i = 0;
  int j = 0;
  int lenght = 0;
  unsigned short CheckSum;//16λCRCУ�����
  unsigned char *INT8UP;
  unsigned char Res;
  static unsigned char  DLUARTState= 0;//��ʼ��Ϊ�жϵ�һ������ͷ
  static unsigned int DLUartRDCnt=0; 
  
  pPACKAGE pPack = (pPACKAGE)pPackage;
  ASSERT(pPack);
  ASSERT(pPack->pData);
  
  lenght = pPack->iLen;
  Res=0;
  while(lenght > i)//ÿ�δ���һ���ֽ�,һֱ�����ڽ��ջ�������
  {
    switch( DLUARTState)
    {
    case  0:   //�жϵ�һ��ͷ�ֽ�'T'
      if(pPack->pData[i]=='T')  DLUARTState= 1;//��һ������ͷ����,�����жϵڶ�������ͷ״̬
      else  DLUARTState= 0;//��һ������ͷ����,���ص��жϵ�һ��������ͷ״̬
      break;   
    case  1://�жϵڶ���ͷ�ֽ�'M'
      if(pPack->pData[i]=='M')  DLUARTState= 2;//�ڶ�������ͷ��ȷ,�������class������״̬
      else  DLUARTState= 0; //�ڶ�������ͷ����,���ص��жϵ�һ����ͷ״̬
      break;
    case  2://����class��
      DataLink.Class=pPack->pData[i];
      DLUARTState= 3;    //�������ID��״̬ 
      break;
    case  3://����ID��
      DataLink.ID=pPack->pData[i];
      DLUARTState= 4; //����������ݳ��ȵ�һ�ֽ�״̬
      break;   
    case  4://���������ֽڵĳ���
      DataLink.Length=pPack->pData[i];
      DLUARTState= 5; //����������ݳ��ȵڶ��ֽ�״̬
      break;
    case  5://���յ������ֽڵĳ���
      DataLink.Length=DataLink.Length|(((unsigned short)(pPack->pData[i]))<<8);
      if(DataLink.Length> DATA_LINK_MAX_LENGTH)//���ݳ��ȳ�����󳤶�,�����ݰ��϶��Ǵ����
      {
        DLUARTState= 0; //���ص��жϵ�һ������ͷ״̬
        Res=DL_PAYLOAD_LENGTH_ERR;
      }
      else 
      {
        DLUARTState= 6; //����û������Χ,�������payload����״̬
        DLUartRDCnt=0;
      }
      break;
    case  6://������̬�ṹ����
      if(DataLink.Length==0)//û��payload���ݣ�ֱ�ӽ������У���ֽ�״̬
      {
        DLUARTState= 7; //������ռ����ֽ�״̬
        i--;
      }
      else
      {
        DataLink.Payload[ DLUartRDCnt]=pPack->pData[i];//�ѽ��յ���payload���ݷ���DataLink.Payload������
        DLUartRDCnt++;
        if( DLUartRDCnt==DataLink.Length )//����payload�������
        {
          DLUARTState= 7; //������ռ����ֽ�״̬
          DLUartRDCnt=0;
        }
      }        
      break;
    case  7://���յ�һ��У���ֽ�
      DataLink.CheckSum[0]=pPack->pData[i];
      DLUARTState= 8; //������յڶ���У���ֽ�״̬
      break;
    case  8://���յڶ���У���ֽڣ���ʾһ�����ݽ������
      DataLink.CheckSum[1]=pPack->pData[i];
      DLUARTState= 0; 
      INT8UP=(unsigned char*)&DataLink.Class;//ָ��ָ����ҪУ��У��͵ĵ�һ���ֽ���
      
      CheckSum=CRC16(INT8UP, DataLink.Length+4);//������յ�������CRCУ��,CRCУ�����������̰�
      if((CheckSum>>8==DataLink.CheckSum[1])&&((CheckSum&0x00ff)==DataLink.CheckSum[0]))//CRCУ����ȷ
      { 
        ������	switch(DataLink.Class)//��֤Class�Ƿ���ȷ
          ������	{
            ������	case DL_CLASS_MINIAHRS://Mini AHRS ��
              ����     �� switch(DataLink.ID)//��֤ID���Ƿ���ȷ
                ������		{
                  ������		case DL_MINIAHRS_ATTITUDE_AND_SENSORS:
                    ������			INT8UP=(unsigned char *) & AHRSData.Flags;//ȡ�׵�ַ
                    ������			for(i=0;i<DataLink.Length;i++)//���ݸ��µ��ṹ����
                      ������			{
                        ������				(*INT8UP)=DataLink.Payload[i];
                        ������				INT8UP++;
                        ������			}
                                              //����AHRSData�ṹ���м����������µ�һ����̬���ݣ�����������������Ӧ�ı���������ʾ����������
                    ������			 //printf��ʾ
                      ������			str.Format(_T("Roll:  %6.1f \r\n Pitch: %6.1f \r\n Yaw:   %6.1f \r\n"),AHRSData.Euler[0],AHRSData.Euler[1],AHRSData.Euler[2]);//��ʾ���Ͱ�ֵ
                                                pDlgAHRS->m_Attitude_Data=str;//�Ի���������¡���         
                                                pDlgAHRS->UpdateData(false);//���¶Ի������
                    ������                   break;
                    ������      }
                         break;
                       }

          }
        break;
      default:
        break;
      }
      
      i++;
    }
    
    pPack->Clear();
    delete pPack;
    
    return Res;
    
  }
  
  
  