
// SANYO DENKI Power Conditioner Monitor
// 2021/2/16
// 


//#include <EthernetServer.h>
#include <Ethernet.h>
#include <MsTimer2.h>
#include <SD.h>
#include "sd_read_write.h"          //SDReadWriteの読み込み
#include <avr/wdt.h>

#include <ArduinoJson.h>

#define PWCON_MAX 10
#define SIZE_OF_ARRAY(ary)  (sizeof(ary)/sizeof((ary)[0]))
#define ACK 0x06
#define NAK 0x15

#define IP_ADDRESS  0
#define SUBNETMASK  1
#define GATEWAY  2
#define MAC_ADDRESS  3
#define SERVER_ADDRESS  4
#define SERVER_PORT  5
#define POWER_PLANT_NUMBER  6
#define INTERBAL_TIME  7
#define SENSOR_STATUS 8


//INPUT/OUTPUT PIN Configration
#
#define PIN_STATUS_LED 13

SDReadWrite configFile("\r\n");             //SDReadWriteのインスタンスを生成（改行コードを引数に指定）
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xF0, 0x0D };    //インサーネットMACアドレスデフォル値
byte ip[] = { 192, 168, 3, 88 };                        //IP Address Default Value
byte dnsserver[] = { 192, 168, 3, 1 };                  //DNS Server Address
byte gateway[] = { 192, 168, 3, 1 };                    //Gateway Address
byte subnet[] = { 255, 255, 255, 0 };                   //Subnet MASK
char server[] = "122.222.157.190"; // Google
char serverAdr[20];
uint16_t  serverPort = 8082;       //初期値ポート番号を設定

String serverHost = "Host: 122.222.157.190";

char serialBuffer[1024];

typedef struct {
  char  stx;
  char  header[2];
  char  send_num[2];
  char  command;
  char  parameter;
    
  char  opelate1[2];                //運転状態１
  char  split1;
  char  systemErrorStatus1[2];      //系統異常状態1
  char  split2;
  char  systemErrorStatus2[2];      //系統異常状態2
  char  split3;
  char  failTrans[2];               //通信異常
  char  split4;
  char  fail1[2];                   //故障状態1
  char  split5;
  char  fail2[2];                   //故障状態2
  char  split6;
  char  fail3[2];                   //故障状態3
  char  split7;
  char  extStatus[2];               //外部ステータス
  char  split8;
  char  setScale[2];                //スケール設定
  char  split9;
  char  dcVoltage[4];                //直流電圧
  char  split10;
  char  dcAmpere[6];                 //直流電流
  char  split11;
  char  dcPower[6];                  //直流電力
  char  split12;
  char  acVoltage[4];                //交流電圧
  char  split13;
  char  acAmpere[5];                 //交流電流
  char  split14;
  char  acPower[6];                  //交流電力
  char  split15;
  char  frequency[3];                //周波数
  char  split16;
  char  singleTotalGenerate[6];        //単機積算発電量
  char  split17;
  char  singleTotalCharge[6];          //単機積算充電量
  char  split18;
  char  totalAcPowerPo[5];           //総合交流電力正
  char  split19;
  char  totalAcPowerNe[5];           //総合交流電力負
  char  split20;
  char  totalGenerate[6];              //総合積算発電量
  char  split21;
  char  totalCharge[6];                //総合積算充電量
  char  split22;
  char  cellVoltage[4];              //太陽電池電圧
  char  split23;
  char  cellAmpere[5];               //太陽電池電流
  char  split24;
  char  cellDischargeA[5];           //蓄電池放電電流
  char  split25;
  char  solarIrradiance[4];          //日射量
  char  split26;
  char  outTemperature[4];           //温度
  char  split27;
  char  solarIrradianceRsv[4];       //日射強度予備
  char  split28;
  char  temperatureRsv[5];           //温度予備
  char  split29;
  char  resv1[4];                    //予備
  char  split30;
  char  resv2[4];                    //予備
  char etx;
  char bcc;
} power_cont;

typedef struct {
  char  adr_header[2];
  char  send_num[2];
  char  command;
  char  parameter;
  uint8_t	opelate1;                //運転状態１
  uint8_t	systemErrorStatus1;      //系統異常状態1
  uint8_t	systemErrorStatus2;      //系統異常状態2
  uint8_t  failTrans;               //通信異常
  uint8_t  fail1;                   //故障状態1
  uint8_t  fail2;                   //故障状態2
  uint8_t  fail3;                   //故障状態3
  uint8_t  extStatus;               //外部ステータス
  float			dcVoltage;                //直流電圧
  float			dcAmpere;                 //直流電流
  float			dcPower;                  //直流電力
  float			acVoltage;                //交流電圧
  float			acAmpere;                 //交流電流
  float			acPower;                  //交流電力
  float			frequency;                //周波数
  float			solarIrradiance;          //日射量
  float			outTemperature;           //温度
  float			counterAveraging;			//平均化カウンター
  int8_t		recvFlag;
} power_con_real;



EthernetClient client;

power_cont SER_BUF;
power_con_real pw_con[PWCON_MAX];  //パワコンの台数分用意する。
//char rxData[128][PWCON_MAX];

#define SERIAL_IDLE 0
#define SERIAL_START 1
#define SERIAL_END 2
#define SERIAL_BCC 3
int8_t serialState = SERIAL_IDLE;



const int rcvTimerValue = 2;
int keyMode = 1;                    //Elevator MODE=1:Room MODE=2:SW MODE=3
const int chipSelect = 4;           //SD CARD Chipselect Pin Config
int8_t httpFlag = 0;
int16_t _sendount = 0;
int8_t totalPwconNum = 6;
int16_t timerCounter = 0;
int16_t intervalTimer = 120;        //120S
int16_t powerPlantNumber = 0;
int8_t sensorStatus = 0;
int8_t recvMonitorFlag = 0;
int8_t recvMonitorCounter = 0;
int8_t httpNotReqestCounter = 0;
int8_t httpErrorCounter = 0;
int8_t httpConnectionFailCounter = 0;

//const size_t capacity = JSON_OBJECT_SIZE(17) + 270;
//DynamicJsonDocument _pv_monitor_json(capacity);

//BCCの計算
uint8_t bccx(char *bufData, int sizeData)
{
	int16_t i;
	uint32_t bcc;
	uint8_t bcc8;

	bcc = 0x00000000;
	for (i = 0; i < sizeData; i++)
	{
		//Console.WriteLine("char deta =[{0}]", bufData[i]);
		bcc += (uint32_t)bufData[i];

	}

	bcc = bcc & 0x000000ff;
	bcc8 = (uint8_t)bcc;
	return bcc8;
}


//ASCIIから10進数へ
float ascii_dec(char *char_data, int16_t size_data)
{
	int i;
	float x = 0;
	float base[11] = { 0,1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000 };
	for (i = 0; i < size_data; i++)
	{
		x = x + (float)(char_data[i] - 0x30) * base[size_data - i];
	}
	return x;
}
//符号付きASCIIから符号付き10進数へ
float ascii_dec_code(char *char_data, int16_t size_data)
{
	int16_t i;
	float x = 0;
	float base[12] = { 1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000 };
	for (i = 1; i < size_data; i++)
	{
		x = x + (float)(char_data[i] - 0x30) * base[size_data - (i + 1)];
		//printf("char=%02X %f",char_data[i],base[size_data - (i + 1)]);
	}
	if (char_data[0] == '-')
	{
		x = x * -1;
	}
	return x;
}


//debug print
void debug_print(String p, int n)
{
	Serial.print(p);
	Serial.print(n);
	Serial.println();
}

//ASCII -> HEX
unsigned char conv_2(const char s[2])
{
	int i;
	unsigned char c, v = 0;
	for (i = 0; i < 2; i++) {
		c = (unsigned char)s[i];
		v = (c & 0x10 ? c : c + 9) & 0x0f | (v << 4);
	}

	return v;
}
//BCCの計算
char bcc(char *bufData, int sizeData)
{
	int i;
	char bcc;

	bcc = 0x00;
	for (i = 1; i < sizeData; i++)
	{
		//Console.WriteLine("char deta =[{0}]", bufData[i]);
		bcc ^= bufData[i];

	}

	bcc = bcc & 0x00ff;
	return bcc;
}


//------------------------------------------------------------
//
//------------------------------------------------------------
void get_serial()
{
	int16_t bufferCounter = 0;
	char* _serialBuffer;
	char _bcc_buf[2];   //text BCCに対応するためのバッファ
	int8_t _bcc_flag = 0;
	uint32_t _recvCouner = 0;
	int16_t _recvOverCount = 0;

	for (_sendount = 0; _sendount < totalPwconNum; _sendount++)
	{
		bufferCounter = 0;
		_recvCouner = 0;
		_recvOverCount = 0;
		while (true)
		{
			_recvCouner++;
			if (_recvCouner > 1000)
			{
				break;
			}
			if (Serial1.available() > 0) 
			{
				_recvCouner = 0;
				recvMonitorCounter = 0;
				recvMonitorFlag = 0;
				char inData = Serial1.read();
				//Serial.print(inData);
				if (serialState == SERIAL_IDLE)
				{
					if (inData == 0x02)
					{
						serialBuffer[bufferCounter] = inData;
						bufferCounter++;
						serialState = SERIAL_END;
					}
				}
        		else if (serialState == SERIAL_END)
        		{
          			serialBuffer[bufferCounter] = inData;
          			bufferCounter++;
          			if (inData == 0x03)
          			{
            			serialState = SERIAL_BCC;
            			//Serial.println("");
            			//Serial.println(bufferCounter);
            			//Serial.println(_sendount);
          			}
        		}
        		else if (serialState == SERIAL_BCC)
        		{
					//Serial.println("BCC");
          			serialBuffer[bufferCounter] = inData;
					bufferCounter++;
					serialState = SERIAL_IDLE;
					break;
      			}
			}
			//delay(1);
		}
		if (bufferCounter > 50)
		{
    		//Serial.println(serialBuffer);
			uint8_t _out_bcc = (uint8_t)bcc(serialBuffer, bufferCounter - 1);      //BCCコードの計算
			//_bcc_buf[1] = serialBuffer[bufferCounter - 2];
			//_bcc_buf[0] = serialBuffer[bufferCounter - 3];
			uint8_t _bcc_rscv = (uint8_t)serialBuffer[bufferCounter - 1];
			//Serial.println(_out_bcc,HEX);
			//Serial.println(_bcc_rscv,HEX);
			if (_bcc_rscv == _out_bcc)      //BCCコードのチェック
			{
				memcpy(&SER_BUF, serialBuffer, bufferCounter);      //シリアルデータを構造体にコピー
				_bcc_flag = 1;
        		//Serial.println("Recv Data OK");
			}
     		else
     		{
        		Serial.println("Recv Data NG");
     		}

			if ((SER_BUF.command == 'A') & (SER_BUF.parameter == 'a')  & (_bcc_flag == 1))
			{
				
				int _pwconNumber = (int)ascii_dec(SER_BUF.send_num, 2) - 1;    //パワコン番号取り出し
				Serial.println(_pwconNumber);
				pw_con[_pwconNumber].recvFlag = 0;
				pw_con[_pwconNumber].opelate1 = conv_2(SER_BUF.opelate1);
				pw_con[_pwconNumber].systemErrorStatus1 = conv_2(SER_BUF.systemErrorStatus1);
				pw_con[_pwconNumber].systemErrorStatus2 = conv_2(SER_BUF.systemErrorStatus2);
				pw_con[_pwconNumber].failTrans = conv_2(SER_BUF.failTrans);
				pw_con[_pwconNumber].fail1 = conv_2(SER_BUF.fail1);
				pw_con[_pwconNumber].fail2 = conv_2(SER_BUF.fail2);
				pw_con[_pwconNumber].fail3 = conv_2(SER_BUF.fail3);
				pw_con[_pwconNumber].extStatus = conv_2(SER_BUF.extStatus);
				pw_con[_pwconNumber].dcVoltage += (ascii_dec(SER_BUF.dcVoltage,4) * 0.1);
				pw_con[_pwconNumber].dcAmpere += (ascii_dec_code(SER_BUF.dcAmpere,6) * 0.01);
				pw_con[_pwconNumber].dcPower += (ascii_dec_code(SER_BUF.dcPower,6) * 0.01);
				pw_con[_pwconNumber].acVoltage += (ascii_dec(SER_BUF.acVoltage,4) * 0.1);
				pw_con[_pwconNumber].acAmpere += (ascii_dec(SER_BUF.acAmpere,5) * 0.01);
				pw_con[_pwconNumber].acPower += (ascii_dec_code(SER_BUF.acPower,6) * 0.01);
				pw_con[_pwconNumber].frequency += (ascii_dec(SER_BUF.frequency,3) * 0.1);
				pw_con[_pwconNumber].counterAveraging++;

				//Serial.printf("PWNUM=[%d] ACP=[%f] DCP=[%f]\r\n",_pwconNumber,ascii_dec_code(SER_BUF.dcPower,6) * 0.01,)

				if (sensorStatus == 0)
				{
					pw_con[_pwconNumber].solarIrradiance = 0.0;
					pw_con[_pwconNumber].outTemperature = 0.0;
				}
				else
				{
					pw_con[_pwconNumber].solarIrradiance += (ascii_dec(SER_BUF.solarIrradiance,4) * 0.001);
					pw_con[_pwconNumber].outTemperature += (ascii_dec(SER_BUF.outTemperature,4) * 0.001);
				}

			}
		}
	}
}

void init_data()
{
	for (int8_t _iii = 0; _iii < PWCON_MAX; _iii++)
	{	
		pw_con[_iii].recvFlag = 0;
		pw_con[_iii].opelate1 = 0;
		pw_con[_iii].systemErrorStatus1 = 0;
		pw_con[_iii].systemErrorStatus2 = 0;
		pw_con[_iii].failTrans = 0;
		pw_con[_iii].fail1 = 0;
		pw_con[_iii].fail2 = 0;
		pw_con[_iii].fail3 = 0;
		pw_con[_iii].extStatus = 0;
		pw_con[_iii].dcVoltage = 0;
		pw_con[_iii].dcAmpere = 0;
		pw_con[_iii].dcPower = 0;
		pw_con[_iii].acVoltage = 0;
		pw_con[_iii].acAmpere = 0;
		pw_con[_iii].acPower = 0;
		pw_con[_iii].dcPower = 0;
		pw_con[_iii].acVoltage = 0;
		pw_con[_iii].acAmpere = 0;
		pw_con[_iii].acPower = 0;
		pw_con[_iii].frequency = 0;
		pw_con[_iii].counterAveraging = 0;
	}

}

void timerFire() {
	//mySerial.print("*");
	//100ms毎にここが呼び出される
	//Serial.print(".");
	wdt_reset();
	timerCounter++;
	if (timerCounter > intervalTimer)
	{
		timerCounter = 0;
		httpFlag = 1;
	}
	recvMonitorCounter++;
	if (recvMonitorCounter > 10)
	{
		recvMonitorFlag = 1;
		recvMonitorCounter = 99;
	}

}

int split(String *result, size_t resultsize, String data, char delimiter) {
	unsigned int index = 0;
	int datalength = data.length();
	for (int i = 0; i < datalength; i++) {
		char tmp = data.charAt(i);
		if (tmp == delimiter) {
			index++;
			if (index > (resultsize - 1)) return -1;
		}
		else result[index] += tmp;
	}
	return (index + 1);
}



void setup() {
	String configArry[100];
	//char bccx[3];
	//char bccdata;

	pinMode(PIN_STATUS_LED, OUTPUT);
	digitalWrite(PIN_STATUS_LED, LOW);
	//pinMode( 59, OUTPUT );
	//digitalWrite( 59, LOW );
	pinMode(12, OUTPUT);
	digitalWrite(12, LOW);

	
	Serial.begin(115200);        //Monitor Serial
	while (!Serial);
	Serial1.begin(19200);        //pwcon Serial
	while (!Serial1);

	delay(2000);
	Serial.println("Tabuchi Pwcon Monitor Ver1.0 2020.12.06");
	//--------------------------------------------------
	//SD Cardから設定情報を読み取る
	//Door Key Open Timer S
	//IP Address
	//Subnet MASK
	//Gateway
	//--------------------------------------------------
	Serial.println(F("ReadStart!"));
	if (configFile.init() == -1) {
		Serial.println(F("initialization failed!"));
	}
	else {
		int lineCount = 0;
		configFile.fileOpen("config.txt", 1);        //ファイルを開く
		while (1) {
			String str;
			str = configFile.readLine();        //１行読み込む（改行コードは含まれない）
			if (str == "\e")break;      //エスケープ文字の場合は終了
			Serial.print(lineCount);
			configArry[lineCount] = str;
			Serial.print("[");
			Serial.print(configArry[lineCount]);       //読み取った文字列を表示 
			Serial.println("]");
			lineCount++;
			if (lineCount >= 30) break;
		}

		configFile.fileClose();                 //ファイルを閉じる
		Serial.println(F("Complete!"));


		//IP ADDRESSの設定
		String splitstring[10] = { "\0" };                        //分割された文字列を格納する配列
		size_t arraysize = SIZE_OF_ARRAY(splitstring);          //配列の要素数
		char delimiter = '.';  //区切り文字
		//int index = split(splitstring, arraysize, configArry[IP_ADDRESS], delimiter);
		split(splitstring, arraysize, configArry[IP_ADDRESS], delimiter);
		ip[0] = splitstring[0].toInt();
		ip[1] = splitstring[1].toInt();
		ip[2] = splitstring[2].toInt();
		ip[3] = splitstring[3].toInt();

		//SUBNET MASKの設定
		splitstring[0] = { "\0" };
		splitstring[1] = { "\0" };
		splitstring[2] = { "\0" };
		splitstring[3] = { "\0" };
		split(splitstring, arraysize, configArry[SUBNETMASK], delimiter);
		subnet[0] = splitstring[0].toInt();
		subnet[1] = splitstring[1].toInt();
		subnet[2] = splitstring[2].toInt();
		subnet[3] = splitstring[3].toInt();
		//Serial.println(subnet[0]);
		//Serial.println(subnet[1]);
		//Serial.println(subnet[2]);
		//Serial.println(subnet[3]);

		//GATEWAYの設定
		splitstring[0] = { "\0" };
		splitstring[1] = { "\0" };
		splitstring[2] = { "\0" };
		splitstring[3] = { "\0" };
		split(splitstring, arraysize, configArry[GATEWAY], delimiter);
		gateway[0] = splitstring[0].toInt();
		gateway[1] = splitstring[1].toInt();
		gateway[2] = splitstring[2].toInt();
		gateway[3] = splitstring[3].toInt();
		//Serial.println(gateway[0]);
		//Serial.println(gateway[1]);
		//Serial.println(gateway[2]);
		//Serial.println(gateway[3]);

		//MAC ADDRESSの設定
		splitstring[0] = { "\0" };
		splitstring[1] = { "\0" };
		splitstring[2] = { "\0" };
		splitstring[3] = { "\0" };
		splitstring[4] = { "\0" };
		splitstring[5] = { "\0" };
		split(splitstring, arraysize, configArry[MAC_ADDRESS], delimiter);
		mac[0] = splitstring[0].toInt();
		mac[1] = splitstring[1].toInt();
		mac[2] = splitstring[2].toInt();
		mac[3] = splitstring[3].toInt();
		mac[4] = splitstring[4].toInt();
		mac[5] = splitstring[5].toInt();
		Serial.println(splitstring[0]);
		Serial.println(splitstring[1]);
		Serial.println(splitstring[2]);
		Serial.println(splitstring[3]);
		Serial.println(splitstring[4]);
		Serial.println(splitstring[5]);

		Serial.println(mac[0]);
		Serial.println(mac[1]);
		Serial.println(mac[2]);
		Serial.println(mac[3]);
		Serial.println(mac[4]);
		Serial.println(mac[5]);


		//SERVER ADDRESSの設定

		String _serverAddress = configArry[SERVER_ADDRESS];
		_serverAddress.toCharArray(serverAdr, _serverAddress.length()+1);
		Serial.print("SERVER ADDRESS:");
		Serial.println(serverAdr);
		serverHost = "Host: " + _serverAddress;
		Serial.print("SERVER HOST:");
		Serial.println(serverHost);

		serverPort = (uint16_t)configArry[SERVER_PORT].toInt(); //SERVER PORT Number

		intervalTimer = configArry[INTERBAL_TIME].toInt(); //SERVER PORT Number
		debug_print("INTERBAL TIMER:", intervalTimer);
		powerPlantNumber = configArry[POWER_PLANT_NUMBER].toInt(); //SERVER PORT Number
		debug_print("POWER PLANT NUMBER:", powerPlantNumber);
		sensorStatus = configArry[SENSOR_STATUS].toInt(); //SERVER PORT Number
		debug_print("SENSOR_STATU:", sensorStatus);

	}




	Ethernet.begin(mac, ip, dnsserver, gateway, subnet);
	//client.begin();
	delay(2000);

	Serial.print("My IP address: ");
	for (byte thisByte = 0; thisByte < 4; thisByte++) {
		// print the value of each byte of the IP address:
		Serial.print(Ethernet.localIP()[thisByte], DEC);
		Serial.print(".");
	}
	Serial.println();
	init_data();



	//--------------------------------------------------
	//状態通知送信
	//--------------------------------------------------
	  //state_packet();
	wdt_enable(WDTO_8S);
	MsTimer2::set(1000, timerFire);       //Timer Config
	MsTimer2::start();                    //Start Timer

}

void loop() {


	get_serial();

	if (httpFlag == 1)
	{
		httpFlag = 0;
		for (int8_t _pwconNumber = 0; _pwconNumber < totalPwconNum; _pwconNumber++)
		{
			
			const size_t capacity = JSON_OBJECT_SIZE(18) + 290;
			DynamicJsonDocument _pv_monitor_json(capacity);
			_pv_monitor_json["pp_num"] = String(powerPlantNumber);
			_pv_monitor_json["pwconNum"] = String(_pwconNumber + 1);
			_pv_monitor_json["opelate1"] = String(pw_con[_pwconNumber].opelate1);
			_pv_monitor_json["systemErrorStatus1"] = String(pw_con[_pwconNumber].systemErrorStatus1);
			_pv_monitor_json["systemErrorStatus2"] = String(pw_con[_pwconNumber].systemErrorStatus2);
			_pv_monitor_json["failTrans"] = String(pw_con[_pwconNumber].failTrans);
			_pv_monitor_json["fail1"] = String(pw_con[_pwconNumber].fail1);
			_pv_monitor_json["fail2"] = String(pw_con[_pwconNumber].fail2);
			_pv_monitor_json["fail3"] = String(pw_con[_pwconNumber].fail3);
			_pv_monitor_json["extStatus"] = String(pw_con[_pwconNumber].extStatus);
			_pv_monitor_json["dcVoltage"] = String(pw_con[_pwconNumber].dcVoltage / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["dcAmpere"] = String(pw_con[_pwconNumber].dcAmpere / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["dcPower"] = String(pw_con[_pwconNumber].dcPower / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["acVoltage"] = String(pw_con[_pwconNumber].acVoltage / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["acAmpere"] = String(pw_con[_pwconNumber].acAmpere / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["acPower"] = String(pw_con[_pwconNumber].acPower / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["frequency"] = String(pw_con[_pwconNumber].frequency / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["solarIrradiance"] = String(pw_con[_pwconNumber].solarIrradiance / pw_con[_pwconNumber].counterAveraging);
			_pv_monitor_json["outTemperature"] = String(pw_con[_pwconNumber].outTemperature / pw_con[_pwconNumber].counterAveraging);
			
			if (recvMonitorFlag == 1)
			{
				_pv_monitor_json["monitor"] = "1";
			}
			else
			{
				_pv_monitor_json["monitor"] = String(pw_con[_pwconNumber].recvFlag);
			}
			
			// not used in this example
			Serial.println(F("Connecting..."));

			// Connect to HTTP server
			EthernetClient client;
			client.setTimeout(10000);
			if (!client.connect(serverAdr, serverPort)) {
				Serial.println(F("Connection failed"));
				httpConnectionFailCounter++;
				Serial.println(httpConnectionFailCounter);
				delay(5000);
				if (httpConnectionFailCounter > 5)
				{
					MsTimer2::stop();
					while (1);
				}
				return;
			}
			else
			{
				httpConnectionFailCounter = 0;
			}
			Serial.println(F("Connected!"));
			// Send HTTP request
			client.println(F("POST /pv_moni_sanyo.py HTTP/1.0"));
			client.println(F("User-Agent: Arduino/1.0"));
			client.println(F("Content-Type: application/json;charset=UTF-8"));
			client.println(serverHost);
			client.println(F("Connection: close"));
			client.print(F("Content-Length: "));
			client.println(measureJson(_pv_monitor_json));
			client.println();
			serializeJson(_pv_monitor_json, client);
			if (client.println() == 0) {
				Serial.println(F("Failed to send request"));
				httpNotReqestCounter++;
				Serial.println(httpNotReqestCounter);
				delay(5000);
			}
			else
			{
				httpNotReqestCounter = 0;
			}
			// Check HTTP status
			char status[32] = { 0 };
			client.readBytesUntil('\r', status, sizeof(status));
			// It should be "HTTP/1.0 200 OK" or "HTTP/1.1 200 OK"
			if (strcmp(status + 9, "200 OK") != 0) {
				Serial.print(F("Unexpected response: "));
				Serial.println(status);
				httpErrorCounter++;
				Serial.println(httpErrorCounter);
				delay(60000);
			}
			else
			{
				httpErrorCounter = 0;
			}
			//http error reset
			if ((httpConnectionFailCounter > 5 ) | (httpNotReqestCounter > 5) | (httpErrorCounter > 5))
			{
				MsTimer2::stop();
				while(1);
			}

			Serial.println(F("Connected End!"));
			client.stop();

			delay(100);
		}
		init_data();
		
	}

	//Serial.println(F("Loop End!"));
}
