#ifndef _DYNAMIXEL_HEADER
#define _DYNAMIXEL_HEADER


#ifdef __cplusplus
extern "C" {
#endif

#define MAXNUM_TXPACKET     (65535)
#define MAXNUM_RXPACKET     (65535)
#define BROADCAST_ID        (0xFE)

#define PING_STATUS_LENGTH  (14)
#define MAX_ID              (252)

#define LATENCY_TIME        (16)    //ms    (USB2Serial Latency timer)

///////////////// utility for value ///////////////////////////
#define DXL_MAKEWORD(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b)     ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

////////////////// packet indexes //////////////////////////
enum PACKET_INDEX {
    PKT_HEADER0,
    PKT_HEADER1,
    PKT_HEADER2,
    PKT_RESERVED,
    PKT_ID,
    PKT_LENGTH_L,
    PKT_LENGTH_H,
    PKT_INSTRUCTION,
    PKT_PARAMETER
};

enum COMM_RESULT {
	COMM_TXSUCCESS,
	COMM_RXSUCCESS,
	COMM_TXFAIL,
	COMM_RXFAIL,
	COMM_TXERROR,
	COMM_RXWAITING,
	COMM_RXTIMEOUT,
	COMM_RXCORRUPT
};

enum INSTRUCTION {
	INST_PING           = 1,
	INST_READ           = 2,
	INST_WRITE          = 3,
	INST_REG_WRITE      = 4,
	INST_ACTION         = 5,
	INST_FACTORY_RESET  = 6,
	INST_REBOOT         = 8,
	INST_SYSTEM_WRITE   = 13,   // 0x0D
	INST_STATUS         = 85,   // 0x55
	INST_SYNC_READ      = 130,  // 0x82
	INST_SYNC_WRITE     = 131,  // 0x83
	INST_BULK_READ      = 146,  // 0x92
	INST_BULK_WRITE     = 147   // 0x93
};

enum ERROR_BIT {
    ERRBIT_VOLTAGE      = 1,
    ERRBIT_ANGLE        = 2,
    ERRBIT_OVERHEAT     = 4,
    ERRBIT_RANGE        = 8,
    ERRBIT_CHECKSUM     = 16,
    ERRBIT_OVERLOAD     = 32,
    ERRBIT_INSTRUCTION  = 64
};

typedef struct comm{
    void*           hComm;
    int             iBusUsing;
    float           fByteTransferTime;
    float           fPacketWaitTime;
    double          dPacketStartTime;
} SerialPort, *PSerialPort;

typedef struct ping_data {
    int iID;
    int iModelNo;
    int iFirmVer;
} PingData, *PPingData;

typedef struct data {
    int             iID;
    int             iStartAddr;
    int             iLength;
    int             iError;
    unsigned char*  pucTable;
} BulkData, *PBulkData;

////////// packet communication methods ///////////////////////
int  __stdcall dxl_tx_packet( PSerialPort comm, unsigned char *txpacket );
int  __stdcall dxl_rx_packet( PSerialPort comm, unsigned char *rxpacket );
int  __stdcall dxl_txrx_packet( PSerialPort comm, unsigned char *txpacket, unsigned char *rxpacket, int *error );

///////////// device control methods ////////////////////////
int  __stdcall dxl_initialize( PSerialPort comm, int devIndex, int baudnum );
void __stdcall dxl_terminate( PSerialPort comm );
int  __stdcall dxl_change_baudrate( PSerialPort comm, int baudnum );

//////////// high communication methods ///////////////////////
int  __stdcall dxl_ping( PSerialPort comm, int id, PPingData data, int *error );
int  __stdcall dxl_broadcast_ping( PSerialPort comm, PPingData arr_data, int *count );
int  __stdcall dxl_reboot( PSerialPort comm, int id, int *error );
int  __stdcall dxl_factory_reset( PSerialPort comm, int id, int option, int *error );

int  __stdcall dxl_read( PSerialPort comm, int id, int address, int length, unsigned char *data, int *error );
int  __stdcall dxl_read_byte( PSerialPort comm, int id, int address, int *value, int *error );
int  __stdcall dxl_read_word( PSerialPort comm, int id, int address, int *value, int *error );
int  __stdcall dxl_read_dword( PSerialPort comm, int id, int address, unsigned int *value, int *error );

int  __stdcall dxl_write( PSerialPort comm, int id, int address, int length, unsigned char *data, int *error );
int  __stdcall dxl_write_byte( PSerialPort comm, int id, int address, int value, int *error );
int  __stdcall dxl_write_word( PSerialPort comm, int id, int address, int value, int *error );
int  __stdcall dxl_write_dword( PSerialPort comm, int id, int address, unsigned int value, int *error );

int  __stdcall dxl_system_write( PSerialPort comm, int id, int address, int length, unsigned char *data, int *error );

int  __stdcall dxl_sync_write( PSerialPort comm, int start_addr, int data_length, unsigned char *param, int param_length);
// SyncRead??

int  __stdcall dxl_bulk_read( PSerialPort comm, unsigned char *param, int param_length, BulkData **rxdata);
int  __stdcall dxl_get_bulk_byte(PBulkData *data, int id, int addr, int *value);
int  __stdcall dxl_get_bulk_word(PBulkData *data, int id, int addr, int *value);
int  __stdcall dxl_get_bulk_dword(PBulkData *data, int id, int addr, unsigned int *value);
// BulkWrite


#ifdef __cplusplus
}
#endif

#endif