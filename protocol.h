#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>
#include <QByteArray>
#include "lconnectHardware.h"
#define IS_DEBUG 1

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif


#define TURBO_RELEASED      0
#define TURBO_TEMP_PRESSED	1
#define TURBO_PRESSING			2
#define TURBO_TEMP_RELEASED 3


/** DEVICE IMFORMATION **/
#define PROTOCOL_DATA             0x00
#define PROTOCOL_CMD			  0x10

#define DEVICE_ID_MASTER		  0x01
#define DEVICE_ID_PC              0x02
#define DEVICE_ID_ACTUATOR        0x04
#define DEVICE_ID_RADIO           0x08
#define DEVICE_ID_RADUNDANCY1     0x10
#define DEVICE_ID_RADUNDANCY2     0x20
#define DEVICE_ID_RADUNDANCY3     0x40
#define DEVICE_ID_RADUNDANCY4     0x80
#define DEVICE_ID_ALL             0xFF

#define DEVICE_ID									DEVICE_ID_MASTER                  /**  <<<<<<<<<<<<<<<<<<<<<<<<< Change It When Switch Device */
#define DEVICE_PROTOCOL_ADDRESS 	DEVICE_ID
#define INIT_PREV_ADD 						DEVICE_ID_ACTUATOR
#define INIT_FOLL_ADD 						DEVICE_ID_ACTUATOR

#define DEVICE_STATE_READY                           0x00
#define DEVICE_STATE_STOP                            0x01
#define DEVICE_STATE_HALT                            0x02
#define DEVICE_STATE_RECEVING                        0x04
#define DEVICE_STATE_SENDING_DATA                    0x08
#define DEVICE_STATE_SENDING_CMD                     0x10
#define DEVICE_STATE_WAITING_RECEIVING_CMD			 0x20
#define DEVICE_STATE_WAITING_RECEIVING_DATA			 0x40
#define DEVICE_STATE_ERROR                           0x80

#define NET_DEVICE_STATE_ONLINE						 1
#define NET_DEVICE_STATE_OFFLINE					 0


#define UNPACK_DATA_OK 					PROTOCOL_DATA
#define UNPACK_CMD_OK 					PROTOCOL_CMD

/** PROTOCOL SETTING **/
#define MAX_CMD_DATA_LENGTH 		  	10
#define MAX_PROTOBUF_LENGTH 		  	50
#define MAX_FLAME_INTERVAL 			 	20
#define MAX_BYTE_INTERVAL         		2
#define MAX_RETRANSMIT_TIMES 			5
#define MAX_PING_TIMES					5

#define USART_START_MARK1 				0xFF
#define USART_START_MARK2 				0x00
#define USART_END_MARK1  				0x00
#define USART_END_MARK2  				0xFF

#define USART_RCV_TIMEOUT_MS 5

/** CMD_mode_table */
#define CMD_HALT_ALL					PROTOCOL_CMD + 0
#define CMD_PING				 		PROTOCOL_CMD + 1
#define CMD_RESPONSE_PING 				PROTOCOL_CMD + 2
#define CMD_SPECIFIED_SEND				PROTOCOL_CMD + 3
#define CMD_PARDON				 		PROTOCOL_CMD + 4
#define CMD_CHANGE_ADDRESS				PROTOCOL_CMD + 5
#define CMD_FEEDBACK_CHANGES			PROTOCOL_CMD + 6
#define CMD_SPECIFIED_READY				PROTOCOL_CMD + 7
#define CMD_RESPONSE_READY				PROTOCOL_CMD + 8
#define CMD_RESUME_ALL					PROTOCOL_CMD + 9

/** BIT OPRATION **/
#define __SET_BITx(num, x) ( (num) | (0x01 << (x) ) )
#define __RESET_BITx(num, x) ( (num) & (~(0x01 << (x) )) )

#define __IS_BITx_SET(num, x) !!( (num) & (0x01 << (x) ) )
#define __IS_IN_CLOSE_RANGE(num, a, b) ( ( (num) >= (a) ) && ( (num) <= (b) ) )?1:0
#define __IS_SMALLER(num, a) ((num) < (a))?1:0
#define __GET_2BIT(num, x) ((num) & (0x03 << (x))) >> (x)

#define __IS_CMD_CALLING_ME(id) !!((id) & DEVICE_ID)

#define __IS_DEVICE_STATE_READY(device_state)                           (!!((device_state)|0x00))
#define __IS_DEVICE_STATE_STOP(device_state)                            (!!((device_state)|0x01))
#define __IS_DEVICE_STATE_HALT(device_state)                            (!!((device_state)|0x02))
#define __IS_DEVICE_STATE_RECEVING(device_state)                        (!!((device_state)|0x04))
#define __IS_DEVICE_STATE_SENDING_DATA(device_state)                    (!!((device_state)|0x08))
#define __IS_DEVICE_STATE_SENDING_CMD(device_state)                     (!!((device_state)|0x10))
#define __IS_DEVICE_STATE_WAITING_RECEIVING_CMD(device_state)			(!!((device_state)|0x20))
#define __IS_DEVICE_STATE_WAITING_RECEIVING_DATA(device_state)			(!!((device_state)|0x40))
#define __IS_DEVICE_STATE_ERROR(device_state)                           (!!((device_state)|0x80))

/** PARAMETER SETTING */
#define JOYSTICK_DEADZONE 400
#define RANGE_RES 																	 100

#define ROT_RANGE_PLUS                               2000
#define ROT_RANGE_SUB                                2000
#define LINEAR_RANGE_PLUS                            3000
#define LINEAR_RANGE_SUB                             1000

#define ORIGIN_POINT_CURRENT_GUIDEWIRE_POSITION      RANGE_RES + LINEAR_RANGE_SUB
#define ORIGIN_POINT_CURRENT_GUIDEWIRE_ANGLE         RANGE_RES + ROT_RANGE_SUB
#define ORIGIN_POINT_CURRENT_TUBE_POSITION		     RANGE_RES + LINEAR_RANGE_SUB
#define ORIGIN_POINT_CURRENT_TUBE_ANGLE              RANGE_RES + ROT_RANGE_SUB
#define ORIGIN_POINT_CURRENT_BALLON_POSITION         RANGE_RES + LINEAR_RANGE_SUB

#define UPPER_LIMITS_CURRENT_GUIDEWIRE_POSITION      ORIGIN_POINT_CURRENT_GUIDEWIRE_POSITION + LINEAR_RANGE_PLUS
#define UPPER_LIMITS_CURRENT_GUIDEWIRE_ANGLE         ORIGIN_POINT_CURRENT_GUIDEWIRE_ANGLE    +    ROT_RANGE_PLUS
#define UPPER_LIMITS_CURRENT_TUBE_POSITION		     ORIGIN_POINT_CURRENT_TUBE_POSITION	 + LINEAR_RANGE_PLUS
#define UPPER_LIMITS_CURRENT_TUBE_ANGLE              ORIGIN_POINT_CURRENT_TUBE_ANGLE         +    ROT_RANGE_PLUS
#define UPPER_LIMITS_CURRENT_BALLON_POSITION         ORIGIN_POINT_CURRENT_BALLON_POSITION    + LINEAR_RANGE_PLUS

#define LOWER_LIMITS_CURRENT_GUIDEWIRE_POSITION      RANGE_RES
#define LOWER_LIMITS_CURRENT_GUIDEWIRE_ANGLE         RANGE_RES
#define LOWER_LIMITS_CURRENT_TUBE_POSITION		     RANGE_RES
#define LOWER_LIMITS_CURRENT_TUBE_ANGLE              RANGE_RES
#define LOWER_LIMITS_CURRENT_BALLON_POSITION         RANGE_RES


#define __START_TRANSMIT_RS485(RE_GPIOX, RE_PIN) HAL_GPIO_WritePin((RE_GPIOX), (RE_PIN), GPIO_PIN_SET)
#define __END_TRANSMIT_RS485(RE_GPIOX, RE_PIN) HAL_GPIO_WritePin((RE_GPIOX), (RE_PIN), GPIO_PIN_RESET)

#define __485_FIELD_BUS_TRANSMIT RS485Transmit(&huart1, uart1_tx_buf, tx_data.frame_len)

#define	__SET_BALLON_SPEED_MODE                     do{tx_data.mode=__RESET_BITx(tx_data.mode,1);tx_data.mode=__RESET_BITx(tx_data.mode,0);}while(0)
#define	__SET_TUBE_SPEED_MODE	                    do{tx_data.mode=__RESET_BITx(tx_data.mode,3);tx_data.mode=__RESET_BITx(tx_data.mode,2);}while(0)
#define __SET_GUIDEWIRE_SPEED_MODE                  do{tx_data.mode=__RESET_BITx(tx_data.mode,5);tx_data.mode=__RESET_BITx(tx_data.mode,4);}while(0)
#define	__SET_BALLON_STEP_MODE 	                    do{tx_data.mode=__RESET_BITx(tx_data.mode,1);tx_data.mode=__SET_BITx(tx_data.mode,0);}while(0)
#define	__SET_TUBE_STEP_MODE                        do{tx_data.mode=__RESET_BITx(tx_data.mode,3);tx_data.mode=__SET_BITx(tx_data.mode,2);}while(0)
#define __SET_GUIDEWIRE_STEP_MODE                   do{tx_data.mode=__RESET_BITx(tx_data.mode,5);tx_data.mode=__SET_BITx(tx_data.mode,4);}while(0)


typedef struct
{
    /** @Header 0xff 0x00 */

    /*Frame Header -- 3 bytes */
    uint8_t 	address; 											/* ADDRESS: [Master: 0x01], [Slave: 0x02], [PC: 0x03]. */
    uint8_t 	mode; 												/* 8 Bit: [0-1: guidewire] [2-3: tube] [4-5: ballon]   ----mode control
                                                                                                        [6: exist control_part] [7: exist parameter_part]  --------Frame length control
                                                                                                        [00B: speed mode] [01B: position mode] [10B and 11B res]*/
    uint8_t 	frame_len;										/* Length of frame */

    /** @control_part | Control platform setting parameter -- 6 bytes */
    uint8_t 	button_state;									/* 8 Bit: [Button ABCD and re.] */
    uint8_t 	guidewire_linear_bar;						/* Value of ADC - guidewire front and back bar */
    uint8_t 	guidewire_rot_bar;						/* Value of ADC - guidewire rotation bar */
    uint8_t 	tube_linear_bar;									/* Value of ADC - Tube bar */
    uint8_t 	tube_rot_bar;									/* Value of ADC - Tube front and back bar */
    uint8_t 	ballon_linear_bar;								/* Value of ADC - Ballon bar */

    /** @parameter_part | Set para -- 10 bytes */
    uint8_t 	maxSpeed_guidewire_linear_bar;		/* Set the maxium movement speed of guidewire */
    uint8_t 	maxSpeed_guidewire_rot_bar;		/* Set the maxium rotation speed of guidewire */
    uint8_t 	maxSpeed_tube_linear_bar;				/* Set the maxium movement speed of tube */
    uint8_t 	maxSpeed_tube_rot_bar;				/* Set the maxium rotation speed of tube */
    uint8_t 	maxSpeed_ballon_linear_bar;			/* Set the maxium movement speed of ballon */

    uint8_t 	step_guidewire_linear_bar;			/* Set the length of the displacement of guidewire */
    uint8_t 	step_guidewire_rot_bar;			/* Set the rotation of the angle of the guidewire */
    uint8_t 	step_tube_linear_bar;					/* Set the length of the displacement of the tube */
    uint8_t 	step_tube_rot_bar;					/* Set the rotation of the angle of the tube */
    uint8_t 	step_ballon_linear_bar;				/* Set the length of the displacement of ballon */

    /** @information_part | Imformation -- 10 bytes */
    uint16_t 	current_guidewire_position;	/* Get or calculate current position of guidewire */
    uint16_t 	current_guidewire_angle;			/* Get or calculate current angle of guidewire */
    uint16_t 	current_tube_position;			  /* Get or calculate current position of tube */
    uint16_t 	current_tube_angle;			    /* Get or calculate current angle of tube */
    uint16_t 	current_ballon_position;			/* Get or calculate current position of ballon */

    /** @frame_tail | end of frame -- 3 bytes */
    uint8_t token_to_next_address;				/* Send the token to next address, prevent the conflict of bus */
    uint16_t crc_check;

    /** @Tail 0x00 0xff */
}DATA_ProtocolFrameTypedef;

typedef struct
{
    /*Frame Header*/
    uint8_t address; 							/* ADDRESS: [CMD: 0x00] */
    uint8_t target_address; 			/* Set the target address, Range: 0x01-0xff, IF 0x00, ALL DEVICE SET */
    uint8_t data_length;						/* Length of data */
    uint8_t mode;			/** @ref CMD_mode_table */
    uint8_t data[MAX_CMD_DATA_LENGTH];
        /** @frame_tail | end of frame -- 2 bytes */
    uint16_t crc_check;
}CMD_ProtocolFrameTypedef;

typedef struct
{
    uint16_t AD_DMA_DATA[5];

    /** @pJoystickValue */
    uint16_t *ballon_fnb_bar;
    uint16_t *tube_fnb_bar;
    uint16_t *tube_rot_bar;
    uint16_t *guidewire_fnb_bar;
    uint16_t *guidewire_rot_bar;

    uint8_t  Turbo_current_state;
}masterPanelOperationDataTypedef;


typedef struct
{
    /** @All */
    uint8_t device_state;

    uint8_t previous_address;
    uint8_t following_address;

    uint8_t reTransmit_count;
    uint8_t reTransmit_which;

    uint8_t prev_sending_what;

    #if DEVICE_ID == DEVICE_ID_MASTER
        uint8_t _online_devices_state[10];

    #endif

}networkInformationTypedef;

extern DATA_ProtocolFrameTypedef 				tx_data;
extern DATA_ProtocolFrameTypedef 				rx_data;
extern CMD_ProtocolFrameTypedef 				cmd_tx_data;
extern CMD_ProtocolFrameTypedef 				cmd_rx_data;
extern masterPanelOperationDataTypedef 		masterPanelData;
extern networkInformationTypedef 					networkImformation;

extern uint8_t uart1_tx_buf[MAX_PROTOBUF_LENGTH];

/** CMD CALL DEVICES **/
extern uint8_t _online_devices[10];
extern uint8_t add_call;
const unsigned char crc_array[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F,
    0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77, 0x7E, 0x79,
    0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53,
    0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97,
    0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1,
    0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC,
    0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88,
    0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E,
    0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04,
    0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E,
    0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8,
    0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2,
    0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56,
    0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
    0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A,
    0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39,
    0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F,
    0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5,
    0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1,
    0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3 };

class protocol
{

private:

    DATA_ProtocolFrameTypedef 				tx_data;
    DATA_ProtocolFrameTypedef 				rx_data;
    CMD_ProtocolFrameTypedef 				cmd_tx_data;
    CMD_ProtocolFrameTypedef 				cmd_rx_data;

    masterPanelOperationDataTypedef 		masterPanelData;

    uint8_t 								uart1_tx_buf[MAX_PROTOBUF_LENGTH];
    SerialPort port;
    networkInformationTypedef networkImformation;

private:
    /** Driven Layer **/
    uint8_t CRC8_Table(uint8_t *p, char counter);

    uint8_t checkProtocolFrameTypedefLegal(DATA_ProtocolFrameTypedef *proto);

        /** Abstract layer **/
    uint8_t packProtocolFrame(DATA_ProtocolFrameTypedef *proto);
    uint8_t packCMDFrame(CMD_ProtocolFrameTypedef *proto);
    uint8_t unpackProtocolFrame(DATA_ProtocolFrameTypedef *master, CMD_ProtocolFrameTypedef *CMD);

    void updateControlPart(void);
    void updateImformationPart(void);

    void CMD_Handle(void);
/**************************************begin @CMDsend definition***************************************************/
    #if DEVICE_ID == DEVICE_ID_MASTER
    void CMD_HaltAll(void);			//master only
    void CMD_SpecifiedSend(uint8_t Address);			//master only
    void CMD_ChangeAddress(uint8_t Address, uint8_t prevAddress, uint8_t nextAddress);			//master only
    void CMD_SpecifiedReady(uint8_t Address);		//master only
    void CMD_ResumeAll(void);			//master only
    #endif
    void CMD_Ping(uint8_t Address);
    void CMD_ResponsePing(uint8_t Address);
    void CMD_Pardon(void);

    void CMD_ResponseSpecifiedSend(uint8_t Address); 	//slave only
    void CMD_ResponseSpecifiedReady(uint8_t Address);			//slave only
    void CMD_FeedbackChanges(uint8_t Address);			//slave only

    uint8_t CMD_PollingDeviceOnline(void);
/**************************************end @CMDsend definition***************************************************/


/**************************************begin @CMDrecv definition***************************************************/
    void executeWhenReceiveCmdHaltAll(void);		//slave only
    void executeWhenReceiveSpecifiedSend(void);	//slave only
    void executeWhenReceiveChangeAddress(void);	//slave only
    void executeWhenReceiveSpecifiedReady(void);//slave only
    void executeWhenReceiveResumeAll(void);			//slave only


    void executeWhenReceiveCmdPing(void);		//master or slave
    void executeWhenReceiveResponsePing(void);	//master or slave
    void executeWhenReceivePardon(void);		//master or slave

    #if DEVICE_ID == DEVICE_ID_MASTER
        void executeWhenReceiveResponseReady(void);		//master only
        void executeWhenReceiveFeedbackChanges(void);	//master only
    #endif
/**************************************end @CMDrecv definition***************************************************/

    void DATA_SendFlame(uint8_t Address);
    void DATA_HANDLE(void);
    void resetActuator(uint8_t errorType);
    void errCodeCallback(uint8_t errorCode);

    QByteArray array2ByteArray();
};

#endif // PROTOCOL_H
