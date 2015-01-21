#include "platform.h"
#include "bleprofile.h"
#include "LEBoard_firmware.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "pwm.h"
#include "aclk.h"
#include "devicelpm.h"

/******************************************************
 *                    Constants
 ******************************************************/

//This GPIO definition is only used for test mode
#define DIN0        25
#define DIN1        24
#define DOUT0       8
#define DOUT1       27  // This dummy
#define DIN_BYTE    1   // number of bytes in aggregate input
#define AIN_BYTE    4   // number of bytes in aggregate input

#define DIGITAL     0
#define ANALOG      1

#define AIO_PWM1       26
#define AIO_PWM2       28
#define AIO_PWM3       -1 //28
#define AIO_PWM_BASE   26
#define AIO_PWM_STEPS 1000

#define AIO_ADC0
#define AIO_ADC1
#define AIO_ADC2

/******************************************************
 *               Function Declarations
 ******************************************************/
static void  bleaio_Timeout( UINT32 count );
static void  bleaio_FineTimeout( UINT32 finecount );
static void  bleaio_DBInit( void );
static void  bleaio_connUp( void );
static void  bleaio_connDown( void );
static void  bleaio_advStop( void );
static void  bleaio_appTimerCb( UINT32 arg );
static void  bleaio_appFineTimerCb( UINT32 arg );
static void  bleaio_smpBondResult( LESMP_PARING_RESULT result );
static void  bleaio_encryptionChanged( HCI_EVT_HDR *evt );
static int   bleaio_writeCb( LEGATTDB_ENTRY_HDR *p );
static void  bleaio_IntInit( void );
static void  bleaio_IntCb( void *data );
static UINT8 bleaio_checktrigger( UINT16 value, UINT8 type, UINT8 index );
static void  bleaio_checktimetrigger( void );
static void  bleaio_resettimetrigger( UINT8 index );
static void  bleaio_stoptimetrigger( UINT8 index );
static void  bleaio_analoginput( UINT8 index, UINT16 value );
static void  bleaio_digitalinput( UINT8 value );
static void  bleaio_output( UINT16 handle );
static void  bleaio_IndicationConf( void );
extern void  bleprofile_regAppEvtHandler( BLECM_APP_EVT_ENUM idx, BLECM_NO_PARAM_FUNC func );
static void  bleprofile_setidletimer_withNotification(void);
extern void  bleprofile_appTimerCb( UINT32 arg );

static void  bleaio_pwm_init( UINT8 gpio, UINT8 clock, UINT16 total_step, UINT16 toggle_step );
static void  bleaio_pwm_set( UINT8 gpio, UINT16 total_step, UINT16 toggle_step );
static void  bleaio_pwm_off( UINT8 gpio );


/******************************************************
 *               Variables Definitions
 ******************************************************/

const UINT8 bleaio_db_data[]=
{
    // Handle 0x01: GATT service
    PRIMARY_SERVICE_UUID16 (0x0001, UUID_SERVICE_GATT),

    // Handle 0x02: characteristic Service Changed, handle 0x03 characteristic value
    CHARACTERISTIC_UUID16  (0x0002, 0x0003, UUID_CHARACTERISTIC_SERVICE_CHANGED, LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_NONE, 4),
    0x00, 0x00, 0x00, 0x00,

    // Handle 0x14: GAP service
    PRIMARY_SERVICE_UUID16 (0x0014, UUID_SERVICE_GAP ),

    // Handle 0x15: characteristic Device Name, handle 0x16 characteristic value
    CHARACTERISTIC_UUID16 (0x0015, 0x0016, UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 16),
    'B','L','E',' ','A','u','t','o','m','a','t','i','o','n','I','O',

    // Handle 0x17: characteristic Appearance, handle 0x18 characteristic value
    CHARACTERISTIC_UUID16 (0x0017, 0x0018, UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
    0x00, 0x00,

    // Handle 0x21:  Automation IO
    PRIMARY_SERVICE_UUID16 (0x0021, UUID_SERVICE_AUTOMATION_IO),

    // Handle 0x22: Characteristic Digital Input, handle 0x2a characteristic value
    CHARACTERISTIC_UUID16 (0x0022, 0x0023, UUID_CHARACTERISTIC_DIGITAL_INPUT,
    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
    LEGATTDB_PERM_READABLE, 1),
    0x00,

    // Handle 0x24: Client Characteristic Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0024, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
    0x00, 0x00,

    // Handle 0x25: Number of Digitals Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0025, UUID_DESCRIPTOR_NUMBER_OF_DIGITALS, LEGATTDB_PERM_READABLE, 1),
    0x02,                       // 2 digital input

    // Handle 0x26: Trigger Settings Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0026, UUID_DESCRIPTOR_TRIGGER_SETTING,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 4),
    0x00, 0x00, 0x00, 0x00,

    // Handle 0x27: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0027, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT,
    LEGATTDB_PERM_READABLE, 7),
    0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, // 2bit

    // Handle 0x28: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0028, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'D','I','0','0',

    // Handle 0x32: Characteristic Digital Output, handle 0x33 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE (0x0032, 0x0033, UUID_CHARACTERISTIC_DIGITAL_OUTPUT,
    LEGATTDB_CHAR_PROP_WRITE| LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
    LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ,  1),
    0x00,

    // Handle 0x34: Number of Digitals Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0034, UUID_DESCRIPTOR_NUMBER_OF_DIGITALS, LEGATTDB_PERM_READABLE, 1),
    0x02,                       // 2 digital output

    // Handle 0x37: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0037, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE, 7),
    0x03,0x00,0x00,0x00,0x01,0x00,0x00, // 4bit

    // Handle 0x38: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16( 0x0038, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'D','O','0','0',

    // Handle 0x42: Characteristic Analog Input, handle 0x43 characteristic value
    CHARACTERISTIC_UUID16 (0x0042, 0x0043, UUID_CHARACTERISTIC_ANALOG_INPUT,
    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
    LEGATTDB_PERM_READABLE, 2),
    0x00, 0x00,

    // Handle 0x44: Client Characteristic Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0044, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
    0x00, 0x00,                 // write 1 will trigger notification, 2 - indication

    // Handle 0x46: Trigger Settings Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0046, UUID_DESCRIPTOR_TRIGGER_SETTING,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 4),
    0x00, 0x00, 0x00, 0x00,     //

    // Handle 0x47: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16( 0x0047, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE, 7),
    0x06,0x00,0x00,0x00,0x01,0x00,0x00, //uint16

    // Handle 0x48: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16( 0x0048, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'A','I','0','0',

    // Handle 0x52: Characteristic Analog Input, handle 0x53 characteristic value
    CHARACTERISTIC_UUID16 (0x0052, 0x0053, UUID_CHARACTERISTIC_ANALOG_INPUT,
    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
    LEGATTDB_PERM_READABLE, 2),
    0x00, 0x00,

    // Handle 0x54: Client Characteristic Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0054, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
    0x00, 0x00,                 // write 1 will trigger notification, 2 - indication

    // Handle 0x56: Trigger Settings Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0056, UUID_DESCRIPTOR_TRIGGER_SETTING,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 4),
    0x00, 0x00, 0x00, 0x00,     //

    // Handle 0x57: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0057, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE, 7),
    0x06,0x00,0x00,0x00,0x01,0x00,0x00, //uint16

    // Handle 0x58: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0058, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'A','I','0','1',

    // Handle 0x62: Characteristic Analog Input, handle 0x53 characteristic value
    CHARACTERISTIC_UUID16 (0x0062, 0x0063, UUID_CHARACTERISTIC_ANALOG_INPUT,
    LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
    LEGATTDB_PERM_READABLE, 2),
    0x00, 0x00,

    // Handle 0x64: Client Characteristic Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0064, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
    0x00, 0x00,                 // write 1 will trigger notification, 2 - indication

    // Handle 0x66: Trigger Settings Descriptor
    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0066, UUID_DESCRIPTOR_TRIGGER_SETTING,
    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 4),
    0x00, 0x00, 0x00, 0x00,     //

    // Handle 0x67: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0067, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE, 7),
    0x06,0x00,0x00,0x00,0x01,0x00,0x00, //uint16

    // Handle 0x68: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0068, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'A','I','0','2',

    // Handle 0x72: Characteristic Analog Output, handle 0x73 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE (0x0072, 0x0073, UUID_CHARACTERISTIC_ANALOG_OUTPUT,
    LEGATTDB_CHAR_PROP_WRITE| LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
    LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ, 2),
    0x00, 0x00,

    // Handle 0x77: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0077, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT,LEGATTDB_PERM_READABLE, 7),
    0x06,0x00,0x00,0x00,0x01,0x00,0x00, // uint16

    // Handle 0x78: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0078, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'A','O','0','0',

    // Handle 0x82: Characteristic Analog Output, handle 0x83 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE (0x0082, 0x0083, UUID_CHARACTERISTIC_ANALOG_OUTPUT,
    LEGATTDB_CHAR_PROP_WRITE| LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
    LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ, 2),
    0x00, 0x00,

    // Handle 0x87: Presentation Format Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0087, UUID_DESCRIPTOR_CHARACTERISTIC_PRESENTATION_FORMAT, LEGATTDB_PERM_READABLE, 7),
    0x06,0x00,0x00,0x00,0x01,0x00,0x00, // uint16

    // Handle 0x88: User Description Descriptor
    CHAR_DESCRIPTOR_UUID16 (0x0088, UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, LEGATTDB_PERM_READABLE, 4),
    'A','O','0','1',

    // Handle 0xa1: Battery service
    PRIMARY_SERVICE_UUID16 (0x00a1, UUID_SERVICE_BATTERY),

    // Handle 0xa2: characteristic Battery Level, handle 0xa3 characteristic value
    CHARACTERISTIC_UUID16 (0x00a2, 0x00a3, UUID_CHARACTERISTIC_BATTERY_LEVEL,
    LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 1),
    0x64
};

const UINT16 bleaio_db_size = sizeof( bleaio_db_data );

const BLE_PROFILE_CFG bleaio_cfg =
{
    /*.fine_timer_interval            =*/ 1000, //ms
    /*.default_adv                    =*/ MANDATORY_DISCOVERABLE,    // HIGH_UNDIRECTED_DISCOVERABLE
    /*.button_adv_toggle              =*/ 0,    // pairing button make adv toggle (if 1) or always on (if 0)
    /*.high_undirect_adv_interval     =*/ 32,   // slots
    /*.low_undirect_adv_interval      =*/ 2048, // slots
    /*.high_undirect_adv_duration     =*/ 30,   // seconds
    /*.low_undirect_adv_duration      =*/ 300,  // seconds
    /*.high_direct_adv_interval       =*/ 0,    // seconds
    /*.low_direct_adv_interval        =*/ 0,    // seconds
    /*.high_direct_adv_duration       =*/ 0,    // seconds
    /*.low_direct_adv_duration        =*/ 0,    // seconds
    /*.local_name                     =*/ "BLE AutomationIO", // [LOCAL_NAME_LEN_MAX];
    /*.cod                            =*/ "\x00\x00\x00",     // [COD_LEN];
    /*.ver                            =*/ "1.00",             // [VERSION_LEN];
    /*.encr_required                  =*/ 0,    // if 1, encryption is needed before sending indication/notification
    /*.disc_required                  =*/ 0,    // if 1, disconnection after confirmation
    /*.test_enable                    =*/ 1,    // TEST MODE is enabled when 1
    /*.tx_power_level                 =*/ 0x04, // dbm
    /*.con_idle_timeout               =*/ 0,    // second  0-> no timeout
    /*.powersave_timeout              =*/ 0,    // second  0-> no timeout
    /*.hdl                            =*/ {0x0025, 0x00a3, 0x00, 0x00, 0x00}, // [HANDLE_NUM_MAX];   //GATT HANDLE number
    /*.serv                           =*/ {UUID_SERVICE_AUTOMATION_IO, UUID_SERVICE_BATTERY, 0x00, 0x00, 0x00},
    /*.cha                            =*/ {0x00, UUID_CHARACTERISTIC_BATTERY_LEVEL, 0x00, 0x00, 0x00},
    /*.findme_locator_enable          =*/ 0,    // if 1 Find me locator is enable
    /*.findme_alert_level             =*/ 0,    // alert level of find me
    /*.client_grouptype_enable        =*/ 0,    // if 1 grouptype read can be used
    /*.linkloss_button_enable         =*/ 0,    // if 1 linkloss button is enable
    /*.pathloss_check_interval        =*/ 0,    // second
    /*.alert_interval                 =*/ 0,    // interval of alert
    /*.high_alert_num                 =*/ 0,    // number of alert for each interval
    /*.mild_alert_num                 =*/ 0,    // number of alert for each interval
    /*.status_led_enable              =*/ 0,    // if 1 status LED is enable
    /*.status_led_interval            =*/ 0,    // second
    /*.status_led_con_blink           =*/ 0,    // blink num of connection
    /*.status_led_dir_adv_blink       =*/ 0,    // blink num of dir adv
    /*.status_led_un_adv_blink        =*/ 0,    // blink num of undir adv
    /*.led_on_ms                      =*/ 0,    // led blink on duration in ms
    /*.led_off_ms                     =*/ 0,    // led blink off duration in ms
    /*.buz_on_ms                      =*/ 0,    // buzzer on duration in ms
    /*.button_power_timeout           =*/ 0,    // seconds
    /*.button_client_timeout          =*/ 0,    // seconds
    /*.button_discover_timeout        =*/ 0,    // seconds
    /*.button_filter_timeout          =*/ 0,    // seconds
    #ifdef BLE_UART_LOOPBACK_TRACE
    /*.button_uart_timeout            =*/ 15,   // seconds
    #endif
};

const BLE_PROFILE_PUART_CFG bleaio_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ 32,       // GPIO pin number 20730A0 module need to use 32 instead
    /*.rxpin      =*/ 33,       // GPIO pin number
};

const BLE_PROFILE_GPIO_CFG bleaio_gpio_cfg =
{
    /*.gpio_pin =*/
    {
        GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
        GPIO_PIN_BATTERY, // Battery monitoring GPIO. When it is lower than particular level, it will give notification to the application.
        DIN0, DIN1, DOUT0, DOUT1,
        AIO_PWM1, AIO_PWM2, AIO_PWM3,
        -1, -1, -1, -1, -1, -1, -1
    },
    /*.gpio_flag =*/
    {
        GPIO_SETTINGS_WP,
        GPIO_SETTINGS_BATTERY,
        GPIO_INPUT | GPIO_INIT_LOW | GPIO_INT, //this is set up with IntInit()
        GPIO_INPUT | GPIO_INIT_LOW | GPIO_INT,
        GPIO_OUTPUT | GPIO_INIT_HIGH,
        GPIO_OUTPUT | GPIO_INIT_HIGH,
        GPIO_OUTPUT | GPIO_INIT_HIGH,
        GPIO_OUTPUT | GPIO_INIT_HIGH,
        GPIO_OUTPUT | GPIO_INIT_HIGH,
        0, 0, 0, 0, 0, 0, 0
    }
};


BLE_AIO_GATT_CFG bleaio_gatt_cfg =
{
    /*.hdl  =*/ {0x33, 0x73, 0x83, 0x00, 0x00},
    /*.serv =*/ {UUID_SERVICE_AUTOMATION_IO, UUID_SERVICE_AUTOMATION_IO, UUID_SERVICE_AUTOMATION_IO, 0, 0},
    /*.cha  =*/ {UUID_CHARACTERISTIC_DIGITAL_OUTPUT, UUID_CHARACTERISTIC_ANALOG_OUTPUT, UUID_CHARACTERISTIC_ANALOG_OUTPUT, 0, 0}
};

// service notification configuration
BLE_AIO_NOT_GATT_CFG bleaio_not_gatt_cfg =
{
    /*.tick    =*/ {0, 0, 0, 0, 0},  // [AIO_NOT_HANDLE_NUM_MAX];
    /*.timeout =*/ {0, 0, 0, 0, 0},
    /*.count   =*/ {0, 0, 0, 0, 0},
    /*.hdl	=*/ {0x23, 0x43, 0x53, 0x63, 0},
    /*.cl_hdl  =*/ {0x24, 0x44, 0x54, 0x64, 0},
    /*.tr_hdl  =*/ {0x26, 0x46, 0x56, 0x66, 0},
    /*.tr2_hdl =*/ {0x93, 0x95, 0x97, 0x99, 0},
    /*.serv    =*/ {UUID_SERVICE_AUTOMATION_IO, UUID_SERVICE_AUTOMATION_IO,
                    UUID_SERVICE_AUTOMATION_IO, UUID_SERVICE_AUTOMATION_IO, 0},
    /*.cha     =*/ {UUID_CHARACTERISTIC_DIGITAL_INPUT, UUID_CHARACTERISTIC_ANALOG_INPUT,
                    UUID_CHARACTERISTIC_ANALOG_INPUT, UUID_CHARACTERISTIC_ANALOG_INPUT, 0}
};

UINT32 	bleaio_apptimer_count		= 0;
UINT32 	bleaio_appfinetimer_count	= 0;
UINT16 	bleaio_con_handle               = 0;
BD_ADDR bleaio_remote_addr;
UINT8 	bleaio_bat_enable               = 0;
UINT8 	bleaio_indication_sent 		= 0;

//NVRAM save area
BLEAIO_HOSTINFO bleaio_hostinfo;

/******************************************************
 *               Function Definitions
 ******************************************************/

APPLICATION_INIT()
{
    bleapp_set_cfg((UINT8 *)bleaio_db_data,
                   bleaio_db_size,
                   (void *)&bleaio_cfg,
                   (void *)&bleaio_puart_cfg,
                   (void *)&bleaio_gpio_cfg,
                   bleaio_Create);
    bleapp_trace_enable = 0;
}

UINT32 bleautoio_queryPowersave(LowPowerModePollType type, UINT32 context)
{
    // prevent sleep with return 0
    return 0;
}

void bleaio_Create(void)
{
    extern UINT32 blecm_configFlag ;
    blecm_configFlag |= BLECM_DBGUART_LOG;

    ble_trace0("bleaio_Create()");
    ble_trace0(bleprofile_p_cfg->ver);

    // dump the database to debug uart.
    legattdb_dumpDb();

    if(!bleaio_cfg.powersave_timeout)
    {
        devlpm_init();

        devlpm_enableWakeFrom(DEV_LPM_WAKE_SOURCE_GPIO);
    }

    devlpm_registerForLowPowerQueries(
        (DeviceLpmQueriableMethodCallback) bleautoio_queryPowersave, 0);

    bleprofile_Init(bleprofile_p_cfg);
    bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

    // Initialize and configure the ADC driver
    adc_config();

    bleaio_DBInit(); //load handle number

    // register connection up and connection down handler.
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP, bleaio_connUp );
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, bleaio_connDown );
    bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, bleaio_advStop );

    // handler for Encryption changed.
    blecm_regEncryptionChangedHandler(bleaio_encryptionChanged);
    // handler for Bond result
    lesmp_regSMPResultCb((LESMP_SINGLE_PARAM_CB) bleaio_smpBondResult);

    legattdb_regWriteHandleCb((LEGATTDB_WRITE_CB)bleaio_writeCb);

    if (bleprofile_p_cfg->test_enable)
    {
        bleaio_IntInit();
    }

    bleprofile_regTimerCb(bleaio_appFineTimerCb, bleaio_appTimerCb);
    bleprofile_StartTimer();

    //This is only for PMU_CLK
    // LHL_CLK's base frequency is 125 Hz
    aclk_configure(24000000, ACLK1, ACLK_FREQ_24_MHZ); // base clock 23.46 kHz, use 0x200-0x300 makes 46.78 kHz

    bleaio_pwm_init(AIO_PWM1,PMU_CLK,AIO_PWM_STEPS, 0); //PMU_CLK or LHL_CLK
    bleaio_pwm_init(AIO_PWM2,PMU_CLK,AIO_PWM_STEPS, 0);
    bleaio_pwm_init(AIO_PWM3,PMU_CLK,AIO_PWM_STEPS, 0);

    bleaio_connDown();
}

void bleaio_FakeUART(char *bpm_char, UINT32 count)
{
    //This function does data change or setting for test

}

void bleaio_Timeout(UINT32 count)
{
    ble_trace1("Normaltimer:%d", count);

    if (bleaio_bat_enable)
    {
        blebat_pollMonitor();
    }

    bleprofile_pollPowersave();

    if (bleprofile_p_cfg->test_enable)
    {
        bleaio_checktimetrigger();
    }

    // ADC0
    bleaio_analoginput(0, adc_readVoltage(7));

    // ADC1
    bleaio_analoginput(1, adc_readVoltage(6));

    // ADC 2
    bleaio_analoginput(2, adc_readVoltage(1));
}

void bleaio_FineTimeout(UINT32 finecount)
{
    char aio_char[READ_UART_LEN + 1];

    //Reading
    bleprofile_ReadUART(aio_char);

    if (aio_char[0] == 'D' && aio_char[1] == 'D') //download start
    {
        blecm_setFilterEnable(0);
        ble_trace0("CSA_filter disabled");
    }
    else if (aio_char[0] == 'A' && aio_char[1] == 'A') //download start
    {
        blecm_setFilterEnable(1);
        ble_trace0("CSA_filter enabled");
        bleaio_connDown();
    }

    // button control
    bleprofile_ReadButton();

}

void bleaio_DBInit(void)
{
    BLEPROFILE_DB_PDU db_pdu;
    int i;
    //load handle number

    for (i = 0; i < HANDLE_NUM_MAX; i++)
    {
        if (bleprofile_p_cfg->serv[i] == UUID_SERVICE_BATTERY &&
                bleprofile_p_cfg->cha[i]  == UUID_CHARACTERISTIC_BATTERY_LEVEL)
        {
            bleaio_bat_enable = 1;
            blebat_Init();
        }
    }

    for (i = 0; i < AIO_HANDLE_NUM_MAX; i++)
    {
        ble_trace2("bleaio_gatt_cfg.hdl[%d]:%04x", i, bleaio_gatt_cfg.hdl[i]);
        bleprofile_ReadHandle(bleaio_gatt_cfg.hdl[i], &db_pdu);
        ble_tracen((char *)db_pdu.pdu, db_pdu.len);
    }

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        ble_trace2("bleaio_not_gatt_cfg.hdl[%d]:%04x", i, bleaio_not_gatt_cfg.hdl[i]);
        bleprofile_ReadHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
        ble_tracen((char *)db_pdu.pdu, db_pdu.len);

        ble_trace2("bleaio_not_gatt_cfg.cl_hdl[%d]:%04x", i, (bleaio_not_gatt_cfg.cl_hdl[i]));
        bleprofile_ReadHandle((bleaio_not_gatt_cfg.hdl[i]), &db_pdu);
        ble_tracen((char *)db_pdu.pdu, db_pdu.len);

        ble_trace2("bleaio_not_gatt_cfg.tr_hdl[%d]:%04x", i, (bleaio_not_gatt_cfg.tr_hdl[i]));
        bleprofile_ReadHandle((bleaio_not_gatt_cfg.hdl[i]), &db_pdu);
        ble_tracen((char *)db_pdu.pdu, db_pdu.len);
    }
}

void bleaio_connUp(void)
{
    BLEPROFILE_DB_PDU db_cl_pdu;
    int i;

    bleaio_con_handle = (UINT16)emconinfo_getConnHandle();

    if (bleprofile_p_cfg->encr_required==0)
    {
        //set variable when encryption is not requested
    }

    // print the bd address.
    memcpy(bleaio_remote_addr, (UINT8 *)emconninfo_getPeerAddr(), sizeof(bleaio_remote_addr));

    ble_trace3("\rbleaio_connUp: %08x%04x %d",
               (bleaio_remote_addr[5] << 24) + (bleaio_remote_addr[4] << 16) +
            (bleaio_remote_addr[3] << 8) + bleaio_remote_addr[2],
            (bleaio_remote_addr[1] << 8) + bleaio_remote_addr[0],
            bleaio_con_handle);

    bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo);

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        //using default value first
        db_cl_pdu.len    = 2;
        db_cl_pdu.pdu[0] = 0x00;
        db_cl_pdu.pdu[1] = 0x00;

        // Save NVRAM to client characteristic descriptor
        if (memcmp(bleaio_remote_addr, bleaio_hostinfo.bdAddr, 6) == 0)
        {
            if ((bleaio_hostinfo.serv[i] == bleaio_not_gatt_cfg.serv[i]) &&
                    (bleaio_hostinfo.cha[i]  == bleaio_not_gatt_cfg.cha[i]))
            {
                db_cl_pdu.pdu[0] = bleaio_hostinfo.cli_cha_desc[i] & 0xFF;
                db_cl_pdu.pdu[1] = bleaio_hostinfo.cli_cha_desc[i] >> 8;
            }
        }

        //reset client char cfg
        if (bleaio_not_gatt_cfg.hdl[i])
        {
            bleprofile_WriteHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);   //assume client is using next handle
            ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);
        }
    }
    // Set idle timer if there are no characteristics configured for notification
    bleprofile_setidletimer_withNotification();
    bleprofile_Discoverable(NO_DISCOVERABLE, NULL);
}

void bleaio_connDown(void)
{
    ble_trace3("\rbleaio_connDown: %08x%04x %d",
               (bleaio_remote_addr[5] << 24) + (bleaio_remote_addr[4] << 16) +
            (bleaio_remote_addr[3] << 8) + bleaio_remote_addr[2],
            (bleaio_remote_addr[1] << 8) + bleaio_remote_addr[0],
            bleaio_con_handle);

    bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo);

    // Save client characteristic descriptor to NVRAM
    if (memcmp(bleaio_remote_addr, bleaio_hostinfo.bdAddr, 6) == 0)
    {
        BLEPROFILE_DB_PDU db_cl_pdu;
        UINT8 writtenbyte;
        int i;

        for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
        {
            if (bleaio_not_gatt_cfg.hdl[i])
            {
                bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                bleaio_hostinfo.serv[i] = bleaio_not_gatt_cfg.serv[i];
                bleaio_hostinfo.cha[i] = bleaio_not_gatt_cfg.cha[i];
                bleaio_hostinfo.cli_cha_desc[i] = db_cl_pdu.pdu[0]+(db_cl_pdu.pdu[1] << 8);
            }
        }
        writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo);
        ble_trace1("NVRAM write:%04x", writtenbyte);
    }

    // go back to Adv.
    // should be select after read NVRAM data
    //bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, NULL);
    //bleprofile_Discoverable(HIGH_DIRECTED_DISCOVERABLE, bleprofile_remote_addr);	//for test only


    // Mandatory discovery mode
    if (bleprofile_p_cfg->default_adv == MANDATORY_DISCOVERABLE)
    {
        bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, NULL);
    }
    // check NVRAM for previously paired BD_ADDR
    else
    {
        bleprofile_Discoverable(bleprofile_p_cfg->default_adv, bleaio_hostinfo.bdAddr);

        ble_trace3("\rADV start: %08x%04x %d",
                   (bleaio_hostinfo.bdAddr[5] << 24) + (bleaio_hostinfo.bdAddr[4] << 16) +
                (bleaio_hostinfo.bdAddr[3] << 8) + bleaio_hostinfo.bdAddr[2],
                (bleaio_hostinfo.bdAddr[1] << 8) + bleaio_hostinfo.bdAddr[0],
                bleaio_con_handle);
    }

    bleaio_con_handle = 0; //reset connection handle
}

void bleaio_advStop(void)
{
    ble_trace0("ADV stop!!!!");
}

void bleaio_appTimerCb(UINT32 arg)
{
    switch(arg)
    {
    case BLEPROFILE_GENERIC_APP_TIMER:
    {
        bleaio_apptimer_count++;

        bleaio_Timeout(bleaio_apptimer_count);
    }
        break;
    }
}

void bleaio_appFineTimerCb(UINT32 arg)
{
    bleaio_appfinetimer_count++;

    bleaio_FineTimeout(bleaio_appfinetimer_count);
}

void bleaio_smpBondResult(LESMP_PARING_RESULT  result)
{
    ble_trace1("\rbleaio, bond result %02x", result);

    if (result == LESMP_PAIRING_RESULT_BONDED)
    {
        // saving bd_addr in nvram
        UINT8 *bda;
        UINT8 writtenbyte;
        BLEPROFILE_DB_PDU db_cl_pdu;
        int i;

        memcpy(bleaio_remote_addr, (UINT8 *)emconninfo_getPeerAddr(), sizeof(bleaio_remote_addr));
        memcpy(bleaio_hostinfo.bdAddr, bleaio_remote_addr, sizeof(bleaio_remote_addr));

        for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
        {
            if (bleaio_not_gatt_cfg.hdl[i])
            {
                bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                bleaio_hostinfo.serv[i] = bleaio_not_gatt_cfg.serv[i];
                bleaio_hostinfo.cha[i] = bleaio_not_gatt_cfg.cha[i];
                bleaio_hostinfo.cli_cha_desc[i] = db_cl_pdu.pdu[0]+(db_cl_pdu.pdu[1] << 8);
            }
        }

        writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo);
        //writtenbyte = bleprofile_WriteNVRAM(0x70, 6, (UINT8 *)&bleaio_hostinfo);

        ble_trace1("NVRAM write:%04x", writtenbyte);
    }
}

void bleaio_encryptionChanged(HCI_EVT_HDR *evt)
{
    ble_trace0("\rbleaio, encryption changed");

    if (bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo))
    {
        if (memcmp(bleaio_hostinfo.bdAddr, emconninfo_getPeerAddr(), 6)==0)
        {
            ble_trace2("\rEncOn for Last paired device: %08x%04x",
                       (bleaio_hostinfo.bdAddr[5] << 24) + (bleaio_hostinfo.bdAddr[4] << 16) +
                    (bleaio_hostinfo.bdAddr[3] << 8) + bleaio_hostinfo.bdAddr[2],
                    (bleaio_hostinfo.bdAddr[1] << 8) + bleaio_hostinfo.bdAddr[0]);
        }
    }

    if (bleprofile_p_cfg->encr_required)
    {
        // set variable when encryption is requested
    }
}

int bleaio_writeCb(LEGATTDB_ENTRY_HDR *p)
{
    UINT16 handle = legattdb_getHandle(p);
    int     len = legattdb_getAttrValueLen(p);
    int     check_idle_timer = 0;
    int     i;

    ble_trace1("\rWriteCb: handle %04x", handle);

    //client setting
    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if (bleaio_not_gatt_cfg.cl_hdl[i] &&
                (bleaio_not_gatt_cfg.cl_hdl[i] == handle))
        {
            BLEPROFILE_DB_PDU db_cl_pdu;

            bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
            ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

            check_idle_timer = 1;

            bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo);

            // Save client characteristic descriptor to NVRAM
            if (memcmp(bleaio_remote_addr, bleaio_hostinfo.bdAddr, 6) == 0)
            {
                UINT8 writtenbyte;

                bleaio_hostinfo.serv[i] = bleaio_not_gatt_cfg.serv[i];
                bleaio_hostinfo.cha[i] = bleaio_not_gatt_cfg.cha[i];
                bleaio_hostinfo.cli_cha_desc[i] = db_cl_pdu.pdu[0]+(db_cl_pdu.pdu[1] << 8);

                writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(BLEAIO_HOSTINFO), (UINT8 *)&bleaio_hostinfo);

                ble_trace1("NVRAM write:%04x", writtenbyte);
            }
        }
    }
    // Set idle timer if there are no characteristics configured for notification
    if (check_idle_timer)
        bleprofile_setidletimer_withNotification();

    //trigger setting
    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if (bleaio_not_gatt_cfg.tr_hdl[i] &&
                (bleaio_not_gatt_cfg.tr_hdl[i] == handle))
        {
            BLEPROFILE_DB_PDU db_tr_pdu;

            bleprofile_ReadHandle(bleaio_not_gatt_cfg.tr_hdl[i], &db_tr_pdu);
            ble_tracen((char *)db_tr_pdu.pdu, db_tr_pdu.len);

            //stop time trigger for all case
            bleaio_stoptimetrigger(i);

            //check trigger condition
            if (db_tr_pdu.len)
            {
                if (db_tr_pdu.pdu[0]  == AIO_NO_IND_TIME)
                {
                    bleaio_not_gatt_cfg.tick[i] =
                            db_tr_pdu.pdu[1]*60*60 + db_tr_pdu.pdu[2]*60 + db_tr_pdu.pdu[3];
                    bleaio_not_gatt_cfg.timeout[i] = 0;
                }
                else if (db_tr_pdu.pdu[0]  == AIO_NO_IND_INTERVAL)
                {
                    bleaio_not_gatt_cfg.tick[i] =
                            db_tr_pdu.pdu[1]*60*60 + db_tr_pdu.pdu[2]*60 + db_tr_pdu.pdu[3];
                    bleaio_not_gatt_cfg.timeout[i] = bleaio_not_gatt_cfg.tick[i];
                }
                else if (db_tr_pdu.pdu[0]  == AIO_CHANGED_MORE_THAN)
                {
                    bleaio_not_gatt_cfg.count[i] = 0;
                }
                else if (db_tr_pdu.pdu[0]  == AIO_CHANGED_MORE_OFTEN_THAN)
                {
                    bleaio_not_gatt_cfg.count[i] = 0;
                }
            }
        }
    }

    //general characteristic
    for (i = 0; i < AIO_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_gatt_cfg.hdl[i]) &&
                ((bleaio_gatt_cfg.hdl[i]) == handle))
        {
            BLEPROFILE_DB_PDU db_pdu;

            bleprofile_ReadHandle(bleaio_gatt_cfg.hdl[i], &db_pdu);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);

            // handle write
            if (bleprofile_p_cfg->test_enable)
            {
                bleaio_output(handle);
            }
        }
    }

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.hdl[i]) &&
                ((bleaio_not_gatt_cfg.hdl[i]) == handle))
        {
            BLEPROFILE_DB_PDU db_pdu;

            bleprofile_ReadHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);

            // handle write
        }
    }

    return 0;
}

void bleaio_IntInit(void)
{
    UINT16 masks[GPIO_NUM_PORTS];

    memset(masks, 0x00, sizeof(UINT16)*GPIO_NUM_PORTS);

    // Make Input detects both edges
    gpio_configurePin(DIN0 >> 4, DIN0 & 0x0F,
                      GPIO_INPUT_ENABLE | GPIO_PULL_DOWN | GPIO_EN_INT_BOTH_EDGE,
                      GPIO_PIN_OUTPUT_HIGH);
    gpio_configurePin(DIN1 >> 4, DIN1 & 0x0F,
                      GPIO_INPUT_ENABLE | GPIO_PULL_UP |GPIO_EN_INT_BOTH_EDGE,
                      GPIO_PIN_OUTPUT_HIGH);

    // Interrupt setting
    masks[DIN0 >> 4] |= (1 << (DIN0 & 0x0F));
    masks[DIN1 >> 4] |= (1 << (DIN1 & 0x0F));

    gpio_registerForInterrupt(masks, bleaio_IntCb, NULL);
    gpio_clearPinInterruptStatus(DIN0 >> 4, DIN0 & 0x0F);
    gpio_clearPinInterruptStatus(DIN1 >> 4, DIN1 & 0x0F);
}

void bleaio_IntCb(void *data)
{
    UINT8 button = 0;
    UINT8 pre_button = 0;
    int i;

    //Debounce
    for (i = 0; i < 5; i++) //max delay here is 1ms
    {
        //make delay to read proper value
        extern void bleapputils_Sleep(UINT8 hund_us);
        bleapputils_Sleep(2); //0.2ms

        pre_button = button;
        button = 0;

        if (gpio_getPinInput(DIN0 >> 4, DIN0 & 0x0F))
        {
            //add here if active high

            //add here if active low
            button |= 0x01;
        }
        else
        {
        }

        if (gpio_getPinInput(DIN1 >> 4, DIN1 & 0x0F))
        {
            //add here if active high
        }
        else
        {
            //add here if active low
            button |= 0x02;
        }

        if (pre_button == button)
        {
            break;
        }
    }

    if (button & 0x01)
    {

    }

    if ((button & 0x02) >> 1)
    {

    }

    if (bleprofile_p_cfg->test_enable)
    {
        bleaio_digitalinput(button);
    }
}

UINT8 bleaio_checktrigger(UINT16 value, UINT8 type, UINT8 index)
{
    BLEPROFILE_DB_PDU db_tr_pdu;
    int i, j;

    if (type == DIGITAL)
    {
        for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
        {
            if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                    (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_DIGITAL_INPUT))
            {
                if (bleaio_not_gatt_cfg.tr_hdl[i])
                {
                    bleprofile_ReadHandle(bleaio_not_gatt_cfg.tr_hdl[i], &db_tr_pdu);
                    ble_tracen((char *)db_tr_pdu.pdu, db_tr_pdu.len);

                    //check trigger condition
                    if (db_tr_pdu.len)
                    {
                        if (db_tr_pdu.pdu[0]  == AIO_NO_OPERATION)
                        {
                            return 1; //always triggered
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_NO_IND_TIME)
                        {
                            return 0; //checktimetrigger instead
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_NO_IND_INTERVAL)
                        {
                            return 0; //checktimetrigger instead
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_IND_CHANGED)
                        {
                            return 1; //when changed this function will be called.
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_CHANGED_MORE_OFTEN_THAN)
                        {
                            if (bleaio_not_gatt_cfg.count[i] >
                                    (db_tr_pdu.pdu[1] + ((db_tr_pdu.pdu[2]) << 8)))
                            {
                                bleaio_not_gatt_cfg.count[i] = 0;
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_MASK)
                        {
                            if (db_tr_pdu.pdu[1] & value)
                            {
                                return 1;
                            }
                            return 0; //masked bit map is all 0x00
                        }
                    }
                }
                else
                {
                    return 1; // no trigger descriptor, return as triggered
                }
            }
        }
    }
    else if (type == ANALOG)
    {
        j = 0;
        for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
        {
            if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                    (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_ANALOG_INPUT))
            {
                if (j == index) // find n-th analog input
                {
                    if (bleaio_not_gatt_cfg.tr_hdl[i])
                    {
                        bleprofile_ReadHandle(bleaio_not_gatt_cfg.tr_hdl[i], &db_tr_pdu);
                        ble_tracen((char *)db_tr_pdu.pdu, db_tr_pdu.len);

                        //check trigger condition
                        if (db_tr_pdu.pdu[0] == AIO_NO_OPERATION)
                        {
                            return 1; //always triggered
                        }
                        else if (db_tr_pdu.pdu[0] == AIO_NO_IND_TIME)
                        {
                            return 0; //checktimetrigger instead
                        }
                        else if (db_tr_pdu.pdu[0] == AIO_NO_IND_INTERVAL)
                        {
                            return 0; //checktimetrigger instead
                        }
                        else if (db_tr_pdu.pdu[0] == AIO_IND_CHANGED)
                        {
                            return 1; //when changed this function will be called.
                        }
                        else if (db_tr_pdu.pdu[0] == AIO_LESS_THAN)
                        {
                            if ( value < (db_tr_pdu.pdu[1] + ((db_tr_pdu.pdu[2]) << 8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_LESS_THAN_EQUAL)
                        {
                            if(value <=(db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_GREATER_THAN)
                        {
                            if(value >(db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_GREATER_THAN_EQUAL)
                        {
                            if( value >=(db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_EQUAL)
                        {
                            if(value == (db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_NOT_EQUAL)
                        {
                            if(value != (db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_CHANGED_MORE_THAN)
                        {
                            if(bleaio_not_gatt_cfg.count[i] > (db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                        else if (db_tr_pdu.pdu[0]  == AIO_CHANGED_MORE_OFTEN_THAN)
                        {
                            if(bleaio_not_gatt_cfg.count[i] > (db_tr_pdu.pdu[1]+((db_tr_pdu.pdu[2])<<8)))
                            {
                                bleaio_not_gatt_cfg.count[i] = 0; //reset
                                return 1;
                            }
                            else
                            {
                                return 0;
                            }
                        }
                    }
                    else
                    {
                        return 1; // no trigger descriptor, return as triggered
                    }
                }

                j++;
            }
        }
    }


    return 1; //nothing detected return 1;
}

void bleaio_checktimetrigger(void)
{
    BLEPROFILE_DB_PDU db_pdu, db_cl_pdu;
    UINT8 sent=0;
    int i, j;

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        ble_trace0("tick");
        if (bleaio_not_gatt_cfg.tick[i])
        {
            ble_trace0("ok");
            bleaio_not_gatt_cfg.tick[i]--;

            if (bleaio_not_gatt_cfg.tick[i]==0)
            {
                //send aggregate notification
                for (j = 0; j<AIO_NOT_HANDLE_NUM_MAX; j++)
                {
                    if ((bleaio_not_gatt_cfg.serv[j] == UUID_SERVICE_AUTOMATION_IO) &&
                            (bleaio_not_gatt_cfg.cha[j]  == UUID_CHARACTERISTIC_AGGREGATE_INPUT))
                    {
                        bleprofile_ReadHandle(bleaio_not_gatt_cfg.hdl[j], &db_pdu);
                        ble_tracen((char *)db_pdu.pdu, db_pdu.len);

                        bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[j], &db_cl_pdu);
                        ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                        if (db_cl_pdu.len == 2)
                        {
                            if (db_cl_pdu.pdu[0] & CCC_NOTIFICATION)
                            {
                                bleprofile_sendNotification(bleaio_not_gatt_cfg.hdl[j],
                                                            (UINT8 *)db_pdu.pdu, db_pdu.len);
                                sent = 1;
                            }
                            else if (db_cl_pdu.pdu[0] & CCC_INDICATION)
                            {
                                if (bleaio_indication_sent==0)
                                {
                                    bleprofile_sendIndication(bleaio_not_gatt_cfg.hdl[j],
                                                              (UINT8 *)db_pdu.pdu, db_pdu.len, bleaio_IndicationConf);
                                }
                                sent = 1;
                            }
                        }
                    }
                }

                //send individial notification
                if (sent==0)
                {
                    ble_trace0("Send individual notif");
                    bleprofile_ReadHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
                    ble_tracen((char *)db_pdu.pdu, db_pdu.len);

                    bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                    ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                    if (db_cl_pdu.len == 2)
                    {
                        if (db_cl_pdu.pdu[0] & CCC_NOTIFICATION)
                        {
                            bleprofile_sendNotification(bleaio_not_gatt_cfg.hdl[i],
                                                        (UINT8 *)db_pdu.pdu, db_pdu.len);
                        }
                        else if (db_cl_pdu.pdu[0] & CCC_INDICATION)
                        {
                            if (bleaio_indication_sent==0)
                            {
                                bleprofile_sendIndication(bleaio_not_gatt_cfg.hdl[i],
                                                          (UINT8 *)db_pdu.pdu, db_pdu.len, bleaio_IndicationConf);
                            }
                        }
                    }
                }

                //reset timer setting
                bleaio_resettimetrigger(i);
            }
        }
    }
}

void bleaio_resettimetrigger(UINT8 index)
{
    //reset timer setting
    bleaio_not_gatt_cfg.tick[index] = bleaio_not_gatt_cfg.timeout[index];
}

void bleaio_stoptimetrigger(UINT8 index)
{
    //reset timer setting
    bleaio_not_gatt_cfg.tick[index] = 0;
}

void bleaio_analoginput(UINT8 index, UINT16 value)
{
    BLEPROFILE_DB_PDU db_pdu, db_cl_pdu;
    UINT8 trigger = 0;
    UINT8 sent=0;
    int i, j;

    j = 0;
    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_ANALOG_INPUT))
        {
            if (j == index) // find n-th analog input
            {
                //update count
                (bleaio_not_gatt_cfg.count[i])++;
            }
        }

        j++;
    }

    trigger = bleaio_checktrigger(value, ANALOG, index);

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_AGGREGATE_INPUT))
        {
            bleprofile_ReadHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
            if (db_pdu.len == DIN_BYTE + AIN_BYTE)
            {
                BT_MEMCPY(&(db_pdu.pdu[DIN_BYTE + index * 2]), &value, 2); //analog input is 2bytes
            }

            bleprofile_WriteHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);

            if (trigger)
            {
                bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                if (db_cl_pdu.len == 2)
                {
                    if (db_cl_pdu.pdu[0] & CCC_NOTIFICATION)
                    {
                        bleprofile_sendNotification(bleaio_not_gatt_cfg.hdl[i],
                                                    (UINT8 *)db_pdu.pdu, db_pdu.len);
                        sent = 1;
                    }
                    else if (db_cl_pdu.pdu[0] & CCC_INDICATION)
                    {
                        if (bleaio_indication_sent == 0)
                        {
                            bleprofile_sendIndication(bleaio_not_gatt_cfg.hdl[i],
                                                      (UINT8 *)db_pdu.pdu, db_pdu.len, bleaio_IndicationConf);
                        }
                        sent = 1;
                    }
                }
            }
        }
    }

    j = 0;
    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_ANALOG_INPUT))
        {
            if (j == index) // find n-th analog input
            {
                db_pdu.len    = 2;
                db_pdu.pdu[0] = value & 0xFF;
                db_pdu.pdu[1] = (value & 0xFF00) >> 8;
                bleprofile_WriteHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
                ble_tracen((char *)db_pdu.pdu, db_pdu.len);

                if (trigger && (sent == 0))
                {
                    bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                    ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                    if (db_cl_pdu.len == 2)
                    {
                        if (db_cl_pdu.pdu[0] & CCC_NOTIFICATION)
                        {
                            bleprofile_sendNotification(bleaio_not_gatt_cfg.hdl[i],
                                                        (UINT8 *)db_pdu.pdu, db_pdu.len);
                            sent = 1;
                        }
                        else if (db_cl_pdu.pdu[0] & CCC_INDICATION)
                        {
                            if (bleaio_indication_sent==0)
                            {
                                bleprofile_sendIndication(bleaio_not_gatt_cfg.hdl[i],
                                                          (UINT8 *)db_pdu.pdu, db_pdu.len, bleaio_IndicationConf);
                            }
                            sent = 1;
                        }
                    }
                }

                if (sent)
                {
                    //reset timer setting
                    bleaio_resettimetrigger(i);
                }
            }

            j++;
        }
    }
}

void bleaio_digitalinput(UINT8 value)
{
    BLEPROFILE_DB_PDU db_pdu, db_cl_pdu;
    UINT8 trigger=0;
    UINT8 sent=0;
    int i;

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_DIGITAL_INPUT))
        {
            //update count
            (bleaio_not_gatt_cfg.count[i])++;
        }
    }

    trigger = bleaio_checktrigger((UINT16)value, DIGITAL, 0);

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_AGGREGATE_INPUT))
        {
            bleprofile_ReadHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
            if (db_pdu.len == DIN_BYTE + AIN_BYTE)
            {
                BT_MEMCPY(&(db_pdu.pdu[0]), &value, DIN_BYTE);
            }

            bleprofile_WriteHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);

            if (trigger)
            {
                bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                if (db_cl_pdu.len == 2)
                {
                    if (db_cl_pdu.pdu[0] & CCC_NOTIFICATION)
                    {
                        bleprofile_sendNotification(bleaio_not_gatt_cfg.hdl[i],
                                                    (UINT8 *)db_pdu.pdu, db_pdu.len);
                        sent = 1;
                    }
                    else if (db_cl_pdu.pdu[0] & CCC_INDICATION)
                    {
                        if (bleaio_indication_sent == 0)
                        {
                            bleprofile_sendIndication(bleaio_not_gatt_cfg.hdl[i],
                                                      (UINT8 *)db_pdu.pdu, db_pdu.len, bleaio_IndicationConf);
                        }
                        sent = 1;
                    }
                }
            }
        }
    }

    for (i = 0; i < AIO_NOT_HANDLE_NUM_MAX; i++)
    {
        if ((bleaio_not_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                (bleaio_not_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_DIGITAL_INPUT))
        {
            db_pdu.len    = 1;
            db_pdu.pdu[0] = value;
            bleprofile_WriteHandle(bleaio_not_gatt_cfg.hdl[i], &db_pdu);
            ble_tracen((char *)db_pdu.pdu, db_pdu.len);

            if (trigger && (sent == 0))
            {
                bleprofile_ReadHandle(bleaio_not_gatt_cfg.cl_hdl[i], &db_cl_pdu);
                ble_tracen((char *)db_cl_pdu.pdu, db_cl_pdu.len);

                if (db_cl_pdu.len == 2)
                {
                    if (db_cl_pdu.pdu[0] & CCC_NOTIFICATION)
                    {
                        bleprofile_sendNotification(bleaio_not_gatt_cfg.hdl[i],
                                                    (UINT8 *)db_pdu.pdu, db_pdu.len);
                        sent = 1;
                    }
                    else if (db_cl_pdu.pdu[0] & CCC_INDICATION)
                    {
                        if (bleaio_indication_sent == 0)
                        {
                            bleprofile_sendIndication(bleaio_not_gatt_cfg.hdl[i],
                                                      (UINT8 *)db_pdu.pdu, db_pdu.len, bleaio_IndicationConf);
                        }
                        sent = 1;
                    }
                }
            }

            if (sent)
            {
                //reset timer setting
                bleaio_resettimetrigger(i);
            }
        }
    }
}

void bleaio_output(UINT16 handle)
{
    BLEPROFILE_DB_PDU db_pdu;
    int output;
    int i;

    bleprofile_ReadHandle(handle, &db_pdu);

    for (i = 0; i < AIO_HANDLE_NUM_MAX; i++)
    {
        if (bleaio_gatt_cfg.hdl[i] == handle)
        {
            if ((bleaio_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                    (bleaio_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_DIGITAL_OUTPUT))
            {
                BT_MEMCPY(&output, (char *)db_pdu.pdu, db_pdu.len);
                ble_trace1("Digital output:%02x", output);
                output = output&(0x04|0x01); //change polarity

                gpio_setPinOutput(DOUT0 >> 4, DOUT0 & 0x0F, output & 0x01);
                gpio_setPinOutput(DOUT1 >> 4, DOUT1 & 0x0F, (output & 0x04) >> 2);
            }
            else if ((bleaio_gatt_cfg.serv[i] == UUID_SERVICE_AUTOMATION_IO) &&
                     (bleaio_gatt_cfg.cha[i]  == UUID_CHARACTERISTIC_ANALOG_OUTPUT))
            {
                BT_MEMCPY(&output, (char *)db_pdu.pdu, db_pdu.len);
                ble_trace1("Analog output:%d", output);

                //find index
                {
                    int j;
                    UINT8 count = 0;
                    for (j = 0; j < i; j++)
                    {
                        if ((bleaio_gatt_cfg.serv[j] == UUID_SERVICE_AUTOMATION_IO) &&
                                (bleaio_gatt_cfg.cha[j]  == UUID_CHARACTERISTIC_ANALOG_OUTPUT))
                        {
                            count++;
                        }
                    }
                    {
                        UINT32 set_steps = ((UINT16)output) * AIO_PWM_STEPS / 0xFFFF;
                        UINT8 gpio;

                        ble_trace1("count = %d", count);

                        if (count == 0)
                        {
                            gpio = AIO_PWM1;
                        }
                        if (count == 1)
                        {
                            gpio = AIO_PWM2;
                        }
                        if (count == 2)
                        {
                            gpio = AIO_PWM3;
                        }
                        bleaio_pwm_set(gpio, AIO_PWM_STEPS, (UINT16)set_steps);
                    }
                }
            }
        }
    }
}

void bleaio_IndicationConf(void)
{
    if (bleaio_indication_sent)
    {
        bleaio_indication_sent = 0;
    }
    else
    {
        ble_trace0("AIO Wrong Confirmation!!!");
        return;
    }
}

void bleaio_pwm_init(UINT8 gpio, UINT8 clock, UINT16 total_step, UINT16 toggle_step)
{
    UINT16 init_value = 0x3FF - total_step;
    UINT16 toggle_val = 0x3FF - toggle_step;

    // toggle_step / total_step will be the ducy cycle

    pwm_setInversion(gpio - AIO_PWM_BASE, 1);
    pwm_start(gpio - AIO_PWM_BASE, clock,toggle_val,init_value);
}

void bleaio_pwm_set(UINT8 gpio, UINT16 total_step, UINT16 toggle_step)
{
    UINT16 init_value = 0x3FF - total_step;
    UINT16 toggle_val = 0x3FF - toggle_step;

    ble_trace2("total=%d, toggle=%d\n", total_step, toggle_step);
    ble_trace2("toggle=%d, init=%d\n", toggle_val, init_value);
    pwm_transitionToSubstituteValues(gpio - AIO_PWM_BASE, toggle_val, init_value);
}

void bleaio_pwm_off(UINT8 gpio)
{
    // turn off PWM
    pwm_disableChannel(1 << (gpio - AIO_PWM_BASE));
}

// Stop connection idle timer if it is running
void bleprofile_StopConnIdleTimer(void)
{
    if(emconinfo_getAppTimerId() >= 0)
    {
        blecm_stopConnIdleTimer();
        emconinfo_setAppTimerId(-1);
        ble_trace0("profile:idletimer stopped\n");
    }
}

// Start connection idle timer if it is not running
void bleprofile_StartConnIdleTimer(UINT8 timeout, BLEAPP_TIMER_CB cb)
{
    if (emconinfo_getAppTimerId() < 0)
    {
        emconinfo_setIdleConnTimeout(timeout);
        blecm_startConnIdleTimer(cb);
        ble_trace1("profile:start idletimer %d\n", timeout);
    }
}

// start or stop idle timer depending on if there is somebody registered for nitifications
void bleprofile_setidletimer_withNotification(void)
{
    // idle timer on/off is based on the last notification setting
    // We can decide notification with this result
    if (legattdb_checkNotificationDb())
    {
        //turn off idle timer
        bleprofile_StopConnIdleTimer();
    }
    else
    {
        //turn on idle timer
        bleprofile_StartConnIdleTimer(bleprofile_p_cfg->con_idle_timeout, bleprofile_appTimerCb);
    }
}
