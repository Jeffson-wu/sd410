
/************************************************************************************
 *
 *  Filename:      blemeshtest.c
 *
 *  Description:   Bluetooth Mesh Test application
 *
 ***********************************************************************************/
#define LOG_TAG "blemeshtest"
#include <utils/Log.h>

#include <stdio.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <sys/capability.h>
#include <assert.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

#include <private/android_filesystem_config.h>
#include <android/log.h>

#include <hardware/hardware.h>
#include <hardware/bluetooth.h>


/************************************************************************************
**  Constants & Macros
************************************************************************************/

#define MAX(a,b) (((a)>(b))?(a):(b))

#define TAG "ble_mesh_test"

#define DEVICE_DISCOVERY_TIMEOUT 20

static esp_ble_mesh_model_pub_t gen_onoff_pub = {
    .msg = NET_BUF_SIMPLE(2 + 1),
    .update = NULL,
};

static esp_ble_mesh_model_op_t gen_onoff_op[] = {
        { ESP_BLE_MESH_MODEL_OP_2(0x82, 0x01), 0, 0},
        { ESP_BLE_MESH_MODEL_OP_2(0x82, 0x02), 2, 0},
        { ESP_BLE_MESH_MODEL_OP_2(0x82, 0x03), 2, 0},
        /* Each model operation struct array must use this terminator 
         * as the end tag of the operation uint. */
        ESP_BLE_MESH_MODEL_OP_END,
    };

/* Define the SIG light onoff model */
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_SIG_MODEL(ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_op,
    &gen_onoff_pub, NULL),
};

/* Define the vendor light model operation */
static esp_ble_mesh_model_op_t light_op[] = {
    {ESP_BLE_MESH_MODEL_OP_3(0x01, 0x0D01), 1, 0}, //BRIGHINESS_GET
    {ESP_BLE_MESH_MODEL_OP_3(0x10, 0x0D10), 1, 0}, //BRIGHINESS_GET_RESULT
    {ESP_BLE_MESH_MODEL_OP_3(0x08, 0x0D08), 1, 0}, //BRIGHINESS_SET
    /* Each model operation struct array must use this terminator 
     * as the end tag of the operation uint. */
    ESP_BLE_MESH_MODEL_OP_END,
};

/* Define the vendor light model, the componey id is 0x01, and the model id is 0x7802 */
static esp_ble_mesh_model_t light_model[] = {
    ESP_BLE_MESH_VENDOR_MODEL(0x01, 0x7802, light_op, NULL, NULL),
};

/* There are two element will register to the stack, one is the sig onoff model, the
   other is vendor light model */
static esp_ble_mesh_elem_t element[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, ESP_BLE_MESH_MODEL_NONE, light_model),
};

static const esp_ble_mesh_comp_t comp = {
    .cid = 0,
    .elements = element,
    .element_count = MESH_ARRAY_SIZE(element),
};

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t prov = {
         .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_actions = ESP_BLE_MESH_PUSH,
    .input_size = 4,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};
/************************************************************************************
**  Local type definitions
************************************************************************************/

/************************************************************************************
**  Static variables
************************************************************************************/
static unsigned char main_done = 0;
static bt_status_t status;

/* Main API */
static bluetooth_device_t* bt_device;

const bt_interface_t* sBtInterface = NULL;

static gid_t groups[] = { AID_NET_BT, AID_INET, AID_NET_BT_ADMIN,
                          AID_SYSTEM, AID_MISC, AID_SDCARD_RW,
                          AID_NET_ADMIN, AID_VPN};

/* Set to 1 when the Bluedroid stack is enabled */
static unsigned char bt_enabled = 0;
static int deviceCount;
static int wantMore = 0;
pthread_mutex_t deviceCount_mutex;
pthread_cond_t deviceCount_cond;

/************************************************************************************
**  Static functions
************************************************************************************/

static void process_cmd(char *p, unsigned char is_job);
static void job_handler(void *param);
static void bdt_log(const char *fmt_str, ...);
static void discover_device(void *arg);


/************************************************************************************
**  Externs
************************************************************************************/

/************************************************************************************
**  Functions
************************************************************************************/


/************************************************************************************
**  Shutdown helper functions
************************************************************************************/

static void bdt_shutdown(void)
{
    bdt_log("shutdown bdroid test app\n");
    main_done = 1;
}


/*****************************************************************************
** Android's init.rc does not yet support applying linux capabilities
*****************************************************************************/

static void config_permissions(void)
{
    struct __user_cap_header_struct header;
    struct __user_cap_data_struct cap;

    bdt_log("set_aid_and_cap : pid %d, uid %d gid %d", getpid(), getuid(), getgid());

    header.pid = 0;

    prctl(PR_SET_KEEPCAPS, 1, 0, 0, 0);

    setuid(AID_BLUETOOTH);
    setgid(AID_BLUETOOTH);

    header.version = _LINUX_CAPABILITY_VERSION;

    cap.effective = cap.permitted =  cap.inheritable =
                    1 << CAP_NET_RAW |
                    1 << CAP_NET_ADMIN |
                    1 << CAP_NET_BIND_SERVICE |
                    1 << CAP_SYS_RAWIO |
                    1 << CAP_SYS_NICE |
                    1 << CAP_SETGID;

    capset(&header, &cap);
    setgroups(sizeof(groups)/sizeof(groups[0]), groups);
}



/*****************************************************************************
**   Logger API
*****************************************************************************/

void bdt_log(const char *fmt_str, ...)
{
    static char buffer[1024];
    va_list ap;

    va_start(ap, fmt_str);
    vsnprintf(buffer, 1024, fmt_str, ap);
    va_end(ap);

    fprintf(stdout, "%s\n", buffer);
}

/*******************************************************************************
 ** Misc helper functions
 *******************************************************************************/
static const char* dump_bt_status(bt_status_t status)
{
#if 0
    	switch(status)
    {
        CASE_RETURN_STR(BT_STATUS_SUCCESS)
        CASE_RETURN_STR(BT_STATUS_FAIL)
        CASE_RETURN_STR(BT_STATUS_NOT_READY)
        CASE_RETURN_STR(BT_STATUS_NOMEM)
        CASE_RETURN_STR(BT_STATUS_BUSY)
        CASE_RETURN_STR(BT_STATUS_UNSUPPORTED)

        default:
            return "unknown status code";
    }
#endif
  return;
}

static void hex_dump(char *msg, void *data, int size, int trunc)
{
    unsigned char *p = data;
    unsigned char c;
    int n;
    char bytestr[4] = {0};
    char addrstr[10] = {0};
    char hexstr[ 16*3 + 5] = {0};
    char charstr[16*1 + 5] = {0};

    bdt_log("%s  \n", msg);

    /* truncate */
    if(trunc && (size>32))
        size = 32;

    for(n=1;n<=size;n++) {
        if (n%16 == 1) {
            /* store address for this line */
            snprintf(addrstr, sizeof(addrstr), "%.4x",
               (unsigned int)((uintptr_t)p-(uintptr_t)data) );
        }

        c = *p;
        if (isalnum(c) == 0) {
            c = '.';
        }

        /* store hex str (for left side) */
        snprintf(bytestr, sizeof(bytestr), "%02X ", *p);
        strncat(hexstr, bytestr, sizeof(hexstr)-strlen(hexstr)-1);

        /* store char str (for right side) */
        snprintf(bytestr, sizeof(bytestr), "%c", c);
        strncat(charstr, bytestr, sizeof(charstr)-strlen(charstr)-1);

        if(n%16 == 0) {
            /* line completed */
            bdt_log("[%4.4s]   %-50.50s  %s\n", addrstr, hexstr, charstr);
            hexstr[0] = 0;
            charstr[0] = 0;
        } else if(n%8 == 0) {
            /* half line: add whitespaces */
            strncat(hexstr, "  ", sizeof(hexstr)-strlen(hexstr)-1);
            strncat(charstr, " ", sizeof(charstr)-strlen(charstr)-1);
        }
        p++; /* next byte */
    }

    if (strlen(hexstr) > 0) {
        /* print rest of buffer if not empty */
        bdt_log("[%4.4s]   %-50.50s  %s\n", addrstr, hexstr, charstr);
    }
}

/*******************************************************************************
 ** Console helper functions
 *******************************************************************************/

void skip_blanks(char **p)
{
  while (**p == ' ')
    (*p)++;
}

uint32_t get_int(char **p, int DefaultValue)
{
  uint32_t Value = 0;
  unsigned char   UseDefault;

  UseDefault = 1;
  skip_blanks(p);

  while ( ((**p)<= '9' && (**p)>= '0') )
    {
      Value = Value * 10 + (**p) - '0';
      UseDefault = 0;
      (*p)++;
    }

  if (UseDefault)
    return DefaultValue;
  else
    return Value;
}

int get_signed_int(char **p, int DefaultValue)
{
  int    Value = 0;
  unsigned char   UseDefault;
  unsigned char  NegativeNum = 0;

  UseDefault = 1;
  skip_blanks(p);

  if ( (**p) == '-')
    {
      NegativeNum = 1;
      (*p)++;
    }
  while ( ((**p)<= '9' && (**p)>= '0') )
    {
      Value = Value * 10 + (**p) - '0';
      UseDefault = 0;
      (*p)++;
    }

  if (UseDefault)
    return DefaultValue;
  else
    return ((NegativeNum == 0)? Value : -Value);
}

void get_str(char **p, char *Buffer)
{
  skip_blanks(p);

  while (**p != 0 && **p != ' ')
    {
      *Buffer = **p;
      (*p)++;
      Buffer++;
    }

  *Buffer = 0;
}

uint32_t get_hex(char **p, int DefaultValue)
{
  uint32_t Value = 0;
  unsigned char   UseDefault;

  UseDefault = 1;
  skip_blanks(p);

  while ( ((**p)<= '9' && (**p)>= '0') ||
          ((**p)<= 'f' && (**p)>= 'a') ||
          ((**p)<= 'F' && (**p)>= 'A') )
    {
      if (**p >= 'a')
        Value = Value * 16 + (**p) - 'a' + 10;
      else if (**p >= 'A')
        Value = Value * 16 + (**p) - 'A' + 10;
      else
        Value = Value * 16 + (**p) - '0';
      UseDefault = 0;
      (*p)++;
    }

  if (UseDefault)
    return DefaultValue;
  else
    return Value;
}

void get_bdaddr(const char *str, bt_bdaddr_t *bd) {
    char *d = ((char *)bd), *endp;
    int i;
    for(i = 0; i < 6; i++) {
        *d++ = strtol(str, &endp, 16);
        if (*endp != ':' && i != 5) {
            memset(bd, 0, sizeof(bt_bdaddr_t));
            return;
        }
        str = endp + 1;
    }
}

#define is_cmd(str) ((strlen(str) == strlen(cmd)) && strncmp((const char *)&cmd, str, strlen(str)) == 0)
#define if_cmd(str)  if (is_cmd(str))

typedef void (t_console_cmd_handler) (char *p);

typedef struct {
    const char *name;
    t_console_cmd_handler *handler;
    const char *help;
    unsigned char is_job;
} t_cmd;


const t_cmd console_cmd_list[];
static int console_cmd_maxlen = 0;

static void cmdjob_handler(void *param)
{
    char *job_cmd = (char*)param;

    bdt_log("cmdjob starting (%s)", job_cmd);

    process_cmd(job_cmd, 1);

    bdt_log("cmdjob terminating");

    free(job_cmd);
}

static int create_cmdjob(char *cmd)
{
    pthread_t thread_id;
    char *job_cmd;

    job_cmd = malloc(strlen(cmd)+1); /* freed in job handler */
    if (job_cmd)
    {
        strcpy(job_cmd, cmd);

        if (pthread_create(&thread_id, NULL,
                           (void*)cmdjob_handler, (void*)job_cmd)!=0)
        perror("pthread_create");
    }
    else
    {
        perror("create_cmdjob(): Failed to allocate memory");
    }
    return 0;
}

/*******************************************************************************
 ** Load stack lib
 *******************************************************************************/

int HAL_load(void)
{
    int err = 0;

    hw_module_t* module;
    hw_device_t* device;

    bdt_log("Loading HAL lib + extensions");

    err = hw_get_module(BT_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
    if (err == 0)
    {
        err = module->methods->open(module, BT_HARDWARE_MODULE_ID, &device);
        if (err == 0) {
            bt_device = (bluetooth_device_t *)device;
            sBtInterface = bt_device->get_bluetooth_interface();
        }
    }

    bdt_log("HAL library loaded (%s)", strerror(err));

    return err;
}

int HAL_unload(void)
{
    int err = 0;

    bdt_log("Unloading HAL lib");

    sBtInterface = NULL;

    bdt_log("HAL library unloaded (%s)", strerror(err));

    return err;
}

/*******************************************************************************
 ** HAL test functions & callbacks
 *******************************************************************************/

void setup_test_env(void)
{
    int i = 0;

    while (console_cmd_list[i].name != NULL)
    {
        console_cmd_maxlen = MAX(console_cmd_maxlen, (int)strlen(console_cmd_list[i].name));
        i++;
    }
}

void check_return_status(bt_status_t status)
{
    if (status != BT_STATUS_SUCCESS)
    {
        bdt_log("HAL REQUEST FAILED status : %d (%s)", status, dump_bt_status(status));
    }
    else
    {
        bdt_log("HAL REQUEST SUCCESS");
    }
}

static void adapter_state_changed(bt_state_t state)
{
    bdt_log("ADAPTER STATE UPDATED : %s", (state == BT_STATE_OFF)?"OFF":"ON");
    if (state == BT_STATE_ON) {
        bt_enabled = 1;
    } else {
        bt_enabled = 0;
    }
}

static void dut_mode_recv(uint16_t opcode, uint8_t *buf, uint8_t  len)
{
    bdt_log("DUT MODE RECV : NOT IMPLEMENTED");
}

static void le_test_mode(bt_status_t status, uint16_t packet_count)
{
    bdt_log("LE TEST MODE END status:%s number_of_packets:%d", dump_bt_status(status), packet_count);
}

static void device_found_cb(int num_properties, bt_property_t *properties)
{
    int i;
    for (i = 0; i < num_properties; i++)
    {
        if (properties[i].type == BT_PROPERTY_BDNAME)
        {
            pthread_mutex_lock(&deviceCount_mutex);
            deviceCount++;
            bdt_log("Device name is : %s\n",
                  (char*)properties[i].val);
            if (deviceCount > 0 && wantMore == 0)
            {
                pthread_cond_signal(&deviceCount_cond);
            }
            pthread_mutex_unlock(&deviceCount_mutex);
        }
    }
}

static bt_callbacks_t bt_callbacks = {
    sizeof(bt_callbacks_t),
    adapter_state_changed,
    NULL, /* adapter_properties_cb */
    NULL, /* remote_device_properties_cb */
    device_found_cb, /* device_found_cb */
    NULL, /* discovery_state_changed_cb */
    NULL, /* pin_request_cb  */
    NULL, /* ssp_request_cb  */
    NULL, /* bond_state_changed_cb */
    NULL, /* acl_state_changed_cb */
    NULL, /* thread_evt_cb */
    dut_mode_recv, /* dut_mode_recv_cb */
#if BLE_INCLUDED == TRUE
    le_test_mode, /* le_test_mode_cb */
#else
    NULL, /* le_test_mode_cb */
#endif
    NULL, /* energy_info_cb */
    NULL, /*le_lpp_write_rssi_thresh_cb*/
    NULL, /*le_lpp_read_rssi_thresh_cb*/
    NULL, /*le_lpp_enable_rssi_monitor_cb*/
    NULL, /*le_lpp_rssi_threshold_evt_cb*/
    NULL  /* hci_event_recv_cb */
};

static bool set_wake_alarm(uint64_t delay_millis, bool should_wake, alarm_cb cb, void *data) {
  static timer_t timer;
  static bool timer_created;

  if (!timer_created) {
    struct sigevent sigevent;
    memset(&sigevent, 0, sizeof(sigevent));
    sigevent.sigev_notify = SIGEV_THREAD;
    sigevent.sigev_notify_function = (void (*)(union sigval))cb;
    sigevent.sigev_value.sival_ptr = data;
    timer_create(CLOCK_MONOTONIC, &sigevent, &timer);
    timer_created = true;
  }

  struct itimerspec new_value;
  new_value.it_value.tv_sec = delay_millis / 1000;
  new_value.it_value.tv_nsec = (delay_millis % 1000) * 1000 * 1000;
  new_value.it_interval.tv_sec = 0;
  new_value.it_interval.tv_nsec = 0;
  timer_settime(timer, 0, &new_value, NULL);

  return true;
}

static int acquire_wake_lock(const char *lock_name) {
  return BT_STATUS_SUCCESS;
}

static int release_wake_lock(const char *lock_name) {
  return BT_STATUS_SUCCESS;
}

static bt_os_callouts_t callouts = {
    sizeof(bt_os_callouts_t),
    set_wake_alarm,
    acquire_wake_lock,
    release_wake_lock,
};

void bdt_init(void)
{
    bdt_log("INIT BT ");
    status = sBtInterface->init(&bt_callbacks);

    if (status == BT_STATUS_SUCCESS) {
        status = sBtInterface->set_os_callouts(&callouts);
    }

    check_return_status(status);
}

void bdt_enable(void)
{
    bdt_log("ENABLE BT");
    if (bt_enabled) {
        bdt_log("Bluetooth is already enabled");
        return;
    }
    status = sBtInterface->enable();

    check_return_status(status);
}

void bdt_disable(void)
{
    bdt_log("DISABLE BT");
    if (!bt_enabled) {
        bdt_log("Bluetooth is already disabled");
        return;
    }
    status = sBtInterface->disable();

    check_return_status(status);
}
void bdt_dut_mode_configure(char *p)
{
    int32_t mode = -1;

    bdt_log("BT DUT MODE CONFIGURE");
    if (!bt_enabled) {
        bdt_log("Bluetooth must be enabled for test_mode to work.");
        return;
    }
    mode = get_signed_int(&p, mode);
    if ((mode != 0) && (mode != 1)) {
        bdt_log("Please specify mode: 1 to enter, 0 to exit");
        return;
    }
    status = sBtInterface->dut_mode_configure(mode);

    check_return_status(status);
}

#define HCI_LE_RECEIVER_TEST_OPCODE 0x201D
#define HCI_LE_TRANSMITTER_TEST_OPCODE 0x201E
#define HCI_LE_END_TEST_OPCODE 0x201F

void bdt_le_test_mode(char *p)
{
    int cmd;
    unsigned char buf[3];
    int arg1, arg2, arg3;

    bdt_log("BT LE TEST MODE");
    if (!bt_enabled) {
        bdt_log("Bluetooth must be enabled for le_test to work.");
        return;
    }

    memset(buf, 0, sizeof(buf));
    cmd = get_int(&p, 0);
    switch (cmd)
    {
        case 0x1: /* RX TEST */
           arg1 = get_int(&p, -1);
           if (arg1 < 0) bdt_log("%s Invalid arguments", __FUNCTION__);
           buf[0] = arg1;
           status = sBtInterface->le_test_mode(HCI_LE_RECEIVER_TEST_OPCODE, buf, 1);
           break;
        case 0x2: /* TX TEST */
            arg1 = get_int(&p, -1);
            arg2 = get_int(&p, -1);
            arg3 = get_int(&p, -1);
            if ((arg1 < 0) || (arg2 < 0) || (arg3 < 0))
                bdt_log("%s Invalid arguments", __FUNCTION__);
            buf[0] = arg1;
            buf[1] = arg2;
            buf[2] = arg3;
            status = sBtInterface->le_test_mode(HCI_LE_TRANSMITTER_TEST_OPCODE, buf, 3);
           break;
        case 0x3: /* END TEST */
            status = sBtInterface->le_test_mode(HCI_LE_END_TEST_OPCODE, buf, 0);
           break;
        default:
            bdt_log("Unsupported command");
            return;
            break;
    }
    if (status != BT_STATUS_SUCCESS)
    {
        bdt_log("%s Test 0x%x Failed with status:0x%x", __FUNCTION__, cmd, status);
    }
    return;
}

void bdt_cleanup(void)
{
    bdt_log("CLEANUP");
    sBtInterface->cleanup();
}

/*******************************************************************************
 ** Console commands
 *******************************************************************************/

void do_help(char *p)
{
    int i = 0;
    int max = 0;
    char line[128];
    int pos = 0;

    while (console_cmd_list[i].name != NULL)
    {
        pos = sprintf(line, "%s", (char*)console_cmd_list[i].name);
        bdt_log("%s %s\n", (char*)line, (char*)console_cmd_list[i].help);
        i++;
    }
}

void do_quit(char *p)
{
    bdt_shutdown();
}

/*******************************************************************
 *
 *  BT TEST  CONSOLE COMMANDS
 *
 *  Parses argument lists and passes to API test function
 *
*/

void do_init(char *p)
{
    bdt_init();
}

void do_enable(char *p)
{
    bdt_enable();
}

void do_disable(char *p)
{
    bdt_disable();
}
void do_dut_mode_configure(char *p)
{
    bdt_dut_mode_configure(p);
}

void do_le_test_mode(char *p)
{
    bdt_le_test_mode(p);
}

void do_cleanup(char *p)
{
    bdt_cleanup();
}

/*******************************************************************
 *
 *  CONSOLE COMMAND TABLE
 *
*/

const t_cmd console_cmd_list[] =
{
    /*
     * INTERNAL
     */

    { "help", do_help, "lists all available console commands", 0 },
    { "quit", do_quit, "", 0},

    /*
     * API CONSOLE COMMANDS
     */

     /* Init and Cleanup shall be called automatically */
    { "enable", do_enable, ":: enables bluetooth", 0 },
    { "disable", do_disable, ":: disables bluetooth", 0 },
    { "dut_mode_configure", do_dut_mode_configure, ":: DUT mode - 1 to enter,0 to exit", 0 },
    { "le_test_mode", do_le_test_mode, ":: LE Test Mode - RxTest - 1 <rx_freq>, \n\t \
                      TxTest - 2 <tx_freq> <test_data_len> <payload_pattern>, \n\t \
                      End Test - 3 <no_args>", 0 },
    /* add here */

    /* last entry */
    {NULL, NULL, "", 0},
};

/*
 * Main console command handler
*/

static void process_cmd(char *p, unsigned char is_job)
{
    char cmd[64];
    int i = 0;
    char *p_saved = p;

    get_str(&p, cmd);

    /* table commands */
    while (console_cmd_list[i].name != NULL)
    {
        if (is_cmd(console_cmd_list[i].name))
        {
            if (!is_job && console_cmd_list[i].is_job)
                create_cmdjob(p_saved);
            else
            {
                console_cmd_list[i].handler(p);
            }
            return;
        }
        i++;
    }
    bdt_log("%s : unknown command\n", p_saved);
    do_help(NULL);
}

static void discover_device(void *arg)
{
    struct timespec ts = {0, 0};
    ts.tv_sec = time(NULL) + DEVICE_DISCOVERY_TIMEOUT;

    sBtInterface->start_discovery();
    pthread_mutex_lock(&deviceCount_mutex);
    pthread_cond_timedwait(&deviceCount_cond, &deviceCount_mutex, &ts);
    if (deviceCount == 0)
    {
        bdt_log("No device found\n");
    }
    else
    {
        deviceCount = 0;
    }
    pthread_mutex_unlock(&deviceCount_mutex);
    wantMore = 0;
    bdt_log("Cancelling discovery\n");
    sBtInterface->cancel_discovery();
    pthread_exit(0);
}

static char *esp_ble_mesh_prov_event_to_str(esp_ble_mesh_prov_cb_event_t event)
{
    switch(event) {
        case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
            return "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT";
        case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
            return "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT";
        case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
            return "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT";
        case ESP_BLE_MESH_NODE_PROV_OUTPUT_STRING_EVT:
            return "ESP_BLE_MESH_NODE_PROV_OUTPUT_STRING_EVT";
        case ESP_BLE_MESH_NODE_PROV_INPUT_EVT:
            return "ESP_BLE_MESH_NODE_PROV_INPUT_EVT";
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            return "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT";
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            return "ESP_BLE_MESH_NODE_PROV_RESET_EVT";
            //case ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT:
            //    return "ESP_BLE_MESH_MODEL_PUBLISH_UPDATE_EVT";
            //case ESP_BLE_MESH_MODEL_OPERATION_EVT:
            //    return "ESP_BLE_MESH_MODEL_OPERATION_EVT";
        default:
            return "Invalid mesh provision event.";
    }

    return NULL;
}

static void esp_mesh_prov_cb(esp_ble_mesh_prov_cb_event_t event, 
                      esp_ble_mesh_prov_cb_param_t *param)
{
    ALOGE(TAG, "%s, event = %s", __func__, esp_ble_mesh_prov_event_to_str(event));
    switch(event) {
        case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
            break;
        case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
            break;
        case ESP_BLE_MESH_NODE_PROV_OUTPUT_STRING_EVT:
            
            break;
        case ESP_BLE_MESH_NODE_PROV_INPUT_EVT:
            break;
        case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
            //prov_complete(param->prov_comp.net_idx,param->prov_comp.addr);
            break;
        case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
            break;
        default:
            break;
    }
    return;
}

void esp_mesh_model_cb(esp_ble_mesh_model_cb_event_t event, 
                       esp_ble_mesh_model_cb_param_t *param)
{
    switch(event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
    break;
    case ESP_BLE_MESH_MODEL_PUBLISH_COMP_EVT:
        break;
    default:
        break;
    }
}

static int test_ble_mesh_init(void)
{
    int err = 0;

    // Register the provisioning callback function
    sBtInterface->ble_mesh_register_prov_callback(esp_mesh_prov_cb);
    bdt_log(TAG, "Register the provisioning callback function\n");
    // Register the model send callback function
    sBtInterface->ble_mesh_register_custom_model_callback(esp_mesh_model_cb);
    //memcpy(dev_uuid, esp_bt_dev_get_address(), 6);
    bdt_log(TAG, "Register the model send callback function\n");

    err = sBtInterface->ble_mesh_init(&prov, &comp);
    if (err) {
        bdt_log(TAG, "Initializing mesh failed (err %d)\n", err);
        return err;
    }

    sBtInterface->ble_mesh_node_prov_enable(/*BT_MESH_PROV_ADV |*/ BT_MESH_PROV_GATT);

    bdt_log(TAG, "Mesh initialized\n");

    return err;
}

void test_main ()
{
    int opt;
    char cmd[128];
    int args_processed = 0;
    int pid = -1;
    int enable_wait_count = 0;
    pthread_t discoveryThread;

    config_permissions();
    bdt_log("\n:::::::::::::::::::::::::::::::::::::::::::::::::::");
    bdt_log(":: Bluetooth mesh test app starting");
#if 1
    if ( HAL_load() < 0 ) {
        perror("HAL failed to initialize, exit\n");
        exit(0);
    }

    setup_test_env();
    pthread_mutex_init(&deviceCount_mutex, NULL);
    pthread_cond_init (&deviceCount_cond, NULL);

    /* Automatically perform the init */
    bdt_init();
    bdt_enable();
    do {
        if (bt_enabled)
         break;
         bdt_log("Waiting for bt_enabled to become true\n");
         sleep(2);
     } while(enable_wait_count++ < 10);
    
    if(bt_enabled)
    {
#endif
    	bdt_log("bluetooth mesh starting");
        test_ble_mesh_init();
        bdt_log("bluetooth mesh ending");
#if 1
    }
    else
    {
	bdt_log("Failed to enable BT\n");
	goto cleanup;
    }
#endif
    /* FIXME: Commenting this out as for some reason, the application does not exit otherwise*/
    //bdt_cleanup();
#if 1
cleanup:
    HAL_unload();

    pthread_mutex_destroy(&deviceCount_mutex);
    pthread_cond_destroy(&deviceCount_cond);
#endif
    bdt_log(":: Bluedroid test app terminating");

    return 0;
}

void main( )
{
    test_main();
//  assert(ret == ESP_OK);
    /** We wait init finish 200ms here */
    const struct bt_mesh_comp *dev_comp;// = bt_mesh_comp_get();
    /* We need to check if the corret model and element values are stored in
       the dev_comp pointer. */
    assert(dev_comp);
    /* We are register the cid = 0 to the stack, so we should check if its value is correct */
    assert(dev_comp->cid == 0);
    /* We are register 2 element, one have root_models and the other have light_model */
    assert(dev_comp->elem_count == 2);
    /* Check the element pointer */
    assert(dev_comp->elem);
    for (int i = 0; i < dev_comp->elem_count; i++) {
        struct bt_mesh_elem *elem = &dev_comp->elem[i];
        /* We are register 1 sig model in it and register 1 vendor model in it */
        assert((elem->model_count == 1) || (elem->vnd_model_count == 1));

        for (int j = 0; j < elem->model_count; j++) {
            struct bt_mesh_model *models = &elem->models[j];
            assert(models);
            /* The model id should equal the onoff server model id */
            assert(models->id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV);
            /* The elem pointer in the models shouldn't NULL */
            assert(models->elem);
            /* The models op pointer shouldn't NULL */
            assert(models->op);
            assert(models->op[0].opcode == ESP_BLE_MESH_MODEL_OP_2(0x82, 0x01));
            assert(models->op[1].opcode == ESP_BLE_MESH_MODEL_OP_2(0x82, 0x02));
            assert(models->op[2].opcode == ESP_BLE_MESH_MODEL_OP_2(0x82, 0x03));
            assert(models->op[0].min_len == 0);
            assert(models->op[1].min_len == 2);
            assert(models->op[2].min_len == 2);
        }
        
        for (int k = 0; k < elem->vnd_model_count; k++) {
            struct bt_mesh_model *vnd_models = &elem->vnd_models[k];
            assert(vnd_models);
            /* The vendor model company id should equal 0x01 */
            assert(vnd_models->vnd.company == 0x01);
            /* The vendor model company id should equal 0 */
            assert(vnd_models->vnd.id == 0x7802);
            /* The elem pointer in the models shouldn't NULL */
            assert(vnd_models->elem);
            /* The models op pointer shouldn't NULL */
            assert(vnd_models->op);
            assert(vnd_models->op[0].opcode == ESP_BLE_MESH_MODEL_OP_3(0x01, 0x0D01));
            assert(vnd_models->op[1].opcode == ESP_BLE_MESH_MODEL_OP_3(0x10, 0x0D10));
            assert(vnd_models->op[2].opcode == ESP_BLE_MESH_MODEL_OP_3(0x08, 0x0D08));
            assert(vnd_models->op[0].min_len == 1);
            assert(vnd_models->op[1].min_len == 1);
            assert(vnd_models->op[2].min_len == 1);
        }
    }
}
