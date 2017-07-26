#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "user_interface.h"
#include "wpa2_enterprise.h"
#include "mem.h"
#include "espconn.h"
#include <string.h>

static const int pin = 2;
bool got_ip = false, now_connected = false;
bool buffer_full = false;
struct espconn *my_connection;

#define BUF_SIZ 512
char buff_send[BUF_SIZ];

#define FRC1_SOURCE 0

#define SIG_RX 0
#define TEST_QUEUE_LEN 1
os_event_t *testQueue;


#define MEASURE_SEND_TIME

#define gpio_high(gpio_nr) gpio_output_set((1 << gpio_nr), 0, 0, 0)
#define gpio_low(gpio_nr) gpio_output_set(0, (1 << gpio_nr), 0, 0);

#define MCU_RESET_ASSERT gpio_low(5)
#define MCU_RESET_DEASSERT gpio_high(5)

#define CS 15
#define MOSI 13
#define MISO 12
#define SCLK 14

void  ICACHE_FLASH_ATTR user_set_station_config(void)
{
	char ssid[32] = "Name_of_AP";
	char password[64] = "WIFI_password";
	struct station_config stationConf;
	
	stationConf.bssid_set = 0; //don't check MAC of AP
	os_memcpy(&stationConf.ssid, ssid, 32);
	os_memcpy(&stationConf.password, password, 64);
	wifi_station_set_config_current(&stationConf);
}

void ICACHE_FLASH_ATTR now_connected_cb(void *arg)
{
	os_printf("Now we are connected to TCP server\n");
	now_connected = true;
	MCU_RESET_DEASSERT;
}

void ICACHE_FLASH_ATTR disconnected_cb(void *arg, sint8 err)
{
	os_printf("Shit's broken %d\n", err);
	espconn_connect(my_connection);
}

void bitbanged_spi_read_buf(uint8_t *buf, uint32_t count)
{
	uint8_t cnt = 0;
	uint8_t read_byte = 0;
	
	gpio_low(CS);

	while(count > 0)
	{
		for(cnt = 8; cnt > 0 ; cnt--)
		{
			gpio_high(SCLK);
			read_byte |= (GPIO_INPUT_GET(MISO) << (cnt - 1 ));
			gpio_low(SCLK);
		}
		*buf = read_byte;
		buf++;
		os_delay_us(8);
		count--;
		read_byte = 0;
	}
	gpio_high(CS);
}

void ICACHE_FLASH_ATTR bitbanged_spi_init()
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(MISO));
	GPIO_OUTPUT_SET(MOSI, 0);
	GPIO_OUTPUT_SET(SCLK, 0);
	GPIO_OUTPUT_SET(CS, 1);
}

void adc_to_wifi(void)
{
	uint8_t audio_data[BUF_SIZ];
	sint8 ret;

	bitbanged_spi_read_buf(audio_data, BUF_SIZ);
	os_memcpy(buff_send, audio_data, BUF_SIZ);

	if(now_connected)
	{
		ret = espconn_send(my_connection, buff_send, BUF_SIZ);
		if((ret != ESPCONN_MAXNUM) && (ret != 0))
		{
			/* This means that our connection broke, but unfortunatelly the
			reconnect callback is not called as stated in the documentation */
			os_printf("Connection broke\n");
			now_connected = false;
			/* Try to connect again, if failing here, the reconnect callback
			function will be called */
			espconn_connect(my_connection);
		}
		else if(ret != 0) /* this condition means that ret can only be ESPCONN_MAXNUM */
			os_printf("espconn_send err is %d\n", ret);
#ifdef MEASURE_SEND_TIME
		gpio_high(10);
#endif
	}
	
}

void gpio_interrupt_handler(void)
{
	uint32 gpio_status;

	system_os_post(USER_TASK_PRIO_0, SIG_RX, 'a');

	gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
	//clear interrupt status
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status);
}

void my_task(os_event_t *e)
{
	static int i = 0;
	uint8_t x;
	uint32_t y = 122;
	switch(e->sig)
	{
		case SIG_RX:
			adc_to_wifi();
			break;
		default:
			break;
	}
}

#ifdef MEASURE_SEND_TIME
void ICACHE_FLASH_ATTR data_sent_cb_function(void)
{
	gpio_low(10);
}
#endif

void ICACHE_FLASH_ATTR serv_tcp_connect(void)
{
	uint32 ip = 0;
	my_connection = (struct espconn *) os_zalloc(sizeof(struct espconn));
	my_connection->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
	my_connection->state = ESPCONN_NONE;

	ip = ipaddr_addr("192.168.0.102");
	os_memcpy(my_connection->proto.tcp->remote_ip, &ip, sizeof(ip));
	my_connection->proto.tcp->local_port = espconn_port();
	my_connection->proto.tcp->remote_port = 1235;

	my_connection->type = ESPCONN_TCP;
	espconn_set_opt(my_connection, ESPCONN_NODELAY); /* disable Nagle algorithm to increase transmit speed */
	os_printf("callback said %d\n", espconn_regist_connectcb(my_connection, now_connected_cb));
	espconn_regist_reconcb(my_connection, disconnected_cb);
	os_printf("connect said %d\n", espconn_connect(my_connection));
#ifdef MEASURE_SEND_TIME
	espconn_regist_sentcb(my_connection, (espconn_sent_callback)data_sent_cb_function);
#endif
}

void ICACHE_FLASH_ATTR wifi_event_handle_cb_func(System_Event_t *evt)
{
	switch(evt->event)
	{
		case EVENT_STAMODE_GOT_IP:
			got_ip = true;
			serv_tcp_connect();
		break;
	}
}

uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void) {
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}


void ICACHE_FLASH_ATTR user_init()
{
	gpio_init();
	bitbanged_spi_init();
	uart_init(115200, 115200);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA3_U, FUNC_GPIO10);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA2_U, FUNC_GPIO9); //adc data ready interrupt
	GPIO_DIS_OUTPUT(GPIO_ID_PIN(9)); // GPIO_9 as input
	ETS_GPIO_INTR_ATTACH((ets_isr_t) gpio_interrupt_handler, NULL);
	gpio_pin_intr_state_set(GPIO_ID_PIN(9), GPIO_PIN_INTR_NEGEDGE);
	ETS_GPIO_INTR_ENABLE();
	
	gpio_output_set(0, 0, (1 << pin), 0);
	gpio_output_set(0, 0, (1 << 10), 0); //enable pin GPIO10 as output
	gpio_high(10);

	gpio_output_set(0, 0, (1 << 5), 0); //enable pin GPIO5 as output
	MCU_RESET_ASSERT;

	os_printf("\nSTART START START START\n");

	wifi_set_opmode_current(STATION_MODE);
	user_set_station_config();
	wifi_set_event_handler_cb(wifi_event_handle_cb_func);

	testQueue=(os_event_t *) os_malloc(sizeof(os_event_t) * TEST_QUEUE_LEN);
				
	system_os_task(my_task, USER_TASK_PRIO_0, testQueue, TEST_QUEUE_LEN);
}
