/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <stdio.h>
#include <modem/lte_lc.h>	  // LTE link control
#include <nrf_modem_at.h>	  // includes AT command handling
#include <modem/modem_info.h> // includes modem info module
#include <net/socket.h>		  // includes TCP/IP socket handling
#include <device.h>			  // Zephyr device API
#include <drivers/sensor.h>	  // Zephyr sensor driver API
#include <drivers/gpio.h>
#include <sys_clock.h>

/* UI drivers for Thingy:91 */
#include "buzzer.h"
#include "ui.h"
#include "led_pwm.h"

/* Defines */
#define MODEM_APN "cdp.iot.t-mobile.nl"
#define TEMP_CALIBRATION_OFFSET 3
#define RECV_BUF_SIZE 2048
#define RCV_POLL_TIMEOUT_MS 1000 /* Milliseconds */

/* Sensor data */
const struct device *bme_680;
struct sensor_value temp, press, humidity, gas_res;
float pressure;

/* GPIO */
const struct device *sx1509b_dev;

#define GPIO_PIN_OPEN_ENDSTOP 15
#define GPIO_PIN_CLOSED_ENDSTOP 14
#define GPIO_PIN_RELAY 13

/* Ultrasonic */
#define GPIO_PIN_ULTRASONIC_TRIG 12
#define GPIO_PIN_ULTRASONIC_ECHO 11

/* UDP Socket */
static int client_fd;
static bool socket_open = false;
static struct sockaddr_storage host_addr;

/* Workqueues */
static struct k_work_delayable server_transmission_work;
static struct k_work_delayable data_fetch_work;
static struct k_work_delayable data_poll_work;
static struct k_work_delayable gpio_fetch_work;

/* Garage Door state */
#define GARAGE_DOOR_OPENING 1
#define GARAGE_DOOR_OPEN 2
#define GARAGE_DOOR_CLOSING 3
#define GARAGE_DOOR_CLOSED 4
#define GARAGE_DOOR_STALLED 5
#define GARAGE_DOOR_FAILED -1
static int garage_door_state = GARAGE_DOOR_STALLED;
static int garage_door_prev_state = GARAGE_DOOR_STALLED;
static int garage_door_seconds_since_last_certain_state = 0;
#define GARAGE_DOOR_MAX_MOVE_TIME 25 // how many periods (CONFIG_GPIO_POLL_FREQUENCY_SECONDS) has to pass

/* Car presence */
#define CAR_PRESENCE_TRESHOLD 50
#define CAR_PRESENCE_YES 1
#define CAR_PRESENCE_NO 0
#define CAR_PRESENCE_FAILED -1
static int car_presence_state = CAR_PRESENCE_FAILED;
static int car_presence_prev_state = CAR_PRESENCE_FAILED;

K_SEM_DEFINE(lte_connected, 0, 1);
struct modem_param_info modem_param;

static char recv_buf[RECV_BUF_SIZE];

static int server_init(void)
{
	struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr);

	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_UDP_SERVER_PORT);

	inet_pton(AF_INET, CONFIG_UDP_SERVER_ADDRESS_STATIC,
			  &server4->sin_addr);

	return 0;
}

static void server_disconnect(void)
{
	socket_open = false;
	(void)close(client_fd);
}

static int server_connect(void)
{
	int err;

	client_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (client_fd < 0)
	{
		printk("[ERROR] Failed to create UDP socket: %d\n", errno);
		err = -errno;
		goto error;
	}

	err = connect(client_fd, (struct sockaddr *)&host_addr,
				  sizeof(struct sockaddr_in));
	if (err < 0)
	{
		printk("[ERROR] Connect failed : %d\n", errno);
		goto error;
	}
	socket_open = true;
	return 0;

error:
	server_disconnect();

	return err;
}

static bool server_reconnect(void)
{
	printk("[INFO] Reconnecting to UDP server\n");
	server_disconnect();

	int err = server_connect();
	if (err)
	{
		printk("[ERROR] Not able to reconnect to UDP server\n");
		return false;
	}
	return true;
}

static void transmit_udp_data(char *data, size_t len)
{
	int err;
	if (data != NULL)
	{
		printk("[INFO] Sending UDP payload, length: %u, data: %s\n", len, data);
		err = send(client_fd, data, len, 0);
		if (err == -ENOTCONN && server_reconnect())
		{
			err = send(client_fd, data, len, 0);
		}

		if (err < 0)
		{
			printk("[ERROR] Failed to transmit UDP packet, %d\n", errno);
		}
	}
}

/*
 * *** Static functions of main.c ***
 */
static void fetch_sensor_data(void)
{
	sensor_sample_fetch(bme_680);
	sensor_channel_get(bme_680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(bme_680, SENSOR_CHAN_PRESS, &press);
	sensor_channel_get(bme_680, SENSOR_CHAN_HUMIDITY, &humidity);
	sensor_channel_get(bme_680, SENSOR_CHAN_GAS_RES, &gas_res);

	/* apply calibration offsets */
	temp.val1 = temp.val1 - TEMP_CALIBRATION_OFFSET;

	/* Debug print raw sensor data from Zephyr API */
	// printk("[DEBUG] T: %d.%06d; P: %d.%06d; H: %d.%06d; G: %d.%06d\n",
	// 	temp.val1, temp.val2, press.val1, press.val2,
	// 	humidity.val1, humidity.val2, gas_res.val1,
	// 	gas_res.val2);

	/* format data, shrink factional part length of sensor_value to 2 digits */
	temp.val2 = temp.val2 / 10000;
	humidity.val2 = humidity.val2 / 10000;

	/* convert pressure in Bar - sensor_value struct stores 2 integers (dec + fraction) */
	pressure = (float)press.val1 + (float)press.val2 / 1000000; // reformat to float for conversion
	pressure = pressure / 100;									// convert in Bar, but skip storing back into integer parts
}

static void fetch_ultrasonic_data(void)
{
	uint32_t cycles_spent;
	uint32_t val;
	uint32_t stop_time;
	uint32_t start_time;
	uint32_t iter = 0;
	int distance;

	car_presence_prev_state = car_presence_state;

	gpio_pin_set(sx1509b_dev, GPIO_PIN_ULTRASONIC_TRIG, 1);
	k_sleep(K_MSEC(10));
	gpio_pin_set(sx1509b_dev, GPIO_PIN_ULTRASONIC_TRIG, 0);

	do
	{
		iter++;
		val = gpio_pin_get(sx1509b_dev, GPIO_PIN_ULTRASONIC_ECHO);
	} while (val == 0 && iter < 1000);

	if (iter >= 1000)
	{
		printk("[ERROR] No reponse from HC-SR04\n");
		car_presence_state = CAR_PRESENCE_FAILED;
		return;
	}

	start_time = k_cycle_get_32();

	do
	{
		val = gpio_pin_get(sx1509b_dev, GPIO_PIN_ULTRASONIC_ECHO);
		stop_time = k_cycle_get_32();
		cycles_spent = stop_time - start_time;
		if (cycles_spent > 1266720) // 260cm for 84MHz (((MAX_RANGE * 58000) / 1000000000) * (CLOCK * 1000000))
		{
			break;
		}
	} while (val == 1);

	distance = SYS_CLOCK_HW_CYCLES_TO_NS_AVG(cycles_spent, 58000); // in centimeters
	car_presence_state = distance > CAR_PRESENCE_TRESHOLD ? CAR_PRESENCE_NO : CAR_PRESENCE_YES;

	if (car_presence_state != car_presence_prev_state)
	{
		printk("[DEBUG] car_presence_state has changed from %d to %d (distance=%d)\n",
			   car_presence_prev_state, car_presence_state, distance);

		/* Transmit garage via UDP Socket  */
		char data_output[128];
		sprintf(data_output, "{\"Car\":%d}", car_presence_state);
		transmit_udp_data(data_output, strlen(data_output));
	}
}

static void fetch_gpio_data(void)
{
	garage_door_prev_state = garage_door_state;

	int open_endstop_state = gpio_pin_get(sx1509b_dev, GPIO_PIN_OPEN_ENDSTOP);
	int closed_endstop_state = gpio_pin_get(sx1509b_dev, GPIO_PIN_CLOSED_ENDSTOP);

	if (open_endstop_state < 0 || closed_endstop_state < 0)
	{
		printk("[ERROR] gpio_pin_get for sx1509b_dev failed\n");
		garage_door_state = GARAGE_DOOR_FAILED;
		return;
	}

	// printk("[DEBUG] open_endstop: %d, closed_endstop: %d\n",
	//	   open_endstop_state, closed_endstop_state);

	if (open_endstop_state == 1 && closed_endstop_state == 0)
	{
		garage_door_state = GARAGE_DOOR_OPEN;
		garage_door_seconds_since_last_certain_state = 0;
	}
	else if (open_endstop_state == 0 && closed_endstop_state == 1)
	{
		garage_door_state = GARAGE_DOOR_CLOSED;
		garage_door_seconds_since_last_certain_state = 0;
	}
	else if (open_endstop_state == 0 && closed_endstop_state == 0)
	{
		garage_door_seconds_since_last_certain_state += 1;
		if (garage_door_seconds_since_last_certain_state > GARAGE_DOOR_MAX_MOVE_TIME)
		{
			garage_door_state = GARAGE_DOOR_STALLED;
		}
		else
		{
			if (garage_door_prev_state == GARAGE_DOOR_OPEN || garage_door_prev_state == GARAGE_DOOR_CLOSING)
			{
				garage_door_state = GARAGE_DOOR_CLOSING;
			}
			else if (garage_door_prev_state == GARAGE_DOOR_CLOSED || garage_door_prev_state == GARAGE_DOOR_OPENING)
			{
				garage_door_state = GARAGE_DOOR_OPENING;
			}
		}
	}
	else
	{
		// open and close at the same time - impossible
		garage_door_state = GARAGE_DOOR_FAILED;
	}

	if (garage_door_state != garage_door_prev_state)
	{
		printk("[DEBUG] garage_door_state has changed from %d to %d\n",
			   garage_door_prev_state, garage_door_state);

		/* Transmit garage via UDP Socket  */
		char data_output[128];
		sprintf(data_output, "{\"Door\":%d}", garage_door_state);
		transmit_udp_data(data_output, strlen(data_output));
	}
}

static void click_garage_door_relay(void)
{
	printk("[DEBUG] garage door relay ON\n");
	gpio_pin_set(sx1509b_dev, GPIO_PIN_RELAY, 1);
	k_sleep(K_MSEC(500));
	printk("[DEBUG] garage door relay OFF\n");
	gpio_pin_set(sx1509b_dev, GPIO_PIN_RELAY, 0);
}

static int receive_udp_data(char *buf, int buf_size)
{
	int bytes;
	bytes = recv(client_fd, buf, buf_size, 0);
	if (bytes == -ENOTCONN && server_reconnect())
	{
		bytes = recv(client_fd, buf, buf_size, 0);
	}

	if (bytes < 0)
	{
		printk("[ERROR] recv() failed, err %d\n", errno);
	}
	else if (bytes > 0)
	{
		// Make sure buf is NULL terminated (for safe use)
		if (bytes < buf_size)
		{
			buf[bytes] = '\0';
		}
		else
		{
			buf[buf_size - 1] = '\0';
		}
		printk("[DEBUG] Recived UDP data, length: %u, data: %s\n", bytes, buf);
		return bytes;
	}
	return 0;
}

/* Event Handler - used when pressing the button */
static void ui_evt_handler(struct ui_evt evt)
{
	if (evt.type == 1)
	{
		if (socket_open)
		{
			char data_output[128];
			sprintf(data_output, "{\"Door\":%d,\"Btn\":1}", garage_door_state);
			transmit_udp_data(data_output, strlen(data_output));
			click_garage_door_relay();
		}
	}
}

/* Event Handler - used when data received via UDP */
static void udp_evt_handler(char *buf)
{
	printk("[DEBUG] Handling UDP data, data: %s\n", buf);

	if (strcmp(buf, "garage-door-open") == 0)
	{
		if (garage_door_state == GARAGE_DOOR_CLOSED)
		{
			click_garage_door_relay();
		}
		else
		{
			printk("[WARN] Ignoring the open action because the garage door is not closed\n");
		}
	}
	else if (strcmp(buf, "garage-door-close") == 0)
	{
		if (garage_door_state == GARAGE_DOOR_OPEN)
		{
			click_garage_door_relay();
		}
		else
		{
			printk("[WARN] Ignoring the close action because the garage door is not open\n");
		}
	}
	else if (strcmp(buf, "garage-door-click") == 0)
	{
		click_garage_door_relay();
	}
}

static void data_fetch_work_fn(struct k_work *work)
{
	/* Update sensor + modem data */
	fetch_sensor_data();
	modem_info_params_get(&modem_param);

	/* Reschedule work task */
	k_work_schedule(&data_fetch_work, K_SECONDS(CONFIG_UDP_DATA_UPLOAD_FREQUENCY_SECONDS));
}

static void data_poll_work_fn(struct k_work *work)
{
	if (socket_open)
	{
		struct pollfd fds[1];
		int ret = 0;

		fds[0].fd = client_fd;
		fds[0].events = POLLIN;
		fds[0].revents = 0;

		printk("[DEBUG] Polling UDP socket\n");
		ret = poll(fds, 1, RCV_POLL_TIMEOUT_MS);
		if (ret == -ENOTCONN && server_reconnect())
		{
			ret = poll(fds, 1, RCV_POLL_TIMEOUT_MS);
		}

		if (ret > 0)
		{
			int bytes;
			bytes = receive_udp_data(recv_buf, RECV_BUF_SIZE);
			if (bytes > 0)
			{
				udp_evt_handler(recv_buf);
			}
		}
	}
}

static void gpio_fetch_work_fn(struct k_work *work)
{
	fetch_gpio_data();
	fetch_ultrasonic_data();

	/* Reschedule work task */
	k_work_schedule(&gpio_fetch_work, K_SECONDS(CONFIG_GPIO_POLL_FREQUENCY_SECONDS));
}

static void initial_data_transmission(void)
{
	/* Get current modem parameters */
	modem_info_params_get(&modem_param);
	char data_output[128];
	char imei[16];
	char operator[8];
	modem_info_string_get(MODEM_INFO_IMEI, imei, sizeof(imei));
	modem_info_string_get(MODEM_INFO_OPERATOR, operator, sizeof(operator));

	/* format to JSON */
	sprintf(data_output, "{\"Msg\":\"reconnected, %s, %s, %d\",\"Oper\":\"%s\",\"Bd\":%d}",
			imei, operator, modem_param.network.current_band.value, operator, modem_param.network.current_band.value);
	/* Transmit data via UDP Socket */
	transmit_udp_data(data_output, strlen(data_output));
	k_work_schedule(&data_poll_work, K_MSEC(500));
}

static void server_transmission_work_fn(struct k_work *work)
{
	/* Format sensor data to JSON */
	char data_output[128];

	/* format to JSON */
	sprintf(data_output, "{\"Temp\":%d.%02d,\"Press\":%.4f,\"Humid\":%d.%02d,\"Gas\":%d,\"Vbat\":%d,\"Door\":%d,\"Car\":%d}",
			temp.val1, temp.val2, pressure,
			humidity.val1, humidity.val2, gas_res.val1, modem_param.device.battery.value,
			garage_door_state, car_presence_state);

	/* Transmit data via UDP Socket */
	transmit_udp_data(data_output, strlen(data_output));

	/* Reschedule work task */
	k_work_schedule(&server_transmission_work, K_SECONDS(CONFIG_UDP_DATA_UPLOAD_FREQUENCY_SECONDS));
}

static void work_init(void)
{
	k_work_init_delayable(&server_transmission_work, server_transmission_work_fn);
	k_work_init_delayable(&data_fetch_work, data_fetch_work_fn);
	k_work_init_delayable(&data_poll_work, data_poll_work_fn);
	k_work_init_delayable(&gpio_fetch_work, gpio_fetch_work_fn);
}

#if defined(CONFIG_NRF_MODEM_LIB)
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type)
	{
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
			(evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING))
		{
			socket_open = false;
			ui_led_set_effect(UI_LTE_CONNECTING);
			break;
		}
		printk("[LTE] Network registration status: %s\n",
			   evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "Connected - home network" : "Connected - roaming");
		k_sem_give(&lte_connected);
		ui_led_set_effect(UI_LTE_CONNECTED);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("[LTE] PSM parameter update: TAU: %d, Active time: %d\n",
			   evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE:
	{
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
					   "[LTE] eDRX parameter update: eDRX: %f, PTW: %f\n",
					   evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0)
		{
			printk("%s\n", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		printk("[LTE] RRC mode: %s\n",
			   evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");

		if (evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED)
		{
			k_work_schedule(&data_poll_work, K_MSEC(500));
		}
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("[LTE] LTE cell changed: Cell ID: %d, Tracking area: %d\n",
			   evt->cell.id, evt->cell.tac);
		break;
	case LTE_LC_EVT_LTE_MODE_UPDATE:
		printk("[LTE] LTE mode: %s\n",
			   evt->lte_mode == LTE_LC_LTE_MODE_LTEM ? "LTE-M" : "NB-IoT");
	default:
		// printk("[DEBUG] unknown event type: %d\n", evt->type);
		break;
	}
}

static int configure_low_power(void)
{
	int err;

#if defined(CONFIG_UDP_PSM_ENABLE)
	/** Power Saving Mode */
	err = lte_lc_psm_req(true);
	if (err)
	{
		printk("[ERROR] lte_lc_psm_req, error: %d\n", err);
	}
#else
	err = lte_lc_psm_req(false);
	if (err)
	{
		printk("[ERROR] lte_lc_psm_req, error: %d\n", err);
	}
#endif

#if defined(CONFIG_UDP_EDRX_ENABLE)
	/** enhanced Discontinuous Reception */
	err = lte_lc_edrx_req(true);
	if (err)
	{
		printk("[ERROR] lte_lc_edrx_req, error: %d\n", err);
	}
#else
	err = lte_lc_edrx_req(false);
	if (err)
	{
		printk("[ERROR] lte_lc_edrx_req, error: %d\n", err);
	}
#endif

#if defined(CONFIG_UDP_RAI_ENABLE)
	/** Release Assistance Indication  */
	err = lte_lc_rai_req(true);
	if (err)
	{
		printk("[ERROR] lte_lc_rai_req, error: %d\n", err);
	}
#endif

	return err;
}

static void modem_init(void)
{
	int err;
	char response[128];

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
	{
		/* Do nothing, modem is already configured and LTE connected. */
	}
	else
	{
		err = lte_lc_init();
		if (err)
		{
			printk("[ERROR] Modem initialization failed, error: %d\n", err);
			return;
		}

		/* Enable 3GGP CME error codes*/
		nrf_modem_at_printf("AT+CMEE=%d", 1);
		/* Read out Modem IMEI */
		printk("[INFO] Read Modem IMEI\n");
		err = nrf_modem_at_cmd(response, sizeof(response), "AT+CGSN=%d", 1);
		if (err)
		{
			printk("[ERROR] Read Modem IMEI failed, err %d\n", err);
			return;
		}
		else
		{
			printk("[AT] %s\n", response);
		}

		/* Setup APN for the PDP Context */
		printk("[INFO] Setting up the APN\n");
		char *apn_stat = "AT%XAPNSTATUS=1,\"" MODEM_APN "\"";
		char *at_cgdcont = "AT+CGDCONT=0,\"IPV4V6\",\"" MODEM_APN "\"";
		nrf_modem_at_printf(apn_stat);		   // allow use of APN
		err = nrf_modem_at_printf(at_cgdcont); // use conf. APN for PDP context 0 (default LTE bearer)
		if (err)
		{
			printk("[ERROR] AT+CGDCONT set cmd failed, err %d\n", err);
			return;
		}
		err = nrf_modem_at_cmd(response, sizeof(response), "AT+CGDCONT?", NULL);
		if (err)
		{
			printk("[ERROR] APN check failed, err %d\n", err);
			return;
		}
		else
		{
			printk("[AT] %s\n", response);
		}
		/* Init modem info module */
		modem_info_init();
		modem_info_params_init(&modem_param);
	}
}

static void modem_connect(void)
{
	int err;

	if (IS_ENABLED(CONFIG_LTE_AUTO_INIT_AND_CONNECT))
	{
		/* Do nothing, modem is already configured and LTE connected. */
	}
	else
	{
		err = lte_lc_connect_async(lte_handler);
		if (err)
		{
			printk("[ERROR] Connecting to LTE network failed, error: %d\n",
				   err);
			return;
		}
	}
}
#endif

/*
 * *** Main Applicatin Entry ***
 */
void main(void)
{
	int err;

	// some dalay so that we can connect with serial port and read debugging messages from the start
	k_sleep(K_SECONDS(5));

	printk("\n*** Garage Door ***\n");

	/* Init routines */
	ui_init(ui_evt_handler);
	ui_led_set_effect(UI_LTE_CONNECTING);
	bme_680 = device_get_binding(DT_LABEL(DT_INST(0, bosch_bme680)));

	sx1509b_dev = device_get_binding(DT_LABEL(DT_INST(0, semtech_sx1509b)));

	if (!device_is_ready(sx1509b_dev))
	{
		printk("[ERROR] sx1509b device is not ready\n");
		return;
	}

	gpio_pin_configure(sx1509b_dev, GPIO_PIN_OPEN_ENDSTOP, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
	gpio_pin_configure(sx1509b_dev, GPIO_PIN_CLOSED_ENDSTOP, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
	gpio_pin_configure(sx1509b_dev, GPIO_PIN_RELAY, GPIO_OUTPUT_LOW);

	gpio_pin_configure(sx1509b_dev, GPIO_PIN_ULTRASONIC_TRIG, GPIO_OUTPUT);
	gpio_pin_configure(sx1509b_dev, GPIO_PIN_ULTRASONIC_ECHO, GPIO_INPUT | GPIO_ACTIVE_HIGH);

	k_sleep(K_MSEC(1)); /* Wait for the i2c rail to come up and stabilize */

	work_init();

	/* Initialize the modem before calling configure_low_power(). This is
	 * because the enabling of RAI is dependent on the
	 * configured network mode which is set during modem initialization.
	 */
	modem_init();
	err = configure_low_power();
	if (err)
	{
		printk("[ERROR] Unable to set low power configuration, error: %d\n", err);
	}

	modem_connect();
	k_sem_take(&lte_connected, K_FOREVER);
	k_msleep(500);

	/* Init UDP Socket */
	err = server_init();
	if (err)
	{
		printk("[ERROR] Not able to initialize UDP server connection\n");
		return;
	}

	/* Connect UDP Socket */
	err = server_connect();
	if (err)
	{
		printk("[ERROR] Not able to connect to UDP server\n");
		return;
	}

	/* Perform initial data transmission & schedule periodic tasks */
	initial_data_transmission();
	k_work_schedule(&data_fetch_work, K_NO_WAIT);
	k_work_schedule(&gpio_fetch_work, K_NO_WAIT);
	k_work_schedule(&server_transmission_work, K_SECONDS(2));
}
