#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>

#define REQUEST 0
#define STANDARD_REQUEST 1
#define RECIPIENT 2

/* connector - connect to usb device and send transfers
 * it sends a selected number of transfers of desired type etc to the endpoint chosen
 * with timeout of 1 second, each transfer sent with 2 seconds interval
 * (just in case)
 *
 * there are several things that need to be done I need to ask about first;
 * also it is not tested and it may lead to memory leaks - I am not sure
 * where and if should I free any resources. */

/*
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 */
#define BUF_LEN		8192

/*
 * struct test_state - describes test program state
 * @list: list of devices returned by libusb_get_device_list function
 * @found: pointer to struct describing tested device
 * @ctx: context, set to NULL
 * @handle: handle of tested device
 * @attached: indicates that device was attached to kernel, and has to be
 *            reattached at the end of test program
 */

struct test_state {
	libusb_device *found;
	libusb_context *ctx;
	libusb_device_handle *handle;
	int attached;
};

/*
 * test_init - initialize test program
 */

int test_init(struct test_state *state, uint16_t vendor, uint16_t product)
{
	int i, ret;
	ssize_t cnt;
	libusb_device **list;

	state->found = NULL;
	state->ctx = NULL;
	state->handle = NULL;
	state->attached = 0;

	ret = libusb_init(&state->ctx);
	if (ret) {
		printf("cannot init libusb: %s\n", libusb_error_name(ret));
		return 1;
	}

	cnt = libusb_get_device_list(state->ctx, &list);
	if (cnt <= 0) {
		printf("no devices found\n");
		goto error1;
	}

	for (i = 0; i < cnt; ++i) {
		libusb_device *dev = list[i];
		struct libusb_device_descriptor desc;
		ret = libusb_get_device_descriptor(dev, &desc);
		if (ret) {
			printf("unable to get device descriptor: %s\n",
			       libusb_error_name(ret));
			goto error2;
		}
		if (desc.idVendor == vendor && desc.idProduct == product) {
			state->found = dev;
			break;
		}
	}

	if (!state->found) {
		printf("no devices found\n");
		goto error2;
	}

	ret = libusb_open(state->found, &state->handle);
	if (ret) {
		printf("cannot open device: %s\n", libusb_error_name(ret));
		goto error2;
	}

	if (libusb_claim_interface(state->handle, 0)) {
		ret = libusb_detach_kernel_driver(state->handle, 0);
		if (ret) {
			printf("unable to detach kernel driver: %s\n",
			       libusb_error_name(ret));
			goto error3;
		}
		state->attached = 1;
		ret = libusb_claim_interface(state->handle, 0);
		if (ret) {
			printf("cannot claim interface: %s\n",
			       libusb_error_name(ret));
			goto error4;
		}
	}

	return 0;

error4:
	if (state->attached == 1)
		libusb_attach_kernel_driver(state->handle, 0);

error3:
	libusb_close(state->handle);

error2:
	libusb_free_device_list(list, 1);

error1:
	libusb_exit(state->ctx);
	return 1;
}

/*
 * test_exit - cleanup test program
 */

void test_exit(struct test_state *state)
{
	libusb_release_interface(state->handle, 0);
	if (state->attached == 1)
		libusb_attach_kernel_driver(state->handle, 0);
	libusb_close(state->handle);
	libusb_exit(state->ctx);
}

/* TODO callback right now doesn't do much and needs a more complex iso response handling */
void callback(struct libusb_transfer *transfer) {
	int i = 0;
	switch(transfer->status) {
	case LIBUSB_TRANSFER_COMPLETED :
		printf("It works! Data amount: %d, actual amount: %d.\n",
				transfer->length, transfer->actual_length);
		printf("Data from gadget: %.*s\n",
					transfer->length, transfer->buffer);
		break;
	case LIBUSB_TRANSFER_CANCELLED :
		printf("Transfer cancelled.\n");
		break;
	case LIBUSB_TRANSFER_TIMED_OUT :
		printf("Transfer timed out.\n");
		break;
	case LIBUSB_TRANSFER_OVERFLOW :
		printf("Transfer overflow. Buffer length: %d,"
				"actual length of transfered data: %d.\n",
				transfer->length, transfer->actual_length);
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		printf("No device found.\n");
		break;
	case LIBUSB_TRANSFER_ERROR :
		printf("Transfer error.\n");
		break;
	default :
		printf("Something weird happened to the transfer.\n");
	}
}

/*void transfer_data(libusb_device_handle *handle) {
	struct libusb_transfer *transfer;
	static unsigned char buffer[BUF_LEN];

	transfer = libusb_alloc_transfer(0);

	uint8_t requestType = LIBUSB_RECIPIENT_DEVICE
							| LIBUSB_REQUEST_TYPE_VENDOR
							| LIBUSB_ENDPOINT_IN;


	libusb_fill_control_setup(buffer, requestType, 1, 2, 3, BUF_LEN);
	libusb_fill_control_transfer(transfer, handle, buffer,
									callback, NULL, 1000);
	libusb_submit_transfer(transfer);
}*/

void transfer_data(libusb_device_handle *handle, uint8_t transfer_type,
					unsigned char addr, unsigned char *buffer, int iso_packet_count,
					struct libusb_control_setup control_setup) {
	struct libusb_transfer *transfer;
	transfer = libusb_alloc_transfer(iso_packet_count);

	switch (transfer_type) {
	case LIBUSB_TRANSFER_TYPE_CONTROL :
		libusb_fill_control_setup(buffer, control_setup.bmRequestType,
									control_setup.bRequest, control_setup.wIndex,
									control_setup.wValue, control_setup.wLength);
		libusb_fill_control_transfer(transfer, handle, buffer,
										callback, NULL, 1000);
		break;
	case LIBUSB_TRANSFER_TYPE_BULK :
		libusb_fill_bulk_transfer(transfer, handle, addr, buffer, BUF_LEN,
									callback, NULL, 1000);
		break;
	case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS : /* TODO dopytac sie */
		libusb_fill_iso_transfer(transfer, handle, addr, buffer, BUF_LEN, iso_packet_count,
									callback, NULL, 1000);
		break;
	case LIBUSB_TRANSFER_TYPE_INTERRUPT :
		libusb_fill_interrupt_transfer(transfer, handle, addr, buffer, BUF_LEN,
										callback, NULL, 1000);
		break;
	/* TODO bulk stream */
	}
	libusb_submit_transfer(transfer);
}

static void print_usage(char *program) {
	printf("Usage: %s -v VENDOR -p PRODUCT -e ENDPOINT -c MESSAGES_COUNT [OPTIONS]\n"
						"\t-i: Count of iso packets to send (if using isochronous transfer)\n"
						"\t-d: Direction (IN/OUT), needs to be specified if ENDPOINT == 0\n"
						"\t-m: Message (some data if you want to send anything to an ep)\n"
						"\t-h: Prints this message and exits program.\n"
						"\t-t: Request type (applies to control request): STANDARD|CLASS|VENDOR|RESERVED\n"
						"\t-r: Recipient (applies to control request): DEVICE|INTERFACE|ENDPOINT|OTHER\n"
						"\t-s: Standard request format: bRequest:wValue:wIndex:wLength\n"
						"\t\t(for values allowed see Table 9-3 of USB 3.0 spec)\n",
						program);
}

struct request_type {
	const char* name;
	const uint8_t value;
};

/*static uint8_t getRequestType(char *type) {
	const struct request_type types[] = {
		{ "STANDARD", LIBUSB_REQUEST_TYPE_STANDARD },
		{ "CLASS", LIBUSB_REQUEST_TYPE_CLASS },
		{ "VENDOR", LIBUSB_REQUEST_TYPE_VENDOR },
		{ "RESERVED", LIBUSB_REQUEST_TYPE_RESERVED }
	};
	const int request_count = 4;
	for (int i = 0; i < request_count; i++) {
		if (strcmp(types[i].name, type) == 0) {
			return types[i].value;
		}
	}
	fprintf(stderr, "Request type: %s not found. Abort.\n", type);
	abort();
}*/

static uint8_t getConstant(char *name, int constant_type) {
	static struct request_type requests[] = { /* 0 - REQUEST */
			{ "STANDARD", LIBUSB_REQUEST_TYPE_STANDARD },
			{ "CLASS", LIBUSB_REQUEST_TYPE_CLASS },
			{ "VENDOR", LIBUSB_REQUEST_TYPE_VENDOR },
			{ "RESERVED", LIBUSB_REQUEST_TYPE_RESERVED }
	};
	static struct request_type standard_requests[] = { /* 1 - STANDARD_REQUEST */
			{ "GET_STATUS", LIBUSB_REQUEST_GET_STATUS },
			{ "CLEAR_FEATURE", LIBUSB_REQUEST_CLEAR_FEATURE },
			{ "SET_FEATURE", LIBUSB_REQUEST_SET_FEATURE },
			{ "SET_ADDRESS", LIBUSB_REQUEST_SET_ADDRESS },
			{ "GET_DESCRIPTOR", LIBUSB_REQUEST_GET_DESCRIPTOR },
			{ "SET_DESCRIPTOR", LIBUSB_REQUEST_SET_DESCRIPTOR },
			{ "GET_CONFIGURATION", LIBUSB_REQUEST_GET_CONFIGURATION },
			{ "SET_CONFIGURATION", LIBUSB_REQUEST_SET_CONFIGURATION },
			{ "GET_INTERFACE", LIBUSB_REQUEST_GET_INTERFACE },
			{ "SET_INTERFACE", LIBUSB_REQUEST_SET_INTERFACE },
			{ "SYNCH_FRAME", LIBUSB_REQUEST_SYNCH_FRAME },
			{ "SET_SEL", LIBUSB_REQUEST_SET_SEL },
			{ "SET_ISOCH_DELAY", LIBUSB_SET_ISOCH_DELAY }
	};
	static struct request_type recipients[] = {
			{ "DEVICE", LIBUSB_RECIPIENT_DEVICE },
			{ "INTERFACE", LIBUSB_RECIPIENT_INTERFACE },
			{ "ENDPOINT", LIBUSB_RECIPIENT_ENDPOINT },
			{ "OTHER", LIBUSB_RECIPIENT_OTHER }
	};
	static struct request_type* type_arrays[] = {
			requests, standard_requests, recipients
	};
	static size_t arr_size[] = {
			sizeof(requests), sizeof(standard_requests), sizeof(recipients)
	};
	int request_count = arr_size[constant_type] / sizeof(struct request_type);
	int i;
	printf("Size: %d\n", request_count);
	for (i = 0; i < request_count; i++) {
		if (strcmp(type_arrays[constant_type][i].name, name) == 0) {
			printf("Constant chosen: %s found.\n", name);
			return type_arrays[constant_type][i].value;
		}
	}
	fprintf(stderr, "Constant: %s not found. Abort.\n", name);
	abort();
}

int main(int argc, char** argv)
{
	struct test_state state;
	struct libusb_config_descriptor *conf;
	struct libusb_interface_descriptor const *iface;
	struct libusb_control_setup control_setup;
	uint16_t *fields[] = { &control_setup.wValue,
						   &control_setup.wIndex,
						   &control_setup.wLength };
	unsigned char addr;
	int calls_count = 1, i = 1, endpoint, iso_packet_count = 0;
	uint8_t direction, transfer_type;
	static unsigned char buffer[BUF_LEN];
	char c;
	static const char opts[] = {'e', 'c', 'i', 'd', 'm', 'h', 't', 'r', 's'};
	const int opt_count = sizeof(opts) / sizeof(opts[0]);
	char *tmp;
	uint16_t vendor = 0, product = 0;
	memset(buffer, 0, BUF_LEN);

	if (argc == 1) {
		print_usage(argv[0]);
		return 1;
	}

	while ((c = getopt(argc, argv, "v:p:e:c:i:d:m:h:t:r:s:")) != -1) {
		switch(c) {
		case 'v':
			vendor = (uint16_t) strtol(optarg, NULL, 16);
			break;
		case 'p':
			product = (uint16_t) strtol(optarg, NULL, 16);
			break;
		case 's':
			i = 0;
			while ((tmp = strsep(&optarg, ":"))) {
				if (!i) {
					control_setup.bRequest = getConstant(tmp, STANDARD_REQUEST); break;
				} else {
					printf("Setting field %d with %d.\n", i-1, atoi(tmp));
					*fields[i-1] = (uint16_t) atoi(tmp);
				}
				i++;
			}
			printf("Done.\n");
			break;
		case 't': case 'r' :
			control_setup.bmRequestType |=
					getConstant(optarg, c == 't' ? REQUEST : RECIPIENT);
			break;
		case 'h':
			print_usage(argv[0]);
			break;
		case 'e':
			endpoint = atoi(optarg);
			break;
		case 'c':
			calls_count = atoi(optarg);
			break;
		case 'i':
			iso_packet_count = atoi(optarg);
			break;
		case 'd':
			if (strcmp(optarg, "in") == 0 || strcmp(optarg, "IN") == 0) {
				direction = LIBUSB_ENDPOINT_IN;
			} else if (strcmp(optarg, "out") == 0 || strcmp(optarg, "OUT") == 0) {
				direction = LIBUSB_ENDPOINT_OUT;
			} else {
				fprintf(stderr, "Invalid argument for option -d."
								"Got: %s, expected IN/OUT.\n", optarg);
				return 1;
			}
			control_setup.bmRequestType |= direction;
			break;
		case 'm':
			strcpy(buffer, optarg);
			break;
		case '?' :
			for (i = 0; i < opt_count; i++) {
				if (opts[i] == optopt) {
					fprintf(stderr, "Option -%c requires an argument.\n", optopt);
					return 1;
				}
			}
			if (isprint(optopt)) {
				fprintf(stderr, "Unknown option: -%c.\n", optopt);
				return 1;
			}
			fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
			return 1;
		default:
			abort();
		}
	}

	printf("Endpoint: %d, msgs count: %d.\n", endpoint, calls_count);

	/*if (argc < 5) {
		printf("Usage: %s ENDPOINT DIRECTION TRANSFER_TYPE COUNT [ISO_PACKET_COUNT] [DATA]\n"
				"\tENDPOINT - ep number to which we want to send a transfer\n"
				"\tDIRECTION - IN or OUT\n"
				"\tTRANSFER_TYPE - control, interrupt, bulk, isochronous\n"
				"\t\t if type is control then endpoint is always 0, direction in\n"
				"\t\t if type is isochronous, the iso_packet_count must be specified\n"
				"\tCOUNT - how many transfers we want to send\n"
				"\tISO_PACKET_COUNT - how many iso packets we want to send in each transfer\n"
				"\tDATA - data to fill the buffer\n"
				"\tExample: %s 0 in control 5\n",
				prog_name, prog_name);
		return 1;
	}*/

	/*endpoint = atoi(argv[1]);
	direction = (strcmp(argv[2], "in") == 0) ? LIBUSB_ENDPOINT_IN : LIBUSB_ENDPOINT_OUT;
	if (strcmp(argv[3], "control") == 0) {
		transfer_type = LIBUSB_TRANSFER_TYPE_CONTROL;
	} else if (strcmp(argv[3], "bulk") == 0) {
		transfer_type = LIBUSB_TRANSFER_TYPE_BULK;
	} else if (strcmp(argv[3], "iso") == 0) {
		transfer_type = LIBUSB_TRANSFER_TYPE_ISOCHRONOUS;
	} else if (strcmp(argv[3], "interrupt") == 0) {
		transfer_type = LIBUSB_TRANSFER_TYPE_INTERRUPT;
	}
	calls_count = atoi(argv[4]);
	if (transfer_type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) {
		iso_packet_count = atoi(argv[5]);
	}*/

	/* check if user sent any data */
	/*if (argc-5-(transfer_type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS)) {
		strcpy(buffer, argv[6 + (transfer_type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS)]);
	}*/

	if (test_init(&state, vendor, product))
		return 1;

	libusb_get_config_descriptor(state.found, 0, &conf);
	iface = &conf->interface[0].altsetting[0];
	addr = iface->endpoint[endpoint].bEndpointAddress;

	/* check if endpoint direction and requested direction match */
	/*if (((addr >> 7) != (direction == LIBUSB_ENDPOINT_IN)) && !endpoint) {
		printf("Fatal: Requested direction: %s does not match endpoint direction: %s.\n",
				argv[2], (addr >> 7) ? "IN" : "OUT");
		return -1;
	}*/

	/* TODO dopytac sie o to */
	/*if ((iface->endpoint[endpoint].bmAttributes & 0x03) != transfer_type) {
		printf("Fatal: Requested transfer type: %s does not match endpoint type.\n",
				argv[3]);
		return -1;
	}*/

	transfer_type = iface->endpoint[endpoint].bmAttributes & 0x03;

	while (calls_count) {
		printf("Begin transfer %d. Remaining transfers: %d.\n", i++, calls_count--);
		transfer_data(state.handle, transfer_type, addr,
				buffer, iso_packet_count, control_setup);
		printf("Going to sleep...\n");
	}
	int res;
	while (1) {
		res = libusb_handle_events(state.ctx);
		if (res < 0) {
			printf("handle_events()error # %d\n", res);
			/* Break out of this loop only on fatal error.*/
			if
			(res != LIBUSB_ERROR_BUSY &&
			res != LIBUSB_ERROR_TIMEOUT &&
			res != LIBUSB_ERROR_OVERFLOW &&
			res != LIBUSB_ERROR_INTERRUPTED) {
				break;
			}
		}
	}

	test_exit(&state);
	return 0;
}
