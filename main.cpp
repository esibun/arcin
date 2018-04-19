#include <rcc/rcc.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <timer/timer.h>
#include <os/time.h>
#include <usb/usb.h>
#include <usb/descriptor.h>
#include <usb/hid.h>
#include <math.h>

#define TT1_INVERTED -1
#define TT2_INVERTED 1
#define TT1_SENS 0.25
#define TT2_SENS 1

static uint32_t& reset_reason = *(uint32_t*)0x10000000;

static bool do_reset_bootloader;

void reset_bootloader() {
	reset_reason = 0xb007;
	SCB.AIRCR = (0x5fa << 16) | (1 << 2); // SYSRESETREQ
}

auto keyb_report_desc = keyboard(
	usage_page(UsagePage::Keyboard),
	report_size(1),
	report_count(13),
	logical_minimum(0),
	logical_maximum(255),
	usage_minimum(0),
	usage_maximum(255),
	report_count(13),
	report_size(8),
	input(0x00)
);


auto report_desc = gamepad(
	// Inputs.
	buttons(15),
	padding_in(1),
	
	usage_page(UsagePage::Desktop),
	usage(DesktopUsage::X),
	logical_minimum(0),
	logical_maximum(255),
	report_count(1),
	report_size(8),
	input(0x02),

	usage_page(UsagePage::Desktop),
	usage(DesktopUsage::Y),
	logical_minimum(0),
	logical_maximum(255),
	report_count(1),
	report_size(8),
	input(0x02),
	
	// Outputs.
	usage_page(UsagePage::Ordinal),
	usage(1),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(2),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(3),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(4),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(5),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(6),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(7),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(8),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(9),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(10),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),
	
	usage_page(UsagePage::Ordinal),
	usage(11),
	collection(Collection::Logical, 
		usage_page(UsagePage::LED),
		usage(0x4b),
		report_size(1),
		report_count(1),
		output(0x02)
	),

	padding_out(5),
	
	usage_page(0xff55),
	usage(0xb007),
	logical_minimum(0),
	logical_maximum(255),
	report_size(8),
	report_count(1),
	
	feature(0x02) // HID bootloader function
);

auto dev_desc = device_desc(0x200, 0, 0, 0, 64, 0x1d50, 0x6080, 0x110, 1, 2, 3, 1);
auto conf_desc = configuration_desc(2, 1, 0, 0xc0, 0,
	// HID interface.
	interface_desc(0, 0, 1, 0x03, 0x00, 0x00, 0,
		hid_desc(0x111, 0, 1, 0x22, sizeof(report_desc)),
		endpoint_desc(0x81, 0x03, 16, 1)
	),
	interface_desc(1, 0, 1, 0x03, 0x00, 0x00, 0,
		hid_desc(0x111, 0, 1, 0x22, sizeof(keyb_report_desc)),
		endpoint_desc(0x82, 0x03, 16, 1)
	)
);

desc_t dev_desc_p = {sizeof(dev_desc), (void*)&dev_desc};
desc_t conf_desc_p = {sizeof(conf_desc), (void*)&conf_desc};
desc_t report_desc_p = {sizeof(report_desc), (void*)&report_desc};
desc_t keyb_report_desc_p = {sizeof(keyb_report_desc), (void*)&keyb_report_desc};

static Pin usb_dm = GPIOA[11];
static Pin usb_dp = GPIOA[12];
static Pin usb_pu = GPIOA[15];

static PinArray button_inputs = GPIOB.array(0, 10);
static PinArray button_leds = GPIOC.array(0, 10);

static Pin qe1a = GPIOA[0];
static Pin qe1b = GPIOA[1];
static Pin qe2a = GPIOA[6];
static Pin qe2b = GPIOA[7];

static Pin led1 = GPIOA[8];
static Pin led2 = GPIOA[9];

USB_f1 usb(USB, dev_desc_p, conf_desc_p);

uint32_t last_led_time;

class HID_arcin : public USB_HID {
	public:
		HID_arcin(USB_generic& usbd, desc_t rdesc) : USB_HID(usbd, rdesc, 0, 1, 64) {}
	
	protected:
		virtual bool set_output_report(uint32_t* buf, uint32_t len) {
			last_led_time = Time::time();
			button_leds.set(*buf);
			return true;
		}
		
		virtual bool set_feature_report(uint32_t* buf, uint32_t len) {
			if(len != 1) {
				return false;
			}
			
			switch(*buf & 0xff) {
				case 0:
					return true;
				
				case 0x10: // Reset to bootloader
					do_reset_bootloader = true;
					return true;
				
				default:
					return false;
			}
		}
};

class HID_keyb : public USB_HID {
	public:
		HID_keyb(USB_generic& usbd, desc_t rdesc) : USB_HID(usbd, rdesc, 1, 2, 64) {}

	protected:
		virtual bool set_output_report(uint32_t* buf, uint32_t len) {
			// ignore
			return true;
		}

		virtual bool set_feature_report(uint32_t* buf, uint32_t len) {
			// ignore
			return false;
		}
};

HID_arcin usb_hid(usb, report_desc_p);
HID_keyb usb_hid_keyb(usb, keyb_report_desc_p);

uint32_t serial_num() {
	uint32_t* uid = (uint32_t*)0x1ffff7ac;
	
	return uid[0] * uid[1] * uid[2];
}

class USB_strings : public USB_class_driver {
	private:
		USB_generic& usb;
		
	public:
		USB_strings(USB_generic& usbd) : usb(usbd) {
			usb.register_driver(this);
		}
	
	protected:
		virtual SetupStatus handle_setup(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
			// Get string descriptor.
			if(bmRequestType == 0x80 && bRequest == 0x06 && (wValue & 0xff00) == 0x0300) {
				const void* desc = nullptr;
				uint16_t buf[9];
				
				switch(wValue & 0xff) {
					case 0:
						desc = u"\u0304\u0409";
						break;
					
					case 1:
						desc = u"\u0308zyp";
						break;
					
					case 2:
						desc = u"\u030carcin";
						break;
					
					case 3:
						{
							buf[0] = 0x0312;
							uint32_t id = serial_num();
							for(int i = 8; i > 0; i--) {
								buf[i] = (id & 0xf) > 9 ? 'A' + (id & 0xf) - 0xa : '0' + (id & 0xf);
								id >>= 4;
							}
							desc = buf;
						}
						break;
				}
				
				if(!desc) {
					return SetupStatus::Unhandled;
				}
				
				uint8_t len = *(uint8_t*)desc;
				
				if(len > wLength) {
					len = wLength;
				}
				
				usb.write(0, (uint32_t*)desc, len);
				
				return SetupStatus::Ok;
			}
			
			return SetupStatus::Unhandled;
		}
};

class analog_button {
	public:
		// config

		// Number of ticks we need to advance before recognizing an input
		uint32_t deadzone;
		// How long to sustain the input before clearing it (if opposite direction is input, we'll release immediately)
		uint32_t sustain_ms;
		// Always provide a zero-input for one poll before reversing?
		bool clear;

		const volatile uint32_t &counter;

		// State: Center of deadzone
		uint32_t center;
		// times to: reset to zero, reset center to counter
		uint32_t t_timeout;

		int8_t state; // -1, 0, 1
		int8_t last_delta;
	public:
		analog_button(volatile uint32_t &counter, uint32_t deadzone, uint32_t sustain_ms, bool clear)
			: deadzone(deadzone), sustain_ms(sustain_ms), clear(clear), counter(counter)
		{
			center = counter;
			t_timeout = 0;
			state = 0;
		}

		int8_t poll() {
			uint8_t observed = counter;
			int8_t delta = observed - center;
			last_delta = delta;

			uint8_t direction = 0;
			if (delta >= (int32_t)deadzone) {
				direction = 1;
			} else if (delta <= -(int32_t)deadzone) {
				direction = -1;
			}

			if (direction != 0) {
				center = observed;
				t_timeout = Time::time() + sustain_ms;
			} else if (t_timeout != 0 && Time::time() >= t_timeout) {
				state = 0;
				center = observed;
				t_timeout = 0;
			}

			if (direction == -state && clear) {
				state = direction;
				return 0;
			} else if (direction != 0) {
				state = direction;
			}

			return state * TT1_INVERTED;
		}
};

USB_strings usb_strings(usb);

struct report_t {
	uint16_t buttons;
	uint8_t axis_x;
	uint8_t axis_y;
} __attribute__((packed));

#define DEBOUNCE_TIME_MS 5

uint16_t debounce_state = 0;
uint16_t debounce_history[DEBOUNCE_TIME_MS] = { 0 };
uint32_t debounce_sample_time = 0;
int debounce_index = 0;

/* 
 * Perform debounce processing. The buttons input is sampled at most once per ms
 * (when update is true); buttons is then set to the last stable state for each
 * bit (i.e., the last state maintained for DEBOUNCE_TIME_MS consequetive samples
 *
 * We use update to sync to the USB polls; this helps avoid additional latency when
 * debounce samples just after the USB poll.
 */
void debounce(uint16_t &buttons) {
	if (Time::time() == debounce_sample_time) {
		buttons = debounce_state;
        return;
	}

    debounce_sample_time = Time::time();

	debounce_history[debounce_index] = buttons;
	debounce_index = (debounce_index + 1) % DEBOUNCE_TIME_MS;

    uint16_t dbg = 0;

	uint16_t has_ones = 0, has_zeroes = 0;
	for (int i = 0; i < DEBOUNCE_TIME_MS; i++) {
		has_ones |= debounce_history[i];
		has_zeroes |= ~debounce_history[i];

        dbg |= (debounce_history[i] & 1) ? (1 << i) : 0;
	}

	uint16_t stable = has_ones ^ has_zeroes;
    dbg |= (stable & 1) ? (1 << 5) : 0;

	debounce_state = (debounce_state & ~stable) | (has_ones & stable);
    dbg |= (debounce_state & 1) ? (1 << 6) : 0;

    dbg |= (buttons & 1) ? (1 << 10) : 0;

	buttons = debounce_state;
//    button_leds.set(dbg);
}

#define SETUP_IDLE 0
#define SETUP_AWAIT_RELEASE 1
#define SETUP_ACTIVE 2

uint16_t setup_state = SETUP_IDLE;
uint32_t setup_timer;
int enable_keyboard = 0;
int enable_debounce = 0;

// returns true to suppress buttons
int check_setup(uint16_t buttons) {
	if (setup_state == SETUP_IDLE) {
		if (!((buttons & (1 << 9)) && (buttons & (1 << 10)))) {
			setup_timer = Time::time();
			return 0;
		}
		
		if (Time::time() - setup_timer > 5000) {
			setup_state = SETUP_AWAIT_RELEASE;
            return 1;
		}

		return 0;
	}

	// Update LEDs first
	uint16_t led = 0;
	if ((Time::time() / 1000) & 1) {
		led |= (1 << 9);
	} else {
		led |= (1 << 10);
	}

	if (enable_keyboard) {
		led |= (1 << 0);
	}
	if (enable_debounce) {
		led |= (1 << 1);
	}

	button_leds.set(led);

	if (setup_state == SETUP_AWAIT_RELEASE) {
		// The user just pressed a button, so wait for them to release it
		if (buttons) {
			return 1;
		} else {
			setup_state = SETUP_ACTIVE;
			setup_timer = Time::time();
		}
	}

	// Check for attempts to leave setup mode
	if (buttons & (1 << 9)) {
		if (Time::time() - setup_timer > 1000) {
			setup_state = SETUP_IDLE;
			setup_timer = Time::time();
			return 1;
		}

		return 1; // await timeout
	}
	setup_timer = Time::time();

	if (buttons & (1 << 0)) {
		// SW1 - Enable keyboard
		enable_keyboard = !enable_keyboard;
		setup_state = SETUP_AWAIT_RELEASE;
	} else if (buttons & (1 << 1)) {
		// SW2 - Enable debounce
		enable_debounce = !enable_debounce;
		setup_state = SETUP_AWAIT_RELEASE;
	}

	return 1;
}

// emulated pos
float tt1_pos = 0;
float tt2_pos = 0;

// actual, physical pos
uint8_t tt1_last_pos = 0;
uint8_t tt2_last_pos = 0;

inline float get_delta(uint8_t newpos, uint8_t last, float sens) {
	// get actual difference
	float delta = (float)(newpos - last);

	// normalize overflow
	if ( delta < -127 ) 
		delta += 256;
	
	// normalize underflow
	if ( delta > 127 )
		delta -= 256;
	
	// apply sensitivity
	delta *= sens;

	return delta;
}

inline float clip(float pos) {
	while ( pos < 0 )
		pos += 256;
	while ( pos > 256 )
		pos -= 256;
	return pos;
}

int main() {
	rcc_init();
	
	// Initialize system timer.
	STK.LOAD = 72000000 / 8 / 1000; // 1000 Hz.
	STK.CTRL = 0x03;
	
	RCC.enable(RCC.GPIOA);
	RCC.enable(RCC.GPIOB);
	RCC.enable(RCC.GPIOC);
	
	usb_dm.set_mode(Pin::AF);
	usb_dm.set_af(14);
	usb_dp.set_mode(Pin::AF);
	usb_dp.set_af(14);
	
	RCC.enable(RCC.USB);
	
	usb.init();
	
	usb_pu.set_mode(Pin::Output);
	usb_pu.on();
	
	button_inputs.set_mode(Pin::Input);
	button_inputs.set_pull(Pin::PullUp);
	
	button_leds.set_mode(Pin::Output);
	
	led1.set_mode(Pin::Output);
	led1.on();
	
	led2.set_mode(Pin::Output);
	led2.on();
	
	RCC.enable(RCC.TIM2);
	RCC.enable(RCC.TIM3);
	
	TIM2.CCMR1 = (1 << 8) | (1 << 0);
	#if TT1_INVERTED == 1
	TIM2.CCER = 1 << 1;
	#endif
	TIM2.SMCR = 3;
	TIM2.CR1 = 1;
	
	TIM3.CCMR1 = (1 << 8) | (1 << 0);
	#if TT2_INVERTED == 1
	TIM3.CCER = 1 << 1;
	#endif
	TIM3.SMCR = 3;
	TIM3.CR1 = 1;
	
	qe1a.set_af(1);
	qe1b.set_af(1);
	qe1a.set_mode(Pin::AF);
	qe1b.set_mode(Pin::AF);
	
	qe2a.set_af(2);
	qe2b.set_af(2);
	qe2a.set_mode(Pin::AF);
	qe2b.set_mode(Pin::AF);

	analog_button tt1(TIM2.CNT, 4, 100, true);
	analog_button tt2(TIM3.CNT, 4, 100, true);

	while(1) {
		usb.process();
		
		int setup_active = 0;
		uint16_t buttons = button_inputs.get() ^ 0x7ff;
		uint16_t debounced = buttons;

		debounce(debounced);

		if (check_setup(debounced)) {
			debounced = buttons = 0;
			setup_active = 1;
		}

		if (enable_debounce) {
			buttons = debounced;
		}

		// The arcin has 11 digital inputs. We'll add an extra four for TT conversion.
		// First, mask off the unused top five bits
		buttons = buttons & ~(0xf800);

		// And OR in our decoded quadrature directions
		switch (tt1.poll()) {
			case -1:
				buttons |= (1 << 11);
				break;
			case 1:
				buttons |= (1 << 12);
				break;
		}

		switch (tt2.poll()) {
			case -1:
				buttons |= (1 << 13);
				break;
			case 1:
				buttons |= (1 << 14);
				break;
		}
		
		if(do_reset_bootloader) {
			Time::sleep(10);
			reset_bootloader();
		}
		
		if(!setup_active && (Time::time() - last_led_time > 1000)) {
			button_leds.set(buttons);
		}
		
		if(usb.ep_ready(1)) {
			tt1_pos += get_delta(TIM2.CNT, tt1_last_pos, TT1_SENS);
			tt2_pos += get_delta(TIM3.CNT, tt2_last_pos, TT2_SENS);

			tt1_pos = clip(tt1_pos);
			tt2_pos = clip(tt2_pos);

			tt1_last_pos = TIM2.CNT;
			tt2_last_pos = TIM3.CNT;

			report_t report = {buttons, (uint8_t)tt1_pos, (uint8_t)tt2_pos};
			
			usb.write(1, (uint32_t*)&report, sizeof(report));
		}

		if (usb.ep_ready(2)) {
			unsigned char scancodes[13] = { 0 };
			unsigned int nextscan = 0;

			if (enable_keyboard) {

				for (int i = 0; i < 9; i++) {
					if (buttons & (1 << i)) {
						scancodes[nextscan++] = 4 + i;
					}
				}

				if (buttons & (1 << 9)) { //start
					scancodes[nextscan++] = 40; // ENTER
				}

				if (buttons & (1 << 10)) { //select
					scancodes[nextscan++] = 42; // BACKSPACE
				}

				switch (tt1.poll()) {
					case -1:
						scancodes[nextscan++] = 4 + 21;
						break;
					case 1:
						scancodes[nextscan++] = 4 + 22;
						break;
				}

				switch (tt2.poll()) {
					case -1:
						scancodes[nextscan++] = 4 + 13;
						break;
					case 1:
						scancodes[nextscan++] = 4 + 14;
						break;
				}

			}

			while (nextscan < sizeof(scancodes)) {
				scancodes[nextscan++] = 0;
			}

			usb.write(2, (uint32_t*)scancodes, sizeof(scancodes));
		}
	}
}
