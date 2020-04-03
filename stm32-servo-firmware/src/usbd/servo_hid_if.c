
#include "servo_hid_if.h"

#include "hid/usage_desktop.h"
#include "hid/usage_lighting.h"

#include <stdbool.h>
#include <string.h>

// #include "hid/mouse.h"

#define REPORT_INTERVAL 100

enum {
	SERVO_HID_REPORT_JOYSTICK,
	SERVO_HID_REPORT_LA_ATTR,
	SERVO_HID_REPORT_LAMP_ATTR_REQ,
	SERVO_HID_REPORT_LAMP_ATTR_RESP,
	SERVO_HID_REPORT_LAMP_MULTI_UPDATE,
	SERVO_HID_REPORT_LAMP_RANGE_UPDATE,
	SERVO_HID_REPORT_LA_CTRL
};

static const uint8_t _servoReport[] __align(USBD_DATA_ALIGNMENT) = {
	HID_USAGE_PAGE_DESKTOP,
		HID_USAGE_DT_JOYSTICK,
		HID_COLLECTION_APPLICATION(
			HID_REPORT_ID(SERVO_HID_REPORT_JOYSTICK),
			HID_COLLECTION_PHYSICAL(
				HID_USAGE_PAGE_DESKTOP,
					HID_USAGE_DT_X,
		            HID_LOGICAL_MIN_16(INT16_MIN),
		            HID_LOGICAL_MAX_16(INT16_MAX),
		            HID_PHYSICAL_MIN_16(0),
		            HID_PHYSICAL_MAX_16(0xFFFF),
					HID_REPORT_SIZE(16),
					HID_REPORT_COUNT(1),
					HID_INPUT(Data_Var_Abs),
			),
		),

	// RGB LED controls based on lighting example
	HID_USAGE_PAGE_LIGHTING,
		HID_USAGE_LI_LAMPARRAY,
		HID_COLLECTION_APPLICATION(
			HID_REPORT_ID(SERVO_HID_REPORT_LA_ATTR),
			HID_USAGE_LI_LA_ATTR_REPORT,
			HID_COLLECTION_LOGICAL(
				HID_USAGE_PAGE_LIGHTING, // not sure if this needs repeating inside collection
					HID_USAGE_LI_LAMPCOUNT,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFF),
					HID_REPORT_SIZE(16),
					HID_REPORT_COUNT(1),
					HID_FEATURE(Const_Var_Abs),
					HID_USAGE_LI_BOUNDINGBOX_X,
					HID_USAGE_LI_BOUNDINGBOX_Y,
					HID_USAGE_LI_BOUNDINGBOX_Z,
					HID_USAGE_LI_LA_KIND,
					HID_USAGE_LI_MIN_UPDATE_INTERVAL,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFFFFFF),
					HID_REPORT_SIZE(32),
					HID_REPORT_COUNT(5),
					HID_FEATURE(Const_Var_Abs),
			),
			HID_REPORT_ID(SERVO_HID_REPORT_LAMP_ATTR_REQ),
			HID_USAGE_LI_LAMP_ATTR_REQ_REPORT,
			HID_COLLECTION_LOGICAL(
				HID_USAGE_PAGE_LIGHTING,
					HID_USAGE_LI_LAMPID,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFF),
					HID_REPORT_SIZE(16),
					HID_REPORT_COUNT(1),
					HID_FEATURE(Data_Var_Abs),
			),
			HID_REPORT_ID(SERVO_HID_REPORT_LAMP_ATTR_RESP),
			HID_USAGE_LI_LAMP_ATTR_RESP_REPORT,
			HID_COLLECTION_LOGICAL(
				HID_USAGE_PAGE_LIGHTING,
					HID_USAGE_LI_LAMPID,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFF),
					HID_REPORT_SIZE(16),
					HID_REPORT_COUNT(1),
					HID_FEATURE(Data_Var_Abs),
					HID_USAGE_LI_POSITION_X,
					HID_USAGE_LI_POSITION_Y,
					HID_USAGE_LI_POSITION_Z,
					HID_USAGE_LI_UPDATE_LATENCY,
					HID_USAGE_LI_LAMP_PURPOSES,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFFFFFF),
					HID_REPORT_SIZE(32),
					HID_REPORT_COUNT(5),
					HID_FEATURE(Data_Var_Abs),
					HID_USAGE_LI_RED_LEVEL_COUNT,
					HID_USAGE_LI_GREEN_LEVEL_COUNT,
					HID_USAGE_LI_BLUE_LEVEL_COUNT,
					HID_USAGE_LI_INTENSITY_LEVEL_COUNT,
					HID_USAGE_LI_PROGRAMMABLE,
					HID_USAGE_1(0x2D), // INPUT_BINDING
					HID_LOGICAL_MIN_8(0),
					HID_LOGICAL_MAX_8(0xFF),
					HID_REPORT_SIZE(8),
					HID_REPORT_COUNT(6),
					HID_FEATURE(Data_Var_Abs),
			),
			HID_REPORT_ID(SERVO_HID_REPORT_LAMP_MULTI_UPDATE),
			HID_USAGE_LI_LAMP_MULTI_UPDATE_REPORT,
			HID_COLLECTION_LOGICAL(
				HID_USAGE_PAGE_LIGHTING,
					HID_USAGE_LI_LAMPCOUNT,
					HID_USAGE_LI_LAMP_UPDATE_FLAGS,
					HID_LOGICAL_MIN_8(0),
					HID_LOGICAL_MAX_8(8),
					HID_REPORT_SIZE(8),
					HID_REPORT_COUNT(2),
					HID_FEATURE(Data_Var_Abs),
					HID_USAGE_LI_LAMPID,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFF),
					HID_REPORT_SIZE(16),
					HID_REPORT_COUNT(8),
					HID_FEATURE(Data_Var_Abs),
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_LOGICAL_MIN_8(0),
					HID_LOGICAL_MAX_8(0xFF),
					HID_REPORT_SIZE(8),
					HID_REPORT_COUNT(32),
					HID_FEATURE(Data_Var_Abs),
			),
			HID_REPORT_ID(SERVO_HID_REPORT_LAMP_RANGE_UPDATE),
			HID_USAGE_LI_LAMP_RANGE_UPDATE_REPORT,
			HID_COLLECTION_LOGICAL(
				HID_USAGE_PAGE_LIGHTING,
					HID_USAGE_LI_LAMP_UPDATE_FLAGS,
					HID_LOGICAL_MIN_8(0),
					HID_LOGICAL_MAX_8(8),
					HID_REPORT_SIZE(8),
					HID_REPORT_COUNT(1),
					HID_FEATURE(Data_Var_Abs),
					HID_USAGE_LI_LAMPID_START,
					HID_USAGE_LI_LAMPID_END,
					HID_LOGICAL_MIN_16(0),
					HID_LOGICAL_MAX_16(0xFFFF),
					HID_REPORT_SIZE(16),
					HID_REPORT_COUNT(2),
					HID_FEATURE(Data_Var_Abs),
					HID_USAGE_LI_RED_UPDATE_CHANNEL,
					HID_USAGE_LI_GREEN_UPDATE_CHANNEL,
					HID_USAGE_LI_BLUE_UPDATE_CHANNEL,
					HID_USAGE_LI_INTENSITY_UPDATE_CHANNEL,
					HID_LOGICAL_MIN_8(0),
					HID_LOGICAL_MAX_8(0xFF),
					HID_REPORT_SIZE(8),
					HID_REPORT_COUNT(4),
					HID_FEATURE(Data_Var_Abs),
			),
			HID_REPORT_ID(SERVO_HID_REPORT_LA_CTRL),
			HID_USAGE_LI_LA_CTRL_REPORT,
			HID_COLLECTION_LOGICAL(
				HID_USAGE_PAGE_LIGHTING,
					HID_USAGE_LI_AUTONOMOUS_MODE,
					HID_LOGICAL_MIN_8(0),
					HID_LOGICAL_MAX_8(1),
					HID_REPORT_SIZE(8),
					HID_REPORT_COUNT(1),
					HID_FEATURE(Data_Var_Abs),					
			),
		),
};

// static const uint8_t _servoReport[] = {
// 	HID_MOUSE_REPORT_DESC(),
// };

typedef struct {
	uint8_t reportId;
	uint16_t x;
} __packed servoInputReport_t;

typedef struct {
	uint8_t reportId;
	uint16_t lampCount;
	uint32_t boundsX, boundsY, boundsZ;
	uint32_t arrayKind, minUpdateInterval;
} __packed servoLampArrayAttributesFeatureReport_t;

typedef struct {
	uint8_t reportId;
	uint16_t lampId;
} __packed servoLampAttributeRequestFeatureReport_t;

typedef struct {
	uint8_t reportId;
	uint16_t lampId;
	uint32_t x, y, z;
	uint32_t latency, purpose;
	uint8_t redLevels, greenLevels, blueLevels, intensityLevels, programmable, inputBinding;
} __packed servoLampAttributeResponseFeatureReport_t;

typedef struct {
	uint8_t reportId;
	uint8_t count, flags;
	uint16_t lampIds[8];
	struct {
		uint8_t r,g,b,intensity;
	} __packed updates[8];
} __packed servoLampMultiUpdateFeatureReport_t;

typedef struct {
	uint8_t reportId;
	uint8_t flags;
	uint16_t startId, endId;
	uint8_t r,g,b,intensity;
} __packed servoLampRangeUpdateFeatureReport_t;

typedef struct {
	uint8_t reportId;
	uint8_t autonomousMode;
} __packed servoLampControlFeatureReport_t;

static const USBD_HID_ReportConfigType _servoReportConfig = {
        .Desc = _servoReport,
        .DescLength = sizeof(_servoReport),
        .Input.MaxSize = sizeof(servoInputReport_t),
        .Input.Interval_ms = REPORT_INTERVAL,
        .Feature.MaxSize = sizeof(servoLampMultiUpdateFeatureReport_t),
};

static bool _lampArrayAutonomousMode = true;

static void _sendInputReport(void* interface)
{
	static uint16_t counter = 0;
	servoInputReport_t report = {
		.reportId = SERVO_HID_REPORT_JOYSTICK,
		.x = ++counter,
	};

	USBD_HID_ReportIn(interface, &report, sizeof(report));
}

static void _getReport(void* interface, USBD_HID_ReportType type, uint8_t reportId)
{
	switch (type)
	{
		case HID_REPORT_INPUT:
		{
			_sendInputReport(interface);
			break;
		}
		case HID_REPORT_FEATURE:
		{	
			switch(reportId)
			{
				case SERVO_HID_REPORT_LA_ATTR:
				{
					servoLampArrayAttributesFeatureReport_t report = {
						.reportId = SERVO_HID_REPORT_LA_ATTR,
						.lampCount = 1,
						.boundsX = 1,
						.boundsY = 1,
						.boundsZ = 1,
						.arrayKind = 0x04, // peripheral
						.minUpdateInterval = 10,
					};
					USBD_HID_ReportIn(interface, &report, sizeof(report));
					break;
				}
				case SERVO_HID_REPORT_LAMP_ATTR_RESP:
				{
					servoLampAttributeResponseFeatureReport_t report = {
						.reportId = SERVO_HID_REPORT_LAMP_ATTR_RESP,
						.lampId = 0,
						.x= 0, .y = 0, .z = 0,
						.latency = 10,
						.purpose = 0x08, // status
					};
					USBD_HID_ReportIn(interface, &report, sizeof(report));
					break;
				}
				case SERVO_HID_REPORT_LA_CTRL:
				{
					servoLampControlFeatureReport_t report = {
						.reportId = SERVO_HID_REPORT_LA_CTRL,
						.autonomousMode = _lampArrayAutonomousMode,
					};
					USBD_HID_ReportIn(interface, &report, sizeof(report));
					break;
				}
			}
			break;
		}
		default:
		{
			// USBD_HID_ReportIn(interface, &feature, 0);
			break;
		}
	}
}
static void _setReport(void* interface, USBD_HID_ReportType type, uint8_t* data, uint16_t length)
{
	if (length > 0)
	{
		uint8_t reportId = data[0];
		switch(reportId)
		{
			case SERVO_HID_REPORT_LAMP_RANGE_UPDATE:
			{
				servoLampRangeUpdateFeatureReport_t report;
				if (length > sizeof(report))
				{
					memcpy(&report, data+1, sizeof(report));
					// TODO: update LED colors
				}
				break;
			}
			case SERVO_HID_REPORT_LAMP_MULTI_UPDATE:
			{
				servoLampMultiUpdateFeatureReport_t report;
				if (length > sizeof(report))
				{
					memcpy(&report, data+1, sizeof(report));
					// TODO: update LED colors
				}
				break;
			}
			case SERVO_HID_REPORT_LA_CTRL:
			{
				servoLampControlFeatureReport_t report;
				if (length > sizeof(report))
				{
					memcpy(&report, data+1, sizeof(report));
					_lampArrayAutonomousMode = report.autonomousMode;
				}
				break;
			}


		}
	}
}

static bool _sendReports = false;

static void _resume(void* interface)
{
	_sendReports = true;
}

static void _halt(void* interface)
{
	_sendReports = false;
}

void servo_hid_interface_run(uint32_t time_us)
{
	static uint32_t lastTime = 0;

	if (_sendReports && ((int)(time_us - lastTime) >= REPORT_INTERVAL))
	{
		lastTime = time_us;
		_sendInputReport(servo_hid_if);
	}
}

static const USBD_HID_AppType _servoApp =
{
    .Name       = "Servo Status Collection",
    .Init       = _resume,
    .Deinit     = _halt,
    .SetReport  = _setReport,
    .GetReport  = _getReport,
    .Report     = &_servoReportConfig,
};

static USBD_HID_IfHandleType _servo_hid_if = {
    .App = &_servoApp,
    .Base.AltCount = 1,
};

USBD_HID_IfHandleType* const servo_hid_if = &_servo_hid_if;


#if 0
// see also for force feedback: https://github.com/Ultrawipf/OpenFFBoard/blob/master/USB/Src/usbd_custom_hid_if.c

/*
	APPL(
		REPORT_ID 1
		
		USAGE_X
		
		PHYS(
			PAGE_DT
			USAGE_Y
		)

		PAGE_BUTTON

		PAGE_GENERIC_DT
		?

		PAGE_PHYS_INTERFACE
		USAGE_ES_PLAYING

		LOGICAL(
			REPORT_ID(2)
		)
	)
*/
// from https://www.microchip.com/forums/FindPost/378772
static const uint8_t ff_report[] = {
0x05,0x01,  //    Usage Page Generic Desktop
 0x09,0x04,  //    Usage Joystick
 0xA1,0x01,  //    Collection Application
    0x85,0x01,        //    Report ID 1
    0x09,0x30,        //    Usage X
    0x16,0x00,0xFE,   //    Logical Minimum FE00h (-512d)
    0x26,0xFF,0x01,   //    Logical Maximum 1FFh (511d)
    0x35,0x00,        //    Physical Minimum 0
    0x46,0xFF,0x03,   //    Physical Maximum 3FFh (1023d)
    0x75,0x0A,        //    Report Size Ah (10d)
    0x95,0x01,        //    Report Count 1
    0x81,0x02,        //    Input (Variable)
    0x75,0x06,        //    Report Size 6
    0x81,0x03,        //    Input (Constant, Variable)
    0xA1,0x00,        //    Collection Linked
       0x05,0x01,        //    Usage Page Generic Desktop
       0x09,0x31,        //    Usage Y
       0x15,0x00,        //    Logical Minimum 0
       0x25,0x3F,        //    Logical Maximum 3Fh (63d)
       0x35,0x00,        //    Physical Minimum 0
       0x45,0x3F,        //    Physical Maximum 3Fh (63d)
       0x75,0x06,        //    Report Size 6
       0x95,0x01,        //    Report Count 1
       0x81,0x02,        //    Input (Variable)
       0x75,0x02,        //    Report Size 2
       0x81,0x03,        //    Input (Constant, Variable)
       0x09,0x35,        //    Usage Rz
       0x75,0x06,        //    Report Size 6
       0x81,0x02,        //    Input (Variable)
       0x75,0x02,        //    Report Size 2
       0x81,0x03,        //    Input (Constant, Variable)
    0xC0    ,         //    End Collection
    0x05,0x09,        //    Usage Page Button
    0x15,0x00,        //    Logical Minimum 0
    0x19,0x01,        //    Usage Minimum Button 1
    0x29,0x08,        //    Usage Maximum Button 8
    0x25,0x01,        //    Logical Maximum 1
    0x35,0x00,        //    Physical Minimum 0
    0x45,0x01,        //    Physical Maximum 1
    0x75,0x01,        //    Report Size 1
    0x95,0x08,        //    Report Count 8
    0x81,0x02,        //    Input (Variable)
    0x06,0x01,0xFF,   //    Usage Page Generic Desktop
    0x09,0x49,        //    Usage Undefined
    0x75,0x01,        //    Report Size 1
    0x95,0x01,        //    Report Count 1
    0x81,0x02,        //    Input (Variable)
    0x75,0x07,        //    Report Size 7
    0x81,0x03,        //    Input (Constant, Variable)
    0x05,0x0F,        //    Usage Page Physical Interface
    0x09,0x92,        //    Usage ES Playing
    0xA1,0x02,        //    Collection Datalink
       0x85,0x02,    //    Report ID 2
       0x09,0x9F,    //    Usage DS Device is Reset
       0x09,0xA0,    //    Usage DS Device is Pause
       0x09,0xA4,    //    Usage Actuator Power
       0x09,0xA5,    //    Usage Undefined
       0x09,0xA6,    //    Usage Undefined
       0x15,0x00,    //    Logical Minimum 0
       0x25,0x01,    //    Logical Maximum 1
       0x35,0x00,    //    Physical Minimum 0
       0x45,0x01,    //    Physical Maximum 1
       0x75,0x01,    //    Report Size 1
       0x95,0x05,    //    Report Count 5
       0x81,0x02,    //    Input (Variable)
       0x95,0x03,    //    Report Count 3
       0x81,0x03,    //    Input (Constant, Variable)
       0x09,0x94,    //    Usage PID Device Control
       0x15,0x00,    //    Logical Minimum 0
       0x25,0x01,    //    Logical Maximum 1
       0x35,0x00,    //    Physical Minimum 0
       0x45,0x01,    //    Physical Maximum 1
       0x75,0x01,    //    Report Size 1
       0x95,0x01,    //    Report Count 1
       0x81,0x02,    //    Input (Variable)
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x07,    //    Report Size 7
       0x95,0x01,    //    Report Count 1
       0x81,0x02,    //    Input (Variable)
    0xC0    ,    // End Collection
    0x09,0x21,    //    Usage Set Effect Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x01,    //    Report ID 1
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x25,    //    Usage Effect Type
       0xA1,0x02,    //    Collection Datalink
          0x09,0x26,    //    Usage ET Constant Force
          0x09,0x27,    //    Usage ET Ramp
          0x09,0x30,    //    Usage ET Square
          0x09,0x31,    //    Usage ET Sine
          0x09,0x32,    //    Usage ET Triangle
          0x09,0x33,    //    Usage ET Sawtooth Up
          0x09,0x34,    //    Usage ET Sawtooth Down
          0x09,0x40,    //    Usage ET Spring
          0x09,0x41,    //    Usage ET Damper
          0x09,0x42,    //    Usage ET Inertia
          0x09,0x43,    //    Usage ET Friction
          0x09,0x28,    //    Usage ET Custom Force Data
          0x25,0x0C,    //    Logical Maximum Ch (12d)
          0x15,0x01,    //    Logical Minimum 1
          0x35,0x01,    //    Physical Minimum 1
          0x45,0x0C,    //    Physical Maximum Ch (12d)
          0x75,0x08,    //    Report Size 8
          0x95,0x01,    //    Report Count 1
          0x91,0x00,    //    Output
       0xC0    ,          //    End Collection
       0x09,0x50,         //    Usage Duration
       0x09,0x54,         //    Usage Trigger Repeat Interval
       0x09,0x51,         //    Usage Sample Period
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x03,         //    Report Count 3
       0x91,0x02,         //    Output (Variable)
       0x55,0x00,         //    Unit Exponent 0
       0x66,0x00,0x00,    //    Unit 0
       0x09,0x52,         //    Usage Gain
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x53,         //    Usage Trigger Button
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x08,         //    Logical Maximum 8
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x08,         //    Physical Maximum 8
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x55,         //    Usage Axes Enable
       0xA1,0x02,         //    Collection Datalink
          0x05,0x01,    //    Usage Page Generic Desktop
          0x09,0x30,    //    Usage X
          0x09,0x31,    //    Usage Y
          0x15,0x00,    //    Logical Minimum 0
          0x25,0x01,    //    Logical Maximum 1
          0x75,0x01,    //    Report Size 1
          0x95,0x02,    //    Report Count 2
          0x91,0x02,    //    Output (Variable)
       0xC0     ,    // End Collection
       0x05,0x0F,    //    Usage Page Physical Interface
       0x09,0x56,    //    Usage Direction Enable
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x95,0x05,    //    Report Count 5
       0x91,0x03,    //    Output (Constant, Variable)
       0x09,0x57,    //    Usage Direction
       0xA1,0x02,    //    Collection Datalink
          0x0B,0x01,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 1
          0x0B,0x02,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 2
          0x66,0x14,0x00,              //    Unit 14h (20d)
          0x55,0xFE,                   //    Unit Exponent FEh (254d)
          0x15,0x00,                   //    Logical Minimum 0
          0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
          0x35,0x00,                   //    Physical Minimum 0
          0x47,0xA0,0x8C,0x00,0x00,    //    Physical Maximum 8CA0h (36000d)
          0x66,0x00,0x00,              //    Unit 0
          0x75,0x08,                   //    Report Size 8
          0x95,0x02,                   //    Report Count 2
          0x91,0x02,                   //    Output (Variable)
          0x55,0x00,                   //    Unit Exponent 0
          0x66,0x00,0x00,              //    Unit 0
       0xC0     ,         //    End Collection
       0x05,0x0F,         //    Usage Page Physical Interface
       0x09,0xA7,         //    Usage Undefined
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x66,0x00,0x00,    //    Unit 0
       0x55,0x00,         //    Unit Exponent 0
    0xC0     ,    //    End Collection
    0x05,0x0F,    //    Usage Page Physical Interface
    0x09,0x5A,    //    Usage Set Envelope Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x02,         //    Report ID 2
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x5B,         //    Usage Attack Level
       0x09,0x5D,         //    Usage Fade Level
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
       0x09,0x5C,         //    Usage Attack Time
       0x09,0x5E,         //    Usage Fade Time
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x91,0x02,         //    Output (Variable)
       0x45,0x00,         //    Physical Maximum 0
       0x66,0x00,0x00,    //    Unit 0
       0x55,0x00,         //    Unit Exponent 0
    0xC0     ,            //    End Collection
    0x09,0x5F,    //    Usage Set Condition Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x03,    //    Report ID 3
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x23,    //    Usage Parameter Block Offset
       0x15,0x00,    //    Logical Minimum 0
       0x25,0x01,    //    Logical Maximum 1
       0x35,0x00,    //    Physical Minimum 0
       0x45,0x01,    //    Physical Maximum 1
       0x75,0x04,    //    Report Size 4
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x58,    //    Usage Type Specific Block Off...
       0xA1,0x02,    //    Collection Datalink
          0x0B,0x01,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 1
          0x0B,0x02,0x00,0x0A,0x00,    //    Usage Ordinals: Instance 2
          0x75,0x02,                   //    Report Size 2
          0x95,0x02,                   //    Report Count 2
          0x91,0x02,                   //    Output (Variable)
       0xC0     ,         //    End Collection
       0x15,0x80,         //    Logical Minimum 80h (-128d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x09,0x60,         //    Usage CP Offset
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x09,0x61,         //    Usage Positive Coefficient
       0x09,0x62,         //    Usage Negative Coefficient
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x09,0x63,         //    Usage Positive Saturation
       0x09,0x64,         //    Usage Negative Saturation
       0x75,0x08,         //    Report Size 8
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
       0x09,0x65,         //    Usage Dead Band
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x6E,    //    Usage Set Periodic Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x04,                   //    Report ID 4
       0x09,0x22,                   //    Usage Effect Block Index
       0x15,0x01,                   //    Logical Minimum 1
       0x25,0x28,                   //    Logical Maximum 28h (40d)
       0x35,0x01,                   //    Physical Minimum 1
       0x45,0x28,                   //    Physical Maximum 28h (40d)
       0x75,0x08,                   //    Report Size 8
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x09,0x70,                   //    Usage Magnitude
       0x15,0x00,                   //    Logical Minimum 0
       0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
       0x35,0x00,                   //    Physical Minimum 0
       0x46,0x10,0x27,              //    Physical Maximum 2710h (10000d)
       0x75,0x08,                   //    Report Size 8
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x09,0x6F,                   //    Usage Offset
       0x15,0x80,                   //    Logical Minimum 80h (-128d)
       0x25,0x7F,                   //    Logical Maximum 7Fh (127d)
       0x36,0xF0,0xD8,              //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,              //    Physical Maximum 2710h (10000d)
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x09,0x71,                   //    Usage Phase
       0x66,0x14,0x00,              //    Unit 14h (20d)
       0x55,0xFE,                   //    Unit Exponent FEh (254d)
       0x15,0x00,                   //    Logical Minimum 0
       0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
       0x35,0x00,                   //    Physical Minimum 0
       0x47,0xA0,0x8C,0x00,0x00,    //    Physical Maximum 8CA0h (36000d)
       0x91,0x02,                   //    Output (Variable)
       0x09,0x72,                   //    Usage Period
       0x26,0xFF,0x7F,              //    Logical Maximum 7FFFh (32767d)
       0x46,0xFF,0x7F,              //    Physical Maximum 7FFFh (32767d)
       0x66,0x03,0x10,              //    Unit 1003h (4099d)
       0x55,0xFD,                   //    Unit Exponent FDh (253d)
       0x75,0x10,                   //    Report Size 10h (16d)
       0x95,0x01,                   //    Report Count 1
       0x91,0x02,                   //    Output (Variable)
       0x66,0x00,0x00,              //    Unit 0
       0x55,0x00,                   //    Unit Exponent 0
    0xC0     ,    // End Collection
    0x09,0x73,    //    Usage Set Constant Force Rep...
    0xA1,0x02,    //    Collection Datalink
       0x85,0x05,         //    Report ID 5
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x70,         //    Usage Magnitude
       0x16,0x01,0xFF,    //    Logical Minimum FF01h (-255d)
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x74,    //    Usage Set Ramp Force Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x06,         //    Report ID 6
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x75,         //    Usage Ramp Start
       0x09,0x76,         //    Usage Ramp End
       0x15,0x80,         //    Logical Minimum 80h (-128d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x36,0xF0,0xD8,    //    Physical Minimum D8F0h (-10000d)
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x08,         //    Report Size 8
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x68,    //    Usage Custom Force Data Rep...
    0xA1,0x02,    //    Collection Datalink
       0x85,0x07,         //    Report ID 7
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x6C,         //    Usage Custom Force Data Offset
       0x15,0x00,         //    Logical Minimum 0
       0x26,0x10,0x27,    //    Logical Maximum 2710h (10000d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x69,         //    Usage Custom Force Data
       0x15,0x81,         //    Logical Minimum 81h (-127d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x75,0x08,         //    Report Size 8
       0x95,0x0C,         //    Report Count Ch (12d)
       0x92,0x02,0x01,    //       Output (Variable, Buffered)
    0xC0     ,    //    End Collection
    0x09,0x66,    //    Usage Download Force Sample
    0xA1,0x02,    //    Collection Datalink
       0x85,0x08,         //    Report ID 8
       0x05,0x01,         //    Usage Page Generic Desktop
       0x09,0x30,         //    Usage X
       0x09,0x31,         //    Usage Y
       0x15,0x81,         //    Logical Minimum 81h (-127d)
       0x25,0x7F,         //    Logical Maximum 7Fh (127d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x75,0x08,         //    Report Size 8
       0x95,0x02,         //    Report Count 2
       0x91,0x02,         //    Output (Variable)
    0xC0     ,   //    End Collection
    0x05,0x0F,   //    Usage Page Physical Interface
    0x09,0x77,   //    Usage Effect Operation Report
    0xA1,0x02,   //    Collection Datalink
       0x85,0x0A,    //    Report ID Ah (10d)
       0x09,0x22,    //    Usage Effect Block Index
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
       0x09,0x78,    //    Usage Operation
       0xA1,0x02,    //    Collection Datalink
          0x09,0x79,    //    Usage Op Effect Start
          0x09,0x7A,    //    Usage Op Effect Start Solo
          0x09,0x7B,    //    Usage Op Effect Stop
          0x15,0x01,    //    Logical Minimum 1
          0x25,0x03,    //    Logical Maximum 3
          0x75,0x08,    //    Report Size 8
          0x95,0x01,    //    Report Count 1
          0x91,0x00,    //    Output
       0xC0     ,         //    End Collection
       0x09,0x7C,         //    Usage Loop Count
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x91,0x02,         //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x90,    //    Usage PID State Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0B,    //    Report ID Bh (11d)
       0x09,0x22,    //    Usage Effect Block Index
       0x25,0x28,    //    Logical Maximum 28h (40d)
       0x15,0x01,    //    Logical Minimum 1
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x28,    //    Physical Maximum 28h (40d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x02,    //    Output (Variable)
    0xC0     ,    //    End Collection
    0x09,0x96,    //    Usage DC Disable Actuators
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0C,    //    Report ID Ch (12d)
       0x09,0x97,    //    Usage DC Stop All Effects
       0x09,0x98,    //    Usage DC Device Reset
       0x09,0x99,    //    Usage DC Device Pause
       0x09,0x9A,    //    Usage DC Device Continue
       0x09,0x9B,    //    Usage PID Device State
       0x09,0x9C,    //    Usage DS Actuators Enabled
       0x15,0x01,    //    Logical Minimum 1
       0x25,0x06,    //    Logical Maximum 6
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0x91,0x00,    //    Output
    0xC0     ,    //    End Collection
    0x09,0x7D,    //    Usage PID Pool Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0D,         //    Report ID Dh (13d)
       0x09,0x7E,         //    Usage RAM Pool Size
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0x10,0x27,    //    Physical Maximum 2710h (10000d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
    0xC0     ,            //    End Collection
    0x09,0x6B,    //    Usage Set Custom Force Report
    0xA1,0x02,    //    Collection Datalink
       0x85,0x0E,         //    Report ID Eh (14d)
       0x09,0x22,         //    Usage Effect Block Index
       0x15,0x01,         //    Logical Minimum 1
       0x25,0x28,         //    Logical Maximum 28h (40d)
       0x35,0x01,         //    Physical Minimum 1
       0x45,0x28,         //    Physical Maximum 28h (40d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x6D,         //    Usage Sample Count
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x00,    //    Logical Maximum FFh (255d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x00,    //    Physical Maximum FFh (255d)
       0x75,0x08,         //    Report Size 8
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x09,0x51,         //    Usage Sample Period
       0x66,0x03,0x10,    //    Unit 1003h (4099d)
       0x55,0xFD,         //    Unit Exponent FDh (253d)
       0x15,0x00,         //    Logical Minimum 0
       0x26,0xFF,0x7F,    //    Logical Maximum 7FFFh (32767d)
       0x35,0x00,         //    Physical Minimum 0
       0x46,0xFF,0x7F,    //    Physical Maximum 7FFFh (32767d)
       0x75,0x10,         //    Report Size 10h (16d)
       0x95,0x01,         //    Report Count 1
       0x91,0x02,         //    Output (Variable)
       0x55,0x00,         //    Unit Exponent 0
       0x66,0x00,0x00,    //    Unit 0
    0xC0     ,    //    End Collection
    0x09,0xAB,    //    Usage Undefined
    0xA1,0x02,    //    Collection Datalink
       0x85,0x01,    //    Report ID 1
       0x09,0x25,    //    Usage Effect Type
       0xA1,0x02,    //    Collection Datalink
       0x09,0x26,    //    Usage ET Constant Force
       0x09,0x27,    //    Usage ET Ramp
       0x09,0x30,    //    Usage ET Square
       0x09,0x31,    //    Usage ET Sine
       0x09,0x32,    //    Usage ET Triangle
       0x09,0x33,    //    Usage ET Sawtooth Up
       0x09,0x34,    //    Usage ET Sawtooth Down
       0x09,0x40,    //    Usage ET Spring
       0x09,0x41,    //    Usage ET Damper
       0x09,0x42,    //    Usage ET Inertia
       0x09,0x43,    //    Usage ET Friction
       0x09,0x28,    //    Usage ET Custom Force Data
       0x25,0x0C,    //    Logical Maximum Ch (12d)
       0x15,0x01,    //    Logical Minimum 1
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x0C,    //    Physical Maximum Ch (12d)
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0xB1,0x00,    //    Feature
    0xC0     ,    // End Collection
    0x05,0x01,         //    Usage Page Generic Desktop
    0x09,0x3B,         //    Usage Byte Count
    0x15,0x00,         //    Logical Minimum 0
    0x26,0xFF,0x01,    //    Logical Maximum 1FFh (511d)
    0x35,0x00,         //    Physical Minimum 0
    0x46,0xFF,0x01,    //    Physical Maximum 1FFh (511d)
    0x75,0x0A,         //    Report Size Ah (10d)
    0x95,0x01,         //    Report Count 1
    0xB1,0x02,         //    Feature (Variable)
    0x75,0x06,         //    Report Size 6
    0xB1,0x01,         //    Feature (Constant)
 0xC0     ,    //    End Collection
 0x05,0x0F,    //    Usage Page Physical Interface
 0x09,0x89,    //    Usage Block Load Status
 0xA1,0x02,    //    Collection Datalink
    0x85,0x02,    //    Report ID 2
    0x09,0x22,    //    Usage Effect Block Index
    0x25,0x28,    //    Logical Maximum 28h (40d)
    0x15,0x01,    //    Logical Minimum 1
    0x35,0x01,    //    Physical Minimum 1
    0x45,0x28,    //    Physical Maximum 28h (40d)
    0x75,0x08,    //    Report Size 8
    0x95,0x01,    //    Report Count 1
    0xB1,0x02,    //    Feature (Variable)
    0x09,0x8B,    //    Usage Block Load Full
    0xA1,0x02,    //    Collection Datalink
       0x09,0x8C,    //    Usage Block Load Error
       0x09,0x8D,    //    Usage Block Handle
       0x09,0x8E,    //    Usage PID Block Free Report
       0x25,0x03,    //    Logical Maximum 3
       0x15,0x01,    //    Logical Minimum 1
       0x35,0x01,    //    Physical Minimum 1
       0x45,0x03,    //    Physical Maximum 3
       0x75,0x08,    //    Report Size 8
       0x95,0x01,    //    Report Count 1
       0xB1,0x00,    //    Feature
    0xC0     ,                   // End Collection
    0x09,0xAC,                   //    Usage Undefined
    0x15,0x00,                   //    Logical Minimum 0
    0x27,0xFF,0xFF,0x00,0x00,    //    Logical Maximum FFFFh (65535d)
    0x35,0x00,                   //    Physical Minimum 0
    0x47,0xFF,0xFF,0x00,0x00,    //    Physical Maximum FFFFh (65535d)
    0x75,0x10,                   //    Report Size 10h (16d)
    0x95,0x01,                   //    Report Count 1
    0xB1,0x00,                   //    Feature
 0xC0     ,    //    End Collection
 0x09,0x7F,    //    Usage ROM Pool Size
 0xA1,0x02,    //    Collection Datalink
    0x85,0x03,                   //    Report ID 3
    0x09,0x80,                   //    Usage ROM Effect Block Count
    0x75,0x10,                   //    Report Size 10h (16d)
    0x95,0x01,                   //    Report Count 1
    0x15,0x00,                   //    Logical Minimum 0
    0x35,0x00,                   //    Physical Minimum 0
    0x27,0xFF,0xFF,0x00,0x00,    //    Logical Maximum FFFFh (65535d)
    0x47,0xFF,0xFF,0x00,0x00,    //    Physical Maximum FFFFh (65535d)
    0xB1,0x02,                   //    Feature (Variable)
    0x09,0x83,                   //    Usage PID Pool Move Report
    0x26,0xFF,0x00,              //    Logical Maximum FFh (255d)
    0x46,0xFF,0x00,              //    Physical Maximum FFh (255d)
    0x75,0x08,                   //    Report Size 8
    0x95,0x01,                   //    Report Count 1
    0xB1,0x02,                   //    Feature (Variable)
    0x09,0xA9,                   //    Usage Undefined
    0x09,0xAA,                   //    Usage Undefined
    0x75,0x01,                   //    Report Size 1
    0x95,0x02,                   //    Report Count 2
    0x15,0x00,                   //    Logical Minimum 0
    0x25,0x01,                   //    Logical Maximum 1
    0x35,0x00,                   //    Physical Minimum 0
    0x45,0x01,                   //    Physical Maximum 1
    0xB1,0x02,                   //    Feature (Variable)
    0x75,0x06,                   //    Report Size 6
    0x95,0x01,                   //    Report Count 1
    0xB1,0x03,                   //    Feature (Constant, Variable)
    0xC0,    //    End Collection
 0xC0    //    End Collection

};
#endif // if 0
