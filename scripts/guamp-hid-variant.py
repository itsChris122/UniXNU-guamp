"""Apply the owned HID collection variant to the exact pinned build-tree source.

Step 1 (unchanged): opt-in Consumer-only primary collection (guamp-hid-buttons-only=1).
Step 2: ring/silent switch. Report 2 keeps its two bytes and both collections keep their usages: the four constant
padding bits after Menu/Power/VolUp/VolDown become one Telephony (0x0B) usage 0x2E input bit (bit 4) and three padding
bits. backboardd (BKButtonHIDEventProcessor, page 0x0B usage 0x2E -> _processRingerSwitchEvent:down:) and SpringBoard
(__handleHIDEvent: 0xB002E -> _handleRingerSwitchEvent:) read it as a level: 1 ring, 0 silent. IOHIDEventDriver
stores Telephony elements as keyboard elements and dispatches them on value change. The button sink keeps bit 4.
Step 3 (28/10): sheng-touch-hid defaults to 1 (the touch screen collection the USB mirror's touches use, verified on every
guamp boot line); sheng-touch-hid=0 turns it off.
"""
from pathlib import Path
import hashlib
root=Path(__file__).resolve().parents[1]
path=root/'drivers/NativeHidInput/NativeHidInput.cpp'
PRISTINE='0445e5f451ffcc827bdd603e822180444d998427fa3a6d63559b1a7b8f850998'
CONSUMER='dc43b25ae0dc0191ab7a23674c3604262149769d7fa55c1372abe5135eea2929'
FINAL=None
RINGER='f5fa8fa8d63f498d648435957e0e2f5fa3dabba09d59cff930a34fc4e9bc58d0'
FINAL_RECORDED='08eff226eb1aad34be27b5aab1e21129ca5eed32faed072e20f30c98ec2a03f5'
SENSORS_RECORDED='7432dbacae81b5568d5c72befa444a708205278c1664aa0fc462dd709b2c86ae'
# Step 4 edits. Axes: logical/physical -32768..32767 in 32-bit fields, unit exponent -3 (milli-g, mrad/s), as
# IOHIDFamily's IOHIDGyroDescriptor.h; feature Report Interval (us) so the driver publishes the ReportInterval property.
SENSOR_DESCRIPTORS=r'''#define TOUCH_FINGERS 10
// Sensor devices (guamp-sensors >= 1; one IOHIDDevice per sensor, analisi/sensori/SENSORI-20261029.md). Report 4 accelerometer
// (Sensor 0x73: X/Y/Z milli-g), 5 gyro (Sensor 0x7A: X/Y/Z mrad/s), 6 device orientation (Sensor 0x8A: value 1..8 = AppleVendorMotion
// 0xFF0C usages 0x64..0x6B, 1 ambiguous 2 portrait 3 upside down 4 landscape left 5 landscape right 6 face up 7 face down),
// 7 proximity (Consumer Control with Consumer 0x109 Proximity: 1 near, 0 far). Producer: native_sensor_set_sink (NativeEmmc).
static const unsigned char accelDescriptor[]={
    0x05,0x20,0x09,0x73,0xa1,0x01,0x85,0x04,
    0x16,0x00,0x80,0x26,0xff,0x7f,0x36,0x00,0x80,0x46,0xff,0x7f,0x55,0x0d,0x75,0x20,0x95,0x01,
    0x0a,0x53,0x04,0x81,0x02, 0x0a,0x54,0x04,0x81,0x02, 0x0a,0x55,0x04,0x81,0x02,
    0x55,0x00,0x0a,0x0e,0x03,0x15,0x00,0x27,0xff,0xff,0xff,0x7f,0x35,0x00,0x47,0xff,0xff,0xff,0x7f,0x75,0x20,0x95,0x01,0xb1,0x02,
    0xc0
};
static const unsigned char gyroDescriptor[]={
    0x05,0x20,0x09,0x7a,0xa1,0x01,0x85,0x05,
    0x16,0x00,0x80,0x26,0xff,0x7f,0x36,0x00,0x80,0x46,0xff,0x7f,0x55,0x0d,0x75,0x20,0x95,0x01,
    0x0a,0x57,0x04,0x81,0x02, 0x0a,0x58,0x04,0x81,0x02, 0x0a,0x59,0x04,0x81,0x02,
    0x55,0x00,0x0a,0x0e,0x03,0x15,0x00,0x27,0xff,0xff,0xff,0x7f,0x35,0x00,0x47,0xff,0xff,0xff,0x7f,0x75,0x20,0x95,0x01,0xb1,0x02,
    0xc0
};
static const unsigned char orientDescriptor[]={
    0x05,0x20,0x09,0x8a,0xa1,0x01,0x85,0x06,
    0x06,0x0c,0xff,0x19,0x64,0x2a,0x6b,0x00,0x15,0x01,0x25,0x08,0x95,0x01,0x75,0x08,0x81,0x00,
    0xc0
};
static const unsigned char proxDescriptor[]={
    0x05,0x0c,0x09,0x01,0xa1,0x01,0x85,0x07,
    0x0a,0x09,0x01,0x15,0x00,0x25,0x01,0x75,0x08,0x95,0x01,0x81,0x02,
    0xc0
};
extern "C" void native_sensor_set_sink(unsigned, void (*)(void *, const int32_t *), void *) __attribute__((weak_import));
// [kind]: reports sent to the event driver; results: last handleReport status (1 accel, 2 gyro, 3 orientation, 4 proximity)
extern "C" {volatile uint32_t native_hid_sensor_reports[5];volatile int native_hid_sensor_results[5];}
'''
SENSOR_METHODS=r'''    // Sensor devices: the hub (NativeEmmc GuampSensors.inc) calls sensorNotify with three values per sample, any thread.
    static void sensorNotify(void *context,const int32_t *values) {
        NativeHidInput *self=static_cast<NativeHidInput *>(context);
        if(self && self->inputLoop && values)self->inputLoop->runAction(&NativeHidInput::sensorAction,self,const_cast<int32_t *>(values));
    }
    static IOReturn sensorAction(OSObject *owner,void *values,void *,void *,void *) {
        NativeHidInput *self=OSDynamicCast(NativeHidInput,owner);if(!self || !values)return kIOReturnSuccess;
        const unsigned kind=self->sensorKind;
        if(kind<1 || kind>4 || !self->hasEventDriverCached())return kIOReturnSuccess;
        const int32_t *v=static_cast<const int32_t *>(values);
        unsigned char report[13]={};
        size_t size=2;
        if(kind<=2) {
            report[0]=kind==1?4:5;
            for(unsigned i=0;i<3;++i) {
                int32_t c=v[i];if(c>32767)c=32767;if(c<-32768)c=-32768;
                const uint32_t u=(uint32_t)c;
                report[1+4*i]=(unsigned char)u;report[2+4*i]=(unsigned char)(u>>8);
                report[3+4*i]=(unsigned char)(u>>16);report[4+4*i]=(unsigned char)(u>>24);
            }
            size=13;
        } else if(kind==3) {report[0]=6;report[1]=(unsigned char)((v[0]>=1 && v[0]<=8)?v[0]:0);}
        else {report[0]=7;report[1]=v[0]?1:0;}
        IOBufferMemoryDescriptor *buffer=IOBufferMemoryDescriptor::withBytes(report,size,kIODirectionNone);
        IOReturn result=buffer?self->handleReport(buffer,0,0):kIOReturnNoMemory;
        OSSafeReleaseNULL(buffer);
        native_hid_sensor_reports[kind]=native_hid_sensor_reports[kind]+1;native_hid_sensor_results[kind]=result;
        return kIOReturnSuccess;
    }
    // The button driver's thread calls buttonsNotify on every change of the pressed set.
'''
SENSOR_DESCRIPTOR_CODE=r'''        if(sensorKind>=1 && sensorKind<=4) {
            static_assert(sizeof(accelDescriptor)<=sizeof(combined) && sizeof(gyroDescriptor)<=sizeof(combined),"sensor descriptor fits");
            const unsigned char *d=sensorKind==1?accelDescriptor:sensorKind==2?gyroDescriptor:sensorKind==3?orientDescriptor:proxDescriptor;
            size=sensorKind==1?sizeof(accelDescriptor):sensorKind==2?sizeof(gyroDescriptor):sensorKind==3?sizeof(orientDescriptor):sizeof(proxDescriptor);
            memcpy(combined,d,size);
        } else if(touchOnly){size=sizeof(touchDescriptor);memcpy(combined,touchDescriptor,size);}
'''
SENSOR_EDITS=[
    ('#define TOUCH_FINGERS 10\n',SENSOR_DESCRIPTORS),
    ('    bool touchOnly=false;\n','    bool touchOnly=false;\n    unsigned sensorKind=0;   // 1 accel, 2 gyro, 3 orientation, 4 proximity; 0 = keyboard/touch device\n'),
    ('    // The button driver\'s thread calls buttonsNotify on every change of the pressed set.\n',SENSOR_METHODS),
    ('        if(self->touchOnly){source->setTimeoutMS(1000);return;}','        if(self->touchOnly || self->sensorKind){source->setTimeoutMS(1000);return;}'),
    ('    void setTouchOnly() {touchOnly=true;}','    void setTouchOnly() {touchOnly=true;}\n    void setSensorKind(unsigned kind) {sensorKind=kind;}'),
    ('return OSString::withCString(touchOnly?"SPI":"Virtual");}','return OSString::withCString(sensorKind?"I2C":touchOnly?"SPI":"Virtual");}'),
    ('return OSString::withCString(touchOnly?"Offline touch digitizer":"Offline HID input probe");}',
     'return OSString::withCString(sensorKind==1?"Guamp accelerometer":sensorKind==2?"Guamp gyro":sensorKind==3?"Guamp device orientation":sensorKind==4?"Guamp proximity":touchOnly?"Offline touch digitizer":"Offline HID input probe");}'),
    ('        if(touchOnly){size=sizeof(touchDescriptor);memcpy(combined,touchDescriptor,size);}\n',SENSOR_DESCRIPTOR_CODE),
    ('        PE_parse_boot_argn("sheng-touch-hid",&touch,sizeof(touch));touchEnabled=touch==1;\n',
     '        PE_parse_boot_argn("sheng-touch-hid",&touch,sizeof(touch));touchEnabled=touch==1;\n'
     '        if(sensorKind){touchEnabled=false;setProperty("Built-In",true);}   // the orientation personality matches Built-In\n'),
    ('        if(!touchOnly && &native_buttons_set_sink)native_buttons_set_sink(&NativeHidInput::buttonsNotify,this);\n',
     '        if(!touchOnly && !sensorKind && &native_buttons_set_sink)native_buttons_set_sink(&NativeHidInput::buttonsNotify,this);\n'
     '        if(sensorKind && &native_sensor_set_sink)native_sensor_set_sink(sensorKind,&NativeHidInput::sensorNotify,this);\n'),
    ('        if(!touchOnly && &native_buttons_set_sink)native_buttons_set_sink(nullptr,nullptr);\n',
     '        if(!touchOnly && !sensorKind && &native_buttons_set_sink)native_buttons_set_sink(nullptr,nullptr);\n'
     '        if(sensorKind && &native_sensor_set_sink)native_sensor_set_sink(sensorKind,nullptr,nullptr);\n'),
    ('if(device)device->setTouchOnly();return device;}\n',
     'if(device)device->setTouchOnly();return device;}\n'
     'extern "C" IOService *native_hid_sensor_allocate(unsigned kind) {\n'
     '    if(kind<1 || kind>4)return nullptr;\n'
     '    NativeHidInput *device=new NativeHidInput;if(device)device->setSensorKind(kind);return device;\n'
     '}\n'),
]
raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
text=raw.decode()
if digest==PRISTINE:
    assert text.count('// Report 3: ten-finger Touch Screen collection')==1 and text.count('            size=sizeof(descriptor);memcpy(combined,descriptor,size);')==1
    text=text.replace('// Report 3: ten-finger Touch Screen collection','static const unsigned char guampConsumerDescriptor[] = {0x05,0x0c,0x09,0x01,0xa1,0x01,0x85,0x02,0x09,0x40,0x09,0x30,0x09,0xe9,0x09,0xea,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x04,0x81,0x02,0x75,0x04,0x95,0x01,0x81,0x03,0xc0};\n'+'// Report 3: ten-finger Touch Screen collection').replace('            size=sizeof(descriptor);memcpy(combined,descriptor,size);','            unsigned buttonsOnly=0;\n            PE_parse_boot_argn("guamp-hid-buttons-only", &buttonsOnly, sizeof(buttonsOnly));\n            const unsigned char *primary=buttonsOnly==1?guampConsumerDescriptor:descriptor;\n            size=buttonsOnly==1?sizeof(guampConsumerDescriptor):sizeof(descriptor);\n            memcpy(combined,primary,size);\n            IOLog("GuampHid: keyboard collection %s; consumer report 2 preserved; touch enabled %u\\n",\n                  buttonsOnly==1?"disabled":"enabled", touchEnabled?1u:0u);')
    assert hashlib.sha256(text.encode()).hexdigest()==CONSUMER
    digest=CONSUMER
    print('GuampHid: Consumer-only opt-in prepared; report 2 and digitizer retained')
if digest==CONSUMER:
    ringer='0x05,0x0b,0x09,0x2e,0x15,0x00,0x25,0x01,0x75,0x01,0x95,0x01,0x81,0x02,0x75,0x03,0x95,0x01,0x81,0x03,'
    old_main='    0x75,0x04,0x95,0x01,0x81,0x03,0xc0\n};'
    new_main=('    // bit 4: Telephony (0x0B) usage 0x2E, the ring/silent switch (level: 1 ring, 0 silent); bits 5..7 constant\n'
              '    '+ringer[:-1].replace(',0x75,0x03',',\n    0x75,0x03')+',0xc0\n};')
    old_consumer='0x75,0x01,0x95,0x04,0x81,0x02,0x75,0x04,0x95,0x01,0x81,0x03,0xc0};'
    new_consumer='0x75,0x01,0x95,0x04,0x81,0x02,'+ringer+'0xc0};'
    old_mask='self->buttons=static_cast<uint8_t>(uintptr_t(mask)&0xe);'
    new_mask='self->buttons=static_cast<uint8_t>(uintptr_t(mask)&0x1e);   // bit 4: ring/silent switch level'
    for old in (old_main,old_consumer,old_mask):
        assert text.count(old)==1,'HID ringer variant: anchor not unique: '+old
    text=text.replace(old_main,new_main).replace(old_consumer,new_consumer).replace(old_mask,new_mask)
    digest=hashlib.sha256(text.encode()).hexdigest()
    assert digest==RINGER,'HID ringer variant result differs from the recorded source'
    print('GuampHid: ring/silent switch (Telephony 0x0B/0x2E) added as report 2 bit 4; collections unchanged')
if digest==RINGER:
    old_touch='        unsigned touch=0;\n        touchEnabled=PE_parse_boot_argn("sheng-touch-hid",&touch,sizeof(touch)) && touch==1;'
    new_touch=('        unsigned touch=1;   // default on (guamp 28/10): sheng-touch-hid=0 turns the touch collection off\n'
               '        PE_parse_boot_argn("sheng-touch-hid",&touch,sizeof(touch));touchEnabled=touch==1;')
    assert text.count(old_touch)==1,'HID touch default: anchor not unique'
    text=text.replace(old_touch,new_touch)
    FINAL=hashlib.sha256(text.encode()).hexdigest()
    assert FINAL==FINAL_RECORDED,'HID touch default result differs from the recorded source: '+FINAL
    digest=FINAL
    print('GuampHid: sheng-touch-hid default 1 (=0 off)')
if digest==FINAL_RECORDED:
    # Step 4 (29/10): motion/proximity sensor devices (analisi/sensori/SENSORI-20261029.md). Each sensor is its own
    # IOHIDDevice with one application collection whose pair matches an IOHIDEventDriver personality of the pinned
    # IOHIDFamily ("HID SensorEvent Driver" 0x20/0x73 and 0x20/0x7A, "HID Built-In Orientation Driver" 0x20/0x8A with
    # Built-In, "HID Pointing/Consumer/System Control Driver" page 0x0C). The descriptors follow IOHIDFamily's own unit
    # tests (IOHIDGyroDescriptor.h, IOHIDDeviceOrientationCoreMotionDescriptor.h); the event driver turns them into
    # accelerometer, gyro, orientation (CM usage) and proximity events. Ambient light has no IOHIDEventDriver path.
    for old,new in SENSOR_EDITS:
        assert text.count(old)==1,'HID sensor variant: anchor not unique: '+old[:60]
        text=text.replace(old,new)
    digest=hashlib.sha256(text.encode()).hexdigest()
    assert digest==SENSORS_RECORDED,'HID sensor variant result differs from the recorded source: '+digest
    path.write_text(text)
    print('GuampHid: sensor devices (accel 4, gyro 5, orientation 6, proximity 7) behind guamp-sensors')
elif digest==SENSORS_RECORDED:
    print('GuampHid: pinned Consumer-only opt-in, ring/silent switch and sensor devices already prepared')
else:
    raise SystemExit('Pinned NativeHidInput source differs; do not guess an ABI or source layout')
