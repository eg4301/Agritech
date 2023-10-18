#include <pgmspace.h>
 
#define SECRET
#define THINGNAME "ESP32_all_in_one_sensor"                         //change this
 
const char WIFI_SSID[] = "localize_project";               //change this
const char WIFI_PASSWORD[] = "localize_project";           //change this
const char AWS_IOT_ENDPOINT[] = "a29hjx8aa2fj0d-ats.iot.us-east-2.amazonaws.com";       //change this
 
// Amazon Root CA 1
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";
 
// Device Certificate                                               //change this
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUQ2QDdwl1MSy5EW6hbU2NZF6cmNYwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIzMDkyNTA1Mzgz
OFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALyyLPrCUtixLBJZcEa5
63J6RvZX7TyA9aILEUhuXMO5up0xwuaGudRKV3w14dmU7Lmu4YtCv2sS89DMgnhq
IZWbGOLWKISsCniebOXGEW9gy/VUQgRZ8TWmMIvVeyIWShsjp+PLsKWft8YwLovo
Mb3DzG8p0Bszguh3KMMHhvmFabcW52dCB96sdYQJYtwR+dmtNUGATIsRV2P0Bl/x
U/xHDu3Bh3isbUEugi+FQbhgEb6Chm9925U7CbgN94B05MRptXijBAMZym13FgH6
AhlL5byofg0ivEOtFvDD3jldZ8i2uodE00rsHddNi44IgPlRs8m8F5h7S+mKN7X9
938CAwEAAaNgMF4wHwYDVR0jBBgwFoAUJi9x2AUV1hxPF3fb2OR429IMgiYwHQYD
VR0OBBYEFNQX2vZUh4jFAiGTQpQavJApPBVDMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQACq3ErtDt20mS2lWR9W1YFLkxr
NfEwkTVD2WcqcFjVn8crBpjuL5xIV40m26S6psGqnCryL1Pfjf8RtAidqR9fDFWr
tf/uCNHXnagW4nVmjY3ZVIE64KGhAS+fg9ykPxJ+lgdIGbLQhS9vx0nabbhyHKbF
fycFMxgth0MZG0ecwoyR2t1PTik9L+SNf8X5U/DyOqm/pU9W1N0XRt9qjA7SRJFn
TL+xDmkmINmk09GjH+xWaUUHcWobDFeo0/EnckoBxrz8u0AbfwhbLW/+PEKSFGUw
f8RjX6MyHGERHZk7wuQHQnH+EGrAFnP8Kw1Dwa9pJLupQkFo1ypVgtmgG3PF
-----END CERTIFICATE-----


 
 
)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEogIBAAKCAQEAvLIs+sJS2LEsEllwRrnrcnpG9lftPID1ogsRSG5cw7m6nTHC
5oa51EpXfDXh2ZTsua7hi0K/axLz0MyCeGohlZsY4tYohKwKeJ5s5cYRb2DL9VRC
BFnxNaYwi9V7IhZKGyOn48uwpZ+3xjAui+gxvcPMbynQGzOC6HcowweG+YVptxbn
Z0IH3qx1hAli3BH52a01QYBMixFXY/QGX/FT/EcO7cGHeKxtQS6CL4VBuGARvoKG
b33blTsJuA33gHTkxGm1eKMEAxnKbXcWAfoCGUvlvKh+DSK8Q60W8MPeOV1nyLa6
h0TTSuwd102LjgiA+VGzybwXmHtL6Yo3tf33fwIDAQABAoIBAEcH00K5r7H+T5bA
sDEPf3/iY6ALzTP/X3eWLvVTBfxvh73wo2tfv8gYDAKnzdK36ryjBigEAkZuJfWt
Zlepq+bDMYUCFNKMaIyrJlcATkPc7in1g7xblx8Y5iFdRTK5rT+2Flb7sA1IqEck
kAD+cOr8L5KQ7NOwelDDukZgKQD3/7ZkF0qOAaGoH+pgFu8oKb6BCtGxqzbztlbS
Bp/JCKmL4rUtMKbphD61/3krUotCIJ/aGjgthTZb8ZlxC0/inEdutN5Rtj/ofioQ
Xhz1MQFdb0evxSifRYZcxTvoDIrFLyRY+Dmmojky7ZEizzTjUPkYFy8mV8Ujm9Nj
Uyi+xdECgYEA7Sje3iiLC6OUQsTsOp/VJUMfaSUgWACuGsV8+vvnWNxYFk+HSQgf
zJnU8vuAVQb9sFZEt2asHAUxOXzQThd4Nd4RdMECBL2HESo+c//juwqhGjiDTz2N
eJs05wVr7X9kvCjb7isOVmIF7QcnffFUvqLnYfqWavaKQv2mfSZ/Y4sCgYEAy6+y
cjuGpbtbQjlK2AIiy61/5jDf6ig7irXpxzH757a52/IjBaKQib0uM9UKeAdnB9VW
YXNfcR0siQKeDlkB3rdo6kQ1zA4RwgPVSCaJxhDSLeYnv2D7FOc580TszcSu6Wnv
TBgm1zuSK7T6Ui+ybqyTHnN2C3vIDnkhB75mKl0CgYBkt06umhkywC1ejbKk0uv5
/x9/z7yEowVNaUFCdWgyXKEsGGojocefZFUDO7usYGOsV3anTHsbCzl0aIUuJNlw
Rvay1qS8zqBlscMEEE/uRv1T3VwmcsV5yrAUzEChg9CL9+4unypflRFfMAKdgbez
ngsVmQ34LutYt8/UplzDMQKBgCOFV2ROdzPN0aNh5Gh5LuJANo7JNTNbos6U3NkN
IXYOSqlNt8fSyswKftUxMOsvt0sJbFAAU8OhJ9QY7cFoykexyo0YKh2U0on4Kyiy
99HfuxegDIx2eDL0BAolmfpkc3BnNgEaMSPV0q7ml8TfZomgdOOZlRt5kbDcdOMp
1B1NAoGAY0Nih3ivQWvbB4gfxQO0IiqLNw/DB9YxuYPGt2dImo3HFDOTF4pIcEix
cPl1LTShZ+0clAbHGUkZQLaoKD2cT1AWhj1UNSCsxHWLtTu5AxJifU83hPgD4B1E
09luudY79f62UUJ1xeihUoS8PF7jJthTvarqrb3V5cDlyHc1vG4=
-----END RSA PRIVATE KEY-----
 
 
)KEY";