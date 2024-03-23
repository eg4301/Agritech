#include <pgmspace.h>
 
#define SECRET
#define THINGNAME "RAC_Sensor_Hydro"                         //change this
 
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
MIIDWjCCAkKgAwIBAgIVAJcpSIuG76ZVhD3XnwRLyqpsoPP2MA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMzExMDExMTU3
NDBaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDhvGpV238FEQo1iXtv
nVy5H4g7uTTr33N+wTfnpk2WPRCislhmUk6lRobWU053pQML0gOmvSD66hYc2yCv
OIxzetXeV/uJ9y5F7/SxdtP8mnMm9utu+rx3y592MEe665rleJ6beDF2w92/5S1S
JtZUA03ZEoaAFCljW2e/I+UO0PmDUr7GZDF1XuAx4HwhQm8pX82125BMRGiIp32d
tJDVEbgEdgXm/fmdwZdaeBctC+BRxxRL0UhQ8cs9uZWctkgAm4dUaZrzd10dsk+2
/gGVL0FQg5D6y9KsRWSM/mpZdaQQpPuCbfwfsmcs/V4PHirByyW4w7SbsER5vPem
BTQtAgMBAAGjYDBeMB8GA1UdIwQYMBaAFAXDBsd1V8ApkfKj/JGhYyGZ5khPMB0G
A1UdDgQWBBSOdb3NY9/LVJ2JthVpI/WpOAWxFjAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAn2CE8QHbL4de3FYTDL0tX9cc
JjGoIdyxSlT/3P6A3CM5yrhwlTaTwR33dyy3CFh3OH6nw++e49QeQ08Kji6J9FIs
Nsw6hO9BstosVB9EqWIvJnEa7RzVOlQY+J62/9KBfYIJ3FKjFC11Plrn+tXw33dT
GWXi/z8j6JDeaYZ/YYY5BHQoTQ+kX6uqIlIObx5Lf3/z3icTW+8zAa1tnwDjk7HH
CJweZ9qWo6kR6+nnB59qQmVyMv192Q5aPgezUTM8YGRzdVhJs9lthbconEukQLx0
C8nIbwqlzsoPeUkBE5uhNIyO2lvfDcFYqLmugyARADMF2xX5dfycpL+SsQSHJw==
-----END CERTIFICATE-----


 
 
)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpQIBAAKCAQEA4bxqVdt/BREKNYl7b51cuR+IO7k0699zfsE356ZNlj0QorJY
ZlJOpUaG1lNOd6UDC9IDpr0g+uoWHNsgrziMc3rV3lf7ifcuRe/0sXbT/JpzJvbr
bvq8d8ufdjBHuuua5Xiem3gxdsPdv+UtUibWVANN2RKGgBQpY1tnvyPlDtD5g1K+
xmQxdV7gMeB8IUJvKV/NtduQTERoiKd9nbSQ1RG4BHYF5v35ncGXWngXLQvgUccU
S9FIUPHLPbmVnLZIAJuHVGma83ddHbJPtv4BlS9BUIOQ+svSrEVkjP5qWXWkEKT7
gm38H7JnLP1eDx4qwcsluMO0m7BEebz3pgU0LQIDAQABAoIBAQClYhSKCMFb5esT
5EESOjlnzAlUsFFsio6kjE04I6hubRjWClInzR+fiTdaqTgxAi1kKJ7SN3iFPKGs
zIJ6vbRxGsq8FrvMjdNSHAarSkLvq/y3zGobKgqQvhd7UM4MzXbzrlaN0xZyS/gY
EoEM0jVu4ejUI/V0utDD4YD2EgdMvT+L+Xn4dWxJdqfqpgbKCUcUt+AmBWt0eARd
PodcSZVFRFLLlaRwl2Y18pDrn0f2dZubHmw/Vrvj8S55GCnetZMebKvo4hJkpK4b
mtUQk3+wXLCVZ5Sl6bblmq8JBqjRm7CjzPbrQaDKWjWMUhqUHDtHDZAFec4CsAv3
TtLA9i9hAoGBAPz/3aXAayyfRujpKaTKnZNaOiRQYX/S8h9YtnLYQt3vs50SUGve
NP01fsmW76LOKHhVnWLliiSljBrYfnLYMxhpXjLNZ2M1geZSJz04exG1SAhrezOv
fwHu5nSgEZJxShJ1Qy0cCvnRtmLI21eVIFZdePLT3kWS9GFT49F8XFNpAoGBAORp
xk9b05NxaSTWD/aF5e+znrUOqz25cALFQsSaKBj1+7WBjkR8iVUOF834geS8pz5J
mY9fxJ0SxGdl6lcExcz4j0KYEfHXDfr57HptcndQ0hEExBDEHnk6uHfuupk8sL3p
Fn95XrIH+W0upx7ITzRvXSOU1N2a3Sy7zs4ozTYlAoGBAIB2dxgbrQ+6TGolQkTv
sLVKYi1gNzGNbe2mwPVtg/5nIP+Hdhhfeu57REaeAk3bekcnycOWo99Zw9s5xHMR
/lfN4MvQ6kdAuxW070GxtSPXtZy43pTu7oGPyaq8QE/ogGEA87wq+hPBfWeJb354
NTRFQA2ksL8JEFd/HjWXfNXJAoGBAMO+Dc+GbvuFDkkeFVH7j4Bq2tBowMFFwELv
j3dycHhrnQj//M6eNwan7Ch8FbzTGzPa2PkwZgsBwUSm5zspgOVWAVddgMtSs81o
Dosgbh7oKQ2imu+PUlorw1gsQsb9Zq1pKfmWJlvFT7CMcXWEzbOu2A+dBQrgIzip
veGYv7vtAoGAGTC3TpbEXZfevQx/h5B0ZwcOcrSE/A/6wWzu8o0XgRQmwDK7rsKu
hhEi3R7Np+PpVdpztmbdEdCj+k7jEU1oNiDJONjn900Vl/7s402fX3k2nCUtBKQ5
P2GG6+s7dJyHWYsNNIcZMRa2IigUW5roAUWfOlooG6Iak8JLtzwP6Dw=
-----END RSA PRIVATE KEY-----
 
 
)KEY";