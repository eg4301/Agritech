#include <pgmspace.h>
 
#define SECRET
#define THINGNAME "ESP32_all_in_one_sensor"                         //change this
 
const char WIFI_SSID[] = "***************";               //change this
const char WIFI_PASSWORD[] = "***************";           //change this
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
MIIDWTCCAkGgAwIBAgIUXDmgBbPGteuetZ6C005GazwNIiswDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIzMDkwNjA5NTEx
OFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMvX9ABbMzB6kDArdfbi
YSLg4jaKU/evfVq6krf7d6uQqWJGKpFQeToMgsg0vb5dYV0DGZcIeFqrUCzhhskM
BIpe4mkR6/krcyE/PGi1vM68ibyJmL4QZnkLG2SqRE8axoPkLsf3YBrl017QTJ3+
NDvRoidfZFSrnLfFuDisiEbhk6oIncgxNF4p42NvIRa0ZSCbLm4JKVMRQDjngD22
yGjw0jjtBmXazm5y6QxjahUO8veDuF45dW/zN5C6VtI0MESPMFD3A/ZNWtTGb8T8
V486mJ90xOIOykyj3kRJBbXLjyANaYfLQ5Fe/S5HL2tBL7hfCzn98Em727lFt2pJ
Ke0CAwEAAaNgMF4wHwYDVR0jBBgwFoAUWrTbhjOd03cOm+sprOARHjKHZEEwHQYD
VR0OBBYEFDj/1ih1SMoPfikZGvpZoh+WJg1GMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCIIyZbh0XysYalZqZpti3sXpie
u0chj7FDiSl1mymR0EkWsqLvRhHSuZR8c0Y9G9nyg5txxjDsG1rTtdvANPgEOqW+
DGAv3yQu66u/7mY+RPgteesQJexs3schOXqEOTP/JzNhInQTGs6DLYZ1vnszFmap
eSFG8/3qYtrVfiRFgSYfy+wu9Yzppy64q24OZBovVAK94jhNWCJhK1cJ79DtFTB5
7HLs0j588nzKcIR93z2pZiJIWSYHrsdljnGqavF9iRkOfAm0SaK/PZEjwA5ulFNN
0G/sqZ4a7rOfiUFRkZ1HFUBkrhTOvdUhlwmhgS7b9npwwuXbCJ9dwm/E0nZF
-----END CERTIFICATE-----
 
 
)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAy9f0AFszMHqQMCt19uJhIuDiNopT9699WrqSt/t3q5CpYkYq
kVB5OgyCyDS9vl1hXQMZlwh4WqtQLOGGyQwEil7iaRHr+StzIT88aLW8zryJvImY
vhBmeQsbZKpETxrGg+Qux/dgGuXTXtBMnf40O9GiJ19kVKuct8W4OKyIRuGTqgid
yDE0XinjY28hFrRlIJsubgkpUxFAOOeAPbbIaPDSOO0GZdrObnLpDGNqFQ7y94O4
Xjl1b/M3kLpW0jQwRI8wUPcD9k1a1MZvxPxXjzqYn3TE4g7KTKPeREkFtcuPIA1p
h8tDkV79Lkcva0EvuF8LOf3wSbvbuUW3akkp7QIDAQABAoIBAHnt3bzM0IzB8zds
963KcXHsdENNcLZqGSBJE3PLjiRobnkjIVk0ep7XVu8sZQbWUmPRc+Acp+kMKfP+
nqHpd1nRMbrJho3PiJktexPjCWHb8sN8xomBw9nJZARNuHhcuruvpK/l722HZWyE
SiADKvlRBp5od9oRoLdoJC56PTWvrRtcDC1BTK2j38TpXpJHxnJODQ4PAZZKXcJe
f74k3eR2WMsAVCA0B7y0whwnqIxCOtpOuywtlPv1UQJFG7fn2F2iguRCanfPSsdJ
g33fNeHf4pWvc6qR87wAOWTOUhXw3EARbMjkJsYOV9haJviMAv8JgWuGzwd1Cupg
31zU/h0CgYEA7nliwxLxUlLFb4u+Ye4lURR/6mX+/8sKWR6QoWYnZL5FxV+Ixm5z
E4CfD9DTaMJLvq9crU93EB99N7lrFjSuyVlFC5Exlte5pm4JR+q8YRWyqHmtdJOB
FeigMi07+TPTOawnvVCZsxigIrhH0RzBl6RxHuXrIiXKY1Ftt4TdqgMCgYEA2tMI
bYsInBc8KdxLITwl4ZKtGEN+EswEB24plm9N2Xr0hccXrffjLp/FvMsLYVqiFm17
try0tqJo1zsdFemi5EmXDkJ+4tr5sX9+cE/VHP2hYLCGMJac6yluE6SjsaKEpU0L
CnerS0YRiTYLfknG6oE7kt4czmZe1v1nyDgykU8CgYEAxi+JHXejqHfzb0Vivj5N
MvnnaKTFbE2QVwooxomAGaBrmaFQqSBncIDHMDyrRXebvWg9jCbrWzHgPaxRy82S
qPEQiZFr9eFof2EZQ3RYhBKlPJCKz8Q6VjvSzoB7RBqG43ooOJRAd+/yPYiK3sWY
Oe9Gt5f6dpGThkDI/uhj04MCgYEAhCwNniTpeWarZ1lxY8yx0NNIjCxQHUhvcpF+
uyfVrPQQeDFuQx1Ecqf8xvEZmZ8l4kgU/qZyvRHSI5bIFbE+O4ebnvhZLes4AJdS
psIBpnYoIH/fPb/rXjtzCphaa3QPjMzBS3xG0bur4eXsAUbMyF5wvmA8pAq/ZDpB
knUnTg8CgYB9TF2HH8mUuGPfOTeGVw5dxaw1bpvLaoO7/c8z4nzdPa6UwDi9xS3+
T9TvHksPqjnMOWgbzHnSAxhFt/0cqWaYjxqiD5/kqEFDO6JlxKAJ7Eg68yxp5Y1L
SuZ1wviRxMz9pVhYy5ivVxwhejZFoLq377IcB+NY1jZW1Ynzktdl4A== 
-----END RSA PRIVATE KEY-----
 
 
)KEY";