#include <pgmspace.h>

#define SECRET
#define DEVICE_ID "testesp32"
#define THINGNAME "testesp32"
//MSB
const char WIFI_SSID[] = "AGHALAYAM LAB";
const char WIFI_PASSWORD[] = "levenspiel";
const char AWS_IOT_ENDPOINT[] = "a3p0dq4aro0hoi-ats.iot.ap-south-1.amazonaws.com";

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

// Device Certificate
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUeyoKyu2pC/JTEF610BuVDo08luYwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIxMTAwODA5NTU0
MVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAL8I4wh/kBdIZ/wgZIiP
Delx2bYyE0qm3Kj+QhQVlE9XHroO3Q9MrfNnyqVZhE/UiKGnDVeyR6YGdoeua7dG
zgGa+Zwy+vr3JGTv/BgregZ9QtngbdJ+LnvutmmAecGoiFVJWkQlUI7c4gbbGi4u
iIrLdW5gVmzPYvLVZ2VD8ez0a5LQeNilkKmbcerTuyCxtZKJxaNHCej/1nbBse2Y
iIuzS1yZ2fXcNH+3xjfy9v4SjikCwGdINw2/PfOhKrPThiOSNnYBnzX5qML80MXf
NKStPxPtfFb2JSqGv9logj+6jrDJDxvTN/ICetvoJG2OEaMk2SpkARoHS/dDP78G
NjkCAwEAAaNgMF4wHwYDVR0jBBgwFoAUkdPVcrDXr8MiB1cEaF/4rJFUwvowHQYD
VR0OBBYEFFSktucTAGLNkrgVajH8bhGb2R4aMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQBOZrlz92XPulF1NHpgCVzM7LCi
8binsG/nfqGF0AgQNSW7wB0i8RmJzAko5E6ke1+JMlHaMg+v7NkGrOnQtYEdBC1p
cIw8xU/u4H0OxyuBLdKNlz/LcabB8sLDNVckREE/YBS9DNmTa7mogWIcict1ARa4
x8MewoLJqrrpU9ljJSnNko3xDML3q29eM/w9O5AJ5+3Dqr8etn8V/GLRhl4epPPs
I/gxMah8zKpZuagKZdXEXF+PaKDd277i9oShCZYNjykXCyKK3xSLuLPU9yHUoKSi
i4d4Q+KFzZ7KrEC3BLGXVYa29xog0GrPevYwze+UsvaGBWuFeF7n5kcNkqsz
-----END CERTIFICATE-----
)KEY";

// Device Private Key
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAvwjjCH+QF0hn/CBkiI8N6XHZtjITSqbcqP5CFBWUT1ceug7d
D0yt82fKpVmET9SIoacNV7JHpgZ2h65rt0bOAZr5nDL6+vckZO/8GCt6Bn1C2eBt
0n4ue+62aYB5waiIVUlaRCVQjtziBtsaLi6Iist1bmBWbM9i8tVnZUPx7PRrktB4
2KWQqZtx6tO7ILG1konFo0cJ6P/WdsGx7ZiIi7NLXJnZ9dw0f7fGN/L2/hKOKQLA
Z0g3Db8986Eqs9OGI5I2dgGfNfmowvzQxd80pK0/E+18VvYlKoa/2WiCP7qOsMkP
G9M38gJ62+gkbY4RoyTZKmQBGgdL90M/vwY2OQIDAQABAoIBAHmUAVGORbjbpvop
73GNgbAZehJfSjHPgk23C3zWwv2/l1mYTg4Hoglv++NB1lgsDgy7UV2Ho9vA7zyo
v/rzf/Y7mDJVq69/DFWE+i6ztMRegeZB1AMLtS/Am60HS/X4+VM2DNGj4eIaJ2tv
k/jiOpoCpKjrfPVhAztCn7wuAtDnH7ykkmon0EazAy3KnCD1hnYaDdSk5Rc3Bkbv
xTFX5CqS7WcZr1zXI8SmNMZXY5FTSHidlL9RNFRb/09fDGmxQhu0LBEaQ7cqofoG
sI0sCdHNACVOpFhmBBzDLvY4L+5SOW4eSCpYYQpVQsQ6a0y7Ly1STgxtankzUO+v
KEmnmmUCgYEA5juWAwbffTIMowZPCwOYoRT1emxCbP8kh632ZJTCbD9MsZPvIjHh
oAiW5HYW7WRpAHSc20kDPNQpKNcbpHcORmAUJj9Yu5qOtvXKJQpIXJuii6r4RiEv
9anjAu7EwoxnB4AHLVmgGik2ln7gStvgUpuIbH+/1hMhEOc/YqPZgp8CgYEA1Go8
MNOVPcikXvJ6FRayyqfQ8pZjZmpmLxVCFFBwfAUowCgwN63w8J54v1udms8o9NNs
khZSQwDKoUCyAx/CYkuA6aRh61NlJLqKiqnRGPMFRn5UsvdLr1dZWyhtdyxt9wfb
aDb5MrKOt2IDbW1HXm40+fDtmEHHuzgU848qsCcCgYBWsNcSerMKbB8P/58hsY/2
D3NDTqJQcl3p7NobE1yF7+vL7b0cbhXb8YAHtGN7rdRPxJkPhNWAQQ7ifuS8r8uj
Oig8Ipwhv9e/Epu0CMQogr3Yw17K4VIyuW5uO0PBUaa1z0JklBvR3fTdBBcmeNZl
pITg/dVZIUjRGZHnbXTIVwKBgQCAN2wDsQOp2p3VLhog9qWPjEKk28A+RsVNcFCF
wJRVWCvj6Z5JTbZdF9sc7ukwVBGNzrz/ZXfcN/MX6NxK5HXKLXcwEOiGSr2HsQ4F
a0HrBG5PwdvQd1N5EInAa6O/xtLNxTFcM6qyPnPFH+WfvNHSn0VdBV/1gudUDCMh
bW7ACwKBgHGuUjiuSJo24TTDVsmZR6reEjgLHq4fWOn3S/TgDiWCNMBckO5BreD1
9hsCWNxGJsL5vpjOjg6YxEsDrby7kRb6dS/PT7VosUqSLOosIK0UyYlTwF/UyZx5
ui6/V8SawIszmJ9PmsCzxCuwHA4RrkxHvkdYP6STlvDEFlZOVBcR
-----END RSA PRIVATE KEY-----
)KEY";