#include <pgmspace.h>
 
#define SECRET
#define THINGNAME "ESP32_Arduino"  //change this
 
const char WIFI_SSID[] = "SHUJA-BB";  //change this
const char WIFI_PASSWORD[] = "click123";           //change this
const char AWS_IOT_ENDPOINT[] = "a2w1yq8cwf2ptk-ats.iot.us-east-2.amazonaws.com";       //change this
 
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
MIIDWTCCAkGgAwIBAgIUZ+4PPdQ1SQdkqQA//HD5lmVPqswwDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIzMTAyNDE5MzMx
NloXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMJvNskQIskMA35vn9+k
9o7JEHGnGKb6CzrpHxvC6Z/FeEc49RHFHfBjaT58nSh/NLqFHHFavH7YPE15VQ+1
ObUC6JHN3amhQdSDQ7HUZma4fQqtH2Xb7ibDd6dFJjTdmLk0lMelSDoROZ8WPoCm
2ss0qXA2MgHvi7vijQYC3Oh4SGtAOypOF7kK2PmKrpW/w3CHExm+nwBPkEqOezXj
Ibf1ZXBhTYxroEwztJlZ7/HdCIkO44NfewzX/+Ld+T4nXoYN8dIPo9ACcykH9Ixz
c7uVT9jSJAYDH9ZxCOwOOqrpeUaIri6P3AiPohJpr+vyk+XG8g5182DoAHkXEx9B
iPkCAwEAAaNgMF4wHwYDVR0jBBgwFoAU7MxPMxq9eE/xn3J/xXirlVZ5zRswHQYD
VR0OBBYEFFtbj6qcN9OHW62YtSoH5XUGsLl1MAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAwqfEgjy/TSVU3nwwYYr7BMoHL
Fue4EVNZbq4UBxSBrsjxIcwHdpUmtUz3V++lcKYL9l5EI8ze1Xo1PvP9f8hJb6iq
bkfmCFNGMV4gvSflOSBQhXvBD5u5kBAGpA0k+NwifWG5XU4qKrQiUstMGLve5c7Q
2/XC2Qi8sE5hvtI98fEX9hBmyULspp1zsfSl5K4iDIMuPcuYG51iJzgDCCiumcd7
FWuLnPMxTCmaOcZe844odiHFhVZXPUspY9ZyrsnoGrfW21asR+w+LQZ7vjd0HG95
HoTP8KDPbSJ6rHBQnDA4QkuvPyaqdSu3B1/ubboljGFMvjgFj4ZHuIH/du71
-----END CERTIFICATE-----

 
 
)KEY";
 
// Device Private Key                                               //change this
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEpAIBAAKCAQEAwm82yRAiyQwDfm+f36T2jskQcacYpvoLOukfG8Lpn8V4Rzj1
EcUd8GNpPnydKH80uoUccVq8ftg8TXlVD7U5tQLokc3dqaFB1INDsdRmZrh9Cq0f
ZdvuJsN3p0UmNN2YuTSUx6VIOhE5nxY+gKbayzSpcDYyAe+Lu+KNBgLc6HhIa0A7
Kk4XuQrY+Yqulb/DcIcTGb6fAE+QSo57NeMht/VlcGFNjGugTDO0mVnv8d0IiQ7j
g197DNf/4t35Pidehg3x0g+j0AJzKQf0jHNzu5VP2NIkBgMf1nEI7A46qul5Roiu
Lo/cCI+iEmmv6/KT5cbyDnXzYOgAeRcTH0GI+QIDAQABAoIBAC5r3RmuF5AGpULn
pbUArrkyehGYgoNDSbEb/YhkReEWhXwDtkcy184P/g2kNOV5UX6Cz2Kk9kIWoHci
09M3QnrcIejFzPl1/cCeQmhPvM8jlv8YjFmGY9qvan7uPIv8qDz3t9netVrfx/Ea
n3uZ2r0ParDWz6O0rJkEIwVfY5zAyHODQNRml1zPVviO+z882y5eofGT1lMoEJTt
jXovK3DrjOvPumD+o65q/07SfrD4PPMh9VOYyb1uKwiinZxxA09KZ0TZBx8aMvFN
t7dLt/a1mHiT5xK+mqfqO6EnpN0UbvFOgbGa5nIXctpi2/57HlC3L0pQWLI3alXW
l0CSjHECgYEA6BwP9AI0ytCPlmwt0d2bCtCvV97Zzx2b3/Ta6VtmvbYcifzW8KCM
xxWgG0QPavGH/T8ILJMHTY2Yfv+doGP5dkxT51DkBfCGF+qrBWiH7smTxGsqc7U0
LnbuXDGHvU0LzuclWuWkNZHkG6zAgWDeuXRajae90QW4nuWt4+faRUUCgYEA1nJv
WAfLn1guwnrw6m0cQXa7C6QIbnTEJvfefokUW39/2y8CmLmojT+QYV2B7PvmSBz4
OkaMaatLFFX/HKSacyhnuElf6CZOoHVoKE4oKkhAe3k7hAhE5hu2YgBymC/SPsVT
Z/aZSc/B152ajIPcS3eh5Hd6lYc16qChufiYziUCgYBs/MhM5kBNv3K8yh0Vvz3O
7UB4PS7+/sMzfPDRxFZQ3FM4Q93VbT3NOY1NYD7xE2qsck1vSLaUJtmlQHeOWYNI
lfyL0ELRuyfr0Qfu6S9y43ocL8nmPP68lqHD7onzTFswEv9KWCwr704x5IMb/RV3
9XeTrmEF5e7TxbtKGZM3FQKBgQCv1sGp4yp6DCq7WWe2MaWboCmbT/OB7sjwUwrj
oq9In3FNXmy4onOs7lQLIkto8NxQgUChODvmYYh8sk8vQ2FJz/XMTI4Ro2eRdvPj
KMMSeJA3Lp84kjc8n7ia/1b3py60PcU4WCI1Pd/qVtblJTI7x2q+FaI0fdwd+Xzj
Kw7Q6QKBgQCuTHC7y8CXyHabCIxLu8ODKuTvvsgo8eh169QuBfVJJiobNTBmoXlM
cPrz2LHvgYLzWYeD0SiIu9wF9Rjrqs25IZ6QwcYlHRiTYWBjmYxCbIErwJklwVX6
+xq8kbhmGTr5PNfV15tA3H0XZDsYAcxM0MrxZ6/IWNoAjoNYJsChQw==
-----END RSA PRIVATE KEY-----


)KEY";
